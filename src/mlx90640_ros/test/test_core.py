#!/usr/bin/env python3
"""Run with unittest; no ROS, serial device or native library required.

Set MLX90640_LIBRARY to a freshly built shared library to include capture
regression tests. The native wrapper supports one Decoder per process.
"""

import ctypes
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mlx90640_ros import (Decoder, PacketParser, Sample, capture_payload, crc16,
                          encode_packet, iter_capture_payloads, load_capture,
                          render_image, TYPE_EEPROM, TYPE_FRAME, TYPE_STATUS)
from mlx90640_ros.protocol import MAX_PACKET_SIZE

CAPTURE = Path(__file__).parent / "data" / "capture.bin"
MOCK_EEPROM = b"\x01\x00" * 832


def frame_payload(subpage, ms, mode=1):
    words = [0] * 834
    words[832] = mode << 12
    words[833] = subpage
    return struct.pack("<IBB834H", ms, subpage, 0, *words)


class FakeFunction:
    def __init__(self, implementation):
        self.implementation = implementation

    def __call__(self, *args):
        return self.implementation(*args)


class FakeLibrary:
    """Only replace vendor math: exercise actual ctypes buffers/state logic."""
    def __init__(self):
        self.init_result = 0
        self.frame_result = 0
        self.calls = []
        self.corrections = []
        self.nan_pixels = False
        self.nan_ambient = False
        self.mlx_init = FakeFunction(lambda words: self.init_result)
        self.mlx_frame_to_temp = FakeFunction(self.convert)
        self.mlx_bad_pixels = FakeFunction(self.correct)

    def convert(self, words, emissivity, shift, output, ambient):
        self.calls.append(ctypes.addressof(output.contents))
        mode, subpage = (words[832] >> 12) & 1, words[833]
        for row in range(24):
            for col in range(32):
                selected = ((row ^ col) & 1) if mode else row & 1
                if selected == subpage:
                    output[row * 32 + col] = float("nan") if self.nan_pixels else 20 + subpage
        ctypes.cast(ambient, ctypes.POINTER(ctypes.c_float))[0] = (
            float("nan") if self.nan_ambient else 29.56)
        return self.frame_result

    def correct(self, output, control):
        self.corrections.append(control)


class ProtocolTests(unittest.TestCase):
    def test_crc_standard_check_and_wire_coverage(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)
        self.assertEqual(crc16(b""), 0xFFFF)
        packet = encode_packet(TYPE_STATUS, 255, bytes(range(11)))
        self.assertEqual(packet[:6], b"\xaa\x55\x03\xff\x0b\x00")
        self.assertEqual(struct.unpack("<H", packet[-2:])[0], crc16(packet[2:-2]))

    def test_every_split_and_concatenation(self):
        expected = [(TYPE_STATUS, 7, b"x" * 11)]
        packet = encode_packet(*expected[0])
        for split in range(len(packet) + 1):
            parser = PacketParser()
            self.assertEqual(parser.feed(packet[:split]) + parser.feed(packet[split:]), expected)
        frames = [(typ, seq, bytes(length)) for seq, (typ, length) in enumerate(
            [(TYPE_EEPROM, 1664), (TYPE_FRAME, 1674), (TYPE_STATUS, 11)])]
        wire = b"".join(encode_packet(*frame) for frame in frames)
        parser = PacketParser()
        got = []
        for byte in wire:
            got.extend(parser.feed(bytes([byte])))
        self.assertEqual(got, frames)

    def test_noise_bad_crc_unknown_type_length_and_recovery(self):
        valid = encode_packet(TYPE_STATUS, 4, b"q" * 11)
        bad_crc = bytearray(valid)
        bad_crc[-1] ^= 1
        for length in (0, 10, 12, 65535):
            damaged = b"\xaa\x55\x03\x01" + struct.pack("<H", length)
            parser = PacketParser()
            stream = b"garbage\xaa" + damaged + bytes(bad_crc) + b"\xaa\x55\xff\x00\x0b\x00" + valid
            self.assertEqual(parser.feed(stream), [(TYPE_STATUS, 4, b"q" * 11)])
            self.assertEqual(parser.crc_errors, 1)
            self.assertEqual(parser.invalid_headers, 2)
        # A frame with a corrupted length must not swallow the next packet.
        broken = bytearray(encode_packet(TYPE_FRAME, 1, bytes(1674)))
        broken[4:6] = b"\xff\xff"
        self.assertEqual(PacketParser().feed(bytes(broken) + valid),
                         [(TYPE_STATUS, 4, b"q" * 11)])

    def test_sync_split_and_bounded_noise(self):
        packet = encode_packet(TYPE_STATUS, 0, bytes(11))
        parser = PacketParser()
        self.assertEqual(parser.feed(b"noise" * 100000 + b"\xaa"), [])
        self.assertLess(len(parser._buffer), MAX_PACKET_SIZE)
        self.assertEqual(parser.feed(packet[1:]), [(TYPE_STATUS, 0, bytes(11))])
        for _ in range(5):
            parser.feed(b"\xaa\x55\x02\x00\x8a\x06" + b"n" * 1000)
            self.assertLess(len(parser._buffer), MAX_PACKET_SIZE)

    def test_encoder_rejects_invalid_input(self):
        for args in [(9, 0, bytes(11)), (TYPE_STATUS, -1, bytes(11)),
                     (TYPE_STATUS, 256, bytes(11)), (TYPE_EEPROM, 0, bytes(11))]:
            with self.assertRaises(ValueError):
                encode_packet(*args)


class DecoderTests(unittest.TestCase):
    def setUp(self):
        self.library = FakeLibrary()
        self.patcher = patch("mlx90640_ros.core.ctypes.CDLL", return_value=self.library)
        self.cdll = self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.decoder = Decoder()
        self.decoder.load_eeprom(MOCK_EEPROM)

    def feed(self, subpage, ms, mode=1):
        return self.decoder.process_frame(frame_payload(subpage, ms, mode))

    def test_ctypes_default_and_explicit_library(self):
        self.cdll.assert_called_once_with("libmlx90640.so")
        Decoder("/custom/library.so")
        self.cdll.assert_called_with("/custom/library.so")

    def test_pair_no_fixed_warmup_repeat_and_independent_output(self):
        self.assertIsNone(self.feed(1, 0))
        self.assertIsNone(self.feed(1, 100))
        sample = self.feed(0, 200)
        self.assertIsInstance(sample, Sample)
        self.assertEqual((sample.temps.shape, sample.temps.dtype), ((24, 32), np.dtype("float32")))
        self.assertAlmostEqual(sample.ambient, 29.56, places=4)
        self.assertEqual((sample.device_ms, sample.subpage), (200, 0))
        before = sample.temps.copy()
        self.assertIsNone(self.feed(0, 300))
        self.assertIsNone(self.feed(0, 400))
        self.assertIsNotNone(self.feed(1, 500))
        np.testing.assert_array_equal(sample.temps, before)
        self.assertEqual(len(set(self.library.calls)), 1, "native calls must share a buffer")

    def test_no_eeprom_and_explicit_reset(self):
        self.decoder.reset()
        self.assertFalse(self.decoder.ready)
        self.assertIsNone(self.feed(0, 100))
        self.assertIsNone(self.feed(1, 200))
        self.assertEqual(self.library.calls, [])
        self.decoder.load_eeprom(MOCK_EEPROM)
        self.assertIsNone(self.feed(1, 300))
        self.assertIsNotNone(self.feed(0, 400))

    def test_rollover_and_real_time_reversal(self):
        self.assertIsNone(self.feed(0, 0xFFFFFFF0))
        self.assertIsNotNone(self.feed(1, 16))
        self.assertIsNone(self.feed(0, 30))
        self.assertIsNone(self.feed(1, 10))
        self.assertFalse(self.decoder.ready)
        count = len(self.library.calls)
        self.assertIsNone(self.feed(0, 40))
        self.assertEqual(len(self.library.calls), count)
        self.decoder.load_eeprom(MOCK_EEPROM)
        self.assertIsNone(self.feed(0, 50))
        self.assertIsNotNone(self.feed(1, 60))

    def test_gap_duplicate_timestamp_and_mode_change(self):
        self.assertIsNone(self.feed(0, 0))
        self.assertIsNone(self.feed(1, 3001))
        self.assertIsNotNone(self.feed(0, 3100))
        self.assertIsNone(self.feed(0, 3200))
        self.assertIsNone(self.feed(1, 3200))
        self.assertIsNotNone(self.feed(1, 3300))
        self.assertIsNone(self.feed(0, 3400))
        self.assertIsNone(self.feed(1, 3500, mode=0))
        self.assertIsNotNone(self.feed(0, 3600, mode=0))

    def test_nan_results_clear_pending_pair(self):
        self.assertIsNone(self.feed(0, 0))
        self.library.nan_pixels = True
        with self.assertRaisesRegex(ValueError, "non-finite pixel"):
            self.feed(1, 100)
        self.library.nan_pixels = False
        self.assertIsNone(self.feed(0, 200))
        self.assertIsNotNone(self.feed(1, 300))
        self.library.nan_ambient = True
        with self.assertRaisesRegex(ValueError, "non-finite ambient"):
            self.feed(0, 400)
        self.library.nan_ambient = False
        self.assertIsNone(self.feed(1, 500))
        self.assertIsNotNone(self.feed(0, 600))
        self.library.nan_pixels = True
        with self.assertRaisesRegex(ValueError, "non-finite pixel"):
            self.feed(0, 700)
        self.library.nan_pixels = False
        self.assertIsNone(self.feed(1, 800))
        self.assertIsNotNone(self.feed(0, 900))

    def test_invalid_subpage_size_and_vendor_errors(self):
        cases = [b"", bytes(1673), bytes(1675), frame_payload(2, 1)]
        inconsistent = bytearray(frame_payload(0, 2))
        inconsistent[4] = 1
        cases.append(bytes(inconsistent))
        for payload in cases:
            with self.assertRaises(ValueError):
                self.decoder.process_frame(payload)
        self.library.frame_result = -1
        with self.assertRaisesRegex(ValueError, "mlx_frame_to_temp=-1"):
            self.feed(0, 100)
        self.library.init_result = -2
        with self.assertRaisesRegex(ValueError, "mlx_init=-2"):
            self.decoder.load_eeprom(MOCK_EEPROM)
        self.assertFalse(self.decoder.ready)
        with self.assertRaises(ValueError):
            self.decoder.load_eeprom(bytes(1663))
        for blank in (bytes(1664), b"\xff" * 1664):
            with self.assertRaisesRegex(ValueError, "all zero or all 0xff"):
                self.decoder.load_eeprom(blank)
            self.assertFalse(self.decoder.ready)

    def test_bad_pixel_correction_precedes_final_finite_check(self):
        self.decoder = Decoder(bad_pixels=True)
        self.decoder.load_eeprom(MOCK_EEPROM)
        self.library.nan_pixels = True

        def repair(output, control):
            # Stand-in for vendor replacement of a known defective pixel.
            values = np.ctypeslib.as_array(output, shape=(768,))
            values[~np.isfinite(values)] = 25.0

        self.library.mlx_bad_pixels.implementation = repair
        self.assertIsNone(self.feed(0, 0))
        sample = self.feed(1, 100)
        self.assertTrue(np.isfinite(sample.temps).all())
        self.library.mlx_bad_pixels.implementation = lambda output, control: None
        self.assertIsNone(self.feed(0, 200))
        with self.assertRaisesRegex(ValueError, "non-finite pixel"):
            self.feed(1, 300)

    def test_bad_pixel_control_register_and_parameters(self):
        self.decoder = Decoder(bad_pixels=True)
        self.decoder.load_eeprom(MOCK_EEPROM)
        self.assertIsNone(self.feed(0, 0, mode=0))
        self.assertIsNotNone(self.feed(1, 100, mode=0))
        self.assertEqual(self.library.corrections, [0])
        self.assertIsNone(self.feed(1, 200))
        self.assertIsNotNone(self.feed(0, 300))
        self.assertEqual(self.library.corrections, [0, 4096])
        for kwargs in ({"emissivity": 0}, {"emissivity": 1.1},
                       {"emissivity": float("nan")}, {"tr_shift": float("inf")}):
            with self.assertRaises(ValueError):
                Decoder(**kwargs)


class CaptureTests(unittest.TestCase):
    def test_real_capture_layout_and_replay(self):
        eeprom, frames = load_capture(CAPTURE)
        self.assertEqual(len(eeprom), 1664)
        self.assertEqual(len(frames), 4)
        payloads = list(iter_capture_payloads(frames, 500))
        self.assertEqual([struct.unpack_from("<IB", p) for p in payloads],
                         [(0, 1), (500, 0), (1000, 1), (1500, 0)])
        for index, payload in enumerate(payloads):
            self.assertEqual(len(payload), 1674)
            self.assertEqual(payload[6:], frames[index])
        self.assertEqual(struct.unpack_from("<I", capture_payload(frames[0], 2**32, 1))[0], 0)

    def test_strict_capture_rejects_damage_and_uart(self):
        data = CAPTURE.read_bytes()
        variants = [b"", data[:11], data[:-1], data + b"x",
                    b"bad!" + data[4:], data[:4] + struct.pack("<I", 831) + data[8:],
                    data[:8] + struct.pack("<I", 0) + data[12:],
                    data[:8] + struct.pack("<I", 0xFFFFFFFF) + data[12:],
                    encode_packet(TYPE_EEPROM, 0, bytes(1664))]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.bin"
            for variant in variants:
                path.write_bytes(variant)
                with self.assertRaises(ValueError):
                    load_capture(path)

    def test_invalid_capture_payload(self):
        _, frames = load_capture(CAPTURE)
        for args in [(b"", 0, 500), (frames[0], -1, 500),
                     (frames[0], 0, 0), (frames[0], 0, 1.5)]:
            with self.assertRaises(ValueError):
                capture_payload(*args)
        with self.assertRaises(ValueError):
            capture_payload(frames[0][:-2] + b"\x02\x00", 0, 500)


@unittest.skipUnless(os.environ.get("MLX90640_LIBRARY"),
                     "set MLX90640_LIBRARY to enable real vendor capture decoding")
class NativeCaptureTests(unittest.TestCase):
    def test_real_capture_temperatures(self):
        decoder = Decoder(os.environ["MLX90640_LIBRARY"])
        eeprom, frames = load_capture(CAPTURE)
        decoder.load_eeprom(eeprom)
        samples = [sample for payload in iter_capture_payloads(frames)
                   if (sample := decoder.process_frame(payload)) is not None]
        self.assertEqual(len(samples), 2)
        # Fresh non-overlapping pairs, independently checked against the SO
        # supplied in the delivery archive (same values as the rebuilt library).
        # The delivery README's rounded values describe its sliding-buffer UI,
        # not both of these two newly assembled complete images.
        for sample, (maximum, minimum) in zip(samples, [(33.04648, 21.29190),
                                                       (32.84659, 21.37926)]):
            self.assertAlmostEqual(float(sample.temps.max()), maximum, delta=.05)
            self.assertAlmostEqual(float(sample.temps.min()), minimum, delta=.05)
            self.assertAlmostEqual(sample.ambient, 29.56266, delta=.05)
            self.assertTrue(np.isfinite(sample.temps).all())


class RenderTests(unittest.TestCase):
    def test_render_preserves_raw_temperature(self):
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("OpenCV is optional")
        temps = np.linspace(20, 35, 768, dtype=np.float32).reshape(24, 32)
        original = temps.copy()
        sample = Sample(temps, 29.56, 100, 1)
        image = render_image(sample, scale=12, tmin=22, tmax=30)
        self.assertEqual(image.shape, (24 * 12 + 42, 32 * 12, 3))
        self.assertEqual(image.dtype, np.dtype("uint8"))
        np.testing.assert_array_equal(temps, original)
        self.assertIsNotNone(render_image(Sample(np.full((24, 32), 25, np.float32), 25, 0, 0)))
        with self.assertRaises(ValueError):
            render_image(sample, tmin=30, tmax=20)


if __name__ == "__main__":
    unittest.main()
