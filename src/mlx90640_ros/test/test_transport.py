#!/usr/bin/env python3
"""Offline adapter regression tests: never import pyserial or open hardware."""
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mlx90640_ros import core
from mlx90640_ros.node import run as run_node, sample_to_messages
from mlx90640_ros.protocol import encode_packet
from mlx90640_ros.transport import ReplayTransport, SerialTransport
from mlx90640_ros.viewer import build_parser, save_image


EEPROM = b"\x01\x00" * 832


def frame(ms, subpage=0):
    words = [0] * 834
    words[-1] = subpage
    return struct.pack("<IBB834H", ms, subpage, 0, *words)


def packet(typ, payload):
    return encode_packet(typ, 0, payload)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Decoder:
    def __init__(self):
        self.ready = False
        self.resets = 0
        self.loads = 0
        self.last_ms = None
        self.reject_next = False

    def reset(self):
        self.resets += 1
        self.ready = False
        self.last_ms = None

    def load_eeprom(self, payload):
        if payload != EEPROM:
            raise ValueError("bad EEPROM")
        self.loads += 1
        self.ready = True

    def process_frame(self, payload):
        if self.reject_next:
            self.reject_next = False
            raise ValueError("bad frame")
        ms = struct.unpack_from("<I", payload)[0]
        if self.last_ms is not None and ms < self.last_ms:
            self.reset()
            return None
        self.last_ms = ms
        return ms


class Serial:
    def __init__(self, clock, events=()):
        self.clock = clock
        self.events = list(events)
        self.writes = []
        self.closed = False
        self.buffer_resets = 0

    def reset_input_buffer(self):
        self.buffer_resets += 1

    def write(self, data):
        self.writes.append((self.clock(), data))

    def read(self, size):
        self.clock.sleep(0.2)
        event = self.events.pop(0) if self.events else b""
        if isinstance(event, Exception):
            raise event
        return event

    def close(self):
        self.closed = True


class TransportTests(unittest.TestCase):
    def make_transport(self, clock, decoder, serials, deadline):
        calls = []
        pending = list(serials)

        def factory(**kwargs):
            calls.append((clock(), kwargs))
            result = pending.pop(0)
            if isinstance(result, Exception):
                raise result
            return result

        transport = SerialTransport("/FAKE/ESP32", decoder, serial_factory=factory,
                                    clock=clock, sleep=clock.sleep,
                                    stop=lambda: clock() >= deadline)
        return transport, calls

    def test_explicit_port_and_no_eager_hardware_access(self):
        decoder = Decoder()
        for port in (None, "", "   "):
            with self.assertRaises(ValueError):
                SerialTransport(port, decoder)
        with patch("builtins.__import__", side_effect=AssertionError("no imports expected")):
            transport = SerialTransport("/FAKE", decoder)
        self.assertIsNone(transport.serial)

    def test_eeprom_retries_once_per_second_and_serial_timeout(self):
        clock, decoder = Clock(), Decoder()
        serial = Serial(clock)
        transport, calls = self.make_transport(clock, decoder, [serial], 2.6)
        self.assertEqual(list(transport.samples()), [])
        self.assertGreaterEqual(len(serial.writes), 3)
        for (before, command), (after, _) in zip(serial.writes, serial.writes[1:]):
            self.assertEqual(command, b"\x01")
            self.assertGreaterEqual(after - before, 1.0 - 1e-9)
        self.assertEqual(calls[0][1]["timeout"], 0.2)
        self.assertEqual(calls[0][1]["baudrate"], 460800)
        self.assertTrue(serial.closed)

    def test_invalid_eeprom_replies_do_not_bypass_request_rate_limit(self):
        clock, decoder = Clock(), Decoder()
        serial = Serial(clock, [packet(1, b"\xff" * 1664)] * 10)
        transport, _ = self.make_transport(clock, decoder, [serial], 2.6)
        self.assertEqual(list(transport.samples()), [])
        self.assertGreaterEqual(len(serial.writes), 3)
        self.assertFalse(decoder.ready)
        for (before, _), (after, _) in zip(serial.writes, serial.writes[1:]):
            self.assertGreaterEqual(after - before, 1.0 - 1e-9)

    def test_calibrated_device_does_not_repeat_eeprom_requests(self):
        clock, decoder = Clock(), Decoder()
        serial = Serial(clock, [packet(1, EEPROM), packet(2, frame(100))])
        transport, _ = self.make_transport(clock, decoder, [serial], 2.5)
        self.assertEqual(list(transport.samples()), [100])
        self.assertEqual(len(serial.writes), 1)
        self.assertEqual(decoder.loads, 1)

    def test_three_seconds_without_valid_frame_resets_and_requests(self):
        clock, decoder = Clock(), Decoder()
        serial = Serial(clock, [packet(1, EEPROM)])
        transport, _ = self.make_transport(clock, decoder, [serial], 3.7)
        self.assertEqual(list(transport.samples()), [])
        self.assertEqual(len(serial.writes), 2)
        self.assertGreaterEqual(serial.writes[1][0], 3.0)
        self.assertFalse(decoder.ready)

    def test_reconnect_clears_parser_decoder_and_input_buffer(self):
        clock, decoder = Clock(), Decoder()
        wire = packet(3, bytes(11))
        first = Serial(clock, [wire[:8], OSError("disconnected")])
        second = Serial(clock, [wire[8:] + packet(1, EEPROM) + packet(2, frame(50))])
        transport, calls = self.make_transport(
            clock, decoder, [first, OSError("not present"), second], 5.2)
        self.assertEqual(list(transport.samples()), [50])
        self.assertEqual(len(calls), 3)
        self.assertGreaterEqual(calls[1][0] - 0.4, 2.0 - 1e-9)
        self.assertGreaterEqual(calls[2][0] - calls[1][0], 2.0 - 1e-9)
        self.assertTrue(first.closed)
        self.assertTrue(second.closed)
        self.assertEqual(second.buffer_resets, 1)
        self.assertGreaterEqual(decoder.resets, 4)

    def test_device_ms_rollback_ready_gate_re_requests_eeprom(self):
        clock, decoder = Clock(), Decoder()
        serial = Serial(clock, [packet(1, EEPROM), packet(2, frame(1000)),
                                packet(2, frame(0)), packet(1, EEPROM),
                                packet(2, frame(100))])
        transport, _ = self.make_transport(clock, decoder, [serial], 1.4)
        self.assertEqual(list(transport.samples()), [1000, 100])
        self.assertEqual(decoder.loads, 2)
        # Firmware's unsolicited EEPROM arrived before the 1s retry was due.
        self.assertEqual(len(serial.writes), 1)

    def test_frame_value_error_re_requests_without_publishing_old_value(self):
        clock, decoder = Clock(), Decoder()
        decoder.reject_next = True
        serial = Serial(clock, [packet(1, EEPROM), packet(2, frame(100)),
                                packet(2, frame(200)), packet(1, EEPROM),
                                packet(2, frame(300))])
        transport, _ = self.make_transport(clock, decoder, [serial], 1.4)
        self.assertEqual(list(transport.samples()), [300])
        # The next EEPROM was already in the input, so no early retry is needed.
        self.assertEqual(len(serial.writes), 1)

    def test_crc_and_invalid_header_warnings_are_aggregated_and_throttled(self):
        clock, decoder = Clock(), Decoder()
        corrupt = bytearray(packet(3, bytes(11)))
        corrupt[-1] ^= 1
        invalid = b"\xaa\x55\xff\x00\x00\x00"
        serial = Serial(clock, [bytes(corrupt) + invalid] * 7)
        transport, _ = self.make_transport(clock, decoder, [serial], 2.6)
        warnings = []
        transport.log = lambda level, text: warnings.append((clock(), text))
        self.assertEqual(list(transport.samples()), [])
        warnings = [(at, text) for at, text in warnings if "UART rejected" in text]
        self.assertGreaterEqual(len(warnings), 2)
        self.assertIn("CRC errors=1 invalid headers=1", warnings[0][1])
        for (before, _), (after, _) in zip(warnings, warnings[1:]):
            self.assertGreaterEqual(after - before, 1.0 - 1e-9)

    def test_slow_replay_retains_nominal_device_timestamps(self):
        clock, decoder = Clock(), Decoder()
        frames = [frame(0, 0)[6:], frame(0, 1)[6:]]
        with patch.object(core, "load_capture", return_value=(EEPROM, frames)):
            transport = ReplayTransport("offline", decoder, replay_rate=0.1,
                                        clock=clock, sleep=clock.sleep)
            self.assertEqual(list(transport.samples()), [0, 500])
        self.assertAlmostEqual(clock(), 20.0)

    def test_replay_rate_loop_stop_and_reset(self):
        clock, decoder = Clock(), Decoder()
        frames = [frame(0, 0)[6:], frame(0, 1)[6:]]
        with patch.object(core, "load_capture", return_value=(EEPROM, frames)):
            transport = ReplayTransport("offline", decoder, loop=True, replay_rate=2,
                                        clock=clock, sleep=clock.sleep,
                                        stop=lambda: clock() >= 1.2)
            self.assertEqual(list(transport.samples()), [0, 500, 0])
        self.assertEqual(decoder.loads, 2)
        self.assertFalse(decoder.ready)
        for rate in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                ReplayTransport("offline", decoder, replay_rate=rate)


class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id=None)


class MessageTests(unittest.TestCase):
    def setUp(self):
        self.temps = np.arange(768, dtype=np.float32).reshape(24, 32)
        self.sample = core.Sample(self.temps, 23.5, 500, 1)
        self.color = np.zeros((330, 384, 3), dtype=np.uint8)

    def test_encoding_dimensions_shared_stamp_and_no_mutation(self):
        original = self.temps.copy()
        stamp = object()
        raw, thermal, ambient = sample_to_messages(
            self.sample, self.color, stamp, "mlx90640_link", Message, Message)
        self.assertEqual((raw.encoding, raw.width, raw.height, raw.step),
                         ("32FC1", 32, 24, 128))
        self.assertEqual((thermal.encoding, thermal.width, thermal.height, thermal.step),
                         ("bgr8", 384, 330, 1152))
        self.assertEqual(len(raw.data), 24 * 32 * 4)
        self.assertEqual(len(thermal.data), 330 * 384 * 3)
        self.assertEqual(raw.is_bigendian, 0)
        np.testing.assert_array_equal(np.frombuffer(raw.data, dtype="<f4").reshape(24, 32), original)
        np.testing.assert_array_equal(self.temps, original)
        for message in (raw, thermal, ambient):
            self.assertIs(message.header.stamp, stamp)
            self.assertEqual(message.header.frame_id, "mlx90640_link")
        self.assertEqual((ambient.temperature, ambient.variance), (23.5, 0.0))

    def test_node_rejects_invalid_parameters_before_loading_library(self):
        modules = {"sensor_msgs": SimpleNamespace(),
                   "sensor_msgs.msg": SimpleNamespace(Image=Message, Temperature=Message),
                   "std_msgs": SimpleNamespace(),
                   "std_msgs.msg": SimpleNamespace(String=Message)}
        invalid = [
            {}, {"~capture_file": "offline", "~port": "/FAKE"},
            {"~capture_file": "offline", "~baud": 0},
            {"~capture_file": "offline", "~frame_id": ""},
            {"~capture_file": "offline", "~frame_id": "/mlx90640_link"},
            {"~capture_file": "offline", "~tmin": float("nan")},
            {"~capture_file": "offline", "~tmax": float("inf")},
            {"~capture_file": "offline", "~tmin": 30, "~tmax": 20},
        ]
        for params in invalid:
            with self.subTest(params=params):
                rospy = SimpleNamespace(get_param=lambda name, default: params.get(name, default))
                with patch.dict(sys.modules, modules), patch.object(core, "Decoder") as decoder:
                    with self.assertRaises(ValueError):
                        run_node(rospy)
                    decoder.assert_not_called()

    def test_node_relative_topics_nonlatched_and_replay_grace_period(self):
        publishers, waits, stamps = [], [], []
        sample = self.sample

        class Publisher:
            def __init__(self, name, typ, **kwargs):
                self.name, self.options, self.messages = name, kwargs, []
                publishers.append(self)

            def publish(self, message):
                self.messages.append(message)

        class Replay:
            def __init__(self, *args, **kwargs):
                pass

            def samples(self):
                yield sample

            def close(self):
                pass

            def _wait(self, seconds):
                waits.append(seconds)

        def now():
            stamp = object()
            stamps.append(stamp)
            return stamp

        params = {"~capture_file": "offline"}
        rospy = SimpleNamespace(get_param=lambda name, default: params.get(name, default),
                                is_shutdown=lambda: False, logwarn=lambda message: None,
                                loginfo=lambda message: None, on_shutdown=lambda fn: None,
                                Publisher=Publisher, Time=SimpleNamespace(now=now))
        modules = {"sensor_msgs": SimpleNamespace(),
                   "sensor_msgs.msg": SimpleNamespace(Image=Message, Temperature=Message),
                   "std_msgs": SimpleNamespace(),
                   "std_msgs.msg": SimpleNamespace(String=Message)}
        with patch.dict(sys.modules, modules), patch.object(core, "Decoder"), \
                patch.object(core, "render_image", return_value=self.color), \
                patch("mlx90640_ros.node.ReplayTransport", Replay):
            self.assertEqual(run_node(rospy), 0)
        self.assertEqual([p.name for p in publishers],
                         ["temperature_image", "thermal_image", "ambient_temperature",
                          "hotspot"])
        self.assertTrue(all(p.options == {"queue_size": 1, "latch": False} for p in publishers))
        self.assertEqual(waits, [0.5])
        self.assertEqual(len(stamps), 1)
        self.assertTrue(all(p.messages[0].header.stamp is stamps[0] for p in publishers[:3]))
        payload = json.loads(publishers[3].messages[0].data)
        self.assertEqual(payload["floor"], "")
        self.assertEqual(payload["row_index"], 23)
        self.assertEqual(payload["col_index"], 31)


class ViewerTests(unittest.TestCase):
    def test_cli_source_is_required_and_mutually_exclusive(self):
        parser = build_parser()
        for args in ([], ["--port", "/FAKE", "--capture", "offline"]):
            with self.assertRaises(SystemExit):
                parser.parse_args(args)
        args = parser.parse_args(["--capture", "offline", "--headless"])
        self.assertEqual((args.baud, args.scale, args.save_every, args.replay_rate),
                         (460800, 12, 10, 2))

    def test_save_reports_false_return_and_rejects_repository_path(self):
        with tempfile.TemporaryDirectory(prefix="mlx90640-test-") as directory:
            with self.assertRaises(IOError):
                save_image(np.zeros((2, 2, 3), np.uint8), directory, 1,
                           imwrite=lambda path, image: False)
        repo = Path(__file__).resolve().parents[3]
        with self.assertRaises(ValueError):
            save_image(np.zeros((2, 2, 3), np.uint8), repo, 1,
                       imwrite=lambda path, image: True)


if __name__ == "__main__":
    unittest.main()
