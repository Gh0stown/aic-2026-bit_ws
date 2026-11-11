#!/usr/bin/env python3
"""Offline tests for hotspot extraction and floor mapping; no ROS needed.

Set MLX90640_LIBRARY to a freshly built shared library to include the real
capture regression (same requirement as test_core.py).
"""

import json
import os
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mlx90640_ros import (Decoder, Sample, iter_capture_payloads, load_capture,
                          render_image)
from mlx90640_ros.hotspot import (find_hotspot, floor_for_row,
                                  parse_floor_bands, parse_roi)
from mlx90640_ros.node import hotspot_message, hotspot_payload

CAPTURE = Path(__file__).parent / "data" / "capture.bin"


def gaussian(row, col, amplitude=10.0, sigma=1.8, base=22.0):
    ys, xs = np.mgrid[0:24, 0:32]
    return (base + amplitude * np.exp(
        -((ys - row) ** 2 + (xs - col) ** 2) / (2.0 * sigma ** 2))).astype(np.float32)


class FakeString:
    def __init__(self):
        self.data = ""


class HotspotGeometryTests(unittest.TestCase):
    def test_subpixel_centre_beats_integer_pixel(self):
        temps = gaussian(10.3, 20.7)
        hotspot = find_hotspot(temps, delta_c=2.0)
        self.assertEqual((hotspot.row_index, hotspot.col_index), (10, 21))
        self.assertAlmostEqual(hotspot.row, 10.3, delta=0.25)
        self.assertAlmostEqual(hotspot.col, 20.7, delta=0.25)
        self.assertAlmostEqual(hotspot.peak_c, float(temps[10, 21]), places=5)

    def test_box_covers_the_hottest_pixels_only(self):
        temps = gaussian(12.0, 16.0)
        hotspot = find_hotspot(temps, delta_c=2.0)
        r0, c0, r1, c1 = hotspot.box
        self.assertLessEqual(r0, 12)
        self.assertLessEqual(c0, 16)
        self.assertGreaterEqual(r1, 12)
        self.assertGreaterEqual(c1, 16)
        self.assertGreater(hotspot.area, 1)
        # Every box pixel is at least threshold; a cold pixel is outside.
        self.assertGreaterEqual(float(temps[12, 16]), hotspot.threshold_c)
        self.assertLess(float(temps[0, 0]), hotspot.threshold_c)
        self.assertGreater(hotspot.mean_c, hotspot.threshold_c)

    def test_wider_delta_grows_the_box(self):
        temps = gaussian(8.0, 8.0)
        small = find_hotspot(temps, delta_c=0.5)
        large = find_hotspot(temps, delta_c=5.0)
        self.assertGreaterEqual(large.area, small.area)
        self.assertGreaterEqual(large.box[2] - large.box[0], small.box[2] - small.box[0])

    def test_two_regions_keeps_only_the_hottest(self):
        temps = gaussian(5.0, 5.0, amplitude=10.0) + gaussian(18.0, 26.0, amplitude=5.0) - 22.0
        hotspot = find_hotspot(temps, delta_c=2.0)
        self.assertEqual((hotspot.row_index, hotspot.col_index), (5, 5))
        r0, c0, r1, c1 = hotspot.box
        self.assertLess(r1, 12)
        self.assertLess(c1, 12)

    def test_border_peak_falls_back_to_integer_pixel(self):
        temps = gaussian(0.0, 0.0)
        hotspot = find_hotspot(temps)
        self.assertEqual((hotspot.row_index, hotspot.col_index), (0, 0))
        self.assertEqual(hotspot.row, 0.0)
        self.assertEqual(hotspot.col, 0.0)
        self.assertEqual(hotspot.box[:2], (0, 0))

    def test_flat_frame_is_one_block_not_a_crash(self):
        hotspot = find_hotspot(np.full((24, 32), 25.0, dtype=np.float32))
        self.assertEqual((hotspot.row, hotspot.col), (0.0, 0.0))
        self.assertEqual(hotspot.area, 24 * 32)
        self.assertEqual(hotspot.box, (0, 0, 23, 31))

    def test_roi_ignores_a_hotter_background(self):
        temps = gaussian(5.0, 20.0, amplitude=5.0) + gaussian(20.0, 5.0, amplitude=10.0) - 22.0
        self.assertEqual(find_hotspot(temps).row_index, 20)
        limited = find_hotspot(temps, roi=(0, 0, 12, 32))
        self.assertEqual((limited.row_index, limited.col_index), (5, 20))

    def test_invalid_inputs_are_rejected(self):
        good = gaussian(10.0, 10.0)
        for bad in (good[:23], good[:, :31], np.where(good > 30, np.nan, good).astype(np.float32)):
            with self.assertRaises(ValueError):
                find_hotspot(bad)
        for delta in (-1.0, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                find_hotspot(good, delta_c=delta)


class FloorMappingTests(unittest.TestCase):
    def test_bands_parse_and_sort(self):
        bands = parse_floor_bands("1F:16-23, 2F:8-15;3F:0-7")
        self.assertEqual([band[0] for band in bands], ["3F", "2F", "1F"])

    def test_floor_for_row_hits_and_nearest(self):
        bands = parse_floor_bands("1F:16-23,2F:8-15,3F:0-7")
        self.assertEqual(floor_for_row(20, bands), "1F")
        self.assertEqual(floor_for_row(20.9, bands), "1F")
        self.assertEqual(floor_for_row(8, bands), "2F")
        self.assertEqual(floor_for_row(15, bands), "2F")
        self.assertEqual(floor_for_row(0, bands), "3F")
        self.assertEqual(floor_for_row(12.4, parse_floor_bands("2F:8-15")), "2F")
        self.assertEqual(floor_for_row(0, ()), "")

    def test_invalid_bands_are_rejected(self):
        for text in ("2F", "2F:8", "2F:8-24", "2F:10-8", "2F:1-10,1F:5-20", " :1-2"):
            with self.assertRaises(ValueError):
                parse_floor_bands(text)

    def test_empty_band_text_is_allowed(self):
        self.assertEqual(parse_floor_bands(""), ())
        self.assertEqual(parse_floor_bands(None), ())


class RoiTests(unittest.TestCase):
    def test_parse_roi(self):
        self.assertIsNone(parse_roi(""))
        self.assertIsNone(parse_roi(None))
        self.assertEqual(parse_roi("2,4,20,30"), (2, 4, 20, 30))
        self.assertEqual(parse_roi("0,0,24,32"), (0, 0, 24, 32))

    def test_invalid_roi(self):
        for text in ("1,2,3", "a,b,c,d", "0,0,25,32", "5,0,2,4", "-1,0,24,32"):
            with self.assertRaises(ValueError):
                parse_roi(text)


class MessageTests(unittest.TestCase):
    def test_payload_and_message_are_consistent(self):
        hotspot = find_hotspot(gaussian(12.3, 7.6))
        payload = hotspot_payload(hotspot, "2F")
        self.assertEqual(payload["box"], list(hotspot.box))
        self.assertEqual(payload["floor"], "2F")
        self.assertEqual(payload["row_index"], hotspot.row_index)
        message = hotspot_message(hotspot, "2F", FakeString)
        self.assertEqual(json.loads(message.data)["floor"], "2F")
        self.assertEqual(json.loads(message.data)["box"], list(hotspot.box))


class RenderBoxTests(unittest.TestCase):
    def test_box_and_shape(self):
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("OpenCV is optional")
        temps = gaussian(12.0, 16.0)
        hotspot = find_hotspot(temps, delta_c=2.0)
        image = render_image(Sample(temps, 29.0, 0, 1), scale=12,
                             hotspot=hotspot, floor="2F")
        self.assertEqual(image.shape, (24 * 12 + 42, 32 * 12, 3))
        green = (image[:, :, 1] > 200) & (image[:, :, 0] < 100) & (image[:, :, 2] < 100)
        self.assertTrue(green.any())
        plain = render_image(Sample(temps, 29.0, 0, 1), scale=12)
        self.assertEqual(plain.shape, image.shape)


@unittest.skipUnless(os.environ.get("MLX90640_LIBRARY"),
                     "set MLX90640_LIBRARY to enable real vendor capture decoding")
class NativeCaptureHotspotTests(unittest.TestCase):
    def test_real_capture_hotspot_matches_maximum(self):
        decoder = Decoder(os.environ["MLX90640_LIBRARY"])
        eeprom, frames = load_capture(CAPTURE)
        decoder.load_eeprom(eeprom)
        samples = [sample for payload in iter_capture_payloads(frames)
                   if (sample := decoder.process_frame(payload)) is not None]
        self.assertEqual(len(samples), 2)
        for sample in samples:
            hotspot = find_hotspot(sample.temps)
            self.assertAlmostEqual(hotspot.peak_c, float(sample.temps.max()), places=5)
            r0, c0, r1, c1 = hotspot.box
            self.assertTrue(r0 <= hotspot.row_index <= r1)
            self.assertTrue(c0 <= hotspot.col_index <= c1)
            self.assertGreaterEqual(hotspot.area, 1)


if __name__ == "__main__":
    unittest.main()
