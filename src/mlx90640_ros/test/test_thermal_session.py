#!/usr/bin/env python3
"""Tests for the GUI-facing thermal session; no tkinter and no hardware.

Set MLX90640_LIBRARY to include replay/session tests (same as test_core.py).
"""

import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mlx90640_ros import Sample
from mlx90640_ros.thermal_session import FloorTable, ThermalSession

CAPTURE = Path(__file__).parent / "data" / "capture.bin"


def gaussian(row, col, amplitude=10.0, sigma=1.8, base=22.0):
    ys, xs = np.mgrid[0:24, 0:32]
    return (base + amplitude * np.exp(
        -((ys - row) ** 2 + (xs - col) ** 2) / (2.0 * sigma ** 2))).astype(np.float32)


class FloorTableTests(unittest.TestCase):
    def test_text_roundtrip(self):
        table = FloorTable.from_text("1F:16-23,2F:8-15,3F:0-7")
        self.assertEqual(table.to_text(), "3F:0-7,2F:8-15,1F:16-23")
        self.assertEqual(table.floor_for(20), "1F")
        self.assertEqual(table.floor_for(4), "3F")

    def test_record_rebuilds_contiguous_bands(self):
        table = FloorTable()
        table.record("2F", 12)
        table.record("1F", 20)
        table.record("3F", 4)
        self.assertEqual(table.to_text(), "3F:0-7,2F:8-15,1F:16-23")
        self.assertEqual(table.floor_for(0), "3F")
        self.assertEqual(table.floor_for(23), "1F")

    def test_record_replaces_existing_floor(self):
        table = FloorTable.from_text("1F:16-23,2F:8-15")
        table.record("1F", 17)
        self.assertEqual(table.to_text(), "2F:0-13,1F:14-23")
        self.assertEqual(table.floor_for(10), "2F")
        self.assertEqual(table.floor_for(17), "1F")

    def test_record_rejects_bad_values(self):
        table = FloorTable()
        for name, row in (("", 12), ("1F", -1), ("1F", 24)):
            with self.assertRaises(ValueError):
                table.record(name, row)


class SessionValidationTests(unittest.TestCase):
    def test_source_must_be_exclusive_and_explicit(self):
        for kwargs in ({}, {"port": "/FAKE", "capture": "offline"}):
            with self.assertRaises(ValueError):
                ThermalSession(**kwargs)

    def test_invalid_parameters_are_rejected(self):
        for kwargs in ({"capture": "offline", "scale": 0},
                       {"capture": "offline", "delta_c": -1},
                       {"capture": "offline", "tmin": 30, "tmax": 20},
                       {"capture": "offline", "tmax": float("inf")}):
            with self.assertRaises(ValueError):
                ThermalSession(**kwargs)


@unittest.skipUnless(os.environ.get("MLX90640_LIBRARY"),
                     "set MLX90640_LIBRARY to enable replay/session tests")
class SessionTests(unittest.TestCase):
    def setUp(self):
        self.session = ThermalSession(capture=str(CAPTURE), loop=True,
                                      replay_rate=8.0,
                                      floor_table="1F:16-23,2F:8-15,3F:0-7")

    def tearDown(self):
        self.session.stop()

    def wait_for_frame(self, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.session.latest()
            if frame is not None:
                return frame
            time.sleep(0.05)
        self.fail("no thermal frame within {} s".format(timeout))

    def test_process_builds_frame_and_summary(self):
        sample = Sample(gaussian(12.3, 7.6), 29.5, 0, 1)
        frame = self.session.process(sample)
        self.assertEqual(frame.bgr.shape, (24 * 12 + 42, 32 * 12, 3))
        self.assertEqual(frame.temps.shape, (24, 32))
        self.assertEqual(frame.temps.dtype, np.float32)
        self.assertNotEqual(frame.temps.ctypes.data, sample.temps.ctypes.data)
        summary = frame.summary()
        self.assertEqual(summary["floor"], "2F")
        self.assertAlmostEqual(summary["peak_c"], float(sample.temps.max()), places=3)
        self.assertEqual(len(summary["box"]), 4)

    def test_replay_session_publishes_latest_frame(self):
        self.session.start()
        frame = self.wait_for_frame()
        self.assertIsNone(self.session.error())
        self.assertGreaterEqual(frame.hotspot.area, 1)
        self.assertIn(frame.floor, ("1F", "2F", "3F"))

    def test_record_floor_uses_explicit_row_and_latest(self):
        self.session.record_floor("2F", 12)
        self.assertEqual(self.session.floor_table.floor_for(12), "2F")
        self.session.start()
        frame = self.wait_for_frame()
        updated = self.session.record_floor("1F")
        self.assertEqual(updated.floor_for(frame.hotspot.row), "1F")

    def test_save_snapshot_writes_png_and_json(self):
        self.session.start()
        frame = self.wait_for_frame()
        with tempfile.TemporaryDirectory(prefix="thermal-session-") as folder:
            png, data = self.session.save_snapshot(folder, prefix="unit", temps=True)
            self.assertTrue(png.is_file())
            self.assertTrue(data.is_file())
            payload = json.loads(data.read_text(encoding="utf-8"))
            self.assertEqual(payload["floor"], frame.floor)
            self.assertEqual(len(payload["temps"]), 24)


if __name__ == "__main__":
    unittest.main()
