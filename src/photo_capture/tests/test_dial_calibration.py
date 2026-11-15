#!/usr/bin/env python3
"""Synthetic tests for the G1b0 candidate-calibration tool.

All images, ticks, rings and blue boundaries here are synthetic fixtures.
They are not the reference photo, not a real deli calibration and not a
real accuracy measurement.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import dial_calibration as dc
import dial_math as dm


def write_image(path: Path, width: int = 200, height: int = 200) -> str:
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.rectangle(image, (0, 0), (width - 1, height - 1), (0, 0, 0), 1)
    cv2.imwrite(str(path), image)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_annotations(image_path: Path, sha: str, width: int = 200,
                     height: int = 200) -> dict:
    axis = np.array([100.0, 100.0])
    ticks = []
    for index, value in enumerate(range(0, 101, 10)):
        angle = 300.0 + index * 10.0
        point = axis + 80.0 * np.array([
            math.cos(math.radians(angle)),
            math.sin(math.radians(angle)),
        ])
        ticks.append({
            "value": value,
            "point": [round(float(point[0]), 4), round(float(point[1]), 4)],
            "uncertainty_px": 1.0,
            "note": "synthetic fixture",
        })
    ring = []
    for index in range(24):
        angle = index * 15.0
        point = axis + 90.0 * np.array([
            math.cos(math.radians(angle)),
            math.sin(math.radians(angle)),
        ])
        ring.append([round(float(point[0]), 4), round(float(point[1]), 4)])
    blue = []
    for name, angle in (("green_blue", 30.0), ("blue_end", 60.0)):
        point = axis + 90.0 * np.array([
            math.cos(math.radians(angle)),
            math.sin(math.radians(angle)),
        ])
        blue.append({
            "name": name,
            "point": [round(float(point[0]), 4), round(float(point[1]), 4)],
            "uncertainty_px": 1.0,
        })
    return {
        "version": "g1b0-annotations-1",
        "model": "synthetic fixture",
        "source_image": {
            "path": str(image_path),
            "sha256": sha,
            "width": width,
            "height": height,
        },
        "coordinate_convention": "synthetic x right, y down, clockwise positive",
        "humidity_axis": {"point": [100.0, 100.0], "note": "synthetic"},
        "ticks": ticks,
        "blue_segment": {"boundaries": blue, "note": "synthetic"},
        "ring_points": ring,
        "ring_edge": "synthetic circle",
        "notes": "synthetic fixture only",
    }


class GeometryTests(unittest.TestCase):
    def test_angle_from_axis_hand_computed(self):
        axis = np.array([100.0, 100.0])
        matrix = np.eye(2)
        cases = (
            (np.array([200.0, 100.0]), 0.0),
            (np.array([100.0, 200.0]), 90.0),
            (np.array([0.0, 100.0]), 180.0),
            (np.array([100.0, 0.0]), 270.0),
        )
        for point, expected in cases:
            with self.subTest(point=point):
                self.assertAlmostEqual(
                    dc.angle_from_axis(point, axis, matrix), expected, places=9)

    def test_short_arc_midpoint_crosses_zero(self):
        cases = (
            (350.0, 10.0, 0.0),
            (10.0, 350.0, 0.0),
            (30.0, 60.0, 45.0),
            (300.0, 30.0, 345.0),
            (166.0, 226.0, 196.0),
        )
        for first, second, expected in cases:
            with self.subTest(first=first, second=second):
                self.assertAlmostEqual(
                    dc.short_arc_midpoint(first, second), expected, places=9)

    def test_unwrap_angles_hand_computed(self):
        self.assertEqual(dc.unwrap_angles([350.0, 10.0, 30.0]),
                         [350.0, 370.0, 390.0])
        self.assertEqual(dc.unwrap_angles([0.0, 10.0, 20.0]),
                         [0.0, 10.0, 20.0])
        self.assertEqual(dc.unwrap_angles([359.0, 1.0, 2.0]),
                         [359.0, 361.0, 362.0])

    def test_ellipse_matrix_circle_and_axis_swap(self):
        center = np.array([50.0, 60.0])
        circle_a = dc.ellipse_matrix(center, (200.0, 200.0), 0.0)
        circle_b = dc.ellipse_matrix(center, (200.0, 200.0), 37.0)
        self.assertTrue(np.allclose(circle_a["matrix"], circle_b["matrix"]))
        self.assertTrue(np.allclose(circle_a["matrix"], np.eye(2) * 0.01))

        ellipse = dc.ellipse_matrix(center, (200.0, 100.0), 0.0)
        swapped = dc.ellipse_matrix(center, (100.0, 200.0), 90.0)
        self.assertTrue(np.allclose(ellipse["matrix"], swapped["matrix"]))

    def test_zero_vector_is_rejected(self):
        axis = np.array([100.0, 100.0])
        with self.assertRaises(dc.CalibrationError):
            dc.angle_from_axis(axis, axis, np.eye(2))

    def test_near_circle_axis_swap_does_not_rotate_points(self):
        center = np.array([50.0, 60.0])
        first = dc.ellipse_matrix(center, (200.0, 199.0), 0.0)
        second = dc.ellipse_matrix(center, (199.0, 200.0), 90.0)
        self.assertTrue(np.allclose(first["matrix"], second["matrix"], atol=1e-12))
        point = np.array([150.0, 60.0])
        axis = np.array([50.0, 60.0])
        self.assertAlmostEqual(
            dc.angle_from_axis(point, axis, first["matrix"]),
            dc.angle_from_axis(point, axis, second["matrix"]), places=9)


class ValidationTests(unittest.TestCase):
    def _fixture(self, tmp: Path):
        image = tmp / "synthetic.png"
        sha = write_image(image)
        annotations = make_annotations(image, sha)
        return image, annotations

    def test_valid_synthetic_annotation_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            image, annotations = self._fixture(Path(directory))
            data = dc.validate_annotations(annotations, (200, 200, 3))
            self.assertEqual(len(data["ticks"]), 11)
            self.assertEqual(len(data["ring_points"]), 24)
            self.assertIsNotNone(dc.load_image(str(image), annotations["source_image"]))

    def test_hash_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            image, annotations = self._fixture(Path(directory))
            bad = dict(annotations)
            bad["source_image"] = dict(annotations["source_image"])
            bad["source_image"]["sha256"] = "0" * 64
            with self.assertRaises(dc.CalibrationError):
                dc.load_image(str(image), bad["source_image"])

    def test_size_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            image, annotations = self._fixture(Path(directory))
            bad = dict(annotations)
            bad["source_image"] = dict(annotations["source_image"])
            bad["source_image"]["width"] = 201
            with self.assertRaises(dc.CalibrationError):
                dc.load_image(str(image), bad["source_image"])

    def test_missing_ticks_and_bad_values_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            _, annotations = self._fixture(Path(directory))
            short = dict(annotations)
            short["ticks"] = annotations["ticks"][:10]
            with self.assertRaises(dc.CalibrationError):
                dc.validate_annotations(short, (200, 200, 3))
            bad_value = json.loads(json.dumps(annotations))
            bad_value["ticks"][0]["value"] = 5
            with self.assertRaises(dc.CalibrationError):
                dc.validate_annotations(bad_value, (200, 200, 3))

    def test_out_of_bounds_and_duplicate_points_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            _, annotations = self._fixture(Path(directory))
            outside = json.loads(json.dumps(annotations))
            outside["ticks"][0]["point"] = [-1.0, 10.0]
            with self.assertRaises(dc.CalibrationError):
                dc.validate_annotations(outside, (200, 200, 3))
            duplicate = json.loads(json.dumps(annotations))
            duplicate["ring_points"][1] = list(duplicate["ring_points"][0])
            with self.assertRaises(dc.CalibrationError):
                dc.validate_annotations(duplicate, (200, 200, 3))

    def test_degenerate_ring_and_clustered_ring_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            _, annotations = self._fixture(Path(directory))
            clustered = json.loads(json.dumps(annotations))
            clustered["ring_points"] = [
                [100.0 + 0.01 * index, 100.0] for index in range(12)
            ]
            with self.assertRaises(dc.CalibrationError):
                dc.validate_annotations(clustered, (200, 200, 3))

            collinear = [np.array([50.0 + index, 100.0]) for index in range(12)]
            with self.assertRaises(dc.CalibrationError):
                dc.fit_ring_ellipse(collinear)

    def test_zero_vector_tick_and_blue_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            _, annotations = self._fixture(Path(directory))
            tick_zero = json.loads(json.dumps(annotations))
            tick_zero["ticks"][6]["point"] = [100.0, 100.0]
            with self.assertRaises(dc.CalibrationError):
                dc.validate_annotations(tick_zero, (200, 200, 3))
            blue_zero = json.loads(json.dumps(annotations))
            blue_zero["blue_segment"]["boundaries"][0]["point"] = [100.0, 100.0]
            with self.assertRaises(dc.CalibrationError):
                dc.validate_annotations(blue_zero, (200, 200, 3))

    def test_invalid_tick_geometry_is_rejected(self):
        axis = np.array([100.0, 100.0])
        matrix = np.eye(2)
        points = [
            axis + 80.0 * np.array([math.cos(math.radians(angle)),
                                    math.sin(math.radians(angle))])
            for angle in (100.0, 100.0, 100.0, 100.0, 100.0,
                          100.0, 100.0, 100.0, 100.0, 100.0, 100.0)
        ]
        with self.assertRaises(dc.CalibrationError):
            dc.build_table(points, axis, matrix)


class RunTests(unittest.TestCase):
    def _fixture(self, tmp: Path, bad_hash: bool = False):
        image = tmp / "synthetic.png"
        sha = write_image(image)
        annotations = make_annotations(image, sha)
        if bad_hash:
            annotations["source_image"]["sha256"] = "0" * 64
        annotation_path = tmp / "annotations.json"
        annotation_path.write_text(
            json.dumps(annotations, ensure_ascii=False), encoding="utf-8")
        return image, annotation_path

    def test_run_writes_candidate_and_overlay(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image, annotation_path = self._fixture(tmp)
            output = tmp / "review"
            candidate = dc.run(str(image), str(annotation_path), str(output))
            self.assertEqual(candidate["status"], "candidate")
            self.assertEqual(len(candidate["table_deg"]), 11)
            self.assertLess(candidate["table_deg"][-1] - candidate["table_deg"][0], 360.0)
            self.assertTrue((output / "candidate_calib.json").is_file())
            self.assertTrue((output / "calibration_overlay.png").is_file())
            saved = json.loads((output / "candidate_calib.json").read_text(encoding="utf-8"))
            self.assertEqual(saved["status"], "candidate")
            self.assertEqual(saved["table_val"], [float(value) for value in range(0, 101, 10)])

    def test_existing_empty_output_dir_is_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image, annotation_path = self._fixture(tmp)
            output = tmp / "review"
            output.mkdir()
            dc.run(str(image), str(annotation_path), str(output))
            self.assertTrue((output / "candidate_calib.json").is_file())
            self.assertTrue((output / "calibration_overlay.png").is_file())

    def test_nonempty_output_is_rejected_and_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image, annotation_path = self._fixture(tmp)
            output = tmp / "review"
            output.mkdir()
            stale = output / "candidate_calib.json"
            stale.write_text("stale-result", encoding="utf-8")
            with self.assertRaises(dc.CalibrationError):
                dc.run(str(image), str(annotation_path), str(output))
            self.assertEqual(stale.read_text(encoding="utf-8"), "stale-result")
            self.assertFalse((output / "calibration_overlay.png").exists())
            self.assertEqual(len(list(output.iterdir())), 1)

    def test_bad_hash_in_empty_output_leaves_no_final_files(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image, annotation_path = self._fixture(tmp, bad_hash=True)
            output = tmp / "review"
            with self.assertRaises(dc.CalibrationError):
                dc.run(str(image), str(annotation_path), str(output))
            self.assertFalse((output / "candidate_calib.json").exists())
            self.assertFalse((output / "calibration_overlay.png").exists())
            self.assertFalse(output.exists() and any(output.iterdir()))

    def test_input_alias_or_inside_output_is_rejected_and_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image, annotation_path = self._fixture(tmp)
            image_hash = hashlib.sha256(image.read_bytes()).hexdigest()
            with self.assertRaises(dc.CalibrationError):
                dc.run(str(image), str(annotation_path), str(image))
            self.assertEqual(hashlib.sha256(image.read_bytes()).hexdigest(), image_hash)

            output = tmp / "review"
            output.mkdir()
            nested_annotation = output / "annotations.json"
            nested_annotation.write_bytes(annotation_path.read_bytes())
            annotation_hash = hashlib.sha256(nested_annotation.read_bytes()).hexdigest()
            with self.assertRaises(dc.CalibrationError):
                dc.run(str(image), str(nested_annotation), str(output))
            self.assertEqual(
                hashlib.sha256(nested_annotation.read_bytes()).hexdigest(),
                annotation_hash)
            self.assertEqual(list(output.iterdir()), [nested_annotation])

    def test_imwrite_false_or_exception_leaves_no_final_files(self):
        for mode in ("false", "raise"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                tmp = Path(directory)
                image, annotation_path = self._fixture(tmp)
                output = tmp / "review"
                if mode == "false":
                    patcher = mock.patch.object(dc.cv2, "imwrite", return_value=False)
                else:
                    patcher = mock.patch.object(
                        dc.cv2, "imwrite", side_effect=RuntimeError("injected"))
                with patcher:
                    with self.assertRaises(dc.CalibrationError):
                        dc.run(str(image), str(annotation_path), str(output))
                self.assertFalse((output / "candidate_calib.json").exists())
                self.assertFalse((output / "calibration_overlay.png").exists())
                self.assertFalse(output.exists() and any(output.iterdir()))

    def test_json_write_failure_leaves_no_final_files(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image, annotation_path = self._fixture(tmp)
            output = tmp / "review"
            with mock.patch.object(Path, "write_text", side_effect=OSError("injected")):
                with self.assertRaises(dc.CalibrationError):
                    dc.run(str(image), str(annotation_path), str(output))
            self.assertFalse((output / "candidate_calib.json").exists())
            self.assertFalse((output / "calibration_overlay.png").exists())
            self.assertFalse(output.exists() and any(output.iterdir()))

    def test_zero_vector_tick_blocks_candidate(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image, annotation_path = self._fixture(tmp)
            data = json.loads(annotation_path.read_text(encoding="utf-8"))
            data["ticks"][6]["point"] = [100.0, 100.0]
            annotation_path.write_text(json.dumps(data), encoding="utf-8")
            output = tmp / "review"
            with self.assertRaises(dc.CalibrationError):
                dc.run(str(image), str(annotation_path), str(output))
            self.assertFalse((output / "candidate_calib.json").exists())

    def test_cli_success_and_failure_exit_codes(self):
        script = Path(__file__).resolve().parents[1] / "scripts" / "dial_calibration.py"
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image, annotation_path = self._fixture(tmp)
            output = tmp / "review"
            success = subprocess.run(
                [sys.executable, "-B", str(script),
                 "--image", str(image), "--annotations", str(annotation_path),
                 "--output-dir", str(output)],
                cwd=str(tmp), capture_output=True, text=True, timeout=60)
            self.assertEqual(success.returncode, 0, success.stderr)
            self.assertTrue((output / "candidate_calib.json").is_file())

            bad = tmp / "bad.json"
            bad.write_text(
                json.dumps({**json.loads(annotation_path.read_text(encoding="utf-8")),
                            "source_image": {
                                **json.loads(annotation_path.read_text(encoding="utf-8"))["source_image"],
                                "sha256": "0" * 64,
                            }}, ensure_ascii=False),
                encoding="utf-8")
            failure = subprocess.run(
                [sys.executable, "-B", str(script),
                 "--image", str(image), "--annotations", str(bad),
                 "--output-dir", str(tmp / "bad-review")],
                cwd=str(tmp), capture_output=True, text=True, timeout=60)
            self.assertEqual(failure.returncode, 2, failure.stderr)


class ImportTests(unittest.TestCase):
    def test_import_has_no_forbidden_modules_threads_or_files(self):
        scripts = Path(__file__).resolve().parents[1] / "scripts"
        code = (
            "import json, sys, threading\n"
            "sys.path.insert(0, {path!r})\n"
            "import dial_calibration\n"
            "forbidden = [name for name in "
            "('gauge', 'yolo_detect', 'ultralytics', 'rospy', 'tkinter') "
            "if name in sys.modules]\n"
            "print(json.dumps({{'forbidden': forbidden, "
            "'threads': threading.active_count()}}))\n"
        ).format(path=str(scripts))
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(
                [sys.executable, "-B", "-c", code],
                cwd=tmp, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            payload = json.loads(result.stdout.strip())
            self.assertEqual(payload["forbidden"], [])
            self.assertEqual(payload["threads"], 1)
            self.assertEqual(os.listdir(tmp), [])


if __name__ == "__main__":
    unittest.main()
