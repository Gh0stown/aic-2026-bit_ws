#!/usr/bin/env python3
"""Synthetic tests for the G1b1 specified-ROI single-frame reader.

All rings, needles, colors and calibrations here are synthetic fixtures.  They
are not the reference photo, not real dial truth and not real accuracy data.
"""
import argparse
import csv
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
import dial_reading as dr

REPO_ROOT = Path(__file__).resolve().parents[3]
REFERENCE_IMAGE = Path(
    '/home/mowen/.dsh/attachments/v1/objects/be/'
    'be71a511ebfe297c18c86feebc8d99cdefe9030e507a82e65f50d2aed8d36f30')
REFERENCE_SHA256 = ('be71a511ebfe297c18c86feebc8d99cdefe9030e507a82e65f50d2aed8d36f30')
CANDIDATE_PATH = REPO_ROOT / '.gauge_calibration_review' / 'r2-final' / 'candidate_calib.json'
CANDIDATE_SHA256 = 'd7dccc3e6d80f24199405dce07a516fd119d4502724efa413983682d2e5df198'
LOWRES_FIRST = Path(
    '/home/mowen/Pictures/photo_capture/仪表识别/'
    '仪表识别_20261114_215027_542_0001.jpg')
LOWRES_LAST = Path(
    '/home/mowen/Pictures/photo_capture/仪表识别/'
    '仪表识别_20261114_215902_300_0200.jpg')
REFERENCE_ROI = [430, 870, 420, 390]
LOWRES_ROI = [578, 222, 80, 72]


def _matrix(axes, angle_deg=0.0):
    width, height = float(axes[0]), float(axes[1])
    if width >= height:
        major, minor, major_angle = width / 2.0, height / 2.0, angle_deg
    else:
        major, minor, major_angle = height / 2.0, width / 2.0, angle_deg + 90.0
    theta = math.radians(major_angle)
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]], dtype=float)
    return rotation @ np.diag([1.0 / major, 1.0 / minor]) @ rotation.T


def make_calibration(center=(300.0, 300.0), axes=(354.0, 334.0),
                     angle_deg=0.0, blue_mid=45.0,
                     blue_angles=(30.0, 60.0), table_deg=None) -> dict:
    center = np.asarray(center, dtype=float)
    axes = np.asarray(axes, dtype=float)
    matrix = _matrix(axes, angle_deg)
    inverse = np.linalg.inv(matrix)
    if table_deg is None:
        table_deg = [300.0, 320.0, 340.0, 360.0, 380.0,
                     400.0, 420.0, 440.0, 460.0, 480.0, 500.0]
    boundaries = []
    for degree in blue_angles:
        normalized = 0.94 * np.array([math.cos(math.radians(degree)),
                                      math.sin(math.radians(degree))])
        boundaries.append(center + inverse @ normalized)
    return {
        "calib_version": "synthetic-fixture",
        "calib_sha256": "0" * 64,
        "table_deg": np.asarray(table_deg, dtype=float),
        "table_val": np.arange(0, 101, 10, dtype=float),
        "ring_center": center,
        "ring_axes": axes,
        "matrix": matrix,
        "axis": center.copy(),
        "blue_mid": float(blue_mid),
        "blue_boundaries": boundaries,
    }


def _short_arc_mid_deg(first_deg, second_deg):
    delta = (second_deg - first_deg) % 360.0
    if delta > 180.0:
        delta -= 360.0
    return (first_deg + delta / 2.0) % 360.0


def _angle_about(point, origin, matrix):
    vector = np.asarray(matrix, dtype=float) @ (
        np.asarray(point, dtype=float) - np.asarray(origin, dtype=float))
    return math.degrees(math.atan2(float(vector[1]), float(vector[0]))) % 360.0


def _unit_deg(degree):
    return np.array([math.cos(math.radians(degree)),
                     math.sin(math.radians(degree))], dtype=float)


def make_offset_calibration(center=(300.0, 300.0), radius=177.0,
                            axis_offset_px=(12.0, -9.0),
                            ring_blue_angles=(20.0, 50.0),
                            table_deg=None) -> dict:
    """Fixture whose humidity axis deliberately differs from the ring centre.

    The ring is isotropic, so normalised angles equal pixel angles.  The blue
    boundary points sit on the ring at ``0.94 * radius`` from the ring centre,
    exactly where ``sample_blue`` samples.  ``blue_mid`` is therefore the
    *axis-centred* midpoint, while the ring-centred midpoint is 35 degrees.
    """
    center = np.asarray(center, dtype=float)
    axis = center + np.asarray(axis_offset_px, dtype=float)
    matrix = np.eye(2, dtype=float) / float(radius)
    boundaries = [center + float(radius) * 0.94 * _unit_deg(phi)
                  for phi in ring_blue_angles]
    axis_angles = [_angle_about(point, axis, matrix) for point in boundaries]
    blue_mid = _short_arc_mid_deg(axis_angles[0], axis_angles[1])
    if table_deg is None:
        table_deg = [300.0, 320.0, 340.0, 360.0, 380.0,
                     400.0, 420.0, 440.0, 460.0, 480.0, 500.0]
    return {
        "calib_version": "synthetic-offset-axis",
        "calib_sha256": "1" * 64,
        "table_deg": np.asarray(table_deg, dtype=float),
        "table_val": np.arange(0, 101, 10, dtype=float),
        "ring_center": center,
        "ring_axes": np.array([2.0 * radius, 2.0 * radius]),
        "matrix": matrix,
        "axis": axis,
        "blue_mid": float(blue_mid),
        "blue_boundaries": boundaries,
        "ring_blue_angles": tuple(float(v) for v in ring_blue_angles),
    }


def make_offset_scene(needle_deg=340.0, roll_deg=0.0,
                      axis_offset_px=(12.0, -9.0), center=(300.0, 300.0),
                      radius=177.0, ring_blue_angles=(20.0, 50.0),
                      size=600, blur_sigma=0.0, extra_lines=()):
    """Synthetic scene with a non-zero axis/ring-centre offset.

    Rotation is applied about the ring centre: the axis, the needle and the
    colour bands all move together, so a correct reader must estimate the roll
    from the ring-centred blue midpoint and still compensate the needle to the
    unrotated table angle.
    """
    calibration = make_offset_calibration(
        center=center, radius=radius, axis_offset_px=axis_offset_px,
        ring_blue_angles=ring_blue_angles)
    image = np.full((int(size), int(size), 3), 255, dtype=np.uint8)
    c = (int(round(center[0])), int(round(center[1])))
    half = (int(round(radius)), int(round(radius)))
    blue_start = ring_blue_angles[0] + roll_deg
    blue_end = ring_blue_angles[1] + roll_deg
    arcs = [(blue_start, blue_end, (255, 0, 0)),
            (blue_end, 200.0 + roll_deg, (0, 255, 0)),
            (200.0 + roll_deg, 380.0 + roll_deg, (0, 0, 255))]
    for start, end, color in arcs:
        cv2.ellipse(image, c, half, 0, start, end, color, 18)
    theta = math.radians(roll_deg)
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]], dtype=float)
    axis_runtime = np.asarray(center, dtype=float) + rotation @ np.asarray(
        axis_offset_px, dtype=float)
    needle_angle = math.radians(needle_deg + roll_deg)
    length = 0.68 * radius
    end = (int(round(axis_runtime[0] + length * math.cos(needle_angle))),
           int(round(axis_runtime[1] + length * math.sin(needle_angle))))
    cv2.line(image, (int(round(axis_runtime[0])), int(round(axis_runtime[1]))),
             end, (0, 0, 0), 6)
    for extra in extra_lines:
        extra_angle = math.radians(extra + roll_deg)
        extra_end = (int(round(axis_runtime[0] + length * math.cos(extra_angle))),
                     int(round(axis_runtime[1] + length * math.sin(extra_angle))))
        cv2.line(image, (int(round(axis_runtime[0])), int(round(axis_runtime[1]))),
                 extra_end, (0, 0, 0), 6)
    cv2.circle(image, (int(round(axis_runtime[0])), int(round(axis_runtime[1]))),
               5, (0, 0, 0), -1)
    if blur_sigma > 0:
        image = cv2.GaussianBlur(image, (0, 0), blur_sigma)
    return image, calibration


def make_scene(needle_deg=340.0, mode="normal", extra_lines=(),
               size=600, center=(300.0, 300.0), outer=(354.0, 334.0),
               second_ring=False, scale=1.0, roll_deg=0.0, blur_sigma=0.0):
    size = int(round(size * scale))
    center = (center[0] * scale, center[1] * scale)
    outer = (outer[0] * scale, outer[1] * scale)
    image = np.full((size, size, 3), 255, dtype=np.uint8)
    half = (int(round(outer[0] / 2.0)), int(round(outer[1] / 2.0)))
    thickness = max(4, int(round(18 * scale)))
    if mode == "normal":
        arcs = [((30, 60), (255, 0, 0)),
                ((60, 200), (0, 255, 0)),
                ((200, 390), (0, 0, 255))]
        blue_mid, blue_angles = 45.0, (30.0, 60.0)
    elif mode == "cross_zero":
        arcs = [((350, 370), (255, 0, 0)),
                ((0, 10), (255, 0, 0)),
                ((10, 200), (0, 255, 0)),
                ((200, 350), (0, 0, 255))]
        blue_mid, blue_angles = 0.0, (350.0, 10.0)
    elif mode == "missing_blue":
        arcs = [((0, 200), (0, 255, 0)),
                ((200, 360), (0, 0, 255))]
        blue_mid, blue_angles = 45.0, (30.0, 60.0)
    else:
        raise ValueError(mode)
    c = (int(round(center[0])), int(round(center[1])))
    for (start, end), color in arcs:
        cv2.ellipse(image, c, half, 0, start + roll_deg, end + roll_deg, color, thickness)
    if second_ring:
        second_center = (int(round(center[0] - 220)), int(round(center[1] + 230)))
        second_half = (max(20, half[0] // 3), max(20, half[1] // 3))
        cv2.ellipse(image, second_center, second_half, 0, 30, 60, (255, 0, 0), 8)
        cv2.ellipse(image, second_center, second_half, 0, 60, 200, (0, 255, 0), 8)
        cv2.ellipse(image, second_center, second_half, 0, 200, 390, (0, 0, 255), 8)
    angle = math.radians(needle_deg + roll_deg)
    length = 0.68 * max(half)
    end = (int(round(center[0] + length * math.cos(angle))),
           int(round(center[1] + length * math.sin(angle))))
    cv2.line(image, c, end, (0, 0, 0), 6)
    for extra in extra_lines:
        extra_angle = math.radians(extra + roll_deg)
        extra_end = (int(round(center[0] + length * math.cos(extra_angle))),
                     int(round(center[1] + length * math.sin(extra_angle))))
        cv2.line(image, c, extra_end, (0, 0, 0), 6)
    cv2.circle(image, c, 5, (0, 0, 0), -1)
    if blur_sigma > 0:
        image = cv2.GaussianBlur(image, (0, 0), blur_sigma)
    calibration = make_calibration(center=center, axes=outer,
                                   blue_mid=blue_mid, blue_angles=blue_angles)
    return image, calibration


def write_candidate(path: Path, calibration: dict) -> str:
    data = {
        "status": "candidate",
        "calib_version": calibration["calib_version"],
        "source_image": {"sha256": "0" * 64, "width": 600, "height": 600},
        "annotation_sha256": "0" * 64,
        "table_deg": [float(v) for v in calibration["table_deg"]],
        "table_val": [float(v) for v in calibration["table_val"]],
        "ring": {
            "center_px": [float(v) for v in calibration["ring_center"]],
            "fit_axes_px": [float(v) for v in calibration["ring_axes"]],
            "matrix": [[float(v) for v in row] for row in calibration["matrix"]],
        },
        "humidity_axis_px": [float(v) for v in calibration["axis"]],
        "blue_segment": {
            "midpoint_deg": float(calibration["blue_mid"]),
            "boundaries": [{"point_px": [float(v) for v in point]}
                           for point in calibration["blue_boundaries"]],
        },
    }
    path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    return hashlib.sha256(path.read_bytes()).hexdigest()


class SyntheticReadingTests(unittest.TestCase):
    def test_in_range_single_frame_returns_candidate_only(self):
        image, calibration = make_scene(needle_deg=340.0)
        result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        self.assertEqual(result["frame_status"], "ok")
        self.assertEqual(result["status"], "low_conf")
        self.assertEqual(result["reason"], "multi_frame_required")
        self.assertIsNone(result["value"])
        self.assertIsInstance(result["candidate_value"], float)
        self.assertAlmostEqual(result["candidate_value"], 20.0, delta=2.0)
        self.assertIsNone(result["spread_deg"])

    def test_blank_area_returns_out_of_range(self):
        image, calibration = make_scene(needle_deg=220.0)
        result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        self.assertEqual(result["frame_status"], "out_of_range")
        self.assertEqual(result["status"], "out_of_range")
        self.assertIsNone(result["candidate_value"])
        self.assertIsNone(result["value"])

    def test_no_dial_is_not_an_exception(self):
        image = np.full((600, 600, 3), 255, dtype=np.uint8)
        _, calibration = make_scene()
        result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        self.assertEqual(result["frame_status"], "no_dial")
        self.assertEqual(result["status"], "no_dial")
        self.assertIsNone(result["candidate_value"])

    def test_multiple_rings_are_rejected(self):
        image, calibration = make_scene(needle_deg=340.0, second_ring=True)
        result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        self.assertIn(result["frame_status"], ("no_dial", "low_conf"))
        self.assertIsNone(result["candidate_value"])

    def test_missing_blue_is_rejected(self):
        image, calibration = make_scene(mode="missing_blue")
        result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        self.assertEqual(result["frame_status"], "low_conf")
        self.assertIn(result["reason"], ("blue_missing", "blue_too_short"))

    def test_blue_segment_crossing_zero_is_handled(self):
        image, calibration = make_scene(needle_deg=340.0, mode="cross_zero")
        result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        self.assertEqual(result["frame_status"], "ok")
        self.assertAlmostEqual(result["roll_deg"], 0.0, delta=1.0)
        self.assertAlmostEqual(result["candidate_value"], 20.0, delta=2.0)

    def test_two_dark_lines_are_rejected_as_competition(self):
        image, calibration = make_scene(needle_deg=340.0, extra_lines=(100.0,))
        result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        self.assertEqual(result["frame_status"], "low_conf")
        self.assertEqual(result["reason"], "competing_peaks")

    def test_major_values_15_percent_and_blur(self):
        cases = ((302.0, 2.0), (320.0, 10.0), (330.0, 15.0), (340.0, 20.0),
                 (360.0, 30.0), (380.0, 40.0), (400.0, 50.0), (420.0, 60.0),
                 (440.0, 70.0), (460.0, 80.0), (480.0, 90.0), (498.0, 98.0))
        for needle, expected in cases:
            with self.subTest(needle=needle):
                image, calibration = make_scene(needle_deg=needle)
                result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
                self.assertEqual(result["frame_status"], "ok")
                self.assertAlmostEqual(result["candidate_value"], expected, delta=2.0)
        image, calibration = make_scene(needle_deg=340.0, blur_sigma=1.2)
        result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        self.assertEqual(result["frame_status"], "ok")
        self.assertAlmostEqual(result["candidate_value"], 20.0, delta=2.0)

    def test_roll_and_scale_variants(self):
        for roll in (-6.0, 0.0, 6.0):
            for scale in (0.5, 0.75, 1.5):
                with self.subTest(roll=roll, scale=scale):
                    image, calibration = make_scene(
                        needle_deg=340.0, scale=scale, roll_deg=roll)
                    size = image.shape[0]
                    result = dr.estimate_frame(
                        image, calibration, [0, 0, size, size])
                    self.assertEqual(result["frame_status"], "ok")
                    self.assertAlmostEqual(
                        result["candidate_value"], 20.0, delta=2.0)

    def test_near_circle_axis_swap_does_not_rotate_angle(self):
        image, calibration = make_scene(needle_deg=340.0, outer=(354.0, 354.0))
        swapped = dict(calibration)
        swapped["matrix"] = _matrix((354.0, 354.0), 90.0)
        first = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        second = dr.estimate_frame(image, swapped, [0, 0, 600, 600])
        self.assertEqual(first["frame_status"], "ok")
        self.assertEqual(second["frame_status"], "ok")
        self.assertAlmostEqual(first["angle_deg"], second["angle_deg"], delta=0.5)

    def test_roi_is_mapped_to_original_coordinates(self):
        image, calibration = make_scene(needle_deg=340.0, size=700,
                                        center=(350.0, 350.0))
        result = dr.estimate_frame(image, calibration, [50, 50, 600, 600])
        self.assertEqual(result["roi"], [50, 50, 600, 600])
        self.assertEqual(result["frame_status"], "ok")
        self.assertGreater(result["diagnostics"]["ring_center"][0], 300.0)

    def test_near_range_endpoints_are_inside_range(self):
        # Exact 0/100 synthetic needles are sensitive to the 0.1-degree image
        # sampling; use +2/-2 degrees to test the in-range endpoint sides.
        for needle, expected in ((302.0, 2.0), (498.0, 98.0)):
            with self.subTest(needle=needle):
                image, calibration = make_scene(needle_deg=needle)
                result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
                self.assertEqual(result["frame_status"], "ok")
                self.assertAlmostEqual(result["candidate_value"], expected, delta=2.0)

    def test_reference_ring_center_midpoint_hand_geometry(self):
        # Hand-computed fixture: ring centre (300, 300), radius 200, offset
        # (12, -9).  Blue boundaries sit at ring-centred 20/50 degrees, so the
        # reference ring-centred midpoint is exactly 35 degrees.  The
        # axis-centred midpoint follows atan2(0.94*sin(phi) - ey, 0.94*cos(phi)
        # - ex) with (ex, ey) = (12/200, -9/200); it must differ by > 1 degree.
        radius = 200.0
        calibration = make_offset_calibration(
            radius=radius, axis_offset_px=(12.0, -9.0),
            ring_blue_angles=(20.0, 50.0))
        self.assertAlmostEqual(
            dr.reference_ring_center_blue_mid(calibration), 35.0, places=9)
        self.assertGreater(abs(float(calibration["blue_mid"]) - 35.0), 1.0)
        center = np.asarray(calibration["ring_center"], dtype=float)
        for roll in (0.0, 4.0, -6.0):
            with self.subTest(roll=roll):
                ring = {"center": center, "matrix": calibration["matrix"]}
                blue = {
                    "midpoint_deg": (35.0 + roll) % 360.0,
                    "boundary_points": [
                        center + radius * 0.94 * _unit_deg(phi + roll)
                        for phi in (20.0, 50.0)],
                }
                migrated = dr.migrate_axis(ring, blue, calibration)
                self.assertTrue(migrated["ok"])
                wrapped = roll % 360.0
                self.assertAlmostEqual(migrated["roll1_deg"], wrapped, places=9)
                self.assertAlmostEqual(migrated["roll2_deg"], wrapped, places=9)
                theta = math.radians(roll)
                expected_axis = center + np.array([
                    math.cos(theta) * 12.0 - math.sin(theta) * (-9.0),
                    math.sin(theta) * 12.0 + math.cos(theta) * (-9.0)])
                np.testing.assert_allclose(
                    migrated["axis1"], expected_axis, atol=1e-9)

    def test_reference_midpoint_rejects_zero_vector(self):
        calibration = make_offset_calibration()
        broken = dict(calibration)
        broken["blue_boundaries"] = [np.asarray(calibration["ring_center"]),
                                     calibration["blue_boundaries"][1]]
        with self.assertRaises(dr.ReadingConfigError):
            dr.reference_ring_center_blue_mid(broken)

    def test_offset_axis_scene_accepts_and_compensates(self):
        # Regression for the mixed-origin roll bug: every previous synthetic
        # fixture had axis == ring centre, which hides the defect completely.
        image, calibration = make_offset_scene(needle_deg=340.0, roll_deg=0.0)
        result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        self.assertEqual(result["frame_status"], "ok", result["reason"])
        self.assertAlmostEqual(result["roll_deg"], 0.0, delta=0.5)
        self.assertAlmostEqual(result["candidate_value"], 20.0, delta=2.0)
        self.assertLess(result["quality"]["axis_roll_delta_deg"], 0.5)
        self.assertGreater(
            abs(result["diagnostics"]["reference_axis_mid_deg"]
                - result["diagnostics"]["reference_ring_center_mid_deg"]), 1.0)

    def test_offset_axis_roll_variants(self):
        for roll in (-6.0, 6.0):
            with self.subTest(roll=roll):
                image, calibration = make_offset_scene(
                    needle_deg=340.0, roll_deg=roll)
                result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
                self.assertEqual(result["frame_status"], "ok", result["reason"])
                self.assertLess(abs((result["roll_deg"] - roll + 180.0) % 360.0 - 180.0), 1.0)
                self.assertAlmostEqual(
                    result["candidate_value"], 20.0, delta=2.0)

    def test_offset_axis_roi_migration(self):
        image, calibration = make_offset_scene(
            needle_deg=340.0, center=(350.0, 350.0), size=700)
        result = dr.estimate_frame(image, calibration, [50, 50, 600, 600])
        self.assertEqual(result["frame_status"], "ok", result["reason"])
        self.assertEqual(result["roi"], [50, 50, 600, 600])
        self.assertAlmostEqual(result["candidate_value"], 20.0, delta=2.0)
        center = result["diagnostics"]["ring_center"]
        self.assertTrue(330.0 < center[0] < 370.0 and 330.0 < center[1] < 370.0)

    def test_axis_swap_near_circle_keeps_angle(self):
        # Swapping the semiaxis labels and rotating the ellipse representation
        # by 90 degrees is the same SPD matrix; it must not rotate the reading.
        axes = (359.3536376953125, 364.356689453125)
        angle = 120.83535766601562
        first = dr._ellipse_matrix(axes, angle)[0]
        second = dr._ellipse_matrix((axes[1], axes[0]), angle + 90.0)[0]
        np.testing.assert_allclose(first, second, atol=1e-12)
        image, calibration = make_offset_scene(needle_deg=340.0)
        swapped = dict(calibration)
        swapped["matrix"] = dr._ellipse_matrix(
            calibration["ring_axes"], 90.0)[0]
        base = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
        other = dr.estimate_frame(image, swapped, [0, 0, 600, 600])
        self.assertEqual(base["frame_status"], "ok", base["reason"])
        self.assertEqual(other["frame_status"], "ok", other["reason"])
        self.assertAlmostEqual(base["angle_deg"], other["angle_deg"], delta=0.5)

    def test_needle_overlay_direction_uses_inverse_matrix(self):
        matrix = _matrix((400.0, 200.0), 0.0)
        direction = dr._needle_direction_px(matrix, 30.0)
        normalised = matrix @ direction
        expected = _unit_deg(30.0)
        np.testing.assert_allclose(
            normalised / np.linalg.norm(normalised), expected, atol=1e-12)
        self.assertGreater(np.linalg.norm(direction - expected), 0.1)
        result = {
            "frame_status": "ok", "status": "low_conf", "reason": "multi_frame_required",
            "roll_deg": 0.0, "candidate_value": 20.0, "roi": [0, 0, 600, 600],
            "diagnostics": {
                "axis_2_px": [300.0, 300.0], "ring_axes": [400.0, 200.0],
                "ring_angle_deg": 0.0, "needle_raw_angle_deg": 30.0,
                "needle_direction_px": direction.tolist(),
                "needle_draw_length_px": 100.0,
            },
        }
        canvas = dr.draw_debug_overlay(
            np.full((600, 600, 3), 255, dtype=np.uint8), result)
        mapped = (int(round(300 + 80 * direction[0])),
                  int(round(300 + 80 * direction[1])))
        naive = (int(round(300 + 80 * expected[0])),
                 int(round(300 + 80 * expected[1])))
        self.assertTrue(np.any(canvas[mapped[1] - 2:mapped[1] + 3,
                                       mapped[0] - 2:mapped[0] + 3, 2] > 150))
        self.assertFalse(np.all(canvas[naive[1], naive[0]] == np.array([0, 0, 255])))


def _red_hue_wrap_band(blue_arc, red_sectors, radius=170.0,
                       center=(300.0, 300.0), size=600, blur_sigma=0.6):
    """Ring band painted per pixel from the polar angle.

    Red pixels alternate between BGR ``(0, 0, 255)`` and ``(10, 0, 255)``,
    whose quantised OpenCV hues are 0 and 179.  This is a portable equivalent
    of the real 1.75x-upscaled, blurred red annulus where neighbouring red
    pixels land on both sides of the 179/0 hue wrap.  ``blue_arc`` and every
    ``red_sectors`` entry are ``(start, end)`` degree pairs; a pair may wrap
    past 0.  No pixel in these scenes is actually blue unless ``blue_arc`` is
    given.
    """
    calibration = make_calibration(center=center,
                                   axes=(2.0 * radius, 2.0 * radius))
    image = np.full((int(size), int(size), 3), 255, dtype=np.uint8)
    yy, xx = np.indices((int(size), int(size)))
    dx = xx - float(center[0])
    dy = yy - float(center[1])
    radius_px = np.hypot(dx, dy)
    angle = np.degrees(np.arctan2(dy, dx)) % 360.0
    band = (radius_px >= 0.86 * radius) & (radius_px <= 0.98 * radius)

    def sector(arc):
        start, end = float(arc[0]), float(arc[1])
        if start <= end:
            return (angle >= start) & (angle < end)
        return (angle >= start) | (angle < end)

    blue = np.zeros_like(band)
    if blue_arc is not None:
        blue = band & sector(blue_arc)
    red = np.zeros_like(band)
    for arc in red_sectors:
        red |= sector(arc)
    red &= band & ~blue
    image[red] = (0, 0, 255)
    image[red & (((yy + xx) % 2) == 1)] = (10, 0, 255)
    image[blue] = (255, 0, 0)
    image = cv2.GaussianBlur(image, (0, 0), blur_sigma)
    ring = {"matrix": calibration["matrix"],
            "center": calibration["ring_center"]}
    return image, calibration, ring


def _legacy_hue_remap_blue(image_bgr, calibration):
    """Pre-fix sampling order retained only as a negative control.

    The old reader converted the whole frame to HSV first and then bilinearly
    interpolated the hue plane, which averages the circular 179/0 red hues
    into a spurious ~90 "blue".  This mirrors that bug on purpose and is not
    production code.
    """
    inverse = np.linalg.inv(np.asarray(calibration["matrix"], dtype=float))
    center = np.asarray(calibration["ring_center"], dtype=float)
    samples = dr.BLUE_ANGLE_SAMPLES
    angles = np.linspace(0.0, 360.0, samples, endpoint=False)
    unit = np.stack([np.cos(np.radians(angles)),
                     np.sin(np.radians(angles))], axis=1) * dr.BLUE_RHO
    points = center + unit @ inverse.T
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    map_x = points[:, 0].astype(np.float32).reshape(1, -1)
    map_y = points[:, 1].astype(np.float32).reshape(1, -1)

    def sample(plane):
        return cv2.remap(plane, map_x, map_y, cv2.INTER_LINEAR).ravel()

    hue, sat, val = [sample(hsv[:, :, index]) for index in range(3)]
    return ((hue >= dr.BLUE_HUE_MIN) & (hue <= dr.BLUE_HUE_MAX)
            & (sat >= dr.BLUE_SAT_MIN) & (val >= dr.BLUE_VAL_MIN))


def _legacy_blue_verdict(mask):
    step_deg = 360.0 / dr.BLUE_ANGLE_SAMPLES
    closed = dr._circular_fill_gaps(
        mask.copy(), int(round(dr.BLUE_GAP_TOL_DEG / step_deg)))
    start, length = dr._longest_circular_run(closed)
    angles = np.linspace(0.0, 360.0, dr.BLUE_ANGLE_SAMPLES, endpoint=False)
    end = (start + length - 1) % dr.BLUE_ANGLE_SAMPLES
    midpoint = (dr._short_arc_midpoint(angles[start], angles[end])
                if length else None)
    return mask.mean(), length * step_deg, midpoint


def _circular_offset(first_deg, second_deg):
    return abs((float(first_deg) - float(second_deg) + 180.0) % 360.0 - 180.0)


class BlueSamplingOrderRegressionTests(unittest.TestCase):
    """Hue is circular: resample BGR before converting to HSV."""

    def test_red_hue_179_0_texture_never_becomes_blue(self):
        image, calibration, ring = _red_hue_wrap_band(
            blue_arc=None, red_sectors=((180.0, 360.0),))
        result = dr.sample_blue(image, ring)
        self.assertFalse(result["ok"])
        self.assertEqual(result["status"], "low_conf")
        self.assertEqual(result["reason"], "blue_missing")
        # Negative control: the old hue-plane interpolation accepts half the
        # ring as a fake ~180 degree blue segment even though no pixel is blue.
        legacy_coverage, legacy_run, _ = _legacy_blue_verdict(
            _legacy_hue_remap_blue(image, calibration))
        self.assertGreater(legacy_coverage, 0.15)
        self.assertGreaterEqual(legacy_run, dr.BLUE_MIN_RUN_DEG)

    def test_blue_run_crossing_zero_survives_red_wrap_texture(self):
        # Real blue 345..25 (crossing 0, midpoint 5, run 40) plus a
        # 100..280 red hue-wrap sector that the old order misreads as blue.
        image, calibration, ring = _red_hue_wrap_band(
            blue_arc=(345.0, 25.0), red_sectors=((100.0, 280.0),))
        result = dr.sample_blue(image, ring)
        self.assertTrue(result["ok"])
        self.assertEqual(set(result), {"ok", "boundary_angles",
                                       "boundary_points", "midpoint_deg",
                                       "run_deg", "blue_coverage"})
        self.assertAlmostEqual(result["midpoint_deg"], 5.0, delta=1.0)
        self.assertAlmostEqual(result["run_deg"], 40.0, delta=2.0)
        self.assertAlmostEqual(result["blue_coverage"], 40.0 / 360.0, delta=0.02)
        start, end = result["boundary_angles"]
        self.assertLess(_circular_offset(start, 345.0), 1.5)
        self.assertLess(_circular_offset(end, 25.0), 1.5)
        # Negative control: the old order latches onto the 100..280 red
        # sector (about 180 degrees, midpoint near 190) instead of the real
        # 40 degree blue run.
        legacy_coverage, legacy_run, legacy_midpoint = _legacy_blue_verdict(
            _legacy_hue_remap_blue(image, calibration))
        self.assertGreater(legacy_coverage, 0.25)
        self.assertGreater(legacy_run, 4.0 * result["run_deg"])
        self.assertGreater(_circular_offset(legacy_midpoint, 5.0), 90.0)


class ConfigAndCliTests(unittest.TestCase):
    def test_roi_and_calibration_config_errors(self):
        _, calibration = make_scene()
        with self.assertRaises(dr.ReadingConfigError):
            dr.estimate_frame(np.zeros((100, 100, 3), dtype=np.uint8),
                              calibration, [0, 0, 101, 10])
        with self.assertRaises(dr.ReadingConfigError):
            dr.load_calibration("missing.json", allow_candidate=False)
        with self.assertRaises(dr.ReadingConfigError):
            dr.load_calibration("missing.json", allow_candidate=True)

    def test_cli_success_and_normal_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image, calibration = make_scene(needle_deg=340.0)
            image_path = tmp / "frame.png"
            cv2.imwrite(str(image_path), image)
            calib_path = tmp / "candidate.json"
            calib_sha = write_candidate(calib_path, calibration)
            script = Path(__file__).resolve().parents[1] / "scripts" / "dial_reading.py"
            debug = tmp / "debug"
            completed = subprocess.run(
                [sys.executable, "-B", str(script), str(image_path),
                 "--calib", str(calib_path), "--allow-candidate",
                 "--roi", "0", "0", "600", "600",
                 "--debug-dir", str(debug), "--calib-sha256", calib_sha],
                cwd=str(tmp), capture_output=True, text=True, timeout=60)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            payload = json.loads(completed.stdout)
            self.assertEqual(payload["frame_status"], "ok")
            self.assertTrue((debug / "debug_overlay.png").is_file())
            self.assertTrue((debug / "frame_result.json").is_file())

            blank = np.full((600, 600, 3), 255, dtype=np.uint8)
            blank_path = tmp / "blank.png"
            cv2.imwrite(str(blank_path), blank)
            rejected = subprocess.run(
                [sys.executable, "-B", str(script), str(blank_path),
                 "--calib", str(calib_path), "--allow-candidate",
                 "--roi", "0", "0", "600", "600",
                 "--debug-dir", str(tmp / "blank-debug"),
                 "--calib-sha256", calib_sha],
                cwd=str(tmp), capture_output=True, text=True, timeout=60)
            self.assertEqual(rejected.returncode, 0, rejected.stderr)
            self.assertEqual(json.loads(rejected.stdout)["frame_status"], "no_dial")

    def test_cli_config_errors_are_nonzero(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image, calibration = make_scene()
            image_path = tmp / "frame.png"
            cv2.imwrite(str(image_path), image)
            calib_path = tmp / "candidate.json"
            calib_sha = write_candidate(calib_path, calibration)
            script = Path(__file__).resolve().parents[1] / "scripts" / "dial_reading.py"
            cases = (
                [str(image_path), "--calib", str(calib_path),
                 "--roi", "0", "0", "600", "600", "--debug-dir", str(tmp / "a")],
                [str(image_path), "--calib", str(calib_path), "--allow-candidate",
                 "--roi", "0", "0", "9999", "600", "--debug-dir", str(tmp / "b")],
                [str(image_path), "--calib", str(calib_path), "--allow-candidate",
                 "--roi", "0", "0", "600", "600", "--debug-dir", str(tmp / "c"),
                 "--calib-sha256", "0" * 64],
            )
            for args in cases:
                with self.subTest(args=args):
                    completed = subprocess.run(
                        [sys.executable, "-B", str(script)] + args,
                        cwd=str(tmp), capture_output=True, text=True, timeout=60)
                    self.assertNotEqual(completed.returncode, 0)

    def test_debug_output_failure_does_not_leave_final_files(self):
        image, calibration = make_scene()
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            image_path = tmp / "frame.png"
            cv2.imwrite(str(image_path), image)
            output = tmp / "debug"
            result = dr.estimate_frame(image, calibration, [0, 0, 600, 600])
            with mock.patch.object(dr.cv2, "imwrite", return_value=False):
                with self.assertRaises(dr.ReadingConfigError):
                    dr._write_debug(image, result, output)
            self.assertFalse((output / "debug_overlay.png").exists())
            self.assertFalse((output / "frame_result.json").exists())

            output2 = tmp / "debug2"
            output2.mkdir()
            (output2 / "old.txt").write_text("old", encoding="utf-8")
            with self.assertRaises(dr.ReadingConfigError):
                dr._write_debug(image, result, output2)
            self.assertEqual((output2 / "old.txt").read_text(encoding="utf-8"), "old")
            self.assertEqual(len(list(output2.iterdir())), 1)


class ReferenceDataTests(unittest.TestCase):
    @unittest.skipUnless(REFERENCE_IMAGE.is_file() and CANDIDATE_PATH.is_file(),
                         "reference photo or A08 candidate is absent")
    def test_reference_image_is_out_of_range(self):
        payload = REFERENCE_IMAGE.read_bytes()
        self.assertEqual(hashlib.sha256(payload).hexdigest(), REFERENCE_SHA256)
        calibration = dr.load_calibration(str(CANDIDATE_PATH), True)
        image = cv2.imread(str(REFERENCE_IMAGE), cv2.IMREAD_COLOR)
        self.assertIsNotNone(image)
        result = dr.estimate_frame(image, calibration, REFERENCE_ROI)
        # The reference humidity needle points into the blank area: the reader
        # must detect it, report out_of_range and return no usable reading.
        self.assertEqual(result["frame_status"], "out_of_range", result["reason"])
        self.assertEqual(result["status"], "out_of_range")
        self.assertIsNone(result["candidate_value"])
        self.assertIsNone(result["value"])
        self.assertIsNone(result["spread_deg"])
        self.assertLess(result["quality"]["axis_roll_delta_deg"], 0.5)
        self.assertGreaterEqual(result["quality"]["needle_coverage"], 0.7)
        self.assertGreater(
            abs(result["diagnostics"]["reference_axis_mid_deg"]
                - result["diagnostics"]["reference_ring_center_mid_deg"]), 0.4)
        self.assertAlmostEqual(
            result["diagnostics"]["needle_raw_angle_deg"], 272.7, delta=3.0)

    @unittest.skipUnless(
        LOWRES_FIRST.is_file() and LOWRES_LAST.is_file() and CANDIDATE_PATH.is_file(),
        "original 200-frame low-resolution set or A08 candidate is absent")
    def test_low_resolution_representatives_keep_the_contract(self):
        calibration = dr.load_calibration(str(CANDIDATE_PATH), True)
        for path in (LOWRES_FIRST, LOWRES_LAST):
            with self.subTest(path=path.name):
                image = cv2.imread(str(path), cv2.IMREAD_COLOR)
                self.assertIsNotNone(image)
                self.assertEqual(image.shape, (720, 1280, 3))
                result = dr.estimate_frame(image, calibration, LOWRES_ROI)
                self.assertIn(result["frame_status"], dr.STATUS_VALUES)
                self.assertIsNone(result["value"])
                self.assertIsNone(result["spread_deg"])
                self.assertEqual(result["roi"], LOWRES_ROI)
                if result["frame_status"] == "ok":
                    self.assertEqual(result["reason"], "multi_frame_required")
                    self.assertIsInstance(result["candidate_value"], float)
                else:
                    self.assertIsNone(result["candidate_value"])
                    self.assertIsInstance(result["reason"], str)
                    self.assertTrue(result["reason"])


class ImportTests(unittest.TestCase):
    def test_import_has_no_forbidden_modules_threads_or_files(self):
        scripts = Path(__file__).resolve().parents[1] / "scripts"
        code = (
            "import json, sys, threading\n"
            "sys.path.insert(0, {path!r})\n"
            "import dial_reading\n"
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


def _wrap180(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


def _table_angle_for_value(table_deg, table_val, target):
    values = [float(v) for v in table_val]
    angles = [float(v) for v in table_deg]
    if target < values[0] or target > values[-1]:
        raise ValueError("target value outside the synthetic table")
    for index in range(len(values) - 1):
        low, high = values[index], values[index + 1]
        if low <= target <= high:
            fraction = (target - low) / (high - low)
            return angles[index] + fraction * (angles[index + 1] - angles[index])
    return angles[-1]


def _prepare_output_root(path):
    root = Path(path)
    if root.exists():
        if not root.is_dir() or any(root.iterdir()):
            raise SystemExit("output directory must not exist or be empty: {}".format(root))
    root.mkdir(parents=True, exist_ok=True)
    return root


def _line_kernel(angle_deg, length=28, thickness=3):
    size = int(length) + 4
    kernel = np.zeros((size, size), dtype=np.uint8)
    center = size // 2
    direction = _unit_deg(angle_deg) * (length / 2.0)
    start = (int(round(center - direction[0])), int(round(center - direction[1])))
    end = (int(round(center + direction[0])), int(round(center + direction[1])))
    cv2.line(kernel, start, end, 1, thickness)
    return kernel


def _mask_overlay(image, mask):
    overlay = image.copy()
    overlay[mask > 0] = (0, 0, 255)
    return overlay


def build_erased_reference_base(image, axis, ring_semi_px):
    """Erase the original humidity needle from a copy of the reference photo.

    The corridor is explicit: a straight band from 0.20R to 0.90R around the
    supplied axis, opened with a directional kernel so only the long straight
    shaft survives, then widened and replaced by a median background.  Text
    that the physical needle occludes cannot be recovered exactly; that region
    is inpainted from its surroundings and is a documented limitation.
    """
    axis = np.asarray(axis, dtype=float)
    height, width = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    radius = float(ring_semi_px)

    def corridor(angle_deg):
        canvas = np.zeros((height, width), dtype=np.uint8)
        direction = _unit_deg(angle_deg)
        start = (axis + 0.20 * radius * direction).round().astype(int)
        end = (axis + 0.90 * radius * direction).round().astype(int)
        cv2.line(canvas, tuple(start), tuple(end), 255, 29)
        return canvas

    # The reference needle points roughly upward.  Only that coarse prior is
    # used to build the first corridor; the exact angle is measured from the
    # dark pixels below instead of being taken from the reader or a constant.
    initial_angle = 270.0
    corridor_mask = corridor(initial_angle)
    dark = ((gray < 175) & (corridor_mask > 0)).astype(np.uint8) * 255
    core = cv2.morphologyEx(dark, cv2.MORPH_OPEN,
                            _line_kernel(initial_angle, length=21, thickness=7))
    points = np.column_stack(np.nonzero(core)[::-1]).astype(np.float32)
    if len(points) < 50:
        raise RuntimeError("cannot locate the reference humidity needle shaft")
    vx, vy = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01).ravel()[:2]
    angle = math.degrees(math.atan2(float(vy), float(vx))) % 360.0
    if float(np.dot(points.mean(axis=0) - axis, _unit_deg(angle))) < 0:
        angle = (angle + 180.0) % 360.0

    corridor_mask = corridor(angle)
    dark = ((gray < 175) & (corridor_mask > 0)).astype(np.uint8) * 255
    core = cv2.morphologyEx(dark, cv2.MORPH_OPEN, _line_kernel(angle))
    mask = cv2.dilate(core, np.ones((13, 13), np.uint8))
    mask = np.maximum(mask, ((gray < 140) & (corridor_mask > 0)).astype(np.uint8) * 255)
    mask = cv2.dilate(mask, np.ones((3, 3), np.uint8))
    yy, xx = np.ogrid[:height, :width]
    mask[((xx - axis[0]) ** 2 + (yy - axis[1]) ** 2) < (0.20 * radius) ** 2] = 0

    background = cv2.medianBlur(image, 31)
    base = image.copy()
    base[mask > 0] = background[mask > 0]
    erased = cv2.cvtColor(base, cv2.COLOR_BGR2GRAY)[mask > 0]
    stats = {
        "mask_pixels": int((mask > 0).sum()),
        "residual_min_gray_in_mask": int(erased.min()) if len(erased) else None,
        "residual_fraction_below_135": (float((erased < 135).mean())
                                        if len(erased) else None),
        "measured_original_needle_angle_deg": float(angle),
        "corridor_inner_outer_rho": [0.20, 0.90],
        "corridor_half_width_px": 10.5,
    }
    return {"base": base, "mask": mask, "stats": stats}


def _warp_scene(base, center, roll_deg, scale):
    height, width = base.shape[:2]
    matrix = cv2.getRotationMatrix2D(
        (float(center[0]), float(center[1])), -float(roll_deg), float(scale))
    corners = np.array([[0.0, 0.0], [width, 0.0], [0.0, height],
                        [width, height]], dtype=float)
    transformed = cv2.transform(corners.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    min_x, min_y = transformed.min(axis=0)
    max_x, max_y = transformed.max(axis=0)
    matrix[0, 2] -= min_x
    matrix[1, 2] -= min_y
    out_size = (int(math.ceil(max_x - min_x)), int(math.ceil(max_y - min_y)))
    warped = cv2.warpAffine(base, matrix, out_size, flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT,
                            borderValue=(225, 225, 225))
    return warped, matrix


def _apply_affine(matrix, point):
    return (np.asarray(matrix, dtype=float)
            @ np.array([float(point[0]), float(point[1]), 1.0]))[:2]


def _warp_roi(matrix, roi, width, height):
    x, y, w, h = [float(v) for v in roi]
    corners = np.array([[x, y], [x + w, y], [x, y + h], [x + w, y + h]], dtype=float)
    mapped = cv2.transform(corners.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    x0 = max(0, int(math.floor(mapped[:, 0].min())))
    y0 = max(0, int(math.floor(mapped[:, 1].min())))
    x1 = min(int(width), int(math.ceil(mapped[:, 0].max())))
    y1 = min(int(height), int(math.ceil(mapped[:, 1].max())))
    if x1 <= x0 or y1 <= y0:
        raise RuntimeError("warped ROI is empty")
    return [x0, y0, x1 - x0, y1 - y0]


def _draw_synthetic_needle(image, axis, angle_deg, length_px, thickness_px, hub_px):
    direction = _unit_deg(angle_deg)
    center = (int(round(axis[0])), int(round(axis[1])))
    end = (int(round(axis[0] + length_px * direction[0])),
           int(round(axis[1] + length_px * direction[1])))
    cv2.line(image, center, end, (0, 0, 0), int(thickness_px), cv2.LINE_AA)
    cv2.circle(image, center, int(hub_px), (0, 0, 0), -1)
    return image


def run_reference_cli(output_root):
    root = _prepare_output_root(output_root)
    reference_dir = root / "reference"
    reference_dir.mkdir()
    script = Path(__file__).resolve().parents[1] / "scripts" / "dial_reading.py"
    command = [
        sys.executable, "-B", str(script), str(REFERENCE_IMAGE),
        "--calib", str(CANDIDATE_PATH), "--allow-candidate",
        "--roi", "430", "870", "420", "390",
        "--debug-dir", str(reference_dir),
    ]
    completed = subprocess.run(command, cwd=str(REPO_ROOT), capture_output=True,
                               text=True, timeout=180)
    if completed.returncode != 0:
        raise RuntimeError("reference CLI failed: {}".format(completed.stderr))
    payload = json.loads(completed.stdout)
    if payload.get("frame_status") != "out_of_range":
        raise RuntimeError("reference CLI did not report out_of_range: {!r}".format(
            payload.get("frame_status")))
    return payload


def run_real_base_synthetic_eval(output_dir):
    """Optional offline evaluation on a needle-erased copy of the real photo.

    This is a synthetic-geometry evaluation, not accuracy acceptance: needles
    are redrawn at angles derived from the candidate table, the base is one
    resampled real photo, and upscaling adds no real resolution.
    """
    if not REFERENCE_IMAGE.is_file() or not CANDIDATE_PATH.is_file():
        raise RuntimeError("reference photo or A08 candidate is absent")
    root = _prepare_output_root(output_dir)
    reference_bytes = REFERENCE_IMAGE.read_bytes()
    if hashlib.sha256(reference_bytes).hexdigest() != REFERENCE_SHA256:
        raise RuntimeError("reference image hash mismatch")
    if hashlib.sha256(CANDIDATE_PATH.read_bytes()).hexdigest() != CANDIDATE_SHA256:
        raise RuntimeError("candidate calibration hash mismatch")
    candidate = json.loads(CANDIDATE_PATH.read_text(encoding="utf-8"))
    image = cv2.imread(str(REFERENCE_IMAGE), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot decode the reference image")
    calibration = dr.load_calibration(str(CANDIDATE_PATH), True)
    table_deg = candidate["table_deg"]
    table_val = candidate["table_val"]
    ring_center = np.asarray(candidate["ring"]["center_px"], dtype=float)
    axis = np.asarray(candidate["humidity_axis_px"], dtype=float)
    ring_semi = 0.5 * (float(candidate["ring"]["semi_major_px"])
                       + float(candidate["ring"]["semi_minor_px"]))
    humidity_diameter = 2.0 * float(candidate["ring"]["semi_major_px"])

    prepared = build_erased_reference_base(image, axis, ring_semi)
    cv2.imwrite(str(root / "reference_needle_mask.png"),
                _mask_overlay(image, prepared["mask"]))
    cv2.imwrite(str(root / "reference_base_erased.png"), prepared["base"])
    crop = (slice(870, 1250), slice(470, 830))
    comparison = np.hstack([image[crop], prepared["base"][crop]])
    cv2.imwrite(str(root / "reference_needle_erased_compare.png"),
                cv2.resize(comparison, None, fx=1.5, fy=1.5,
                           interpolation=cv2.INTER_NEAREST))

    values = [float(v) for v in range(0, 101, 10)] + [15.0]
    rolls = (-6.0, 0.0, 6.0)
    scales = (0.5, 1.0, 1.75)
    blurs = (0.0, 1.2)
    cases = []
    rows = []
    saved = 0
    failures_saved = 0
    wrong_saved = 0
    for roll in rolls:
        for scale in scales:
            warped, matrix = _warp_scene(prepared["base"], ring_center, roll, scale)
            height, width = warped.shape[:2]
            axis_out = _apply_affine(matrix, axis)
            roi = _warp_roi(matrix, REFERENCE_ROI, width, height)
            needle_length = 0.68 * ring_semi * scale
            thickness = max(3, int(round(9.0 * scale)))
            hub = max(3, int(round(5.0 * scale)))
            for target in values:
                base_angle = _table_angle_for_value(table_deg, table_val, target)
                for blur in blurs:
                    frame = warped.copy()
                    _draw_synthetic_needle(frame, axis_out, (base_angle + roll) % 360.0,
                                           needle_length, thickness, hub)
                    if blur > 0:
                        frame = cv2.GaussianBlur(frame, (0, 0), blur)
                    case = {
                        "case_id": "roll{:+05.1f}_scale{:.2f}_val{:05.1f}_blur{:.1f}".format(
                            roll, scale, target, blur),
                        "roll_deg": roll,
                        "scale": scale,
                        "blur_sigma_px": blur,
                        "humidity_ring_diameter_px": humidity_diameter * scale,
                        "image_width": width,
                        "image_height": height,
                        "roi": list(roi),
                        "target_value": target,
                        "synthetic_needle_angle_deg": base_angle,
                        "synthetic_needle_angle_in_frame_deg": (base_angle + roll) % 360.0,
                    }
                    try:
                        result = dr.estimate_frame(frame, calibration, roi)
                    except Exception as error:  # noqa: BLE001 - recorded, not hidden
                        case.update({
                            "frame_status": "error", "status": "error",
                            "reason": "exception: {}".format(error),
                            "candidate_value": None, "value": None,
                            "angle_deg": None, "roll_deg": None,
                            "value_error": None, "angle_error_deg": None,
                            "roll_error_deg": None, "value_pass": False,
                            "roll_pass": False,
                        })
                    else:
                        value = result.get("candidate_value")
                        angle = result.get("angle_deg")
                        measured_roll = result.get("roll_deg")
                        value_error = (None if value is None
                                       else float(value) - target)
                        angle_error = (None if angle is None
                                       else _wrap180(float(angle) - base_angle))
                        roll_error = (None if measured_roll is None
                                      else _wrap180(float(measured_roll) - roll))
                        value_pass = (result.get("frame_status") == "ok"
                                      and value_error is not None
                                      and abs(value_error) <= 2.0)
                        roll_pass = (result.get("frame_status") == "ok"
                                     and roll_error is not None
                                     and abs(roll_error) <= 1.0)
                        case.update({
                            "frame_status": result.get("frame_status"),
                            "status": result.get("status"),
                            "reason": result.get("reason"),
                            "candidate_value": value,
                            "value": result.get("value"),
                            "angle_deg": angle,
                            "roll_measured_deg": measured_roll,
                            "value_error": value_error,
                            "angle_error_deg": angle_error,
                            "roll_error_deg": roll_error,
                            "needle_coverage": result.get("quality", {}).get(
                                "needle_coverage"),
                            "ring_residual_rms_px": result.get("quality", {}).get(
                                "ring_residual_rms_px"),
                            "roll_delta_deg": result.get("quality", {}).get(
                                "axis_roll_delta_deg"),
                            "value_pass": value_pass,
                            "roll_pass": roll_pass,
                        })
                        representative = (
                            blur == 0.0 and target in (0.0, 50.0, 100.0)
                            and ((roll == 0.0 and scale == 1.0)
                                 or (roll == 6.0 and scale == 0.5)
                                 or (roll == -6.0 and scale == 1.75)))
                        if representative or (blur == 1.2 and target == 50.0
                                              and roll == 0.0 and scale == 1.0):
                            overlay = dr.draw_debug_overlay(frame, result)
                            name = "overlay_{}.png".format(case["case_id"])
                            cv2.imwrite(str(root / name), overlay)
                            case["overlay"] = name
                            saved += 1
                        elif result.get("frame_status") != "ok" and failures_saved < 2:
                            overlay = dr.draw_debug_overlay(frame, result)
                            name = "overlay_failure_{}.png".format(case["case_id"])
                            cv2.imwrite(str(root / name), overlay)
                            case["overlay"] = name
                            failures_saved += 1
                        elif (result.get("frame_status") == "ok"
                              and not value_pass and wrong_saved < 2):
                            overlay = dr.draw_debug_overlay(frame, result)
                            name = "overlay_wrong_ok_{}.png".format(case["case_id"])
                            cv2.imwrite(str(root / name), overlay)
                            case["overlay"] = name
                            wrong_saved += 1
                    cases.append(case)
                    rows.append(case)

    out_of_range = None
    warped, matrix = _warp_scene(prepared["base"], ring_center, 0.0, 1.0)
    height, width = warped.shape[:2]
    axis_out = _apply_affine(matrix, axis)
    roi = _warp_roi(matrix, REFERENCE_ROI, width, height)
    frame = warped.copy()
    _draw_synthetic_needle(frame, axis_out, prepared["stats"][
        "measured_original_needle_angle_deg"], 0.68 * ring_semi, 9, 5)
    result = dr.estimate_frame(frame, calibration, roi)
    out_of_range = {
        "case_id": "original_needle_angle_roll+00.0_scale1.00_blur0.0",
        "synthetic_needle_angle_deg": prepared["stats"][
            "measured_original_needle_angle_deg"],
        "frame_status": result["frame_status"],
        "status": result["status"],
        "reason": result["reason"],
        "candidate_value": result["candidate_value"],
        "value": result["value"],
        "angle_deg": result["angle_deg"],
        "expected": "out_of_range (outside the synthetic 0..100 table)",
    }
    overlay = dr.draw_debug_overlay(frame, result)
    cv2.imwrite(str(root / "overlay_out_of_range_original_angle.png"), overlay)
    out_of_range["overlay"] = "overlay_out_of_range_original_angle.png"
    cases.append(out_of_range)

    status_counts = {}
    for case in cases:
        status = case.get("frame_status", "error")
        status_counts[status] = status_counts.get(status, 0) + 1
    accepted = [case for case in cases if case.get("frame_status") == "ok"]
    by_target = {}
    for target in sorted({case.get("target_value") for case in cases
                          if case.get("target_value") is not None}):
        subset = [case for case in cases if case.get("target_value") == target]
        by_target[str(target)] = {
            "cases": len(subset),
            "ok": sum(1 for case in subset if case.get("frame_status") == "ok"),
            "out_of_range": sum(1 for case in subset
                                if case.get("frame_status") == "out_of_range"),
            "value_pass": sum(1 for case in subset if case.get("value_pass")),
            "max_abs_value_error": (max(
                [abs(case["value_error"]) for case in subset
                 if case.get("value_error") is not None] or [None])
                if any(case.get("value_error") is not None for case in subset)
                else None),
        }
    by_condition = {}
    for roll in rolls:
        for scale in scales:
            for blur in blurs:
                subset = [case for case in cases
                          if case.get("target_value") is not None
                          and case["roll_deg"] == roll and case["scale"] == scale
                          and case["blur_sigma_px"] == blur]
                key = "roll{:+05.1f}_scale{:.2f}_blur{:.1f}".format(roll, scale, blur)
                by_condition[key] = {
                    "cases": len(subset),
                    "ok": sum(1 for case in subset if case.get("frame_status") == "ok"),
                    "out_of_range": sum(1 for case in subset
                                        if case.get("frame_status") == "out_of_range"),
                    "value_pass": sum(1 for case in subset if case.get("value_pass")),
                }
    wrong_ok = [case["case_id"] for case in cases
                if case.get("frame_status") == "ok" and not case.get("value_pass")]
    value_errors = [abs(case["value_error"]) for case in accepted
                    if case.get("value_error") is not None]
    roll_errors = [abs(case["roll_error_deg"]) for case in accepted
                   if case.get("roll_error_deg") is not None]
    summary = {
        "kind": "real_base_synthetic_evaluation",
        "synthetic_not_accuracy_acceptance": True,
        "source": {
            "image": str(REFERENCE_IMAGE),
            "image_sha256": REFERENCE_SHA256,
            "image_size": [int(image.shape[1]), int(image.shape[0])],
            "candidate": str(CANDIDATE_PATH),
            "candidate_sha256": CANDIDATE_SHA256,
            "candidate_status": candidate.get("status"),
        },
        "geometry": {
            "reference_image_size_px": [int(image.shape[1]), int(image.shape[0])],
            "reference_humidity_ring_outer_diameter_px": humidity_diameter,
            "whole_dial_outer_diameter": "not measured; not claimed",
            "scale_factors": list(scales),
            "humidity_ring_outer_diameter_by_scale_px": {
                str(scale): humidity_diameter * scale for scale in scales},
        },
        "needle_erasure": prepared["stats"],
        "rotation_deg": list(rolls),
        "blur_sigma_px": list(blurs),
        "angle_source": ("candidate 0/10/.../100 and 15 %RH table angles; "
                         "synthetic convention, not real humidity truth"),
        "totals": {
            "cases": len(cases),
            "status_counts": status_counts,
            "accepted_ok": len(accepted),
            "rejected": len(cases) - len(accepted),
            "value_pass": sum(1 for case in cases if case.get("value_pass")),
            "value_fail": sum(1 for case in cases if case.get("frame_status") == "ok"
                              and not case.get("value_pass")),
            "roll_pass": sum(1 for case in cases if case.get("roll_pass")),
            "max_abs_value_error": max(value_errors) if value_errors else None,
            "mean_abs_value_error": (sum(value_errors) / len(value_errors)
                                     if value_errors else None),
            "max_abs_roll_error_deg": max(roll_errors) if roll_errors else None,
            "by_target": by_target,
            "by_condition": by_condition,
            "wrong_ok_cases": wrong_ok,
            "wrong_ok_conditions": sorted(
                {"scale={:.2f},blur={:.1f}".format(case["scale"], case["blur_sigma_px"])
                 for case in cases
                 if case.get("frame_status") == "ok" and not case.get("value_pass")}),
        },
        "limitations": [
            "Synthetic needles are redrawn on a needle-erased copy of one real photo; "
            "this is not independent real capture and not accuracy acceptance.",
            "The original needle overlapped printed text near its tip; inpainting "
            "cannot recover occluded text exactly.",
            "Upscaling by 1.75x is interpolation, not real optical resolution.",
            "The whole-dial outer diameter was not measured; only the fitted "
            "humidity-ring outer diameter is reported.",
            "Perspective is unchanged from the single reference photo; affine "
            "roll/scale cannot prove real perspective compensation.",
        ],
        "cases": cases,
    }
    (root / "synthetic_eval.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    columns = [
        "case_id", "roll_deg", "scale", "blur_sigma_px",
        "humidity_ring_diameter_px", "image_width", "image_height",
        "roi", "target_value", "synthetic_needle_angle_deg",
        "frame_status", "status", "reason", "candidate_value", "value",
        "angle_deg", "roll_measured_deg", "value_error", "angle_error_deg",
        "roll_error_deg", "needle_coverage", "ring_residual_rms_px",
        "roll_delta_deg", "value_pass", "roll_pass", "overlay",
    ]
    with (root / "synthetic_eval.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return summary


def run_lowres_challenge(output_dir):
    if not (LOWRES_FIRST.is_file() and LOWRES_LAST.is_file()
            and CANDIDATE_PATH.is_file()):
        raise RuntimeError("low-resolution representatives or candidate are absent")
    root = _prepare_output_root(output_dir)
    calibration = dr.load_calibration(str(CANDIDATE_PATH), True)
    records = []
    for path in (LOWRES_FIRST, LOWRES_LAST):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("cannot decode: {}".format(path))
        image_sha = hashlib.sha256(path.read_bytes()).hexdigest()
        result = dr.estimate_frame(image, calibration, LOWRES_ROI)
        overlay = dr.draw_debug_overlay(image, result)
        name = "lowres_{}.png".format(path.stem)
        cv2.imwrite(str(root / name), overlay)
        records.append({
            "image": str(path),
            "image_sha256": image_sha,
            "image_size": [int(image.shape[1]), int(image.shape[0])],
            "roi": LOWRES_ROI,
            "frame_status": result["frame_status"],
            "status": result["status"],
            "reason": result["reason"],
            "candidate_value": result["candidate_value"],
            "value": result["value"],
            "quality": result["quality"],
            "overlay": name,
            "note": ("low-resolution challenge only; no real humidity truth and "
                     "no MAE is computed"),
        })
    summary = {
        "kind": "low_resolution_challenge",
        "note": ("720p frames from the original 200-frame set; explicit ROI; "
                 "visibility/rejection is recorded, never a real accuracy claim"),
        "candidate_sha256": CANDIDATE_SHA256,
        "whole_dial_diameter_px_approx_from_G0": 119,
        "whole_dial_diameter_note": (
            "G0 measured a median whole-dial outer diameter of about 119 px; "
            "this script did not re-measure it and does not claim it"),
        "humidity_ring_outer_diameter_note": (
            "not measured on the 720p frames; the explicit ROI was chosen from "
            "visual inspection and the humidity ring is clearly smaller than "
            "the whole dial"),
        "not_upscaled_for_reading": True,
        "records": records,
    }
    (root / "lowres_challenge.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def _real_eval_main(argv):
    parser = argparse.ArgumentParser(
        description="Optional offline G1b1 real-image evaluation helpers")
    parser.add_argument("--real-eval", action="store_true", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)
    root = _prepare_output_root(args.output_dir)
    report = {}
    reference = run_reference_cli(root)
    report["reference"] = {
        "frame_status": reference.get("frame_status"),
        "status": reference.get("status"),
        "reason": reference.get("reason"),
        "candidate_value": reference.get("candidate_value"),
        "value": reference.get("value"),
        "roll_deg": reference.get("roll_deg"),
        "angle_deg": reference.get("angle_deg"),
        "needle_raw_angle_deg": reference.get("diagnostics", {}).get(
            "needle_raw_angle_deg"),
        "axis_roll_delta_deg": reference.get("quality", {}).get(
            "axis_roll_delta_deg"),
        "debug_overlay": str(root / "reference" / "debug_overlay.png"),
    }
    report["synthetic"] = run_real_base_synthetic_eval(root / "synthetic")
    report["lowres"] = run_lowres_challenge(root / "lowres")
    (root / "eval_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "reference": report["reference"]["frame_status"],
        "synthetic": report["synthetic"]["totals"],
        "lowres": [record["frame_status"] for record in report["lowres"]["records"]],
        "report": str(root / "eval_report.json"),
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    if "--real-eval" in sys.argv:
        sys.exit(_real_eval_main(sys.argv[1:]))
    unittest.main()
