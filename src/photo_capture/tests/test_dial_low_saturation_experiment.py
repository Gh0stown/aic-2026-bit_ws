#!/usr/bin/env python3
"""Tests for the isolated edge-first low-saturation dial experiment.

All synthetic fixtures are generated in memory.  The only external assets are
the already approved candidate calibration, its reference image and the phone
control; those tests skip when the local files are absent.  Nothing here is a
real dial truth, an accuracy result or a trusted reading.
"""

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import dial_low_saturation_experiment as exp

REPO_ROOT = Path(__file__).resolve().parents[3]
CANDIDATE_PATH = REPO_ROOT / '.gauge_calibration_review' / 'r2-final' / 'candidate_calib.json'
CANDIDATE_SHA256 = 'd7dccc3e6d80f24199405dce07a516fd119d4502724efa413983682d2e5df198'
REFERENCE_IMAGE = Path(
    '/home/mowen/.dsh/attachments/v1/objects/be/'
    'be71a511ebfe297c18c86feebc8d99cdefe9030e507a82e65f50d2aed8d36f30')
PHONE_IMAGE = Path(
    '/home/mowen/.dsh/attachments/v1/objects/fe/'
    'fe554e70eab8087ff60d8d449539db38af90c7c79debed5b97d2603b42504eb4')


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


def make_calibration(center=(100.0, 100.0), axes=(84.0, 78.0)):
    center = np.asarray(center, dtype=float)
    matrix = _matrix(axes, 0.0)
    return {
        "calib_version": "synthetic-fixture",
        "calib_sha256": "0" * 64,
        "source_sha256": "0" * 64,
        "table_deg": [100.0 + 20.0 * index for index in range(11)],
        "table_val": np.arange(0.0, 101.0, 10.0),
        "ring_center": center,
        "ring_axes": np.asarray(axes, dtype=float),
        "matrix": matrix,
        "axis": center.copy(),
        "blue_mid": 0.0,
        "blue_boundaries": [center, center],
    }


def make_reference(image, center=(100.0, 100.0), axes=(84.0, 78.0)):
    return exp.angular_signature(image, np.asarray(center), _matrix(axes, 0.0))


def _ellipse_points(center=(100.0, 100.0), axes=(84.0, 78.0), count=120):
    angles = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
    return np.stack([
        float(center[0]) + 0.5 * float(axes[0]) * np.cos(angles),
        float(center[1]) + 0.5 * float(axes[1]) * np.sin(angles),
    ], axis=1)


class RoiTests(unittest.TestCase):
    def test_top_expansion_is_recorded_and_other_edges_unchanged(self):
        record = exp.expanded_roi([630, 220, 75, 65], (720, 1280, 3), 10)
        self.assertEqual(record["declared_roi"], [630, 220, 75, 65])
        self.assertEqual(record["used_roi"], [630, 210, 75, 75])
        self.assertEqual(record["top_expansion_px"], 10)

    def test_invalid_or_outside_roi_is_rejected(self):
        with self.assertRaises(exp.ExperimentConfigError):
            exp.expanded_roi([630, 220, 0, 65], (720, 1280, 3))
        with self.assertRaises(exp.ExperimentConfigError):
            exp.expanded_roi([1200, 220, 200, 65], (720, 1280, 3))


class RingTests(unittest.TestCase):
    def test_clean_synthetic_colour_ring_is_recovered(self):
        image = exp.make_synthetic_scene('color')
        result = exp.detect_edge_ring(image)
        self.assertTrue(result["ok"], result.get("reason"))
        ring = result["ring"]
        self.assertLess(math.hypot(ring["center"][0] - 100.0,
                                   ring["center"][1] - 100.0), 3.0)
        self.assertGreater(max(ring["axes"]), 80.0)
        self.assertGreater(ring["support"]["coverage"], 0.55)
        self.assertGreater(ring["band_contrast"]["contrast"], 0.0)

    def test_too_small_ring_is_not_accepted(self):
        image = exp.make_synthetic_scene('color', axes=(30.0, 28.0))
        result = exp.detect_edge_ring(image)
        self.assertFalse(result["ok"])
        reasons = [item.get("reject") for item in result.get("top_rejected", [])]
        self.assertIn("ring_too_small", reasons)

    def test_gray_ring_without_colour_band_is_rejected(self):
        image = exp.make_synthetic_scene('gray')
        result = exp.detect_edge_ring(image)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "no_colour_band_edge")
        values = [item.get("band_contrast") for item in result.get("all_candidates", [])]
        self.assertTrue(all(value is not None and abs(value) < 5 for value in values))

    def test_blank_image_has_no_ring(self):
        image = exp.make_synthetic_scene('blank')
        result = exp.detect_edge_ring(image)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "no_ring")

    def test_multi_ring_does_not_select_the_second_ring(self):
        image = exp.make_synthetic_scene('color', second_ring=True)
        result = exp.detect_edge_ring(image)
        if result["ok"]:
            center = result["ring"]["center"]
            distance_to_second = math.hypot(center[0] - 60.0, center[1] - 140.0)
            self.assertGreater(distance_to_second, 10.0)
        else:
            self.assertIn(result["reason"], ("ambiguous_ring", "no_ring",
                                             "no_colour_band_edge"))

    def test_blurred_ring_is_detected_or_rejected_with_evidence(self):
        image = exp.make_synthetic_scene('color', blur_sigma=1.2)
        result = exp.detect_edge_ring(image)
        if result["ok"]:
            center = result["ring"]["center"]
            self.assertLess(math.hypot(center[0] - 100.0, center[1] - 100.0), 4.0)
        else:
            self.assertIn(result["reason"], ("ambiguous_ring", "weak_edge_geometry",
                                             "no_colour_band_edge"))

    def test_noise_on_gray_does_not_create_a_colour_ring(self):
        image = exp.make_synthetic_scene('gray', blur_sigma=1.2, noise_sigma=4.0)
        result = exp.detect_edge_ring(image)
        self.assertFalse(result["ok"])
        self.assertIn(result["reason"], ("no_colour_band_edge", "no_ring"))

    def test_off_dial_roi_produces_no_candidate(self):
        image = exp.make_synthetic_scene('color')
        calibration = make_calibration()
        reference = make_reference(image)
        result = exp.estimate_experiment_frame(
            image, calibration, [0, 0, 60, 60], reference,
            stage="negative", top_expansion_px=0)
        self.assertIsNone(result["value"])
        self.assertNotEqual(result["frame_status"], "ok")
        self.assertIsNone(result["diagnostic_candidate"])


class RollTests(unittest.TestCase):
    @staticmethod
    def _signature(bumps):
        bins = 360
        values = np.zeros((bins, 2), dtype=float)
        angles = np.arange(bins)
        for centre, width, amplitude in bumps:
            delta = ((angles - centre + 180) % 360) - 180
            values[:, 0] += amplitude * np.exp(-0.5 * (delta / width) ** 2)
            values[:, 1] += amplitude * np.exp(-0.5 * (delta / (width * 1.5)) ** 2)
        values -= values.mean(axis=0)
        amplitude = float(np.sqrt(np.mean(values ** 2)))
        values /= amplitude
        return {
            "ok": True,
            "signature": values,
            "valid_bins": np.ones(bins, dtype=bool),
            "amplitude": 0.2,
            "valid_frac": 1.0,
            "reason": None,
        }

    def test_known_circular_shift_is_recovered(self):
        reference = self._signature([(30.0, 12.0, 1.0), (120.0, 18.0, 0.7),
                                     (250.0, 25.0, 1.2)])
        current = dict(reference)
        current["signature"] = np.roll(reference["signature"], 25, axis=0)
        result = exp.estimate_roll(reference, current, np.eye(2))
        self.assertTrue(result["ok"], result.get("reason"))
        self.assertLess(abs(result["roll_deg"] - 25.0), 2.0)
        self.assertGreater(result["best_corr"], 0.8)

    def test_flat_signature_is_rejected(self):
        reference = self._signature([(30.0, 12.0, 1.0)])
        current = dict(reference)
        current["amplitude"] = 0.001
        result = exp.estimate_roll(reference, current, np.eye(2))
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "roll_signal_weak")

    def test_two_equal_peaks_are_ambiguous(self):
        reference = self._signature([(0.0, 10.0, 1.0), (180.0, 10.0, 1.0)])
        current = dict(reference)
        result = exp.estimate_roll(reference, current, np.eye(2))
        self.assertFalse(result["ok"])
        self.assertIn(result["reason"], ("roll_ambiguous", "roll_low_snr"))


class AxisTests(unittest.TestCase):
    def test_axis_offset_is_reported_not_collapsed(self):
        center = np.array([100.0, 100.0])
        ring = {
            "center": center.tolist(),
            "axes": [84.0, 78.0],
            "angle_deg": 0.0,
            "inlier_points": _ellipse_points(count=160),
        }
        hub = {"center": [104.0, 101.5], "score": 10.0}
        result = exp.assess_axis(ring, hub, make_calibration())
        self.assertTrue(result["ok"])
        self.assertAlmostEqual(result["observed_axis_px"][0], 104.0, places=3)
        self.assertNotAlmostEqual(result["observed_axis_px"][0],
                                  result["ring_center_px"][0], places=3)
        self.assertGreater(result["ring_axis_offset_frac"], 0.0)
        self.assertLess(result["ring_axis_offset_frac"], exp.AXIS_OFFSET_MAX_FRAC)

    def test_missing_hub_is_marked_as_fallback(self):
        ring = {
            "center": [100.0, 100.0],
            "axes": [84.0, 78.0],
            "angle_deg": 0.0,
            "inlier_points": _ellipse_points(),
        }
        result = exp.assess_axis(ring, None, make_calibration())
        self.assertTrue(result["ok"])
        self.assertIsNone(result["hub_axis_px"])
        self.assertEqual(result["axis_source"], "ring centre fallback")


class HubEvidenceTests(unittest.TestCase):
    """Independent synthetic caps/needles, not calls into the detector."""

    @staticmethod
    def _cap_with_needle(direction_deg, needle_length=14, cap_radius=5, size=120):
        image = np.full((size, size, 3), 230, dtype=np.uint8)
        hub = (60, 60)
        cv2.circle(image, hub, cap_radius, (20, 20, 20), -1)
        angle = math.radians(float(direction_deg))
        end = (int(round(60.0 + needle_length * math.cos(angle))),
               int(round(60.0 + needle_length * math.sin(angle))))
        cv2.line(image, hub, end, (20, 20, 20), 2)
        return image

    def test_dt_plateau_center_is_not_pulled_by_needle(self):
        offsets = []
        for direction in (0, 45, 90, 135, 180, 225, 270, 315):
            for length in (8, 14):
                crop = self._cap_with_needle(direction, length)
                candidates = exp._dark_hub_candidates(crop)
                self.assertTrue(candidates)
                best = max(candidates, key=lambda item: item["score"])
                self.assertTrue(best["usable"], best.get("reject"))
                self.assertLess(math.hypot(best["center"][0] - 60.0,
                                           best["center"][1] - 60.0), 0.75)
                self.assertGreater(best["threshold_support"], 1)
                self.assertGreater(best["centroid_offset_px"], 0.5)
                offsets.append(best["centroid_offset_px"])
        # The construction truth is (60,60); only the blob centroid moves.
        self.assertGreater(max(offsets), 2.0)

    def test_uniform_width_needle_is_not_a_cap(self):
        image = np.full((120, 120, 3), 230, dtype=np.uint8)
        cv2.line(image, (45, 60), (75, 60), (20, 20, 20), 4)
        candidates = exp._dark_hub_candidates(image)
        self.assertTrue(candidates)
        self.assertTrue(all(not item["usable"] for item in candidates))
        self.assertTrue(any(item["reject"] == "hub_needle_thickness"
                            for item in candidates))
        hub = exp.select_hub(image, [60.0, 60.0])
        self.assertIsNotNone(hub)
        self.assertFalse(hub["usable"])

    def test_two_competing_caps_are_rejected(self):
        image = np.full((120, 120, 3), 230, dtype=np.uint8)
        cv2.circle(image, (54, 60), 4, (20, 20, 20), -1)
        cv2.circle(image, (66, 60), 4, (20, 20, 20), -1)
        candidates = exp._dark_hub_candidates(image)
        self.assertTrue(candidates)
        self.assertTrue(all(not item["usable"] for item in candidates))
        self.assertTrue(any(item["competition"] for item in candidates))
        self.assertTrue(any(item["reject"] == "hub_competition"
                            for item in candidates))

    def test_single_threshold_cap_has_no_consensus(self):
        image = np.full((120, 120, 3), 230, dtype=np.uint8)
        cv2.circle(image, (60, 60), 6, (95, 95, 95), -1)
        candidates = exp._dark_hub_candidates(image)
        self.assertTrue(candidates)
        best = max(candidates, key=lambda item: item["score"])
        self.assertEqual(best["thresholds"], [100])
        self.assertFalse(best["usable"])
        self.assertTrue(best["ambiguous"])
        self.assertEqual(best["reject"], "hub_no_threshold_consensus")

    def test_separated_thick_lumps_are_ambiguous(self):
        image = np.full((140, 140, 3), 230, dtype=np.uint8)
        cv2.circle(image, (60, 70), 4, (20, 20, 20), -1)
        cv2.circle(image, (74, 70), 4, (20, 20, 20), -1)
        cv2.rectangle(image, (64, 68), (70, 72), (20, 20, 20), -1)
        candidates = exp._dark_hub_candidates(image)
        self.assertTrue(candidates)
        best = max(candidates, key=lambda item: item["score"])
        self.assertTrue(best["plateau_separated"])
        self.assertTrue(best["ambiguous"])
        self.assertFalse(best["competition"])


class AxisEvidenceTests(unittest.TestCase):
    def test_circular_interval_does_not_wrap_through_359(self):
        interval = exp._circular_interval_deg([359.5, 0.4, 1.2])
        self.assertTrue(interval["ok"])
        self.assertLess(interval["span_deg"], 2.0)
        self.assertTrue(interval["wraps_zero"])
        self.assertAlmostEqual(interval["start_deg"], 359.5, places=6)
        self.assertAlmostEqual(interval["end_deg"], 1.2, places=6)
        self.assertAlmostEqual(exp._angle_difference_deg(359.5, 0.4), 0.9,
                              places=6)

    def test_unverified_or_failed_axis_is_never_stable(self):
        stable, reason = exp._axis_stability(False, False, {"failed": []}, 0.0)
        self.assertFalse(stable)
        self.assertEqual(reason, "axis_unverified_fallback")
        stable, reason = exp._axis_stability(
            True, False, {"failed": [{"role": "plateau_cluster"}]}, 0.2)
        self.assertFalse(stable)
        self.assertEqual(reason, "axis_evidence_unstable")
        stable, reason = exp._axis_stability(True, True, {"failed": []}, 0.2)
        self.assertFalse(stable)
        self.assertEqual(reason, "axis_hub_ambiguous")

    def test_legal_nonzero_offset_is_not_axis_inconsistent(self):
        ring = {
            "center": [100.0, 100.0],
            "axes": [84.0, 78.0],
            "angle_deg": 0.0,
            "inlier_points": _ellipse_points(count=160),
        }
        hub = {
            "center": [106.0, 100.0],
            "blob_centroid_px": [102.0, 100.0],
            "centroid_offset_px": 4.0,
            "center_uncertainty_px": 0.5,
            "source": "dark_cap",
            "usable": True,
            "ambiguous": False,
        }
        result = exp.assess_axis(ring, hub, make_calibration())
        self.assertTrue(result["ok"], result.get("reason"))
        self.assertTrue(result["axis_verified"])
        self.assertAlmostEqual(result["hub_axis_px"][0], 106.0, places=6)
        self.assertGreater(result["ring_axis_offset_frac"], 0.06)
        self.assertLess(result["ring_axis_offset_frac"], exp.AXIS_OFFSET_MAX_FRAC)
        self.assertAlmostEqual(result["axis_uncertainty_px"], 0.5, places=6)

    def test_unverified_fallback_is_not_overwritten_by_zero_delta(self):
        # The ring centre is used only as a fallback diagnostic, so the needle
        # angle difference to it is zero; that must not make the axis stable.
        stable, reason = exp._axis_stability(
            False, False, {"failed": [], "interval": {"span_deg": 0.0}}, 0.0)
        self.assertFalse(stable)
        self.assertEqual(reason, "axis_unverified_fallback")

    def test_no_hub_fallback_has_no_axis_interval(self):
        image = exp.make_synthetic_scene('color', with_hub=False,
                                         needle_deg=330.0)
        result = exp.estimate_experiment_frame(
            image, make_calibration(), [0, 0, 200, 200],
            make_reference(image), stage="test", top_expansion_px=0)
        self.assertEqual(result["frame_status"], "low_conf")
        # R4 still refuses to pick a line without a coarse hub, but the reason
        # now belongs to the line stage; the R2 axis fallback stays unverified.
        self.assertEqual(result["reason"], "line_hub_missing")
        axis = result["axis"]
        self.assertFalse(axis["axis_verified"])
        self.assertFalse(axis["angle_stable"])
        self.assertIsNone(axis["angle_interval_deg"])
        self.assertIsNone(axis["angle_span_deg"])
        self.assertFalse(result["line"]["ok"])
        self.assertIsNone(result["line"]["angle_deg"])
        self.assertIsNone(result["diagnostic_candidate"])


class ContractTests(unittest.TestCase):
    def test_base_contract_has_experimental_true_and_null_value(self):
        result = exp._base_result()
        self.assertTrue(result["experimental"])
        self.assertIsNone(result["value"])
        self.assertIsNone(result["diagnostic_candidate"])

    def test_compact_and_csv_never_expose_a_trusted_value(self):
        result = {
            "index": 1, "split": "eval", "file": "x.jpg", "frame_sha256": "0" * 64,
            "width": 10, "height": 10, "frame_status": "low_conf",
            "reason": "axis_angle_uncertain", "roi": {}, "ring": {},
            "axis": {}, "roll": {}, "needle": {}, "diagnostic_candidate": None,
        }
        compact = exp._compact_frame_result(result)
        self.assertIsNone(compact["value"])
        self.assertTrue(compact["experimental"])
        row = exp._csv_row(result)
        self.assertEqual(row["value"], "")
        self.assertFalse(row["candidate_trusted"])
        self.assertTrue(row["experimental"])

    def test_default_modules_do_not_import_the_experiment(self):
        for name in ("dial_reading.py", "gauge.py"):
            text = (REPO_ROOT / "src" / "photo_capture" / "scripts" / name).read_text(
                encoding="utf-8")
            self.assertNotIn("dial_low_saturation_experiment", text)

    def test_module_has_no_camera_ros_or_gui_calls(self):
        source = Path(exp.__file__).read_text(encoding="utf-8")
        for forbidden in ("VideoCapture", "import rospy", "import roslib",
                          "cv2.imshow", "subprocess"):
            self.assertNotIn(forbidden, source)

    def test_output_target_refuses_non_empty_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "existing.txt").write_text("x", encoding="utf-8")
            with self.assertRaises(exp.ExperimentConfigError):
                exp._output_target(path)


@unittest.skipUnless(CANDIDATE_PATH.is_file() and REFERENCE_IMAGE.is_file()
                     and PHONE_IMAGE.is_file(),
                     "approved candidate/reference/phone control not present")
def _line_ring(center=(130.0, 130.0), axes=(120.0, 110.0),
               angle_deg=0.0):
    return {"center": [float(center[0]), float(center[1])],
            "axes": [float(axes[0]), float(axes[1])],
            "angle_deg": float(angle_deg)}


def _line_hub(center, cap_radius=7):
    return {"center": [float(center[0]), float(center[1])],
            "usable": True, "ambiguous": False, "source": "dark_cap",
            "plateau": {"span_px": [2 * cap_radius, 2 * cap_radius]}}


def make_line_scene(needle_deg=170.0, length=40.0, width=2, size=260,
                    center=(130.0, 130.0), axes=(120.0, 110.0),
                    ring_angle=0.0, hub_offset=(0.0, 0.0),
                    tail=None, second=None, cap_radius=7,
                    background=235, needle_value=60, noise_sigma=0.0):
    """Independent AA-drawn needle scene; truth is the drawn pixel angle.

    ``tail`` is ``(length, width)`` on the opposite side; ``second`` is
    ``(angle, length, width)`` for a second pointer.  Nothing here calls the
    estimator to construct the scene.
    """
    image = np.full((int(size), int(size), 3), int(background), dtype=np.uint8)
    hub = (float(center[0]) + float(hub_offset[0]),
           float(center[1]) + float(hub_offset[1]))

    def _draw(angle_deg, draw_length, draw_width):
        theta = math.radians(float(angle_deg))
        end = (int(round(hub[0] + draw_length * math.cos(theta))),
               int(round(hub[1] + draw_length * math.sin(theta))))
        cv2.line(image, (int(round(hub[0])), int(round(hub[1]))), end,
                 (int(needle_value),) * 3, int(draw_width),
                 lineType=cv2.LINE_AA)

    _draw(needle_deg, length, width)
    if tail is not None:
        _draw((float(needle_deg) + 180.0) % 360.0, tail[0], tail[1])
    if second is not None:
        _draw(second[0], second[1], second[2])
    cv2.circle(image, (int(round(hub[0])), int(round(hub[1]))),
               int(cap_radius), (40, 40, 40), -1)
    if noise_sigma > 0.0:
        rng = np.random.RandomState(20261115)
        noisy = image.astype(np.float32) + rng.normal(
            0.0, float(noise_sigma), image.shape)
        image = np.clip(noisy, 0, 255).astype(np.uint8)
    return image, _line_ring(center, axes, ring_angle), _line_hub(hub, cap_radius)


def make_tapered_line_scene(needle_deg=170.0, length=42.0, base_width=3.4,
                            tip_width=0.8, size=260, center=(130.0, 130.0),
                            axes=(120.0, 110.0), ring_angle=0.0,
                            hub_offset=(0.0, 0.0), tail=None, cap_radius=7,
                            background=235, needle_value=60, bend=0.0,
                            edge_bump=None, width_wave=None):
    """Independent supersampled tapered-needle scene.

    The body is a filled polygon whose half-width changes linearly from
    ``base_width/2`` at the pivot to ``tip_width/2`` at the tip.  For a
    symmetric straight taper the two edges are not parallel,
    ``2*atan((base_width-tip_width)/2/length)`` apart, while their midpoints
    stay on the drawn line.  Nothing here calls the estimator to construct the
    scene; ``needle_deg`` is the drawn pixel-space centreline angle.

    ``bend`` bends the centreline by that many pixels (normal, sine profile).
    ``edge_bump=(start, end, extra_px)`` moves only one edge outward over that
    fraction of the body.  ``width_wave=(amplitude, cycles)`` moves both edges
    symmetrically, keeping the midpoint straight while making the width
    non-linear.  ``tail=(length, width)`` is a thick opposite-side tail.
    """
    scale = 4
    canvas = np.full((int(size) * scale, int(size) * scale, 3),
                     int(background), dtype=np.uint8)
    hub = np.array([float(center[0]) + float(hub_offset[0]),
                    float(center[1]) + float(hub_offset[1])], dtype=float)
    theta = math.radians(float(needle_deg))
    unit = np.array([math.cos(theta), math.sin(theta)], dtype=float)
    normal = np.array([-math.sin(theta), math.cos(theta)], dtype=float)

    def half_width(t):
        value = 0.5 * (float(base_width)
                       + (float(tip_width) - float(base_width)) * t)
        if width_wave is not None:
            amplitude, cycles = width_wave
            value += float(amplitude) * math.sin(
                2.0 * math.pi * float(cycles) * t)
        return max(0.15, value)

    def one_edge_extra(t):
        if edge_bump is None:
            return 0.0
        start, end, extra = edge_bump
        if start <= t <= end:
            return float(extra) * math.sin(
                math.pi * (t - start) / max(1e-9, end - start))
        return 0.0

    upper = []
    lower = []
    count = 80
    for index in range(count + 1):
        t = index / float(count)
        point = (hub + float(length) * t * unit
                 + float(bend) * math.sin(math.pi * t) * normal)
        upper.append(point + (half_width(t) + one_edge_extra(t)) * normal)
        lower.append(point - half_width(t) * normal)
    polygon = np.round(np.asarray(upper + lower[::-1]) * scale).astype(np.int32)
    cv2.fillPoly(canvas, [polygon], (int(needle_value),) * 3,
                 lineType=cv2.LINE_AA)
    if tail is not None:
        tail_length, tail_width = tail
        end = np.round((hub - float(tail_length) * unit) * scale).astype(int)
        start = np.round(hub * scale).astype(int)
        cv2.line(canvas, tuple(start), tuple(end), (int(needle_value),) * 3,
                 max(1, int(round(float(tail_width) * scale))),
                 lineType=cv2.LINE_AA)
    cv2.circle(canvas, tuple(np.round(hub * scale).astype(int)),
               int(round(float(cap_radius) * scale)), (40, 40, 40), -1,
               lineType=cv2.LINE_AA)
    image = cv2.resize(canvas, (int(size), int(size)),
                       interpolation=cv2.INTER_AREA)
    return image, _line_ring(center, axes, ring_angle), _line_hub(hub, cap_radius)


def _angle_error(first, second):
    return abs(((float(first) - float(second) + 180.0) % 360.0) - 180.0)


class LineDirectionPositiveTests(unittest.TestCase):
    """Drawn-angle truth, never the detector output."""

    def test_multi_direction_recovery(self):
        for degree in (0, 30, 45, 90, 135, 170, 180, 225, 270, 315):
            image, ring, hub = make_line_scene(needle_deg=degree)
            line = exp.estimate_needle_line(image, ring, hub)
            truth = exp.synthetic_needle_truth(
                degree, axes=(120.0, 110.0))["expected_normalised_deg"]
            self.assertIsNotNone(line["angle_deg"], (degree, line["reason"]))
            self.assertLess(_angle_error(line["angle_deg"], truth), 1.5,
                            (degree, line["angle_deg"], truth))
            if line["ok"]:
                self.assertLessEqual(line["angle_span_deg"],
                                     exp.LINE_DIRECTION_TOL_DEG)
            else:
                # A rejected direction must say why and must not hide a wide
                # interval behind an accepted label.
                self.assertGreater(line["angle_span_deg"],
                                   exp.LINE_DIRECTION_TOL_DEG)

    def test_legal_pivot_offset_is_angle_invariant(self):
        results = []
        for offset in ((0.0, 0.0), (4.0, -3.0), (-5.0, 1.0), (6.0, 0.0)):
            image, ring, hub = make_line_scene(needle_deg=150.0,
                                               hub_offset=offset)
            line = exp.estimate_needle_line(image, ring, hub)
            self.assertTrue(line["ok"], (offset, line["reason"]))
            results.append(line["angle_deg"])
        spread = max(_angle_error(first, second)
                     for first in results for second in results)
        self.assertLess(spread, 0.1)
        truth = exp.synthetic_needle_truth(
            150.0, axes=(120.0, 110.0))["expected_normalised_deg"]
        self.assertLess(_angle_error(results[0], truth), 1.5)

    def test_longer_needle_keeps_drawn_angle(self):
        truth = exp.synthetic_needle_truth(
            170.0, axes=(120.0, 110.0))["expected_normalised_deg"]
        accepted = []
        for length in (40.0, 48.0):
            image, ring, hub = make_line_scene(needle_deg=170.0,
                                               length=length)
            line = exp.estimate_needle_line(image, ring, hub)
            self.assertTrue(line["ok"], (length, line["reason"]))
            accepted.append(line["angle_deg"])
            self.assertLess(_angle_error(line["angle_deg"], truth), 1.5)
        self.assertLess(_angle_error(accepted[0], accepted[1]), 1.0)
        # A short body is reported as short, not silently accepted.
        short, ring, hub = make_line_scene(needle_deg=170.0, length=20.0)
        line = exp.estimate_needle_line(short, ring, hub)
        self.assertFalse(line["ok"])
        self.assertIn(line["reason"], ("line_support_low", "line_needle_short"))

    def test_ring_angle_uses_the_spd_matrix_truth(self):
        for ring_angle in (0.0, 20.0, 70.0):
            image, ring, hub = make_line_scene(needle_deg=170.0,
                                               ring_angle=ring_angle)
            line = exp.estimate_needle_line(image, ring, hub)
            self.assertTrue(line["ok"], (ring_angle, line["reason"]))
            truth = exp.synthetic_needle_truth(
                170.0, axes=(120.0, 110.0),
                ring_angle_deg=ring_angle)["expected_normalised_deg"]
            self.assertLess(_angle_error(line["angle_deg"], truth), 1.5)

    def test_thicker_tail_does_not_flip_the_tip(self):
        image, ring, hub = make_line_scene(needle_deg=170.0,
                                           tail=(20.0, 6))
        line = exp.estimate_needle_line(image, ring, hub)
        self.assertTrue(line["ok"], line["reason"])
        truth = exp.synthetic_needle_truth(
            170.0, axes=(120.0, 110.0))["expected_normalised_deg"]
        self.assertLess(_angle_error(line["angle_deg"], truth), 1.5)
        self.assertGreater(line["tip_extent_px"], line["tail_extent_px"])


class LineDirectionRejectTests(unittest.TestCase):
    """Negative/stress cases are separate from the positive controls."""

    def test_cap_without_line_is_rejected(self):
        image = np.full((260, 260, 3), 235, dtype=np.uint8)
        cv2.circle(image, (130, 130), 7, (40, 40, 40), -1)
        line = exp.estimate_needle_line(image, _line_ring(),
                                        _line_hub((130.0, 130.0)))
        self.assertFalse(line["ok"])
        self.assertIn(line["reason"], ("line_no_needle", "line_support_low"))

    def test_broad_block_is_rejected(self):
        image = np.full((260, 260, 3), 235, dtype=np.uint8)
        cv2.rectangle(image, (118, 118), (162, 136), (60, 60, 60), -1)
        cv2.circle(image, (130, 130), 7, (40, 40, 40), -1)
        line = exp.estimate_needle_line(image, _line_ring(),
                                        _line_hub((130.0, 130.0)))
        self.assertFalse(line["ok"])
        self.assertIn(line["reason"],
                      ("line_no_needle", "line_fit_failed",
                       "line_support_low", "line_needle_short"))

    def test_wide_needle_is_rejected(self):
        image, ring, hub = make_line_scene(needle_deg=170.0, width=9)
        line = exp.estimate_needle_line(image, ring, hub)
        self.assertFalse(line["ok"])
        self.assertIn(line["reason"],
                      ("line_no_needle", "line_fit_failed",
                       "line_support_low", "line_needle_short"))

    def test_equal_double_needle_is_ambiguous(self):
        image, ring, hub = make_line_scene(needle_deg=170.0,
                                           second=(350.0, 40.0, 2))
        line = exp.estimate_needle_line(image, ring, hub)
        self.assertFalse(line["ok"])
        self.assertEqual(line["reason"], "line_direction_ambiguous")
        self.assertLess(line["tip_tail_ratio"],
                        exp.LINE_TIP_TAIL_RATIO)

    def test_second_needle_at_an_angle_is_rejected(self):
        image, ring, hub = make_line_scene(needle_deg=170.0,
                                           second=(300.0, 40.0, 2))
        line = exp.estimate_needle_line(image, ring, hub)
        self.assertFalse(line["ok"])
        self.assertIn(line["reason"],
                      ("line_competing_peaks", "line_direction_ambiguous"))

    def test_low_contrast_noise_is_not_silently_accepted(self):
        image, ring, hub = make_line_scene(needle_deg=170.0, width=2,
                                           needle_value=195, background=235,
                                           noise_sigma=2.0)
        line = exp.estimate_needle_line(image, ring, hub)
        if line["ok"]:
            self.assertLessEqual(line["angle_span_deg"],
                                 exp.LINE_DIRECTION_TOL_DEG)
        else:
            self.assertTrue(str(line["reason"]).startswith("line_"))


class LineSpanMathTests(unittest.TestCase):
    """Hand-calculated angular spans, independent of the detector output."""

    def test_direction_span_crosses_zero_without_doubling(self):
        # Smallest circular arcs: 359.5..1.2 is 1.7 deg, 358..2 is 4 deg.
        self.assertAlmostEqual(
            exp._direction_span_deg([359.5, 0.4, 1.2]), 1.7, places=6)
        self.assertAlmostEqual(
            exp._direction_span_deg([358.0, 0.0, 2.0]), 4.0, places=6)
        self.assertAlmostEqual(
            exp._direction_span_deg([0.0, 4.0]), 4.0, places=6)

    def test_bootstrap_known_slopes_is_not_doubled(self):
        # Stub the fit so the bootstrap directions are exactly -1/0/+1 deg.
        # The true circular span is 2 deg; the old 2*atan(max-min) returned
        # about 4 deg.  The expected value is fixed by hand, not by the code
        # under test.
        slopes = [math.tan(math.radians(-1.0)), 0.0,
                  math.tan(math.radians(1.0))]
        state = {"index": 0}

        def fake_fit(s, t, w):
            value = slopes[state["index"] % len(slopes)]
            state["index"] += 1
            return {"slope": value, "intercept": 0.0, "rms_px": 0.0,
                    "n": int(len(s))}

        s = np.arange(16, dtype=float)
        t = np.zeros_like(s)
        w = np.ones_like(s)
        with mock.patch.object(exp, "_line_robust_fit", fake_fit):
            span = exp._line_bootstrap_span(
                s, t, w, {"slope": 0.0}, base_angle_deg=179.0)
        self.assertIsNotNone(span)
        self.assertAlmostEqual(span, 2.0, places=6)
        self.assertLess(span, 3.0)

    def test_perfect_line_bootstrap_is_zero_not_missing(self):
        s = np.arange(16, dtype=float)
        t = 0.5 * s
        w = np.ones_like(s)
        fit = exp._line_robust_fit(s, t, w)
        self.assertIsNotNone(fit)
        span = exp._line_bootstrap_span(s, t, w, fit)
        self.assertIsNotNone(span)
        self.assertLess(span, 1e-6)


class TaperedNeedleTests(unittest.TestCase):
    """Straight tapered bodies whose edges open but whose midpoints are
    collinear.  These are the cases R3 wrongly rejected as non-parallel."""

    LEGITIMATE_REJECTIONS = (
        "line_direction_unstable", "line_needle_short", "line_body_not_straight",
        "line_width_inconsistent", "line_direction_evidence_missing",
    )

    def test_open_edged_tapered_needle_uses_centreline(self):
        # Construction truth: full edge opening from 3.4 px to 0.8 px over
        # 42 px is 2*atan(1.3/42) = 3.55 deg, while the midpoint is exactly on
        # the drawn line.
        construction = 2.0 * math.degrees(
            math.atan(((3.4 - 0.8) / 2.0) / 42.0))
        self.assertGreater(construction, 2.0)
        for degree in (0, 30, 45, 90, 135, 170, 180, 225, 270, 300):
            image, ring, hub = make_tapered_line_scene(needle_deg=degree)
            line = exp.estimate_needle_line(image, ring, hub)
            truth = exp.synthetic_needle_truth(
                degree, axes=(120.0, 110.0))["expected_normalised_deg"]
            self.assertIsNotNone(line["angle_deg"], (degree, line["reason"]))
            self.assertLess(_angle_error(line["angle_deg"], truth), 2.0,
                            (degree, line["angle_deg"], truth))
            self.assertGreater(line["edge_opening_deg"], 0.0)
            if line["ok"]:
                self.assertLessEqual(line["angle_span_deg"],
                                     exp.LINE_DIRECTION_TOL_DEG)
            else:
                # Rejection must be a real stability/shape reason, never the
                # removed "edges not parallel" model gate.
                self.assertIn(line["reason"], self.LEGITIMATE_REJECTIONS,
                              (degree, line["reason"]))
                self.assertNotEqual(line["reason"], "line_edges_not_parallel")

    def test_primary_angle_is_centreline_not_strip_peak(self):
        image, ring, hub = make_tapered_line_scene(needle_deg=170.0)
        line = exp.estimate_needle_line(image, ring, hub)
        self.assertIsNotNone(line["angle_deg"], line["reason"])
        self.assertAlmostEqual(line["angle_px_deg"],
                               line["edge_center_angle_deg"], places=9)
        self.assertGreater(_angle_error(line["angle_px_deg"],
                                        line["strip_angle_deg"]), 0.1)

    def test_tapered_needle_translation_and_taper(self):
        cases = (
            ((130.0, 130.0), (0.0, 0.0), 3.2, 1.0),
            ((120.0, 140.0), (4.0, -3.0), 3.4, 0.8),
            ((130.0, 130.0), (-5.0, 1.0), 3.0, 1.0),
        )
        truth = exp.synthetic_needle_truth(
            170.0, axes=(120.0, 110.0))["expected_normalised_deg"]
        for center, offset, base_width, tip_width in cases:
            construction = 2.0 * math.degrees(
                math.atan(((base_width - tip_width) / 2.0) / 42.0))
            self.assertGreater(construction, 2.0)
            image, ring, hub = make_tapered_line_scene(
                needle_deg=170.0, center=center, hub_offset=offset,
                base_width=base_width, tip_width=tip_width)
            line = exp.estimate_needle_line(image, ring, hub)
            self.assertIsNotNone(line["angle_deg"], line["reason"])
            self.assertLess(_angle_error(line["angle_deg"], truth), 2.0,
                            (center, offset, line["reason"]))
            if not line["ok"]:
                self.assertIn(line["reason"], self.LEGITIMATE_REJECTIONS,
                              (center, offset, line["reason"]))

    def test_tapered_thick_tail_does_not_flip_tip(self):
        image, ring, hub = make_tapered_line_scene(
            needle_deg=170.0, tail=(18.0, 6.0))
        line = exp.estimate_needle_line(image, ring, hub)
        truth = exp.synthetic_needle_truth(
            170.0, axes=(120.0, 110.0))["expected_normalised_deg"]
        self.assertIsNotNone(line["angle_deg"], line["reason"])
        self.assertLess(_angle_error(line["angle_deg"], truth), 2.0)
        self.assertGreater(line["tip_extent_px"], line["tail_extent_px"])
        if not line["ok"]:
            self.assertIn(line["reason"], self.LEGITIMATE_REJECTIONS,
                          line["reason"])

    def test_bent_tapered_needle_is_rejected(self):
        image, ring, hub = make_tapered_line_scene(
            needle_deg=170.0, bend=2.0)
        line = exp.estimate_needle_line(image, ring, hub)
        self.assertFalse(line["ok"])
        self.assertIn(line["reason"],
                      ("line_direction_unstable", "line_body_not_straight"))
        self.assertTrue(line["angle_span_deg"] is None
                        or line["angle_span_deg"] > exp.LINE_DIRECTION_TOL_DEG
                        or line["centerline_rms_px"]
                        > exp.LINE_MAX_CENTERLINE_RMS_PX)

    def test_one_edge_irregular_is_rejected(self):
        image, ring, hub = make_tapered_line_scene(
            needle_deg=170.0, edge_bump=(0.35, 0.65, 1.5))
        line = exp.estimate_needle_line(image, ring, hub)
        self.assertFalse(line["ok"])
        self.assertIn(line["reason"],
                      ("line_needle_short", "line_body_not_straight",
                       "line_width_inconsistent", "line_direction_unstable"))
        self.assertTrue(line["tip_extent_px"]
                        < exp.LINE_MIN_TIP_LENGTH_FRAC * 60.0
                        or (line["centerline_rms_px"] or 0.0)
                        > exp.LINE_MAX_CENTERLINE_RMS_PX
                        or (line["width_rms_px"] or 0.0)
                        > exp.LINE_MAX_WIDTH_RMS_PX
                        or (line["angle_span_deg"] or 0.0)
                        > exp.LINE_DIRECTION_TOL_DEG)

    def test_nonlinear_width_is_rejected_by_shape(self):
        image, ring, hub = make_tapered_line_scene(
            needle_deg=170.0, width_wave=(0.8, 2.0))
        line = exp.estimate_needle_line(image, ring, hub)
        self.assertFalse(line["ok"])
        self.assertEqual(line["reason"], "line_width_inconsistent")
        self.assertGreater(line["width_rms_px"],
                           exp.LINE_MAX_WIDTH_RMS_PX)


class LineEvidenceTests(unittest.TestCase):
    """Missing perturb/bootstrap evidence is never zero variation."""

    def test_missing_hub_perturbation_is_rejected(self):
        image, ring, hub = make_line_scene(needle_deg=170.0)
        nominal = np.asarray(hub["center"], dtype=float)
        original = exp._line_refine

        def wrapper(gray, centre, matrix, hub_px, *args, **kwargs):
            if not np.allclose(np.asarray(hub_px, dtype=float), nominal):
                return None
            return original(gray, centre, matrix, hub_px, *args, **kwargs)

        with mock.patch.object(exp, "_line_refine", wrapper):
            line = exp.estimate_needle_line(image, ring, hub)
        self.assertFalse(line["ok"])
        self.assertEqual(line["reason"], "line_direction_evidence_missing")
        self.assertIn("hub_perturbation", line["missing_evidence"])
        self.assertIsNone(line["uncertainty_components_deg"]["hub_span_deg"])
        self.assertEqual(len(line["perturbation_failures"]["hub"]), 4)
        self.assertIsNone(line["angle_span_deg"])

    def test_missing_ring_perturbation_is_rejected(self):
        image, ring, hub = make_line_scene(needle_deg=170.0)
        nominal = np.asarray(ring["center"], dtype=float)
        nominal_matrix, _, _, _ = exp._ellipse_matrix(
            (float(ring["axes"][0]), float(ring["axes"][1])),
            float(ring["angle_deg"]))
        original = exp._line_refine

        def wrapper(gray, centre, matrix, hub_px, *args, **kwargs):
            if (not np.allclose(np.asarray(centre, dtype=float), nominal)
                    or not np.allclose(np.asarray(matrix, dtype=float),
                                       nominal_matrix)):
                return None
            return original(gray, centre, matrix, hub_px, *args, **kwargs)

        with mock.patch.object(exp, "_line_refine", wrapper):
            line = exp.estimate_needle_line(image, ring, hub)
        self.assertFalse(line["ok"])
        self.assertEqual(line["reason"], "line_direction_evidence_missing")
        self.assertIn("ring_perturbation", line["missing_evidence"])
        self.assertIsNone(line["uncertainty_components_deg"]["ring_span_deg"])
        self.assertEqual(len(line["perturbation_failures"]["ring"]), 6)

    def test_missing_bootstrap_is_rejected(self):
        image, ring, hub = make_line_scene(needle_deg=170.0)
        with mock.patch.object(exp, "_line_bootstrap_span",
                               lambda *args, **kwargs: None):
            line = exp.estimate_needle_line(image, ring, hub)
        self.assertFalse(line["ok"])
        self.assertEqual(line["reason"], "line_direction_evidence_missing")
        self.assertTrue(line["bootstrap_missing"])
        self.assertIn("bootstrap_span_deg", line["missing_evidence"])
        self.assertIsNone(line["angle_span_deg"])


class FixtureConventionTests(unittest.TestCase):
    def test_synthetic_scene_draws_the_requested_angle(self):
        # The old R1/R2 fixture drew ~10 deg for needle_deg=170; the corrected
        # half-axis [cos, sin] convention must give ~170.
        image = exp.make_synthetic_scene('color', needle_deg=170.0)
        ring = exp.detect_edge_ring(image)
        self.assertTrue(ring["ok"], ring.get("reason"))
        line = exp.estimate_needle_line(image, ring["ring"], ring["hub"])
        truth = exp.synthetic_needle_truth(170.0)["expected_normalised_deg"]
        self.assertIsNotNone(line["angle_deg"], line["reason"])
        self.assertLess(_angle_error(line["angle_deg"], truth), 2.0)
        self.assertGreater(_angle_error(line["angle_deg"], 10.0), 20.0)

    def test_truth_helper_uses_half_axis_length(self):
        truth = exp.synthetic_needle_truth(170.0, axes=(84.0, 78.0),
                                           needle_length_frac=0.55)
        self.assertAlmostEqual(truth["length_px"], 0.55 * 0.5 * 84.0,
                               places=6)
        self.assertAlmostEqual(truth["drawn_pixel_deg"], 170.0, places=6)


class JointBurstTests(unittest.TestCase):
    @staticmethod
    def _profile(center):
        angles = np.arange(360)
        delta = ((angles - center + 180.0) % 360.0) - 180.0
        return np.exp(-0.5 * (delta / 2.0) ** 2).tolist()

    def _frame(self, index, center, roll):
        return {"index": index,
                "line": {"ok": True, "angle_deg": center,
                         "angle_span_deg": 1.0,
                         "profile_norm_deg": self._profile(center)},
                "roll": {"ok": True, "roll_deg": roll}}

    def test_shared_direction_is_internal_consistency_only(self):
        frames = [self._frame(1, 100.0, 0.0),
                  self._frame(2, 105.0, 5.0),
                  self._frame(3, 450.0 % 360.0, 350.0)]
        joint = exp._joint_burst_direction(frames)
        self.assertTrue(joint["ok"], joint.get("reason"))
        self.assertEqual(joint["frames_used"], 3)
        self.assertLess(_angle_error(joint["joint_compensated_deg"], 100.0),
                        1.0)
        self.assertTrue(joint["internal_consistency_only"])
        self.assertFalse(joint["independent_verification"])

    def test_inconsistent_frames_are_not_forced_together(self):
        frames = [self._frame(1, 100.0, 0.0),
                  self._frame(2, 130.0, 0.0)]
        joint = exp._joint_burst_direction(frames)
        self.assertFalse(joint["ok"])
        self.assertEqual(joint["reason"], "joint_direction_inconsistent")


class PhoneIntegrationTests(unittest.TestCase):
    def test_phone_control_runs_and_keeps_value_null(self):
        calibration = exp.load_calibration(str(CANDIDATE_PATH), True, CANDIDATE_SHA256)
        reference_path = Path(calibration["raw"]["source_image"]["path"])
        self.assertEqual(exp._sha256_file(reference_path),
                         calibration["source_sha256"])
        reference_image = cv2.imread(str(reference_path), cv2.IMREAD_COLOR)
        reference = exp.reference_signature(reference_image, calibration)
        self.assertTrue(reference["ok"])
        phone = cv2.imread(str(PHONE_IMAGE), cv2.IMREAD_COLOR)
        result = exp.estimate_experiment_frame(
            phone, calibration, [300, 700, 320, 320], reference, stage="phone")
        self.assertTrue(result["experimental"])
        self.assertIsNone(result["value"])
        self.assertTrue(result["ring"]["ok"], result["ring"].get("reason"))
        self.assertGreater(max(result["ring"]["axes_px"]), 250.0)
        self.assertTrue(result["roll"]["ok"], result["roll"].get("reason"))
        self.assertIsNotNone(result["diagnostic_candidate"])
        self.assertFalse(result["diagnostic_candidate"]["trusted"])
        self.assertIsNotNone(result["diagnostic_candidate"]["candidate_value"])
        # R2's pivot diagnostics are retained, but R4 judges the centreline.
        axis = result["axis"]
        self.assertTrue(axis["axis_verified"])
        self.assertGreater(axis["ring_axis_offset_frac"], 0.01)
        self.assertGreater(axis["needle_angle_delta_deg"],
                           exp.AXIS_ANGLE_TOL_DEG)
        self.assertTrue(axis["angle_stable"], axis.get("angle_interval_deg"))
        line = result["line"]
        self.assertIsNotNone(line["angle_deg"])
        self.assertGreater(line["support_frac"], 0.3)
        # This phone pointer is thicker at the base than the 75x75 camera
        # needle; R4 reports its width/opening/support evidence instead of hiding it.
        if line["ok"]:
            self.assertEqual(result["frame_status"], "ok")
            self.assertLessEqual(line["angle_span_deg"],
                                 exp.LINE_DIRECTION_TOL_DEG)
        else:
            self.assertEqual(result["frame_status"], "low_conf")
            self.assertEqual(result["reason"], line["reason"])
        self.assertGreater(line["tip_extent_px"], line["tail_extent_px"])
        self.assertFalse(result["trusted"])

    def test_phone_60px_control_is_retested(self):
        calibration = exp.load_calibration(str(CANDIDATE_PATH), True, CANDIDATE_SHA256)
        reference_path = Path(calibration["raw"]["source_image"]["path"])
        reference_image = cv2.imread(str(reference_path), cv2.IMREAD_COLOR)
        reference = exp.reference_signature(reference_image, calibration)
        controls = exp.run_phone_controls(
            PHONE_IMAGE, [300, 700, 320, 320], calibration, reference,
            target_major_px=296.0, target_size_px=60.0)
        names = [name for name, _, _ in controls]
        self.assertEqual(names, ["phone_full", "phone_60px"])
        small = controls[1][1]
        self.assertTrue(small["ring"]["ok"], small["ring"].get("reason"))
        self.assertGreater(max(small["ring"]["axes_px"]), 50.0)
        self.assertLess(max(small["ring"]["axes_px"]), 70.0)
        self.assertIsNone(small["value"])
        self.assertFalse(small["trusted"])
        # At 60 px the tiny pivot may disappear; R4 then refuses to pick a line
        # (line_hub_missing) instead of inventing one.
        line = small["line"]
        if line["angle_deg"] is None:
            self.assertEqual(line["reason"], "line_hub_missing")
            self.assertIsNone(small["diagnostic_candidate"])
        elif line["ok"]:
            self.assertLessEqual(line["angle_span_deg"],
                                 exp.LINE_DIRECTION_TOL_DEG)
        else:
            self.assertTrue(str(line["reason"]).startswith("line_"))


class CoordinateUnitGateTests(unittest.TestCase):
    """Hand-calculated checks for the pixel -> normalised gate correction."""

    @staticmethod
    def _mapped_angle(matrix, pixel_deg):
        theta = math.radians(float(pixel_deg))
        mapped = matrix @ np.array([math.cos(theta), math.sin(theta)])
        return math.degrees(math.atan2(mapped[1], mapped[0])) % 360.0

    @classmethod
    def _mapped_span(cls, matrix, pixel_angles):
        mapped = [cls._mapped_angle(matrix, value) for value in pixel_angles]
        base = mapped[0]
        relative = [((value - base + 180.0) % 360.0) - 180.0
                    for value in mapped]
        return max(relative) - min(relative)

    def test_anisotropic_diagonal_rejects_old_pixel_span(self):
        # A=diag(0.5, 1) has kappa=2.  A 1.9 deg pixel span was accepted by the
        # old pixel-only gate, but maps to 3.8 deg at worst, so it must fail.
        kappa = exp._spd_condition_number(np.diag([0.5, 1.0]))
        self.assertAlmostEqual(kappa, 2.0, places=12)
        pixel_span = 1.9
        self.assertLess(pixel_span, exp.LINE_DIRECTION_TOL_DEG)
        bound = exp._normalized_span_upper_bound_deg(pixel_span, kappa)
        self.assertAlmostEqual(bound, 3.8, places=12)
        self.assertGreater(bound, exp.LINE_DIRECTION_TOL_DEG)

    def test_isotropic_kappa_one_keeps_span(self):
        # Every isotropic positive matrix has kappa=1, so the bound is exactly
        # the pixel span (not scaled and not clamped).
        for matrix in (np.eye(2), np.diag([0.5, 0.5]),
                       _matrix((120.0, 120.0), 37.0)):
            kappa = exp._spd_condition_number(matrix)
            self.assertAlmostEqual(kappa, 1.0, places=12)
            bound = exp._normalized_span_upper_bound_deg(1.9, kappa)
            self.assertAlmostEqual(bound, 1.9, places=12)

    def test_rotated_spd_mapped_span_stays_within_bound(self):
        # A=rot(30) diag(2,1) rot(30)^T has kappa=2.  Three pixel directions
        # spanning 1.0 deg map to an actual span of ~1.499 deg, below 2.0.
        matrix = _matrix((120.0, 60.0), 30.0)
        kappa = exp._spd_condition_number(matrix)
        self.assertAlmostEqual(kappa, 2.0, places=12)
        pixel_span = 1.0
        bound = exp._normalized_span_upper_bound_deg(pixel_span, kappa)
        self.assertAlmostEqual(bound, 2.0, places=12)
        actual = self._mapped_span(matrix, (10.0, 10.5, 11.0))
        self.assertLessEqual(actual, bound + 1e-12)
        # The same bound also covers a mapped arc that crosses 0/360.
        matrix_zero = _matrix((120.0, 60.0), 0.0)
        kappa_zero = exp._spd_condition_number(matrix_zero)
        bound_zero = exp._normalized_span_upper_bound_deg(0.9, kappa_zero)
        mapped = [self._mapped_angle(matrix_zero, value)
                  for value in (359.5, 0.4)]
        self.assertGreater(max(mapped), 180.0)
        self.assertLess(min(mapped), 180.0)
        self.assertLessEqual(self._mapped_span(matrix_zero, (359.5, 0.4)),
                             bound_zero + 1e-9)

    def test_invalid_matrix_fails_closed(self):
        bad = (np.array([[np.nan, 0.0], [0.0, 1.0]]),
               np.array([[np.inf, 0.0], [0.0, 1.0]]),
               np.diag([0.0, 1.0]),
               np.array([[1.0, 2.0], [0.0, 1.0]]),
               np.array([[1.0, 0.0], [0.0, -1.0]]))
        for matrix in bad:
            self.assertIsNone(exp._spd_condition_number(matrix))
            self.assertIsNone(exp._normalized_span_upper_bound_deg(1.9, None))
        image, ring, hub = make_line_scene(needle_deg=170.0)
        for axes in ((float("nan"), 110.0), (0.0, 110.0),
                     (110.0, float("inf"))):
            broken = dict(ring)
            broken["axes"] = [float(axes[0]), float(axes[1])]
            line = exp.estimate_needle_line(image, broken, hub)
            self.assertFalse(line["ok"])
            self.assertEqual(line["reason"], "line_geometry_invalid")

    def test_direction_angle_is_unchanged_by_span_unit(self):
        image, ring, hub = make_line_scene(needle_deg=45.0,
                                           axes=(120.0, 60.0), ring_angle=20.0)
        line = exp.estimate_needle_line(image, ring, hub)
        self.assertIsNotNone(line["angle_px_deg"], line.get("reason"))
        self.assertIsNotNone(line["angle_deg"], line.get("reason"))
        matrix = _matrix(ring["axes"], ring["angle_deg"])
        theta = math.radians(float(line["angle_px_deg"]))
        mapped = matrix @ np.array([math.cos(theta), math.sin(theta)])
        expected = math.degrees(math.atan2(mapped[1], mapped[0])) % 360.0
        self.assertAlmostEqual(line["angle_deg"], expected, places=9)
        if line["pixel_span_deg"] is not None:
            self.assertAlmostEqual(
                line["normalized_span_upper_bound_deg"],
                min(360.0, line["matrix_condition_number"]
                    * line["pixel_span_deg"]), places=9)
        if line["ok"]:
            self.assertLessEqual(line["normalized_span_upper_bound_deg"],
                                 exp.LINE_DIRECTION_TOL_DEG)


if __name__ == "__main__":
    unittest.main()
