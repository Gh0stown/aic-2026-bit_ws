#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Specified-ROI offline single-frame humidity reading (G1b1).

This module does not search the whole image for a dial and does not produce a
final trusted humidity value.  The caller supplies a rectangular humidity ROI
and an explicitly allowed candidate calibration.  A single frame can only
reach ``frame_status=ok`` with a ``candidate_value``; the public ``value`` is
deliberately null until multi-frame evidence exists.

Only NumPy, OpenCV, the standard library and the already approved ``dial_math``
helpers are used.  ``estimate_frame`` is pure: no file I/O, no threads, no GUI.
"""

import argparse
import hashlib
import json
import math
import os
import sys
import uuid
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from dial_math import lookup_value, validate_scale, wrap_deg


A08_CANDIDATE_SHA256 = "d7dccc3e6d80f24199405dce07a516fd119d4502724efa413983682d2e5df198"
TOOL_VERSION = "dial_reading.py 2"

COLOR_SAT_MIN = 90.0
COLOR_VAL_MIN = 60.0
RING_CLOSE_KERNEL = 9
RING_MIN_AREA = 300.0
RING_MIN_AXIS_PX = 20.0
RING_MAX_AXIS_RATIO = 3.0
RING_MAX_RESIDUAL_RMS_PX = 3.0
RING_MIN_COVERAGE_BINS = 120
RING_AMBIGUITY_MARGIN = 0.5

BLUE_RHO = 0.94
BLUE_HUE_MIN = 90
BLUE_HUE_MAX = 135
BLUE_SAT_MIN = 120
BLUE_VAL_MIN = 80
BLUE_GAP_TOL_DEG = 8.0
BLUE_MIN_RUN_DEG = 8.0
BLUE_ANGLE_SAMPLES = 7200

NEEDLE_RHO_MIN = 0.14
NEEDLE_RHO_MAX = 0.70
NEEDLE_RADIAL_POINTS = 70
NEEDLE_ANGLE_SAMPLES = 3600
NEEDLE_DARK_MIN = 15.0
NEEDLE_DARK_BASE = 150.0
NEEDLE_COVERAGE_MIN = 0.70
NEEDLE_PEAK_SEPARATION_DEG = 8.0
NEEDLE_MIN_PEAK_SCORE = 10.0
NEEDLE_COMPETITION_RATIO = 0.80
AXIS_ROLL_MAX_DELTA_DEG = 0.5

STATUS_VALUES = ("ok", "out_of_range", "low_conf", "no_dial", "unstable")


class ReadingConfigError(ValueError):
    """Invalid calibration, ROI, image or output configuration."""


def _finite(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ReadingConfigError("{} must be a finite number".format(name))
    result = float(value)
    if not math.isfinite(result):
        raise ReadingConfigError("{} must be finite".format(name))
    return result


def _point2(raw, name: str) -> np.ndarray:
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise ReadingConfigError("{} must be a two-element point".format(name))
    return np.array([_finite(raw[0], name + "[0]"),
                     _finite(raw[1], name + "[1]")], dtype=float)


def _degree_table(raw, name: str) -> List[float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 11:
        raise ReadingConfigError("{} must contain 11 values".format(name))
    return [_finite(value, "{}[{}]".format(name, index))
            for index, value in enumerate(raw)]


def load_calibration(path: str, allow_candidate: bool,
                     expected_sha256: str = A08_CANDIDATE_SHA256) -> Dict:
    if not allow_candidate:
        raise ReadingConfigError("loading a candidate calibration requires --allow-candidate")
    candidate_path = Path(path)
    try:
        payload = candidate_path.read_bytes()
    except OSError as error:
        raise ReadingConfigError("cannot read calibration: {}".format(error))
    actual_sha = hashlib.sha256(payload).hexdigest()
    if actual_sha != str(expected_sha256).lower():
        raise ReadingConfigError(
            "candidate calibration SHA-256 mismatch: expected {}, got {}".format(
                expected_sha256, actual_sha))
    try:
        data = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ReadingConfigError("calibration is not valid JSON: {}".format(error))
    if not isinstance(data, dict) or data.get("status") != "candidate":
        raise ReadingConfigError("calibration status must be 'candidate'")

    table_deg = _degree_table(data.get("table_deg"), "table_deg")
    table_val = _degree_table(data.get("table_val"), "table_val")
    try:
        validate_scale(table_deg, table_val)
    except ValueError as error:
        raise ReadingConfigError("invalid candidate table: {}".format(error))

    ring = data.get("ring")
    if not isinstance(ring, dict):
        raise ReadingConfigError("ring must be an object")
    center = _point2(ring.get("center_px"), "ring.center_px")
    axes = ring.get("fit_axes_px")
    if not isinstance(axes, (list, tuple)) or len(axes) != 2:
        raise ReadingConfigError("ring.fit_axes_px must have two values")
    axes = np.array([_finite(axes[0], "ring.fit_axes_px[0]"),
                     _finite(axes[1], "ring.fit_axes_px[1]")], dtype=float)
    if np.any(axes <= 0.0):
        raise ReadingConfigError("ring axes must be positive")
    matrix_raw = ring.get("matrix")
    if not isinstance(matrix_raw, (list, tuple)) or len(matrix_raw) != 2:
        raise ReadingConfigError("ring.matrix must be 2x2")
    try:
        matrix = np.array([[float(matrix_raw[0][0]), float(matrix_raw[0][1])],
                           [float(matrix_raw[1][0]), float(matrix_raw[1][1])]],
                          dtype=float)
    except (TypeError, IndexError, ValueError) as error:
        raise ReadingConfigError("ring.matrix must be numeric 2x2: {}".format(error))
    if matrix.shape != (2, 2) or not np.all(np.isfinite(matrix)):
        raise ReadingConfigError("ring.matrix must be finite 2x2")
    try:
        np.linalg.inv(matrix)
    except np.linalg.LinAlgError:
        raise ReadingConfigError("ring.matrix is singular")

    axis = _point2(data.get("humidity_axis_px"), "humidity_axis_px")
    blue = data.get("blue_segment")
    if not isinstance(blue, dict):
        raise ReadingConfigError("blue_segment must be an object")
    blue_mid = _finite(blue.get("midpoint_deg"), "blue_segment.midpoint_deg")
    boundaries = blue.get("boundaries")
    if not isinstance(boundaries, list) or len(boundaries) != 2:
        raise ReadingConfigError("blue_segment.boundaries must contain two points")
    boundary_points = [
        _point2(boundaries[0].get("point_px"), "blue_segment.boundaries[0].point_px"),
        _point2(boundaries[1].get("point_px"), "blue_segment.boundaries[1].point_px"),
    ]

    source = data.get("source_image") if isinstance(data.get("source_image"), dict) else {}
    return {
        "calib_version": str(data.get("calib_version", "")),
        "calib_sha256": actual_sha,
        "source_sha256": str(source.get("sha256", "")),
        "annotation_sha256": str(data.get("annotation_sha256", "")),
        "table_deg": np.array(table_deg, dtype=float),
        "table_val": np.array(table_val, dtype=float),
        "ring_center": center,
        "ring_axes": axes,
        "matrix": matrix,
        "axis": axis,
        "blue_mid": blue_mid,
        "blue_boundaries": boundary_points,
        "raw": data,
    }


def _roi_tuple(roi, shape: Sequence[int]) -> Tuple[int, int, int, int]:
    if not isinstance(roi, (list, tuple)) or len(roi) != 4:
        raise ReadingConfigError("roi must contain x y w h")
    values = []
    for index, raw in enumerate(roi):
        value = _finite(raw, "roi[{}]".format(index))
        if int(value) != value:
            raise ReadingConfigError("roi values must be integers")
        values.append(int(value))
    x, y, width, height = values
    image_h, image_w = int(shape[0]), int(shape[1])
    if width <= 0 or height <= 0 or x < 0 or y < 0:
        raise ReadingConfigError("roi must be a positive rectangle inside the image")
    if x + width > image_w or y + height > image_h:
        raise ReadingConfigError("roi is outside the image")
    return x, y, width, height


def _ellipse_residual(points: np.ndarray, center: np.ndarray,
                      axes: Tuple[float, float], angle_deg: float) -> np.ndarray:
    semi_a = max(float(axes[0]), 1e-12) / 2.0
    semi_b = max(float(axes[1]), 1e-12) / 2.0
    theta = math.radians(float(angle_deg))
    dx = points[:, 0] - center[0]
    dy = points[:, 1] - center[1]
    xr = dx * math.cos(theta) + dy * math.sin(theta)
    yr = -dx * math.sin(theta) + dy * math.cos(theta)
    normalised = np.hypot(xr / semi_a, yr / semi_b)
    radius = np.hypot(xr, yr)
    return np.abs(normalised - 1.0) * radius / np.maximum(normalised, 1e-12)


def _ellipse_matrix(axes: Tuple[float, float], angle_deg: float) -> Tuple[np.ndarray, float, float, float]:
    width, height = float(axes[0]), float(axes[1])
    if width >= height:
        major, minor, major_angle = width / 2.0, height / 2.0, angle_deg
    else:
        major, minor, major_angle = height / 2.0, width / 2.0, angle_deg + 90.0
    theta = math.radians(float(major_angle))
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]], dtype=float)
    matrix = rotation @ np.diag([1.0 / major, 1.0 / minor]) @ rotation.T
    return matrix, major, minor, float(major_angle % 180.0)


def _outer_edge_from_contour(points: np.ndarray) -> np.ndarray:
    centroid = np.mean(points, axis=0)
    angles = np.degrees(np.arctan2(points[:, 1] - centroid[1],
                                   points[:, 0] - centroid[0])) % 360.0
    radii = np.hypot(points[:, 0] - centroid[0],
                     points[:, 1] - centroid[1])
    selected = []
    for degree in range(360):
        mask = np.abs(((angles - degree + 180.0) % 360.0) - 180.0) <= 0.5
        if not np.any(mask):
            continue
        indices = np.where(mask)[0]
        best = indices[np.argmax(radii[indices])]
        selected.append(points[best])
    return np.asarray(selected, dtype=np.float32)


def detect_ring(image_bgr: np.ndarray, roi: Tuple[int, int, int, int]) -> Dict:
    x, y, width, height = roi
    crop = image_bgr[y:y + height, x:x + width]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    mask = ((saturation >= COLOR_SAT_MIN) & (value >= COLOR_VAL_MIN)).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (RING_CLOSE_KERNEL, RING_CLOSE_KERNEL))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < RING_MIN_AREA or len(contour) < 20:
            continue
        points = contour.reshape(-1, 2).astype(np.float32)
        outer = _outer_edge_from_contour(points)
        if len(outer) < RING_MIN_COVERAGE_BINS:
            continue
        try:
            (cx, cy), (axis_w, axis_h), angle = cv2.fitEllipse(outer)
        except cv2.error:
            continue
        if min(axis_w, axis_h) < RING_MIN_AXIS_PX:
            continue
        if max(axis_w, axis_h) / min(axis_w, axis_h) > RING_MAX_AXIS_RATIO:
            continue
        residuals = _ellipse_residual(outer, np.array([cx, cy]), (axis_w, axis_h), angle)
        rms = float(np.sqrt(np.mean(residuals ** 2)))
        if not math.isfinite(rms) or rms > RING_MAX_RESIDUAL_RMS_PX:
            continue
        coverage = len(outer)
        score = rms + 0.05 * abs(max(axis_w, axis_h) / min(axis_w, axis_h) - 1.0)
        candidates.append({
            "center": np.array([cx + x, cy + y], dtype=float),
            "axes": (float(axis_w), float(axis_h)),
            "angle_deg": float(angle),
            "residual_rms_px": rms,
            "coverage_bins": int(coverage),
            "score": float(score),
            "outer_points": outer + np.array([x, y], dtype=np.float32),
        })

    if not candidates:
        return {"ok": False, "status": "no_dial", "reason": "no_dial",
                "detail": "no qualified humidity ring candidate"}
    candidates.sort(key=lambda item: item["score"])
    best = candidates[0]
    if len(candidates) > 1 and candidates[1]["score"] - best["score"] <= RING_AMBIGUITY_MARGIN:
        return {"ok": False, "status": "low_conf", "reason": "ambiguous_ring",
                "detail": "multiple similarly credible humidity ring candidates",
                "candidates": len(candidates)}
    matrix, major, minor, major_angle = _ellipse_matrix(best["axes"], best["angle_deg"])
    best.update({
        "ok": True,
        "matrix": matrix,
        "semi_major_px": major,
        "semi_minor_px": minor,
        "major_axis_angle_deg": major_angle,
    })
    return best


def _circular_fill_gaps(mask: np.ndarray, max_gap_samples: int) -> np.ndarray:
    result = mask.copy()
    count = len(mask)
    runs = []
    index = 0
    while index < count:
        if not result[index]:
            index += 1
            continue
        end = index
        while end < count and result[end]:
            end += 1
        runs.append((index, end))
        index = end
    if len(runs) < 2:
        return result
    for run_index in range(len(runs)):
        end = runs[run_index][1]
        next_start = runs[(run_index + 1) % len(runs)][0]
        if run_index == len(runs) - 1:
            gap = (next_start + count) - end
        else:
            gap = next_start - end
        if 0 < gap <= max_gap_samples:
            for offset in range(gap):
                result[(end + offset) % count] = True
    return result


def _longest_circular_run(mask: np.ndarray) -> Tuple[int, int]:
    count = len(mask)
    doubled = np.r_[mask, mask]
    best_start = 0
    best_length = 0
    index = 0
    while index < 2 * count:
        if not doubled[index]:
            index += 1
            continue
        end = index
        while end < 2 * count and doubled[end]:
            end += 1
        length = end - index
        if length > best_length and length <= count:
            best_start = index % count
            best_length = length
        index = end
    return best_start, best_length


def sample_blue(image_bgr: np.ndarray, ring: Dict) -> Dict:
    matrix = np.asarray(ring["matrix"], dtype=float)
    inverse = np.linalg.inv(matrix)
    center = np.asarray(ring["center"], dtype=float)
    samples = BLUE_ANGLE_SAMPLES
    step_deg = 360.0 / samples
    angles = np.linspace(0.0, 360.0, samples, endpoint=False)
    unit = np.stack([np.cos(np.radians(angles)),
                     np.sin(np.radians(angles))], axis=1) * BLUE_RHO
    points = center + unit @ inverse.T
    # Resample in BGR *before* converting to HSV.  OpenCV hue is a circular
    # 0..179 quantity, so bilinearly interpolating the hue plane averages red
    # pixels whose hues straddle 179/0 into a spurious ~90 "blue".  BGR is a
    # linear space, so interpolating it first keeps red and blue separate.
    map_x = points[:, 0].astype(np.float32).reshape(1, -1)
    map_y = points[:, 1].astype(np.float32).reshape(1, -1)
    sampled_bgr = cv2.remap(image_bgr, map_x, map_y, cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(sampled_bgr, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0].ravel()
    sat = hsv[:, :, 1].ravel()
    val = hsv[:, :, 2].ravel()
    blue = ((hue >= BLUE_HUE_MIN) & (hue <= BLUE_HUE_MAX)
            & (sat >= BLUE_SAT_MIN) & (val >= BLUE_VAL_MIN))
    if int(blue.sum()) < 5:
        return {"ok": False, "status": "low_conf", "reason": "blue_missing",
                "detail": "no blue samples at rho={}".format(BLUE_RHO)}
    max_gap = int(round(BLUE_GAP_TOL_DEG / step_deg))
    closed = _circular_fill_gaps(blue, max_gap)
    start, length = _longest_circular_run(closed)
    run_deg = length * step_deg
    if run_deg < BLUE_MIN_RUN_DEG:
        return {"ok": False, "status": "low_conf", "reason": "blue_too_short",
                "detail": "longest blue run is only {:.1f} deg".format(run_deg)}
    end = (start + length - 1) % samples
    boundary_angles = [angles[start], angles[end]]
    midpoint = _short_arc_midpoint(boundary_angles[0], boundary_angles[1])
    boundary_points = [
        center + inverse @ np.array([BLUE_RHO * math.cos(math.radians(a)),
                                     BLUE_RHO * math.sin(math.radians(a))])
        for a in boundary_angles
    ]
    return {
        "ok": True,
        "boundary_angles": [float(boundary_angles[0]), float(boundary_angles[1])],
        "boundary_points": boundary_points,
        "midpoint_deg": float(midpoint),
        "run_deg": float(run_deg),
        "blue_coverage": float(blue.mean()),
    }


def _short_arc_midpoint(first_deg: float, second_deg: float) -> float:
    first = wrap_deg(first_deg)
    second = wrap_deg(second_deg)
    delta = (second - first) % 360.0
    if delta > 180.0:
        delta -= 360.0
    return wrap_deg(first + delta / 2.0)


def _rotate_vector(vector: np.ndarray, angle_deg: float) -> np.ndarray:
    theta = math.radians(float(angle_deg))
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]], dtype=float)
    return rotation @ np.asarray(vector, dtype=float)


def _angle_from_axis(point: np.ndarray, axis: np.ndarray, matrix: np.ndarray) -> float:
    delta = np.asarray(point, dtype=float) - np.asarray(axis, dtype=float)
    if np.linalg.norm(delta) < 1e-6:
        raise ReadingConfigError("blue boundary coincides with migrated axis")
    vector = np.asarray(matrix, dtype=float) @ delta
    return wrap_deg(math.degrees(math.atan2(float(vector[1]), float(vector[0]))))


def reference_ring_center_blue_mid(calibration: Dict) -> float:
    """Reference blue-segment midpoint measured from the reference ring centre.

    The candidate stores the axis-centred midpoint (``blue_mid``) plus the two
    blue boundary points.  The first roll estimate must be relative to the ring
    centre, because ``sample_blue`` measures the runtime midpoint from the
    fitted ring centre.  Subtracting the axis-centred midpoint instead mixes two
    origins and injects the reference axis offset into the roll.
    """
    matrix = np.asarray(calibration["matrix"], dtype=float)
    center = np.asarray(calibration["ring_center"], dtype=float)
    boundaries = calibration.get("blue_boundaries")
    if not isinstance(boundaries, (list, tuple)) or len(boundaries) != 2:
        raise ReadingConfigError("calibration blue boundaries must contain two points")
    angles = []
    for point in boundaries:
        delta = np.asarray(point, dtype=float) - center
        if not np.all(np.isfinite(delta)) or np.linalg.norm(delta) < 1e-6:
            raise ReadingConfigError("blue boundary coincides with reference ring centre")
        vector = matrix @ delta
        angles.append(wrap_deg(math.degrees(math.atan2(float(vector[1]),
                                                       float(vector[0])))))
    return _short_arc_midpoint(angles[0], angles[1])


def migrate_axis(ring: Dict, blue: Dict, calibration: Dict) -> Dict:
    matrix = np.asarray(ring["matrix"], dtype=float)
    inverse = np.linalg.inv(matrix)
    center = np.asarray(ring["center"], dtype=float)
    d0 = np.asarray(calibration["matrix"], dtype=float) @ (
        np.asarray(calibration["axis"], dtype=float)
        - np.asarray(calibration["ring_center"], dtype=float))
    # Both roll estimates below must share one origin: the runtime midpoint is
    # measured from the fitted ring centre, therefore the reference midpoint
    # must be recomputed about the reference ring centre as well.  The
    # axis-centred ``calibration["blue_mid"]`` is only correct after migration.
    reference_mid = reference_ring_center_blue_mid(calibration)
    roll1 = wrap_deg(float(blue["midpoint_deg"]) - reference_mid)
    axis1 = center + inverse @ _rotate_vector(d0, roll1)
    first_angles = [_angle_from_axis(point, axis1, matrix)
                    for point in blue["boundary_points"]]
    mid1 = _short_arc_midpoint(first_angles[0], first_angles[1])
    roll2 = wrap_deg(mid1 - float(calibration["blue_mid"]))
    axis2 = center + inverse @ _rotate_vector(d0, roll2)
    delta = abs((roll1 - roll2 + 180.0) % 360.0 - 180.0)
    if delta > AXIS_ROLL_MAX_DELTA_DEG:
        return {"ok": False, "status": "low_conf", "reason": "axis_roll_inconsistent",
                "detail": "roll changed by {:.3f} deg after axis migration".format(delta),
                "d0": d0, "roll1_deg": roll1, "roll2_deg": roll2,
                "reference_ring_center_mid_deg": reference_mid}
    return {
        "ok": True,
        "d0": d0,
        "axis1": axis1,
        "axis2": axis2,
        "roll1_deg": float(roll1),
        "roll2_deg": float(roll2),
        "roll_delta_deg": float(delta),
        "reference_ring_center_mid_deg": reference_mid,
        "reference_axis_mid_deg": float(calibration["blue_mid"]),
    }


def _needle_direction_px(matrix: np.ndarray, angle_deg: float) -> np.ndarray:
    """Map a needle direction from the normalised dial plane back to pixels.

    ``detect_needle`` measures ``angle_deg`` after applying the ring matrix, so
    the angle lives in the normalised (circularised) plane.  Drawing it with
    plain ``cos``/``sin`` in pixel coordinates is wrong for an anisotropic
    ellipse; the direction must be carried back by ``inv(matrix)``.
    """
    theta = math.radians(float(angle_deg))
    unit = np.array([math.cos(theta), math.sin(theta)], dtype=float)
    direction = np.linalg.inv(np.asarray(matrix, dtype=float)) @ unit
    norm = float(np.linalg.norm(direction))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ReadingConfigError("cannot map needle direction back to pixels")
    return direction / norm


def detect_needle(image_bgr: np.ndarray, axis: np.ndarray, matrix: np.ndarray) -> Dict:
    inverse = np.linalg.inv(matrix)
    angles = np.linspace(0.0, 360.0, NEEDLE_ANGLE_SAMPLES, endpoint=False)
    radii = np.linspace(NEEDLE_RHO_MIN, NEEDLE_RHO_MAX, NEEDLE_RADIAL_POINTS)
    cos_a = np.cos(np.radians(angles))[:, None]
    sin_a = np.sin(np.radians(angles))[:, None]
    qx = radii[None, :] * cos_a
    qy = radii[None, :] * sin_a
    px = axis[0] + inverse[0, 0] * qx + inverse[0, 1] * qy
    py = axis[1] + inverse[1, 0] * qx + inverse[1, 1] * qy
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    values = cv2.remap(gray, px.astype(np.float32), py.astype(np.float32),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    dark = np.maximum(NEEDLE_DARK_BASE - values, 0.0)
    valid = dark > NEEDLE_DARK_MIN
    coverage = valid.mean(axis=1)
    score = dark.sum(axis=1)
    kernel_size = 101
    padded = np.r_[score[-kernel_size:], score, score[:kernel_size]]
    smooth = np.convolve(padded, np.ones(kernel_size) / kernel_size, mode='same')
    smooth = smooth[kernel_size:-kernel_size]
    high_pass = np.maximum(score - smooth, 0.0)
    high_pass[coverage < NEEDLE_COVERAGE_MIN] = 0.0
    if float(high_pass.max()) < NEEDLE_MIN_PEAK_SCORE:
        return {"ok": False, "status": "low_conf", "reason": "needle_weak",
                "detail": "needle score below threshold",
                "coverage": float(coverage.max()), "score": float(high_pass.max())}
    peaks = []
    for index in range(len(angles)):
        value = high_pass[index]
        if value <= 0.0:
            continue
        if value >= high_pass[index - 1] and value >= high_pass[(index + 1) % len(angles)]:
            peaks.append((float(value), index))
    peaks.sort(reverse=True)
    if not peaks:
        return {"ok": False, "status": "low_conf", "reason": "needle_weak",
                "detail": "no local peak"}
    main_score, main_index = peaks[0]
    separation = int(round(NEEDLE_PEAK_SEPARATION_DEG / 0.1))
    second_score = 0.0
    second_index = None
    for value, index in peaks[1:]:
        distance = min(abs(index - main_index),
                       NEEDLE_ANGLE_SAMPLES - abs(index - main_index))
        if distance >= separation:
            second_score, second_index = value, index
            break
    if second_index is not None and second_score >= main_score * NEEDLE_COMPETITION_RATIO:
        return {"ok": False, "status": "low_conf", "reason": "competing_peaks",
                "detail": "second needle peak is too close in strength",
                "coverage": float(coverage[main_index]),
                "score": float(main_score), "second_score": float(second_score),
                "peak_gap_deg": float(min(abs(second_index - main_index),
                                          NEEDLE_ANGLE_SAMPLES - abs(second_index - main_index)) * 0.1)}
    return {
        "ok": True,
        "angle_deg": float(angles[main_index]),
        "coverage": float(coverage[main_index]),
        "score": float(main_score),
        "second_score": float(second_score),
        "peak_gap_deg": None if second_index is None else float(
            min(abs(second_index - main_index),
                NEEDLE_ANGLE_SAMPLES - abs(second_index - main_index)) * 0.1),
    }


def _quality(ring: Dict, blue: Dict, needle: Dict, axis: Dict) -> Dict:
    return {
        "ring_residual_rms_px": round(float(ring.get("residual_rms_px", 0.0)), 4),
        "ring_coverage_bins": int(ring.get("coverage_bins", 0)),
        "ring_semi_major_px": round(float(ring.get("semi_major_px", 0.0)), 4),
        "ring_semi_minor_px": round(float(ring.get("semi_minor_px", 0.0)), 4),
        "blue_run_deg": round(float(blue.get("run_deg", 0.0)), 4),
        "blue_coverage": round(float(blue.get("blue_coverage", 0.0)), 6),
        "needle_coverage": round(float(needle.get("coverage", 0.0)), 6),
        "needle_score": round(float(needle.get("score", 0.0)), 6),
        "needle_second_score": round(float(needle.get("second_score", 0.0)), 6),
        "needle_peak_gap_deg": needle.get("peak_gap_deg"),
        "axis_roll_delta_deg": round(float(axis.get("roll_delta_deg", 0.0)), 6),
    }


def _json_safe(value):
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


def _base_result(calibration: Dict, roi: Tuple[int, int, int, int]) -> Dict:
    return {
        "status": "no_dial",
        "value": None,
        "unit": "%RH",
        "frame_status": "no_dial",
        "candidate_value": None,
        "angle_deg": None,
        "roll_deg": None,
        "frames_used": 1,
        "spread_deg": None,
        "reason": "no_dial",
        "calib_version": calibration.get("calib_version", ""),
        "calib_sha256": calibration.get("calib_sha256", ""),
        "roi": [int(v) for v in roi],
        "quality": {},
        "diagnostics": {},
    }


def estimate_frame(image_bgr: np.ndarray, calibration: Dict,
                   roi: Sequence[int]) -> Dict:
    """Pure specified-ROI single-frame estimate.  No file/thread/GUI access."""
    if not isinstance(image_bgr, np.ndarray) or image_bgr.ndim != 3:
        raise ReadingConfigError("image_bgr must be an HxWx3 array")
    roi_tuple = _roi_tuple(roi, image_bgr.shape)
    result = _base_result(calibration, roi_tuple)

    ring = detect_ring(image_bgr, roi_tuple)
    if not ring.get("ok"):
        result.update({
            "frame_status": ring.get("status", "no_dial"),
            "status": ring.get("status", "no_dial"),
            "reason": ring.get("reason", "no_dial"),
            "diagnostics": {"ring": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                     for k, v in ring.items()}},
        })
        return result

    blue = sample_blue(image_bgr, ring)
    if not blue.get("ok"):
        result.update({
            "frame_status": blue.get("status", "low_conf"),
            "status": blue.get("status", "low_conf"),
            "reason": blue.get("reason", "blue_missing"),
            "quality": _quality(ring, blue, {}, {}),
            "diagnostics": {"ring_center": ring["center"].tolist(),
                            "ring_axes": list(ring["axes"]),
                            "ring_angle_deg": float(ring["angle_deg"])},
        })
        return result

    migration = migrate_axis(ring, blue, calibration)
    if not migration.get("ok"):
        result.update({
            "frame_status": migration.get("status", "low_conf"),
            "status": migration.get("status", "low_conf"),
            "reason": migration.get("reason", "axis_roll_inconsistent"),
            "quality": _quality(ring, blue, {}, migration),
            "diagnostics": {
                "ring_center": ring["center"].tolist(),
                "ring_axes": list(ring["axes"]),
                "ring_angle_deg": float(ring["angle_deg"]),
                "blue_boundary_angles_deg": blue["boundary_angles"],
                "blue_boundary_points_px": [p.tolist() for p in blue["boundary_points"]],
                "blue_midpoint_deg_from_ring_center": blue["midpoint_deg"],
                "axis": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                         for k, v in migration.items()},
            },
        })
        return result

    needle = detect_needle(image_bgr, migration["axis2"], ring["matrix"])
    if not needle.get("ok"):
        result.update({
            "frame_status": needle.get("status", "low_conf"),
            "status": needle.get("status", "low_conf"),
            "reason": needle.get("reason", "needle_weak"),
            "quality": _quality(ring, blue, needle, migration),
            "diagnostics": {"axis": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                     for k, v in migration.items()},
                            "blue": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                     for k, v in blue.items()}},
        })
        return result

    compensated = wrap_deg(float(needle["angle_deg"]) - float(migration["roll2_deg"]))
    zero = float(calibration["table_deg"][0])
    unwrapped = zero + wrap_deg(compensated - zero)
    value = lookup_value(unwrapped, list(calibration["table_deg"]),
                         list(calibration["table_val"]))
    quality = _quality(ring, blue, needle, migration)
    direction_px = _needle_direction_px(ring["matrix"], needle["angle_deg"])
    semi = 0.5 * (float(ring["semi_major_px"]) + float(ring["semi_minor_px"]))
    diagnostics = {
        "ring_center": ring["center"].tolist(),
        "ring_axes": list(ring["axes"]),
        "ring_angle_deg": float(ring["angle_deg"]),
        "ring_matrix": np.asarray(ring["matrix"], dtype=float).tolist(),
        "blue_boundary_angles_deg": blue["boundary_angles"],
        "blue_boundary_points_px": [p.tolist() for p in blue["boundary_points"]],
        "blue_midpoint_deg_from_ring_center": blue["midpoint_deg"],
        "reference_ring_center_mid_deg": migration["reference_ring_center_mid_deg"],
        "reference_axis_mid_deg": migration["reference_axis_mid_deg"],
        "axis_d0_normalised": migration["d0"].tolist(),
        "axis_1_px": migration["axis1"].tolist(),
        "axis_2_px": migration["axis2"].tolist(),
        "roll_1_deg": migration["roll1_deg"],
        "roll_2_deg": migration["roll2_deg"],
        "roll_delta_deg": migration["roll_delta_deg"],
        "needle_raw_angle_deg": needle["angle_deg"],
        "needle_direction_px": direction_px.tolist(),
        "needle_draw_length_px": 0.80 * semi,
        "compensated_angle_deg": compensated,
        "unwrapped_angle_deg": unwrapped,
        "table_deg_first_last": [float(calibration["table_deg"][0]),
                                 float(calibration["table_deg"][-1])],
    }
    result.update({
        "angle_deg": float(unwrapped),
        "roll_deg": float(migration["roll2_deg"]),
        "quality": quality,
        "diagnostics": diagnostics,
    })
    if value is None:
        result.update({
            "status": "out_of_range",
            "frame_status": "out_of_range",
            "candidate_value": None,
            "reason": "out_of_range",
        })
    else:
        result.update({
            "status": "low_conf",
            "frame_status": "ok",
            "candidate_value": float(value),
            "reason": "multi_frame_required",
        })
    return result


def _output_target(output: Path) -> None:
    if output.is_symlink():
        raise ReadingConfigError("debug directory must not be a symlink")
    if output.exists():
        if not output.is_dir():
            raise ReadingConfigError("debug path exists and is not a directory")
        try:
            if any(output.iterdir()):
                raise ReadingConfigError("debug directory is not empty; use a new empty directory")
        except OSError as error:
            raise ReadingConfigError("cannot inspect debug directory: {}".format(error))


def _temp_path(output: Path, stem: str, suffix: str) -> Path:
    return output / ".{}-{}{}".format(stem, uuid.uuid4().hex, suffix)


def _safe_unlink(path: Path) -> None:
    try:
        path.unlink()
    except FileNotFoundError:
        pass
    except OSError:
        pass


def draw_debug_overlay(image_bgr: np.ndarray, result: Dict) -> np.ndarray:
    canvas = image_bgr.copy()
    x, y, width, height = result["roi"]
    cv2.rectangle(canvas, (x, y), (x + width, y + height), (0, 255, 255), 2)
    diagnostics = result.get("diagnostics", {})
    if "ring_center" in diagnostics:
        center = tuple(int(round(v)) for v in diagnostics["ring_center"])
        axes = diagnostics.get("ring_axes")
        if axes:
            cv2.ellipse(canvas, center,
                        (int(round(axes[0] / 2.0)), int(round(axes[1] / 2.0))),
                        float(diagnostics.get("ring_angle_deg", 0.0)),
                        0, 360, (0, 255, 0), 2)
        cv2.drawMarker(canvas, center, (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
    if "axis_2_px" in diagnostics:
        axis = tuple(int(round(v)) for v in diagnostics["axis_2_px"])
        cv2.drawMarker(canvas, axis, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
        direction = diagnostics.get("needle_direction_px")
        if direction is not None and "needle_raw_angle_deg" in diagnostics:
            length = float(diagnostics.get("needle_draw_length_px", 90.0))
            end = (int(round(axis[0] + length * float(direction[0]))),
                   int(round(axis[1] + length * float(direction[1]))))
            cv2.line(canvas, axis, end, (0, 0, 255), 2)
            label = "needle {:.1f}deg".format(float(diagnostics["needle_raw_angle_deg"]))
            cv2.putText(canvas, label, (end[0] + 5, end[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1, cv2.LINE_AA)
    for index, point in enumerate(diagnostics.get("blue_boundary_points_px", [])):
        p = (int(round(point[0])), int(round(point[1])))
        cv2.circle(canvas, p, 5, (255, 255, 0), -1)
        cv2.putText(canvas, "blue{}".format(index), (p[0] + 5, p[1] + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 0), 1, cv2.LINE_AA)
    lines = [
        "frame_status={}".format(result.get("frame_status")),
        "status={}".format(result.get("status")),
        "reason={}".format(result.get("reason")),
        "roll={}".format(None if result.get("roll_deg") is None
                         else round(float(result["roll_deg"]), 3)),
        "candidate_value={}".format(result.get("candidate_value")),
    ]
    y_text = 24
    for line in lines:
        cv2.putText(canvas, line, (12, y_text), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(canvas, line, (12, y_text), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1, cv2.LINE_AA)
        y_text += 22
    return canvas


def _write_debug(image_bgr: np.ndarray, result: Dict, output: Path) -> Dict:
    _output_target(output)
    created = False
    if not output.exists():
        output.mkdir(parents=True, exist_ok=False)
        created = True
    temporary: List[Path] = []
    published: List[Path] = []
    try:
        rendered = draw_debug_overlay(image_bgr, result)
        overlay_tmp = _temp_path(output, "debug_overlay", ".png")
        try:
            ok = cv2.imwrite(str(overlay_tmp), rendered,
                             [int(cv2.IMWRITE_PNG_COMPRESSION), 3])
        except Exception as error:
            _safe_unlink(overlay_tmp)
            raise ReadingConfigError("debug overlay write failed: {}".format(error))
        if not ok or not overlay_tmp.is_file() or overlay_tmp.stat().st_size <= 0:
            _safe_unlink(overlay_tmp)
            raise ReadingConfigError("debug overlay write failed")
        temporary.append(overlay_tmp)

        overlay_final = output / "debug_overlay.png"
        json_final = output / "frame_result.json"
        result["debug_overlay"] = str(overlay_final)
        result["frame_result_json"] = str(json_final)
        json_tmp = _temp_path(output, "frame_result", ".json")
        try:
            serialised = json.dumps(_json_safe(result), ensure_ascii=False, indent=2)
            json_tmp.write_text(serialised, encoding="utf-8")
        except (OSError, TypeError, ValueError) as error:
            _safe_unlink(json_tmp)
            raise ReadingConfigError("frame result write failed: {}".format(error))
        if not json_tmp.is_file() or json_tmp.stat().st_size <= 0:
            _safe_unlink(json_tmp)
            raise ReadingConfigError("frame result write produced an empty file")
        temporary.append(json_tmp)

        os.replace(str(overlay_tmp), str(overlay_final))
        temporary.remove(overlay_tmp)
        published.append(overlay_final)
        os.replace(str(json_tmp), str(json_final))
        temporary.remove(json_tmp)
        published.append(json_final)
        result["debug_overlay"] = str(overlay_final)
        result["frame_result_json"] = str(json_final)
    except (ReadingConfigError, OSError) as error:
        for path in temporary + published:
            _safe_unlink(path)
        if created:
            try:
                output.rmdir()
            except OSError:
                pass
        if isinstance(error, ReadingConfigError):
            raise
        raise ReadingConfigError("publishing debug output failed: {}".format(error))
    return result


def run_cli(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="G1b1 specified-ROI offline single-frame humidity reading")
    parser.add_argument("image")
    parser.add_argument("--calib", required=True)
    parser.add_argument("--allow-candidate", action="store_true")
    parser.add_argument("--roi", nargs=4, type=int, required=True,
                        metavar=("X", "Y", "W", "H"))
    parser.add_argument("--debug-dir", required=True)
    parser.add_argument("--calib-sha256", default=A08_CANDIDATE_SHA256,
                        help="expected candidate calibration SHA-256 (default: A08)")
    args = parser.parse_args(argv)

    try:
        calibration = load_calibration(args.calib, args.allow_candidate,
                                       args.calib_sha256)
        image_path = Path(args.image)
        if not image_path.is_file():
            raise ReadingConfigError("image not found: {}".format(image_path))
        image_bytes = image_path.read_bytes()
        image_sha = hashlib.sha256(image_bytes).hexdigest()
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ReadingConfigError("cannot decode image: {}".format(image_path))
        result = _json_safe(estimate_frame(image, calibration, args.roi))
        result["frame_id"] = 1
        result["frame_sha256"] = image_sha
        result["image_path"] = str(image_path)
        result = _write_debug(image, result, Path(args.debug_dir))
    except ReadingConfigError as error:
        print("reading failed: {}".format(error), file=sys.stderr)
        return 2
    except OSError as error:
        print("reading failed: {}".format(error), file=sys.stderr)
        return 2

    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(run_cli())
