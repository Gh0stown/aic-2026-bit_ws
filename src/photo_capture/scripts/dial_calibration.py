#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline candidate calibration from one annotated reference image (G1b0).

This tool does not implement the runtime dial detector.  It validates a
hand/semi-automatic annotation of the humidity scale, fits the visible
humidity colour-ring edge, computes a candidate angle table with the already
approved ``dial_math`` helpers, and writes a reviewable overlay.

Only NumPy, OpenCV, the standard library and ``dial_math`` are imported.  The
reference image is read-only; the output directory is local review output.
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


ANNOTATION_VERSION = "g1b0-annotations-1"
CANDIDATE_VERSION = "g1b0-candidate-1"
TOOL_VERSION = "dial_calibration.py 1"
TICK_VALUES = tuple(range(0, 101, 10))
MIN_RING_POINTS = 12
MAX_AXIS_RATIO = 5.0
MAX_RESIDUAL_RMS_PX = 5.0
MAX_RESIDUAL_PX = 10.0
# Numerical zero-vector guard in pixels.  This is a coordinate-degeneracy
# threshold, not an image-size / minimum-radius requirement.
MIN_VECTOR_NORM_PX = 1e-6


class CalibrationError(ValueError):
    """Invalid annotation, image or geometry for candidate calibration."""


def _finite_float(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CalibrationError("{} must be a finite real number".format(name))
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError):
        raise CalibrationError("{} must be a finite real number".format(name))
    if not math.isfinite(result):
        raise CalibrationError("{} must be finite".format(name))
    return result


def _point(raw, name: str) -> np.ndarray:
    if isinstance(raw, dict):
        raw = raw.get("point")
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise CalibrationError("{} must be a two-element point".format(name))
    return np.array([
        _finite_float(raw[0], name + "[0]"),
        _finite_float(raw[1], name + "[1]"),
    ], dtype=float)


def _in_bounds(point: np.ndarray, shape: Sequence[int]) -> bool:
    height, width = int(shape[0]), int(shape[1])
    return 0.0 <= point[0] <= width - 1.0 and 0.0 <= point[1] <= height - 1.0


def _require_nonzero(point: np.ndarray, axis: np.ndarray, name: str) -> None:
    delta = np.asarray(point, dtype=float) - np.asarray(axis, dtype=float)
    if not np.all(np.isfinite(delta)):
        raise CalibrationError("{} is not finite".format(name))
    if float(np.linalg.norm(delta)) < MIN_VECTOR_NORM_PX:
        raise CalibrationError(
            "{} coincides with the humidity axis (zero vector); angle is undefined".format(name))


def _unique_points(points: Sequence[np.ndarray], name: str) -> None:
    seen = set()
    for point in points:
        key = (round(float(point[0]), 6), round(float(point[1]), 6))
        if key in seen:
            raise CalibrationError("{} contains duplicate points".format(name))
        seen.add(key)


def load_annotations(path: str) -> Dict:
    annotation_path = Path(path)
    try:
        raw = annotation_path.read_bytes()
    except OSError as error:
        raise CalibrationError("cannot read annotations: {}".format(error))
    try:
        data = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CalibrationError("annotations are not valid UTF-8 JSON: {}".format(error))
    if not isinstance(data, dict):
        raise CalibrationError("annotations root must be a JSON object")
    return data


def validate_annotations(annotations: Dict, image_shape: Sequence[int]) -> Dict:
    """Validate schema, bounds, tick labels/order and ring distribution."""
    if not isinstance(annotations, dict):
        raise CalibrationError("annotations must be an object")
    if annotations.get("version") != ANNOTATION_VERSION:
        raise CalibrationError(
            "unsupported annotation version: {!r}".format(annotations.get("version")))
    model = annotations.get("model")
    if not isinstance(model, str) or not model.strip():
        raise CalibrationError("model must be a non-empty string")

    source = annotations.get("source_image")
    if not isinstance(source, dict):
        raise CalibrationError("source_image must be an object")
    expected_sha = source.get("sha256")
    if (not isinstance(expected_sha, str) or len(expected_sha) != 64
            or any(ch not in "0123456789abcdef" for ch in expected_sha.lower())):
        raise CalibrationError("source_image.sha256 must be 64 lowercase hex characters")
    expected_w = _finite_float(source.get("width"), "source_image.width")
    expected_h = _finite_float(source.get("height"), "source_image.height")
    if int(expected_w) != expected_w or int(expected_h) != expected_h:
        raise CalibrationError("source_image dimensions must be integers")
    if (int(expected_w), int(expected_h)) != (int(image_shape[1]), int(image_shape[0])):
        raise CalibrationError(
            "source_image dimensions {}x{} do not match loaded image {}x{}".format(
                int(expected_w), int(expected_h), int(image_shape[1]), int(image_shape[0])))

    axis = _point(annotations.get("humidity_axis"), "humidity_axis")
    if not _in_bounds(axis, image_shape):
        raise CalibrationError("humidity_axis is outside the image")

    raw_ticks = annotations.get("ticks")
    if not isinstance(raw_ticks, list) or len(raw_ticks) != len(TICK_VALUES):
        raise CalibrationError("ticks must contain exactly 11 entries")
    ticks = []
    for index, expected in enumerate(TICK_VALUES):
        entry = raw_ticks[index]
        if not isinstance(entry, dict):
            raise CalibrationError("ticks[{}] must be an object".format(index))
        value = _finite_float(entry.get("value"), "ticks[{}].value".format(index))
        if value != float(expected):
            raise CalibrationError(
                "ticks[{}].value must be {} in 0..100 order".format(index, expected))
        point = _point(entry.get("point"), "ticks[{}].point".format(index))
        if not _in_bounds(point, image_shape):
            raise CalibrationError("ticks[{}] is outside the image".format(index))
        uncertainty = _finite_float(
            entry.get("uncertainty_px", 1.0),
            "ticks[{}].uncertainty_px".format(index))
        if uncertainty < 0.0:
            raise CalibrationError("ticks[{}].uncertainty_px must be >= 0".format(index))
        ticks.append({
            "value": int(expected),
            "point": point,
            "uncertainty_px": uncertainty,
            "note": str(entry.get("note", "")),
        })
    _unique_points([entry["point"] for entry in ticks], "ticks")
    for tick in ticks:
        _require_nonzero(tick["point"], axis, "ticks[{}]".format(tick["value"]))

    blue = annotations.get("blue_segment")
    if not isinstance(blue, dict):
        raise CalibrationError("blue_segment must be an object")
    raw_boundaries = blue.get("boundaries")
    if not isinstance(raw_boundaries, list) or len(raw_boundaries) != 2:
        raise CalibrationError("blue_segment.boundaries must contain exactly two points")
    boundaries = []
    for index, entry in enumerate(raw_boundaries):
        point = _point(entry, "blue_segment.boundaries[{}]".format(index))
        if not _in_bounds(point, image_shape):
            raise CalibrationError(
                "blue_segment.boundaries[{}] is outside the image".format(index))
        boundaries.append({
            "name": str(entry.get("name", "boundary{}".format(index)))
            if isinstance(entry, dict) else "boundary{}".format(index),
            "point": point,
            "uncertainty_px": _finite_float(
                entry.get("uncertainty_px", 2.0) if isinstance(entry, dict) else 2.0,
                "blue_segment.boundaries[{}].uncertainty_px".format(index)),
        })
    if np.linalg.norm(boundaries[0]["point"] - boundaries[1]["point"]) < 1.0:
        raise CalibrationError("blue boundary points are identical/too close")
    for boundary in boundaries:
        _require_nonzero(boundary["point"], axis,
                         "blue_segment.{}".format(boundary["name"]))

    raw_ring = annotations.get("ring_points")
    if not isinstance(raw_ring, list) or len(raw_ring) < MIN_RING_POINTS:
        raise CalibrationError(
            "ring_points must contain at least {} points".format(MIN_RING_POINTS))
    ring = []
    for index, entry in enumerate(raw_ring):
        point = _point(entry, "ring_points[{}]".format(index))
        if not _in_bounds(point, image_shape):
            raise CalibrationError("ring_points[{}] is outside the image".format(index))
        ring.append(point)
    _unique_points(ring, "ring_points")

    centroid = np.mean(np.asarray(ring), axis=0)
    angles = np.degrees(np.arctan2(
        np.asarray(ring)[:, 1] - centroid[1],
        np.asarray(ring)[:, 0] - centroid[0])) % 360.0
    used_sectors = {int(angle // 30.0) for angle in angles}
    if len(used_sectors) < 6:
        raise CalibrationError("ring_points are not distributed around the ring")

    return {
        "model": model,
        "axis": axis,
        "ticks": ticks,
        "blue_boundaries": boundaries,
        "ring_points": ring,
        "source": source,
        "coordinate_convention": str(annotations.get("coordinate_convention", "")),
    }


def load_image(path: str, source: Dict) -> Tuple[np.ndarray, str]:
    image_path = Path(path)
    if not image_path.is_file():
        raise CalibrationError("image not found: {}".format(image_path))
    try:
        payload = image_path.read_bytes()
    except OSError as error:
        raise CalibrationError("cannot read image: {}".format(error))
    actual_sha = hashlib.sha256(payload).hexdigest()
    expected_sha = str(source.get("sha256", "")).lower()
    if actual_sha != expected_sha:
        raise CalibrationError(
            "image SHA-256 mismatch: expected {}, got {}".format(expected_sha, actual_sha))
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise CalibrationError("OpenCV cannot decode image: {}".format(image_path))
    height, width = image.shape[:2]
    expected_w = source.get("width")
    expected_h = source.get("height")
    if not isinstance(expected_w, (int, float)) or not isinstance(expected_h, (int, float)):
        raise CalibrationError("source_image.width/height are required")
    if (width, height) != (int(expected_w), int(expected_h)):
        raise CalibrationError(
            "image size {}x{} does not match annotations {}x{}".format(
                width, height, int(expected_w), int(expected_h)))
    return image, actual_sha


def _ellipse_residual_px(points: np.ndarray, center: np.ndarray,
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


def fit_ring_ellipse(points: Sequence[np.ndarray]) -> Dict:
    array = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    if len(array) < MIN_RING_POINTS:
        raise CalibrationError("not enough ring points")
    try:
        (cx, cy), (width, height), angle = cv2.fitEllipse(array)
    except cv2.error as error:
        raise CalibrationError("ellipse fit failed: {}".format(error))
    values = (cx, cy, width, height, angle)
    if not all(math.isfinite(float(value)) for value in values):
        raise CalibrationError("ellipse fit produced non-finite values")
    if min(width, height) <= 20.0:
        raise CalibrationError("fitted ring axes are too small: {:.2f}x{:.2f}".format(
            width, height))
    if max(width, height) / min(width, height) > MAX_AXIS_RATIO:
        raise CalibrationError("fitted ring is too eccentric: {:.2f}x{:.2f}".format(
            width, height))
    residuals = _ellipse_residual_px(
        np.asarray(points, dtype=float), np.array([cx, cy]), (width, height), angle)
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    maximum = float(np.max(residuals))
    if rms > MAX_RESIDUAL_RMS_PX or maximum > MAX_RESIDUAL_PX:
        raise CalibrationError(
            "ring fit residual too large: rms={:.2f}px max={:.2f}px".format(rms, maximum))
    return {
        "center": np.array([cx, cy], dtype=float),
        "axes": (float(width), float(height)),
        "angle_deg": float(angle),
        "residuals_px": residuals,
        "rms_px": rms,
        "max_px": maximum,
    }


def ellipse_matrix(center: np.ndarray, axes: Tuple[float, float],
                   angle_deg: float) -> Dict:
    width, height = float(axes[0]), float(axes[1])
    if width >= height:
        major, minor, major_angle = width / 2.0, height / 2.0, angle_deg
    else:
        major, minor, major_angle = height / 2.0, width / 2.0, angle_deg + 90.0
    if major <= 0.0 or minor <= 0.0:
        raise CalibrationError("ellipse semi-axes must be positive")
    theta = math.radians(float(major_angle))
    rotation = np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)],
    ], dtype=float)
    diagonal = np.diag([1.0 / major, 1.0 / minor])
    matrix = rotation @ diagonal @ rotation.T
    if not np.all(np.isfinite(matrix)):
        raise CalibrationError("ellipse normalisation matrix is not finite")
    return {
        "center": np.asarray(center, dtype=float),
        "matrix": matrix,
        "semi_major_px": float(major),
        "semi_minor_px": float(minor),
        "major_axis_angle_deg": float(major_angle % 180.0),
        "fit_axes_px": (width, height),
        "fit_angle_deg": float(angle_deg),
    }


def angle_from_axis(point: np.ndarray, axis: np.ndarray, matrix: np.ndarray) -> float:
    _require_nonzero(point, axis, "point")
    vector = np.asarray(matrix, dtype=float) @ (
        np.asarray(point, dtype=float) - np.asarray(axis, dtype=float))
    if not np.all(np.isfinite(vector)):
        raise CalibrationError("normalised point is not finite")
    return wrap_deg(math.degrees(math.atan2(float(vector[1]), float(vector[0]))))


def unwrap_angles(angles: Sequence[float]) -> List[float]:
    if not angles:
        raise CalibrationError("empty angle list")
    result = [float(angles[0])]
    for angle in angles[1:]:
        value = float(angle)
        while value <= result[-1]:
            value += 360.0
        result.append(value)
    return result


def short_arc_midpoint(first_deg: float, second_deg: float) -> float:
    first = wrap_deg(first_deg)
    second = wrap_deg(second_deg)
    delta = (second - first) % 360.0
    if delta > 180.0:
        delta -= 360.0
    return wrap_deg(first + delta / 2.0)


def build_table(tick_points: Sequence[np.ndarray], axis: np.ndarray,
                matrix: np.ndarray) -> Tuple[List[float], List[float]]:
    raw = [angle_from_axis(point, axis, matrix) for point in tick_points]
    table = unwrap_angles(raw)
    try:
        validate_scale(table, list(TICK_VALUES))
    except ValueError as error:
        raise CalibrationError("candidate tick table is invalid: {}".format(error))
    return raw, table


def _angle_delta(first: float, second: float) -> float:
    return abs((float(first) - float(second) + 180.0) % 360.0 - 180.0)


def _nearest_unwrapped(angle: float, reference: float) -> float:
    return float(angle) + 360.0 * round((float(reference) - float(angle)) / 360.0)


def compute_sensitivity(ticks: Sequence[Dict], axis: np.ndarray,
                        matrix: np.ndarray, raw_angles: Sequence[float],
                        table_deg: Sequence[float],
                        blue_boundaries: Sequence[Dict],
                        blue_angles: Sequence[float]) -> Dict:
    offsets = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
    tick_reports = []
    max_tick_angle = 0.0
    wet_tick_value = 0.0
    tick_interpolable = 0
    tick_out_of_range = 0
    for index, tick in enumerate(ticks):
        max_angle = 0.0
        max_value = 0.0
        for dx, dy in offsets:
            moved = tick["point"] + np.array([dx, dy], dtype=float)
            moved_angle = angle_from_axis(moved, axis, matrix)
            max_angle = max(max_angle, _angle_delta(moved_angle, raw_angles[index]))
            unwrapped = _nearest_unwrapped(moved_angle, table_deg[index])
            value = lookup_value(unwrapped, table_deg, list(TICK_VALUES))
            if value is None:
                tick_out_of_range += 1
            else:
                tick_interpolable += 1
                max_value = max(max_value, abs(float(value) - tick["value"]))
        max_tick_angle = max(max_tick_angle, max_angle)
        if tick["value"] >= 80:
            wet_tick_value = max(wet_tick_value, max_value)
        tick_reports.append({
            "value": tick["value"],
            "max_angle_delta_deg": round(max_angle, 6),
            "max_value_delta_per_px": round(max_value, 6),
        })

    max_axis_angle = 0.0
    max_axis_wet_value = 0.0
    max_axis_blue_mid = 0.0
    axis_interpolable = 0
    axis_out_of_range = 0
    wet_angle = (float(table_deg[-2]) + float(table_deg[-1])) / 2.0
    original_wet = lookup_value(wet_angle, table_deg, list(TICK_VALUES))
    original_wet = 95.0 if original_wet is None else float(original_wet)
    for dx, dy in offsets:
        moved_axis = axis + np.array([dx, dy], dtype=float)
        moved_raw, moved_table = build_table([tick["point"] for tick in ticks],
                                             moved_axis, matrix)
        for before, after in zip(raw_angles, moved_raw):
            max_axis_angle = max(max_axis_angle, _angle_delta(before, after))
        moved_value = lookup_value(wet_angle, moved_table, list(TICK_VALUES))
        if moved_value is None:
            axis_out_of_range += 1
        else:
            axis_interpolable += 1
            max_axis_wet_value = max(
                max_axis_wet_value, abs(float(moved_value) - original_wet))
        moved_blue_angles = [
            angle_from_axis(boundary["point"], moved_axis, matrix)
            for boundary in blue_boundaries
        ]
        moved_mid = short_arc_midpoint(*moved_blue_angles)
        original_mid = short_arc_midpoint(*blue_angles)
        max_axis_blue_mid = max(
            max_axis_blue_mid, _angle_delta(moved_mid, original_mid))

    max_blue_angle = 0.0
    max_blue_mid = 0.0
    for index, boundary in enumerate(blue_boundaries):
        for dx, dy in offsets:
            moved = boundary["point"] + np.array([dx, dy], dtype=float)
            moved_angle = angle_from_axis(moved, axis, matrix)
            max_blue_angle = max(
                max_blue_angle, _angle_delta(moved_angle, blue_angles[index]))
            other = blue_angles[1 - index]
            moved_mid = short_arc_midpoint(moved_angle, other)
            original_mid = short_arc_midpoint(*blue_angles)
            max_blue_mid = max(
                max_blue_mid, _angle_delta(moved_mid, original_mid))

    return {
        "method": "one-pixel cardinal perturbation of each annotated tick/axis/blue boundary; ring refit not repeated",
        "max_tick_angle_delta_deg": round(max_tick_angle, 6),
        "max_axis_angle_delta_deg": round(max_axis_angle, 6),
        "max_blue_boundary_angle_delta_deg": round(max_blue_angle, 6),
        "max_blue_midpoint_delta_deg": round(max_blue_mid, 6),
        "wet_end_value_delta_per_px": round(max(wet_tick_value, max_axis_wet_value), 6),
        "interpolable_tick_perturbations": tick_interpolable,
        "out_of_range_tick_perturbations": tick_out_of_range,
        "interpolable_axis_perturbations": axis_interpolable,
        "out_of_range_axis_perturbations": axis_out_of_range,
        "value_delta_scope": "max value delta only over perturbations whose lookup_value is not None; out-of-range perturbations are counted, not treated as zero error",
        "ticks": tick_reports,
        "notes": [
            "diagnostic only; not an accuracy measurement",
            "axis perturbation rebuilds the candidate table; tick perturbation uses the original table",
        ],
    }


def _decode_rgb(image: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def _render_overlay(image: np.ndarray, candidate: Dict) -> np.ndarray:
    canvas = image.copy()
    height, width = canvas.shape[:2]
    ellipse = candidate["ring"]
    center = (int(round(float(ellipse["center_px"][0]))),
              int(round(float(ellipse["center_px"][1]))))
    # cv2.ellipse expects half-axes (semi-axes); fit_axes_px stores full diameters.
    axes = tuple(int(round(float(v) / 2.0)) for v in ellipse["fit_axes_px"])
    angle = float(ellipse["fit_ellipse_angle_deg"])
    cv2.ellipse(canvas, center, axes, angle, 0, 360, (0, 255, 0), 2)

    for point in candidate["ring"]["points_px"]:
        cv2.circle(canvas, (int(round(point[0])), int(round(point[1]))),
                   2, (0, 255, 255), -1)
    axis = tuple(float(v) for v in candidate["humidity_axis_px"])
    ring_center = center
    cv2.drawMarker(canvas, (int(round(axis[0])), int(round(axis[1]))),
                   (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
    cv2.drawMarker(canvas, (int(round(ring_center[0])), int(round(ring_center[1]))),
                   (255, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 1)

    for tick in candidate["ticks"]:
        point = tuple(float(v) for v in tick["point_px"])
        cv2.line(canvas, (int(round(axis[0])), int(round(axis[1]))),
                 (int(round(point[0])), int(round(point[1]))), (0, 0, 255), 1)
        cv2.circle(canvas, (int(round(point[0])), int(round(point[1]))),
                   4, (0, 0, 255), -1)
        cv2.putText(canvas, str(tick["value"]),
                    (int(round(point[0])) + 7, int(round(point[1])) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

    blue = candidate["blue_segment"]
    for boundary in blue["boundaries"]:
        point = tuple(float(v) for v in boundary["point_px"])
        cv2.line(canvas, (int(round(axis[0])), int(round(axis[1]))),
                 (int(round(point[0])), int(round(point[1]))), (255, 255, 0), 1)
        cv2.circle(canvas, (int(round(point[0])), int(round(point[1]))),
                   6, (255, 255, 0), -1)
        cv2.putText(canvas, boundary["name"],
                    (int(round(point[0])) + 8, int(round(point[1])) + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)

    title = "G1b0 candidate | {} | {}x{} | sha256 {}".format(
        candidate["model"], width, height, candidate["source_image"]["sha256"][:12])
    cv2.rectangle(canvas, (0, 0), (width, 42), (0, 0, 0), -1)
    cv2.putText(canvas, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (255, 255, 255), 1, cv2.LINE_AA)

    panel_size = 700
    panel = np.full((panel_size, panel_size, 3), 255, dtype=np.uint8)
    pcx = pcy = panel_size // 2
    scale = 240.0
    cv2.circle(panel, (pcx, pcy), int(scale), (0, 0, 0), 1)
    cv2.line(panel, (pcx - int(scale), pcy), (pcx + int(scale), pcy), (190, 190, 190), 1)
    cv2.line(panel, (pcx, pcy - int(scale)), (pcx, pcy + int(scale)), (190, 190, 190), 1)
    matrix = np.asarray(candidate["ring"]["matrix"], dtype=float)
    axis_vec = np.asarray(candidate["humidity_axis_px"], dtype=float)

    def norm_to_px(point: np.ndarray) -> Tuple[int, int]:
        vector = matrix @ (np.asarray(point, dtype=float) - axis_vec)
        return (int(round(pcx + scale * float(vector[0]))),
                int(round(pcy + scale * float(vector[1]))))

    for point in candidate["ring"]["points_px"]:
        cv2.circle(panel, norm_to_px(np.asarray(point)), 2, (0, 140, 255), -1)
    for tick in candidate["ticks"]:
        px = norm_to_px(np.asarray(tick["point_px"]))
        cv2.circle(panel, px, 3, (0, 0, 255), -1)
        cv2.putText(panel, str(tick["value"]), (px[0] + 4, px[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
    for boundary in blue["boundaries"]:
        px = norm_to_px(np.asarray(boundary["point_px"]))
        cv2.circle(panel, px, 5, (255, 255, 0), -1)
        cv2.putText(panel, boundary["name"], (px[0] + 5, px[1] + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 0), 1, cv2.LINE_AA)
    cv2.drawMarker(panel, (pcx, pcy), (0, 160, 0), cv2.MARKER_CROSS, 16, 2)
    ring_norm = matrix @ (np.asarray(ellipse["center_px"], dtype=float) - axis_vec)
    cv2.circle(panel, (int(round(pcx + scale * ring_norm[0])),
                       int(round(pcy + scale * ring_norm[1]))), 4, (180, 0, 180), -1)
    cv2.putText(panel, "normalised A*(p-axis); circle = unit",
                (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

    output = np.full((max(height, panel_size), width + panel_size, 3), 24, dtype=np.uint8)
    output[0:height, 0:width] = canvas
    output[0:panel_size, width:width + panel_size] = panel
    return output


def _to_list(array) -> List:
    return [round(float(value), 6) for value in np.asarray(array).reshape(-1)]


def _canonical(path: Path) -> Path:
    try:
        return path.resolve()
    except OSError:
        return path.absolute()


def _paths_alias(first: Path, second: Path) -> bool:
    try:
        if first.exists() and second.exists() and first.samefile(second):
            return True
    except OSError:
        pass
    return _canonical(first) == _canonical(second)


def _check_output_target(output: Path, image_path: str, annotations_path: str) -> None:
    """Reject missing/unsafe output targets without creating or deleting anything."""
    if output.is_symlink():
        raise CalibrationError("output directory must not be a symlink")
    if output.exists():
        if not output.is_dir():
            raise CalibrationError("output path exists and is not a directory")
        try:
            if any(output.iterdir()):
                raise CalibrationError(
                    "output directory is not empty; use a new empty directory")
        except OSError as error:
            raise CalibrationError("cannot inspect output directory: {}".format(error))

    for label, raw_path in (("image", image_path), ("annotations", annotations_path)):
        input_path = Path(raw_path)
        if _paths_alias(input_path, output):
            raise CalibrationError(
                "{} path aliases the output directory/file".format(label))
        try:
            input_canonical = _canonical(input_path)
            output_canonical = _canonical(output)
            relative = input_canonical.relative_to(output_canonical)
        except ValueError:
            relative = None
        except OSError:
            relative = None
        if relative is not None:
            raise CalibrationError(
                "{} is inside the output directory".format(label))


def _temp_path(output: Path, stem: str, suffix: str) -> Path:
    return output / ".{}-{}{}".format(stem, uuid.uuid4().hex, suffix)


def _safe_unlink(path: Path) -> None:
    try:
        path.unlink()
    except FileNotFoundError:
        pass
    except OSError:
        pass


def _require_written_file(path: Path, message: str) -> None:
    try:
        size = path.stat().st_size
    except OSError as error:
        _safe_unlink(path)
        raise CalibrationError("{}: {}".format(message, error))
    if size <= 0:
        _safe_unlink(path)
        raise CalibrationError("{} produced an empty file".format(message))


def _write_overlay_temp(image: np.ndarray, candidate: Dict, output: Path) -> Path:
    rendered = _render_overlay(image, candidate)
    path = _temp_path(output, "calibration_overlay", ".png")
    try:
        ok = cv2.imwrite(str(path), rendered, [int(cv2.IMWRITE_PNG_COMPRESSION), 3])
    except Exception as error:  # cv2.error and filesystem errors
        _safe_unlink(path)
        raise CalibrationError("overlay write failed: {}".format(error))
    if not ok or not path.is_file():
        _safe_unlink(path)
        raise CalibrationError("overlay write failed (cv2.imwrite returned false/empty)")
    _require_written_file(path, "overlay write")
    return path


def _write_json_temp(candidate: Dict, output: Path) -> Path:
    try:
        text = json.dumps(candidate, ensure_ascii=False, indent=2)
    except (TypeError, ValueError) as error:
        raise CalibrationError("candidate JSON serialisation failed: {}".format(error))
    path = _temp_path(output, "candidate_calib", ".json")
    try:
        path.write_text(text, encoding="utf-8")
    except OSError as error:
        _safe_unlink(path)
        raise CalibrationError("candidate JSON write failed: {}".format(error))
    if not path.is_file():
        _safe_unlink(path)
        raise CalibrationError("candidate JSON write produced no file")
    _require_written_file(path, "candidate JSON write")
    return path


def _cleanup_outputs(paths: Sequence[Path], created_dir: bool, output: Path) -> None:
    for path in paths:
        _safe_unlink(path)
    if created_dir:
        try:
            output.rmdir()
        except OSError:
            pass


def run(image_path: str, annotations_path: str, output_dir: str) -> Dict:
    output = Path(output_dir)
    _check_output_target(output, image_path, annotations_path)

    annotations = load_annotations(annotations_path)
    annotation_bytes = Path(annotations_path).read_bytes()
    annotation_sha = hashlib.sha256(annotation_bytes).hexdigest()
    image, image_sha = load_image(image_path, annotations.get("source_image", {}))
    data = validate_annotations(annotations, image.shape)

    ellipse = fit_ring_ellipse(data["ring_points"])
    geometry = ellipse_matrix(ellipse["center"], ellipse["axes"], ellipse["angle_deg"])
    raw_angles, table_deg = build_table(
        [tick["point"] for tick in data["ticks"]], data["axis"], geometry["matrix"])
    blue_angles = [
        angle_from_axis(boundary["point"], data["axis"], geometry["matrix"])
        for boundary in data["blue_boundaries"]
    ]
    blue_mid = short_arc_midpoint(*blue_angles)
    ring_center = ellipse["center"]
    axis_offset_px = data["axis"] - ring_center
    axis_offset_norm = geometry["matrix"] @ axis_offset_px
    if not math.isfinite(blue_mid):
        raise CalibrationError("blue segment midpoint is not finite")

    sensitivity = compute_sensitivity(
        data["ticks"], data["axis"], geometry["matrix"], raw_angles,
        table_deg, data["blue_boundaries"], blue_angles)

    suspicious = []
    for index, residual in enumerate(ellipse["residuals_px"]):
        if float(residual) > 2.0:
            suspicious.append({
                "kind": "ring_point_residual",
                "index": index,
                "point_px": _to_list(data["ring_points"][index]),
                "residual_px": round(float(residual), 4),
            })

    warnings = []
    if max(ellipse["axes"]) / min(ellipse["axes"]) < 1.03:
        warnings.append(
            "near-circular ellipse: major/minor ratio {:.4f}; orientation is not used as roll reference".format(
                max(ellipse["axes"]) / min(ellipse["axes"])))
    if suspicious:
        warnings.append("ring points with residual > 2 px: {}".format(
            ", ".join(str(item["index"]) for item in suspicious)))
    warnings.append(
        "reference humidity needle is in the blank/out-of-range area; not used as a value")

    candidate = {
        "status": "candidate",
        "calib_version": CANDIDATE_VERSION,
        "tool_version": TOOL_VERSION,
        "annotation_version": annotations.get("version"),
        "model": data["model"],
        "source_image": {
            "path": str(annotations["source_image"].get("path", image_path)),
            "sha256": image_sha,
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
        },
        "annotation_path": str(Path(annotations_path)),
        "annotation_sha256": annotation_sha,
        "coordinate_convention": data["coordinate_convention"],
        "humidity_axis_px": _to_list(data["axis"]),
        "ring": {
            "edge": "outer edge of visible humidity colour band",
            "center_px": _to_list(ellipse["center"]),
            "fit_axes_px": [round(float(v), 6) for v in ellipse["axes"]],
            "fit_ellipse_angle_deg": round(ellipse["angle_deg"], 6),
            "semi_major_px": round(geometry["semi_major_px"], 6),
            "semi_minor_px": round(geometry["semi_minor_px"], 6),
            "major_axis_angle_deg": round(geometry["major_axis_angle_deg"], 6),
            "matrix": [[round(float(v), 12) for v in row] for row in geometry["matrix"]],
            "points_px": [_to_list(point) for point in data["ring_points"]],
            "residual_px": [round(float(v), 4) for v in ellipse["residuals_px"]],
            "residual_rms_px": round(ellipse["rms_px"], 4),
            "residual_max_px": round(ellipse["max_px"], 4),
        },
        "axis_offset": {
            "raw_px": _to_list(axis_offset_px),
            "normalised": _to_list(axis_offset_norm),
            "magnitude_normalised": round(float(np.linalg.norm(axis_offset_norm)), 6),
        },
        "blue_segment": {
            "boundaries": [{
                "name": boundary["name"],
                "point_px": _to_list(boundary["point"]),
                "angle_deg": round(angle, 6),
            } for boundary, angle in zip(data["blue_boundaries"], blue_angles)],
            "midpoint_deg": round(blue_mid, 6),
            "short_arc_deg": round(_angle_delta(blue_angles[0], blue_angles[1]), 6),
            "roll_ref_deg": round(blue_mid, 6),
        },
        "ticks": [{
            "value": tick["value"],
            "point_px": _to_list(tick["point"]),
            "raw_angle_deg": round(raw, 6),
            "angle_deg": round(unwrapped, 6),
            "uncertainty_px": tick["uncertainty_px"],
            "note": tick["note"],
        } for tick, raw, unwrapped in zip(data["ticks"], raw_angles, table_deg)],
        "table_deg": [round(float(v), 6) for v in table_deg],
        "table_val": [float(v) for v in TICK_VALUES],
        "sampling": {
            "colour_rho": 0.94,
            "colour_rho_source": "D02 attachment initial value; not derived from this image",
            "needle_window": [0.14, 0.70],
            "needle_window_source": "D02 attachment initial value",
            "angle_samples": 3600,
            "angle_samples_source": "D02 attachment initial value",
            "roll_ref_deg": round(blue_mid, 6),
            "roll_ref_source": "measured blue-segment midpoint over the short arc",
        },
        "sensitivity": sensitivity,
        "suspicious_points": suspicious,
        "warnings": warnings,
        "annotations": annotations,
    }

    _check_output_target(output, image_path, annotations_path)
    created_dir = False
    if not output.exists():
        output.mkdir(parents=True, exist_ok=False)
        created_dir = True
    elif not output.is_dir() or any(output.iterdir()):
        raise CalibrationError("output directory changed; use a new empty directory")

    temporary = []
    published = []
    try:
        overlay_tmp = _write_overlay_temp(image, candidate, output)
        temporary.append(overlay_tmp)
        json_tmp = _write_json_temp(candidate, output)
        temporary.append(json_tmp)

        overlay_final = output / "calibration_overlay.png"
        json_final = output / "candidate_calib.json"
        os.replace(str(overlay_tmp), str(overlay_final))
        temporary.remove(overlay_tmp)
        published.append(overlay_final)
        os.replace(str(json_tmp), str(json_final))
        temporary.remove(json_tmp)
        published.append(json_final)
    except (CalibrationError, OSError) as error:
        _cleanup_outputs(temporary + published, created_dir, output)
        if isinstance(error, CalibrationError):
            raise
        raise CalibrationError("publishing candidate outputs failed: {}".format(error))
    return candidate


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="G1b0 offline candidate calibration from one annotated image")
    parser.add_argument("--image", required=True, help="reference image (read-only)")
    parser.add_argument("--annotations", required=True, help="annotation JSON")
    parser.add_argument("--output-dir", required=True, help="local review output directory")
    args = parser.parse_args(argv)
    try:
        candidate = run(args.image, args.annotations, args.output_dir)
    except CalibrationError as error:
        print("calibration failed: {}".format(error), file=sys.stderr)
        return 2
    except OSError as error:
        print("calibration failed: {}".format(error), file=sys.stderr)
        return 2
    print("candidate written: {}".format(
        Path(args.output_dir) / "candidate_calib.json"))
    print("table_deg: {}".format(candidate["table_deg"]))
    print("roll_ref_deg: {}".format(candidate["blue_segment"]["roll_ref_deg"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
