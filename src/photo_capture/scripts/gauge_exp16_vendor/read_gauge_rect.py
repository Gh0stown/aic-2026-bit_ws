"""Exp16 temperature reader: black hub, rotated tick rectangles and gap-first reading."""

from __future__ import annotations

import math

import cv2
import numpy as np

from . import CropReading, UnreadableGauge
from .analyze_gauge_opencv import outer_dial_geometry, pointer_angle
from .tick_rect_tuning import detect_tick_rectangles


RECT14 = {
    "block": 41,
    "c": 8.0,
    "clip": 2.0,
    "inner": 0.86,
    "outer": 0.96,
    "min_length": 10.0,
    "max_length": 30.0,
    "min_width": 1.5,
    "max_width": 8.0,
    "min_aspect": 2.5,
    "max_aspect": 9.0,
    "alignment": 10.0,
    "min_area": 12,
}


def black_center_candidates(
    image: np.ndarray,
    expected_center: tuple[int, int],
) -> list[tuple[int, int, int]]:
    height, width = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        1.1,
        minDist=max(12, int(min(width, height) * 0.035)),
        param1=80,
        param2=15,
        minRadius=max(6, int(min(width, height) * 0.012)),
        maxRadius=max(12, min(22, int(min(width, height) * 0.055))),
    )
    if circles is None:
        raise UnreadableGauge("未检测到中心黑色圆轴")

    ex, ey = expected_center
    candidates = []
    for x, y, radius in circles[0]:
        if not (0.30 * width < x < 0.72 * width and 0.50 * height < y < 0.90 * height):
            continue
        distance_prior = math.hypot(x - ex, y - ey) / min(width, height)
        radius_prior = abs(radius - 16.0) / 8.0
        score = -5.0 * distance_prior - 0.10 * radius_prior
        candidates.append((score, x, y, radius))
    if not candidates:
        raise UnreadableGauge("中心圆候选不在预期区域")
    candidates.sort(reverse=True)
    return [
        (int(round(x)), int(round(y)), int(round(radius)))
        for _, x, y, radius in candidates
    ]


def ordered_ticks(
    detections: list[dict[str, object]],
    min_value: float,
    max_value: float,
) -> tuple[list[dict[str, object]], float, float, float]:
    if len(detections) < 2:
        raise UnreadableGauge("有效刻度不足")
    ordered = sorted(detections, key=lambda item: float(item["angle_deg"]))
    gaps = []
    for index, detection in enumerate(ordered):
        following = ordered[(index + 1) % len(ordered)]
        gap = (float(following["angle_deg"]) - float(detection["angle_deg"])) % 360.0
        gaps.append((gap, float(detection["angle_deg"]), float(following["angle_deg"])))
    _, before_gap, after_gap = max(gaps)
    by_angle = {round(float(item["angle_deg"]), 6): item for item in detections}
    endpoint_a = by_angle[round(before_gap, 6)]
    endpoint_b = by_angle[round(after_gap, 6)]
    if float(endpoint_a["center"][0]) <= float(endpoint_b["center"][0]):
        start_angle = before_gap
        end_angle = after_gap
    else:
        start_angle = after_gap
        end_angle = before_gap
    sweep = (start_angle - end_angle) % 360.0
    if not sweep:
        raise UnreadableGauge("invalid temperature scale sweep")

    result = []
    for detection in detections:
        position = (start_angle - float(detection["angle_deg"])) % 360.0
        if position <= sweep + 2.0:
            item = dict(detection)
            item["position_deg"] = position
            item["value"] = min_value + position / sweep * (max_value - min_value)
            result.append(item)
    result.sort(key=lambda item: float(item["position_deg"]))
    for index, item in enumerate(result, start=1):
        item["order"] = index
        item["value"] = min_value + (index - 1) / max(1, len(result) - 1) * (
            max_value - min_value
        )
    return result, start_angle, end_angle, sweep


def read_pointer_by_tick_order(
    ticks: list[dict[str, object]],
    pointer_angle: float,
) -> tuple[float | None, dict[str, object] | None, list[dict[str, object]]]:
    if not ticks:
        return None, None, ticks
    nearest = min(
        ticks,
        key=lambda tick: min(
            abs(float(tick["angle_deg"]) - pointer_angle),
            360.0 - abs(float(tick["angle_deg"]) - pointer_angle),
        ),
    )
    angle_distance = min(
        abs(float(nearest["angle_deg"]) - pointer_angle),
        360.0 - abs(float(nearest["angle_deg"]) - pointer_angle),
    )
    if angle_distance <= 6.0:
        remaining = [tick for tick in ticks if tick is not nearest]
        return float(nearest["value"]), nearest, remaining
    return None, nearest, ticks


def pointer_gap(
    ticks: list[dict[str, object]],
    start_angle: float,
    sweep: float,
    pointer_angle: float,
    min_value: float,
    max_value: float,
) -> dict[str, float] | None:
    if len(ticks) < 3:
        return None
    positions = [float(tick["position_deg"]) for tick in ticks]
    gaps = []
    for index in range(len(ticks) - 1):
        gap = positions[index + 1] - positions[index]
        midpoint = (positions[index] + positions[index + 1]) / 2.0
        angle = (start_angle - midpoint) % 360.0
        pointer_distance = min(abs(angle - pointer_angle), 360.0 - abs(angle - pointer_angle))
        gaps.append((gap, pointer_distance, midpoint, angle, index))
    median_gap = float(np.median([item[0] for item in gaps]))
    candidates = [item for item in gaps if item[0] > median_gap * 1.55]
    if not candidates:
        return None
    gap, pointer_distance, midpoint, angle, index = min(candidates, key=lambda item: item[1])
    if pointer_distance > 12.0:
        return None
    before_value = float(ticks[index]["value"])
    after_value = float(ticks[index + 1]["value"])
    value = (before_value + after_value) / 2.0
    return {
        "gap_deg": gap,
        "median_gap_deg": median_gap,
        "missing_angle_deg": angle,
        "position_deg": midpoint,
        "value": value,
        "before_value": before_value,
        "after_value": after_value,
    }


TEMP_MIN, TEMP_MAX = -29.0, 49.0
TARGET_TICKS = int(TEMP_MAX - TEMP_MIN + 1)


def read_temperature(image: np.ndarray) -> CropReading:
    expected, radius = outer_dial_geometry(image, 0.49, 0.67, 0.60)
    best = None
    for x, y, hub_radius in black_center_candidates(image, expected):
        center = (x, y)
        binary, detections = detect_tick_rectangles(image, center, radius, RECT14)
        distance = math.hypot(x - expected[0], y - expected[1])
        score = -abs(len(detections) - TARGET_TICKS) - 0.25 * distance
        # The remaining key components match the handoff's tie breaking.
        key = (score, x, y, hub_radius, len(detections))
        if best is None or key > best[0]:
            best = key, center, binary, detections
    _, center, binary, detections = best

    # Rescaling a few remaining ticks to the full 79-position scale would
    # generate plausible but unsupported temperatures. Allow up to 25% loss.
    if len(detections) < math.ceil(TARGET_TICKS * 0.75):
        raise UnreadableGauge("too few temperature ticks ({})".format(len(detections)))
    ticks, start, _end, sweep = ordered_ticks(detections, TEMP_MIN, TEMP_MAX)
    if len(ticks) < math.ceil(TARGET_TICKS * 0.75):
        raise UnreadableGauge("incomplete temperature scale")

    angle = pointer_angle(image, binary, center, radius)
    gap = pointer_gap(ticks, start, sweep, angle, TEMP_MIN, TEMP_MAX)
    if gap is not None:
        value, method = gap["value"], "tick-gap"
    else:
        value, _, _ = read_pointer_by_tick_order(ticks, angle)
        if value is not None:
            value, method = float(round(value)), "tick-order"
        else:
            position = (start - angle) % 360.0
            if position > sweep + 3.0:
                raise UnreadableGauge("temperature pointer outside scale")
            value = TEMP_MIN + position / sweep * (TEMP_MAX - TEMP_MIN)
            method = "pointer-angle"

    return CropReading(value, center, radius, angle, len(ticks), method)
