"""Exp16 humidity reader, with geometry from read_humidity_rect."""

from __future__ import annotations

import math

import cv2
import numpy as np

from . import CropReading, UnreadableGauge
from .analyze_gauge_opencv import pointer_angle
from .tick_rect_tuning import detect_tick_rectangles


HUMIDITY_RECT = {
    "block": 9,
    "c": 3.0,
    "clip": 2.0,
    "inner": 0.80,
    "outer": 0.98,
    "min_length": 2.0,
    "max_length": 18.0,
    "min_width": 1.0,
    "max_width": 4.0,
    "min_aspect": 1.2,
    "max_aspect": 12.0,
    "alignment": 20.0,
    "min_area": 2,
    "erode_kernel": 2,
    "erode_iterations": 1,
}


def detect_geometry(
    image: np.ndarray,
) -> tuple[tuple[int, int], int, int]:
    height, width = image.shape[:2]
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    outer = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        1.2,
        minDist=min(width, height) * 0.30,
        param1=100,
        param2=32,
        minRadius=int(min(width, height) * 0.30),
        maxRadius=int(min(width, height) * 0.56),
    )
    if outer is None:
        raise RuntimeError("未检测到湿度表盘外圆")
    candidates = []
    for x, y, radius in outer[0]:
        distance = math.hypot(x - width * 0.49, y - height * 0.47)
        score = -distance / min(width, height) + 0.10 * radius / min(width, height)
        candidates.append((score, x, y, radius))
    _, cx, cy, radius = max(candidates, key=lambda item: item[0])
    center = (int(round(cx)), int(round(cy)))
    radius_i = int(round(radius))

    inner = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        1.1,
        minDist=max(12, int(min(width, height) * 0.10)),
        param1=80,
        param2=14,
        minRadius=max(3, int(min(width, height) * 0.02)),
        maxRadius=max(8, int(min(width, height) * 0.12)),
    )
    hub_radius = 0
    if inner is not None:
        hub_candidates = []
        for x, y, hub in inner[0]:
            distance = math.hypot(x - center[0], y - center[1])
            if distance < min(width, height) * 0.12:
                hub_candidates.append((distance, x, y, hub))
        if hub_candidates:
            _, hx, hy, hub_radius_f = min(hub_candidates, key=lambda item: item[0])
            center = (int(round(hx)), int(round(hy)))
            hub_radius = int(round(hub_radius_f))
    return center, radius_i, hub_radius


def bearing_from_top(
    point: tuple[float, float],
    center: tuple[int, int],
) -> float:
    dx = float(point[0]) - center[0]
    dy = float(point[1]) - center[1]
    return math.degrees(math.atan2(dx, -dy)) % 360.0


def ordered_scale_ticks(
    detections: list[dict[str, object]],
    center: tuple[int, int],
    start_bearing: float = 20.0,
    end_bearing: float = 340.0,
) -> list[dict[str, object]]:
    ticks = []
    for detection in detections:
        bearing = bearing_from_top(detection["center"], center)
        if start_bearing <= bearing <= end_bearing:
            item = dict(detection)
            item["bearing_from_top"] = bearing
            ticks.append(item)
    ticks.sort(key=lambda item: float(item["bearing_from_top"]))
    gaps = [
        float(ticks[index + 1]["bearing_from_top"])
        - float(ticks[index]["bearing_from_top"])
        for index in range(len(ticks) - 1)
    ]
    median_gap = float(np.median(gaps)) if gaps else 1.0
    if median_gap <= 0.0:
        raise UnreadableGauge("invalid humidity tick spacing")
    value = 0.0
    for index, tick in enumerate(ticks):
        missing_before = 0
        if index:
            gap = float(tick["bearing_from_top"]) - float(ticks[index - 1]["bearing_from_top"])
            missing_before = max(0, int(round(gap / median_gap)) - 1)
            value += 2.0 * (missing_before + 1)
        tick["order"] = index + 1
        tick["missing_before"] = missing_before
        tick["value"] = value
    return ticks


def read_pointer_gap_aware(
    ticks: list[dict[str, object]],
    pointer_bearing: float,
) -> tuple[float, dict[str, float] | None]:
    if len(ticks) < 2:
        raise UnreadableGauge("湿度刻度不足")
    bearings = np.array([float(tick["bearing_from_top"]) for tick in ticks])
    values = np.array([float(tick["value"]) for tick in ticks])
    gaps = np.diff(bearings)
    median_gap = float(np.median(gaps))

    for index, gap in enumerate(gaps):
        if bearings[index] <= pointer_bearing <= bearings[index + 1]:
            if ticks[index + 1]["missing_before"] > 0:
                missing_slots = int(ticks[index + 1]["missing_before"])
                ratio = (pointer_bearing - bearings[index]) / max(gap, 1e-6)
                slot = min(missing_slots, max(1, int(round(ratio * (missing_slots + 1)))))
                value = values[index] + slot * 2.0
                gap_info = {
                    "gap_deg": float(gap),
                    "mid_bearing": float((bearings[index] + bearings[index + 1]) / 2),
                    "value": float(value),
                    "missing_slots": missing_slots,
                }
                return gap_info["value"], gap_info
            ratio = (pointer_bearing - bearings[index]) / max(gap, 1e-6)
            value = values[index] + ratio * (values[index + 1] - values[index])
            return float(round(value / 2.0) * 2.0), None
    nearest = int(np.argmin(np.abs(bearings - pointer_bearing)))
    return float(values[nearest]), None


def read_humidity(image: np.ndarray) -> CropReading:
    center, radius, _hub_radius = detect_geometry(image)
    binary, detections = detect_tick_rectangles(image, center, radius, HUMIDITY_RECT)
    start_bearing, end_bearing = 20.0, 340.0
    ticks = ordered_scale_ticks(detections, center, start_bearing, end_bearing)
    # The offline minimum of 10 allowed a mostly missing scale to look valid;
    # sparse ROIs can shift the 0% anchor and strongly underestimate humidity.
    if len(ticks) < 30:
        raise UnreadableGauge("too few humidity ticks ({})".format(len(ticks)))

    angle = pointer_angle(image, binary, center, radius)
    bearing = (90.0 - angle) % 360.0
    if not start_bearing <= bearing <= end_bearing:
        # The offline helper would otherwise snap the lettering sector to a
        # scale endpoint and produce a plausible but unsupported value.
        raise UnreadableGauge("humidity pointer outside scale")
    value, gap = read_pointer_gap_aware(ticks, bearing)
    if gap is not None:
        angle = (90.0 - gap["mid_bearing"]) % 360.0
    return CropReading(value, center, radius, angle, len(ticks), "tick-gap" if gap else "tick-order")
