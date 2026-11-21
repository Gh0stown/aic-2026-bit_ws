"""Runtime thresholding and tick rectangles from tick_tuning and tick_rect_tuning."""

from __future__ import annotations

import math

import cv2
import numpy as np


def make_binary(
    image: np.ndarray,
    center: tuple[int, int],
    radius: int,
    block_size: int,
    c_value: float,
    clip_limit: float,
) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(clip_limit, (8, 8)).apply(gray)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    binary = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        block_size,
        c_value,
    )
    mask = np.zeros_like(binary)
    cv2.circle(mask, center, max(1, int(radius * 0.99)), 255, -1)
    return cv2.bitwise_and(binary, mask)


def component_rectangles(
    binary: np.ndarray,
    center: tuple[int, int],
    radius: int,
    inner_ratio: float,
    outer_ratio: float,
    min_length: float,
    max_length: float,
    min_width: float,
    max_width: float,
    min_aspect: float,
    max_aspect: float,
    max_alignment: float,
    min_area: int,
) -> list[dict[str, object]]:
    annulus = np.zeros_like(binary)
    cv2.circle(annulus, center, int(radius * outer_ratio), 255, -1)
    cv2.circle(annulus, center, int(radius * inner_ratio), 0, -1)
    masked = cv2.bitwise_and(binary, annulus)

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(masked, 8)
    cx, cy = center
    detections = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        x = int(stats[label, cv2.CC_STAT_LEFT])
        y = int(stats[label, cv2.CC_STAT_TOP])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        component = np.column_stack(np.where(labels[y : y + height, x : x + width] == label))
        if len(component) < 5:
            continue
        component[:, 0] += y
        component[:, 1] += x
        points = component[:, ::-1].astype(np.float32)
        rectangle = cv2.minAreaRect(points)
        (rect_x, rect_y), (side_a, side_b), rectangle_angle = rectangle
        long_side = max(side_a, side_b)
        short_side = max(1.0, min(side_a, side_b))
        aspect = long_side / short_side
        midpoint_radius = math.hypot(rect_x - cx, rect_y - cy) / radius
        if not inner_ratio <= midpoint_radius <= outer_ratio:
            continue
        if not min_length <= long_side <= max_length:
            continue
        if not min_width <= short_side <= max_width:
            continue
        if not min_aspect <= aspect <= max_aspect:
            continue

        mean = points.mean(axis=0)
        covariance = np.cov((points - mean).T)
        values, vectors = np.linalg.eigh(covariance)
        long_axis = vectors[:, int(np.argmax(values))]
        long_angle = math.degrees(math.atan2(-long_axis[1], long_axis[0])) % 180.0
        radial_angle = math.degrees(math.atan2(-(rect_y - cy), rect_x - cx)) % 180.0
        alignment = abs(((long_angle - radial_angle + 90.0) % 180.0) - 90.0)
        if alignment > max_alignment:
            continue

        angle = math.degrees(math.atan2(-(rect_y - cy), rect_x - cx)) % 360.0
        detections.append(
            {
                "center": (float(rect_x), float(rect_y)),
                "rect": rectangle,
                "area": area,
                "length": float(long_side),
                "width": float(short_side),
                "aspect": float(aspect),
                "radius_ratio": float(midpoint_radius),
                "angle_deg": float(angle),
                "alignment": float(alignment),
                "score": float(
                    long_side / radius
                    + aspect / max(long_side, short_side)
                    - alignment / 90.0
                ),
            }
        )
    detections.sort(key=lambda item: item["angle_deg"])
    return detections


def detect_tick_rectangles(image, center, radius, config):
    binary = make_binary(image, center, radius, config["block"], config["c"], config["clip"])
    kernel = config.get("erode_kernel", 0)
    if kernel:
        binary = cv2.erode(
            binary, np.ones((kernel, kernel), np.uint8),
            iterations=config["erode_iterations"],
        )
    detections = component_rectangles(
        binary, center, radius, config["inner"], config["outer"],
        config["min_length"], config["max_length"], config["min_width"], config["max_width"],
        config["min_aspect"], config["max_aspect"], config["alignment"], config["min_area"],
    )
    return binary, detections
