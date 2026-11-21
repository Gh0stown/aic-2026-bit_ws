"""Runtime geometry and pointer primitives from the Exp16 handoff."""

from __future__ import annotations

import math

import cv2
import numpy as np

from . import UnreadableGauge


def prepare_gray(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(2.0, (8, 8)).apply(gray)
    return cv2.GaussianBlur(gray, (5, 5), 0)


def outer_dial_geometry(
    image: np.ndarray,
    center_x_ratio: float,
    center_y_ratio: float,
    radius_ratio: float,
) -> tuple[tuple[int, int], int]:
    """Use the tight YOLO crop as the outer-dial coordinate system."""
    height, width = image.shape[:2]
    center = (int(round(width * center_x_ratio)), int(round(height * center_y_ratio)))
    radius = max(20, int(round(min(width, height) * radius_ratio)))
    return center, radius


def unwrap(
    image: np.ndarray,
    center: tuple[int, int],
    radius: int,
    width: int = 1440,
) -> np.ndarray:
    # OpenCV lays out radius along x and angle along y. The handoff's
    # pointer scoring expects the transpose, with one radial run per column.
    return cv2.warpPolar(
        image,
        (radius, width),
        center,
        radius,
        cv2.WARP_POLAR_LINEAR | cv2.WARP_FILL_OUTLIERS,
    ).T


def circular_smooth(values: np.ndarray, radius: int = 5) -> np.ndarray:
    kernel = np.ones(radius * 2 + 1, dtype=np.float32)
    padded = np.pad(values, radius, mode="wrap")
    return np.convolve(padded, kernel / kernel.sum(), mode="valid")


def circular_peaks(
    values: np.ndarray,
    min_distance: int,
    limit: int,
) -> list[tuple[int, float]]:
    count = len(values)
    order = np.argsort(values)[::-1]
    selected: list[tuple[int, float]] = []
    for index in order:
        if values[index] <= 0:
            break
        if all(
            min(abs(int(index) - other), count - abs(int(index) - other)) >= min_distance
            for other, _ in selected
        ):
            selected.append((int(index), float(values[index])))
            if len(selected) >= limit:
                break
    return selected


def longest_run(values: np.ndarray) -> tuple[int, int]:
    best_start = best_length = current_start = current_length = 0
    in_run = False
    for index, value in enumerate(values):
        if value:
            if not in_run:
                current_start = index
                current_length = 0
                in_run = True
            current_length += 1
            if current_length > best_length:
                best_start = current_start
                best_length = current_length
        else:
            in_run = False
    return best_start, best_length


def detect_pointer(
    polar_binary: np.ndarray,
    inner_start: float = 0.10,
    inner_end: float = 0.78,
) -> dict[str, object]:
    height, width = polar_binary.shape
    r1 = max(2, int(height * inner_start))
    r2 = max(r1 + 2, int(height * inner_end))
    band = (polar_binary[r1:r2] > 0).astype(np.uint8)
    span = r2 - r1

    scores = np.zeros(width, dtype=np.float32)
    runs = []
    for angle in range(width):
        start, length = longest_run(band[:, angle])
        runs.append((r1 + start, r1 + start + max(0, length - 1)))
        scores[angle] = length / span
    scores = circular_smooth(scores, radius=4)
    peaks = circular_peaks(scores, min_distance=max(8, width // 90), limit=5)

    candidates = []
    for index, score in peaks:
        start, end = runs[index]
        candidates.append(
            {
                "angle_deg": round((360.0 - index * 360.0 / width) % 360.0, 2),
                "score": round(score, 4),
                "radial_run": [start, end],
            }
        )
    return {
        "best": candidates[0] if candidates else None,
        "candidates": candidates,
    }


def detect_pointer_lines(
    gray: np.ndarray,
    center: tuple[int, int],
    radius: int,
) -> dict[str, object]:
    local_gray = cv2.GaussianBlur(gray, (3, 3), 0)
    detector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    lines = detector.detect(local_gray)[0]
    if lines is None:
        return {"best": None, "candidates": []}

    cx, cy = center
    candidates = []
    for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
        dx, dy = x2 - x1, y2 - y1
        length = float(math.hypot(dx, dy))
        denominator = max(length, 1.0)
        perpendicular = float(abs(dy * (cx - x1) - dx * (cy - y1)) / denominator)
        r1 = float(math.hypot(x1 - cx, y1 - cy))
        r2 = float(math.hypot(x2 - cx, y2 - cy))
        near_center = min(r1, r2)
        far_endpoint = max(r1, r2)
        if length < radius * 0.15 or perpendicular > radius * 0.18:
            continue
        if near_center > radius * 0.45 or far_endpoint < radius * 0.50:
            continue

        tip_x, tip_y = (x1, y1) if r1 > r2 else (x2, y2)
        angle = float(math.degrees(math.atan2(-(tip_y - cy), tip_x - cx)) % 360.0)
        score = float(
            length / radius
            - 1.5 * perpendicular / radius
            + 0.35 * (1.0 - near_center / radius)
        )
        candidates.append(
            {
                "angle_deg": round(angle, 2),
                "score": round(score, 4),
                "line": [int(x1), int(y1), int(x2), int(y2)],
                "length": round(length, 2),
                "center_distance": round(perpendicular, 2),
                "radial_span": [round(near_center, 2), round(far_endpoint, 2)],
            }
        )

    candidates.sort(key=lambda item: item["score"], reverse=True)
    selected = []
    for candidate in candidates:
        angle = candidate["angle_deg"]
        if all(
            min(
                abs(angle - other["angle_deg"]),
                360.0 - abs(angle - other["angle_deg"]),
            )
            >= 3.0
            for other in selected
        ):
            selected.append(candidate)
        if len(selected) >= 8:
            break
    return {"best": selected[0] if selected else None, "candidates": selected}


def pointer_angle(image, binary, center, radius):
    """Prefer the original LSD method, then require a supported polar run."""
    if hasattr(cv2, "createLineSegmentDetector"):
        try:
            best = detect_pointer_lines(prepare_gray(image), center, radius)["best"]
        except cv2.error as exc:
            # Some OpenCV 4.x versions expose LSD but do not implement it.
            if exc.code != cv2.Error.StsNotImplemented:
                raise
        else:
            if best is not None:
                return float(best["angle_deg"])

    polar = unwrap(binary, center, radius)
    for candidate in detect_pointer(polar)["candidates"]:
        start, end = candidate["radial_run"]
        # A positive peak alone can come from dial text or a stray pixel.
        # Require a contiguous run over half of the sampled radial band,
        # starting near the hub.
        if start <= radius * 0.35 and end - start + 1 >= radius * (0.78 - 0.10) / 2:
            return float(candidate["angle_deg"])
    raise UnreadableGauge("no supported pointer")
