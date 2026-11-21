"""Experimental per-crop reader for the Exp16 temperature dial.

The colored temperature arc is printed directly inside the black ticks. Its
observed outer boundary, not the crop size, defines where to look for ticks.
No geometry is saved between frames. The reported radius is at the tick band.
Reject if the printed arc, hub, or enough of the scale is not visible; this is
not a reader for monochrome dials or for strong perspective deformation.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from . import CropReading, UnreadableGauge
from .analyze_gauge_opencv import pointer_angle


_ANGLE_COUNT = 1440
_ANGLE_STEP = 360.0 / _ANGLE_COUNT
_ANGLES = np.arange(_ANGLE_COUNT) * (2 * np.pi / _ANGLE_COUNT)
_COS = np.cos(_ANGLES).astype(np.float32)
_SIN = np.sin(_ANGLES).astype(np.float32)


@dataclass
class _Ring:
    center: tuple[int, int]
    radius: int
    indices: np.ndarray
    values: np.ndarray
    score: float


def _hubs(gray: np.ndarray) -> list[tuple[tuple[int, int], float]]:
    height, width = gray.shape
    short = min(gray.shape)
    x0, x1 = round(width * .26), round(width * .74)
    y0, y1 = round(height * .42), round(height * .91)
    blurred = cv2.GaussianBlur(gray[y0:y1, x0:x1], (5, 5), 0)
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, 1.0, short * .035,
        param1=80, param2=13,
        minRadius=max(3, round(short * .012)), maxRadius=round(short * .075),
    )
    if circles is None:
        return []
    # A real hub is surrounded by mostly light dial face. Sample its dark rim
    # rather than its exact center, which can have a bright metal pin.
    angles = slice(None, None, _ANGLE_COUNT // 32)
    candidates = []
    for x, y, r in circles[0]:
        x, y = round(float(x)) + x0, round(float(y)) + y0
        rr = r * np.array([.40, .65, .85, 1.45, 1.8])[:, None]
        xs = np.rint(x + rr * _COS[angles]).astype(int).clip(0, width - 1)
        ys = np.rint(y - rr * _SIN[angles]).astype(int).clip(0, height - 1)
        samples = gray[ys, xs]
        rim = float(np.median(samples[1:3]))
        contrast = float(np.median(samples[3:])) - rim
        if contrast < 24:
            continue
        # The pointer counterweight also looks circular, but normally lacks
        # the hub's little bright central pin. Reward a pin without requiring one.
        d = max(2, round(float(r) * .38))
        dx, dy = np.meshgrid(np.arange(-d, d + 1), np.arange(-d, d + 1))
        patch = gray[(y + dy).clip(0, height - 1), (x + dx).clip(0, width - 1)]
        pin = float(np.percentile(patch[dx * dx + dy * dy <= d * d], 80)) - rim
        candidates.append((contrast + max(0, pin), (x, y), min(.12, max(0, pin) / 800)))
    return [(center, bonus) for _, center, bonus in sorted(candidates, reverse=True)[:4]]


def _longest_arc(supported: np.ndarray) -> tuple[int, int]:
    """Half-open angular bounds on a circular mask, possibly crossing zero."""
    active = np.flatnonzero(supported & ~np.roll(supported, 1))
    inactive = np.flatnonzero(~supported & np.roll(supported, 1))
    if not active.size:
        return 0, _ANGLE_COUNT if supported[0] else 0
    ends = np.r_[inactive, inactive + _ANGLE_COUNT]
    gaps = ends[np.searchsorted(ends, active)] - active
    best = np.argmax(gaps)
    return int(active[best]), int(active[best] + gaps[best])


def _arc_edge(
    chroma: np.ndarray, valid: np.ndarray, radii: np.ndarray,
) -> tuple[int, int, np.ndarray] | None:
    """Find a thick, hub-centered color arc, then its edge per angle."""
    colorful = (chroma >= 49) & valid
    # A narrow chromatic object at one angle (or the lower humidity subdial)
    # does not create a radius supported over a substantial part of the frame.
    support = colorful.mean(axis=1)
    supported = np.r_[False, support >= .33, False]
    limits = np.flatnonzero(supported[1:] != supported[:-1])
    # Look from the interior outward. Exterior color (including broad patches
    # more saturated than the scale) must not take over an inner arc.
    for first, stop in zip(limits[::2], limits[1::2]):
        matching = support[first:stop]
        plateau = np.flatnonzero(matching >= .96 * matching.max())
        peak = first + int(np.median(plateau))
        if stop - first >= max(3, radii[peak] * .025):
            arc = _color_sector(colorful, valid, radii, peak)
            if arc is not None:
                return arc
    return None


def _color_sector(
    colorful: np.ndarray, valid: np.ndarray, radii: np.ndarray, peak: int,
) -> tuple[int, int, np.ndarray] | None:
    nr = max(4, round(radii[peak] * .17))
    lo, hi = max(0, peak - nr), min(len(radii), peak + nr + 1)
    # A circumferential close bridges small interruptions from the pointer and
    # markings. Wrapping the filter avoids an artificial gap at 0 degrees.
    kernel_width = 15
    mask = colorful[lo:hi].astype(np.uint8)
    padded = np.pad(mask, ((0, 0), (kernel_width, kernel_width)), mode="wrap")
    mask = cv2.morphologyEx(padded, cv2.MORPH_CLOSE, np.ones((1, kernel_width), np.uint8))[
        :, kernel_width:-kernel_width
    ].astype(bool) & valid[lo:hi]
    angles = mask.sum(axis=0) >= max(3, round(radii[peak] * .035))
    start, stop = _longest_arc(angles)
    if not 210 <= (stop - start) * _ANGLE_STEP <= 277:
        return None
    columns = np.arange(start, stop) % _ANGLE_COUNT
    # Ignore isolated colored pixels outside the arc body when locating its edge.
    thick = mask[2:] & mask[1:-1] & mask[:-2]
    outer = hi - 1 - np.argmax(thick[::-1, columns], axis=0)
    good = thick[:, columns].any(axis=0)
    if good.mean() < .95 or np.mean(outer >= hi - 1) > .10:
        return None
    outer = np.interp(np.arange(len(columns)), np.flatnonzero(good), outer[good])
    # Smooth one-degree quantization/label defects, reflecting inside the
    # selected sector rather than mixing in pixels outside it.
    outer = np.convolve(np.pad(outer, 4, mode="reflect"), np.ones(9) / 9, mode="valid")
    return start, stop, outer


def _tick_runs(signed: np.ndarray, positive: np.ndarray, peaks: np.ndarray):
    """Yield peak, first row, last row and score for sustained radial strokes."""
    for peak in peaks:
        stroke = np.r_[False, positive[:, peak], False]
        edges = np.flatnonzero(stroke[1:] != stroke[:-1])
        starts, ends = edges[::2], edges[1::2]
        if not starts.size:
            continue
        longest = int(np.argmax(ends - starts))
        first, stop = starts[longest], ends[longest]
        if stop - first >= max(3, .24 * signed.shape[0]):
            yield peak, first, stop - 1, float(signed[first:stop, peak].mean())


def _ring(pixels: np.ndarray, center: tuple[int, int]) -> _Ring | None:
    height, width = pixels.shape[:2]
    short = min(height, width)
    radii = np.arange(round(short * .30), round(short * .86), dtype=np.float32)
    x = center[0] + radii[:, None] * _COS
    y = center[1] - radii[:, None] * _SIN
    valid = (x >= 1) & (x <= width - 2) & (y >= 1) & (y <= height - 2)
    gray, chroma = cv2.remap(pixels, x, y, cv2.INTER_LINEAR).transpose(2, 0, 1)
    arc = _arc_edge(chroma, valid, radii)
    if arc is None:
        return None
    start, stop, outer = arc
    arc_radius = float(radii[0] + np.median(outer))
    # Test narrow angular dark strokes only just outside the observed arc. The
    # reflected padding means the local contrast never samples the exterior of
    # this sector or uses crop padding as a backdrop for a tick.
    laterals = (3, 4, 5)
    top = max(2, round(arc_radius * .016))
    bottom = round(arc_radius * .113)
    rows = np.rint(outer).astype(int)[None, :] + np.arange(top, bottom)[:, None]
    columns = np.arange(start, stop) % _ANGLE_COUNT
    inside = rows < len(radii)
    rows = rows.clip(0, len(radii) - 1)
    strip = gray[rows, columns].astype(np.float32)
    good = inside & valid[rows, columns]
    pad = laterals[-1]
    strip_pad = np.pad(strip, ((0, 0), (pad, pad)), mode="reflect")
    good_pad = np.pad(good, ((0, 0), (pad, pad)), mode="reflect")
    left_samples = [slice(pad - d, pad - d + len(columns)) for d in laterals]
    right_samples = [slice(pad + d, pad + d + len(columns)) for d in laterals]
    signed = np.minimum(
        sum(strip_pad[:, window] for window in left_samples),
        sum(strip_pad[:, window] for window in right_samples),
    ) / len(laterals) - strip
    for window in left_samples + right_samples:
        good &= good_pad[:, window]
    signed = np.where(good, signed, 0)
    profile = np.maximum(signed, 0).mean(axis=0)
    peaks, = np.nonzero((profile[1:-1] > profile[:-2]) & (profile[1:-1] >= profile[2:]))
    peaks = peaks + 1
    peaks = peaks[profile[peaks] >= 3]
    runs = sorted(_tick_runs(signed, signed >= 7, peaks), key=lambda run: -run[3])
    picked = []
    occupied = np.zeros(len(columns), dtype=bool)
    separation = math.ceil(1.65 / _ANGLE_STEP) - 1
    for run in runs:
        index = run[0]
        if not occupied[index]:
            picked.append(run)
            occupied[max(0, index - separation):index + separation + 1] = True
    if len(picked) < 66:
        return None
    picked.sort()
    peak_ids, first, last, scores = np.asarray(picked).T
    peak_ids = peak_ids.astype(int)
    # The evidence has to span the whole scale. A text patch, a mesh, or a
    # cluster of lines on one side must not establish a temperature ring.
    sectors = np.histogram(peak_ids, bins=np.linspace(0, len(columns), 8))[0]
    gaps = np.diff(peak_ids) * _ANGLE_STEP
    spacing = float(np.median(gaps))
    regular = float(np.mean(np.abs(gaps - spacing) <= .31 * spacing))
    if sectors.min() < 7 or not 2.4 <= spacing <= 3.65 or regular < .79:
        return None
    edge_gaps = np.array([peak_ids[0], len(columns) - 1 - peak_ids[-1]]) * _ANGLE_STEP
    if edge_gaps.max() > 5.0 or gaps.max() > 4.5 * spacing:
        return None
    # The printed -30..50 scale has one slot per degree Celsius. Preserve an
    # occasional occluded tick as a missing slot rather than renumbering all
    # later ticks. The arc ends distinguish a missing terminal stroke from an
    # interior gap. Reject ambiguous anchors rather than stretching the scale.
    slots = np.r_[0, np.cumsum(np.maximum(1, np.rint(gaps / spacing)).astype(int))]
    missing = 80 - slots[-1]
    missing_at_left = 0
    if missing == 2 and edge_gaps.min() >= .72 * spacing:
        missing_at_left = 1
    elif missing == 1 and edge_gaps.max() >= .60 * spacing and abs(np.diff(edge_gaps)[0]) >= .28 * spacing:
        missing_at_left = int(edge_gaps[1] > edge_gaps[0])
    elif missing != 0:
        return None
    values = -30 + missing_at_left + slots[-1] - slots
    radius = round(float(np.median(radii[rows[0, peak_ids]] + (first + last) / 2)))
    # Reward angular regularity and an arc concentric with the hub; never rank
    # by which candidate has a tick count closest to the expected 79.
    arc_wobble = float(np.std(outer) / arc_radius)
    score = regular - 4 * arc_wobble + min(.14, float(np.median(scores)) / 500)
    return _Ring(center, radius, peak_ids + start, values, score)


def _refine_pointer(gray: np.ndarray, ring: _Ring, angle: float) -> float:
    """Center the supported LSD edge on the actual dark needle stroke."""
    offsets = np.arange(-7.0, 7.01, _ANGLE_STEP)
    bearings = np.deg2rad(angle + offsets).astype(np.float32)
    radii = np.linspace(.40 * ring.radius, .74 * ring.radius, 40, dtype=np.float32)[:, None]
    xs = ring.center[0] + radii * np.cos(bearings)
    ys = ring.center[1] - radii * np.sin(bearings)
    if xs.min() < 1 or xs.max() > gray.shape[1] - 2 or ys.min() < 1 or ys.max() > gray.shape[0] - 2:
        return angle
    profile = np.median(cv2.remap(gray, xs, ys, cv2.INTER_LINEAR), axis=0)
    search = np.flatnonzero(np.abs(offsets) <= 3.0)
    best = int(search[np.argmin(profile[search])])
    left = profile[best - 16:best - 9]
    right = profile[best + 10:best + 17]
    # Don't move an angle based on a monotone lighting gradient or a weak
    # local minimum. Both sides of a supported needle should be brighter.
    if min(np.median(left), np.median(right)) - profile[best] < 20:
        return angle
    near = search[(np.abs(search - best) <= 5) & (profile[search] <= profile[best] + 7)]
    return (angle + float(offsets[near].mean())) % 360.0


def read_temperature_auto_ring(image: np.ndarray) -> CropReading:
    """Read a native BGR detector crop, or raise UnreadableGauge."""
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8 or min(image.shape[:2]) < 90:
        raise UnreadableGauge("invalid temperature crop")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hubs = _hubs(gray)
    if not hubs:
        raise UnreadableGauge("no supported temperature hub")
    blue, green, red = cv2.split(image)
    chroma = cv2.subtract(cv2.max(cv2.max(blue, green), red), cv2.min(cv2.min(blue, green), red))
    pixels = cv2.merge((gray, chroma))
    rings = []
    for center, pin_bonus in hubs:
        ring = _ring(pixels, center)
        if ring is not None:
            rings.append((ring.score + pin_bonus, ring))
    if not rings:
        raise UnreadableGauge("no supported temperature tick ring")
    _, ring = max(rings, key=lambda candidate: candidate[0])
    block = max(3, round(ring.radius * .16) | 1)
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, block, 8)
    angle = _refine_pointer(gray, ring, pointer_angle(image, binary, ring.center, ring.radius))
    # Read on the actual detected angles (the pixel sweep is not quite linear
    # under perspective). Interpolation only affects occluded tick slots.
    angles = ring.indices * _ANGLE_STEP
    unwrapped = angles[0] + (angle - angles[0] + 2.0) % 360.0 - 2.0
    if unwrapped > angles[-1] + 2.0:
        raise UnreadableGauge("temperature pointer outside scale")
    value = float(round(float(np.interp(unwrapped, angles, ring.values))))
    return CropReading(value, ring.center, ring.radius, angle, len(angles), "auto-ring")
