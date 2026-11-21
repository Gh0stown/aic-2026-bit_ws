"""Experimental native-crop reader for the colored Exp16 (No.9011) RH dial.

Only the printed scale layout is known: 0–100 clockwise, one long stroke per
10 %RH and four short strokes between long strokes. The red/green/blue band
just outside the strokes distinguishes the two endpoints from interior major
marks. No camera geometry, angle/value table, or prior frame is used.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from . import CropReading, UnreadableGauge


_STEP = .25
_N = round(360 / _STEP)
_BEARINGS = np.deg2rad(np.arange(_N) * _STEP).astype(np.float32)
_SIN, _COS = np.sin(_BEARINGS), np.cos(_BEARINGS)


@dataclass(frozen=True)
class _Arc:
    start: int
    stop: int  # unwrapped, inclusive
    inner: np.ndarray  # inner color boundary in radial pixels
    radius: float  # median inner edge, not the bezel
    transitions: tuple[float, float]  # printed color changes at 40 and 70 %RH


@dataclass(frozen=True)
class _Tick:
    column: int  # unwrapped angular index
    length: int
    strength: float


def _runs(mask):
    ends = np.flatnonzero(np.diff(mask.astype(np.int8), prepend=0, append=0))
    return zip(ends[::2], ends[1::2])


def _sample(image, center, radii, columns=slice(None)):
    radii = np.asarray(radii, dtype=np.float32)
    xs = center[0] + radii * _SIN[columns]
    ys = center[1] - radii * _COS[columns]
    valid = ((xs >= 1) & (xs <= image.shape[1] - 2) &
             (ys >= 1) & (ys <= image.shape[0] - 2))
    return cv2.remap(image, xs, ys, cv2.INTER_LINEAR), valid


def _hubs(gray):
    short = min(gray.shape)
    circles = cv2.HoughCircles(cv2.GaussianBlur(gray, (5, 5), 0), cv2.HOUGH_GRADIENT,
                              1, max(6, short * .045), param1=80, param2=14,
                              minRadius=max(4, round(short * .032)),
                              maxRadius=max(7, round(short * .10)))
    if circles is None:
        return []
    candidates = []
    for xx, yy, r in circles[0]:
        x, y = round(float(xx)), round(float(yy))
        if not r * 2 < min(x, y, gray.shape[1] - x, gray.shape[0] - y):
            continue
        ramps, _ = _sample(gray, (x, y), np.array([.62, .84, 1.33, 1.65])[:, None] * r,
                           slice(None, None, 24))
        rim = float(np.median(ramps[:2]))
        contrast = float(np.median(ramps[2:])) - rim
        if contrast < 35:
            continue
        # The pale pin, when visible within the dark hub, is more precise than
        # the fitted black rim (which is interrupted by needle and tail).
        d = max(2, round(.46 * float(r)))
        pin = gray[y - d:y + d + 1, x - d:x + d + 1]
        jj, ii = np.indices(pin.shape)
        lit = (pin > rim + .66 * contrast) & ((ii - d) ** 2 + (jj - d) ** 2 <= d * d)
        bonus = 0
        if 3 <= lit.sum() <= .56 * (np.pi * d * d):
            dy, dx = np.mean(np.argwhere(lit), axis=0) - d
            if math.hypot(dx, dy) <= .30 * r:
                x, y = x + round(float(dx)), y + round(float(dy))
                bonus = 30
        candidates.append((contrast + bonus, (x, y)))
    return list(dict.fromkeys(center for _, center in sorted(candidates, reverse=True)))[:4]


def _arc(image, center):
    short = min(image.shape[:2])
    radii = np.arange(round(short * .23), round(short * .73))
    strip, valid = _sample(image, center, radii[:, None])
    hue, sat, val = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV).transpose(2, 0, 1)
    red = (((hue <= 25) | (hue >= 166)) & (sat >= 65))
    green = ((hue >= 37) & (hue <= 87) & (sat >= 83))
    blue = ((hue >= 88) & (hue <= 132) & (sat >= 69))
    color = (red | green | blue) & valid & (val >= 55)
    coverage = color.mean(axis=1)
    peak = int(np.argmax(coverage))
    if coverage[peak] < .30:
        return None
    band = max(6, round(.25 * radii[peak]))
    lo, hi = max(0, peak - band), min(len(radii), peak + band + 1)
    mask = color[lo:hi]
    # Demand four consecutive colored pixels, not sparse chromatic clutter.
    thick = mask[:-3] & mask[1:-2] & mask[2:-1] & mask[3:]
    raw = thick.any(axis=0)
    pad = 26  # bridge short interruptions from white lettering in the colored band
    bridge = cv2.morphologyEx(np.pad(raw.astype(np.uint8), pad, mode="wrap")[None, :],
                              cv2.MORPH_CLOSE, np.ones((1, pad + 1), np.uint8))[0, pad:-pad].astype(bool)
    starts = np.flatnonzero(bridge & ~np.roll(bridge, 1))
    if not len(starts):
        return None
    stops = np.flatnonzero(~bridge & np.roll(bridge, 1))
    spans = (stops[:, None] - starts) % _N
    widths = np.min(np.where(spans > 0, spans, _N), axis=0)
    best = int(np.argmax(widths))
    start, size = int(starts[best]), int(widths[best])
    if not 215 <= size * _STEP <= 320:
        return None
    columns = (start + np.arange(size)) % _N
    rows = np.argmax(thick[:, columns], axis=0) + lo
    good = raw[columns]
    if (good.mean() < .96 or np.mean((rows[good] <= lo) | (rows[good] >= hi - 4)) > .04):
        return None
    # A cropped band can end at the image edge and masquerade as a real end.
    # Check that the projected band just before and after both ends is visible.
    for col, row in ((start, rows[0]), (start + size - 1, rows[-1])):
        nearby = (col + np.arange(-12, 13)) % _N
        if not valid[row:min(len(radii), row + 5), nearby].all():
            return None
    # Require the known order of the printed colors independently of tick
    # counts; a dark sector or cropped monochrome fragment is not an origin.
    core = (np.minimum(rows + 2, hi - 1), columns)
    kinds = np.select([red[core], green[core], blue[core]], [1, 2, 3])
    kinds = np.where(good, kinds, 0)
    edge = min(40, size // 10)
    if np.mean(kinds[:edge] == 1) < .76 or np.mean(kinds[-edge:] == 3) < .76:
        return None
    parts = [np.flatnonzero(kinds == i) for i in (1, 2, 3)]
    if any(len(part) < .10 * size for part in parts):
        return None
    q = np.array([np.percentile(part, [5, 50, 95]) for part in parts])
    if not (q[0, 2] < q[1, 1] and q[0, 1] < q[1, 0] and
            q[1, 2] < q[2, 1] and q[1, 1] < q[2, 0]):
        return None
    transitions = []
    for before, after in ((1, 2), (2, 3)):
        loss = np.cumsum(kinds == after) + np.cumsum(kinds[::-1] == before)[::-1]
        boundary = np.flatnonzero(loss == loss.min())
        if np.ptp(boundary) * _STEP > 6:
            return None
        transitions.append(float(start + (boundary[0] + boundary[-1]) / 2))
    # Approximate the observed inner edge with a smooth projected ring. The
    # low harmonics absorb translation/ellipticity; reject a ring that would
    # require a substantial extrapolation through irregular color clutter.
    basis = np.stack((np.ones(size), _SIN[columns], _COS[columns],
                      2 * _SIN[columns] * _COS[columns], 2 * _COS[columns] ** 2 - 1), axis=1)
    measured = radii[rows]
    inliers = good.copy()
    for _ in range(3):
        inner = basis @ np.linalg.lstsq(basis[inliers], measured[inliers], rcond=None)[0]
        inliers = good & (np.abs(inner - measured) <= max(2.5, radii[peak] * .034))
        if inliers.mean() < .76:
            return None
    radius = float(np.median(inner))
    if np.ptp(np.percentile(inner, [5, 95])) > .24 * radius:
        return None
    return _Arc(start, start + size - 1, inner, radius, tuple(transitions))


def _ticks(gray, center, arc):
    margin = round(8 / _STEP)
    indices = np.arange(arc.start - margin, arc.stop + margin + 1)
    columns = indices % _N
    inner = np.pad(arc.inner, margin, mode="edge")
    offsets = np.arange(1, round(arc.radius * .26) + 1)
    strip, valid = _sample(gray, center, inner - offsets[:, None], columns)
    strip = strip.astype(np.float32)
    distance = round(1.65 / _STEP)
    shifted = np.pad(strip, ((0, 0), (distance, distance)), mode="edge")
    signed = np.minimum(shifted[:, :len(columns)], shifted[:, 2 * distance:]) - strip
    signed = np.where(valid, signed, 0)
    active = signed >= 7
    # Ignore strong strokes isolated from the tick band (e.g. numeral stems).
    max_outer = max(5, round(.075 * arc.radius))
    profile = np.maximum(signed[:round(.16 * arc.radius)], 0).sum(axis=0)
    peaks = np.flatnonzero((profile[1:-1] > profile[:-2]) & (profile[1:-1] >= profile[2:])) + 1
    candidates = []
    for k in peaks:
        for lo, hi in _runs(active[:, k]):
            if (lo < max_outer and hi - lo >= max(3, round(.034 * arc.radius)) and
                    hi - lo <= .23 * arc.radius and np.mean(signed[lo:hi, k]) >= 12):
                candidates.append(_Tick(int(indices[k]), hi - lo, float(signed[lo:hi, k].sum())))
    picked = []
    for tick in sorted(candidates, key=lambda tick: -tick.strength):
        if all(abs(tick.column - other.column) * _STEP >= 2.05 for other in picked):
            picked.append(tick)
    ordered = sorted(picked, key=lambda tick: tick.column)
    return [tick for k, tick in enumerate(ordered)
            if not (0 < k < len(ordered) - 1 and
                    tick.strength < .35 * min(ordered[k - 1].strength, ordered[k + 1].strength))]


def _scale(ticks, arc):
    """Verify the major scale and label the full set of graduations."""
    end_tolerance = 3.75 / _STEP
    ticks = [tick for tick in ticks
             if arc.start - end_tolerance <= tick.column <= arc.stop + end_tolerance]
    major = []
    for k, tick in enumerate(ticks):
        if tick.length < max(6, .082 * arc.radius):
            continue
        nearby = ticks[max(0, k - 3):k] + ticks[k + 1:k + 4]
        if len(nearby) < 3:
            continue
        length = float(np.median([other.length for other in nearby]))
        strength = float(np.median([other.strength for other in nearby]))
        if ((tick.length >= 1.25 * length and tick.strength >= 1.28 * strength) or
                (tick.length >= 1.17 * length and tick.strength >= 1.65 * strength)):
            major.append(k)
    if (len(major) != 11 or major[0] != 0 or major[-1] != len(ticks) - 1 or
            abs(ticks[0].column - arc.start) > end_tolerance or
            abs(ticks[-1].column - arc.stop) > end_tolerance):
        raise UnreadableGauge("humidity scale lacks both endpoints and eleven distinct major ticks")
    if any(abs(ticks[major[group]].column - transition) * _STEP > 3
           for group, transition in zip((4, 7), arc.transitions)):
        raise UnreadableGauge("humidity major ticks disagree with printed color anchors")
    scale = [(ticks[0].column, 0)]
    for group, (first, last) in enumerate(zip(major, major[1:])):
        scale.extend(_label_interval(ticks[first:last + 1], 10 * group)[1:])
    return scale


def _label_interval(ticks, first_value):
    columns = np.array([tick.column for tick in ticks])
    gaps = np.diff(columns).astype(float)
    if len(gaps) == 5:
        slots = np.arange(6)
        if gaps.max() > 2.15 * gaps.min():
            raise UnreadableGauge("unresolved humidity minor tick spacing")
    elif len(gaps) == 4:
        # Label visible strokes, not an invented angle for the occluded stroke.
        missing = int(np.argmax(gaps))
        ordinary = np.delete(gaps, missing)
        ratio = gaps[missing] / np.median(ordinary)
        if (not 1.65 <= ratio <= 2.55 or gaps[missing] < 1.36 * ordinary.max() or
                ordinary.max() > 1.75 * ordinary.min()):
            raise UnreadableGauge("ambiguous occluded humidity minor tick")
        slots = np.arange(5)
        slots[missing + 1:] += 1
    else:
        raise UnreadableGauge("unsupported humidity minor tick count")
    return list(zip(columns, first_value + 2 * slots))


def _occluded_tick(ticks, major, approx, arc, pointer):
    """Recover one hidden minor only beside a corroborated weak major mark."""
    labels = sorted(major)
    for lo, hi in zip(labels, labels[1:]):
        if hi - lo != 20:
            continue
        first, last = major[lo], major[hi]
        # Eleven graduations including both endpoints, with exactly one missing.
        if last - first != 9 or not ticks[first].column < pointer < ticks[last].column:
            continue
        shared = (lo + 10) // 10
        allowed_offset = .38 * min(approx[shared] - approx[shared - 1],
                                  approx[shared + 1] - approx[shared])
        candidates = []
        for mid in (first + 4, first + 5):
            tick = ticks[mid]
            if (abs(tick.column - approx[shared]) > allowed_offset or
                    tick.length < max(6, .080 * arc.radius)):
                continue
            nearby = ticks[mid - 3:mid] + ticks[mid + 1:mid + 4]
            if (tick.length < np.median([t.length for t in nearby]) or
                    tick.strength < 1.75 * np.median([t.strength for t in nearby])):
                continue
            try:
                left = _label_interval(ticks[first:mid + 1], lo)
                right = _label_interval(ticks[mid:last + 1], lo + 10)
            except UnreadableGauge:
                continue
            scale = left + right[1:]
            missing = [(a, b) for a, b in zip(scale, scale[1:]) if b[1] - a[1] == 4]
            if len(missing) == 1:
                candidates.append(missing[0])
        if len(candidates) == 1:
            (a, lower), (b, upper) = candidates[0]
            # An inferred tick has no precise angle: only a ray near the
            # middle of its uniquely doubled gap gets the discrete graduation.
            if abs(pointer - (a + b) / 2) <= .18 * (b - a):
                return float((lower + upper) // 2), last - first + 1
    raise UnreadableGauge("humidity pointer lacks a supported local interval")


def _local_scale(ticks, arc, pointer):
    """Allow holes elsewhere, but require physical evidence at the pointer."""
    anchor_values = (0, 40, 70, 100)
    anchors = (arc.start, *arc.transitions, arc.stop)
    labels = np.arange(0, 101, 10)
    approx = np.interp(labels, anchor_values, anchors)
    steps = np.diff(approx)
    # Approximate angles only bound the search for a long printed stroke; they
    # are never used as the angle-to-RH conversion.
    candidates = []
    for index, tick in enumerate(ticks):
        nearby = ticks[max(0, index - 3):index] + ticks[index + 1:index + 4]
        if len(nearby) < 3 or tick.length < max(6, .073 * arc.radius):
            continue
        length = tick.length / float(np.median([t.length for t in nearby]))
        strength = tick.strength / float(np.median([t.strength for t in nearby]))
        if (length >= 1.24 and strength >= 1.08) or (length >= 1.15 and strength >= 1.5):
            candidates.append((index, length, strength))
    major = {}
    for i, (label, target) in enumerate(zip(labels, approx)):
        pitch = min(steps[max(0, i - 1)], steps[min(i, len(steps) - 1)])
        tolerance = 3.0 / _STEP if label in anchor_values else .38 * pitch
        matching = [(index, .25 * min(2, length) + .10 * min(2, strength)
                              - abs(ticks[index].column - target) / tolerance)
                    for index, length, strength in candidates if abs(ticks[index].column - target) <= tolerance]
        matching.sort(key=lambda item: item[1], reverse=True)
        if matching and (len(matching) == 1 or matching[0][1] - matching[1][1] >= .12):
            major[int(label)] = matching[0][0]
    if not all(label in major for label in anchor_values) or len(major) < 7:
        raise UnreadableGauge("humidity scale lacks both endpoints or corroborating major ticks")
    if list(major.values()) != sorted(set(major.values())):
        raise UnreadableGauge("ambiguous humidity major tick order")
    lo, hi = ticks[major[0]].column, ticks[major[100]].column
    at = _unwrap_pointer(pointer, lo, hi)
    nearest = min(major, key=lambda label: abs(ticks[major[label]].column - at))
    # A clearly visible major stroke can be read directly despite gaps elsewhere.
    if abs(at - ticks[major[nearest]].column) <= 1.0 / _STEP:
        return float(nearest), len(major), "humidity_auto_scale_local"
    for label in range(0, 100, 10):
        if label in major and label + 10 in major:
            first, last = major[label], major[label + 10]
            if ticks[first].column < at < ticks[last].column:
                segment = _label_interval(ticks[first:last + 1], label)
                return _value(segment, at), len(segment), "humidity_auto_scale_local"
    value, count = _occluded_tick(ticks, major, approx, arc, at)
    return value, count, "humidity_auto_scale_occluded"


def _unwrap_pointer(pointer, first, last):
    # Never extrapolate beyond the uncertainty in locating a visible endpoint.
    tolerance = 1 / _STEP
    pointer = (pointer - first + tolerance) % _N + first - tolerance
    if pointer > last + tolerance:
        raise UnreadableGauge("humidity pointer outside observed scale")
    return pointer


def _value(scale, pointer):
    columns, values = np.asarray(scale).T
    pointer = _unwrap_pointer(pointer, columns[0], columns[-1])
    if pointer <= columns[0]:
        return float(values[0])
    if pointer >= columns[-1]:
        return float(values[-1])
    right = int(np.searchsorted(columns, pointer))
    if values[right] - values[right - 1] != 2:
        if abs(pointer - columns[right - 1]) <= .5 / _STEP:
            return float(values[right - 1])
        if abs(pointer - columns[right]) <= .5 / _STEP:
            return float(values[right])
        raise UnreadableGauge("humidity pointer between incomplete minor ticks")
    fraction = (pointer - columns[right - 1]) / (columns[right] - columns[right - 1])
    return round(float(values[right - 1] + 2 * fraction), 1)


def _pointer(gray, center, arc):
    start, stop = round(.24 * arc.radius), round(.96 * arc.radius)
    radii = np.arange(start, stop + 1)
    strip, valid = _sample(gray, center, radii[:, None])
    strip = strip.astype(np.float32)
    # Compare against the rest of the same radial level; text near the pointer
    # is often as dark as its stroke but cannot sustain a ray from the hub.
    background = np.nanmedian(np.where(valid, strip, np.nan), axis=1)
    darkness = background[:, None] - strip
    dark = ((darkness >= 29) & valid).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((3, 1), np.uint8)).astype(bool)
    search_end = round(.76 * arc.radius) - start
    support = dark[:search_end].mean(axis=0)
    options = []
    for col in np.flatnonzero(support >= .84):
        for lo, hi in _runs(dark[:, col]):
            if lo <= 2 and start + hi - 1 >= .72 * arc.radius:
                strength = float(np.maximum(darkness[:search_end, col], 0).mean())
                options.append((strength, col, start + hi - 1))
                break
    if not options:
        raise UnreadableGauge("no supported humidity pointer tip")
    strength, best, _ = max(options)
    distance = lambda col: min(abs(col - best), _N - abs(col - best)) * _STEP
    if any(distance(col) > 5 and score >= .80 * strength for score, col, _ in options):
        raise UnreadableGauge("ambiguous humidity pointer directions")
    # The broad base of a tapered hand can be nearly flat in score. Use the
    # outer half of the supported ray to center on the thin tip instead.
    finalists = [(col, tip) for score, col, tip in options
                 if distance(col) <= 3 and score >= .88 * strength]
    last = round(min(tip for _, tip in finalists) - start)
    rank = [(float(darkness[max(0, last - 11):last, col].mean()), col, tip)
            for col, tip in finalists]
    best_tip_score = max(rank)[0]
    tips = [(best + (col - best + _N // 2) % _N - _N // 2, tip)
            for score, col, tip in rank if score >= best_tip_score - 4]
    return float(np.mean([col for col, _ in tips])) % _N, round(float(np.median([tip for _, tip in tips])))


def read_humidity_auto_scale(image: np.ndarray) -> CropReading:
    """Read a native uint8 BGR crop, or raise :class:`UnreadableGauge`.

    Geometry is in native crop pixels; radius reaches the observed pointer tip.
    Interpolate only between adjacent observed 2 %RH graduations. For a
    uniquely occluded minor mark, return the discrete graduation instead.
    The reported decimal place is not an accuracy guarantee.
    """
    if (not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3 or
            image.dtype != np.uint8 or min(image.shape[:2]) < 90):
        raise UnreadableGauge("invalid humidity crop")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    arcs = [(center, arc) for center in _hubs(gray) if (arc := _arc(image, center)) is not None]
    if not arcs:
        raise UnreadableGauge("no fully visible colored humidity scale")
    readings = []
    error = None
    for center, arc in arcs:
        ticks = _ticks(gray, center, arc)
        try:
            method = "humidity_auto_scale"
            try:
                scale = _scale(ticks, arc)
            except UnreadableGauge as full_error:
                try:
                    col, tip = _pointer(gray, center, arc)
                    value, count, method = _local_scale(ticks, arc, col)
                except UnreadableGauge:
                    raise full_error from None
            else:
                col, tip = _pointer(gray, center, arc)
                value, count = _value(scale, col), len(scale)
            readings.append(CropReading(value, center, tip, (90.0 - col * _STEP) % 360,
                                        count, method))
        except UnreadableGauge as exc:
            error = exc
    if not readings:
        raise error
    if len({reading.value for reading in readings}) != 1:
        raise UnreadableGauge("ambiguous humidity scale geometry")
    return readings[0]
