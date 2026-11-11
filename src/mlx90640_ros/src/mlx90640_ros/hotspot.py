"""Hotspot extraction and row-to-floor mapping for the 32x24 thermal frame.

The MLX90640 has no depth channel: this module only answers "where is the
hottest pixel in the thermal image".  Mapping that pixel to a real-world
position needs a separate calibration (camera mount, distance, or a
thermal-to-visible homography) and is deliberately out of scope here.

Image coordinates keep the sensor orientation: row 0..23 top to bottom,
col 0..31 left to right, exactly as ``temperature_image`` publishes them.
"""

from dataclasses import dataclass
import math
import re

import numpy as np

ROWS, COLS = 24, 32
SEARCH_ROI = (0, 0, ROWS, COLS)


@dataclass(frozen=True)
class Hotspot:
    """One hottest region: subpixel centre, integer peak and bounding box."""

    peak_c: float
    row: float
    col: float
    row_index: int
    col_index: int
    box: tuple
    area: int
    mean_c: float
    threshold_c: float


def parse_roi(text):
    """Parse ``r0,c0,r1,c1`` (half-open, clipped to the image) or return None."""
    normalized = str(text or "").strip()
    if not normalized:
        return None
    parts = [part.strip() for part in normalized.split(",")]
    if len(parts) != 4:
        raise ValueError("~roi must be r0,c0,r1,c1")
    try:
        r0, c0, r1, c1 = (int(part) for part in parts)
    except ValueError as exc:
        raise ValueError("~roi must contain four integers") from exc
    if not (0 <= r0 < r1 <= ROWS and 0 <= c0 < c1 <= COLS):
        raise ValueError("~roi must satisfy 0<=r0<r1<=24 and 0<=c0<c1<=32")
    return r0, c0, r1, c1


def _validate_temps(temps):
    array = np.asarray(temps, dtype=float)
    if array.shape != (ROWS, COLS):
        raise ValueError("temperature image must be 24 rows by 32 columns")
    if not np.isfinite(array).all():
        raise ValueError("temperature image must be finite")
    return array


def _component(mask, seed):
    """8-connected component containing ``seed`` (flood fill, no SciPy)."""
    height, width = mask.shape
    stack = [seed]
    seen = {seed}
    while stack:
        row, col = stack.pop()
        for drow in (-1, 0, 1):
            for dcol in (-1, 0, 1):
                if drow == 0 and dcol == 0:
                    continue
                nr, nc = row + drow, col + dcol
                if (0 <= nr < height and 0 <= nc < width
                        and mask[nr, nc] and (nr, nc) not in seen):
                    seen.add((nr, nc))
                    stack.append((nr, nc))
    return seen


def _subpixel_peak(temps, row, col):
    """Fit z=a x^2+b y^2+c xy+d x+e y+f on the 3x3 window around a peak.

    Falls back to the integer pixel when the window is clipped at an image
    border or the fitted surface is not a maximum.  Offsets are clamped to
    +/-0.75 px so a bad fit cannot jump into a neighbouring pixel.
    """
    r0, r1 = max(row - 1, 0), min(row + 2, ROWS)
    c0, c1 = max(col - 1, 0), min(col + 2, COLS)
    if r1 - r0 < 3 or c1 - c0 < 3:
        return float(row), float(col)
    values = temps[r0:r1, c0:c1]
    ys, xs = np.mgrid[0:3, 0:3]
    design = np.column_stack([
        xs.ravel() ** 2, ys.ravel() ** 2, (xs * ys).ravel(),
        xs.ravel(), ys.ravel(), np.ones(9)])
    try:
        coeff, *_ = np.linalg.lstsq(design, values.ravel(), rcond=None)
    except np.linalg.LinAlgError:
        return float(row), float(col)
    a, b, c, d, e, _ = coeff
    det = 4.0 * a * b - c * c
    if det <= 1e-12 or a >= 0:
        return float(row), float(col)
    # The fit solves in window-local coordinates 0..2 where 1 is the centre.
    dx = (c * e - 2.0 * b * d) / det - 1.0
    dy = (c * d - 2.0 * a * e) / det - 1.0
    if not (math.isfinite(dx) and math.isfinite(dy)):
        return float(row), float(col)
    dx = float(np.clip(dx, -0.75, 0.75))
    dy = float(np.clip(dy, -0.75, 0.75))
    return float(row) + dy, float(col) + dx


def find_hotspot(temps, delta_c=2.0, roi=None):
    """Return the hottest region of a 24x32 Celsius frame.

    ``delta_c`` is the temperature drop below the peak that still counts as
    the same hot region.  ``roi`` is an optional half-open ``(r0, c0, r1, c1)``
    limiter (see :func:`parse_roi`) used to ignore background.
    """
    array = _validate_temps(temps)
    delta = float(delta_c)
    if not math.isfinite(delta) or delta < 0:
        raise ValueError("delta_c must be finite and >= 0")
    if roi is None:
        roi = SEARCH_ROI
    r0, c0, r1, c1 = roi

    window = array[r0:r1, c0:c1]
    local = int(np.argmax(window))
    peak_row, peak_col = divmod(local, c1 - c0)
    peak_row += r0
    peak_col += c0
    peak_c = float(array[peak_row, peak_col])

    threshold = peak_c - delta
    mask = np.zeros((ROWS, COLS), dtype=bool)
    region = array[r0:r1, c0:c1] >= threshold
    mask[r0:r1, c0:c1] = region
    pixels = _component(mask, (peak_row, peak_col))

    rows = [p[0] for p in pixels]
    cols = [p[1] for p in pixels]
    box = (min(rows), min(cols), max(rows), max(cols))
    mean_c = float(np.mean([array[p] for p in pixels]))
    row, col = _subpixel_peak(array, peak_row, peak_col)
    return Hotspot(peak_c=peak_c, row=row, col=col,
                   row_index=peak_row, col_index=peak_col,
                   box=box, area=len(pixels), mean_c=mean_c,
                   threshold_c=threshold)


_BAND = re.compile(r"^\s*([^:]+?)\s*:\s*(\d+)\s*-\s*(\d+)\s*$")


def parse_floor_bands(text):
    """Parse ``1F:16-23,2F:8-15,3F:0-7`` into validated ``(name, r0, r1)``.

    Bands are inclusive and must not overlap.  Empty text yields no bands,
    which makes the node report an empty floor instead of guessing.
    """
    normalized = str(text or "").strip()
    if not normalized:
        return ()
    bands = []
    for chunk in re.split(r"[,;]", normalized):
        if not chunk.strip():
            continue
        match = _BAND.match(chunk)
        if not match:
            raise ValueError("floor band {!r} must look like 2F:8-15".format(chunk.strip()))
        name, first, last = match.group(1).strip(), int(match.group(2)), int(match.group(3))
        if not (0 <= first <= last < ROWS):
            raise ValueError("floor band {!r} must satisfy 0<=r0<=r1<=23".format(chunk.strip()))
        bands.append((name, first, last))
    bands.sort(key=lambda band: band[1])
    for earlier, later in zip(bands, bands[1:]):
        if later[1] <= earlier[2]:
            raise ValueError("floor bands {} and {} overlap".format(earlier[0], later[0]))
    return tuple(bands)


def floor_for_row(row, bands):
    """Return the band containing ``row``, else the nearest band by centre.

    Nearest instead of None keeps a slightly mis-calibrated row from
    producing an empty answer; callers that need strictness can compare
    ``row`` against ``band[1] <= row <= band[2]`` themselves.
    """
    if not bands:
        return ""
    value = float(row)
    for name, first, last in bands:
        if first <= value <= last:
            return name
    return min(bands, key=lambda band: abs(value - (band[1] + band[2]) / 2.0))[0]
