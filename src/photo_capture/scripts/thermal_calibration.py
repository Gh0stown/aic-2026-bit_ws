#!/usr/bin/env python3
"""Pure IR-to-RGB planar calibration, with no Tk/ROS or device access.

Point records are always [row, col, u, v]; project(H, col, row) deliberately
uses OpenCV's (x, y) order instead. All coordinates are native image pixels,
not preview pixels. A plane homography depends on camera/target-plane pose
and distance; vehicle motion can invalidate it even with a rigid camera pair.
"""
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import numbers
import os
from pathlib import Path
import tempfile
from typing import Tuple

import cv2
import numpy as np


__all__ = ['Calibration', 'project', 'rms', 'fit_calibration',
           'save_calibration', 'load_calibration']
IR_SIZE = (32, 24)


@dataclass(frozen=True)
class Calibration:
    """Native-pixel calibration; points contain only (row, col, u, v).

    rms_px is recomputed from H/points when saving or loading. scale is preview
    metadata only, never a factor in H. A large RMS is valid data for a caller
    to warn about, not a reason to silently substitute another calibration.
    """
    points: Tuple[Tuple[float, float, float, float], ...]
    H: np.ndarray
    rms_px: float
    rgb_size: Tuple[int, int]
    ir_size: Tuple[int, int] = IR_SIZE
    mirror_x: bool = True
    scale: float = 12
    note: str = ''
    created: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())


def _size(value, name):
    try:
        valid = len(value) == 2 and all(
            isinstance(x, numbers.Integral) and not isinstance(x, (bool, np.bool_))
            and x > 0 for x in value)
    except TypeError:
        valid = False
    if not valid:
        raise ValueError('%s must be [width, height] positive integers' % name)
    return tuple(int(x) for x in value)


def _number(value, name):
    if (not isinstance(value, numbers.Real)
            or isinstance(value, (bool, np.bool_)) or not math.isfinite(value)):
        raise ValueError('%s must be a finite number' % name)
    return float(value)


def _matrix(H):
    try:
        raw = np.asarray(H)
        if raw.dtype.kind not in 'iuf' or raw.shape != (3, 3):
            raise ValueError('H must be a finite numeric 3x3 matrix')
        matrix = raw.astype(np.float64)
        if not np.isfinite(matrix).all():
            raise ValueError('H must be a finite numeric 3x3 matrix')
        magnitude = float(np.max(np.abs(matrix)))
        if magnitude == 0:
            raise ValueError('H is singular')
        # Homographies are equivalent up to any nonzero scalar.
        matrix = matrix / magnitude
        if np.linalg.matrix_rank(matrix) != 3:
            raise ValueError('H is singular')
        if abs(matrix[2, 2]) > np.finfo(float).eps:
            matrix = matrix / matrix[2, 2]
    except (TypeError, OverflowError, np.linalg.LinAlgError) as exc:
        raise ValueError('H must be a nonsingular finite 3x3 matrix') from exc
    return matrix


def _point_array(points, minimum=4):
    try:
        raw = np.asarray(points)
        if (raw.ndim != 2 or raw.shape[1] != 4 or len(raw) < minimum
                or raw.dtype.kind not in 'iuf'):
            raise ValueError('points must contain at least %d [row,col,u,v] rows' % minimum)
        result = raw.astype(np.float64)
        if not np.isfinite(result).all():
            raise ValueError('points coordinates must be finite')
    except (TypeError, OverflowError) as exc:
        raise ValueError('points must be numeric [row,col,u,v] rows') from exc
    return result


def _points(points, ir_size, rgb_size):
    result = _point_array(points)
    upper = np.array([ir_size[1] - 1, ir_size[0] - 1,
                      rgb_size[0] - 1, rgb_size[1] - 1])
    if (result < 0).any() or (result > upper).any():
        raise ValueError('points coordinates outside ir_size or rgb_size pixel bounds')
    for indices, name in (([1, 0], 'IR'), ([2, 3], 'RGB')):
        xy = result[:, indices]
        if len(np.unique(xy, axis=0)) != len(xy):
            raise ValueError('%s points contain duplicate coordinates' % name)
        if np.linalg.matrix_rank(xy - xy.mean(axis=0)) < 2:
            raise ValueError('%s points are collinear' % name)
    return result


def _project_many(H, xy):
    homogeneous = np.column_stack((xy, np.ones(len(xy))))
    with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
        projected = homogeneous @ H.T
        if not np.isfinite(projected).all():
            raise ValueError('projection is non-finite')
        # Relative test keeps the zero-denominator check invariant to H scale.
        # Sum row products rather than vector norms (which can overflow even
        # for a perfectly finite projection of very large coordinates).
        tolerance = np.sum(np.abs(homogeneous * (H[2] * np.finfo(float).eps)), axis=1)
        if (np.abs(projected[:, 2]) <= tolerance).any():
            raise ValueError('projection denominator is zero (point on homography horizon)')
        result = projected[:, :2] / projected[:, 2, None]
    if not np.isfinite(result).all():
        raise ValueError('projection is non-finite')
    return result


def project(H, col, row):
    """Return subpixel (u, v) for native IR (col, row); never round or clip.

    Raises ValueError for invalid/singular H, non-finite input, or projection
    at infinity. No image bounds are imposed here: off-image predictions are
    useful to the caller, which must decide how to display them.
    """
    xy = np.array([[_number(col, 'col'), _number(row, 'row')]])
    result = _project_many(_matrix(H), xy)[0]
    return float(result[0]), float(result[1])


def rms(H, points):
    """sqrt(mean(||project(H,col,row) - (u,v)||**2)), in RGB pixels."""
    records = _point_array(points, minimum=1)
    predicted = _project_many(_matrix(H), records[:, [1, 0]])
    errors = predicted - records[:, 2:4]
    # hypot avoids unnecessary overflow when squaring a large finite error.
    lengths = np.hypot(errors[:, 0], errors[:, 1])
    peak = float(np.max(lengths))
    value = peak * math.sqrt(float(np.mean((lengths / peak) ** 2))) if peak else 0.0
    if not math.isfinite(value):
        raise ValueError('reprojection RMS is non-finite')
    return value


def _validated(points, H, rgb_size, ir_size, mirror_x, scale, note, created):
    ir_size = _size(ir_size, 'ir_size')
    rgb_size = _size(rgb_size, 'rgb_size')
    if type(mirror_x) is not bool:
        raise ValueError('mirror_x must be a boolean')
    scale = _number(scale, 'scale')
    if scale <= 0:
        raise ValueError('scale must be > 0')
    if not isinstance(note, str):
        raise ValueError('note must be a string')
    if not isinstance(created, str) or not created:
        raise ValueError('created must be an ISO datetime string')
    try:
        datetime.fromisoformat(created.replace('Z', '+00:00'))
    except ValueError as exc:
        raise ValueError('created must be an ISO datetime string') from exc
    records = _points(points, ir_size, rgb_size)
    matrix = _matrix(H)
    error = rms(matrix, records)
    return Calibration(tuple(tuple(float(x) for x in row) for row in records),
                       matrix, error, rgb_size, ir_size, mirror_x, scale, note, created)


def fit_calibration(points, rgb_size, mirror_x=True, scale=12, note=''):
    """Fit all >=4 [row,col,u,v] pairs using cv2.findHomography(..., 0).

    IR bounds are fixed at (32,24). Neither preview scale nor mirror_x alters
    coordinates: supplied IR pixels must already use the requested mirroring.
    """
    rgb_size = _size(rgb_size, 'rgb_size')
    records = _points(points, IR_SIZE, rgb_size)
    try:
        H, _ = cv2.findHomography(records[:, [1, 0]], records[:, 2:4], 0)
    except cv2.error as exc:
        raise ValueError('homography fit failed: %s' % exc) from exc
    if H is None:
        raise ValueError('homography fit failed: degenerate point geometry')
    return _validated(records, H, rgb_size, IR_SIZE, mirror_x, scale, note,
                      datetime.now(timezone.utc).isoformat())


def save_calibration(path, calibration):
    """Validate then atomically replace JSON; an error leaves the old file intact.

    The destination directory must already exist. Filesystem errors propagate
    as OSError; invalid calibration data raises ValueError before writing.
    """
    if not isinstance(calibration, Calibration):
        raise ValueError('calibration must be a Calibration object')
    value = _validated(calibration.points, calibration.H, calibration.rgb_size,
                       calibration.ir_size, calibration.mirror_x, calibration.scale,
                       calibration.note, calibration.created)
    payload = dict(version=1, H=value.H.tolist(), points=value.points,
                   rms_px=value.rms_px, ir_size=value.ir_size,
                   rgb_size=value.rgb_size, mirror_x=value.mirror_x,
                   scale=value.scale, note=value.note, created=value.created)
    text = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    destination = Path(path).expanduser()
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                         dir=str(destination.parent),
                                         prefix='.%s.' % destination.name,
                                         suffix='.tmp', delete=False) as stream:
            temporary = stream.name
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
        temporary = None
    finally:
        if temporary is not None:
            os.unlink(temporary)


def _reject_json_constant(value):
    raise ValueError('JSON contains non-finite constant: %s' % value)


def load_calibration(path, rgb_size, mirror_x=True, ir_size=IR_SIZE):
    """Load v1 JSON only when image sizes/mirroring match the current session.

    Recompute RMS from H/points rather than trusting the stored error. Finite,
    nonsingular H and finite point projections are required, but a high actual
    RMS is returned for GUI warnings rather than rejected by a hard threshold.
    """
    expected_ir = _size(ir_size, 'ir_size')
    expected_rgb = _size(rgb_size, 'rgb_size')
    if type(mirror_x) is not bool:
        raise ValueError('mirror_x must be a boolean')
    try:
        with Path(path).expanduser().open(encoding='utf-8') as stream:
            payload = json.load(stream, parse_constant=_reject_json_constant)
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise ValueError('invalid calibration JSON: %s' % exc) from exc
    if not isinstance(payload, dict):
        raise ValueError('calibration JSON must be an object')
    if type(payload.get('version')) is not int or payload['version'] != 1:
        raise ValueError('calibration version must be 1')
    required = {'H', 'points', 'rms_px', 'ir_size', 'rgb_size', 'mirror_x',
                'scale', 'created', 'note'}
    missing = required - payload.keys()
    if missing:
        raise ValueError('calibration missing fields: %s' % ', '.join(sorted(missing)))
    for name, expected in (('ir_size', expected_ir), ('rgb_size', expected_rgb)):
        actual = _size(payload[name], name)
        if actual != expected:
            raise ValueError('%s mismatch: file %s, current %s' % (name, actual, expected))
    if type(payload['mirror_x']) is not bool:
        raise ValueError('mirror_x must be a boolean')
    if payload['mirror_x'] != mirror_x:
        raise ValueError('mirror_x mismatch: file %s, current %s' %
                         (payload['mirror_x'], mirror_x))
    if _number(payload['rms_px'], 'rms_px') < 0:
        raise ValueError('rms_px must be nonnegative')
    return _validated(payload['points'], payload['H'], expected_rgb, expected_ir,
                      mirror_x, payload['scale'], payload['note'], payload['created'])
