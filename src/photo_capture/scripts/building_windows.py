#!/usr/bin/env python3
"""Building-pose geometry and independent window templates; no IR/device access.

Coordinates are native RGB pixels (never preview pixels). Corners are always
TL, TR, BR, BL, without sorting or mirroring. The rectified facade uses (s,t)
in [0,1], with t increasing downwards. Matching returns geometry candidates,
not a temperature/fire classification. This module never changes IR-to-RGB H.
"""
from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import json
import math
import numbers
import os
from pathlib import Path
import re
import tempfile
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from thermal_calibration import project
from yolo_detect import YoloDetector


FLOORS = ('1F', '2F', '3F', '4F')
CORNER_ORDER = ('TL', 'TR', 'BR', 'BL')
_PLANE = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)


@dataclass(frozen=True)
class WindowTemplate:
    centers: Dict[str, Tuple[float, float]]
    reference_corners: Tuple[Tuple[float, float], ...]
    reference_centers: Dict[str, Tuple[float, float]]
    rgb_size: Tuple[int, int]
    rgb_mirror: bool = False
    model_filename: str = ''
    model_sha256: str = ''
    created: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())


@dataclass(frozen=True)
class BuildingDetection:
    corners: np.ndarray
    confidence: float
    box: Tuple[float, float, float, float]
    corner_conf: Tuple[float, float, float, float]


def _number(value, name):
    if (not isinstance(value, numbers.Real) or isinstance(value, (bool, np.bool_))
            or not math.isfinite(value)):
        raise ValueError('%s must be finite numeric data' % name)
    return float(value)


def _array(value, shape, name):
    try:
        raw = np.asarray(value)
        if (raw.shape != shape or raw.dtype.kind not in 'iuf'
                or any(isinstance(x, (bool, np.bool_)) for x in np.asarray(value, dtype=object).flat)):
            raise ValueError('%s must be numeric with shape %s' % (name, shape))
        result = raw.astype(np.float64)
        if not np.isfinite(result).all():
            raise ValueError('%s must be finite' % name)
    except (TypeError, OverflowError) as exc:
        raise ValueError('%s must be finite numeric data' % name) from exc
    return result


def _corners(value):
    corners = _array(value, (4, 2), 'corners')
    # Positive screen-space winding is TL->TR->BR->BL. Strict convexity also
    # rules out duplicate/collinear vertices, concavity and self-intersection.
    span = np.max(np.abs(corners - corners[0]))
    if not math.isfinite(float(span)) or span == 0:
        raise ValueError('corners must form a positive-area convex TL TR BR BL quadrilateral')
    edges = (np.roll(corners, -1, axis=0) - corners) / span
    turns = edges[:, 0] * np.roll(edges[:, 1], -1) - edges[:, 1] * np.roll(edges[:, 0], -1)
    if not np.isfinite(turns).all() or not (turns > 1e-10).all():
        raise ValueError('corners must form a positive-area convex TL TR BR BL quadrilateral')
    return corners


def _homography(corners, inverse=False):
    corners = _corners(corners)
    with np.errstate(over='ignore', invalid='ignore'):
        image = corners.astype(np.float32)
    if not np.isfinite(image).all():
        raise ValueError('corners exceed OpenCV coordinate range')
    try:
        H = cv2.getPerspectiveTransform(_PLANE, image) if inverse else cv2.getPerspectiveTransform(image, _PLANE)
        # Validate at an interior source point, never at an arbitrary pixel
        # which might lie on the (off-facade) homography horizon.
        source_center = (0.5, 0.5) if inverse else tuple(corners.mean(axis=0))
        project(H, *source_center)
    except (cv2.error, ValueError) as exc:
        raise ValueError('invalid building homography: %s' % exc) from exc
    return H


def to_plane(corners, u, v) -> Tuple[float, float]:
    """Project RGB (u,v) onto the facade; off-facade points are not clipped."""
    return project(_homography(corners), u, v)


def to_image(corners, s, t) -> Tuple[float, float]:
    """Project normalized facade (s,t) to RGB, without rounding/clipping."""
    return project(_homography(corners, inverse=True), s, t)


def _size(value):
    try:
        if len(value) == 2 and all(isinstance(x, numbers.Integral)
                and not isinstance(x, (bool, np.bool_)) and x > 0 for x in value):
            return tuple(int(x) for x in value)
    except TypeError:
        pass
    raise ValueError('rgb_size must be positive integer (width, height)')


def _centers(value, plane=False):
    if not isinstance(value, dict) or set(value) != set(FLOORS):
        raise ValueError('centers must contain exactly 1F, 2F, 3F, 4F')
    points = _array([value[floor] for floor in FLOORS], (4, 2), 'centers')
    if len(np.unique(points, axis=0)) != 4:
        raise ValueError('centers must not be duplicate')
    if plane:
        if not ((points > 0) & (points < 1)).all():
            raise ValueError('centers must be strictly inside the facade')
        if not (np.diff(points[:, 1]) < 0).all():
            raise ValueError('1F to 4F center t must be strictly decreasing')
    return {floor: tuple(float(x) for x in point) for floor, point in zip(FLOORS, points)}


def _fingerprint(model_path):
    if model_path == '':
        return '', ''
    path = Path(model_path).expanduser()
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return path.name, digest.hexdigest()


def _validated(template):
    if not isinstance(template, WindowTemplate):
        raise ValueError('template must be a WindowTemplate')
    corners = _corners(template.reference_corners)
    centers = _centers(template.centers, plane=True)
    reference = _centers(template.reference_centers)
    H = _homography(corners)
    recovered = _centers({floor: project(H, *reference[floor]) for floor in FLOORS}, plane=True)
    if not np.allclose(list(recovered.values()), list(centers.values()), atol=1e-7, rtol=0):
        raise ValueError('reference_centers disagree with normalized centers')
    size = _size(template.rgb_size)
    upper = np.array([size[0] - 1, size[1] - 1])
    reference_pixels = np.array(list(reference.values()))
    if (corners < 0).any() or (corners > upper).any() or (reference_pixels < 0).any() or (reference_pixels > upper).any():
        raise ValueError('reference corners/window centers outside rgb_size pixel bounds')
    if type(template.rgb_mirror) is not bool:
        raise ValueError('rgb_mirror must be a boolean')
    filename, sha = template.model_filename, template.model_sha256
    if (not isinstance(filename, str) or not isinstance(sha, str)
            or bool(filename) != bool(sha) or '/' in filename or '\\' in filename
            or filename in ('.', '..') or (sha and re.fullmatch('[0-9a-f]{64}', sha) is None)):
        raise ValueError('invalid model filename/sha256 fingerprint')
    if not isinstance(template.created, str) or not template.created:
        raise ValueError('created must be an ISO datetime string')
    try:
        datetime.fromisoformat(template.created.replace('Z', '+00:00'))
    except ValueError as exc:
        raise ValueError('created must be an ISO datetime string') from exc
    return WindowTemplate(centers, tuple(map(tuple, corners.tolist())), reference,
                          size, template.rgb_mirror, filename, sha, template.created)


def make_template(corners, centers, rgb_size, rgb_mirror=False, model_path='') -> WindowTemplate:
    """Capture four RGB window centers once, in 1F (bottom) to 4F order.

    model_path, when supplied, must exist locally; its filename and SHA256 bind
    the template to those weights. Empty means an explicitly unbound template;
    load_template must then also receive an empty model_path.
    """
    corners = _corners(corners)
    reference = _centers(centers)
    H = _homography(corners)
    normalized = {floor: project(H, *reference[floor]) for floor in FLOORS}
    filename, sha = _fingerprint(model_path)
    return _validated(WindowTemplate(normalized, tuple(map(tuple, corners.tolist())),
                                     reference, rgb_size, rgb_mirror, filename, sha))


def template_centers(template, corners) -> Dict[str, Tuple[float, float]]:
    """Reproject the stored normalized centers onto a new RGB facade pose."""
    value = _validated(template)
    H = _homography(corners, inverse=True)
    return {floor: project(H, *value.centers[floor]) for floor in FLOORS}


def match_floor(template, corners, u, v) -> Optional[str]:
    """Return only a nearby geometric floor candidate, never a hot-spot verdict.

    The gate is 0.45 times the minimum distance between consecutive floors,
    measured in normalized facade Euclidean coordinates, not RGB pixels.
    """
    value = _validated(template)
    point = np.asarray(to_plane(corners, u, v))
    if not ((point >= 0) & (point <= 1)).all():
        return None
    centers = np.asarray([value.centers[floor] for floor in FLOORS])
    spacing = float(np.min(np.linalg.norm(np.diff(centers, axis=0), axis=1)))
    distances = np.linalg.norm(centers - point, axis=1)
    index = int(np.argmin(distances))
    return FLOORS[index] if distances[index] <= 0.45 * spacing else None


def save_template(path, template):
    """Validate, fsync and atomically replace a standalone v1 JSON file.

    The directory must exist. Any failure before replacement preserves the old
    file; filesystem errors propagate. No calibration.json is read or changed.
    """
    value = _validated(template)
    payload = dict(version=1, corner_order=CORNER_ORDER, floor_order=FLOORS,
                   centers=value.centers, reference_corners=value.reference_corners,
                   reference_centers=value.reference_centers, rgb_size=value.rgb_size,
                   rgb_mirror=value.rgb_mirror, model_filename=value.model_filename,
                   model_sha256=value.model_sha256, created=value.created)
    text = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    destination = Path(path).expanduser()
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=str(destination.parent),
                                         prefix='.%s.' % destination.name, suffix='.tmp', delete=False) as stream:
            temporary = stream.name
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
        temporary = None
    finally:
        if temporary is not None:
            os.unlink(temporary)


def _reject_constant(value):
    raise ValueError('JSON contains non-finite constant: %s' % value)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON key: %s' % key)
        result[key] = value
    return result


def load_template(path, rgb_size, rgb_mirror=False, model_path='') -> WindowTemplate:
    """Load only matching size, RGB orientation, model fingerprint and orders."""
    size = _size(rgb_size)
    if type(rgb_mirror) is not bool:
        raise ValueError('rgb_mirror must be a boolean')
    try:
        with Path(path).expanduser().open(encoding='utf-8') as stream:
            payload = json.load(stream, parse_constant=_reject_constant, object_pairs_hook=_unique_object)
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise ValueError('invalid window template JSON: %s' % exc) from exc
    if not isinstance(payload, dict):
        raise ValueError('window template JSON must be an object')
    if type(payload.get('version')) is not int or payload['version'] != 1:
        raise ValueError('window template version must be 1')
    fields = {'centers', 'reference_corners', 'reference_centers', 'rgb_size',
              'rgb_mirror', 'model_filename', 'model_sha256', 'created'}
    if set(payload) != fields | {'version', 'corner_order', 'floor_order'}:
        raise ValueError('window template has missing or unexpected fields')
    if payload['corner_order'] != list(CORNER_ORDER) or payload['floor_order'] != list(FLOORS):
        raise ValueError('corner_order/floor_order mismatch')
    value = _validated(WindowTemplate(**{key: payload[key] for key in fields}))
    if value.rgb_size != size:
        raise ValueError('rgb_size mismatch')
    if value.rgb_mirror != rgb_mirror:
        raise ValueError('rgb_mirror mismatch')
    if (value.model_filename, value.model_sha256) != _fingerprint(model_path):
        raise ValueError('model fingerprint mismatch')
    return value


class BuildingPoseDetector(YoloDetector):
    """Same submit/latest/close thread API, but latest carries BuildingDetection.

    Only local pose weights with four confidence-bearing corners are accepted.
    No real model is loaded merely by importing this module.
    """
    def __init__(self, model_path, device_pref='auto', imgsz=960, conf=0.5):
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise ValueError('building pose model must be an existing local file')
        super().__init__(str(path), device_pref=device_pref, imgsz=imgsz, conf=conf)
        try:
            inner = getattr(self.model, 'model', None)
            shape = getattr(inner, 'kpt_shape', None)
            if shape is None:
                shape = (getattr(inner, 'yaml', None) or {}).get('kpt_shape')
            if shape is None:
                layers = getattr(inner, 'model', None)
                if layers is not None and len(layers):
                    shape = getattr(layers[-1], 'kpt_shape', None)
            if getattr(self.model, 'task', None) != 'pose' or tuple(shape if shape is not None else ()) != (4, 3):
                raise ValueError('building model must have task pose and kpt_shape [4, 3]')
        except Exception as exc:
            self.close()  # The parent constructor has already started its worker.
            if isinstance(exc, ValueError):
                raise
            raise ValueError('cannot validate building pose model') from exc

    @staticmethod
    def _extract_boxes(result) -> List[BuildingDetection]:
        boxes = getattr(result, 'boxes', None)
        keypoints = getattr(result, 'keypoints', None)
        if boxes is None or keypoints is None:
            return []
        try:
            xyxy = np.asarray(boxes.xyxy.cpu().numpy(), dtype=float)
            confs = np.asarray(boxes.conf.cpu().numpy(), dtype=float)
            points = np.asarray(keypoints.xy.cpu().numpy(), dtype=float)
            corner_confs = np.asarray(keypoints.conf.cpu().numpy(), dtype=float)
        except (AttributeError, TypeError, ValueError):
            return []
        count = len(confs) if confs.ndim == 1 else 0
        if (xyxy.shape != (count, 4) or points.shape != (count, 4, 2)
                or corner_confs.shape != (count, 4)):
            return []
        output = []
        for box, conf, corners, scores in zip(xyxy, confs, points, corner_confs):
            if (not np.isfinite(box).all() or not math.isfinite(conf)
                    or not np.isfinite(scores).all() or not 0.5 <= conf <= 1
                    or not ((scores >= 0.5) & (scores <= 1)).all()
                    or box[2] <= box[0] or box[3] <= box[1]):
                continue
            try:
                corners = _corners(corners)
            except ValueError:
                continue
            output.append(BuildingDetection(corners.copy(), float(conf), tuple(map(float, box)),
                                            tuple(map(float, scores))))
        return output
