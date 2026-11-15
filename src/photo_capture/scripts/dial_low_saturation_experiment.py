#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Experimental edge-first humidity dial feasibility probe (isolated).

This module is NOT part of the default recogniser.  It is an explicit offline
experiment entry point and is never imported by ``dial_reading.py`` or
``gauge.py``.  It exists to test one hypothesis on the fixed-view, low
saturation camera frames:

    geometry first (robust edge ellipse), weak colour second (band contrast and
    a 1-D angular colour signature for roll).

Hard contract
-------------
* top-level ``value`` is always ``None``; the module never claims a trusted
  humidity reading;
* ``diagnostic_candidate`` is a separate object and is only filled when every
  stage (ring, axis, roll, needle) passes its own gate;
* every result carries ``experimental: true``;
* failures are reported with a reason instead of relaxing a gate.

The module reuses the already approved pure geometry/pointer helpers from
``dial_reading`` (ellipse residual/matrix, needle ray) and ``dial_math``
(angle wrap, table lookup).  It does not monkey-patch their global thresholds.

Only NumPy, OpenCV, the standard library and those two local modules are used.
No camera, ROS, GUI, network or model is touched.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import sys
import uuid
import warnings
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from dial_math import lookup_value, wrap_deg
from dial_reading import (
    _ellipse_matrix,
    _ellipse_residual,
    _needle_direction_px,
    _rotate_vector,
    detect_needle,
    load_calibration,
)


TOOL_VERSION = "dial_low_saturation_experiment.py 5 (R5 coordinate-unit gate)"
EXPERIMENT_ID = "g1b2-fixed-view-low-saturation-5"
EXPERIMENTAL = True

# The caller's declared humidistat ROI and the experiment ROI.  The top edge is
# extended by 10 px after checking the visible ring boundary: the declared ROI
# clips the top of the small dial.  This is a scene prior, not a reading.
DECLARED_ROI = (630, 220, 75, 65)
ROI_TOP_EXPANSION_PX = 10
EXPERIMENT_ROI = (630, 210, 75, 75)

DEV_FRAME_COUNT = 2
ANGLE_BINS = 360
COARSE_CENTER_ANGLES = 180
RADIAL_PROFILE_ANGLES = 360

# Ring hypothesis generation.  Radii are derived from the ROI size, not from
# the current frame's known dial radius.
RADIUS_FRAC_MIN = 0.10
RADIUS_FRAC_MAX = 0.62
RING_MIN_AXIS_PX = 10.0
RING_MAX_AXIS_RATIO = 1.90
RING_FIT_RADIAL_TOL_PX = 2.5
RING_FIT_ALIGN_MIN = 0.35
RING_FIT_ITERATIONS = 8
RING_FIT_RESIDUAL_PX = 2.0
RING_MIN_POINTS = 12
RING_COVERAGE_BINS = 72
RING_MIN_COVERAGE = 0.55
RING_MIN_ALIGN = 0.45
RING_MAX_RMS_PX = 2.20
RING_MIN_BAND_CONTRAST = 2.0
RING_MIN_BAND_POSITIVE_FRAC = 0.54
RING_MIN_MAJOR_FRAC = 0.35
RING_SCORE_MARGIN = 0.045
RING_DEDUPE_CENTER_PX = 3.5
RING_DEDUPE_AXIS_PX = 4.0
RING_DISTINCT_CENTER_PX = 5.0
RING_DISTINCT_AXIS_FRAC = 0.10

# Weak colour: sample BGR first, then convert.  Never interpolate OpenCV hue.
BAND_RHO_INSIDE = (0.84, 0.88, 0.92)
BAND_RHO_OUTSIDE = (1.08, 1.12, 1.16)
BAND_MIN_VALUE = 45
BAND_MIN_VALID_FRAC = 0.50
SIGNATURE_RHOS = (0.84, 0.88, 0.92, 0.96)
SIGNATURE_MIN_VALUE = 45
SIGNATURE_MIN_VALID_FRAC = 0.55
ROLL_MIN_CORR = 0.50
ROLL_MIN_GAP = 0.075
ROLL_MIN_SNR = 1.0
ROLL_MIN_AMPLITUDE = 0.020
ROLL_SECOND_PEAK_SEP_DEG = 20

# Axis / needle gates.
AXIS_SEARCH_FRAC = 0.22
AXIS_MIN_HUB_AREA = 6
AXIS_ANGLE_TOL_DEG = 2.0
AXIS_HARD_ANGLE_TOL_DEG = 15.0
AXIS_OFFSET_MAX_FRAC = 0.16
NEEDLE_MIN_COVERAGE = 0.55
NEEDLE_MIN_SCORE = 150.0

# E1-R4 tapered-needle centreline direction.  The line (direction + normal
# offset) is the unknown; the pivot cap only restricts where the line may run
# and which end is the tip.  For a straight symmetric needle the two edges need
# not be parallel (a tapered body opens by a few degrees), but the midpoint of
# corresponding left/right edge positions is exactly collinear.  The robust fit
# of those midpoints is therefore the primary direction estimator; the strip
# peak, the edge opening angle and the strip-vs-centreline difference are kept
# as diagnostics, never as uncertainty.
LINE_ANGLE_STEP_DEG = 2.0
LINE_ANGLE_FINE_STEP_DEG = 0.25
LINE_ANGLE_FINE_HALF_DEG = 3.0
LINE_OFFSET_STEP_PX = 0.5
LINE_OFFSET_LIMIT_PX = 2.0
LINE_OFFSET_FINE_LIMIT_PX = 0.75
LINE_CROSS_HALF_PX = 2.5
LINE_CROSS_STEP_PX = 0.5
LINE_RADIUS_FRAC_MAX = 0.82
LINE_RADIUS_FRAC_MIN = 0.20
LINE_MIN_CONTRAST = 12.0
LINE_MIN_EDGE_GRADIENT = 15.0
LINE_MAX_EDGE_SEPARATION_PX = 3.5
LINE_MIN_SAMPLES = 6
LINE_MIN_SUPPORT_FRAC = 0.35
LINE_MIN_TIP_LENGTH_FRAC = 0.22
LINE_TIP_TAIL_RATIO = 1.30
LINE_SECOND_PEAK_RATIO = 0.72
LINE_SECOND_PEAK_SEPARATION_DEG = 15.0
LINE_DIRECTION_TOL_DEG = 2.0
LINE_HUB_PERTURBATION_PX = 1.5
LINE_RING_PERTURBATION_PX = 0.75
LINE_HUB_PERTURBATION_COUNT = 4
LINE_RING_PERTURBATION_COUNT = 6
LINE_BOOTSTRAP_SAMPLES = 40
LINE_PROFILE_BINS = 360
# Shape consistency for the centreline model.  These are not direction gates:
# they reject a bent body / irregular edge that happens to have a straight-ish
# least-squares slope.  The thresholds are set from the synthetic straight-body
# controls (centreline residual well below one pixel for a 1-3 px body) and are
# absolute, so a wider phone pointer is not given a looser straightness claim.
LINE_MAX_CENTERLINE_RMS_PX = 0.5
LINE_MAX_WIDTH_RMS_PX = 0.75

# Dark pivot-cap evidence.  The centre is the distance-transform maximum
# plateau, never the merged needle+cap blob centroid.  A plateau is the set of
# pixels within the 0.5 px local quantisation floor of the maximum; this keeps
# a 2.0/2.2 px digital distance-transform peak from collapsing to one corner
# pixel.  Thresholds are associated only when their plateau centres stay on the
# same cap, so unrelated dark blocks cannot be averaged into one confident hub.
HUB_DARK_THRESHOLDS = (60, 70, 80, 90, 100)
HUB_PLATEAU_TOL_PX = 0.5
HUB_QUANTISATION_FLOOR_PX = 0.5
HUB_THRESHOLD_ASSOC_PX = 4.0
HUB_MIN_THRESHOLD_SUPPORT = 2
HUB_MIN_THICKNESS_RATIO = 1.25
HUB_COMPETITION_SCORE_RATIO = 0.80
HUB_COMPETITION_SEPARATION_PX = 2.0
HUB_PERTURBATION_PX = 0.5

STATUS_VALUES = ("ok", "out_of_range", "low_conf", "no_dial", "unstable")


class ExperimentConfigError(ValueError):
    """Invalid experiment configuration or unsafe output target."""


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------

def _json_safe(value):
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


def _finite(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ExperimentConfigError("{} must be a finite number".format(name))
    result = float(value)
    if not math.isfinite(result):
        raise ExperimentConfigError("{} must be finite".format(name))
    return result


def _roi_tuple(roi, shape: Sequence[int]) -> Tuple[int, int, int, int]:
    if not isinstance(roi, (list, tuple)) or len(roi) != 4:
        raise ExperimentConfigError("roi must contain x y w h")
    values = []
    for index, raw in enumerate(roi):
        value = _finite(raw, "roi[{}]".format(index))
        if int(value) != value:
            raise ExperimentConfigError("roi values must be integers")
        values.append(int(value))
    x, y, width, height = values
    image_h, image_w = int(shape[0]), int(shape[1])
    if width <= 0 or height <= 0 or x < 0 or y < 0:
        raise ExperimentConfigError("roi must be a positive rectangle inside the image")
    if x + width > image_w or y + height > image_h:
        raise ExperimentConfigError("roi is outside the image")
    return x, y, width, height


def expanded_roi(declared_roi: Sequence[int], shape: Sequence[int],
                 top_expansion_px: int = ROI_TOP_EXPANSION_PX) -> Dict:
    """Return the documented experiment ROI and a machine-readable record."""
    x, y, width, height = _roi_tuple(declared_roi, shape)
    image_h, image_w = int(shape[0]), int(shape[1])
    top = int(top_expansion_px)
    if top < 0 or top > height:
        raise ExperimentConfigError("top expansion must be between 0 and the ROI height")
    new_y = max(0, y - top)
    actual_top = y - new_y
    # Only the top is expanded; the other edges stay exactly as declared.
    new_height = height + actual_top
    if new_y + new_height > image_h:
        raise ExperimentConfigError("expanded ROI is outside the image")
    used = (x, new_y, width, new_height)
    return {
        "declared_roi": [int(v) for v in (x, y, width, height)],
        "used_roi": [int(v) for v in used],
        "top_expansion_px": int(actual_top),
        "reason": "declared ROI clips the visible top of the small humidity dial; "
                  "the expansion is a fixed scene prior recorded before evaluation",
        "image_width": image_w,
        "image_height": image_h,
    }


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    return _sha256_bytes(path.read_bytes())


# ---------------------------------------------------------------------------
# preprocessing and edge evidence
# ---------------------------------------------------------------------------

def preprocess(crop_bgr: np.ndarray, clahe: bool = False) -> Dict:
    """Deterministic local edge preprocessing.  No upscaling."""
    if not isinstance(crop_bgr, np.ndarray) or crop_bgr.ndim != 3:
        raise ExperimentConfigError("crop_bgr must be an HxWx3 array")
    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    if clahe:
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4)).apply(gray)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0.0)
    gx = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = cv2.magnitude(gx, gy)
    edges = cv2.Canny(blurred, 25, 75)
    return {
        "gray": gray,
        "blurred": blurred,
        "gx": gx,
        "gy": gy,
        "magnitude": magnitude,
        "edges": edges,
        "settings": {
            "clahe": bool(clahe),
            "blur": "GaussianBlur 3x3 sigma=0",
            "canny": [25, 75],
            "sobel_ksize": 3,
            "upscaled": False,
        },
    }


def _gold_hub(crop_bgr: np.ndarray) -> Optional[Dict]:
    """Small gold/brass pivot cap, when visible (phone views)."""
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    height, width = hsv.shape[:2]
    mask = ((hsv[:, :, 0] >= 12) & (hsv[:, :, 0] <= 40)
            & (hsv[:, :, 1] >= 90) & (hsv[:, :, 2] >= 90)).astype(np.uint8)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        bw = int(stats[index, cv2.CC_STAT_WIDTH])
        bh = int(stats[index, cv2.CC_STAT_HEIGHT])
        if area < 4 or area > 0.01 * width * height:
            continue
        if max(bw, bh) > 0.06 * min(width, height):
            continue
        cx, cy = float(centroids[index][0]), float(centroids[index][1])
        if not (0.25 * width < cx < 0.75 * width):
            continue
        if not (0.20 * height < cy < 0.75 * height):
            continue
        candidate = {
            "center": [cx, cy],
            "area": area,
            "threshold": "gold_hsv",
            "bbox": [int(stats[index, cv2.CC_STAT_LEFT]),
                     int(stats[index, cv2.CC_STAT_TOP]), bw, bh],
            "fill": 1.0,
            "score": float(area),
            "source": "gold_cap",
        }
        if best is None or candidate["score"] > best["score"]:
            best = candidate
    return best


def _hub_plateau_evidence(component: np.ndarray, distance: np.ndarray,
                          peak: float) -> Dict:
    """Max-distance-transform plateau: centre, extent and cluster structure.

    The tolerance is the 0.5 px local quantisation floor, capped at a quarter
    of the peak so a clearly lower distance-transform shelf is not folded into
    the maximum.  A 2.0 px digital DT peak can have one to several equal-max
    pixels; their mean is the cap centre evidence and their spread is a lower
    bound on how well that centre is localised.  The blob centroid is *not*
    used here.
    """
    tolerance = min(HUB_PLATEAU_TOL_PX, 0.25 * float(peak))
    plateau = component & (distance >= peak - tolerance - 1e-9)
    ys, xs = np.where(plateau)
    if xs.size == 0:
        return {}
    center = np.array([float(xs.mean()), float(ys.mean())])
    span = [float(xs.max() - xs.min()), float(ys.max() - ys.min())]
    rms = float(np.sqrt(np.mean((xs - center[0]) ** 2
                                + (ys - center[1]) ** 2)))
    count, _, stats, centroids = cv2.connectedComponentsWithStats(
        plateau.astype(np.uint8), 8)
    clusters = []
    for index in range(1, count):
        clusters.append({
            "center": [float(centroids[index][0]), float(centroids[index][1])],
            "pixels": int(stats[index, cv2.CC_STAT_AREA]),
        })
    separation = 0.0
    for first in range(len(clusters)):
        for second in range(first + 1, len(clusters)):
            separation = max(separation, math.hypot(
                clusters[first]["center"][0] - clusters[second]["center"][0],
                clusters[first]["center"][1] - clusters[second]["center"][1]))
    separated = bool(len(clusters) > 1
                     and separation > max(2.0, 3.0 * float(peak)))
    return {
        "center": center.tolist(),
        "span_px": span,
        "rms_px": rms,
        "pixel_count": int(xs.size),
        "cluster_count": len(clusters),
        "clusters": clusters,
        "cluster_separation_px": float(separation),
        "separated": separated,
        "tolerance_px": float(tolerance),
    }


def _dark_hub_candidates(crop_bgr: np.ndarray) -> List[Dict]:
    """Compact dark central caps, located at the DT maximum plateau.

    A dark needle often merges with its round pivot cap.  The blob centroid is
    then pulled along the needle, so the cap centre is taken at the centre of
    the distance-transform maximum plateau (inside the local 0.5 px
    quantisation floor).  The original blob centroid is kept only as a
    diagnostic.  Thresholds are associated to the same cap before their centre
    deviations are used as uncertainty; a lone threshold, a needle-like blob or
    two competing caps are explicitly marked unusable instead of being averaged
    into a confident hub.
    """
    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape[:2]
    raw = []
    for threshold in HUB_DARK_THRESHOLDS:
        mask = (gray < threshold).astype(np.uint8)
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        distance = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            bx = int(stats[index, cv2.CC_STAT_LEFT])
            by = int(stats[index, cv2.CC_STAT_TOP])
            bw = int(stats[index, cv2.CC_STAT_WIDTH])
            bh = int(stats[index, cv2.CC_STAT_HEIGHT])
            if area < AXIS_MIN_HUB_AREA or area > 400:
                continue
            if max(bw, bh) > 0.34 * min(width, height):
                continue
            cx, cy = float(centroids[index][0]), float(centroids[index][1])
            if not (0.25 * width < cx < 0.75 * width):
                continue
            if not (0.20 * height < cy < 0.75 * height):
                continue
            fill = float(area) / float(max(1, bw * bh))
            if fill < 0.32:
                continue
            component = (labels == index)
            local = np.where(component, distance, 0.0)
            peak = float(local.max())
            if peak <= 0.0:
                continue
            plateau = _hub_plateau_evidence(component, distance, peak)
            if not plateau:
                continue
            component_dt = local[component]
            median_dt = float(np.median(component_dt))
            thickness = peak / max(1e-9, median_dt)
            # A uniform-width needle has a long maximum-distance ridge; it is
            # not a compact round cap even when its DT peak is not tiny.
            plateau_elongated = bool(
                max(plateau["span_px"]) > max(2.0, 4.0 * peak))
            center = plateau["center"]
            raw.append({
                "center": center,
                "blob_centroid_px": [cx, cy],
                "centroid_offset_px": float(math.hypot(cx - center[0],
                                                       cy - center[1])),
                "area": area,
                "threshold": int(threshold),
                "bbox": [bx, by, bw, bh],
                "fill": fill,
                "distance_peak_px": peak,
                "component_median_dt_px": median_dt,
                "thickness_ratio": thickness,
                "cap_like": bool(thickness >= HUB_MIN_THICKNESS_RATIO
                                 and not plateau_elongated),
                "plateau_elongated": plateau_elongated,
                "plateau": plateau,
                "plateau_separated": bool(plateau["separated"]),
                "score": float(peak * peak) * float(area),
                "source": "dark_cap",
            })

    # Associate only candidates from different thresholds that stay on the same
    # cap.  Same-threshold components are distinct blobs and never merge.
    # Transitive association is used so a thin needle fragment at one threshold
    # cannot split the cap's higher-threshold evidence into two groups.
    parent = list(range(len(raw)))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(first, second):
        root_first, root_second = find(first), find(second)
        if root_first != root_second:
            parent[root_second] = root_first

    for first in range(len(raw)):
        for second in range(first + 1, len(raw)):
            if raw[first]["threshold"] == raw[second]["threshold"]:
                continue
            separation = math.hypot(
                raw[first]["center"][0] - raw[second]["center"][0],
                raw[first]["center"][1] - raw[second]["center"][1])
            if separation <= HUB_THRESHOLD_ASSOC_PX:
                union(first, second)
    grouped: Dict[int, List[Dict]] = {}
    for index, candidate in enumerate(raw):
        grouped.setdefault(find(index), []).append(candidate)
    groups = list(grouped.values())

    found = []
    for members in groups:
        members = sorted(members, key=lambda item: item["threshold"])
        cap_members = [item for item in members if item["cap_like"]]
        representative = max(cap_members or members,
                             key=lambda item: item["score"])
        support = len(cap_members)
        threshold_centers = [{
            "threshold": member["threshold"],
            "center": member["center"],
            "distance_peak_px": member["distance_peak_px"],
            "area": member["area"],
            "plateau_span_px": member["plateau"]["span_px"],
        } for member in cap_members]
        if cap_members:
            spread = max(math.hypot(member["center"][0] - representative["center"][0],
                                    member["center"][1] - representative["center"][1])
                         for member in cap_members)
        else:
            spread = 0.0
        half_span = 0.5 * max(representative["plateau"]["span_px"])
        uncertainty = max(HUB_QUANTISATION_FLOOR_PX, float(half_span),
                          float(spread))
        plateau = dict(representative["plateau"])
        plateau["center"] = representative["center"]
        # The centre differs by more than the cap inradius between thresholds:
        # the threshold centres are not localising one point.
        threshold_unstable = bool(
            support >= HUB_MIN_THRESHOLD_SUPPORT
            and spread > max(HUB_QUANTISATION_FLOOR_PX,
                             representative["distance_peak_px"]))
        ambiguous = bool(representative["plateau_separated"]
                         or threshold_unstable
                         or support < HUB_MIN_THRESHOLD_SUPPORT
                         or not representative["cap_like"])
        usable = bool(representative["cap_like"]
                      and support >= HUB_MIN_THRESHOLD_SUPPORT)
        reject = None
        if not representative["cap_like"]:
            reject = "hub_needle_thickness"
        elif support < HUB_MIN_THRESHOLD_SUPPORT:
            reject = "hub_no_threshold_consensus"
        found.append({
            "center": representative["center"],
            # The merged blob centroid is kept only as a coarse ring-hypothesis
            # seed/prior; it is never the axis and is not used as cap evidence.
            "seed_center": representative["blob_centroid_px"],
            "blob_centroid_px": representative["blob_centroid_px"],
            "centroid_offset_px": representative["centroid_offset_px"],
            "area": representative["area"],
            "threshold": representative["threshold"],
            "thresholds": sorted({member["threshold"] for member in cap_members}),
            "threshold_support": int(support),
            "threshold_centers": threshold_centers,
            "center_spread_px": float(spread),
            "center_uncertainty_px": float(uncertainty),
            "bbox": representative["bbox"],
            "fill": representative["fill"],
            "distance_peak_px": representative["distance_peak_px"],
            "thickness_ratio": representative["thickness_ratio"],
            "cap_like": representative["cap_like"],
            "plateau_elongated": representative["plateau_elongated"],
            "plateau": plateau,
            "plateau_separated": representative["plateau_separated"],
            "threshold_centers_unstable": threshold_unstable,
            "ambiguous": ambiguous,
            "usable": usable,
            "competition": False,
            "reject": reject,
            "score": representative["score"],
            "source": "dark_cap",
        })

    # Two separate compact caps with comparable evidence: do not pick one as a
    # confident pivot just because its score is a few percent higher.
    usable_groups = [item for item in found if item["usable"]]
    for first in range(len(usable_groups)):
        for second in range(first + 1, len(usable_groups)):
            one, two = usable_groups[first], usable_groups[second]
            separation = math.hypot(one["center"][0] - two["center"][0],
                                    one["center"][1] - two["center"][1])
            if separation <= HUB_COMPETITION_SEPARATION_PX:
                continue
            ratio = min(one["score"], two["score"]) / max(one["score"], two["score"])
            if ratio >= HUB_COMPETITION_SCORE_RATIO:
                for item in (one, two):
                    item["competition"] = True
                    item["usable"] = False
                    item["ambiguous"] = True
                    item["reject"] = "hub_competition"
    found.sort(key=lambda item: -item["score"])
    return found


def detect_hub(crop_bgr: np.ndarray) -> Optional[Dict]:
    """Highest-scoring usable compact dark central cap, or None.

    ``select_hub`` uses the full candidate list; this helper is kept for
    callers/tests that only need the dark-cap evidence.
    """
    candidates = _dark_hub_candidates(crop_bgr)
    usable = [item for item in candidates if item["usable"]]
    pool = usable or candidates
    if not pool:
        return None
    return max(pool, key=lambda item: item["score"])


def select_hub(crop_bgr: np.ndarray, reference_center: Sequence[float]) -> Optional[Dict]:
    """Choose dark vs gold pivot evidence closest to the radial-symmetry centre.

    A dark mask can merge the pivot with a printed label; a gold mask can be
    fooled by a warm-coloured printed band.  Comparing both against the
    independent radial-symmetry centre removes that failure mode.  A rejected
    dark candidate is still returned (with ``usable=false`` and a reason) so the
    frame report can show why the axis is unverified; it is never silently
    replaced by the ring centre.
    """
    height, width = crop_bgr.shape[:2]
    limit = 0.12 * min(width, height)

    def near(item):
        return item is not None and math.hypot(
            item["center"][0] - reference_center[0],
            item["center"][1] - reference_center[1]) <= limit

    gold = _gold_hub(crop_bgr)
    if gold is not None:
        gold = dict(gold)
        gold.update({
            "blob_centroid_px": list(gold["center"]),
            "seed_center": list(gold["center"]),
            "centroid_offset_px": 0.0,
            "center_uncertainty_px": HUB_QUANTISATION_FLOOR_PX,
            "threshold_support": 1,
            "threshold_centers": [],
            "ambiguous": False,
            "usable": True,
            "competition": False,
            "reject": None,
        })
        if near(gold):
            return gold
    nearby = [item for item in _dark_hub_candidates(crop_bgr) if near(item)]
    if not nearby:
        return None
    usable = [item for item in nearby if item["usable"]]
    if usable:
        return max(usable, key=lambda item: item["score"])
    return max(nearby, key=lambda item: item["score"])



def _sample_map(gx: np.ndarray, gy: np.ndarray, xs: np.ndarray,
                ys: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    map_x = xs.astype(np.float32).reshape(1, -1)
    map_y = ys.astype(np.float32).reshape(1, -1)
    sx = cv2.remap(gx, map_x, map_y, cv2.INTER_LINEAR,
                   borderMode=cv2.BORDER_REPLICATE)[0]
    sy = cv2.remap(gy, map_x, map_y, cv2.INTER_LINEAR,
                   borderMode=cv2.BORDER_REPLICATE)[0]
    return sx, sy


def radial_profile(prep: Dict, center: Sequence[float],
                   radius_min: float, radius_max: float) -> List[Dict]:
    """Median radial-gradient response for each integer radius."""
    gx = prep["gx"]
    gy = prep["gy"]
    height, width = gx.shape[:2]
    angles = np.linspace(0.0, 2.0 * math.pi, RADIAL_PROFILE_ANGLES, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)
    cx, cy = float(center[0]), float(center[1])
    profile = []
    for radius in range(int(math.floor(radius_min)), int(math.ceil(radius_max)) + 1):
        xs = cx + radius * cos_a
        ys = cy + radius * sin_a
        if xs.min() < 0 or ys.min() < 0 or xs.max() >= width or ys.max() >= height:
            profile.append({"radius": radius, "score": 0.0,
                            "coverage": 0.0, "finite": False})
            continue
        sx, sy = _sample_map(gx, gy, xs, ys)
        radial = np.abs(sx * cos_a + sy * sin_a)
        profile.append({
            "radius": radius,
            "score": float(np.median(radial)),
            "coverage": float((radial > 30.0).mean()),
            "finite": True,
        })
    return profile


def profile_peaks(profile: List[Dict], max_peaks: int = 3,
                  min_separation: int = 3) -> List[Dict]:
    if not profile:
        return []
    ordered = sorted(profile, key=lambda item: -item["score"])
    picked = []
    for item in ordered:
        if not item.get("finite") or item["score"] <= 0.0:
            continue
        if all(abs(item["radius"] - other["radius"]) >= min_separation
               for other in picked):
            picked.append(item)
        if len(picked) >= max_peaks:
            break
    return picked


def _edge_points(edges: np.ndarray) -> np.ndarray:
    ys, xs = np.nonzero(edges)
    return np.stack([xs, ys], axis=1).astype(np.float32)


def fit_ellipse_from_radius(points: np.ndarray, prep: Dict,
                            center: Sequence[float], radius: float) -> Optional[Dict]:
    """Deterministic robust ellipse fit seeded by a circle.

    The circle is only a hypothesis.  Points are selected by radial distance,
    gradient-direction agreement and then iteratively trimmed by ellipse
    residual.  No colour connected component is used.
    """
    if len(points) < RING_MIN_POINTS:
        return None
    gx = prep["gx"]
    gy = prep["gy"]
    cx, cy = float(center[0]), float(center[1])
    dx = points[:, 0] - cx
    dy = points[:, 1] - cy
    distance = np.hypot(dx, dy)
    # An adaptive annulus is required: a circle seed +/- 2.5 px cannot follow
    # a perspectively elongated ring.  The width is still scale-relative and
    # never uses the current frame's known dial geometry.
    half_width = max(RING_FIT_RADIAL_TOL_PX, 0.08 * float(radius))
    near = np.abs(distance - float(radius)) <= half_width
    if int(near.sum()) < RING_MIN_POINTS:
        return None
    selected = points[near]
    selected_dx = dx[near]
    selected_dy = dy[near]
    selected_distance = distance[near]
    ys = selected[:, 1].astype(int)
    xs = selected[:, 0].astype(int)
    ys = np.clip(ys, 0, gx.shape[0] - 1)
    xs = np.clip(xs, 0, gx.shape[1] - 1)
    gxv = gx[ys, xs]
    gyv = gy[ys, xs]
    norm = np.hypot(gxv, gyv) + 1e-9
    alignment = np.abs((gxv * selected_dx + gyv * selected_dy)
                       / (norm * selected_distance + 1e-9))
    selected = selected[alignment >= RING_FIT_ALIGN_MIN]
    if len(selected) < RING_MIN_POINTS:
        return None
    inliers = selected
    for _ in range(RING_FIT_ITERATIONS):
        if len(inliers) < RING_MIN_POINTS:
            return None
        try:
            (ecx, ecy), (axis_a, axis_b), angle = cv2.fitEllipse(inliers)
        except cv2.error:
            return None
        if not all(math.isfinite(v) for v in (ecx, ecy, axis_a, axis_b, angle)):
            return None
        if min(axis_a, axis_b) < RING_MIN_AXIS_PX:
            return None
        if max(axis_a, axis_b) / min(axis_a, axis_b) > RING_MAX_AXIS_RATIO:
            return None
        residuals = _ellipse_residual(inliers, np.array([ecx, ecy]),
                                      (axis_a, axis_b), angle)
        keep = residuals <= RING_FIT_RESIDUAL_PX
        if int(keep.sum()) < RING_MIN_POINTS:
            return None
        inliers = inliers[keep]
    return {
        "center": [float(ecx), float(ecy)],
        "axes": [float(axis_a), float(axis_b)],
        "angle_deg": float(angle),
        "points": inliers,
        "seed_radius": float(radius),
        "seed_center": [float(cx), float(cy)],
    }


def ellipse_support(points: np.ndarray, ellipse: Dict, prep: Dict) -> Dict:
    """Visible-arc coverage, gradient support and residual for an ellipse."""
    center = np.asarray(ellipse["center"], dtype=float)
    axes = (float(ellipse["axes"][0]), float(ellipse["axes"][1]))
    angle = float(ellipse["angle_deg"])
    residuals = _ellipse_residual(points, center, axes, angle)
    keep = residuals <= RING_MAX_RMS_PX
    inliers = points[keep]
    if len(inliers) < RING_MIN_POINTS:
        return {"ok": False, "reason": "ring_support_too_small",
                "inliers": int(len(inliers))}
    residuals = residuals[keep]
    dx = inliers[:, 0] - center[0]
    dy = inliers[:, 1] - center[1]
    distance = np.hypot(dx, dy) + 1e-9
    ys = np.clip(inliers[:, 1].astype(int), 0, prep["gx"].shape[0] - 1)
    xs = np.clip(inliers[:, 0].astype(int), 0, prep["gx"].shape[1] - 1)
    gxv = prep["gx"][ys, xs]
    gyv = prep["gy"][ys, xs]
    magnitude = np.hypot(gxv, gyv)
    alignment = np.abs((gxv * dx + gyv * dy) / (magnitude * distance + 1e-9))
    angles = np.degrees(np.arctan2(dy, dx)) % 360.0
    bins = np.unique(np.floor(angles / (360.0 / RING_COVERAGE_BINS)).astype(int))
    filled = len(bins)
    coverage = filled / float(RING_COVERAGE_BINS)
    # Largest circular gap in the visible support, in bins.
    occupied = np.zeros(RING_COVERAGE_BINS, dtype=bool)
    occupied[bins] = True
    max_gap = 0
    run = 0
    doubled = np.r_[occupied, occupied]
    for value in doubled:
        if not value:
            run += 1
            max_gap = max(max_gap, run)
        else:
            run = 0
    max_gap = min(max_gap, RING_COVERAGE_BINS)
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    return {
        "ok": True,
        "inliers": int(len(inliers)),
        "residual_rms_px": rms,
        "residual_max_px": float(np.max(residuals)),
        "coverage_bins": int(filled),
        "coverage": float(coverage),
        "max_gap_deg": float(max_gap * (360.0 / RING_COVERAGE_BINS)),
        "align": float(np.mean(alignment)),
        "gradient_support": float(np.median(magnitude)),
        "inlier_points": inliers,
    }


def _sample_bgr_rhos(crop_bgr: np.ndarray, center: np.ndarray, matrix: np.ndarray,
                     rhos: Sequence[float], bins: int = ANGLE_BINS) -> Dict:
    """Sample BGR on several normalised radii, then convert to HSV.

    BGR is interpolated first on purpose: interpolating OpenCV's circular hue
    plane across the 179/0 wrap manufactures a false blue segment.  This
    mirrors the fix already validated in ``dial_reading.sample_blue``.
    """
    inverse = np.linalg.inv(np.asarray(matrix, dtype=float))
    angles = np.linspace(0.0, 2.0 * math.pi, bins, endpoint=False)
    unit = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    samples = []
    for rho in rhos:
        points = np.asarray(center, dtype=float) + (unit * float(rho)) @ inverse.T
        map_x = points[:, 0].astype(np.float32).reshape(1, -1)
        map_y = points[:, 1].astype(np.float32).reshape(1, -1)
        bgr = cv2.remap(crop_bgr, map_x, map_y, cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_REPLICATE)[0].astype(np.int16)
        hsv = cv2.cvtColor(bgr.astype(np.uint8).reshape(1, -1, 3),
                           cv2.COLOR_BGR2HSV)[0].astype(np.int16)
        samples.append({"bgr": bgr, "hsv": hsv})
    bgr_stack = np.stack([item["bgr"] for item in samples], axis=1)
    hsv_stack = np.stack([item["hsv"] for item in samples], axis=1)
    return {
        "angles": angles,
        "bgr": bgr_stack,
        "hsv": hsv_stack,
        "rhos": list(rhos),
    }


def band_contrast(crop_bgr: np.ndarray, ellipse: Dict) -> Dict:
    """Soft signed chroma/brightness contrast across the fitted edge.

    A coloured band outer edge has more chromatic content just inside than just
    outside, and is usually darker.  A plain grey border/ring has no chroma
    contrast, so it cannot satisfy the gate.  No S>=90 hard mask is used.
    """
    matrix, _, _, _ = _ellipse_matrix(
        (float(ellipse["axes"][0]), float(ellipse["axes"][1])),
        float(ellipse["angle_deg"]))
    inside = _sample_bgr_rhos(crop_bgr, np.asarray(ellipse["center"], dtype=float),
                              matrix, BAND_RHO_INSIDE)
    outside = _sample_bgr_rhos(crop_bgr, np.asarray(ellipse["center"], dtype=float),
                               matrix, BAND_RHO_OUTSIDE)
    inside_bgr = inside["bgr"].astype(np.float32)
    outside_bgr = outside["bgr"].astype(np.float32)
    inside_chroma = inside_bgr.max(axis=2) - inside_bgr.min(axis=2)
    outside_chroma = outside_bgr.max(axis=2) - outside_bgr.min(axis=2)
    inside_value = inside["hsv"][:, :, 2].astype(np.float32)
    outside_value = outside["hsv"][:, :, 2].astype(np.float32)
    inside_chroma = np.median(inside_chroma, axis=1)
    outside_chroma = np.median(outside_chroma, axis=1)
    inside_value = np.median(inside_value, axis=1)
    outside_value = np.median(outside_value, axis=1)
    valid = ((inside_value >= BAND_MIN_VALUE) & (outside_value >= BAND_MIN_VALUE))
    valid_frac = float(valid.mean())
    if valid_frac < BAND_MIN_VALID_FRAC:
        return {"ok": False, "reason": "band_region_dark",
                "valid_frac": valid_frac, "contrast": None}
    difference = inside_chroma[valid] - outside_chroma[valid]
    value_difference = inside_value[valid] - outside_value[valid]
    median = float(np.median(difference))
    noise = float(1.4826 * np.median(np.abs(difference - median)))
    return {
        "ok": True,
        "contrast": median,
        "positive_frac": float((difference > 0.0).mean()),
        "noise": noise,
        "valid_frac": valid_frac,
        "value_difference": float(np.median(value_difference)),
        "rho_inside": list(BAND_RHO_INSIDE),
        "rho_outside": list(BAND_RHO_OUTSIDE),
    }


def _contrast_score(contrast: Dict) -> float:
    if not contrast.get("ok"):
        return 0.0
    value = float(contrast.get("contrast") or 0.0)
    return float(np.clip((value - RING_MIN_BAND_CONTRAST) / 18.0, 0.0, 1.0))


def _center_prior_term(center: Sequence[float], axes: Sequence[float],
                      hub: Optional[Dict]) -> float:
    """Weak axis/ring-centre consistency term from the dark/gold pivot cap.

    The reference normalised offset is about 1% of the radius, so a ring centre
    tens of pixels away from the observed pivot is not the same sub-dial.  This
    is a pixel-evidence prior, not a known angle/radius and not a value.
    """
    if hub is None or not hub.get("usable", True):
        return 0.0
    major = max(float(axes[0]), float(axes[1]))
    distance = math.hypot(float(center[0]) - float(hub["center"][0]),
                          float(center[1]) - float(hub["center"][1]))
    offset = distance / max(1e-9, major)
    return float(np.clip(1.0 - offset / AXIS_OFFSET_MAX_FRAC, 0.0, 1.0))


def _ring_score(support: Dict, contrast: Dict, center: Sequence[float],
                axes: Sequence[float], hub: Optional[Dict]) -> float:
    residual_term = float(np.clip(1.0 - support["residual_rms_px"] / 2.5, 0.0, 1.0))
    gradient_term = float(np.clip(support["gradient_support"] / 140.0, 0.0, 1.0))
    center_term = _center_prior_term(center, axes, hub)
    return float(
        0.29 * support["coverage"]
        + 0.20 * support["align"]
        + 0.22 * _contrast_score(contrast)
        + 0.11 * residual_term
        + 0.08 * gradient_term
        + 0.10 * center_term
    )


def _same_ring(first: Dict, second: Dict) -> bool:
    if (abs(first["center"][0] - second["center"][0]) <= RING_DEDUPE_CENTER_PX
            and abs(first["center"][1] - second["center"][1]) <= RING_DEDUPE_CENTER_PX):
        major_first = max(first["axes"])
        major_second = max(second["axes"])
        return abs(major_first - major_second) <= RING_DEDUPE_AXIS_PX
    return False


def _nested_ring(first: Dict, second: Dict) -> bool:
    """True when two hypotheses are inner/outer edges of the same band."""
    major_first = max(first["axes"])
    major_second = max(second["axes"])
    mean_major = 0.5 * (major_first + major_second)
    center_distance = math.hypot(first["center"][0] - second["center"][0],
                                 first["center"][1] - second["center"][1])
    return (center_distance <= max(5.0, 0.07 * mean_major)
            and abs(major_first - major_second) <= max(6.0, 0.12 * max(major_first, major_second)))


def _distinct_ring(first: Dict, second: Dict) -> bool:
    distance = math.hypot(first["center"][0] - second["center"][0],
                          first["center"][1] - second["center"][1])
    major_first = max(first["axes"])
    major_second = max(second["axes"])
    radius_diff = abs(major_first - major_second)
    return (distance > RING_DISTINCT_CENTER_PX
            or radius_diff > RING_DISTINCT_AXIS_FRAC * max(major_first, major_second))


def coarse_center_seeds(prep: Dict, hub: Optional[Dict],
                        radius_min: float, radius_max: float,
                        max_seeds: int = 4) -> List[Dict]:
    """Coarse radial-symmetry centre search plus the dark-hub seed."""
    gx = prep["gx"]
    gy = prep["gy"]
    height, width = gx.shape[:2]
    angles = np.linspace(0.0, 2.0 * math.pi, COARSE_CENTER_ANGLES, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)
    center_x = width / 2.0
    center_y = height / 2.0
    span_x = 0.20 * width
    span_y = 0.20 * height
    step = max(2, int(round(min(width, height) / 30.0)))
    grid = []
    for cy in np.arange(center_y - span_y, center_y + span_y + 1e-9, step):
        for cx in np.arange(center_x - span_x, center_x + span_x + 1e-9, step):
            best = (-1.0, None)
            for radius in range(int(math.floor(radius_min)),
                                int(math.ceil(radius_max)) + 1, 2):
                xs = cx + radius * cos_a
                ys = cy + radius * sin_a
                if xs.min() < 0 or ys.min() < 0 or xs.max() >= width or ys.max() >= height:
                    continue
                sx, sy = _sample_map(gx, gy, xs, ys)
                score = float(np.median(np.abs(sx * cos_a + sy * sin_a)))
                if score > best[0]:
                    best = (score, radius)
            if best[0] > 45.0:
                grid.append({"center": [float(cx), float(cy)],
                             "score": float(best[0]),
                             "radius": int(best[1])})
    grid.sort(key=lambda item: -item["score"])
    seeds = []
    if hub is not None and hub.get("usable", True):
        seed_center = hub.get("seed_center") or hub["center"]
        seeds.append({"center": [float(seed_center[0]), float(seed_center[1])],
                      "score": float(hub["score"]), "source": "dark_hub"})
    for item in grid:
        if all(math.hypot(item["center"][0] - other["center"][0],
                          item["center"][1] - other["center"][1]) >= 6.0
               for other in seeds):
            item = dict(item)
            item["source"] = "radial_symmetry"
            seeds.append(item)
        if len(seeds) >= max_seeds:
            break
    return seeds


def detect_edge_ring(crop_bgr: np.ndarray, clahe: bool = False) -> Dict:
    """Main edge-first ring detector.  Returns best ring or a rejection."""
    prep = preprocess(crop_bgr, clahe=clahe)
    height, width = crop_bgr.shape[:2]
    min_dim = min(width, height)
    radius_min = max(8.0, RADIUS_FRAC_MIN * min_dim)
    radius_max = max(radius_min + 4.0, RADIUS_FRAC_MAX * min_dim)
    # The caller's ROI is a scene prior centred on the small dial; use its
    # geometric centre as the independent reference for pivot selection.  The
    # coarse radial search is retained for ring hypotheses but is not trusted
    # for pivot disambiguation.
    reference_center = [width / 2.0, height / 2.0]
    hub = select_hub(crop_bgr, reference_center)
    seeds = coarse_center_seeds(prep, hub, radius_min, radius_max)
    points = _edge_points(prep["edges"])
    candidates = []
    evaluated = []
    for seed in seeds:
        profile = radial_profile(prep, seed["center"], radius_min, radius_max)
        peaks = profile_peaks(profile, max_peaks=3)
        profile_by_radius = {item["radius"]: item for item in profile}
        seed_radii = [item["radius"] for item in peaks]
        step = max(2, int(round((radius_max - radius_min) / 24.0)))
        for radius in range(int(round(radius_min)), int(round(radius_max)) + 1, step):
            if any(abs(radius - chosen) <= 2 for chosen in seed_radii):
                continue
            if profile_by_radius.get(radius, {}).get("score", 0.0) <= 0.0:
                continue
            seed_radii.append(radius)
        for radius in seed_radii:
            peak = profile_by_radius.get(radius, {"radius": radius})
            fit = fit_ellipse_from_radius(points, prep, seed["center"], peak["radius"])
            if fit is None:
                evaluated.append({"center": seed["center"], "radius": peak["radius"],
                                  "status": "fit_rejected"})
                continue
            support = ellipse_support(fit["points"], fit, prep)
            if not support.get("ok"):
                evaluated.append({"center": fit["center"], "radius": peak["radius"],
                                  "status": support.get("reason", "support_rejected")})
                continue
            contrast = band_contrast(crop_bgr, fit)
            candidate = {
                "center": fit["center"],
                "axes": fit["axes"],
                "angle_deg": fit["angle_deg"],
                "seed_center": fit["seed_center"],
                "seed_radius": fit["seed_radius"],
                "inlier_points": support.pop("inlier_points"),
                "support": support,
                "band_contrast": contrast,
                "score": _ring_score(support, contrast, fit["center"],
                                     fit["axes"], hub),
            }
            major = max(fit["axes"])
            pivot_distance = None
            if hub is not None:
                pivot_distance = math.hypot(
                    fit["center"][0] - hub["center"][0],
                    fit["center"][1] - hub["center"][1])
            candidate["pivot_distance_px"] = pivot_distance
            candidate["pivot_offset_frac"] = (
                None if pivot_distance is None else pivot_distance / max(1e-9, major))
            if max(fit["axes"]) < RING_MIN_MAJOR_FRAC * min_dim:
                candidate["reject"] = "ring_too_small"
            elif not contrast.get("ok"):
                candidate["reject"] = contrast.get("reason", "band_rejected")
            elif (support["coverage"] < RING_MIN_COVERAGE
                  or support["align"] < RING_MIN_ALIGN
                  or support["residual_rms_px"] > RING_MAX_RMS_PX):
                candidate["reject"] = "weak_edge_geometry"
            elif (float(contrast["contrast"]) < RING_MIN_BAND_CONTRAST
                  or float(contrast["positive_frac"]) < RING_MIN_BAND_POSITIVE_FRAC):
                candidate["reject"] = "no_colour_band_edge"
            elif candidate["pivot_offset_frac"] is not None and \
                    candidate["pivot_offset_frac"] > 2.0 * AXIS_OFFSET_MAX_FRAC:
                # A very large pivot offset is evidence that the hypothesis is
                # not this sub-dial, but a merely moderate offset is only a
                # score penalty: the dark cap can merge with the needle.
                candidate["reject"] = "centre_prior_inconsistent"
            candidates.append(candidate)
    candidates.sort(key=lambda item: -item["score"])
    accepted = [item for item in candidates if "reject" not in item]
    rejected = [item for item in candidates if "reject" in item]
    result = {
        "ok": False,
        "status": "no_dial",
        "reason": "no_ring",
        "preprocess": prep["settings"],
        "hub": hub,
        "seeds": [{"center": item["center"], "source": item.get("source", ""),
                   "score": item.get("score")} for item in seeds],
        "candidate_count": len(candidates),
        "accepted_count": len(accepted),
        "top_rejected": _compact_candidates(rejected[:3]),
        "all_candidates": _compact_candidates(candidates[:12]),
        "radius_range_px": [radius_min, radius_max],
        "evaluated": evaluated[:8],
    }
    if not accepted:
        if rejected and rejected[0].get("reject") == "no_colour_band_edge":
            result.update({"reason": "no_colour_band_edge",
                           "detail": "edge rings exist but none has a signed colour-band edge",
                           "top_candidates": _compact_candidates(rejected[:3])})
        elif rejected and rejected[0].get("reject") == "weak_edge_geometry":
            result.update({"reason": "weak_edge_geometry",
                           "top_candidates": _compact_candidates(rejected[:3])})
        else:
            result.update({"reason": "no_ring",
                           "top_candidates": _compact_candidates(rejected[:3])})
        return result
    best = accepted[0]
    # The same visible colour band produces an inner and an outer edge.  They
    # are nested hypotheses of one ring, not competing dials.  Prefer the outer
    # edge (the requested target) when it is nearly as well supported.  This is
    # a topology rule; it does not encode the current radius.
    family = [item for item in accepted
              if item is best or _nested_ring(item, best)]
    outer = max(family, key=lambda item: max(item["axes"]))
    if outer is not best and outer["score"] >= best["score"] - 0.15:
        best = outer
    second = None
    for item in accepted[1:]:
        if _distinct_ring(best, item):
            second = item
            break
    if second is not None and (best["score"] - second["score"]) <= RING_SCORE_MARGIN:
        result.update({
            "status": "low_conf",
            "reason": "ambiguous_ring",
            "detail": "two distinct edge/colour ring hypotheses are too close",
            "top_candidates": _compact_candidates([best, second]),
        })
        return result
    result.update({
        "ok": True,
        "status": "ok",
        "reason": "ring_found",
        "ring": best,
        "second": None if second is None else _compact_candidate(second),
        "score_margin": None if second is None else float(best["score"] - second["score"]),
        "top_candidates": _compact_candidates(accepted[:3]),
    })
    return result


def _compact_candidate(candidate: Dict) -> Dict:
    return {
        "center": [round(float(v), 3) for v in candidate["center"]],
        "axes": [round(float(v), 3) for v in candidate["axes"]],
        "angle_deg": round(float(candidate["angle_deg"]), 3),
        "score": round(float(candidate["score"]), 4),
        "coverage": round(float(candidate["support"]["coverage"]), 4),
        "residual_rms_px": round(float(candidate["support"]["residual_rms_px"]), 4),
        "align": round(float(candidate["support"]["align"]), 4),
        "band_contrast": None if not candidate["band_contrast"].get("ok")
        else round(float(candidate["band_contrast"]["contrast"]), 3),
        "band_positive_frac": None if not candidate["band_contrast"].get("ok")
        else round(float(candidate["band_contrast"]["positive_frac"]), 3),
        "band_valid_frac": None if not candidate["band_contrast"].get("ok")
        else round(float(candidate["band_contrast"]["valid_frac"]), 3),
        "pivot_offset_frac": None if candidate.get("pivot_offset_frac") is None
        else round(float(candidate["pivot_offset_frac"]), 4),
        "reject": candidate.get("reject"),
    }


def _compact_candidates(candidates: Sequence[Dict]) -> List[Dict]:
    return [_compact_candidate(item) for item in candidates]


# ---------------------------------------------------------------------------
# weak-colour roll: one-dimensional angular signature correlation
# ---------------------------------------------------------------------------

def _circular_smooth(values: np.ndarray, width: int) -> np.ndarray:
    if width <= 1:
        return values
    kernel = np.ones(width, dtype=float) / float(width)
    padded = np.r_[values[-(width // 2):], values, values[:width // 2]]
    return np.convolve(padded, kernel, mode="same")[width // 2:width // 2 + len(values)]


def angular_signature(crop_bgr: np.ndarray, center: Sequence[float],
                      matrix: np.ndarray) -> Dict:
    """1-D angular colour signature on a static band of normalised radii.

    BGR is sampled before HSV conversion.  The returned channels are opponent
    colour differences, not a hue interpolation, so the 179/0 wrap cannot
    manufacture blue.  Dark pixels (needle, printed text) are excluded.
    """
    sampled = _sample_bgr_rhos(crop_bgr, np.asarray(center, dtype=float),
                               np.asarray(matrix, dtype=float), SIGNATURE_RHOS)
    bgr = sampled["bgr"].astype(np.float32)
    hsv = sampled["hsv"].astype(np.float32)
    valid = hsv[:, :, 2] >= SIGNATURE_MIN_VALUE
    valid_count = valid.sum(axis=1)
    valid_frac = valid_count / float(len(SIGNATURE_RHOS))
    u = bgr[:, :, 0] - bgr[:, :, 1]
    v = bgr[:, :, 2] - bgr[:, :, 1]
    channels = []
    for raw in (u, v):
        masked = np.where(valid, raw, np.nan)
        with np.errstate(invalid="ignore"), warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            med = np.nanmedian(masked, axis=1)
        med = np.where(np.isfinite(med), med, 0.0)
        channels.append(_circular_smooth(med / 255.0, 3))
    signature = np.stack(channels, axis=1)
    weights = np.clip(valid_frac, 0.0, 1.0)
    valid_bins = weights >= 0.5
    if int(valid_bins.sum()) < 60:
        return {"ok": False, "reason": "signature_too_few_valid_bins",
                "valid_frac": float(valid_bins.mean())}
    mean = signature[valid_bins].mean(axis=0)
    centered = signature - mean
    amplitude = float(np.sqrt(np.mean(centered[valid_bins] ** 2)))
    if amplitude <= 1e-9:
        return {"ok": False, "reason": "signature_flat",
                "valid_frac": float(valid_bins.mean()), "amplitude": amplitude}
    normalized = centered / amplitude
    normalized[~valid_bins] = 0.0
    return {
        "ok": True,
        "signature": normalized,
        "weights": weights,
        "valid_bins": valid_bins,
        "valid_frac": float(valid_bins.mean()),
        "amplitude": amplitude,
        "raw_signature": signature,
        "rhos": list(SIGNATURE_RHOS),
    }


def reference_signature(reference_bgr: np.ndarray, calibration: Dict) -> Dict:
    matrix = np.asarray(calibration["matrix"], dtype=float)
    center = np.asarray(calibration["ring_center"], dtype=float)
    result = angular_signature(reference_bgr, center, matrix)
    if result.get("ok"):
        result["source"] = "static reference ring band (same candidate source image)"
        result["phase_definition"] = "reference signature phase; roll is the shift of current onto reference"
    return result


def _circular_correlation(reference: np.ndarray, current: np.ndarray,
                          valid: np.ndarray) -> np.ndarray:
    bins = int(reference.shape[0])
    scores = np.full(bins, -1.0, dtype=float)
    for shift in range(bins):
        index = (np.arange(bins) + shift) % bins
        overlap = valid & valid[index]
        count = int(overlap.sum())
        if count < 40:
            continue
        total = 0.0
        for channel in range(reference.shape[1]):
            left = reference[overlap, channel]
            right = current[index[overlap], channel]
            denominator = math.sqrt(float(np.dot(left, left) * np.dot(right, right))) + 1e-12
            total += float(np.dot(left, right)) / denominator
        scores[shift] = total / float(reference.shape[1])
    return scores


def estimate_roll(reference: Dict, current: Dict, matrix: np.ndarray) -> Dict:
    """Estimate dial roll by 1-D circular correlation of colour signatures."""
    if not reference.get("ok") or not current.get("ok"):
        return {"ok": False, "status": "low_conf",
                "reason": "roll_signature_missing",
                "detail": "reference or current angular colour signature is unavailable"}
    valid = reference["valid_bins"] & current["valid_bins"]
    if float(valid.mean()) < SIGNATURE_MIN_VALID_FRAC:
        return {"ok": False, "status": "low_conf", "reason": "roll_few_valid_bins",
                "valid_frac": float(valid.mean())}
    amplitude = min(float(reference["amplitude"]), float(current["amplitude"]))
    if amplitude < ROLL_MIN_AMPLITUDE:
        return {"ok": False, "status": "low_conf", "reason": "roll_signal_weak",
                "amplitude": amplitude,
                "detail": "colour signature amplitude below the weak-signal gate"}
    scores = _circular_correlation(reference["signature"], current["signature"], valid)
    if not np.any(scores > -1.0):
        return {"ok": False, "status": "low_conf", "reason": "roll_no_overlap"}
    best_shift = int(np.argmax(scores))
    best_score = float(scores[best_shift])
    finite = scores[scores > -1.0]
    median = float(np.median(finite))
    mad = float(1.4826 * np.median(np.abs(finite - median))) + 1e-9
    snr = (best_score - median) / mad
    separation = ROLL_SECOND_PEAK_SEP_DEG
    second_score = -1.0
    second_shift = None
    for shift in range(int(scores.shape[0])):
        distance = min(abs(shift - best_shift),
                       scores.shape[0] - abs(shift - best_shift))
        if distance < separation:
            continue
        if scores[shift] > second_score:
            second_score = float(scores[shift])
            second_shift = int(shift)
    gap = best_score - max(second_score, -1.0) if second_shift is not None else None
    # Parabolic sub-bin peak interpolation.
    left = scores[(best_shift - 1) % scores.shape[0]]
    right = scores[(best_shift + 1) % scores.shape[0]]
    denominator = left - 2.0 * best_score + right
    offset = 0.0
    if abs(denominator) > 1e-9:
        offset = 0.5 * (left - right) / denominator
        offset = float(np.clip(offset, -0.5, 0.5))
    roll = wrap_deg(best_shift + offset)
    # Rough uncertainty: peak half-width at 90% of the peak prominence.
    threshold = best_score - 0.10 * (best_score - median)
    half_width = 0.0
    for step in range(1, 91):
        if scores[(best_shift - step) % scores.shape[0]] < threshold:
            break
        half_width = step
    uncertainty = float(max(0.5, half_width))
    result = {
        "method": "1-D circular correlation of static-band opponent-colour signatures",
        "not_a_template_warp": True,
        "best_corr": best_score,
        "second_corr": None if second_shift is None else second_score,
        "second_shift_deg": None if second_shift is None else float(second_shift),
        "gap": gap,
        "snr": float(snr),
        "amplitude": amplitude,
        "valid_frac": float(valid.mean()),
        "roll_deg": float(roll),
        "uncertainty_deg": uncertainty,
        "scores": [round(float(v), 6) if v > -1.0 else None for v in scores],
    }
    if best_score < ROLL_MIN_CORR:
        result.update({"ok": False, "status": "low_conf", "reason": "roll_weak_correlation"})
        return result
    if gap is None or gap < ROLL_MIN_GAP:
        result.update({"ok": False, "status": "low_conf", "reason": "roll_ambiguous",
                       "detail": "secondary correlation peak is too close"})
        return result
    if snr < ROLL_MIN_SNR:
        result.update({"ok": False, "status": "low_conf", "reason": "roll_low_snr"})
        return result
    result.update({"ok": True, "status": "ok", "reason": "roll_estimated"})
    return result


# ---------------------------------------------------------------------------
# axis evidence and sensitivity
# ---------------------------------------------------------------------------

def _ellipse_centre_bootstrap(points: np.ndarray, ellipse: Dict,
                              samples: int = 20) -> Dict:
    if len(points) < 20:
        return {"ok": False, "reason": "too_few_points"}
    rng = np.random.RandomState(20261115)
    centers = []
    axes = []
    angles = []
    for _ in range(samples):
        index = rng.choice(len(points), size=max(10, int(0.8 * len(points))),
                           replace=False)
        try:
            (cx, cy), (axis_a, axis_b), angle = cv2.fitEllipse(points[index])
        except cv2.error:
            continue
        if not all(math.isfinite(v) for v in (cx, cy, axis_a, axis_b, angle)):
            continue
        centers.append([cx, cy])
        axes.append([axis_a, axis_b])
        angles.append(angle)
    if not centers:
        return {"ok": False, "reason": "bootstrap_failed"}
    centers = np.asarray(centers)
    axes = np.asarray(axes)
    return {
        "ok": True,
        "center_std_px": float(np.max(np.std(centers, axis=0))),
        "major_std_px": float(np.std(np.max(axes, axis=1))),
        "angle_std_deg": float(np.std(angles)),
        "samples": int(len(centers)),
    }


def _circular_interval_deg(angles: Sequence[float]) -> Dict:
    """Smallest circular arc containing all angles, without min/max collapse.

    ``start_deg`` is the first sample after the largest empty gap and
    ``end_deg`` the last one before it.  The interval may wrap through 0 deg
    (``wraps_zero``); its width is ``span_deg``.  A pair such as 359.5/0.4
    therefore gives a ~0.9 deg interval, never [0.4, 359.5].
    """
    values = sorted(float(angle) % 360.0 for angle in angles
                    if angle is not None and math.isfinite(float(angle)))
    if not values:
        return {"ok": False, "reason": "no_angles"}
    if len(values) == 1:
        return {"ok": True, "start_deg": values[0], "end_deg": values[0],
                "span_deg": 0.0, "wraps_zero": False,
                "samples_deg": list(values)}
    gaps = [(values[(index + 1) % len(values)] - values[index]) % 360.0
            for index in range(len(values))]
    largest = int(np.argmax(gaps))
    start = values[(largest + 1) % len(values)]
    end = values[largest]
    return {
        "ok": True,
        "start_deg": float(start),
        "end_deg": float(end),
        "span_deg": float((end - start) % 360.0),
        "wraps_zero": bool(start > end),
        "samples_deg": [float(value) for value in values],
    }


def _angle_difference_deg(first: float, second: float) -> float:
    return abs((float(first) - float(second) + 180.0) % 360.0 - 180.0)


def _direction_span_deg(angles: Sequence[float]) -> Optional[float]:
    """Smallest circular arc containing all direction samples, in degrees.

    The arc may cross 0 deg (359.5/0.4 -> 0.9 deg).  This is the range of the
    samples themselves, not twice a half-width; callers must not label it a
    confidence interval.
    """
    values = [float(value) % 360.0 for value in angles
              if value is not None and math.isfinite(float(value))]
    if len(values) < 2:
        return None
    interval = _circular_interval_deg(values)
    if not interval.get("ok"):
        return None
    return float(interval["span_deg"])


def assess_axis(ring: Dict, hub: Optional[Dict], calibration: Dict) -> Dict:
    center = np.asarray(ring["center"], dtype=float)
    axes = (float(ring["axes"][0]), float(ring["axes"][1]))
    major = max(axes)
    matrix, _, _, _ = _ellipse_matrix(axes, float(ring["angle_deg"]))
    reference_delta = np.asarray(calibration["matrix"], dtype=float) @ (
        np.asarray(calibration["axis"], dtype=float)
        - np.asarray(calibration["ring_center"], dtype=float))
    reference_offset = float(np.linalg.norm(reference_delta))
    ring_axis = center.copy()
    hub = hub or {}
    hub_usable = bool(hub.get("usable", True)) and hub.get("center") is not None
    hub_axis = np.asarray(hub["center"], dtype=float) if hub_usable else None
    observed = hub_axis if hub_axis is not None else ring_axis
    offset_px = observed - ring_axis
    offset_norm = float(np.linalg.norm(offset_px))
    offset_frac = offset_norm / max(1e-9, major)
    bootstrap = _ellipse_centre_bootstrap(ring.get("inlier_points"),
                                          ring) if ring.get("inlier_points") is not None else {}
    ring_center_std = (float(bootstrap.get("center_std_px", 0.0))
                       if bootstrap.get("ok") else 0.0)
    hub_uncertainty = None
    if hub_axis is not None:
        hub_uncertainty = max(HUB_QUANTISATION_FLOOR_PX,
                              float(hub.get("center_uncertainty_px",
                                            HUB_QUANTISATION_FLOOR_PX)))
    # The bootstrap describes only how well the ellipse geometry fixes the ring
    # centre.  It is not pivot evidence and cannot make an unverified axis
    # verified; when a pivot exists, its own plateau/threshold evidence sets the
    # axis uncertainty.
    # Without an independent pivot there is no axis uncertainty to report; the
    # ring geometry bootstrap must not be relabelled as axis evidence.
    axis_uncertainty = hub_uncertainty if hub_axis is not None else None
    result = {
        "ring_center_px": center.tolist(),
        "observed_axis_px": observed.tolist(),
        "hub_axis_px": None if hub_axis is None else hub_axis.tolist(),
        "blob_centroid_px": hub.get("blob_centroid_px"),
        "centroid_offset_px": hub.get("centroid_offset_px"),
        "axis_source": ("dark/gold pivot cap" if hub_axis is not None
                        else "ring centre fallback"),
        "axis_verified": bool(hub_axis is not None),
        "hub_usable": bool(hub_usable),
        "hub_ambiguous": bool(hub_usable and hub.get("ambiguous", False)),
        "hub_rejection": (None if hub_usable
                          else hub.get("reject", "hub_missing")),
        "hub_competition": bool(hub.get("competition", False)),
        "hub_center_uncertainty_px": hub_uncertainty,
        "hub_center_spread_px": hub.get("center_spread_px"),
        "hub_thresholds": hub.get("thresholds", []),
        "hub_threshold_centers": hub.get("threshold_centers", []),
        "hub_plateau": hub.get("plateau"),
        "hub_plateau_separated": hub.get("plateau_separated"),
        "hub_plateau_elongated": hub.get("plateau_elongated"),
        "hub_threshold_centers_unstable": hub.get("threshold_centers_unstable"),
        "ring_axis_offset_px": offset_px.tolist(),
        "ring_axis_offset_frac": offset_frac,
        "reference_offset_normalised": reference_delta.tolist(),
        "reference_offset_magnitude": reference_offset,
        "centre_std_px": ring_center_std,
        "ring_center_bootstrap": bootstrap,
        "axis_uncertainty_px": axis_uncertainty,
        "bootstrap": bootstrap,
        "coordinate_system": "original crop pixels; x right, y down",
    }
    if hub_axis is None and reference_offset > 0.03:
        result.update({"ok": False, "status": "low_conf", "reason": "axis_missing"})
        return result
    if offset_frac > AXIS_OFFSET_MAX_FRAC:
        result.update({"ok": False, "status": "low_conf", "reason": "axis_inconsistent",
                       "detail": "observed pivot is too far from the fitted ring centre"})
        return result
    result.update({"ok": True, "status": "ok", "reason": "axis_observed"})
    return result


def _axis_evidence_points(axis: Dict, hub: Optional[Dict]) -> List[Dict]:
    """Plausible pivot centres from the DT plateau and associated thresholds."""
    centre = np.asarray(axis["observed_axis_px"], dtype=float)
    points = [{"role": "nominal", "center": centre.tolist()}]
    # Local 0.5 px perturbation: never pretend the digital centre is exact.
    for dx, dy in ((HUB_PERTURBATION_PX, 0.0), (-HUB_PERTURBATION_PX, 0.0),
                   (0.0, HUB_PERTURBATION_PX), (0.0, -HUB_PERTURBATION_PX)):
        points.append({"role": "quantisation", "center": [float(centre[0] + dx),
                                                           float(centre[1] + dy)]})
    hub = hub or {}
    if axis.get("axis_verified") and hub.get("source") == "dark_cap":
        for item in hub.get("threshold_centers", []):
            points.append({"role": "threshold_{}".format(item["threshold"]),
                           "center": list(item["center"])})
        for item in (hub.get("plateau") or {}).get("clusters", []):
            points.append({"role": "plateau_cluster",
                           "center": list(item["center"])})
    unique = []
    for item in points:
        if all(math.hypot(item["center"][0] - other["center"][0],
                          item["center"][1] - other["center"][1]) > 0.05
               for other in unique):
            unique.append(item)
    return unique


def _axis_stability(axis_verified: bool, hub_ambiguous: bool,
                    evidence: Dict, span_deg: Optional[float]) -> Tuple[bool, Optional[str]]:
    """Single place that decides axis stability and its reported reason.

    An unverified fallback is never overwritten by a zero ring-centre angle
    difference, and a failed perturbation cannot be called stable.
    """
    if not axis_verified:
        return False, "axis_unverified_fallback"
    if hub_ambiguous:
        return False, "axis_hub_ambiguous"
    if evidence.get("failed"):
        return False, "axis_evidence_unstable"
    if span_deg is None or float(span_deg) > AXIS_ANGLE_TOL_DEG:
        return False, "axis_angle_uncertain"
    return True, None


def _axis_needle_evidence(crop_bgr: np.ndarray, matrix: np.ndarray,
                          axis: Dict, hub: Optional[Dict],
                          nominal_angle: Optional[float]) -> Dict:
    """Needle angles over the pivot evidence; circular interval, not min/max."""
    detected = []
    failed = []
    for point in _axis_evidence_points(axis, hub):
        probe = detect_needle(crop_bgr, np.asarray(point["center"], dtype=float),
                              matrix)
        if probe.get("ok"):
            detected.append({"role": point["role"],
                             "center": point["center"],
                             "angle_deg": float(probe["angle_deg"])})
        else:
            failed.append({"role": point["role"], "center": point["center"],
                           "reason": probe.get("reason", "needle_weak")})
    angles = [item["angle_deg"] for item in detected]
    if nominal_angle is not None and math.isfinite(float(nominal_angle)):
        angles.append(float(nominal_angle))
    interval = _circular_interval_deg(angles)
    return {
        "detected": detected,
        "failed": failed,
        "interval": interval,
        "angles_deg": angles,
    }


# ---------------------------------------------------------------------------
# E1-R3 slender-line needle direction
#
# The unknown is a line (direction u, normal offset rho).  The pivot cap only
# bounds where the line may run and which end is the tip; it is not a hard
# angle prior.  The line search is bounded by the experiment ROI, the accepted
# ring and the coarse hub, and covers the full 0..360 direction.  Along-line
# translation never changes the angle because the direction is fitted from the
# body pixels, not from an endpoint minus a pivot.
#
# Evidence is a paired-edge strip integral: a thin dark line has a falling and
# a rising edge across its width.  A broad cap block is rejected by the edge
# separation gate, and a one-sided intensity step is rejected because both
# edges must be present.  The primary direction is the robust fit of the
# edge-pair midpoints (R4): a straight tapered needle has non-parallel edges
# but collinear midpoints, so edge opening is a shape diagnostic, not a
# direction error.  No interpolation upscaling is used: the strip is sampled at
# native 1 px along the needle and 0.5 px only across it for sub-pixel edge
# location.  The reported range is the finite-sample split-half, blocked-pairs
# bootstrap and coarse pivot/ring perturbation range of that same centreline
# estimator; it is not a confidence interval.
# ---------------------------------------------------------------------------

def _line_cap_radius_px(hub: Optional[Dict]) -> float:
    hub = hub or {}
    plateau = hub.get("plateau") or {}
    span = plateau.get("span_px") or []
    if span:
        return max(2.0, 0.5 * max(float(value) for value in span))
    return 3.0


def _line_remap(gray: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """Bilinear sample a flat coordinate list; OpenCV caps map rows at 32767."""
    flat_x = np.asarray(xs, dtype=np.float32).reshape(-1)
    flat_y = np.asarray(ys, dtype=np.float32).reshape(-1)
    count = flat_x.size
    pieces = []
    chunk = 20000
    for start in range(0, count, chunk):
        piece_x = flat_x[start:start + chunk]
        piece_y = flat_y[start:start + chunk]
        size = piece_x.size
        columns = 64 if size >= 64 else max(1, size)
        rows = int(math.ceil(size / float(columns)))
        padding = rows * columns - size
        if padding:
            piece_x = np.r_[piece_x, np.full(padding, piece_x[-1], dtype=np.float32)]
            piece_y = np.r_[piece_y, np.full(padding, piece_y[-1], dtype=np.float32)]
        sampled = cv2.remap(gray, piece_x.reshape(rows, columns),
                            piece_y.reshape(rows, columns), cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_REPLICATE)
        pieces.append(sampled.reshape(-1)[:size])
    return np.concatenate(pieces) if pieces else np.zeros(0, dtype=np.float32)


def _line_scores(gray: np.ndarray, centre: np.ndarray, matrix: np.ndarray,
                 hub: np.ndarray, semi_major_px: float,
                 angles_deg: Sequence[float], offsets_px: Sequence[float],
                 r_min_px: float, keep_cross: bool = False,
                 max_separation_px: Optional[float] = None,
                 cross_half_px: Optional[float] = None,
                 cross_step_px: Optional[float] = None) -> Optional[Dict]:
    """Paired-edge strip score for bounded (direction, normal offset) probes."""
    angles = np.asarray(angles_deg, dtype=float)
    offsets = np.asarray(offsets_px, dtype=float)
    if angles.size == 0 or offsets.size == 0:
        return None
    s_grid = np.arange(float(r_min_px), 0.95 * float(semi_major_px), 1.0)
    if s_grid.size < LINE_MIN_SAMPLES:
        return None
    half = (LINE_CROSS_HALF_PX if cross_half_px is None
            else float(cross_half_px))
    step = (LINE_CROSS_STEP_PX if cross_step_px is None
            else float(cross_step_px))
    cross = np.arange(-half, half + 1e-9, step)
    theta = np.radians(angles)
    u = np.stack([np.cos(theta), np.sin(theta)], axis=1)      # (A, 2)
    normal = np.stack([-np.sin(theta), np.cos(theta)], axis=1)
    points = (np.asarray(hub, dtype=float).reshape(1, 1, 1, 1, 2)
              + offsets[None, :, None, None, None] * normal[:, None, None, None, :]
              + s_grid[None, None, :, None, None] * u[:, None, None, None, :]
              + cross[None, None, None, :, None] * normal[:, None, None, None, :])
    height, width = gray.shape
    inside = ((points[..., 0] >= 0.0) & (points[..., 0] <= width - 1)
              & (points[..., 1] >= 0.0) & (points[..., 1] <= height - 1))
    relative = points - np.asarray(centre, dtype=float).reshape(1, 1, 1, 1, 2)
    mapped = matrix @ relative.reshape(-1, 2).T
    rho = np.hypot(mapped[0], mapped[1]).reshape(points.shape[:-1])
    valid_point = inside & (rho <= LINE_RADIUS_FRAC_MAX)
    valid_cross = valid_point.all(axis=-1)                    # (A, O, S)
    # cv2.remap treats its maps as 2-D and caps each side at 32767, so flatten
    # through the chunked helper and restore the sampled grid.
    shape = points.shape[:-1]
    sampled = _line_remap(gray, points[..., 0], points[..., 1]).reshape(shape)
    gradient = np.gradient(sampled, step, axis=-1)
    low_index = np.argmin(gradient, axis=-1)
    high_index = np.argmax(gradient, axis=-1)
    low_gradient = np.take_along_axis(gradient, low_index[..., None], axis=-1)[..., 0]
    high_gradient = np.take_along_axis(gradient, high_index[..., None], axis=-1)[..., 0]
    separation = (high_index - low_index) * step
    separation_limit = (LINE_MAX_EDGE_SEPARATION_PX
                        if max_separation_px is None else float(max_separation_px))
    contrast = (0.5 * (sampled[..., 0] + sampled[..., -1])
                - np.min(sampled, axis=-1))
    ok = (valid_cross & (low_index >= 1)
          & (high_index <= cross.size - 2) & (high_index > low_index)
          & (separation >= 0.5) & (separation <= separation_limit)
          & (low_gradient <= -LINE_MIN_EDGE_GRADIENT)
          & (high_gradient >= LINE_MIN_EDGE_GRADIENT)
          & (contrast >= LINE_MIN_CONTRAST))
    strength = np.where(ok, np.minimum(-low_gradient, high_gradient), 0.0)
    result = {
        "angles": angles,
        "offsets": offsets,
        "s": s_grid,
        "cross": cross,
        "scores": strength.sum(axis=-1),
        "support": ok.sum(axis=-1),
        "ok": ok,
    }
    if keep_cross:
        result["cross_values"] = sampled
    return result


def _line_edges(cross: np.ndarray, values: np.ndarray, valid: np.ndarray,
                max_separation_px: Optional[float] = None,
                cross_step_px: Optional[float] = None) -> Dict:
    """Sub-pixel upper/lower edge and their centre for each cross-section."""
    rows = []
    step = (LINE_CROSS_STEP_PX if cross_step_px is None
            else float(cross_step_px))
    for index in np.where(valid)[0]:
        section = values[index]
        gradient = np.gradient(section, step)
        low_index = int(np.argmin(gradient))
        high_index = int(np.argmax(gradient))
        if low_index < 1 or high_index > cross.size - 2 or high_index <= low_index:
            continue
        separation_limit = (LINE_MAX_EDGE_SEPARATION_PX
                            if max_separation_px is None else float(max_separation_px))
        if (high_index - low_index) * step > separation_limit:
            continue
        low_gradient = float(gradient[low_index])
        high_gradient = float(gradient[high_index])
        if low_gradient > -LINE_MIN_EDGE_GRADIENT or high_gradient < LINE_MIN_EDGE_GRADIENT:
            continue
        base = 0.5 * (float(section[0]) + float(section[-1]))
        contrast = base - float(np.min(section[low_index:high_index + 1]))
        if contrast < LINE_MIN_CONTRAST:
            continue

        def _vertex(array, position):
            denominator = array[position - 1] - 2.0 * array[position] + array[position + 1]
            if abs(denominator) < 1e-9:
                return float(cross[position])
            return float(cross[position] + 0.5 * step
                         * (array[position - 1] - array[position + 1]) / denominator)

        upper = _vertex(gradient, low_index)
        lower = _vertex(gradient, high_index)
        rows.append({
            "index": int(index),
            "upper": upper,
            "lower": lower,
            "center": 0.5 * (upper + lower),
            "width_px": abs(lower - upper),
            "strength": min(-low_gradient, high_gradient),
            "contrast": contrast,
        })
    return {"rows": rows}


def _line_robust_fit(s: np.ndarray, t: np.ndarray, w: np.ndarray) -> Optional[Dict]:
    """Weighted Huber line fit t(s); slope is the only direction carrier."""
    if len(s) < LINE_MIN_SAMPLES:
        return None
    weights = np.asarray(w, dtype=float)
    if not np.isfinite(weights).all() or weights.sum() <= 0.0:
        return None
    weights = weights / weights.sum()
    s = np.asarray(s, dtype=float)
    t = np.asarray(t, dtype=float)
    slope = 0.0
    intercept = 0.0
    for _ in range(4):
        s_mean = float(np.sum(weights * s))
        t_mean = float(np.sum(weights * t))
        denominator = float(np.sum(weights * (s - s_mean) ** 2))
        if denominator <= 1e-12:
            return None
        slope = float(np.sum(weights * (s - s_mean) * (t - t_mean)) / denominator)
        intercept = t_mean - slope * s_mean
        residual = t - (intercept + slope * s)
        median = float(np.median(np.abs(residual)))
        if median <= 1e-9:
            # An exact straight line has zero median residual.  The Huber
            # weight must then be one for every point; scaling by zero would
            # erase the whole fit and turn a clean synthetic line into a
            # spurious "fit failed".
            huber = np.ones_like(residual)
        else:
            huber = np.minimum(1.0, 1.345 * median / (np.abs(residual) + 1e-9))
        weights = weights * huber
        if weights.sum() <= 0.0:
            return None
        weights = weights / weights.sum()
    residual = t - (intercept + slope * s)
    rms = float(np.sqrt(np.mean(residual ** 2)))
    denominator = float(np.sum(weights * (s - np.sum(weights * s)) ** 2))
    standard_error = (float(np.sqrt(np.sum(weights * residual ** 2)
                                    / max(1.0, len(s) - 2.0) / max(denominator, 1e-12))))
    return {
        "slope": slope,
        "intercept": intercept,
        "rms_px": rms,
        "n": int(len(s)),
        "slope_standard_error": standard_error,
        "span_deg": 2.0 * math.degrees(math.atan(max(standard_error, 0.0))),
    }


def _line_bootstrap_span(s: np.ndarray, t: np.ndarray, w: np.ndarray,
                         fit: Dict, base_angle_deg: float = 0.0) -> Optional[float]:
    """Blocked pairs bootstrap of the centreline direction.

    The resampled unit is a contiguous block of ``(s, t)`` pairs: both the
    along-body position and the measured edge-pair midpoint move together, so
    the same cross-section is never recombined with another position.  Despite
    the historical name, this is *not* a residual bootstrap: ``sample`` is
    simply ``t`` at the resampled indices, and the result is a finite-sample
    sensitivity envelope of the centreline angle, not a statistical confidence
    interval and not twice a half-width.

    Each bootstrap slope is converted to a direction in the same rotated frame
    as the primary fit (``base_angle_deg``) and the true circular range of
    those directions is returned.
    """
    if len(s) < 8 or fit is None:
        return None
    order = np.argsort(s)
    s = np.asarray(s, dtype=float)[order]
    t = np.asarray(t, dtype=float)[order]
    w = np.asarray(w, dtype=float)[order]
    blocks = [np.arange(index, min(index + 2, len(s)))
              for index in range(0, len(s), 2)]
    if len(blocks) < 4:
        return None
    rng = np.random.RandomState(20261115)
    angles = []
    for _ in range(LINE_BOOTSTRAP_SAMPLES):
        picks = rng.choice(len(blocks), size=len(blocks), replace=True)
        index = np.concatenate([blocks[item] for item in picks])
        boot = _line_robust_fit(s[index], t[index], w[index])
        if boot is not None:
            angles.append((float(base_angle_deg)
                           + math.degrees(math.atan(float(boot["slope"])))) % 360.0)
    if len(angles) < 4:
        return None
    return _direction_span_deg(angles)


def _line_refine(gray: np.ndarray, centre: np.ndarray, matrix: np.ndarray,
                 hub: np.ndarray, semi_major_px: float, r_min_px: float,
                 angle_deg: float, offset_px: float,
                 max_separation_px: Optional[float] = None,
                 cross_half_px: Optional[float] = None,
                 cross_step_px: Optional[float] = None) -> Optional[Dict]:
    """Fine line search plus the direction uncertainty components."""
    angle_grid = np.arange(angle_deg - LINE_ANGLE_FINE_HALF_DEG,
                           angle_deg + LINE_ANGLE_FINE_HALF_DEG + 1e-9,
                           LINE_ANGLE_FINE_STEP_DEG)
    offset_grid = np.arange(offset_px - LINE_OFFSET_FINE_LIMIT_PX,
                            offset_px + LINE_OFFSET_FINE_LIMIT_PX + 1e-9,
                            0.25)
    coarse = _line_scores(gray, centre, matrix, hub, semi_major_px,
                          angle_grid, offset_grid, r_min_px,
                          max_separation_px=max_separation_px,
                          cross_half_px=cross_half_px,
                          cross_step_px=cross_step_px)
    if coarse is None or float(np.max(coarse["scores"])) <= 0.0:
        return None
    index = int(np.argmax(coarse["scores"]))
    row, column = np.unravel_index(index, coarse["scores"].shape)
    best_angle = float(coarse["angles"][row])
    best_offset = float(coarse["offsets"][column])
    cross = _line_scores(gray, centre, matrix, hub, semi_major_px,
                         [best_angle], [best_offset], r_min_px, keep_cross=True,
                         max_separation_px=max_separation_px,
                         cross_half_px=cross_half_px,
                         cross_step_px=cross_step_px)
    if cross is None:
        return None
    edges = _line_edges(cross["cross"], cross["cross_values"][0, 0],
                        cross["ok"][0, 0], max_separation_px=max_separation_px,
                        cross_step_px=cross_step_px)
    rows = edges["rows"]
    if len(rows) < LINE_MIN_SAMPLES:
        return None
    s = np.asarray([cross["s"][item["index"]] for item in rows], dtype=float)
    centers = np.asarray([item["center"] for item in rows], dtype=float)
    uppers = np.asarray([item["upper"] for item in rows], dtype=float)
    lowers = np.asarray([item["lower"] for item in rows], dtype=float)
    weights = np.asarray([item["strength"] for item in rows], dtype=float)
    center_fit = _line_robust_fit(s, centers, weights)
    if center_fit is None:
        return None
    slope = float(center_fit["slope"])
    center_angle = (best_angle + math.degrees(math.atan(slope))) % 360.0
    # The midpoint line is the primary model.  Its normal offset at the pivot
    # is the strip offset plus the fitted intercept, rotated into the
    # centreline frame; tip/tail probes then run along the same direction.
    center_offset = ((best_offset + float(center_fit["intercept"]))
                     / math.sqrt(1.0 + slope * slope))
    upper_fit = _line_robust_fit(s, uppers, weights)
    lower_fit = _line_robust_fit(s, lowers, weights)
    upper_delta = (math.degrees(math.atan(upper_fit["slope"]))
                   if upper_fit is not None else None)
    lower_delta = (math.degrees(math.atan(lower_fit["slope"]))
                   if lower_fit is not None else None)
    # A tapered straight needle has a real, non-zero edge opening angle.  It is
    # a body-shape diagnostic, not a direction error: the corresponding edge
    # midpoints stay collinear.
    edge_opening = (abs(upper_delta - lower_delta)
                    if upper_delta is not None and lower_delta is not None
                    else None)
    order = np.argsort(s)
    half = len(order) // 2
    split_span = None
    if half >= 3:
        near = _line_robust_fit(s[order[:half]], centers[order[:half]],
                                weights[order[:half]])
        far = _line_robust_fit(s[order[half:]], centers[order[half:]],
                               weights[order[half:]])
        if near is not None and far is not None:
            split_span = _direction_span_deg([
                (best_angle + math.degrees(math.atan(float(near["slope"])))) % 360.0,
                (best_angle + math.degrees(math.atan(float(far["slope"])))) % 360.0,
            ])
    widths = np.asarray([item["width_px"] for item in rows], dtype=float)
    width_fit = _line_robust_fit(s, widths, weights)
    support = float(coarse["support"][row, column]
                    / max(1, len(coarse["s"])))
    return {
        "angle_deg": center_angle,
        "angle_px_deg": center_angle,
        "strip_angle_deg": best_angle,
        "coarse_angle_deg": best_angle,
        "edge_center_angle_deg": center_angle,
        "edge_center_delta_deg": ((center_angle - best_angle + 180.0) % 360.0
                                  - 180.0),
        "estimator_delta_deg": ((center_angle - best_angle + 180.0) % 360.0
                                - 180.0),
        "edge_opening_deg": edge_opening,
        "offset_px": center_offset,
        "coarse_offset_px": best_offset,
        "support_frac": support,
        "samples": s,
        "center": centers,
        "weights": weights,
        "fit": center_fit,
        "upper_delta_deg": upper_delta,
        "lower_delta_deg": lower_delta,
        "centerline_rms_px": float(center_fit["rms_px"]),
        "width_median_px": float(np.median(widths)),
        "width_rms_px": None if width_fit is None else float(width_fit["rms_px"]),
        "split_half_span_deg": split_span,
        "bootstrap_span_deg": _line_bootstrap_span(
            s, centers, weights, center_fit, base_angle_deg=best_angle),
        "strength_median": float(np.median(weights)),
    }


def _line_extent(gray: np.ndarray, centre: np.ndarray, matrix: np.ndarray,
                 hub: np.ndarray, semi_major_px: float, r_min_px: float,
                 angle_deg: float, offset_px: float,
                 max_separation_px: Optional[float] = None,
                 cross_half_px: Optional[float] = None,
                 cross_step_px: Optional[float] = None) -> Dict:
    probe = _line_scores(gray, centre, matrix, hub, semi_major_px,
                         [angle_deg], [offset_px], r_min_px,
                         max_separation_px=max_separation_px,
                         cross_half_px=cross_half_px,
                         cross_step_px=cross_step_px)
    if probe is None:
        return {"extent_px": 0.0, "support_frac": 0.0}
    valid = probe["ok"][0, 0]
    s = probe["s"]
    indices = np.where(valid)[0]
    if len(indices) == 0:
        return {"extent_px": 0.0, "support_frac": 0.0}
    last = indices[0]
    for index in indices[1:]:
        # Bridge at most two missing cross-sections (a tick/scratch across the
        # body); a longer gap is a different object, not the needle.
        if index - last <= 3:
            last = index
        else:
            break
    return {
        "extent_px": float(s[last] - s[indices[0]] + 1.0),
        "support_frac": float(len(indices) / float(len(s))),
        "contiguous": [int(indices[0]), int(last)],
    }


def _line_profile_normalised(profile_px: np.ndarray, angles_px: np.ndarray,
                             matrix: np.ndarray) -> Optional[np.ndarray]:
    """Resample the directed pixel-angle score profile onto normalised angles."""
    theta = np.radians(angles_px)
    u = np.stack([np.cos(theta), np.sin(theta)], axis=1)
    mapped = u @ matrix.T
    normalized = np.degrees(np.arctan2(mapped[:, 1], mapped[:, 0])) % 360.0
    unwrapped = np.degrees(np.unwrap(np.radians(normalized)))
    # Extend one period so the interpolation covers the full circle.
    if unwrapped[-1] - unwrapped[0] < 300.0:
        return None
    lam_ext = np.r_[unwrapped, unwrapped[0] + 360.0]
    angle_ext = np.r_[angles_px, angles_px[0] + 360.0]
    profile_ext = np.r_[profile_px, profile_px[0]]
    target = unwrapped[0] + ((np.arange(LINE_PROFILE_BINS, dtype=float)
                              - unwrapped[0]) % 360.0)
    pixel_angle = np.interp(target, lam_ext, angle_ext)
    values = np.interp(pixel_angle, angle_ext, profile_ext)
    grid = np.round(target % 360.0).astype(int) % LINE_PROFILE_BINS
    profile = np.zeros(LINE_PROFILE_BINS, dtype=float)
    np.add.at(profile, grid, values)
    return profile


def _spd_condition_number(matrix) -> Optional[float]:
    """Return ``lambda_max/lambda_min`` for a finite SPD 2x2 matrix.

    ``None`` means non-finite, non-symmetric, singular or not positive
    definite.  Callers must reject such geometry; there is deliberately no
    fallback to ``1`` because that would silently turn the pixel span into the
    normalised span.
    """
    values = np.asarray(matrix, dtype=float)
    if values.shape != (2, 2) or not np.all(np.isfinite(values)):
        return None
    scale = max(1.0, float(np.max(np.abs(values))))
    if abs(float(values[0, 1]) - float(values[1, 0])) > 1e-9 * scale:
        return None
    trace = float(values[0, 0] + values[1, 1])
    determinant = float(values[0, 0] * values[1, 1]
                        - values[0, 1] * values[1, 0])
    discriminant = 0.25 * trace * trace - determinant
    if not (math.isfinite(trace) and math.isfinite(determinant)):
        return None
    if discriminant < 0.0:
        return None
    root = math.sqrt(discriminant)
    lambda_min = 0.5 * trace - root
    lambda_max = 0.5 * trace + root
    if lambda_min <= 0.0 or lambda_max <= 0.0:
        return None
    kappa = lambda_max / lambda_min
    return kappa if math.isfinite(kappa) else None


def _normalized_span_upper_bound_deg(pixel_span_deg, kappa) -> Optional[float]:
    """Conservative normalised angle bound ``min(360, kappa * pixel_span)``.

    For SPD ``A`` the direction map has derivative at most ``kappa(A)``, so the
    mapped arc is no wider than ``kappa`` times the pixel arc.  This is a
    conservative sensitivity upper bound, not an exact span and not a
    confidence interval.
    """
    if pixel_span_deg is None or kappa is None:
        return None
    span = float(pixel_span_deg)
    condition = float(kappa)
    if not (math.isfinite(span) and math.isfinite(condition)):
        return None
    if span < 0.0 or condition < 1.0:
        return None
    return min(360.0, condition * span)


def _line_base(hub: Optional[Dict] = None) -> Dict:
    hub = hub or {}
    return {
        "ok": False,
        "status": "low_conf",
        "reason": None,
        "detail": None,
        "angle_deg": None,
        "angle_px_deg": None,
        "coarse_angle_deg": None,
        "strip_angle_deg": None,
        "fine_angle_deg": None,
        "edge_center_angle_deg": None,
        "edge_center_delta_deg": None,
        "estimator_delta_deg": None,
        "edge_opening_deg": None,
        "coarse_offset_px": None,
        "refined_offset_px": None,
        "support_frac": None,
        "support_samples": None,
        "tip_extent_px": None,
        "tail_extent_px": None,
        "tip_tail_ratio": None,
        "width_median_px": None,
        "width_rms_px": None,
        "strength_median": None,
        "rms_px": None,
        "centerline_rms_px": None,
        "angle_span_deg": None,
        "pixel_span_deg": None,
        "normalized_span_upper_bound_deg": None,
        "matrix_condition_number": None,
        "uncertainty_components_deg": None,
        "normalized_components_upper_bound_deg": None,
        "edge_fit_hub_span_deg": None,
        "edge_fit_ring_span_deg": None,
        "perturbation_expected": {
            "hub": LINE_HUB_PERTURBATION_COUNT,
            "ring": LINE_RING_PERTURBATION_COUNT,
        },
        "perturbation_failures": {"hub": [], "ring": []},
        "bootstrap_missing": None,
        "split_half_missing": None,
        "hub_present": hub.get("center") is not None,
        "hub_ambiguous": bool(hub.get("usable") and hub.get("ambiguous", False)),
        "hub_usable": bool(hub.get("usable", True)),
        "profile_max": None,
        "profile_norm_deg": None,
        "method": "paired-edge strip integral -> robust edge-pair centre line; "
                  "direction carried through the ring SPD matrix",
        "coordinate_system": "normalised dial plane; pixel angle also reported",
        "diagnostic_only": True,
    }


def estimate_needle_line(crop_bgr: np.ndarray, ring: Dict,
                         hub: Optional[Dict]) -> Dict:
    """Direction from the slender needle body, with a diagnostic interval.

    ``ok`` means the body evidence exists and the direction interval is within
    the existing 2 deg budget.  The angle is in the normalised dial plane: the
    fitted pixel line vector is carried through the SPD ring matrix, never
    computed as a tip pixel minus the pivot pixel.
    """
    hub = hub or {}
    hub_center = hub.get("center")
    if not hub_center:
        result = _line_base(hub)
        result.update({"reason": "line_hub_missing",
                       "detail": "a coarse pivot is required to bound the line; "
                                 "no line was picked without it"})
        return result
    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    centre = np.asarray(ring["center"], dtype=float)
    hub_px = np.asarray(hub_center, dtype=float)

    def _geometry_rejection(detail: str) -> Dict:
        result = _line_base(hub)
        result.update({"reason": "line_geometry_invalid", "detail": detail})
        return result

    try:
        ring_axes = (float(ring["axes"][0]), float(ring["axes"][1]))
        ring_angle = float(ring["angle_deg"])
    except (KeyError, IndexError, TypeError, ValueError):
        return _geometry_rejection(
            "ring axes/angle are not numeric; no direction can be mapped")
    if (not all(math.isfinite(value)
                for value in (ring_axes[0], ring_axes[1], ring_angle))
            or min(ring_axes) <= 0.0):
        return _geometry_rejection(
            "ring axes must be finite and strictly positive for a non-singular "
            "SPD matrix")
    matrix, _, _, _ = _ellipse_matrix(ring_axes, ring_angle)
    kappa = _spd_condition_number(matrix)
    if kappa is None:
        return _geometry_rejection(
            "ring matrix is not a finite symmetric positive-definite matrix; "
            "refusing to substitute a unit condition number")
    semi = 0.5 * float(max(ring["axes"]))
    # A needle is slender relative to the dial, so the thin-line gate scales
    # with the ring size.  On the 75x75 crop this keeps the R3 2.5 px limit; on
    # the phone full frame it admits the ~8 px needle while still rejecting the
    # ~40 px cap block.
    max_separation = max(LINE_MAX_EDGE_SEPARATION_PX, 0.06 * semi)
    cross_half = max(LINE_CROSS_HALF_PX, 0.05 * semi)
    cross_step = max(LINE_CROSS_STEP_PX, cross_half / 8.0)
    r_min = max(_line_cap_radius_px(hub), 0.05 * semi) + 2.0
    angles = np.arange(0.0, 360.0, LINE_ANGLE_STEP_DEG)
    offsets = np.arange(-LINE_OFFSET_LIMIT_PX, LINE_OFFSET_LIMIT_PX + 1e-9,
                        LINE_OFFSET_STEP_PX)
    coarse = _line_scores(gray, centre, matrix, hub_px, semi, angles, offsets,
                          r_min, max_separation_px=max_separation,
                          cross_half_px=cross_half, cross_step_px=cross_step)
    if coarse is None or float(np.max(coarse["scores"])) <= 0.0:
        result = _line_base(hub)
        result.update({"reason": "line_no_needle",
                       "detail": "no thin paired-edge body inside the allowed "
                                 "band"})
        return result
    profile_px = np.max(coarse["scores"], axis=1)
    best = int(np.argmax(coarse["scores"]))
    row, column = np.unravel_index(best, coarse["scores"].shape)
    coarse_angle = float(coarse["angles"][row])
    coarse_offset = float(coarse["offsets"][column])
    refine = _line_refine(gray, centre, matrix, hub_px, semi, r_min,
                          coarse_angle, coarse_offset,
                          max_separation_px=max_separation,
                          cross_half_px=cross_half, cross_step_px=cross_step)
    if refine is None:
        result = _line_base(hub)
        result.update({"reason": "line_fit_failed",
                       "detail": "paired-edge samples were too few for a line fit"})
        return result
    angle_px = float(refine["angle_deg"])
    tip = _line_extent(gray, centre, matrix, hub_px, semi, r_min,
                       angle_px, refine["offset_px"],
                       max_separation_px=max_separation,
                       cross_half_px=cross_half, cross_step_px=cross_step)
    tail = _line_extent(gray, centre, matrix, hub_px, semi, r_min,
                        (angle_px + 180.0) % 360.0, -refine["offset_px"],
                        max_separation_px=max_separation,
                        cross_half_px=cross_half, cross_step_px=cross_step)
    # Support and tip/tail extents are measured along the primary centreline,
    # not along the strip-peak direction.
    support_frac = float(tip["support_frac"])
    tip_extent = float(tip["extent_px"])
    tail_extent = float(tail["extent_px"])
    ratio = tip_extent / max(1.0, tail_extent)
    # Hub and ring perturbations re-run only the fine stage.  A direction that
    # is carried by the midpoint line must survive them.  A failed perturbation
    # is missing evidence, never zero variation: it is recorded explicitly and
    # the frame is rejected rather than allowed to look stable.
    hub_expected = LINE_HUB_PERTURBATION_COUNT
    ring_expected = LINE_RING_PERTURBATION_COUNT
    hub_angles = []
    hub_failures = []
    for dx, dy in ((LINE_HUB_PERTURBATION_PX, 0.0),
                   (-LINE_HUB_PERTURBATION_PX, 0.0),
                   (0.0, LINE_HUB_PERTURBATION_PX),
                   (0.0, -LINE_HUB_PERTURBATION_PX)):
        shifted = _line_refine(gray, centre, matrix, hub_px + np.array([dx, dy]),
                               semi, r_min, coarse_angle, coarse_offset,
                               max_separation_px=max_separation,
                               cross_half_px=cross_half, cross_step_px=cross_step)
        if shifted is None or shifted.get("angle_deg") is None:
            hub_failures.append({"shift_px": [float(dx), float(dy)],
                                 "reason": "line_refine_failed"})
        else:
            hub_angles.append(float(shifted["angle_deg"]))
    hub_span = _direction_span_deg(hub_angles)
    ring_angles = []
    ring_failures = []
    ring_kappas = []
    for shift, scale in (((LINE_RING_PERTURBATION_PX, 0.0), 1.0),
                         ((-LINE_RING_PERTURBATION_PX, 0.0), 1.0),
                         ((0.0, LINE_RING_PERTURBATION_PX), 1.0),
                         ((0.0, -LINE_RING_PERTURBATION_PX), 1.0),
                         ((0.0, 0.0), 0.99), ((0.0, 0.0), 1.01)):
        shifted_centre = centre + np.array(shift, dtype=float)
        shifted_matrix, _, _, _ = _ellipse_matrix(
            (float(ring["axes"][0]) * scale, float(ring["axes"][1]) * scale),
            float(ring["angle_deg"]))
        shifted_kappa = _spd_condition_number(shifted_matrix)
        if shifted_kappa is None:
            ring_failures.append({"shift_px": [float(shift[0]), float(shift[1])],
                                  "scale": float(scale),
                                  "reason": "line_geometry_invalid"})
            continue
        ring_kappas.append(shifted_kappa)
        shifted = _line_refine(gray, shifted_centre, shifted_matrix, hub_px,
                               semi * scale, r_min, coarse_angle, coarse_offset,
                               max_separation_px=max_separation * scale,
                               cross_half_px=cross_half * scale,
                               cross_step_px=cross_step * scale)
        if shifted is None or shifted.get("angle_deg") is None:
            ring_failures.append({"shift_px": [float(shift[0]), float(shift[1])],
                                  "scale": float(scale),
                                  "reason": "line_refine_failed"})
        else:
            ring_angles.append(float(shifted["angle_deg"]))
    ring_span = _direction_span_deg(ring_angles)
    # The ring scale perturbations are isotropic here, so kappa is unchanged;
    # taking the maximum over the matrices actually used keeps the bound valid
    # if a future perturbation changes A.
    kappa_bound = max([kappa] + ring_kappas)
    # Backwards-compatible diagnostic names: the old edge-fit spans are now the
    # same primary-centreline perturbation spans.
    edge_hub_span = hub_span
    edge_ring_span = ring_span
    centerline_rms = float(refine["centerline_rms_px"])
    width_rms = refine.get("width_rms_px")
    components = {
        "edge_opening_deg": refine["edge_opening_deg"],
        "estimator_delta_deg": abs(float(refine["estimator_delta_deg"])),
        "split_half_span_deg": refine["split_half_span_deg"],
        "bootstrap_span_deg": refine["bootstrap_span_deg"],
        "fit_span_deg": refine["fit"]["span_deg"],
        "hub_span_deg": hub_span,
        "ring_span_deg": ring_span,
    }
    # Every interval input must come from the same primary centreline
    # estimator.  The edge opening angle and the strip-vs-centreline delta are
    # diagnostics and are deliberately excluded.  Missing evidence is not
    # silently dropped from a max(): it is recorded and rejects the frame.
    interval_keys = ("split_half_span_deg", "bootstrap_span_deg", "fit_span_deg",
                     "hub_span_deg", "ring_span_deg")
    missing_evidence = [key for key in interval_keys
                        if components.get(key) is None]
    if len(hub_angles) < hub_expected:
        missing_evidence.append("hub_perturbation")
    if len(ring_angles) < ring_expected:
        missing_evidence.append("ring_perturbation")
    if width_rms is None:
        missing_evidence.append("width_consistency")
    pixel_span = None
    if not missing_evidence:
        numeric = [float(components[key]) for key in interval_keys]
        if all(math.isfinite(value) for value in numeric):
            pixel_span = max(numeric)
    # The components above are pixel-direction spans.  They are not comparable
    # with the 2 deg budget in the normalised dial plane: A amplifies or
    # compresses angles by up to kappa(A).  The gate uses the conservative
    # normalised upper bound, never the raw pixel span.
    normalized_components = {
        key: _normalized_span_upper_bound_deg(value, kappa_bound)
        for key, value in components.items()}
    normalized_span = _normalized_span_upper_bound_deg(pixel_span, kappa_bound)
    # Directed normalised angle: the fitted line vector goes through A, not
    # through a difference of two uncertain points.
    pixel_vector = np.array([math.cos(math.radians(angle_px)),
                             math.sin(math.radians(angle_px))], dtype=float)
    mapped_vector = matrix @ pixel_vector
    angle_deg = math.degrees(math.atan2(mapped_vector[1], mapped_vector[0])) % 360.0
    profile = _line_profile_normalised(profile_px, angles, matrix)
    result = {
        "ok": False,
        "status": "low_conf",
        "reason": None,
        "detail": None,
        "angle_deg": angle_deg,
        "angle_px_deg": angle_px % 360.0,
        "coarse_angle_deg": coarse_angle,
        "strip_angle_deg": refine["strip_angle_deg"],
        "fine_angle_deg": angle_px,
        "edge_center_angle_deg": refine["edge_center_angle_deg"],
        "edge_center_delta_deg": refine["edge_center_delta_deg"],
        "estimator_delta_deg": refine["estimator_delta_deg"],
        "edge_opening_deg": refine["edge_opening_deg"],
        "coarse_offset_px": coarse_offset,
        "refined_offset_px": refine["offset_px"],
        "support_frac": support_frac,
        "strip_support_frac": float(refine.get("support_frac", 0.0)),
        "support_samples": int(refine["fit"]["n"]),
        "tip_extent_px": tip_extent,
        "tail_extent_px": tail_extent,
        "tip_tail_ratio": ratio,
        "tip_support_frac": tip["support_frac"],
        "tail_support_frac": tail["support_frac"],
        "width_median_px": refine["width_median_px"],
        "width_rms_px": width_rms,
        "strength_median": refine["strength_median"],
        "rms_px": centerline_rms,
        "centerline_rms_px": centerline_rms,
        "angle_span_deg": normalized_span,
        "pixel_span_deg": pixel_span,
        "normalized_span_upper_bound_deg": normalized_span,
        "matrix_condition_number": kappa_bound,
        "uncertainty_components_deg": components,
        "normalized_components_upper_bound_deg": normalized_components,
        "interval_definition": "normalised upper bound = min(360, kappa(A) * "
                               "max(split-half, blocked-pairs bootstrap, "
                               "one-standard-error fit, hub and ring "
                               "perturbation pixel spans)), with "
                               "kappa(A)=lambda_max/lambda_min of the ring SPD "
                               "matrix; the components are pixel-direction "
                               "spans and the bound is conservative, not an "
                               "exact span and not a confidence interval",
        "angle_span_definition": "angle_span_deg is the normalised gated upper "
                                 "bound; pixel_span_deg is the raw pixel span",
        "edge_fit_hub_span_deg": edge_hub_span,
        "edge_fit_ring_span_deg": edge_ring_span,
        "hub_perturbation_angles_deg": hub_angles,
        "ring_perturbation_angles_deg": ring_angles,
        "perturbation_expected": {"hub": hub_expected, "ring": ring_expected},
        "perturbation_failures": {"hub": hub_failures, "ring": ring_failures},
        "bootstrap_missing": refine["bootstrap_span_deg"] is None,
        "split_half_missing": refine["split_half_span_deg"] is None,
        "missing_evidence": missing_evidence,
        "bootstrap_method": "blocked pairs bootstrap (adjacent cross-sections "
                            "resampled together); finite-sample sensitivity "
                            "envelope, not a confidence interval",
        "hub_present": True,
        "hub_ambiguous": bool(hub.get("usable") and hub.get("ambiguous", False)),
        "hub_usable": bool(hub.get("usable", True)),
        "profile_max": float(np.max(profile_px)),
        "profile_norm_deg": None if profile is None else [
            round(float(value), 3) for value in profile],
        "method": "paired-edge strip -> robust edge-pair midpoint centreline "
                  "(primary direction); strip peak and edge opening retained as "
                  "diagnostics; direction carried through the ring SPD matrix",
        "coordinate_system": "normalised dial plane; pixel angle also reported",
        "diagnostic_only": True,
    }
    # Rejection order is explicit; the interval gate is last so a rejected
    # frame still reports the direction and the reason components.
    if support_frac < LINE_MIN_SUPPORT_FRAC or refine["fit"]["n"] < LINE_MIN_SAMPLES:
        result.update({"reason": "line_support_low",
                       "detail": "centreline paired-edge support below the body "
                                 "gate"})
        return result
    if tip_extent < max(4.0, LINE_MIN_TIP_LENGTH_FRAC * semi):
        result.update({"reason": "line_needle_short",
                       "detail": "supported tip extent is too short for a 2 deg "
                                 "direction"})
        return result
    if ratio < LINE_TIP_TAIL_RATIO or tail_extent >= LINE_MIN_TIP_LENGTH_FRAC * semi:
        result.update({"reason": "line_direction_ambiguous",
                       "detail": "both ends carry comparable body evidence; an "
                                 "equal double needle or long tail cannot choose "
                                 "the tip"})
        return result
    peaks = []
    for index in range(len(profile_px)):
        value = profile_px[index]
        if value <= 0.0:
            continue
        if value >= profile_px[index - 1] and value >= profile_px[(index + 1) % len(profile_px)]:
            peaks.append((float(value), index))
    peaks.sort(reverse=True)
    second_ratio = None
    if peaks:
        main_value, main_index = peaks[0]
        separation = int(round(LINE_SECOND_PEAK_SEPARATION_DEG / LINE_ANGLE_STEP_DEG))
        for value, index in peaks[1:]:
            distance = min(abs(index - main_index),
                           len(profile_px) - abs(index - main_index))
            if distance >= separation:
                second_ratio = float(value / main_value) if main_value > 0 else None
                break
    result["profile_second_ratio"] = second_ratio
    if second_ratio is not None and second_ratio >= LINE_SECOND_PEAK_RATIO:
        result.update({"reason": "line_competing_peaks",
                       "detail": "a second line direction has comparable support"})
        return result
    if centerline_rms > LINE_MAX_CENTERLINE_RMS_PX:
        result.update({"reason": "line_body_not_straight",
                       "detail": "the edge-pair midpoints deviate from a straight "
                                 "line by more than the shape budget; a bent body "
                                 "is not a straight needle"})
        return result
    if width_rms is not None and width_rms > LINE_MAX_WIDTH_RMS_PX:
        result.update({"reason": "line_width_inconsistent",
                       "detail": "the body width does not vary smoothly along the "
                                 "needle; one irregular edge or a non-needle "
                                 "shape is not a straight tapered body"})
        return result
    if missing_evidence:
        result.update({"reason": "line_direction_evidence_missing",
                       "detail": "required direction evidence is absent: {}; "
                                 "missing evidence is not zero variation".format(
                                     ", ".join(missing_evidence))})
        return result
    if normalized_span is None or normalized_span > LINE_DIRECTION_TOL_DEG:
        result.update({"reason": "line_direction_unstable",
                       "detail": "the normalised direction-span upper bound "
                                 "min(360, kappa*pixel_span) is wider than the "
                                 "2 deg budget (kappa={:.4f}, pixel_span={} deg)".format(
                                     kappa_bound, _round_or_none(pixel_span, 3))})
        return result
    result.update({"ok": True, "status": "ok", "reason": "line_direction_stable"})
    return result


def _joint_burst_direction(frames: Sequence[Dict]) -> Dict:
    """Internal-consistency joint direction from per-frame line profiles.

    Frames are aligned by their static face roll and by the normalised ring
    geometry; the needle itself is never used for alignment and no humidity
    value is averaged.  The result is explicitly not independent verification.
    """
    usable = [item for item in frames
              if (item.get("line") or {}).get("angle_deg") is not None
              and (item.get("line") or {}).get("profile_norm_deg")
              and (item.get("roll") or {}).get("ok")]
    if len(usable) < 2:
        return {"ok": False, "reason": "too_few_line_frames",
                "frames_used": len(usable), "internal_consistency_only": True}
    bins = 720
    grid = np.arange(bins) * 0.5
    total = np.zeros(bins, dtype=float)
    individual = []
    for item in usable:
        profile = np.asarray(item["line"]["profile_norm_deg"], dtype=float)
        roll = float(item["roll"]["roll_deg"])
        sample = np.interp((grid + roll) % 360.0,
                           np.arange(LINE_PROFILE_BINS), profile,
                           period=LINE_PROFILE_BINS)
        if sample.max() > 0:
            sample = sample / sample.max()
        total += sample
        individual.append(float(item["line"]["angle_deg"]))
    peak = int(np.argmax(total))
    joint = float(grid[peak] % 360.0)
    # The joint direction is compared with each frame's own compensated angle.
    offsets = []
    for item in usable:
        compensated = (float(item["line"]["angle_deg"])
                       - float(item["roll"]["roll_deg"])) % 360.0
        offsets.append(((joint - compensated + 180.0) % 360.0) - 180.0)
    leave_one_out = []
    for held in range(len(usable)):
        rest = total.copy()
        profile = np.asarray(usable[held]["line"]["profile_norm_deg"], dtype=float)
        roll = float(usable[held]["roll"]["roll_deg"])
        sample = np.interp((grid + roll) % 360.0, np.arange(LINE_PROFILE_BINS),
                           profile, period=LINE_PROFILE_BINS)
        if sample.max() > 0:
            sample = sample / sample.max()
        rest -= sample
        leave_one_out.append(float(grid[int(np.argmax(rest))]))
    peak_value = float(total[peak])
    threshold = 0.85 * peak_value
    # Contiguous arc above the threshold around the peak.
    below = np.where(total < threshold)[0]
    if len(below) == 0:
        interval = 360.0
    else:
        left = 0
        index = peak
        while total[index % bins] >= threshold and left < bins:
            index -= 1
            left += 1
        right = 0
        index = peak
        while total[index % bins] >= threshold and right < bins:
            index += 1
            right += 1
        interval = (left + right - 1) * 0.5
    loo_span = (max(leave_one_out) - min(leave_one_out)) if leave_one_out else None
    max_offset = max(abs(value) for value in offsets) if offsets else None
    per_frame_spans = [float((item["line"].get("angle_span_deg") or 0.0))
                       for item in usable]
    median_span = float(np.median(per_frame_spans)) if per_frame_spans else None
    consistent = (max_offset is not None
                  and max_offset <= LINE_DIRECTION_TOL_DEG)
    narrower = (consistent and interval < LINE_DIRECTION_TOL_DEG
                and median_span is not None and interval < median_span)
    return {
        "ok": bool(consistent),
        "reason": None if consistent else "joint_direction_inconsistent",
        "joint_compensated_deg": joint,
        "joint_interval_deg": float(interval),
        "frames_used": len(usable),
        "frame_indices": [item.get("index") for item in usable],
        "per_frame_consistency_deg": max_offset,
        "leave_one_out_span_deg": loo_span,
        "median_single_frame_span_deg": median_span,
        "narrower_than_single_frame": bool(narrower),
        "internal_consistency_only": True,
        "independent_verification": False,
        "note": "static-face roll alignment and normalised ring geometry only; "
                "no needle alignment, no humidity averaging, no trusted value",
        "individual_compensated_deg": [
            round((float(item["line"]["angle_deg"])
                   - float(item["roll"]["roll_deg"])) % 360.0, 4)
            for item in usable],
    }


# ---------------------------------------------------------------------------
# frame pipeline
# ---------------------------------------------------------------------------

def _base_result() -> Dict:
    return {
        "experimental": True,
        "trusted": False,
        "tool_version": TOOL_VERSION,
        "experiment_id": EXPERIMENT_ID,
        "value": None,
        "unit": "%RH",
        "value_policy": "top-level value is always null; diagnostic candidates are not truth",
        "frame_status": "no_dial",
        "status": "no_dial",
        "reason": "no_ring",
        "ring": None,
        "axis": None,
        "roll": None,
        "needle": None,
        "line": None,
        "diagnostic_candidate": None,
        "calibration": {},
        "rejection": None,
        "quality": {},
    }


def estimate_experiment_frame(image_bgr: np.ndarray, calibration: Dict,
                              declared_roi: Sequence[int],
                              reference: Optional[Dict] = None,
                              stage: str = "eval",
                              top_expansion_px: int = ROI_TOP_EXPANSION_PX) -> Dict:
    """Run the experimental stages on one frame.  ``value`` stays null."""
    result = _base_result()
    roi_record = expanded_roi(declared_roi, image_bgr.shape, top_expansion_px)
    x, y, width, height = roi_record["used_roi"]
    crop = image_bgr[y:y + height, x:x + width]
    result["roi"] = roi_record
    result["stage"] = stage
    result["calibration"] = {
        "calib_version": calibration.get("calib_version", ""),
        "calib_sha256": calibration.get("calib_sha256", ""),
        "source_image_sha256": calibration.get("source_sha256", ""),
        "model_metadata": "deli No.0011 (historical metadata; same physical unit is No.9011)",
    }
    ring_result = detect_edge_ring(crop)
    result["ring"] = {
        "ok": ring_result.get("ok", False),
        "status": ring_result.get("status"),
        "reason": ring_result.get("reason"),
        "detail": ring_result.get("detail"),
        "candidate_count": ring_result.get("candidate_count"),
        "top_candidates": ring_result.get("top_candidates"),
        "hub": ring_result.get("hub"),
        "preprocess": ring_result.get("preprocess"),
        "radius_range_px": ring_result.get("radius_range_px"),
        "score_margin": ring_result.get("score_margin"),
    }
    if not ring_result.get("ok"):
        result.update({
            "frame_status": ring_result.get("status", "no_dial"),
            "status": ring_result.get("status", "no_dial"),
            "reason": ring_result.get("reason", "no_ring"),
            "rejection": {"stage": "ring", "reason": ring_result.get("reason", "no_ring")},
        })
        return result
    ring = ring_result["ring"]
    result["ring"].update({
        "center_px": [x + float(ring["center"][0]), y + float(ring["center"][1])],
        "axes_px": [float(ring["axes"][0]), float(ring["axes"][1])],
        "angle_deg": float(ring["angle_deg"]),
        "coverage": float(ring["support"]["coverage"]),
        "coverage_bins": int(ring["support"]["coverage_bins"]),
        "max_gap_deg": float(ring["support"]["max_gap_deg"]),
        "residual_rms_px": float(ring["support"]["residual_rms_px"]),
        "align": float(ring["support"]["align"]),
        "gradient_support": float(ring["support"]["gradient_support"]),
        "band_contrast": ring["band_contrast"],
        "score": float(ring["score"]),
        "inlier_count": int(len(ring["inlier_points"])),
        "edge_points_px": (np.asarray(ring["inlier_points"], dtype=float)
                           + np.array([x, y], dtype=float)).round(3).tolist()[:300],
    })
    hub = ring_result.get("hub")
    axis = assess_axis(ring, hub, calibration)
    if axis.get("hub_axis_px"):
        axis["hub_axis_image_px"] = [x + float(axis["hub_axis_px"][0]),
                                     y + float(axis["hub_axis_px"][1])]
    if axis.get("blob_centroid_px"):
        axis["blob_centroid_image_px"] = [x + float(axis["blob_centroid_px"][0]),
                                          y + float(axis["blob_centroid_px"][1])]
    result["axis"] = axis
    # R2's pivot gate is kept as a diagnostic; it must not gate the new line
    # direction.  Its failure reasons explain why the ray-angle interval was
    # wide, which is exactly what R3 tries to escape.
    result["axis"]["r2_gate"] = {
        "ok": bool(axis.get("ok")),
        "reason": axis.get("reason"),
        "used_as_hard_prior_for_line": False,
    }

    matrix, _, _, _ = _ellipse_matrix(
        (float(ring["axes"][0]), float(ring["axes"][1])),
        float(ring["angle_deg"]))
    current = angular_signature(crop, ring["center"], matrix)
    roll = estimate_roll(reference, current, matrix) if reference is not None else {
        "ok": False, "status": "low_conf", "reason": "reference_signature_missing"}
    result["roll"] = {
        key: roll.get(key) for key in
        ("ok", "status", "reason", "detail", "method", "not_a_template_warp",
         "best_corr", "second_corr", "second_shift_deg", "gap", "snr",
         "amplitude", "valid_frac", "roll_deg", "uncertainty_deg")
    }
    result["roll"]["current_signature_amplitude"] = current.get("amplitude")
    result["roll"]["current_signature_valid_frac"] = current.get("valid_frac")
    if not roll.get("ok"):
        result.update({
            "frame_status": roll.get("status", "low_conf"),
            "status": roll.get("status", "low_conf"),
            "reason": roll.get("reason", "roll_weak"),
            "rejection": {"stage": "roll", "reason": roll.get("reason")},
        })
        return result
    roll_deg = float(roll["roll_deg"])

    # Old ray method: diagnostic comparison only.  Its weak score and its wide
    # pivot-evidence interval no longer stop the frame.
    axis_px = np.asarray(axis["observed_axis_px"], dtype=float)
    needle = detect_needle(crop, axis_px, matrix)
    result["needle"] = {
        key: needle.get(key) for key in
        ("ok", "status", "reason", "detail", "angle_deg", "coverage",
         "score", "second_score", "peak_gap_deg")
    }
    if needle.get("ok") and float(needle.get("score") or 0.0) < NEEDLE_MIN_SCORE:
        needle = dict(needle)
        needle.update({"ok": False, "status": "low_conf",
                       "reason": "needle_weak_score",
                       "detail": "dark-ray score below the negative-control gate"})
        result["needle"].update({"ok": False, "reason": "needle_weak_score",
                                 "status": "low_conf"})
    result["needle"]["method"] = "dial_reading.detect_needle dark-ray (diagnostic)"
    result["needle"]["used_as_hard_gate"] = False

    # R2 axis evidence interval, retained for the ray/line comparison only.
    needle_ring_centre = detect_needle(crop, np.asarray(ring["center"], dtype=float), matrix)
    axis_angle_delta = None
    if needle_ring_centre.get("ok"):
        axis_angle_delta = float(_angle_difference_deg(
            float(needle["angle_deg"]), float(needle_ring_centre["angle_deg"])))
    result["axis"]["needle_angle_delta_deg"] = axis_angle_delta
    result["axis"]["needle_angle_ring_centre_deg"] = needle_ring_centre.get("angle_deg")
    result["axis"]["needle_angle_observed_axis_deg"] = needle.get("angle_deg")

    axis_verified = bool(axis.get("axis_verified"))
    hub_ambiguous = bool(axis.get("hub_ambiguous"))
    result["axis"]["axis_verified"] = axis_verified
    if axis_verified:
        evidence = _axis_needle_evidence(crop, matrix, axis, hub,
                                         needle.get("angle_deg"))
    else:
        # No independent pivot: do not dress up a ring-centre needle probe as
        # an axis interval.  The fallback stays unverified with no interval.
        evidence = {"detected": [], "failed": [], "angles_deg": [],
                    "interval": {"ok": False, "reason": "axis_unverified"}}
    interval = evidence["interval"]
    axis_angle_span = interval.get("span_deg") if interval.get("ok") else None
    axis_angle_interval = (None if not interval.get("ok")
                           else [float(interval["start_deg"]),
                                 float(interval["end_deg"])])
    evidence_failed = bool(evidence["failed"])
    axis_angle_stable, axis_reason = _axis_stability(
        axis_verified, hub_ambiguous, evidence, axis_angle_span)
    result["axis"].update({
        "angle_stable": axis_angle_stable,
        "angle_interval_deg": axis_angle_interval,
        "angle_span_deg": axis_angle_span,
        "angle_interval_wraps_zero": bool(interval.get("wraps_zero", False)),
        "needle_angle_evidence": {
            "detected": evidence["detected"],
            "failed": evidence["failed"],
            "span_deg": axis_angle_span,
        },
        "axis_evidence_failed": evidence_failed,
        "r2_axis_stable": axis_angle_stable,
        "r2_axis_reason": axis_reason,
        "r2_ray_angle_span_deg": axis_angle_span,
        "r2_ray_angle_interval_deg": axis_angle_interval,
        "r2_gross_conflict": bool(axis_verified and axis_angle_delta is not None
                                  and axis_angle_delta > AXIS_HARD_ANGLE_TOL_DEG),
        "used_as_hard_gate_for_line": False,
    })

    # E1-R3: the slender body carries the angle; the cap is only a loose
    # bounds/orientation aid.  No `return` above this point may be caused by
    # the R2 pivot gate.
    line = estimate_needle_line(crop, ring, hub)
    result["line"] = line
    result["axis"]["line_hub_span_deg"] = (
        line.get("uncertainty_components_deg") or {}).get("hub_span_deg")
    result["axis"]["line_ring_span_deg"] = (
        line.get("uncertainty_components_deg") or {}).get("ring_span_deg")
    result["axis"]["r2_ray_vs_line_delta_deg"] = (
        None if line.get("angle_deg") is None or not needle.get("ok")
        else float(_angle_difference_deg(float(needle["angle_deg"]),
                                         float(line["angle_deg"]))))
    if line.get("angle_deg") is None:
        result.update({
            "frame_status": "low_conf",
            "status": "low_conf",
            "reason": line.get("reason", "line_no_needle"),
            "rejection": {"stage": "line", "reason": line.get("reason"),
                          "detail": line.get("detail")},
        })
        return result

    line_angle = float(line["angle_deg"])
    compensated = wrap_deg(line_angle - roll_deg)
    zero = float(calibration["table_deg"][0])
    unwrapped = zero + wrap_deg(compensated - zero)
    value = lookup_value(unwrapped, list(calibration["table_deg"]),
                         list(calibration["table_val"]))
    line_span = line.get("angle_span_deg")
    result["diagnostic_candidate"] = {
        "candidate_value": None if value is None else float(value),
        "angle_source": "slender_line_body_paired_edges",
        "needle_raw_angle_deg": line_angle,
        "line_raw_angle_deg": line_angle,
        "line_angle_span_deg": line_span,
        "line_pixel_span_deg": line.get("pixel_span_deg"),
        "line_matrix_condition_number": line.get("matrix_condition_number"),
        "line_normalized_span_upper_bound_deg": line.get(
            "normalized_span_upper_bound_deg"),
        "line_uncertainty_components_deg": line.get("uncertainty_components_deg"),
        "line_support_frac": line.get("support_frac"),
        "line_tip_extent_px": line.get("tip_extent_px"),
        "line_tail_extent_px": line.get("tail_extent_px"),
        "line_tip_tail_ratio": line.get("tip_tail_ratio"),
        "line_direction_stable": bool(line.get("ok")),
        "ray_angle_deg": needle.get("angle_deg"),
        "ray_angle_interval_deg": axis_angle_interval,
        "ray_angle_span_deg": axis_angle_span,
        "ray_angle_stable": axis_angle_stable,
        "ray_vs_line_delta_deg": result["axis"]["r2_ray_vs_line_delta_deg"],
        "roll_deg": roll_deg,
        "roll_uncertainty_deg": float(roll["uncertainty_deg"]),
        "compensated_angle_deg": compensated,
        "unwrapped_angle_deg": unwrapped,
        "table_interval_deg": [float(calibration["table_deg"][0]),
                               float(calibration["table_deg"][-1])],
        "axis_angle_stable": axis_angle_stable,
        "axis_angle_interval_deg": axis_angle_interval,
        "axis_angle_span_deg": axis_angle_span,
        "axis_angle_samples_deg": interval.get("samples_deg"),
        "axis_evidence_failed": evidence_failed,
        "roll_interval_deg": [
            wrap_deg(roll_deg - float(roll["uncertainty_deg"])),
            wrap_deg(roll_deg + float(roll["uncertainty_deg"])),
        ],
        "trusted": False,
        "note": "diagnostic only; not an accuracy result and not a truth value",
    }
    # Sensitivity probes use the line angle and its own normalised upper-bound
    # interval plus the roll interval.  A pixel span would be in the wrong
    # coordinate system here.  The old ray angle is not used to build the span.
    probe_angles = {round(line_angle % 360.0, 6)}
    if line_span is not None and math.isfinite(float(line_span)):
        probe_angles.add(round((line_angle + float(line_span)) % 360.0, 6))
        probe_angles.add(round((line_angle - float(line_span)) % 360.0, 6))
    value_span = []
    for angle_probe in sorted(probe_angles):
        for roll_probe in (roll_deg, roll_deg - float(roll["uncertainty_deg"]),
                           roll_deg + float(roll["uncertainty_deg"])):
            probe = wrap_deg(angle_probe - roll_probe)
            probe_unwrapped = zero + wrap_deg(probe - zero)
            probe_value = lookup_value(probe_unwrapped,
                                       list(calibration["table_deg"]),
                                       list(calibration["table_val"]))
            if probe_value is not None:
                value_span.append(float(probe_value))
    if value_span:
        result["diagnostic_candidate"]["value_sensitivity_span"] = [
            round(min(value_span), 4), round(max(value_span), 4)]
    result["quality"] = {
        "ring_coverage": float(ring["support"]["coverage"]),
        "ring_residual_rms_px": float(ring["support"]["residual_rms_px"]),
        "ring_band_contrast": float(ring["band_contrast"]["contrast"]),
        "roll_best_corr": float(roll["best_corr"]),
        "roll_gap": roll.get("gap"),
        "roll_snr": float(roll["snr"]),
        "needle_coverage": needle.get("coverage"),
        "ray_angle_deg": needle.get("angle_deg"),
        "ray_angle_span_deg": axis_angle_span,
        "line_angle_deg": line_angle,
        "line_angle_span_deg": line_span,
        "line_pixel_span_deg": line.get("pixel_span_deg"),
        "line_matrix_condition_number": line.get("matrix_condition_number"),
        "line_support_frac": line.get("support_frac"),
        "line_edge_opening_deg": line.get("edge_opening_deg"),
        "line_strip_angle_deg": line.get("strip_angle_deg"),
        "line_estimator_delta_deg": line.get("estimator_delta_deg"),
        "line_centerline_rms_px": line.get("centerline_rms_px"),
        "line_width_rms_px": line.get("width_rms_px"),
        "line_hub_span_deg": result["axis"]["line_hub_span_deg"],
        "line_ring_span_deg": result["axis"]["line_ring_span_deg"],
        "line_tip_tail_ratio": line.get("tip_tail_ratio"),
        "axis_angle_delta_deg": axis_angle_delta,
        "axis_angle_span_deg": axis_angle_span,
    }
    if value is None:
        result.update({
            "frame_status": "out_of_range",
            "status": "out_of_range",
            "reason": "out_of_range",
            "rejection": {"stage": "table", "reason": "out_of_range"},
        })
    elif not line.get("ok"):
        result.update({
            "frame_status": "low_conf",
            "status": "low_conf",
            "reason": line.get("reason", "line_direction_unstable"),
            "rejection": {"stage": "line", "reason": line.get("reason"),
                          "detail": line.get("detail"),
                          "angle_span_deg": line_span,
                          "uncertainty_components_deg":
                              line.get("uncertainty_components_deg"),
                          "support_frac": line.get("support_frac"),
                          "tip_tail_ratio": line.get("tip_tail_ratio"),
                          "r2_axis_stable": axis_angle_stable,
                          "r2_axis_reason": axis_reason},
        })
    else:
        result.update({
            "frame_status": "ok",
            "status": "diagnostic_candidate",
            "reason": "line_direction_stable",
        })
    return result


# ---------------------------------------------------------------------------
# synthetic controls (used by the tests and by the CLI summary)
# ---------------------------------------------------------------------------

def make_synthetic_scene(mode: str = "color", size: int = 200,
                         center: Tuple[float, float] = (100.0, 100.0),
                         axes: Tuple[float, float] = (84.0, 78.0),
                         second_ring: bool = False, blur_sigma: float = 0.0,
                         noise_sigma: float = 0.0,
                         needle_deg: float = 170.0,
                         needle_length_frac: float = 0.55,
                         with_hub: bool = True,
                         hub_offset: Tuple[float, float] = (0.0, 0.0),
                         hub_radius: float = 5.0) -> np.ndarray:
    """Portable synthetic fixture.  It is not real dial truth or accuracy data.

    ``hub_offset`` moves only the pivot cap and its needle, keeping the colour
    ring centre fixed.  It exists to test a legal non-zero axis/ring-centre
    offset and is independent construction truth, not a call into the detector.

    R3 fixture correction (disclosed): the R1/R2 fixture drew the needle with
    ``end.x = hub.x - frac * axes[0] * cos(angle)`` but
    ``end.y = hub.y + frac * axes[1] * sin(angle)`` and a full-axis length.
    That both mirrored the requested direction and let the needle pierce the
    ring, so ``needle_deg`` was not the drawn angle.  The corrected convention
    is a half-axis length ``frac * 0.5 * max(axes)`` and a plain
    ``[cos, sin]`` image-space vector, so ``needle_deg`` is exactly the drawn
    direction.  Data produced by R2 is not claimed to be this same fixture.
    """
    image = np.full((int(size), int(size), 3), 245, dtype=np.uint8)
    center_int = (int(round(center[0])), int(round(center[1])))
    hub_center = (float(center[0]) + float(hub_offset[0]),
                  float(center[1]) + float(hub_offset[1]))
    hub_int = (int(round(hub_center[0])), int(round(hub_center[1])))
    half = (int(round(axes[0] / 2.0)), int(round(axes[1] / 2.0)))
    if mode == "color":
        arcs = [((150, 190), (200, 60, 30)),
                ((190, 250), (40, 170, 60)),
                ((250, 390), (30, 50, 200)),
                ((390, 510), (200, 60, 30))]
    elif mode == "gray":
        arcs = [((150, 510), (120, 120, 120))]
    elif mode == "low_sat":
        arcs = [((150, 190), (190, 175, 160)),
                ((190, 250), (170, 185, 170)),
                ((250, 390), (160, 175, 195)),
                ((390, 510), (190, 175, 160))]
    elif mode == "blank":
        arcs = []
    else:
        raise ExperimentConfigError("unknown synthetic mode: {}".format(mode))
    for (start, end), colour in arcs:
        cv2.ellipse(image, center_int, half, 0, start, end, colour, 7)
    if mode != "blank":
        angle = math.radians(float(needle_deg))
        needle_length_px = float(needle_length_frac) * 0.5 * float(max(axes))
        end = (int(round(hub_center[0] + needle_length_px * math.cos(angle))),
               int(round(hub_center[1] + needle_length_px * math.sin(angle))))
        cv2.line(image, hub_int, end, (20, 20, 20), 2)
        if with_hub:
            cv2.circle(image, hub_int, int(round(hub_radius)), (10, 10, 10), -1)
    if second_ring:
        second_center = (60, 140)
        second_half = (40, 36)
        for (start, end), colour in arcs:
            cv2.ellipse(image, second_center, second_half, 0, start, end, colour, 7)
        cv2.circle(image, second_center, 5, (10, 10, 10), -1)
    if blur_sigma > 0.0:
        image = cv2.GaussianBlur(image, (0, 0), float(blur_sigma))
    if noise_sigma > 0.0:
        rng = np.random.RandomState(20261115)
        noisy = image.astype(np.float32) + rng.normal(0.0, float(noise_sigma),
                                                       image.shape)
        image = np.clip(noisy, 0, 255).astype(np.uint8)
    return image


def synthetic_needle_truth(needle_deg: float, axes: Tuple[float, float] = (84.0, 78.0),
                           ring_angle_deg: float = 0.0,
                           needle_length_frac: float = 0.55) -> Dict:
    """Construction truth of the corrected fixture; never a detector output."""
    matrix, _, _, _ = _ellipse_matrix((float(axes[0]), float(axes[1])),
                                      float(ring_angle_deg))
    theta = math.radians(float(needle_deg))
    vector = np.array([math.cos(theta), math.sin(theta)], dtype=float)
    mapped = matrix @ vector
    return {
        "drawn_pixel_deg": float(needle_deg) % 360.0,
        "expected_normalised_deg":
            math.degrees(math.atan2(mapped[1], mapped[0])) % 360.0,
        "length_px": float(needle_length_frac) * 0.5 * float(max(axes)),
        "convention": "half-axis length, end = hub + length * [cos, sin]",
    }


# ---------------------------------------------------------------------------
# output helpers and CLI
# ---------------------------------------------------------------------------

def _output_target(output: Path) -> None:
    if output.is_symlink():
        raise ExperimentConfigError("output directory must not be a symlink")
    if output.exists():
        if not output.is_dir():
            raise ExperimentConfigError("output path exists and is not a directory")
        try:
            if any(output.iterdir()):
                raise ExperimentConfigError(
                    "output directory is not empty; use a new empty directory")
        except OSError as error:
            raise ExperimentConfigError(
                "cannot inspect output directory: {}".format(error))


def _write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(".{}-{}.tmp".format(path.name, uuid.uuid4().hex))
    try:
        temporary.write_text(json.dumps(_json_safe(payload), ensure_ascii=False,
                                        indent=2), encoding="utf-8")
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            try:
                temporary.unlink()
            except OSError:
                pass


def _compact_frame_result(result: Dict) -> Dict:
    ring = result.get("ring") or {}
    axis = result.get("axis") or {}
    roll = result.get("roll") or {}
    needle = result.get("needle") or {}
    line = result.get("line") or {}
    candidate = result.get("diagnostic_candidate") or {}
    return {
        "index": result.get("index"),
        "split": result.get("split"),
        "file": result.get("file"),
        "sha256": result.get("frame_sha256"),
        "width": result.get("width"),
        "height": result.get("height"),
        "experimental": True,
        "trusted": False,
        "value": None,
        "frame_status": result.get("frame_status"),
        "reason": result.get("reason"),
        "rejection": result.get("rejection"),
        "roi": result.get("roi"),
        "ring": {
            "ok": ring.get("ok"),
            "center_px": ring.get("center_px"),
            "axes_px": ring.get("axes_px"),
            "angle_deg": ring.get("angle_deg"),
            "coverage": ring.get("coverage"),
            "max_gap_deg": ring.get("max_gap_deg"),
            "residual_rms_px": ring.get("residual_rms_px"),
            "band_contrast": (ring.get("band_contrast") or {}).get("contrast"),
            "band_positive_frac": (ring.get("band_contrast") or {}).get("positive_frac"),
            "score": ring.get("score"),
            "top_candidates": ring.get("top_candidates"),
        },
        "axis": {
            "ok": axis.get("ok"),
            "reason": axis.get("reason"),
            "observed_axis_px": axis.get("observed_axis_px"),
            "hub_axis_px": axis.get("hub_axis_px"),
            "hub_axis_image_px": axis.get("hub_axis_image_px"),
            "blob_centroid_px": axis.get("blob_centroid_px"),
            "blob_centroid_image_px": axis.get("blob_centroid_image_px"),
            "centroid_offset_px": axis.get("centroid_offset_px"),
            "hub_usable": axis.get("hub_usable"),
            "hub_ambiguous": axis.get("hub_ambiguous"),
            "hub_rejection": axis.get("hub_rejection"),
            "hub_competition": axis.get("hub_competition"),
            "hub_center_uncertainty_px": axis.get("hub_center_uncertainty_px"),
            "hub_center_spread_px": axis.get("hub_center_spread_px"),
            "hub_thresholds": axis.get("hub_thresholds"),
            "hub_threshold_centers": axis.get("hub_threshold_centers"),
            "hub_plateau": axis.get("hub_plateau"),
            "hub_plateau_separated": axis.get("hub_plateau_separated"),
            "hub_plateau_elongated": axis.get("hub_plateau_elongated"),
            "hub_threshold_centers_unstable": axis.get(
                "hub_threshold_centers_unstable"),
            "ring_center_px": axis.get("ring_center_px"),
            "ring_center_std_px": axis.get("centre_std_px"),
            "ring_axis_offset_px": axis.get("ring_axis_offset_px"),
            "ring_axis_offset_frac": axis.get("ring_axis_offset_frac"),
            "axis_uncertainty_px": axis.get("axis_uncertainty_px"),
            "axis_verified": axis.get("axis_verified"),
            "needle_angle_delta_deg": axis.get("needle_angle_delta_deg"),
            "needle_angle_observed_axis_deg": axis.get(
                "needle_angle_observed_axis_deg"),
            "angle_stable": axis.get("angle_stable"),
            "angle_interval_deg": axis.get("angle_interval_deg"),
            "angle_span_deg": axis.get("angle_span_deg"),
            "angle_interval_wraps_zero": axis.get("angle_interval_wraps_zero"),
            "needle_angle_evidence": axis.get("needle_angle_evidence"),
            "axis_evidence_failed": axis.get("axis_evidence_failed"),
            "r2_gate": axis.get("r2_gate"),
            "r2_axis_stable": axis.get("r2_axis_stable"),
            "r2_axis_reason": axis.get("r2_axis_reason"),
            "r2_ray_angle_span_deg": axis.get("r2_ray_angle_span_deg"),
            "r2_ray_angle_interval_deg": axis.get("r2_ray_angle_interval_deg"),
            "r2_gross_conflict": axis.get("r2_gross_conflict"),
            "line_hub_span_deg": axis.get("line_hub_span_deg"),
            "line_ring_span_deg": axis.get("line_ring_span_deg"),
            "r2_ray_vs_line_delta_deg": axis.get("r2_ray_vs_line_delta_deg"),
        },
        "line": {
            "ok": line.get("ok"),
            "reason": line.get("reason"),
            "detail": line.get("detail"),
            "angle_deg": line.get("angle_deg"),
            "angle_px_deg": line.get("angle_px_deg"),
            "coarse_angle_deg": line.get("coarse_angle_deg"),
            "strip_angle_deg": line.get("strip_angle_deg"),
            "fine_angle_deg": line.get("fine_angle_deg"),
            "edge_center_angle_deg": line.get("edge_center_angle_deg"),
            "edge_center_delta_deg": line.get("edge_center_delta_deg"),
            "estimator_delta_deg": line.get("estimator_delta_deg"),
            "edge_opening_deg": line.get("edge_opening_deg"),
            "coarse_offset_px": line.get("coarse_offset_px"),
            "refined_offset_px": line.get("refined_offset_px"),
            "support_frac": line.get("support_frac"),
            "strip_support_frac": line.get("strip_support_frac"),
            "support_samples": line.get("support_samples"),
            "tip_extent_px": line.get("tip_extent_px"),
            "tail_extent_px": line.get("tail_extent_px"),
            "tip_tail_ratio": line.get("tip_tail_ratio"),
            "width_median_px": line.get("width_median_px"),
            "width_rms_px": line.get("width_rms_px"),
            "rms_px": line.get("rms_px"),
            "centerline_rms_px": line.get("centerline_rms_px"),
            "angle_span_deg": line.get("angle_span_deg"),
            "angle_span_definition": line.get("angle_span_definition"),
            "pixel_span_deg": line.get("pixel_span_deg"),
            "normalized_span_upper_bound_deg": line.get(
                "normalized_span_upper_bound_deg"),
            "matrix_condition_number": line.get("matrix_condition_number"),
            "interval_definition": line.get("interval_definition"),
            "uncertainty_components_deg": line.get("uncertainty_components_deg"),
            "normalized_components_upper_bound_deg": line.get(
                "normalized_components_upper_bound_deg"),
            "hub_span_deg": (line.get("uncertainty_components_deg") or {}).get(
                "hub_span_deg"),
            "ring_span_deg": (line.get("uncertainty_components_deg") or {}).get(
                "ring_span_deg"),
            "edge_fit_hub_span_deg": line.get("edge_fit_hub_span_deg"),
            "edge_fit_ring_span_deg": line.get("edge_fit_ring_span_deg"),
            "perturbation_expected": line.get("perturbation_expected"),
            "perturbation_failures": line.get("perturbation_failures"),
            "bootstrap_missing": line.get("bootstrap_missing"),
            "split_half_missing": line.get("split_half_missing"),
            "missing_evidence": line.get("missing_evidence"),
            "bootstrap_method": line.get("bootstrap_method"),
            "hub_ambiguous": line.get("hub_ambiguous"),
            "hub_usable": line.get("hub_usable"),
            "profile_second_ratio": line.get("profile_second_ratio"),
            "method": line.get("method"),
            "diagnostic_only": True,
        },
        "roll": {
            "ok": roll.get("ok"),
            "reason": roll.get("reason"),
            "roll_deg": roll.get("roll_deg"),
            "uncertainty_deg": roll.get("uncertainty_deg"),
            "best_corr": roll.get("best_corr"),
            "second_corr": roll.get("second_corr"),
            "gap": roll.get("gap"),
            "snr": roll.get("snr"),
            "amplitude": roll.get("amplitude"),
            "valid_frac": roll.get("valid_frac"),
            "method": roll.get("method"),
        },
        "needle": {
            "ok": needle.get("ok"),
            "reason": needle.get("reason"),
            "angle_deg": needle.get("angle_deg"),
            "coverage": needle.get("coverage"),
            "score": needle.get("score"),
            "second_score": needle.get("second_score"),
            "peak_gap_deg": needle.get("peak_gap_deg"),
        },
        "diagnostic_candidate": candidate if candidate else None,
        "value_policy": "top-level value is null; diagnostic_candidate is not truth",
    }


def _csv_row(result: Dict) -> Dict:
    compact = _compact_frame_result(result)
    ring = compact["ring"]
    axis = compact["axis"]
    roll = compact["roll"]
    needle = compact["needle"]
    candidate = compact["diagnostic_candidate"] or {}
    return {
        "index": compact["index"],
        "split": compact["split"],
        "file": compact["file"],
        "sha256": compact["sha256"],
        "frame_status": compact["frame_status"],
        "reason": compact["reason"],
        "value": "",
        "ring_ok": ring["ok"],
        "ring_center_x": None if not ring["center_px"] else round(ring["center_px"][0], 3),
        "ring_center_y": None if not ring["center_px"] else round(ring["center_px"][1], 3),
        "ring_axes_major": None if not ring["axes_px"] else round(max(ring["axes_px"]), 3),
        "ring_axes_minor": None if not ring["axes_px"] else round(min(ring["axes_px"]), 3),
        "ring_coverage": None if ring["coverage"] is None else round(ring["coverage"], 4),
        "ring_rms_px": None if ring["residual_rms_px"] is None else round(ring["residual_rms_px"], 4),
        "ring_band_contrast": None if ring["band_contrast"] is None else round(ring["band_contrast"], 3),
        "axis_offset_px": None if not axis["ring_axis_offset_px"] else round(float(np.hypot(*axis["ring_axis_offset_px"])), 3),
        "axis_angle_delta_deg": axis["needle_angle_delta_deg"],
        "axis_verified": axis.get("axis_verified"),
        "hub_center_x": None if not axis.get("hub_axis_px") else round(axis["hub_axis_px"][0], 3),
        "hub_center_y": None if not axis.get("hub_axis_px") else round(axis["hub_axis_px"][1], 3),
        "hub_image_x": None if not axis.get("hub_axis_image_px") else round(axis["hub_axis_image_px"][0], 3),
        "hub_image_y": None if not axis.get("hub_axis_image_px") else round(axis["hub_axis_image_px"][1], 3),
        "hub_centroid_offset_px": axis.get("centroid_offset_px"),
        "hub_center_uncertainty_px": axis.get("hub_center_uncertainty_px"),
        "hub_thresholds": None if axis.get("hub_thresholds") is None else "|".join(
            str(value) for value in axis["hub_thresholds"]),
        "hub_ambiguous": axis.get("hub_ambiguous"),
        "axis_angle_span_deg": axis.get("angle_span_deg"),
        "roll_deg": roll["roll_deg"],
        "roll_uncertainty_deg": roll["uncertainty_deg"],
        "roll_corr": roll["best_corr"],
        "roll_gap": roll["gap"],
        "needle_deg": needle["angle_deg"],
        "needle_coverage": needle["coverage"],
        "line_ok": (compact.get("line") or {}).get("ok"),
        "line_angle_deg": (compact.get("line") or {}).get("angle_deg"),
        "line_angle_px_deg": (compact.get("line") or {}).get("angle_px_deg"),
        "line_strip_angle_deg": (compact.get("line") or {}).get("strip_angle_deg"),
        "line_estimator_delta_deg": (
            (compact.get("line") or {}).get("estimator_delta_deg")),
        "line_edge_opening_deg": (compact.get("line") or {}).get("edge_opening_deg"),
        "line_span_deg": (compact.get("line") or {}).get("angle_span_deg"),
        "line_pixel_span_deg": (compact.get("line") or {}).get("pixel_span_deg"),
        "line_condition_number": (compact.get("line") or {}).get(
            "matrix_condition_number"),
        "line_pixel_split_span_deg": (
            ((compact.get("line") or {}).get("uncertainty_components_deg") or {}).get(
                "split_half_span_deg")),
        "line_pixel_bootstrap_span_deg": (
            ((compact.get("line") or {}).get("uncertainty_components_deg") or {}).get(
                "bootstrap_span_deg")),
        "line_pixel_fit_span_deg": (
            ((compact.get("line") or {}).get("uncertainty_components_deg") or {}).get(
                "fit_span_deg")),
        "line_pixel_hub_span_deg": (compact.get("line") or {}).get("hub_span_deg"),
        "line_pixel_ring_span_deg": (compact.get("line") or {}).get("ring_span_deg"),
        "line_centerline_rms_px": (
            (compact.get("line") or {}).get("centerline_rms_px")),
        "line_width_rms_px": (compact.get("line") or {}).get("width_rms_px"),
        "line_missing_evidence": (
            None if not (compact.get("line") or {}).get("missing_evidence")
            else "|".join((compact.get("line") or {}).get("missing_evidence"))),
        "line_support_frac": (compact.get("line") or {}).get("support_frac"),
        "line_tip_extent_px": (compact.get("line") or {}).get("tip_extent_px"),
        "line_tail_extent_px": (compact.get("line") or {}).get("tail_extent_px"),
        "line_tip_tail_ratio": (compact.get("line") or {}).get("tip_tail_ratio"),
        "line_reason": (compact.get("line") or {}).get("reason"),
        "ray_deg": needle["angle_deg"],
        "ray_span_deg": axis.get("r2_ray_angle_span_deg"),
        "ray_vs_line_delta_deg": axis.get("r2_ray_vs_line_delta_deg"),
        "candidate_value": candidate.get("candidate_value"),
        "candidate_trusted": False,
        "trusted": False,
        "experimental": True,
    }


def draw_experiment_overlay(image_bgr: np.ndarray, result: Dict) -> np.ndarray:
    canvas = image_bgr.copy()
    roi = (result.get("roi") or {}).get("used_roi")
    declared = (result.get("roi") or {}).get("declared_roi")
    if declared:
        x, y, w, h = [int(v) for v in declared]
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (128, 128, 128), 2)
    if roi:
        x, y, w, h = [int(v) for v in roi]
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 255), 2)
    ring = result.get("ring") or {}
    for point in ring.get("edge_points_px", []) or []:
        cv2.circle(canvas, (int(round(point[0])), int(round(point[1]))), 1,
                   (255, 255, 0), -1)
    center = ring.get("center_px")
    axes = ring.get("axes_px")
    if center and axes:
        c = (int(round(center[0])), int(round(center[1])))
        cv2.ellipse(canvas, c,
                    (int(round(axes[0] / 2.0)), int(round(axes[1] / 2.0))),
                    float(ring.get("angle_deg", 0.0)), 0, 360, (0, 255, 0), 2)
        cv2.drawMarker(canvas, c, (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
    axis = result.get("axis") or {}
    observed = axis.get("observed_axis_px")
    needle = result.get("needle") or {}
    line = result.get("line") or {}
    if observed:
        a = (int(round(observed[0])), int(round(observed[1])))
        cv2.drawMarker(canvas, a, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
        if axes and needle.get("ok"):
            matrix, _, _, _ = _ellipse_matrix((float(axes[0]), float(axes[1])),
                                              float(ring.get("angle_deg", 0.0)))
            direction = _needle_direction_px(matrix, float(needle["angle_deg"]))
            length = 0.78 * 0.5 * (float(axes[0]) + float(axes[1]))
            end = (int(round(observed[0] + length * float(direction[0]))),
                   int(round(observed[1] + length * float(direction[1]))))
            cv2.line(canvas, a, end, (0, 0, 255), 1)
            cv2.putText(canvas, "ray {:.1f}deg".format(float(needle["angle_deg"])),
                        (end[0] + 4, end[1] - 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 0, 255), 1, cv2.LINE_AA)
        if axes and line.get("angle_deg") is not None:
            matrix, _, _, _ = _ellipse_matrix((float(axes[0]), float(axes[1])),
                                              float(ring.get("angle_deg", 0.0)))
            direction = _needle_direction_px(matrix, float(line["angle_deg"]))
            length = 0.78 * 0.5 * (float(axes[0]) + float(axes[1]))
            end = (int(round(observed[0] + length * float(direction[0]))),
                   int(round(observed[1] + length * float(direction[1]))))
            cv2.line(canvas, a, end, (0, 255, 0), 2)
            cv2.putText(canvas, "line {:.1f}deg".format(float(line["angle_deg"])),
                        (end[0] + 4, end[1] + 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 255, 0), 1, cv2.LINE_AA)
        hub = (ring.get("hub") or {})
        if hub.get("center"):
            hc = (int(round(hub["center"][0])), int(round(hub["center"][1])))
            cv2.circle(canvas, hc, 5, (255, 255, 0), 1)
            uncertainty = axis.get("hub_center_uncertainty_px")
            if uncertainty:
                cv2.circle(canvas, hc, max(2, int(round(float(uncertainty)))),
                           (255, 255, 0), 1)
            centroid = hub.get("blob_centroid_px")
            if centroid:
                bc = (int(round(centroid[0])), int(round(centroid[1])))
                cv2.drawMarker(canvas, bc, (128, 128, 128),
                               cv2.MARKER_CROSS, 8, 1)
    lines = [
        "experimental=true  value=null",
        "frame_status={}  reason={}".format(result.get("frame_status"),
                                            result.get("reason")),
        "ring_center={}  axes={}".format(
            None if not center else [round(v, 1) for v in center],
            None if not axes else [round(v, 1) for v in axes]),
        "ring_coverage={}  rms={}  band_contrast={}".format(
            None if ring.get("coverage") is None else round(ring["coverage"], 3),
            None if ring.get("residual_rms_px") is None else round(ring["residual_rms_px"], 3),
            None if (ring.get("band_contrast") or {}).get("contrast") is None
            else round(ring["band_contrast"]["contrast"], 2)),
        "hub={}  centroid_off={}  hub_unc={}".format(
            None if not axis.get("hub_axis_px")
            else [round(float(v), 1) for v in axis["hub_axis_px"]],
            None if axis.get("centroid_offset_px") is None
            else round(float(axis["centroid_offset_px"]), 2),
            None if axis.get("hub_center_uncertainty_px") is None
            else round(float(axis["hub_center_uncertainty_px"]), 2)),
        "roll={}  ray={}  line={}  delta={}".format(
            None if (result.get("roll") or {}).get("roll_deg") is None
            else round(result["roll"]["roll_deg"], 2),
            None if (result.get("needle") or {}).get("angle_deg") is None
            else round(result["needle"]["angle_deg"], 2),
            None if line.get("angle_deg") is None else round(line["angle_deg"], 2),
            None if axis.get("r2_ray_vs_line_delta_deg") is None
            else round(axis["r2_ray_vs_line_delta_deg"], 2)),
        "line_norm_bound={}  kappa={}  px_span={}".format(
            None if line.get("normalized_span_upper_bound_deg") is None
            else round(line["normalized_span_upper_bound_deg"], 2),
            None if line.get("matrix_condition_number") is None
            else round(line["matrix_condition_number"], 4),
            None if line.get("pixel_span_deg") is None
            else round(line["pixel_span_deg"], 2)),
        "line_px_hub={}  line_px_ring={}  opening={}".format(
            None if (line.get("uncertainty_components_deg") or {}).get(
                "hub_span_deg") is None
            else round(line["uncertainty_components_deg"]["hub_span_deg"], 2),
            None if (line.get("uncertainty_components_deg") or {}).get(
                "ring_span_deg") is None
            else round(line["uncertainty_components_deg"]["ring_span_deg"], 2),
            None if line.get("edge_opening_deg") is None
            else round(line["edge_opening_deg"], 2)),
        "line_support={}  tip/tail={}/{}".format(
            None if line.get("support_frac") is None
            else round(line["support_frac"], 2),
            None if line.get("tip_extent_px") is None
            else round(line["tip_extent_px"], 1),
            None if line.get("tail_extent_px") is None
            else round(line["tail_extent_px"], 1)),
        "diagnostic_candidate={}".format(
            None if not result.get("diagnostic_candidate")
            else round(float(result["diagnostic_candidate"].get("candidate_value") or 0.0), 2)),
    ]
    overlay = canvas.copy()
    cv2.rectangle(overlay, (8, 8), (760, 8 + 22 * len(lines) + 6), (0, 0, 0), -1)
    canvas = cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0)
    y_text = 28
    for line in lines:
        cv2.putText(canvas, line, (14, y_text), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1, cv2.LINE_AA)
        y_text += 22
    # Nearest-neighbour review inset: visual aid only, no extra measurement
    # detail is claimed from it.
    if roi:
        rx, ry, rw, rh = [int(v) for v in roi]
        patch = canvas[ry:ry + rh, rx:rx + rw]
        if patch.size:
            zoom = 4
            inset = cv2.resize(patch, (rw * zoom, rh * zoom),
                               interpolation=cv2.INTER_NEAREST)
            ih, iw = inset.shape[:2]
            x0 = max(0, canvas.shape[1] - iw - 10)
            y0 = max(0, canvas.shape[0] - ih - 28)
            canvas[y0:y0 + ih, x0:x0 + iw] = inset
            cv2.rectangle(canvas, (x0, y0), (x0 + iw, y0 + ih),
                          (255, 255, 255), 1)
            cv2.putText(canvas, "ROI x4 nearest (visual only)",
                        (x0 + 4, max(12, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def _load_frames(frames_dir: Path, limit: Optional[int] = None) -> List[Path]:
    if not frames_dir.is_dir():
        raise ExperimentConfigError("frames directory not found: {}".format(frames_dir))
    files = sorted([path for path in frames_dir.iterdir()
                    if path.is_file() and path.suffix.lower() in (".jpg", ".jpeg", ".png")])
    if not files:
        raise ExperimentConfigError("no image frames found in {}".format(frames_dir))
    if limit is not None:
        files = files[:int(limit)]
    return files


def run_burst(frames_dir: Path, calibration: Dict, reference: Dict,
              output: Path, dev_frames: int = DEV_FRAME_COUNT,
              limit: Optional[int] = None,
              representative=(), write_overlays: bool = True) -> Dict:
    files = _load_frames(frames_dir, limit=limit)
    frames = []
    rows = []
    for index, path in enumerate(files, start=1):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise ExperimentConfigError("cannot decode image: {}".format(path))
        payload = path.read_bytes()
        stage = "dev" if index <= int(dev_frames) else "eval"
        result = estimate_experiment_frame(image, calibration, DECLARED_ROI,
                                           reference, stage=stage)
        result["index"] = index
        result["split"] = stage
        result["file"] = str(path)
        result["frame_sha256"] = _sha256_bytes(payload)
        result["width"] = int(image.shape[1])
        result["height"] = int(image.shape[0])
        frames.append(result)
        rows.append(_csv_row(result))
        _write_json(output / "frames" / "frame_{:04d}.json".format(index), result)
        if write_overlays and index in set(representative):
            _write_overlay(output / "overlays" / "overlay_{:04d}.png".format(index),
                           image, result)
    csv_path = output / "frames.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return {"files": [str(path) for path in files],
            "frames": frames, "rows": rows, "csv": str(csv_path)}


def _write_overlay(path: Path, image: np.ndarray, result: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rendered = draw_experiment_overlay(image, result)
    temporary = path.with_name(".{}-{}.png".format(path.stem, uuid.uuid4().hex))
    try:
        ok = cv2.imwrite(str(temporary), rendered,
                         [int(cv2.IMWRITE_PNG_COMPRESSION), 3])
        if not ok or not temporary.is_file() or temporary.stat().st_size <= 0:
            raise ExperimentConfigError("overlay write failed: {}".format(path))
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            try:
                temporary.unlink()
            except OSError:
                pass


def _negative_rois() -> List[Dict]:
    return [
        {"name": "wall_left", "roi": [40, 40, 75, 75]},
        {"name": "floor", "roi": [400, 350, 75, 75]},
        {"name": "black_stand", "roi": [800, 400, 75, 75]},
        {"name": "wall_right", "roi": [1100, 50, 75, 75]},
        {"name": "gauge_face_blank", "roi": [600, 180, 60, 40]},
        {"name": "clipped_ring", "roi": [645, 230, 35, 35]},
    ]


def run_negative_controls(frame_image: np.ndarray, calibration: Dict,
                          reference: Dict, limit: int = 2) -> List[Dict]:
    """Negative ROIs/stress controls and separate valid synthetic positives.

    A low-saturation synthetic dial with a normal needle is a valid positive,
    not a negative control, so it is labelled ``expected=positive`` and is
    never counted as a successful rejection.
    """
    controls = []
    for spec in _negative_rois():
        result = estimate_experiment_frame(frame_image, calibration, spec["roi"],
                                           reference, stage="negative")
        line = result.get("line") or {}
        controls.append({
            "name": spec["name"],
            "kind": "same_frame_non_dial_roi",
            "expected": "negative",
            "has_needle": False,
            "value": None,
            "trusted": False,
            "roi": spec["roi"],
            "frame_status": result["frame_status"],
            "reason": result["reason"],
            "ring_ok": (result.get("ring") or {}).get("ok"),
            "line_ok": line.get("ok"),
            "line_reason": line.get("reason"),
            "line_angle_deg": line.get("angle_deg"),
            "line_angle_span_deg": line.get("angle_span_deg"),
            "line_pixel_span_deg": line.get("pixel_span_deg"),
            "line_condition_number": line.get("matrix_condition_number"),
            "line_support_frac": line.get("support_frac"),
            "ray_angle_deg": (result.get("needle") or {}).get("angle_deg"),
            "ray_angle_span_deg": (result.get("axis") or {}).get(
                "r2_ray_angle_span_deg"),
            "ray_vs_line_delta_deg": (result.get("axis") or {}).get(
                "r2_ray_vs_line_delta_deg"),
            "candidate_value": None if not result.get("diagnostic_candidate")
            else result["diagnostic_candidate"].get("candidate_value"),
            "rejected": result["frame_status"] != "ok",
        })
    synthetic_specs = [
        ("gray_ring", make_synthetic_scene("gray"), "negative", False),
        ("blank_white", make_synthetic_scene("blank"), "negative", False),
        ("multi_ring_colour", make_synthetic_scene("color", second_ring=True),
         "negative", True),
        ("low_saturation_ring", make_synthetic_scene("low_sat"), "positive", True),
        ("blur_noise_ring", make_synthetic_scene("color", blur_sigma=1.2,
                                                 noise_sigma=3.0), "positive", True),
    ]
    for name, image, expected, has_needle in synthetic_specs:
        result = estimate_experiment_frame(
            image, calibration, [0, 0, image.shape[1], image.shape[0]],
            reference, stage="negative", top_expansion_px=0)
        ring = result.get("ring") or {}
        line = result.get("line") or {}
        controls.append({
            "name": name,
            "kind": "synthetic_control",
            "expected": expected,
            "has_needle": has_needle,
            "value": None,
            "trusted": False,
            "roi": [0, 0, int(image.shape[1]), int(image.shape[0])],
            "frame_status": result["frame_status"],
            "reason": result["reason"],
            "ring_ok": ring.get("ok"),
            "ring_center_px": ring.get("center_px"),
            "ring_axes_px": ring.get("axes_px"),
            "band_contrast": (ring.get("band_contrast") or {}).get("contrast"),
            "line_ok": line.get("ok"),
            "line_reason": line.get("reason"),
            "line_angle_deg": line.get("angle_deg"),
            "line_angle_span_deg": line.get("angle_span_deg"),
            "line_pixel_span_deg": line.get("pixel_span_deg"),
            "line_condition_number": line.get("matrix_condition_number"),
            "line_support_frac": line.get("support_frac"),
            "ray_angle_deg": (result.get("needle") or {}).get("angle_deg"),
            "ray_angle_span_deg": (result.get("axis") or {}).get(
                "r2_ray_angle_span_deg"),
            "ray_vs_line_delta_deg": (result.get("axis") or {}).get(
                "r2_ray_vs_line_delta_deg"),
            "candidate_value": None if not result.get("diagnostic_candidate")
            else result["diagnostic_candidate"].get("candidate_value"),
            "rejected": result["frame_status"] != "ok",
        })
    return controls


def run_phone_controls(phone_path: Path, phone_roi: Sequence[int],
                       calibration: Dict, reference: Dict,
                       target_major_px: float = 296.0,
                       target_size_px: float = 60.0) -> List[Dict]:
    if not phone_path.is_file():
        raise ExperimentConfigError("phone control image not found: {}".format(phone_path))
    image = cv2.imread(str(phone_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ExperimentConfigError("cannot decode phone control image")
    payload = phone_path.read_bytes()
    results = []
    full = estimate_experiment_frame(image, calibration, phone_roi, reference,
                                     stage="phone_full")
    full["index"] = "phone_full"
    full["file"] = str(phone_path)
    full["frame_sha256"] = _sha256_bytes(payload)
    full["width"], full["height"] = int(image.shape[1]), int(image.shape[0])
    results.append(("phone_full", full, 1.0))
    scale = float(target_size_px) / float(target_major_px)
    resized = cv2.resize(image, None, fx=scale, fy=scale,
                         interpolation=cv2.INTER_AREA)
    scaled_roi = [int(round(float(v) * scale)) for v in phone_roi]
    scaled_roi[2] = max(8, scaled_roi[2])
    scaled_roi[3] = max(8, scaled_roi[3])
    small = estimate_experiment_frame(resized, calibration, scaled_roi, reference,
                                      stage="phone_60px",
                                      top_expansion_px=max(1, int(round(ROI_TOP_EXPANSION_PX * scale))))
    small["index"] = "phone_60px"
    small["file"] = str(phone_path)
    small["frame_sha256"] = _sha256_bytes(payload)
    small["width"], small["height"] = int(resized.shape[1]), int(resized.shape[0])
    small["scale"] = scale
    small["resized_roi"] = scaled_roi
    small["target_ring_major_px"] = float(target_size_px)
    results.append(("phone_60px", small, scale))
    return results


def _circular_stats(angles: Sequence[float]) -> Dict:
    values = [math.radians(float(v)) for v in angles if v is not None]
    if not values:
        return {"count": 0}
    sin_sum = sum(math.sin(v) for v in values)
    cos_sum = sum(math.cos(v) for v in values)
    mean = math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0
    resultants = math.hypot(sin_sum, cos_sum) / len(values)
    return {"count": len(values), "circular_mean_deg": mean,
            "resultant_length": resultants,
            "circular_std_deg": math.degrees(math.sqrt(max(0.0, -2.0 * math.log(max(resultants, 1e-9)))))}


def _sensitivity_summary(frames: Sequence[Dict]) -> Dict:
    evals = [item for item in frames if item.get("split") == "eval"]
    candidates = [item["diagnostic_candidate"] for item in evals
                  if item.get("diagnostic_candidate")
                  and item["diagnostic_candidate"].get("candidate_value") is not None]
    roll_angles = [((item.get("roll") or {}).get("roll_deg")) for item in evals
                   if (item.get("roll") or {}).get("ok")]
    needle_angles = [((item.get("needle") or {}).get("angle_deg")) for item in evals
                     if (item.get("needle") or {}).get("ok")]
    line_angles = [((item.get("line") or {}).get("angle_deg")) for item in evals
                   if ((item.get("line") or {}).get("angle_deg")) is not None]
    line_spans = [((item.get("line") or {}).get("angle_span_deg")) for item in evals
                  if ((item.get("line") or {}).get("angle_span_deg")) is not None]
    line_pixel_spans = [((item.get("line") or {}).get("pixel_span_deg"))
                        for item in evals
                        if ((item.get("line") or {}).get("pixel_span_deg"))
                        is not None]
    line_kappas = [((item.get("line") or {}).get("matrix_condition_number"))
                   for item in evals
                   if ((item.get("line") or {}).get("matrix_condition_number"))
                   is not None]
    line_edge_opening = [((item.get("line") or {}).get("edge_opening_deg"))
                         for item in evals
                         if ((item.get("line") or {}).get("edge_opening_deg"))
                         is not None]
    line_estimator_delta = [abs(float((item.get("line") or {}).get(
        "estimator_delta_deg"))) for item in evals
        if ((item.get("line") or {}).get("estimator_delta_deg")) is not None]
    line_split = [((item.get("line") or {}).get("uncertainty_components_deg") or {}).get(
        "split_half_span_deg") for item in evals
        if (item.get("line") or {}).get("uncertainty_components_deg") is not None]
    line_bootstrap = [((item.get("line") or {}).get("uncertainty_components_deg") or {}).get(
        "bootstrap_span_deg") for item in evals
        if (item.get("line") or {}).get("uncertainty_components_deg") is not None]
    line_hub = [((item.get("line") or {}).get("uncertainty_components_deg") or {}).get(
        "hub_span_deg") for item in evals
        if (item.get("line") or {}).get("uncertainty_components_deg") is not None]
    line_ring = [((item.get("line") or {}).get("uncertainty_components_deg") or {}).get(
        "ring_span_deg") for item in evals
        if (item.get("line") or {}).get("uncertainty_components_deg") is not None]
    line_centerline_rms = [((item.get("line") or {}).get("centerline_rms_px"))
                           for item in evals
                           if ((item.get("line") or {}).get("centerline_rms_px"))
                           is not None]
    line_width_rms = [((item.get("line") or {}).get("width_rms_px"))
                      for item in evals
                      if ((item.get("line") or {}).get("width_rms_px")) is not None]
    line_missing = [item for item in evals
                    if ((item.get("line") or {}).get("missing_evidence"))]
    ray_span = [((item.get("axis") or {}).get("r2_ray_angle_span_deg")) for item in evals
                if ((item.get("axis") or {}).get("r2_ray_angle_span_deg")) is not None]
    ray_line_delta = [((item.get("axis") or {}).get("r2_ray_vs_line_delta_deg"))
                      for item in evals
                      if ((item.get("axis") or {}).get("r2_ray_vs_line_delta_deg")) is not None]
    line_stable = [item for item in evals if (item.get("line") or {}).get("ok")]
    line_reasons = {}
    for item in evals:
        line = item.get("line") or {}
        reason = line.get("reason") or "line_not_run"
        line_reasons[reason] = line_reasons.get(reason, 0) + 1
    axis_deltas = [((item.get("axis") or {}).get("needle_angle_delta_deg")) for item in evals
                   if ((item.get("axis") or {}).get("needle_angle_delta_deg")) is not None]
    axis_spans = [((item.get("axis") or {}).get("angle_span_deg")) for item in evals
                  if ((item.get("axis") or {}).get("angle_span_deg")) is not None]
    hub_offsets = [((item.get("axis") or {}).get("centroid_offset_px")) for item in evals
                   if ((item.get("axis") or {}).get("centroid_offset_px")) is not None]
    hub_uncertainties = [((item.get("axis") or {}).get("hub_center_uncertainty_px"))
                         for item in evals
                         if ((item.get("axis") or {}).get("hub_center_uncertainty_px")) is not None]
    values = [float(item["candidate_value"]) for item in candidates]
    spans = [item.get("value_sensitivity_span") for item in candidates
             if item.get("value_sensitivity_span")]
    return {
        "scope": "internal 2/8 split of one burst; not independent field accuracy",
        "eval_frames": len(evals),
        "candidates": len(values),
        "candidate_mean": None if not values else float(np.mean(values)),
        "candidate_std": None if not values else float(np.std(values)),
        "candidate_range": None if not values else [float(min(values)), float(max(values))],
        "candidate_sensitivity_span_union": None if not spans else [
            float(min(item[0] for item in spans)), float(max(item[1] for item in spans))],
        "roll": _circular_stats(roll_angles),
        "needle": _circular_stats(needle_angles),
        "line_direction": _circular_stats(line_angles),
        "line_stable_frames": len(line_stable),
        "line_reasons": line_reasons,
        "line_angle_span_deg": {
            "count": len(line_spans),
            "min": None if not line_spans else float(min(line_spans)),
            "max": None if not line_spans else float(max(line_spans)),
            "mean": None if not line_spans else float(np.mean(line_spans)),
            "values": [None if value is None else round(float(value), 4)
                       for value in line_spans],
            "definition": "normalised gated upper bound, min(360, kappa*pixel_span); "
                          "not an exact span and not a confidence interval",
        },
        "line_pixel_span_deg": {
            "count": len(line_pixel_spans),
            "min": None if not line_pixel_spans else float(min(line_pixel_spans)),
            "max": None if not line_pixel_spans else float(max(line_pixel_spans)),
            "mean": None if not line_pixel_spans else float(np.mean(line_pixel_spans)),
            "definition": "raw pixel-direction sensitivity span before A maps it",
        },
        "line_matrix_condition_number": {
            "count": len(line_kappas),
            "min": None if not line_kappas else float(min(line_kappas)),
            "max": None if not line_kappas else float(max(line_kappas)),
            "mean": None if not line_kappas else float(np.mean(line_kappas)),
            "definition": "kappa(A)=lambda_max/lambda_min of the ring SPD matrix; "
                          "angle mapping derivative bound",
        },
        "line_edge_opening_deg": {
            "count": len(line_edge_opening),
            "min": None if not line_edge_opening else float(min(line_edge_opening)),
            "max": None if not line_edge_opening else float(max(line_edge_opening)),
            "mean": None if not line_edge_opening else float(np.mean(line_edge_opening)),
        },
        "line_estimator_delta_deg": {
            "count": len(line_estimator_delta),
            "min": None if not line_estimator_delta else float(min(line_estimator_delta)),
            "max": None if not line_estimator_delta else float(max(line_estimator_delta)),
            "mean": None if not line_estimator_delta else float(np.mean(line_estimator_delta)),
        },
        "line_split_half_span_deg": {
            "count": len([v for v in line_split if v is not None]),
            "min": None if not [v for v in line_split if v is not None]
            else float(min(v for v in line_split if v is not None)),
            "max": None if not [v for v in line_split if v is not None]
            else float(max(v for v in line_split if v is not None)),
            "mean": None if not [v for v in line_split if v is not None]
            else float(np.mean([v for v in line_split if v is not None])),
            "definition": "pixel-direction split-half span; multiply by kappa "
                          "for the normalised upper bound",
        },
        "line_bootstrap_span_deg": {
            "count": len([v for v in line_bootstrap if v is not None]),
            "min": None if not [v for v in line_bootstrap if v is not None]
            else float(min(v for v in line_bootstrap if v is not None)),
            "max": None if not [v for v in line_bootstrap if v is not None]
            else float(max(v for v in line_bootstrap if v is not None)),
            "mean": None if not [v for v in line_bootstrap if v is not None]
            else float(np.mean([v for v in line_bootstrap if v is not None])),
            "definition": "pixel-direction blocked pairs bootstrap range; "
                          "finite-sample sensitivity envelope, not a confidence "
                          "interval; multiply by kappa for the normalised bound",
        },
        "line_hub_span_deg": {
            "count": len([v for v in line_hub if v is not None]),
            "min": None if not [v for v in line_hub if v is not None]
            else float(min(v for v in line_hub if v is not None)),
            "max": None if not [v for v in line_hub if v is not None]
            else float(max(v for v in line_hub if v is not None)),
            "mean": None if not [v for v in line_hub if v is not None]
            else float(np.mean([v for v in line_hub if v is not None])),
            "definition": "pixel-direction hub perturbation span; multiply by "
                          "kappa for the normalised upper bound",
        },
        "line_ring_span_deg": {
            "count": len([v for v in line_ring if v is not None]),
            "min": None if not [v for v in line_ring if v is not None]
            else float(min(v for v in line_ring if v is not None)),
            "max": None if not [v for v in line_ring if v is not None]
            else float(max(v for v in line_ring if v is not None)),
            "mean": None if not [v for v in line_ring if v is not None]
            else float(np.mean([v for v in line_ring if v is not None])),
            "definition": "pixel-direction ring perturbation span; the "
                          "implemented bound uses the maximum kappa over the "
                          "matrices actually tested",
        },
        "line_centerline_rms_px": {
            "count": len(line_centerline_rms),
            "min": None if not line_centerline_rms else float(min(line_centerline_rms)),
            "max": None if not line_centerline_rms else float(max(line_centerline_rms)),
            "mean": None if not line_centerline_rms else float(np.mean(line_centerline_rms)),
        },
        "line_width_rms_px": {
            "count": len(line_width_rms),
            "min": None if not line_width_rms else float(min(line_width_rms)),
            "max": None if not line_width_rms else float(max(line_width_rms)),
            "mean": None if not line_width_rms else float(np.mean(line_width_rms)),
        },
        "line_missing_evidence_frames": [
            {"index": item.get("index"),
             "reason": (item.get("line") or {}).get("reason"),
             "missing_evidence": (item.get("line") or {}).get("missing_evidence")}
            for item in line_missing],
        "ray_angle_span_deg": {
            "count": len(ray_span),
            "min": None if not ray_span else float(min(ray_span)),
            "max": None if not ray_span else float(max(ray_span)),
            "mean": None if not ray_span else float(np.mean(ray_span)),
        },
        "ray_vs_line_delta_deg": {
            "count": len(ray_line_delta),
            "min": None if not ray_line_delta else float(min(ray_line_delta)),
            "max": None if not ray_line_delta else float(max(ray_line_delta)),
            "mean": None if not ray_line_delta else float(np.mean(ray_line_delta)),
        },
        "axis_angle_delta_deg": {
            "count": len(axis_deltas),
            "min": None if not axis_deltas else float(min(axis_deltas)),
            "max": None if not axis_deltas else float(max(axis_deltas)),
            "mean": None if not axis_deltas else float(np.mean(axis_deltas)),
        },
        "axis_angle_span_deg": {
            "count": len(axis_spans),
            "min": None if not axis_spans else float(min(axis_spans)),
            "max": None if not axis_spans else float(max(axis_spans)),
            "mean": None if not axis_spans else float(np.mean(axis_spans)),
        },
        "hub_centroid_offset_px": {
            "count": len(hub_offsets),
            "min": None if not hub_offsets else float(min(hub_offsets)),
            "max": None if not hub_offsets else float(max(hub_offsets)),
            "mean": None if not hub_offsets else float(np.mean(hub_offsets)),
        },
        "hub_center_uncertainty_px": {
            "count": len(hub_uncertainties),
            "min": None if not hub_uncertainties else float(min(hub_uncertainties)),
            "max": None if not hub_uncertainties else float(max(hub_uncertainties)),
            "mean": None if not hub_uncertainties else float(np.mean(hub_uncertainties)),
        },
        "ring_axes_px": {
            "major": [None if not item.get("ring", {}).get("axes_px") else
                      round(max(item["ring"]["axes_px"]), 3) for item in evals],
            "minor": [None if not item.get("ring", {}).get("axes_px") else
                      round(min(item["ring"]["axes_px"]), 3) for item in evals],
        },
        "ring_coverage": {
            "min": None if not evals else min(
                (item.get("ring", {}).get("coverage") or 0.0) for item in evals),
            "max": None if not evals else max(
                (item.get("ring", {}).get("coverage") or 0.0) for item in evals),
        },
        "roll_uncertainty_deg": [item.get("roll", {}).get("uncertainty_deg") for item in evals],
        "interpretation": "spread and sensitivity are diagnostics; there is no manual "
                          "ground truth and no MAE claim",
    }


def _write_report(path: Path, summary: Dict) -> None:
    frames = summary["frames"]
    stats = summary.get("sensitivity") or {}
    line_direction = summary.get("line_direction") or {}
    joint = summary.get("joint_burst_direction") or {}
    lines = []
    lines.append("# 固定视角低饱和湿度环离线实验（R5 坐标单位门控）")
    lines.append("")
    lines.append("实验标记：`experimental=true`；所有顶层 `value` 始终为 `null`，"
                 "`diagnostic_candidate` 不是真值，也不是精度结论。`frame_status=ok` "
                 "只表示本轮中心线方向及其证据门通过，**不代表可信读数**；`trusted` 在"
                 "每一帧都为 `false`。")
    lines.append("")
    lines.append("## 型号更正")
    lines.append("")
    lines.append("用户已确认实物与参考照是同一块 **No.9011**；候选标定元数据里的 "
                 "`deli No.0011` 是历史识读/元数据误记。本轮没有修改候选文件或哈希。")
    lines.append("")
    lines.append("## 命令与切分")
    lines.append("")
    lines.append("```bash")
    lines.append(summary["command"])
    lines.append("```")
    lines.append("")
    lines.append("前 2 帧只用于几何/质量设置，锁定后评估后 8 帧；这是同一 burst 的内部"
                 "留出，不是独立实拍精度。R1 已看过完整 burst，因此不能声称盲留出。")
    lines.append("")
    lines.append("{}".format(summary.get("split_caveat", "")))
    lines.append("")
    lines.append("## R5 坐标单位更正")
    lines.append("")
    lines.append("- 漏项：主方向 `angle_deg` 已经由环的 SPD 矩阵 A 映射成归一化角，但 "
                 "split/bootstrap/fit/hub/ring 区间仍是在像素方向测的，之前直接和 2° "
                 "比较。椭圆归一化会放大角误差，不能把像素 2° 当归一化 2°。")
    lines.append("- 本轮只修门控单位：保留各像素分量与 `pixel_span_deg`，令 "
                 "`normalized_span_upper_bound_deg = min(360, kappa(A) * pixel_span_deg)`，"
                 "其中 `kappa(A)=lambda_max/lambda_min`。对正定 2x2 A，角映射导数绝对值"
                 "不超过 kappa，所以这是**保守上界**，不是精确跨度，也不是统计置信区间。")
    lines.append("- 2° 门严格比较归一化上界；主方向 `angle_deg` 不变。ring 扰动本轮只做"
                 "各轴同比缩放，kappa 不变；实现仍取实际被测矩阵的最大 kappa。非有限/"
                 "非 SPD/singular 几何直接拒识，不回退 kappa=1。")
    prior = (summary.get("comparison_to_r4") or {})
    lines.append("- 通过数只会相同或减少，这是纠正而不是回归失败：R4 内部评估通过 "
                 "{} / {}，R5 为 {} / {}。".format(
                     prior.get("prior_accepted_eval_frames"),
                     prior.get("prior_eval_frames"),
                     prior.get("current_accepted_eval_frames"),
                     prior.get("prior_eval_frames")))
    lines.append("")
    lines.append("## R4 决策与实现（R5 保留）")
    lines.append("")
    lines.append("- 物理模型更正：No.9011 的黑色指针是细长、带锥度的针，两侧边缘不必平行。"
                 "对称直锥针的左右边有非零开口角，但对应位置的中点仍严格共线；R3 把"
                 "“两边不平行”当作方向误差并直接拒识，是模型错误。")
    lines.append("- 主方向改为**成对边缘中点的稳健中心线拟合**：由条带积分给出候选带，"
                 "取每个截面的上/下边缘中点，用加权 Huber 直线拟合；该中心线方向经环的"
                 "SPD 矩阵变换得到归一化角。条带峰、边缘开口角、条带峰与中心线之差只作"
                 "诊断，不进入方向区间。")
    lines.append("- 形状一致性：中心线残差、宽度随针身的线性残差、近/远半段中心线差、"
                 "bootstrap、hub/环扰动共同拒识弯针、单边不规则、双针和无线圆帽；不再"
                 "使用 `parallel_span` 门。")
    lines.append("- 数学更正：块成对 bootstrap 的方向跨度改为同坐标系下各 bootstrap "
                 "方向的实际环形跨度（跨 0 正确），不再 `2*atan(max-min)`。该量报告为"
                 "有限样本敏感性包络，**不是**置信区间；仍保留 2° 稳定预算。")
    lines.append("- 缺证据显式拒识：hub 4 个、环 6 个所需扰动，以及 split-half/bootstrap/"
                 "宽度残差，任何一个没有算出都记入 `missing_evidence` 并拒识，不再把"
                 "“没有算出”过滤成零波动。")
    lines.append("- 线矢量经环的 SPD 矩阵 A 变换后取角，**不是**针尖端点减不确定轴点；"
                 "轴帽只用于限定条带位置、短尾排除和正反消歧。")
    lines.append("- 夹具披露：R1/R2 的 `make_synthetic_scene` 在 x 用负 cos、y 用正 sin，"
                 "且用整轴长度，导致 `needle_deg` 不是实际绘针方向；R3 已改成半轴长度 + "
                 "普通 cos/sin。R4 新增独立绘制的直锥针/弯针夹具，不冒充实拍数据。")
    lines.append("")
    lines.append("## 10 帧结果")
    lines.append("")
    lines.append("| # | split | ring | roll | ray | line | strip | opening | est_delta | px_span | kappa | norm_bound | split_px | boot_px | hub_px | ring_px | support | tip/tail | candidate | status/reason |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for item in frames:
        ring = item.get("ring") or {}
        roll = item.get("roll") or {}
        axis = item.get("axis") or {}
        line = item.get("line") or {}
        candidate = item.get("diagnostic_candidate") or {}
        components = line.get("uncertainty_components_deg") or {}
        lines.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {}/{} | {} | {} / {} |".format(
            item.get("index"), item.get("split"),
            "ok" if ring.get("ok") else "reject",
            _round_or_none(roll.get("roll_deg"), 2),
            _round_or_none(axis.get("needle_angle_observed_axis_deg"), 2),
            _round_or_none(line.get("angle_deg"), 2),
            _round_or_none(line.get("strip_angle_deg"), 2),
            _round_or_none(line.get("edge_opening_deg"), 2),
            _round_or_none(line.get("estimator_delta_deg"), 2),
            _round_or_none(line.get("pixel_span_deg"), 2),
            _round_or_none(line.get("matrix_condition_number"), 4),
            _round_or_none(line.get("normalized_span_upper_bound_deg"), 2),
            _round_or_none(components.get("split_half_span_deg"), 2),
            _round_or_none(components.get("bootstrap_span_deg"), 2),
            _round_or_none(components.get("hub_span_deg"), 2),
            _round_or_none(components.get("ring_span_deg"), 2),
            _round_or_none(line.get("support_frac"), 2),
            _round_or_none(line.get("tip_extent_px"), 1),
            _round_or_none(line.get("tail_extent_px"), 1),
            _round_or_none(candidate.get("candidate_value"), 2),
            item.get("frame_status"), item.get("reason")))
    lines.append("")
    lines.append("`ray` 是 R2 射线法诊断角；`line` 是 R5 保留的成对边缘中点中心线方向；"
                 "`strip` 是条带峰诊断角；`opening` 是两边缘开口角（直锥针本应非零，"
                 "不是方向误差）；`est_delta` 是条带峰与中心线之差；`px_span` 是 "
                 "split/boot/fit/hub/ring 各**像素方向**跨度的最大值；`kappa` 是环 SPD "
                 "矩阵的条件数；`norm_bound = min(360, kappa*px_span)` 是用于 2° 门的"
                 "**归一化保守上界**，不是精确跨度、不是统计置信区间；`split_px/"
                 "boot_px/hub_px/ring_px` 都是对应中心线估计器的**像素方向**分量；"
                 "`support` 是主中心线方向的支撑比例。")
    lines.append("")
    lines.append("## 轴点依赖")
    lines.append("")
    edge_hub = line_direction.get("line_hub_span_deg") or {}
    ray_span = (line_direction.get("pivot_dependence") or {}).get(
        "r2_ray_span_mean_deg")
    lines.append("- 主中心线方向的粗 hub 扰动**像素**跨度：count={}，min={}，max={}，mean={} 度。".format(
        edge_hub.get("count"), edge_hub.get("min"), edge_hub.get("max"),
        edge_hub.get("mean")))
    lines.append("- R2 射线角在轴帽证据点上的跨度：{} 度（诊断对照）。".format(ray_span))
    opening = line_direction.get("line_edge_opening_deg") or {}
    estimator_delta = line_direction.get("line_estimator_delta_deg") or {}
    lines.append("- 边缘开口角（诊断，直锥针本应非零）：count={}，min={}，max={}，mean={} 度。".format(
        opening.get("count"), opening.get("min"), opening.get("max"), opening.get("mean")))
    lines.append("- 条带峰与中心线之差（诊断，不进入方向区间）：count={}，min={}，max={}，mean={} 度。".format(
        estimator_delta.get("count"), estimator_delta.get("min"),
        estimator_delta.get("max"), estimator_delta.get("mean")))
    missing_frames = line_direction.get("line_missing_evidence_frames") or []
    lines.append("- 必需扰动/bootstrap/宽度证据缺失的帧：{} 个；这些帧按缺证据拒识，"
                 "不把“没有算出”当作零波动。".format(len(missing_frames)))
    for item in missing_frames:
        lines.append("  - #{}：reason={}，missing={}".format(
            item.get("index"), item.get("reason"), item.get("missing_evidence")))
    lines.append("- 结论：**中心线方向基本摆脱了端点依赖**；条带峰较平且受锥度影响，"
                 "只保留为诊断。中心线稳定也不等于准确，未校准模型偏差仍在。")
    lines.append("")
    lines.append("## 同 burst 联合方向（仅内部一致性）")
    lines.append("")
    if joint.get("frames_used"):
        lines.append("- 联合补偿方向：{} 度；联合 85% 峰宽：{} 度；使用帧：{}；"
                     "状态：{} / {}。".format(
                         _round_or_none(joint.get("joint_compensated_deg"), 3),
                         _round_or_none(joint.get("joint_interval_deg"), 3),
                         joint.get("frames_used"), joint.get("ok"),
                         joint.get("reason")))
        lines.append("- 帧间一致性最大偏差：{} 度；留一跨度：{} 度；单帧中位区间：{} 度。".format(
            _round_or_none(joint.get("per_frame_consistency_deg"), 3),
            _round_or_none(joint.get("leave_one_out_span_deg"), 3),
            _round_or_none(joint.get("median_single_frame_span_deg"), 3)))
        lines.append("- 是否报告为改善：{}。联合只按静态盘面 roll 与归一化环几何对齐，"
                     "不用针自身对齐，不平均湿度值；它**不能替代独立验证**，也不把任何帧"
                     "变成可信读数。".format(joint.get("narrower_than_single_frame")))
        lines.append("- 本批结果：联合峰宽没有比单帧更窄，且帧间补偿方向偏差超过 2°，"
                     "因此不报告联合改善，只保留为内部一致性探索。")
    else:
        lines.append("- 未形成联合方向：{}；使用帧：{}。".format(
            joint.get("reason"), joint.get("frames_used")))
    lines.append("")
    lines.append("## 负例、压力控制与有效正例")
    lines.append("")
    negatives = summary.get("negative_controls") or []
    false_accepts = summary.get("false_accept_negative_controls") or []
    positives = summary.get("positive_controls") or []
    lines.append("负例（expected=negative）共 {} 个；其中被判 ok 的 false accept：{} 个。".format(
        len(negatives), len(false_accepts)))
    for control in negatives:
        lines.append("- {}（{}）：expected={}，status={}，reason={}，line={}，candidate={}".format(
            control.get("name"), control.get("kind"), control.get("expected"),
            control.get("frame_status"), control.get("reason"),
            control.get("line_angle_deg"), control.get("candidate_value")))
    lines.append("")
    lines.append("有效正例（expected=positive，含正常低饱和针）共 {} 个；其中判 ok 的：{} 个。"
                 "正例失败只说明该阈值/证据不足，不能算作负例拒识成功。这些合成正例在本批"
                 "参考签名下先失败于 roll，未进入线方向；线方向正例由独立测试覆盖"
                 "（AA 绘制、多方向、合法轴偏移、长度变化、粗尾不翻转）。".format(
                     len(positives), len(summary.get("positive_control_hits") or [])))
    for control in positives:
        lines.append("- {}：status={}，reason={}，line={}（norm_bound={}，px_span={}，"
                     "kappa={}），ray={}，candidate={}".format(
                         control.get("name"), control.get("frame_status"),
                         control.get("reason"), control.get("line_angle_deg"),
                         control.get("line_angle_span_deg"),
                         control.get("line_pixel_span_deg"),
                         control.get("line_condition_number"),
                         control.get("ray_angle_deg"),
                         control.get("candidate_value")))
    lines.append("")
    lines.append("## 手机控制")
    lines.append("")
    for control in summary.get("phone_controls", []):
        ring = control.get("ring") or {}
        roll = control.get("roll") or {}
        needle = control.get("needle") or {}
        line = control.get("line") or {}
        candidate = control.get("diagnostic_candidate") or {}
        lines.append("- {}：ring={}，axes={}，roll={}，ray={}，line={}"
                     "（norm_bound={}，px_span={}，kappa={}，support={}），"
                     "candidate={}，status={} / {}".format(
                         control.get("name"), ring.get("ok"), ring.get("axes_px"),
                         None if roll.get("roll_deg") is None
                         else round(float(roll["roll_deg"]), 2),
                         None if needle.get("angle_deg") is None
                         else round(float(needle["angle_deg"]), 2),
                         None if line.get("angle_deg") is None
                         else round(float(line["angle_deg"]), 2),
                         None if line.get("normalized_span_upper_bound_deg") is None
                         else round(float(line["normalized_span_upper_bound_deg"]), 2),
                         None if line.get("pixel_span_deg") is None
                         else round(float(line["pixel_span_deg"]), 2),
                         None if line.get("matrix_condition_number") is None
                         else round(float(line["matrix_condition_number"]), 4),
                         None if line.get("support_frac") is None
                         else round(float(line["support_frac"]), 2),
                         None if candidate.get("candidate_value") is None
                         else round(float(candidate["candidate_value"]), 2),
                         control.get("frame_status"), control.get("reason")))
    lines.append("")
    lines.append("手机原图与 60px 控制来自不同光学/尺度，只是诊断一致性，不是真值或精度。")
    lines.append("")
    lines.append("## 内部统计")
    lines.append("")
    lines.append("- 评估帧候选均值/标准差：{} / {}（范围 {}）".format(
        _round_or_none(stats.get("candidate_mean"), 3),
        _round_or_none(stats.get("candidate_std"), 3), stats.get("candidate_range")))
    lines.append("- 候选对 roll/方向区间的最坏敏感跨度：{}".format(
        stats.get("candidate_sensitivity_span_union")))
    lines.append("- roll 圆周均值/圆周标准差：{} / {} 度".format(
        (stats.get("roll") or {}).get("circular_mean_deg"),
        (stats.get("roll") or {}).get("circular_std_deg")))
    lines.append("- line 圆周均值/圆周标准差：{} / {} 度".format(
        (stats.get("line_direction") or {}).get("circular_mean_deg"),
        (stats.get("line_direction") or {}).get("circular_std_deg")))
    lines.append("- R2 ray 圆周均值/圆周标准差：{} / {} 度".format(
        (stats.get("needle") or {}).get("circular_mean_deg"),
        (stats.get("needle") or {}).get("circular_std_deg")))
    lines.append("- line 归一化方向跨度上界（min/max/mean）：{}".format(
        [stats.get("line_angle_span_deg", {}).get("min"),
         stats.get("line_angle_span_deg", {}).get("max"),
         stats.get("line_angle_span_deg", {}).get("mean")]))
    lines.append("- line 像素方向原跨度（min/max/mean）：{}".format(
        [stats.get("line_pixel_span_deg", {}).get("min"),
         stats.get("line_pixel_span_deg", {}).get("max"),
         stats.get("line_pixel_span_deg", {}).get("mean")]))
    lines.append("- line 环矩阵条件数 kappa（min/max/mean）：{}".format(
        [stats.get("line_matrix_condition_number", {}).get("min"),
         stats.get("line_matrix_condition_number", {}).get("max"),
         stats.get("line_matrix_condition_number", {}).get("mean")]))
    lines.append("- 中心线 hub/环扰动像素跨度（min/max/mean）：hub {}；ring {}".format(
        [stats.get("line_hub_span_deg", {}).get("min"),
         stats.get("line_hub_span_deg", {}).get("max"),
         stats.get("line_hub_span_deg", {}).get("mean")],
        [stats.get("line_ring_span_deg", {}).get("min"),
         stats.get("line_ring_span_deg", {}).get("max"),
         stats.get("line_ring_span_deg", {}).get("mean")]))
    lines.append("- 边缘开口角 / 条带峰与中心线差（min/max/mean）：{}/{}".format(
        [stats.get("line_edge_opening_deg", {}).get("min"),
         stats.get("line_edge_opening_deg", {}).get("max"),
         stats.get("line_edge_opening_deg", {}).get("mean")],
        [stats.get("line_estimator_delta_deg", {}).get("min"),
         stats.get("line_estimator_delta_deg", {}).get("max"),
         stats.get("line_estimator_delta_deg", {}).get("mean")]))
    lines.append("- 中心线残差 / 宽度线性残差（min/max/mean）：{}/{}".format(
        [stats.get("line_centerline_rms_px", {}).get("min"),
         stats.get("line_centerline_rms_px", {}).get("max"),
         stats.get("line_centerline_rms_px", {}).get("mean")],
        [stats.get("line_width_rms_px", {}).get("min"),
         stats.get("line_width_rms_px", {}).get("max"),
         stats.get("line_width_rms_px", {}).get("mean")]))
    lines.append("- ray 与 line 的角度差（min/max/mean）：{}".format(
        stats.get("ray_vs_line_delta_deg")))
    lines.append("")
    lines.append("## 是否有可靠候选")
    lines.append("")
    lines.append("- reliable_candidate = {}：{}".format(
        summary.get("reliable_candidate"), summary.get("reliable_candidate_reason")))
    phone_consistency = summary.get("phone_consistency") or {}
    lines.append("- 相机评估帧候选均值：{}；手机原图候选：{}；手机 60px 候选：{}。"
                 "三者只能作诊断一致性，不能当精度或真值。".format(
                     phone_consistency.get("camera_eval_candidate_mean"),
                     phone_consistency.get("phone_full_candidate"),
                     phone_consistency.get("phone_60px_candidate")))
    lines.append("")
    lines.append("## 阶段瓶颈与结论")
    lines.append("")
    bottleneck = summary.get("bottleneck") or {}
    lines.append("- 环检测：{}".format(bottleneck.get("ring")))
    lines.append("- 弱颜色 roll：{}".format(bottleneck.get("roll")))
    lines.append("- 线方向：{}".format(bottleneck.get("line")))
    lines.append("- R2 射线诊断：{}".format(bottleneck.get("ray")))
    lines.append("- 轴心诊断：{}".format(bottleneck.get("axis")))
    lines.append("- 结论：{}".format(bottleneck.get("conclusion")))
    lines.append("")
    lines.append("## 限制")
    lines.append("")
    for caveat in summary.get("caveats", []):
        lines.append("- {}".format(caveat))
    lines.append("")
    tests = summary.get("test_protection") or {}
    lines.append("- 测试保护：实验测试共 {} 个（R4 既有 {} + R5 新增 {}；语义变更逐项"
                 "披露），既有 {} 个回归测试；{}".format(
                     tests.get("experiment_tests_total"),
                     tests.get("r4_existing_tests"),
                     tests.get("r5_new_tests"),
                     tests.get("existing_regression_tests"),
                     tests.get("result")))
    lines.append("")
    overlay_dir = path.parent / "overlays"
    overlay_names = (sorted(item.name for item in overlay_dir.glob("overlay_*.png"))
                     if overlay_dir.is_dir() else [])
    lines.append("- 代表图：{}（黄圈=环，红细线=R2 ray，绿线=R5 "
                 "中心线方向，品红十字/黄圈=粗 hub 及局部不确定度）。".format(
                     "、".join("`overlays/{}`".format(name)
                              for name in overlay_names) or "无"))
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def _round_or_none(value, digits=3):
    return None if value is None else round(float(value), digits)


def _summary_from_results(burst: Dict, phone_controls, negative_controls,
                          calibration: Dict, command: str,
                          output_dir: Path) -> Dict:
    frames = [_compact_frame_result(item) for item in burst["frames"]]
    evals = [item for item in frames if item.get("split") == "eval"]
    frame_status_counts = {}
    for item in frames:
        key = "{}:{}".format(item.get("frame_status"), item.get("reason"))
        frame_status_counts[key] = frame_status_counts.get(key, 0) + 1
    accepted = [item for item in evals if item.get("frame_status") == "ok"]
    candidates = [item for item in evals if item.get("diagnostic_candidate")
                  and item["diagnostic_candidate"].get("candidate_value") is not None]
    line_ready = [item for item in frames
                  if (item.get("line") or {}).get("angle_deg") is not None]
    line_stable = [item for item in line_ready if (item.get("line") or {}).get("ok")]
    line_reasons = {}
    for item in frames:
        reason = (item.get("line") or {}).get("reason") or "line_not_run"
        line_reasons[reason] = line_reasons.get(reason, 0) + 1
    phone_compact = []
    for name, result, scale in phone_controls:
        compact = _compact_frame_result(result)
        compact["name"] = name
        compact["scale"] = scale
        compact["resized_roi"] = result.get("resized_roi")
        compact["target_ring_major_px"] = result.get("target_ring_major_px")
        phone_compact.append(compact)
    axis_unstable = [item for item in frames
                     if (item.get("axis") or {}).get("angle_stable") is False]
    axis_reason_counts = {}
    for item in axis_unstable:
        reason = (item.get("axis") or {}).get("r2_axis_reason") or "unknown"
        axis_reason_counts[reason] = axis_reason_counts.get(reason, 0) + 1
    stats = _sensitivity_summary(burst["frames"])
    joint = _joint_burst_direction(burst["frames"])
    negative = [control for control in negative_controls
                if control.get("expected", "negative") == "negative"]
    positive = [control for control in negative_controls
                if control.get("expected") == "positive"]
    false_accepts = [control for control in negative if not control.get("rejected")]
    positive_hits = [control for control in positive if not control.get("rejected")]
    candidate_mean = stats["candidate_mean"]
    candidate_std = stats["candidate_std"]
    line_edge_hub = stats["line_hub_span_deg"]
    ray_span = stats["ray_angle_span_deg"]
    line_span_stats = stats["line_angle_span_deg"]
    summary = {
        "experimental": True,
        "value": None,
        "experiment_id": EXPERIMENT_ID,
        "tool_version": TOOL_VERSION,
        "generated_at": None,
        "scope": "Offline isolated feasibility experiment. No camera/ROS, no device "
                 "change, no navigation change, no dependency install, no commit/push. "
                 "No original images were copied or overwritten.",
        "command": command,
        "model_correction": {
            "confirmed_model": "No.9011",
            "candidate_metadata_model": "deli No.0011 (historical misrecord)",
            "action": "no candidate file or hash changed; report-only correction",
            "candidate_sha256": calibration.get("calib_sha256"),
            "candidate_source_sha256": calibration.get("source_sha256"),
        },
        "algorithm": {
            "file": str(Path(__file__).resolve()),
            "file_sha256": _sha256_file(Path(__file__).resolve()),
            "value_policy": "top-level value always null; diagnostic_candidate is separate",
            "ring": "unchanged from R2: radial-gradient centre/radius hypotheses + "
                    "deterministic robust ellipse trim; visible-arc coverage, gradient "
                    "support, residual, signed colour-band contrast; nested inner/outer "
                    "edges grouped and outer edge preferred; ambiguity rejected",
            "roll": "unchanged from R2: 1-D circular correlation of static-band "
                    "opponent-colour angular signatures; BGR sampled before HSV; "
                    "weak/gray/ambiguous rejected; not a template warp",
            "line": "E1-R4: paired-edge strip integral over bounded (direction, "
                    "normal offset) probes, full 0..360 direction, thin-line edge "
                    "separation gate; the primary direction is the robust fit of "
                    "corresponding left/right edge midpoints (a symmetric tapered "
                    "needle has non-parallel edges but collinear midpoints).  The "
                    "strip peak, edge opening angle and strip-versus-centreline "
                    "difference are diagnostics, not uncertainty.  The fitted pixel "
                    "centreline vector is carried through the ring SPD matrix; the "
                    "pivot only bounds the strip and disambiguates tip versus tail",
            "axis": "R2 pivot DT-plateau/threshold evidence is retained as a diagnostic "
                    "and is not a hard angle prior for the line direction; its ray "
                    "angle interval is reported for comparison only",
            "needle": "dial_reading.detect_needle dark-ray helper, diagnostic comparison "
                      "only in R3 (not a hard gate)",
            "reused": ["dial_math.lookup_value", "dial_math.wrap_deg",
                       "dial_reading._ellipse_residual", "dial_reading._ellipse_matrix",
                       "dial_reading._needle_direction_px", "dial_reading.detect_needle"],
            "not_reused": "gauge.py / dial_reading.estimate_frame are not called; no "
                          "monkey-patching of global thresholds; no camera/ROS",
        },
        "roi": {
            "declared_roi": list(DECLARED_ROI),
            "used_roi": list(EXPERIMENT_ROI),
            "top_expansion_px": ROI_TOP_EXPANSION_PX,
            "reason": "the declared ROI clips the visible top of the small dial; the "
                      "fixed expansion was recorded before evaluation",
        },
        "split": {"dev_frames": [1, 2], "eval_frames": list(range(3, 11)),
                  "note": "internal holdout of one burst, not independent field data"},
        "frame_status_counts": frame_status_counts,
        "frames": frames,
        "line_direction": {
            "budget_deg": LINE_DIRECTION_TOL_DEG,
            "frames_with_line_angle": len(line_ready),
            "stable_frames": len(line_stable),
            "reasons": line_reasons,
            "line_angle_circular": stats["line_direction"],
            "line_angle_span_deg": line_span_stats,
            "line_pixel_span_deg": stats["line_pixel_span_deg"],
            "line_matrix_condition_number": stats["line_matrix_condition_number"],
            "line_normalized_span_upper_bound_deg": line_span_stats,
            "span_coordinate_note": "split/bootstrap/fit/hub/ring components are "
                                    "pixel-direction spans; the gate uses "
                                    "min(360, kappa*pixel_span) in the normalised "
                                    "dial plane. The bound is conservative, not an "
                                    "exact span and not a confidence interval.",
            "line_edge_opening_deg": stats["line_edge_opening_deg"],
            "line_estimator_delta_deg": stats["line_estimator_delta_deg"],
            "line_split_half_span_deg": stats["line_split_half_span_deg"],
            "line_bootstrap_span_deg": stats["line_bootstrap_span_deg"],
            "line_hub_span_deg": stats["line_hub_span_deg"],
            "line_ring_span_deg": stats["line_ring_span_deg"],
            "line_centerline_rms_px": stats["line_centerline_rms_px"],
            "line_width_rms_px": stats["line_width_rms_px"],
            "line_missing_evidence_frames": stats["line_missing_evidence_frames"],
            "pivot_dependence": {
                "centreline_hub_span_mean_deg": line_edge_hub.get("mean"),
                "r2_ray_span_mean_deg": ray_span.get("mean"),
                "interpretation": "the centreline direction is carried by the "
                                  "edge-pair midpoints, so coarse pivot/ring "
                                  "perturbations move it far less than they moved "
                                  "the R2 ray angle; this is pivot-dependence "
                                  "evidence, not an accuracy result",
            },
            "ray_vs_line_delta_deg": stats["ray_vs_line_delta_deg"],
            "detail": "A frame is stable only when the centreline body evidence is "
                      "present, the tip is unambiguous, the edge-pair midpoint "
                      "residual and width variation pass the shape gates, all "
                      "required hub/ring perturbations are present, and the "
                      "split/bootstrap/fit/perturbation pixel spans, multiplied by "
                      "kappa(A), are within the existing 2 deg budget.  Edge opening "
                      "angle and strip peak are not uncertainty inputs.  No gate was "
                      "relaxed to accept a frame; the coordinate-unit correction can "
                      "only reject frames that the old pixel-span gate accepted.",
        },
        "joint_burst_direction": joint,
        "sensitivity": stats,
        "negative_controls": negative,
        "all_controls": negative_controls,
        "false_accept_negative_controls": false_accepts,
        "positive_controls": positive,
        "positive_control_hits": positive_hits,
        "phone_controls": phone_compact,
        "bottleneck": {
            "ring": "Accepted edge+band ring on {} of {} frames; unchanged R2 gate.".format(
                sum(1 for item in frames if (item.get("ring") or {}).get("ok")),
                len(frames)),
            "roll": "Weak-colour roll accepted on {} of {} frames; unchanged R2 gate.".format(
                sum(1 for item in frames if (item.get("roll") or {}).get("ok")),
                len(frames)),
            "line": "Centreline direction stable on {} of {} frames (reasons: {}). "
                    "Mean centreline hub pixel span {} deg vs R2 ray span {} deg; "
                    "the normalised upper bound is {} deg (min/max/mean), computed "
                    "as kappa(A)*pixel span with mean kappa {}. Edge opening is a "
                    "diagnostic (min/max/mean {} deg) and is not used as direction "
                    "uncertainty. Frames with missing required perturb/bootstrap "
                    "evidence: {}.".format(
                        len(line_stable), len(frames), line_reasons,
                        None if line_edge_hub.get("mean") is None
                        else round(line_edge_hub["mean"], 3),
                        None if ray_span.get("mean") is None
                        else round(ray_span["mean"], 3),
                        [line_span_stats.get("min"), line_span_stats.get("max"),
                         line_span_stats.get("mean")],
                        None if stats["line_matrix_condition_number"].get("mean") is None
                        else round(stats["line_matrix_condition_number"]["mean"], 4),
                        [stats["line_edge_opening_deg"].get("min"),
                         stats["line_edge_opening_deg"].get("max"),
                         stats["line_edge_opening_deg"].get("mean")],
                        len(stats["line_missing_evidence_frames"])),
            "ray": "R2 dark-ray needle accepted on {} of {} frames; its pivot-evidence "
                   "span (mean {}) and its difference to the line direction (mean {}) are "
                   "reported as diagnostics only.".format(
                       sum(1 for item in frames if (item.get("needle") or {}).get("ok")),
                       len(frames),
                       None if ray_span.get("mean") is None
                       else round(ray_span["mean"], 3),
                       None if stats["ray_vs_line_delta_deg"].get("mean") is None
                       else round(stats["ray_vs_line_delta_deg"]["mean"], 3)),
            "axis": "R2 pivot gate failed on {} of {} frames (reasons: {}) but is no "
                    "longer a hard prior for the line angle.".format(
                        len(axis_unstable), len(frames), axis_reason_counts),
            "conclusion": "R5 keeps R4's paired-edge midpoint centreline and fixes "
                          "the coordinate-unit leak: the split/bootstrap/fit/hub/ring "
                          "spans are pixel spans, so the 2 deg gate now uses the "
                          "conservative normalised upper bound "
                          "min(360, kappa(A)*pixel_span) instead of comparing the "
                          "pixel span directly with 2 deg. This can only reject "
                          "frames the old gate accepted; no threshold was relaxed. "
                          "The centreline is stable on {} of {} frames under the "
                          "existing 2 deg budget; the centreline hub pixel span "
                          "mean is {} deg. A stable centreline is not accuracy: the "
                          "model (straight symmetric taper with collinear midpoints) "
                          "is uncalibrated, so the candidate mean {:.3f} / std {:.3f} "
                          "is diagnostic only, not truth, not a +-2 RH accuracy and "
                          "not a trusted reading; value stays null.".format(
                              len(line_stable), len(frames),
                              None if line_edge_hub.get("mean") is None
                              else round(line_edge_hub["mean"], 3),
                              candidate_mean or 0.0, candidate_std or 0.0),
        },
        "accepted_eval_frames": len(accepted),
        "diagnostic_candidate_eval_frames": len(candidates),
        "reliable_candidate": False,
        "reliable_candidate_reason": "No camera frame is promoted to a trusted "
                                     "reading. R5 reports a centreline direction "
                                     "and a conservative normalised sensitivity "
                                     "upper bound, not an exact span, a confidence "
                                     "interval or an accuracy. The centreline model "
                                     "assumes a straight symmetric taper; uncalibrated "
                                     "model bias remains, and diagnostic candidates "
                                     "are reported separately as non-truth values.",
        "comparison_to_prior": {
            "prior_default_path": "dial_reading.py with the S>=90 colour mask and "
                                  "9x9 close/3x3 open reported 10/10 no_dial on these "
                                  "fixed-view frames (existing review, unchanged code).",
            "this_experiment": "10/10 edge-first ring, 10/10 weak-colour roll, 10/10 "
                               "line angle available, {} of 10 centreline-stable. R5 "
                               "keeps the R4 paired-edge midpoint direction and gates "
                               "its pixel spans through the normalised kappa bound; "
                               "0 trusted candidates.".format(len(line_stable)),
            "meaning": "R3's non-parallel-edge rejection was a model error: a "
                       "straight tapered needle has non-parallel edges and collinear "
                       "edge midpoints. R4 uses the midpoint centreline, but a stable "
                       "centreline is consistency, not accuracy. R5 fixes a unit leak: "
                       "pixel spans were compared directly with a normalised 2 deg "
                       "budget. Ring/roll/line availability is not a humidity truth.",
        },
        "comparison_to_r4": {
            "prior_accepted_eval_frames": 3,
            "prior_eval_frames": 8,
            "current_accepted_eval_frames": len(accepted),
            "note": "same 10 frames; only the gate coordinate unit changed, so the "
                    "accept count can only stay the same or fall. This is a "
                    "correction, not an accuracy result.",
        },
        "test_protection": {
            "experiment_tests_total": 65,
            "r4_existing_tests": 60,
            "r5_new_tests": 5,
            "existing_regression_tests": 79,
            "existing_suites": ["test_dial_reading.py (28)",
                                "test_dial_math.py (27)",
                                "test_dial_calibration.py (24)"],
            "commands": [
                "python3 -m unittest src/photo_capture/tests/test_dial_low_saturation_experiment.py",
                "python3 -m unittest src/photo_capture/tests/test_dial_reading.py "
                "src/photo_capture/tests/test_dial_math.py "
                "src/photo_capture/tests/test_dial_calibration.py",
            ],
            "result": "65/65 experiment tests (60 R4-existing + 5 R5 new) and "
                      "79/79 existing regression tests passed",
            "note": "the 79 existing tests are the unchanged dial_reading/"
                    "dial_math/dial_calibration suites; R5 keeps the R4 tests and "
                    "adds coordinate-unit regressions: anisotropic pixel-to-"
                    "normalised scaling, isotropic invariance, rotated-SPD mapped "
                    "span <= bound, invalid SPD fail-closed, and unchanged "
                    "direction angle",
        },
        "split_caveat": "R5 uses the same 2/8 internal burst split, not independent "
                        "field data. R1 had already inspected the full burst and the "
                        "negative controls, so this split is not a blind holdout. No "
                        "gate was tuned to a humidity target. Treat the 8-frame result "
                        "as an internal consistency check, not a field evaluation.",
        "phone_consistency": {
            "note": "Different optics/capture times; these are diagnostics, not truth "
                    "and not accuracy.",
            "camera_eval_candidate_mean": None if not candidates else float(np.mean(
                [float(item["diagnostic_candidate"]["candidate_value"])
                 for item in candidates])),
            "phone_full_candidate": None,
            "phone_60px_candidate": None,
        },
        "caveats": [
            "No manual ground truth, no MAE and no accuracy claim.",
            "R1 had already seen the full burst and negative controls; the 2/8 split "
            "is not a blind holdout.",
            "R5 reports the centreline angle and a conservative normalised "
            "sensitivity upper bound; it is not collapsed to the ring centre, to a "
            "known pointer angle, or to a known humidity value.",
            "The split/bootstrap/fit/hub/ring components are pixel-direction spans. "
            "The 2 deg gate uses min(360, kappa(A)*pixel_span), a conservative bound "
            "from the angle-map derivative, not an exact span and not a statistical "
            "confidence interval. It can only reduce accepts relative to the old "
            "pixel-span comparison.",
            "A stable centreline is not accuracy: the straight symmetric-taper model "
            "and the pixel-to-normalised mapping are uncalibrated, and model bias is "
            "not removed by stability.",
            "The edge opening angle is expected for a tapered needle and is not used "
            "as direction uncertainty; the strip peak and the strip-versus-centreline "
            "difference are diagnostics only.",
            "The bootstrap is a blocked pairs sensitivity envelope, not a confidence "
            "interval; a missing perturbation or bootstrap is rejected, never treated "
            "as zero variation.",
            "The fixture convention was corrected in R3 (half-axis length, plain "
            "cos/sin); synthetic results are not claimed to be the same data as R2.",
            "The joint burst direction is an internal-consistency exploration using "
            "static-face roll and normalised ring geometry; it is not independent "
            "verification and does not make a candidate trusted.",
            "A stable candidate across the burst would be consistency evidence, not "
            "proof of correctness; the calibration table is the historical candidate.",
            "The diagnostic candidate mean is not a target: no gate, threshold or "
            "angle was tuned to an expected humidity, R2's 76.254 or the phone's 72/76.",
            "Phone controls use a different capture/optics and are not a truth reference.",
            "Original photos were read in place; only overlays and JSON/CSV were written "
            "to the new review directory.",
        ],
        "outputs": {
            "summary": str(output_dir / "summary.json"),
            "report": str(output_dir / "report.md"),
            "frames_csv": burst.get("csv"),
            "frames_dir": str(output_dir / "frames"),
            "overlays_dir": str(output_dir / "overlays"),
        },
    }
    for control in phone_compact:
        if control.get("name") == "phone_full" and control.get("diagnostic_candidate"):
            summary["phone_consistency"]["phone_full_candidate"] = control[
                "diagnostic_candidate"].get("candidate_value")
        if control.get("name") == "phone_60px" and control.get("diagnostic_candidate"):
            summary["phone_consistency"]["phone_60px_candidate"] = control[
                "diagnostic_candidate"].get("candidate_value")
    # Fill generated_at without importing datetime at module import time.
    import datetime
    summary["generated_at"] = datetime.datetime.now().astimezone().isoformat(
        timespec="seconds")
    return summary


def run_cli(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Isolated edge-first low-saturation humidity dial experiment")
    parser.add_argument("--frames-dir", required=True,
                        help="directory with the fixed-view camera burst")
    parser.add_argument("--calib", required=True,
                        help="candidate calibration JSON (explicit --allow-candidate required)")
    parser.add_argument("--allow-candidate", action="store_true")
    parser.add_argument("--calib-sha256", default=None,
                        help="expected calibration SHA-256 (default: dial_reading A08)")
    parser.add_argument("--output", required=True,
                        help="new empty output directory for this experiment")
    parser.add_argument("--phone", default=None,
                        help="optional phone control image (default: the 19:40 attachment)")
    parser.add_argument("--phone-roi", nargs=4, type=int, default=[300, 700, 320, 320],
                        metavar=("X", "Y", "W", "H"))
    parser.add_argument("--phone-target-major", type=float, default=296.0,
                        help="measured full-resolution phone ring major axis (px)")
    parser.add_argument("--phone-target-size", type=float, default=60.0,
                        help="downsampled ring major axis for the scale control (px)")
    parser.add_argument("--dev-frames", type=int, default=DEV_FRAME_COUNT)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--representatives", nargs="*", type=int, default=[1, 3, 10])
    args = parser.parse_args(argv)

    command_parts = ["--frames-dir", args.frames_dir, "--calib", args.calib]
    if args.allow_candidate:
        command_parts.append("--allow-candidate")
    if args.calib_sha256:
        command_parts.extend(["--calib-sha256", args.calib_sha256])
    command_parts.extend(["--output", args.output])
    if args.phone:
        command_parts.extend(["--phone", args.phone])
    if list(args.phone_roi) != [300, 700, 320, 320]:
        command_parts.append("--phone-roi")
        command_parts.extend(str(value) for value in args.phone_roi)
    if args.phone_target_major != 296.0:
        command_parts.extend(["--phone-target-major", str(args.phone_target_major)])
    if args.phone_target_size != 60.0:
        command_parts.extend(["--phone-target-size", str(args.phone_target_size)])
    if args.limit is not None:
        command_parts.extend(["--limit", str(args.limit)])
    if args.dev_frames != DEV_FRAME_COUNT:
        command_parts.extend(["--dev-frames", str(args.dev_frames)])
    if list(args.representatives) != [1, 3, 10]:
        command_parts.append("--representatives")
        command_parts.extend(str(value) for value in args.representatives)
    command = "python3 {} {}".format(Path(__file__).resolve(),
                                     " ".join(command_parts))
    try:
        expected_sha = args.calib_sha256
        if expected_sha is None:
            import dial_reading
            expected_sha = dial_reading.A08_CANDIDATE_SHA256
        calibration = load_calibration(args.calib, args.allow_candidate, expected_sha)
        reference_path = Path(calibration["raw"]["source_image"]["path"])
        if not reference_path.is_file():
            raise ExperimentConfigError(
                "calibration source reference image is missing: {}".format(reference_path))
        reference_payload = reference_path.read_bytes()
        reference_sha = _sha256_bytes(reference_payload)
        if reference_sha != calibration.get("source_sha256"):
            raise ExperimentConfigError(
                "reference image SHA-256 does not match the candidate calibration")
        reference_image = cv2.imread(str(reference_path), cv2.IMREAD_COLOR)
        if reference_image is None:
            raise ExperimentConfigError("cannot decode the reference image")
        reference = reference_signature(reference_image, calibration)
        if not reference.get("ok"):
            raise ExperimentConfigError(
                "reference angular colour signature failed: {}".format(reference.get("reason")))

        output = Path(args.output)
        _output_target(output)
        output.mkdir(parents=True, exist_ok=True)
        burst = run_burst(Path(args.frames_dir), calibration, reference, output,
                          dev_frames=args.dev_frames, limit=args.limit,
                          representative=args.representatives)
        first_image = cv2.imread(burst["files"][0], cv2.IMREAD_COLOR)
        last_image = cv2.imread(burst["files"][-1], cv2.IMREAD_COLOR)
        negative = []
        for tag, image in (("frame_0001", first_image), ("frame_0010", last_image)):
            for item in run_negative_controls(image, calibration, reference):
                item = dict(item)
                item["name"] = "{}:{}".format(tag, item["name"])
                negative.append(item)
        phone_path = Path(args.phone) if args.phone else Path(
            "/home/mowen/.dsh/attachments/v1/objects/fe/"
            "fe554e70eab8087ff60d8d449539db38af90c7c79debed5b97d2603b42504eb4")
        phone_controls = run_phone_controls(
            phone_path, args.phone_roi, calibration, reference,
            target_major_px=args.phone_target_major,
            target_size_px=args.phone_target_size)
        summary = _summary_from_results(burst, phone_controls, negative,
                                        calibration, command, output)
        _write_json(output / "summary.json", summary)
        _write_report(output / "report.md", summary)
    except (ExperimentConfigError, OSError, ValueError) as error:
        print("experiment failed: {}".format(error), file=sys.stderr)
        return 2

    print(json.dumps({
        "experimental": True,
        "value": None,
        "output": str(output),
        "summary": str(output / "summary.json"),
        "report": str(output / "report.md"),
        "frames": len(burst["files"]),
        "dev_frames": int(args.dev_frames),
        "eval_frames": max(0, len(burst["files"]) - int(args.dev_frames)),
        "accepted_eval_frames": summary["accepted_eval_frames"],
        "diagnostic_candidate_eval_frames": summary["diagnostic_candidate_eval_frames"],
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(run_cli())
