#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Joint needle-direction study over one fixed-view camera burst (isolated).

R5 accepted 1/10 frames at its per-frame 2 deg gate, but 8 evaluation frames
carry a thin paired-edge body.  This study does not average angles or humidity
values.  It reuses the R5 ring/roll/hub and ``_line_refine`` data, rebuilds real
grey-edge midpoints as ``p = hub + s*u(coarse_angle) + (coarse_offset+t)*n``,
maps ``q = R(-roll) * A * (p - ring_centre)`` into the normalised reference
plane, and fits one common direction with one free intercept per frame:
``min sum_i alpha_i sum_j w_ij rho(n.q_ij - b_i)``.  The final fitted centreline
angle is never used to rebuild ``p``.

The joint stability is conditional on the detected ring/roll.  The R5 roll
``uncertainty_deg`` is a 90%-prominence peak half-width (9-11 deg observed), not
a statistical error; it is kept as a conservative envelope and never divided by
sqrt(N).  Top-level ``value`` is always ``None``, every result is
``trusted: false``, and no MAE / +/-2 %RH / confidence interval is claimed.

Only the fixed 10-frame 1280x720 burst, the candidate calibration and its
reference image are read.  No camera, ROS, navigation, device, dependency,
commit or push action is performed.  The R5 module and its tests are read-only
and SHA-bound.
"""

import argparse
import csv
import hashlib
import itertools
import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
import dial_low_saturation_experiment as r5
from dial_math import lookup_value, wrap_deg
from dial_reading import load_calibration

TOOL_VERSION = "dial_burst_geometry_experiment.py 1 (joint centerline burst study)"
EXPERIMENT_ID = "joint-centerline-20261116"
R5_SHA = "7d773d04282dca8b4b8b5502a9e31b477b69623d52cea0d40d00fb1e45889435"
R5_TEST_SHA = "a4c2338d96c5c7069bda444face1d315b712abb22fd6c0494310cc717502777b"
CANDIDATE_SHA = "d7dccc3e6d80f24199405dce07a516fd119d4502724efa413983682d2e5df198"
DECLARED_ROI = (630, 220, 75, 65)
USED_ROI = (630, 210, 75, 75)
FRAME_SIZE = (1280, 720)
DEV_FRAMES = 2
MIN_FRAMES = 6
TOL_DEG = 2.0
BOOTSTRAP_ITERS = 200
BLOCK_LEN = 3
SEED = 20261116
BODY_REASONS = {"line_direction_ambiguous", "line_competing_peaks", "line_support_low",
                "line_body_not_straight", "line_width_inconsistent", "line_no_needle",
                "line_fit_failed", "line_geometry_invalid", "line_hub_missing",
                "line_direction_evidence_missing"}
CONDITIONAL_GATES = ("joint_loo_span_deg", "hub_perturbation_span_deg",
                     "frame_bootstrap_span_deg", "block_bootstrap_span_deg",
                     "sign_consistency", "unique_eval_frames", "body_evidence")


class ExperimentConfigError(ValueError):
    pass


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha_file(path: Path) -> str:
    return _sha(Path(path).read_bytes())


def verify_r5_binding() -> Dict:
    module = Path(r5.__file__).resolve()
    test = module.parents[1] / "tests" / "test_dial_low_saturation_experiment.py"
    record = {}
    for name, path, expected in (("r5_module", module, R5_SHA),
                                 ("r5_test", test, R5_TEST_SHA)):
        actual = _sha_file(path)
        record[name] = {"path": str(path), "sha256": actual, "expected": expected,
                        "match": actual == expected}
        if actual != expected:
            raise ExperimentConfigError("{} SHA-256 mismatch".format(name))
    if tuple(r5.DECLARED_ROI) != DECLARED_ROI or tuple(r5.EXPERIMENT_ROI) != USED_ROI:
        raise ExperimentConfigError("R5 ROI contract changed")
    return record


def _rot(angle_deg: float) -> np.ndarray:
    t = math.radians(float(angle_deg))
    return np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])


def reconstruct_pixels(s, t, angle_deg: float, coarse_offset: float, hub) -> np.ndarray:
    theta = math.radians(float(angle_deg))
    u = np.array([math.cos(theta), math.sin(theta)])
    n = np.array([-math.sin(theta), math.cos(theta)])
    s = np.asarray(s, float).reshape(-1, 1)
    cross = np.asarray(t, float).reshape(-1, 1)
    return np.asarray(hub, float).reshape(1, 2) + s * u + (float(coarse_offset) + cross) * n


def map_to_reference(points, center, matrix, roll_deg: float) -> np.ndarray:
    relative = np.asarray(points, float) - np.asarray(center, float)
    return (_rot(-float(roll_deg)) @ (np.asarray(matrix, float) @ relative.T)).T


def _unit(vector) -> np.ndarray:
    vector = np.asarray(vector, float)
    length = float(np.linalg.norm(vector))
    if not math.isfinite(length) or length <= 1e-12:
        raise ExperimentConfigError("degenerate direction")
    return vector / length


def _median(values, weights) -> float:
    values, weights = np.asarray(values, float), np.asarray(weights, float)
    order = np.argsort(values)
    values, weights = values[order], weights[order]
    cumulative = np.cumsum(weights)
    if cumulative[-1] <= 0:
        raise ExperimentConfigError("empty weights")
    return float(values[np.searchsorted(cumulative, 0.5 * cumulative[-1])])


def _scatter(q, weights) -> np.ndarray:
    q, weights = np.asarray(q, float), np.asarray(weights, float)
    weights = weights / weights.sum()
    centered = q - (weights[:, None] * q).sum(axis=0)
    return (centered * weights[:, None]).T @ centered


def _angle(vector) -> float:
    vector = np.asarray(vector, float)
    return float(math.degrees(math.atan2(vector[1], vector[0])) % 360.0)


def _signed(first: float, second: float) -> float:
    return float(((float(first) - float(second) + 180.0) % 360.0) - 180.0)


def _span(angles) -> float:
    values = np.sort(np.asarray(angles, float) % 360.0)
    if values.size < 2:
        return 0.0
    return float(360.0 - np.max(np.diff(np.r_[values, values[0] + 360.0])))


def fit_joint(frames: Sequence[Dict], reference_v=None, resolve_tips=False,
              iterations: int = 30) -> Dict:
    scatter = np.zeros((2, 2))
    for frame in frames:
        scatter += _scatter(frame["q"], frame["w"])
    normal = np.linalg.eigh(scatter)[1][:, 0]
    if normal[0] < 0:
        normal = -normal
    for _ in range(int(iterations)):
        intercepts = [_median(np.asarray(f["q"]) @ normal, f["w"]) for f in frames]
        residuals = np.concatenate([np.asarray(f["q"]) @ normal - b
                                    for f, b in zip(frames, intercepts)])
        scale = 1.4826 * float(np.median(np.abs(residuals))) + 1e-12
        scatter = np.zeros((2, 2))
        for f, b in zip(frames, intercepts):
            q = np.asarray(f["q"], float)
            huber = np.minimum(1.0, 1.345 * scale / (np.abs(q @ normal - b) + 1e-12))
            scatter += _scatter(q, np.asarray(f["w"], float) * huber)
        candidate = np.linalg.eigh(scatter)[1][:, 0]
        if np.dot(candidate, normal) < 0:
            candidate = -candidate
        change = float(np.max(np.abs(candidate - normal)))
        normal = candidate
        if change < 1e-12:
            break
    v = np.array([normal[1], -normal[0]])
    conflict = False
    if reference_v is not None:
        if np.dot(v, np.asarray(reference_v, float)) < 0:
            v, normal = -v, -normal
    elif resolve_tips:
        dots = [float(np.dot(f["tip"], v)) for f in frames if f.get("tip") is not None]
        if dots:
            if any(d > 0 for d in dots) and any(d < 0 for d in dots):
                conflict = True
            elif all(d < 0 for d in dots):
                v, normal = -v, -normal
    intercepts = [_median(np.asarray(f["q"]) @ normal, f["w"]) for f in frames]
    stats = []
    for f, b in zip(frames, intercepts):
        residual = np.asarray(f["q"]) @ normal - b
        weights = np.asarray(f["w"], float)
        weights = weights / weights.sum()
        stats.append((float(b), float(np.sqrt(np.sum(weights * residual * residual)))))
    return {"v": v, "normal": normal, "angle_deg": _angle(v), "b": intercepts,
            "stats": stats, "conflict": conflict}


def own_angle(frame: Dict, reference_v=None) -> float:
    v = np.linalg.eigh(_scatter(frame["q"], frame["w"]))[1][:, 1]
    if reference_v is not None and np.dot(v, np.asarray(reference_v, float)) < 0:
        v = -v
    return _angle(v)


def near_far(frames: Sequence[Dict], joint: Dict) -> Dict:
    near, far, per_frame = [], [], []
    for f in frames:
        s = np.asarray(f["s"], float)
        mask = s <= float(np.median(s))
        nf = {"q": np.asarray(f["q"])[mask], "w": np.asarray(f["w"])[mask]}
        ff = {"q": np.asarray(f["q"])[~mask], "w": np.asarray(f["w"])[~mask]}
        near.append(nf)
        far.append(ff)
        per_frame.append(_signed(own_angle(ff, joint["v"]), own_angle(nf, joint["v"])))
    nj, fj = fit_joint(near, reference_v=joint["v"]), fit_joint(far, reference_v=joint["v"])
    return {"near_deg": nj["angle_deg"], "far_deg": fj["angle_deg"],
            "difference_deg": abs(_signed(nj["angle_deg"], fj["angle_deg"])),
            "per_frame_difference_deg": per_frame,
            "consistent_sign": all(v >= 0 for v in per_frame) or all(v <= 0 for v in per_frame)}


def bootstrap_span(frames: Sequence[Dict], joint: Dict, iterations: int,
                   block: int, seed: int) -> Dict:
    count, minimum = len(frames), max(4, len(frames) // 2)
    rng = np.random.RandomState(int(seed))
    result = {}
    for mode in ("frame", "block"):
        angles, skipped = [], 0
        for _ in range(int(iterations)):
            indices = ([int(v) for v in rng.randint(0, count, count)] if mode == "frame"
                       else [])
            if mode == "block":
                while len(indices) < count:
                    start = int(rng.randint(0, count))
                    indices.extend(int((start + k) % count) for k in range(int(block)))
                indices = indices[:count]
            if len(set(indices)) < minimum:
                skipped += 1
                continue
            angles.append(fit_joint([frames[i] for i in indices],
                                    reference_v=joint["v"])["angle_deg"])
        result[mode] = {"samples": len(angles), "skipped": skipped,
                        "minimum_distinct": minimum,
                        "span_deg": _span(angles) if angles else None,
                        "method": "frame bootstrap (sensitivity, not CI)" if mode == "frame"
                                  else "contiguous-block bootstrap of one burst"}
    return result


def roll_span(frames: Sequence[Dict], joint: Dict) -> Dict:
    half = max(float(f["roll_uncertainty_deg"]) for f in frames)
    common = []
    for delta in (-half, 0.0, half):
        rotated = [{"q": (_rot(-delta) @ np.asarray(f["q"]).T).T, "w": f["w"]}
                   for f in frames]
        common.append(fit_joint(rotated, reference_v=joint["v"])["angle_deg"])
    count = len(frames)
    patterns = (list(itertools.product((-1, 1), repeat=count)) if count <= 10 else
                [(1,) * count, (-1,) * count])
    values = []
    for pattern in patterns:
        rotated = [{"q": (_rot(-s * float(f["roll_uncertainty_deg"])) @ np.asarray(f["q"]).T).T,
                    "w": f["w"]} for f, s in zip(frames, pattern)]
        values.append(fit_joint(rotated, reference_v=joint["v"])["angle_deg"])
    return {"common_halfwidth_deg": half, "common_span_deg": _span(common),
            "differential_span_deg": _span(values),
            "differential_min_deg": min(values), "differential_max_deg": max(values),
            "not_reduced_by_n": True}


def _point_gates(refine: Optional[Dict], line: Optional[Dict], semi: float) -> List[str]:
    if refine is None:
        return ["line_refine_failed"]
    reasons = []
    if len(refine.get("samples", [])) < r5.LINE_MIN_SAMPLES:
        reasons.append("samples")
    if float(refine.get("support_frac", 0)) < r5.LINE_MIN_SUPPORT_FRAC:
        reasons.append("support")
    if float(refine.get("centerline_rms_px", 1e9)) > r5.LINE_MAX_CENTERLINE_RMS_PX:
        reasons.append("rms")
    width = refine.get("width_rms_px")
    if width is None or float(width) > r5.LINE_MAX_WIDTH_RMS_PX:
        reasons.append("width")
    if line is not None:
        if float(line.get("tip_tail_ratio") or 0) < r5.LINE_TIP_TAIL_RATIO:
            reasons.append("tip_tail")
        if float(line.get("tip_extent_px") or 0) < max(4.0, r5.LINE_MIN_TIP_LENGTH_FRAC * semi):
            reasons.append("tip_short")
        if float(line.get("tail_extent_px") or 0) >= r5.LINE_MIN_TIP_LENGTH_FRAC * semi:
            reasons.append("tail_long")
        if float(line.get("profile_second_ratio") or 0) >= r5.LINE_SECOND_PEAK_RATIO:
            reasons.append("second_peak")
    return reasons


def extract_points(crop, ring, hub, line, roi, roll_deg) -> Tuple[Optional[Dict], Optional[str]]:
    if not ring.get("center_px") or not ring.get("axes_px"):
        return None, "ring_missing"
    if not hub.get("center"):
        return None, "hub_missing"
    if line.get("angle_deg") is None or line.get("coarse_angle_deg") is None or line.get("coarse_offset_px") is None:
        return None, "line_missing"
    x0, y0 = int(roi[0]), int(roi[1])
    centre = np.asarray(ring["center_px"], float) - [x0, y0]
    axes = (float(ring["axes_px"][0]), float(ring["axes_px"][1]))
    matrix, _, _, _ = r5._ellipse_matrix(axes, float(ring["angle_deg"]))
    semi = 0.5 * max(axes)
    maximum, half, step = (max(r5.LINE_MAX_EDGE_SEPARATION_PX, 0.06 * semi),
                           max(r5.LINE_CROSS_HALF_PX, 0.05 * semi),
                           max(r5.LINE_CROSS_STEP_PX, 0.05 * semi / 8.0))
    hub_px = np.asarray(hub["center"], float)
    cap = r5._line_cap_radius_px(hub)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY).astype(np.float32)
    refine = r5._line_refine(gray, centre, matrix, hub_px, semi,
                             max(cap, 0.05 * semi) + 2.0,
                             float(line["coarse_angle_deg"]),
                             float(line["coarse_offset_px"]),
                             max_separation_px=maximum, cross_half_px=half,
                             cross_step_px=step)
    reasons = _point_gates(refine, line, semi)
    if reasons:
        return None, "point_gate:" + ",".join(reasons)
    s, t = np.asarray(refine["samples"], float), np.asarray(refine["center"], float)
    pixel = reconstruct_pixels(s, t, refine["coarse_angle_deg"],
                               refine["coarse_offset_px"], hub_px)
    q = map_to_reference(pixel, centre, matrix, roll_deg)
    if not (np.all(np.isfinite(pixel)) and np.all(np.isfinite(q))):
        return None, "non_finite_points"
    return {"s": s, "t": t, "w": np.asarray(refine["weights"], float),
            "pixel": pixel, "q": q, "refine": refine, "matrix": matrix,
            "center": centre, "axes": axes, "ring_angle": float(ring["angle_deg"]),
            "cap_radius": cap, "hub": hub_px, "gray": gray, "roi": roi,
            "coarse_angle": float(refine["coarse_angle_deg"]),
            "coarse_offset": float(refine["coarse_offset_px"])}, None


def collect_frame(image, path, index, split, calibration, reference) -> Dict:
    height, width = image.shape[:2]
    record = {"index": index, "split": split, "file": str(path),
              "frame_sha256": _sha(path.read_bytes()), "width": width, "height": height,
              "included": False, "exclusion_reason": None, "points": None,
              "body_gate": None, "single_frame_status": None, "single_frame_reason": None}
    if (width, height) != FRAME_SIZE:
        record["exclusion_reason"] = "non_camera_frame_size"
        return record
    x0, y0, w0, h0 = USED_ROI
    crop = image[y0:y0 + h0, x0:x0 + w0]
    record["crop_sha256"] = _sha(crop.tobytes())
    result = r5.estimate_experiment_frame(image, calibration, DECLARED_ROI, reference, stage=split)
    ring, roll, line = result.get("ring") or {}, result.get("roll") or {}, result.get("line") or {}
    record.update({
        "ring": {k: ring.get(k) for k in ("ok", "reason", "center_px", "axes_px", "angle_deg", "hub")},
        "roll": {k: roll.get(k) for k in ("ok", "reason", "roll_deg", "uncertainty_deg")},
        "line": {k: line.get(k) for k in ("ok", "reason", "angle_deg", "angle_px_deg", "angle_span_deg",
                                          "coarse_angle_deg", "coarse_offset_px", "tip_extent_px",
                                          "tail_extent_px", "tip_tail_ratio", "profile_second_ratio")},
        "single_frame_status": result.get("frame_status"),
        "single_frame_reason": result.get("reason")})
    for stage, payload in (("ring", ring), ("roll", roll)):
        if not payload.get("ok"):
            record["exclusion_reason"] = "{}:{}".format(stage, payload.get("reason"))
            return record
    hub = ring.get("hub") or {}
    if hub.get("center") is None:
        record["exclusion_reason"] = "hub_missing"
        return record
    if line.get("angle_deg") is None or line.get("reason") in BODY_REASONS:
        record["exclusion_reason"] = "line:" + str(line.get("reason"))
        return record
    extracted, error = extract_points(crop, ring, hub, line, USED_ROI, float(roll["roll_deg"]))
    if extracted is None:
        record.update({"exclusion_reason": error, "body_gate": False})
        return record
    angle = line.get("angle_px_deg", line.get("angle_deg", 0.0))
    pixel = [math.cos(math.radians(float(angle))), math.sin(math.radians(float(angle)))]
    extracted.update({"tip": _unit(_rot(-float(roll["roll_deg"])) @
                                   (extracted["matrix"] @ np.asarray(pixel, float))),
                      "roll_deg": float(roll["roll_deg"]),
                      "roll_uncertainty_deg": float(roll["uncertainty_deg"]),
                      "hub_ambiguous": bool(hub.get("ambiguous", False)),
                      "n_points": len(extracted["s"])})
    record.update({"included": True, "body_gate": True, "points": extracted,
                   "included_despite_interval": line.get("reason") == "line_direction_unstable"})
    return record


def dedupe(records: Sequence[Dict]) -> Dict:
    groups, first = {}, {}
    for record in records:
        digest = record.get("crop_sha256")
        if digest is None:
            continue
        if digest in first:
            record.update({"included": False,
                           "exclusion_reason": "duplicate_roi_of_frame_{}".format(first[digest])})
            groups.setdefault(digest, [first[digest]]).append(record["index"])
        else:
            first[digest] = record["index"]
    return {"unique": len(first),
            "groups": [{"crop_sha256": k, "frames": v} for k, v in groups.items()]}


def perturbation(frames: Sequence[Dict], joint: Dict) -> Dict:
    specs = []
    for axis in (0, 1):
        for sign in (1.0, -1.0):
            specs.append(("hub_{}{}".format("xy"[axis], "+" if sign > 0 else "-"),
                          "hub", axis, sign))
    for axis in (0, 1):
        for sign in (1.0, -1.0):
            specs.append(("ring_axis{}_{}".format(axis, "+" if sign > 0 else "-"),
                          "axis", axis, sign))
    for sign in (1.0, -1.0):
        specs.append(("ring_angle_{}".format("+" if sign > 0 else "-"), "angle", 0, sign))
    results, failures = [], []
    for name, kind, axis, sign in specs:
        fit_frames = []
        for frame in frames:
            p = frame["points"]
            hub, centre = np.asarray(p["hub"], float).copy(), np.asarray(p["center"], float).copy()
            axes, ring_angle = list(p["axes"]), float(p["ring_angle"])
            if kind == "hub":
                hub[axis] += 1.5 * sign
            elif kind == "axis":
                axes[axis] *= 1.0 + 0.01 * sign
            else:
                ring_angle += sign
            matrix, _, _, _ = r5._ellipse_matrix((axes[0], axes[1]), ring_angle)
            semi = 0.5 * max(axes)
            maximum, half, step = (max(r5.LINE_MAX_EDGE_SEPARATION_PX, 0.06 * semi),
                                   max(r5.LINE_CROSS_HALF_PX, 0.05 * semi),
                                   max(r5.LINE_CROSS_STEP_PX, 0.05 * semi / 8.0))
            refine = r5._line_refine(p["gray"], centre, matrix, hub, semi,
                                     max(p["cap_radius"], 0.05 * semi) + 2.0,
                                     p["coarse_angle"], p["coarse_offset"],
                                     max_separation_px=maximum, cross_half_px=half,
                                     cross_step_px=step)
            reasons = _point_gates(refine, None, semi)
            if reasons:
                failures.append({"name": name, "frame": frame["index"], "reason": ",".join(reasons)})
                fit_frames = []
                break
            pixel = reconstruct_pixels(refine["samples"], refine["center"],
                                       refine["coarse_angle_deg"],
                                       refine["coarse_offset_px"], hub)
            fit_frames.append({"q": map_to_reference(pixel, centre, matrix, p["roll_deg"]),
                               "w": np.asarray(refine["weights"], float)})
        if fit_frames and len(fit_frames) == len(frames):
            fit = fit_joint(fit_frames, reference_v=joint["v"])
            results.append((name, fit["angle_deg"], _signed(fit["angle_deg"], joint["angle_deg"])))
    angles = [a for _, a, _ in results]
    return {"expected": len(specs), "valid": len(results),
            "complete": len(results) == len(specs) and not failures,
            "span_deg": _span(angles) if angles else None,
            "max_abs_delta_deg": max(abs(d) for _, _, d in results) if results else None,
            "failures": failures}


def _compact(record: Dict) -> Dict:
    points = record.get("points")
    compact_points = None
    if points:
        compact_points = {k: points[k] for k in
                          ("s", "t", "w", "pixel", "q", "tip", "roll_deg", "roll_uncertainty_deg",
                           "hub_ambiguous", "n_points", "axes", "matrix", "center", "cap_radius",
                           "hub", "ring_angle", "coarse_angle", "coarse_offset")}
        compact_points.update({k: points["refine"].get(k) for k in
                               ("support_frac", "centerline_rms_px", "width_rms_px")})
    compact = {k: record.get(k) for k in
               ("index", "split", "file", "frame_sha256", "crop_sha256", "width", "height",
                "included", "exclusion_reason", "body_gate", "single_frame_status",
                "single_frame_reason", "included_despite_interval")}
    compact.update({"ring": record.get("ring"), "roll": record.get("roll"),
                    "line": record.get("line"), "points": compact_points})
    return compact


def _json(path: Path, payload) -> None:
    temp = path.with_name(".{}-{}.json".format(path.stem, os.getpid()))
    temp.write_text(json.dumps(payload, ensure_ascii=False, indent=1, default=_json_default) + "\n",
                    encoding="utf-8")
    os.replace(str(temp), str(path))


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError("not JSON serializable: {}".format(type(value)))


def _csv(path: Path, rows: Sequence[Dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = []
    for row in rows:
        fields.extend(k for k in row if k not in fields)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def ensure_output(path: Path) -> Path:
    path = Path(path)
    if path.exists():
        if not path.is_dir():
            raise ExperimentConfigError("output path is not a directory")
        if any(path.iterdir()):
            raise ExperimentConfigError("output directory is not empty; refusing to overwrite")
    return path


def frame_row(record: Dict, joint: Optional[Dict]) -> Dict:
    points, line, roll = record.get("points") or {}, record.get("line") or {}, record.get("roll") or {}
    row = {"index": record["index"], "split": record["split"], "file": record["file"],
           "sha256": record["frame_sha256"], "width": record["width"],
           "height": record["height"], "included": record["included"],
           "exclusion_reason": record.get("exclusion_reason") or "",
           "single_frame_status": record.get("single_frame_status"),
           "single_frame_reason": record.get("single_frame_reason"),
           "body_gate": record.get("body_gate"),
           "ring_ok": bool((record.get("ring") or {}).get("ok")),
           "roll_deg": roll.get("roll_deg"),
           "roll_uncertainty_deg": roll.get("uncertainty_deg"),
           "line_angle_deg": line.get("angle_deg"),
           "line_span_deg": line.get("angle_span_deg"),
           "support_frac": points.get("support_frac"),
           "centerline_rms_px": points.get("centerline_rms_px"),
           "width_rms_px": points.get("width_rms_px"),
           "n_points": len(points.get("s", [])),
           "hub_ambiguous": bool(points.get("hub_ambiguous", False)),
           "included_despite_interval": bool(record.get("included_despite_interval", False))}
    if joint and record["included"] and record["index"] in joint["frames"]:
        index = joint["frames"].index(record["index"])
        row.update({"own_angle_deg": joint["frame_direction"]["own_deg"][index],
                    "deviation_deg": joint["frame_direction"]["deviation_deg"][index],
                    "intercept_norm": joint["b_norm"][index],
                    "point_rms_norm": joint["frame_rms_norm"][index]})
    return row


def point_rows(record: Dict, joint: Optional[Dict]) -> List[Dict]:
    points = record.get("points")
    if not points:
        return []
    rows = []
    for i in range(len(points["s"])):
        row = {"frame": record["index"], "split": record["split"],
               "included": record["included"], "point": i,
               "s_px": round(float(points["s"][i]), 4),
               "t_px": round(float(points["t"][i]), 4),
               "weight": round(float(points["w"][i]), 6),
               "pixel_x": round(float(points["pixel"][i][0] + USED_ROI[0]), 4),
               "pixel_y": round(float(points["pixel"][i][1] + USED_ROI[1]), 4),
               "q_x": round(float(points["q"][i][0]), 6),
               "q_y": round(float(points["q"][i][1]), 6)}
        if joint and record["included"] and record["index"] in joint["frames"]:
            index = joint["frames"].index(record["index"])
            row["residual_norm"] = round(
                float(np.dot(points["q"][i], joint["normal"])) - joint["b_norm"][index], 6)
        rows.append(row)
    return rows


def _f(value, digits=2):
    return "n/a" if value is None else "{:.{}f}".format(float(value), digits)


def overlay(image, record: Dict, joint: Optional[Dict]) -> np.ndarray:
    canvas = image.copy()
    x0, y0, w0, h0 = USED_ROI
    cv2.rectangle(canvas, (x0, y0), (x0 + w0, y0 + h0), (0, 255, 255), 2)
    ring = record.get("ring") or {}
    if ring.get("center_px") and ring.get("axes_px"):
        cv2.ellipse(canvas, (int(ring["center_px"][0]), int(ring["center_px"][1])),
                    (int(ring["axes_px"][0] / 2), int(ring["axes_px"][1] / 2)),
                    float(ring.get("angle_deg", 0)), 0, 360, (0, 255, 0), 1)
    points = record.get("points") or {}
    if points:
        for pixel in points["pixel"]:
            cv2.circle(canvas, (int(pixel[0] + x0), int(pixel[1] + y0)), 1, (0, 255, 0), -1)
        anchor = np.mean(points["pixel"], axis=0) + [x0, y0]
        if joint:
            matrix, length = np.asarray(points["matrix"], float), 1.25 * 0.5 * max(points["axes"])
            direction = _unit(np.linalg.inv(matrix) @ (_rot(points["roll_deg"]) @ joint["v"]))
            cv2.arrowedLine(canvas, (int(anchor[0]), int(anchor[1])),
                            (int(anchor[0] + length * direction[0]),
                             int(anchor[1] + length * direction[1])), (0, 0, 255), 1, tipLength=0.08)
            own = own_angle({"q": points["q"], "w": points["w"]}, joint["v"])
            own = _unit(np.linalg.inv(matrix) @ (_rot(points["roll_deg"]) @
                                                 np.array([math.cos(math.radians(own)),
                                                           math.sin(math.radians(own))])))
            cv2.line(canvas, (int(anchor[0]), int(anchor[1])),
                     (int(anchor[0] + length * own[0]), int(anchor[1] + length * own[1])),
                     (255, 0, 255), 1, cv2.LINE_AA)
    lines = ["experimental=true  value=null  trusted=false",
             "frame {}  split={}  included={}  reason={}".format(
                 record["index"], record["split"], record["included"],
                 record.get("exclusion_reason") or record.get("single_frame_reason")),
             "roll={} +/-{}  line_span={} deg".format(
                 _f((record.get("roll") or {}).get("roll_deg")),
                 _f((record.get("roll") or {}).get("uncertainty_deg"), 1),
                 _f((record.get("line") or {}).get("angle_span_deg"))),
             "points={}  joint_conditional={} deg".format(
                 len(points.get("s", [])), "n/a" if not joint else _f(joint["angle_deg"], 3)),
             "LOO={}  near/far={}  frame_range={}".format(
                 "n/a" if not joint else _f(joint["loo_span_deg"]),
                 "n/a" if not joint else _f(joint["near_far"]["difference_deg"]),
                 "n/a" if not joint else _f(joint["frame_direction"]["range_deg"])),
             "green=points red=joint magenta=frame own"]
    if joint and record["included"] and record["index"] in joint["frames"]:
        index = joint["frames"].index(record["index"])
        lines.append("own={} deg  dev={} deg  b={} norm".format(
            _f(joint["frame_direction"]["own_deg"][index], 3),
            _f(joint["frame_direction"]["deviation_deg"][index], 3),
            _f(joint["b_norm"][index], 4)))
    shade = canvas.copy()
    cv2.rectangle(shade, (8, 8), (800, 8 + 22 * len(lines) + 8), (0, 0, 0), -1)
    canvas = cv2.addWeighted(shade, 0.55, canvas, 0.45, 0)
    for row, text in enumerate(lines):
        cv2.putText(canvas, text, (14, 30 + 22 * row), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (255, 255, 255), 1, cv2.LINE_AA)
    patch = canvas[y0:y0 + h0, x0:x0 + w0]
    if patch.size:
        inset = cv2.resize(patch, (w0 * 4, h0 * 4), interpolation=cv2.INTER_NEAREST)
        top = max(0, canvas.shape[0] - inset.shape[0] - 28)
        left = max(0, canvas.shape[1] - inset.shape[1] - 10)
        canvas[top:top + inset.shape[0], left:left + inset.shape[1]] = inset
        cv2.rectangle(canvas, (left, top), (left + inset.shape[1], top + inset.shape[0]),
                      (255, 255, 255), 1)
        cv2.putText(canvas, "ROI x4 nearest (review only)", (left + 4, max(12, top - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def report(summary: Dict) -> str:
    joint = summary.get("joint") or {}
    lines = ["# 联合针方向实验（单 burst，条件稳定与整体误差分开）", "", "## 结论",
             "- 联合条件方向 {} deg（归一化参考平面，8 个评估帧，一个 burst）。".format(
                 _f(joint.get("angle_deg"), 3)),
             "- 条件稳定性：LOO {} deg，扰动 {} deg；2 deg 预算见门表。".format(
                 _f(joint.get("loo_span_deg")),
                 _f((joint.get("perturbation") or {}).get("span_deg"))),
             "- **近/远段一致偏弯 {} deg**（逐帧同号：{}），超过 2 deg；多帧平均没有消掉它。".format(
                 _f((joint.get("near_far") or {}).get("difference_deg")),
                 (joint.get("near_far") or {}).get("consistent_sign")),
             "- 因此不把联合方向当作已接受的整体针方向，也不宣称整体 RH 改善；value=null、trusted=false。",
             "- 整体 roll/参考误差：R5 粗略峰半宽 9-11 deg 原样保留；公共包络 {} deg，差分包络 {} deg，不随帧数缩水。".format(
                 _f((joint.get("roll") or {}).get("common_span_deg")),
                 _f((joint.get("roll") or {}).get("differential_span_deg"))),
             "", "## 口径",
             "- 只回答“以检测环/roll 为条件的联合针方向是否稳定”，不回答整体 RH 可信度。",
             "- R5 的 roll uncertainty_deg 是相关峰 90% prominence 左半宽，未经校准，不是统计误差；不能除以 sqrt(N)。",
             "- 候选 73.60 不是真值；联合方向换算的 {} 仅为 diagnostic，不作目标、不作 MAE。".format(
                 _f(joint.get("diagnostic_rh_not_truth"))),
             "- 同一 burst、已见数据；不是 8 个独立场景，也不是 8/8 帧通过单帧 2 deg 门。",
             "", "## 输入与证据",
             "- 帧目录：`{}`；候选标定：`{}`。".format(
                 summary["input"]["frames_dir"], summary["input"]["calibration_path"]),
             "- 10 帧，1280x720；唯一 decoded ROI hash {} 个，重复组 {}。".format(
                 summary["input"]["unique_crop_hashes"], len(summary["input"]["duplicate_groups"])),
             "- 评估帧使用 {} 个；逐帧排除原因见 frames.csv。".format(
                 summary["input"]["included_eval_frames"]),
             "", "## 方法",
             "- R5 只提供环、roll、粗 hub、coarse angle/offset；本模块用 _line_refine 重新提取真实灰度边缘中点 s/t/weights。",
             "- p = hub + s*u(coarse) + (coarse_offset+t)*n(coarse)；q = R(-roll)*A*(p-ring_centre)。不用最终中心线角重建 p。",
             "- 联合模型：每帧一个截距 b_i、共同 v/n；逐帧点权归一化并等帧权；Huber 迭代；180 冲突由针尖/短尾支持消歧。",
             "- 2 deg 稳定预算全部在归一化参考平面比较；bootstrap 只作有限样本敏感性，不是置信区间。",
             "", "## 逐帧", "|帧|split|included|roll|line|n|own|dev|排除/说明|",
             "|---:|---|---|---:|---:|---:|---:|---:|---|"]
    for record in summary["frames"]:
        points, line, roll = record.get("points") or {}, record.get("line") or {}, record.get("roll") or {}
        own = dev = "n/a"
        if joint and record["included"] and record["index"] in joint["frames"]:
            i = joint["frames"].index(record["index"])
            own, dev = _f(joint["frame_direction"]["own_deg"][i]), _f(
                joint["frame_direction"]["deviation_deg"][i])
        lines.append("|{}|{}|{}|{}|{}|{}|{}|{}|{}|".format(
            record["index"], record["split"], record["included"],
            _f(roll.get("roll_deg")), _f(line.get("angle_deg")),
            len(points.get("s", [])), own, dev, record.get("exclusion_reason") or ""))
    lines += ["", "## 联合指标",
              "- 条件方向 {} deg；b_norm {}..{}；逐帧点残差 {}..{} norm。".format(
                  _f(joint.get("angle_deg"), 3), _f(min(joint.get("b_norm") or [0])),
                  _f(max(joint.get("b_norm") or [0])),
                  _f(min(joint.get("frame_rms_norm") or [0]), 4),
                  _f(max(joint.get("frame_rms_norm") or [0]), 4)),
              "- LOO {}；帧 bootstrap {}（{} 次）；块 bootstrap {}（{} 次）。".format(
                  _f(joint.get("loo_span_deg")),
                  _f((joint.get("bootstrap") or {}).get("frame", {}).get("span_deg")),
                  (joint.get("bootstrap") or {}).get("frame", {}).get("samples"),
                  _f((joint.get("bootstrap") or {}).get("block", {}).get("span_deg")),
                  (joint.get("bootstrap") or {}).get("block", {}).get("samples")),
              "- 近 {} / 远 {} / 差 {}；逐帧差 {}。".format(
                  _f((joint.get("near_far") or {}).get("near_deg")),
                  _f((joint.get("near_far") or {}).get("far_deg")),
                  _f((joint.get("near_far") or {}).get("difference_deg")),
                  [round(v, 2) for v in (joint.get("near_far") or {}).get("per_frame_difference_deg", [])]),
              "- 帧 own 范围 {}；最大 |dev| {}。".format(
                  _f((joint.get("frame_direction") or {}).get("range_deg")),
                  _f((joint.get("frame_direction") or {}).get("max_abs_deviation_deg"))),
              "- 扰动：valid={}/{}，complete={}，span {}，max|delta| {}；失败项 {}。".format(
                  (joint.get("perturbation") or {}).get("valid"),
                  (joint.get("perturbation") or {}).get("expected"),
                  (joint.get("perturbation") or {}).get("complete"),
                  _f((joint.get("perturbation") or {}).get("span_deg")),
                  _f((joint.get("perturbation") or {}).get("max_abs_delta_deg")),
                  (joint.get("perturbation") or {}).get("failures")),
              "", "## 门"]
    for gate in summary.get("gates", []):
        lines.append("- {}: value={} threshold={} pass={} role={}".format(
            gate["name"], _f(gate["value"], 4), gate["threshold"], gate["pass"], gate["role"]))
    lines += ["", "## 判定",
              "- accepted_conditional_stability={}；accepted_common_needle_direction={}；accepted_overall={}。".format(
                  summary["conclusion"]["accepted_conditional_stability"],
                  summary["conclusion"]["accepted_common_needle_direction"],
                  summary["conclusion"]["accepted_overall"]),
              "- 结论：{}".format(summary["conclusion"]["reason"]),
              "- 本轮不扩模型、不改阈值；失败也停止，交 Lead 独立复审。",
              "", "## 验证",
              "- 合成真值/负例/坐标契约由 `src/photo_capture/tests/test_dial_burst_geometry_experiment.py` 覆盖；"
              "CLI 不替代测试；有效针正例单独计数，不把有效针拒识算 TN。",
              "- R5 sha {}；R5 测试 sha {}；候选 sha {}。".format(R5_SHA, R5_TEST_SHA, CANDIDATE_SHA)]
    return "\n".join(lines) + "\n"


def write_outputs(output: Path, summary: Dict, overlays: Sequence[Tuple]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    (output / "overlays").mkdir(exist_ok=True)
    _json(output / "summary.json", summary)
    _csv(output / "frames.csv", [frame_row(r, summary.get("joint")) for r in summary["frames"]])
    rows = []
    for record in summary["frames"]:
        rows.extend(point_rows(record, summary.get("joint")))
    _csv(output / "points.csv", rows)
    (output / "report.md").write_text(report(summary), encoding="utf-8")
    for index, image, record, joint in overlays:
        path = output / "overlays" / "overlay_{:04d}.png".format(index)
        temp = path.with_name(".{}-{}.png".format(path.stem, os.getpid()))
        if not cv2.imwrite(str(temp), overlay(image, record, joint)):
            raise ExperimentConfigError("overlay write failed")
        os.replace(str(temp), str(path))


def _gate(name, value, threshold, passed, role) -> Dict:
    return {"name": name, "value": value, "threshold": threshold, "pass": bool(passed), "role": role}


def _finite_le(value, limit: float) -> bool:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number) and number <= float(limit)


def perturbation_complete(perturbation: Optional[Dict]) -> bool:
    if not perturbation:
        return False
    try:
        expected = int(perturbation.get("expected"))
        valid = int(perturbation.get("valid"))
    except (TypeError, ValueError):
        return False
    return (expected > 0 and valid == expected
            and perturbation.get("complete") is True
            and not (perturbation.get("failures") or []))


def conditional_stability(gate_list: Sequence[Dict]) -> Dict:
    by_name = {gate["name"]: gate for gate in gate_list}
    failed = [name for name in CONDITIONAL_GATES
              if name not in by_name or not by_name[name].get("pass")]
    return {"accepted": not failed, "failed": failed}


def gates(summary: Dict) -> List[Dict]:
    joint = summary.get("joint") or {}
    evidence = summary["input"]["included_eval_frames"]
    result = [_gate("unique_eval_frames", evidence, ">=6", evidence >= MIN_FRAMES, "accept"),
              _gate("body_evidence", summary["input"]["body_gate_passed"], "all included",
                    bool(summary["input"]["body_gate_passed"]), "accept"),
              _gate("sign_consistency", not bool(joint.get("sign_conflict", True)),
                    "no 180 conflict", not bool(joint.get("sign_conflict", True)), "accept")]
    if joint:
        near = joint["near_far"]["difference_deg"]
        perturbation = joint.get("perturbation") or {}
        frame_bootstrap = joint["bootstrap"]["frame"]["span_deg"]
        block_bootstrap = joint["bootstrap"]["block"]["span_deg"]
        result += [
            _gate("joint_loo_span_deg", joint["loo_span_deg"], "<=2",
                  joint["loo_span_deg"] is not None and joint["loo_span_deg"] <= TOL_DEG, "accept"),
            _gate("near_far_difference_deg", near, "<=2", near <= TOL_DEG, "accept"),
            _gate("frame_max_deviation_deg", joint["frame_direction"]["max_abs_deviation_deg"],
                  "<=2", joint["frame_direction"]["max_abs_deviation_deg"] <= TOL_DEG, "accept"),
            _gate("hub_perturbation_span_deg", perturbation.get("span_deg"),
                  "<=2 and complete {0}/{0}, no failures".format(
                      perturbation.get("expected")),
                  perturbation_complete(perturbation)
                  and _finite_le(perturbation.get("span_deg"), TOL_DEG),
                  "accept"),
            _gate("frame_bootstrap_span_deg", frame_bootstrap, "<=2",
                  _finite_le(frame_bootstrap, TOL_DEG), "sensitivity"),
            _gate("block_bootstrap_span_deg", block_bootstrap, "<=2",
                  _finite_le(block_bootstrap, TOL_DEG), "sensitivity"),
            _gate("frame_direction_range_deg", joint["frame_direction"]["range_deg"], "<=2",
                  joint["frame_direction"]["range_deg"] <= TOL_DEG, "diagnostic")]
    return result


def run_study(frames_dir: Path, calibration: Dict, reference: Dict, output: Path,
              calibration_path: str = "", dev_frames: int = DEV_FRAMES,
              limit: Optional[int] = None, bootstrap_iterations: int = BOOTSTRAP_ITERS,
              seed: int = SEED) -> Dict:
    if not Path(frames_dir).is_dir():
        raise ExperimentConfigError("frames directory not found")
    files = sorted(p for p in Path(frames_dir).iterdir()
                   if p.is_file() and p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    if not files:
        raise ExperimentConfigError("no image frames found")
    files = files[:int(limit)] if limit else files
    records, images = [], {}
    for index, path in enumerate(files, start=1):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise ExperimentConfigError("cannot decode image: {}".format(path))
        records.append(collect_frame(image, path, index,
                                     "dev" if index <= int(dev_frames) else "eval",
                                     calibration, reference))
        images[index] = image
    dedupe_info = dedupe(records)
    included = [r for r in records if r["split"] == "eval" and r["included"]]
    frames = [{"index": r["index"], "q": r["points"]["q"], "w": r["points"]["w"],
               "s": r["points"]["s"], "tip": r["points"]["tip"],
               "roll_uncertainty_deg": r["points"]["roll_uncertainty_deg"],
               "points": r["points"]} for r in included]
    joint = None
    if len(frames) >= MIN_FRAMES:
        fit = fit_joint(frames, resolve_tips=True)
        loo = [fit_joint([f for j, f in enumerate(frames) if j != i],
                         reference_v=fit["v"])["angle_deg"] for i in range(len(frames))]
        near_far_result = near_far(frames, fit)
        own = [own_angle(f, fit["v"]) for f in frames]
        deviations = [_signed(a, fit["angle_deg"]) for a in own]
        joint = {
            "method": "per-frame intercept + robust shared normal in normalized reference plane",
            "frames": [r["index"] for r in included],
            "angle_deg": fit["angle_deg"], "normal": fit["normal"].tolist(),
            "v": fit["v"].tolist(), "sign_conflict": bool(fit["conflict"]),
            "b_norm": [float(v) for v in fit["b"]],
            "frame_rms_norm": [float(s[1]) for s in fit["stats"]],
            "loo_span_deg": _span(loo), "loo_angles_deg": loo,
            "near_far": near_far_result,
            "frame_direction": {"own_deg": own, "deviation_deg": deviations,
                                "max_abs_deviation_deg": max(abs(v) for v in deviations),
                                "range_deg": _span(own)},
            "bootstrap": bootstrap_span(frames, fit, bootstrap_iterations, BLOCK_LEN, seed),
            "roll": roll_span(frames, fit), "perturbation": perturbation(frames, fit),
            "diagnostic_rh_not_truth": lookup_value(
                float(calibration["table_deg"][0])
                + wrap_deg(fit["angle_deg"] - float(calibration["table_deg"][0])),
                list(calibration["table_deg"]), list(calibration["table_val"])),
            "diagnostic_not_truth": True, "conditional_only": True,
            "independent_verification": False}
    summary = {
        "experimental": True, "value": None, "trusted": False,
        "experiment_id": EXPERIMENT_ID, "tool_version": TOOL_VERSION,
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "scope": "offline fixed-view single-burst geometry experiment; no camera/ROS/"
                 "navigation/device change; no commit/push; input read-only",
        "model_correction": {"confirmed_model": "No.9011",
                             "candidate_metadata_model": "deli No.0011 (historical misrecord)",
                             "candidate_sha256": CANDIDATE_SHA,
                             "action": "report-only correction; no candidate or hash changed"},
        "r5_binding": verify_r5_binding(),
        "input": {"frames_dir": str(frames_dir), "calibration_path": calibration_path,
                  "calibration_sha256": calibration.get("calib_sha256"),
                  "reference_sha256": calibration.get("source_sha256"),
                  "unique_crop_hashes": dedupe_info["unique"],
                  "duplicate_groups": dedupe_info["groups"],
                  "included_eval_frames": len(included),
                  "body_gate_passed": bool(included) and all(
                      r.get("body_gate") for r in records
                      if r["split"] == "eval" and r.get("body_gate") is not None),
                  "camera_frame_size": list(FRAME_SIZE), "burst_only": True,
                  "independent_scenes": False,
                  "phone_or_non_camera_frames_rejected": [
                      r["index"] for r in records if r["exclusion_reason"] == "non_camera_frame_size"],
                  "seen_data_note": "R5 baseline and this burst were already inspected; "
                                    "parameters were fixed before this joint fit, not a blind holdout"},
        "parameters": {"declared_roi": list(DECLARED_ROI), "used_roi": list(USED_ROI),
                       "dev_frames": int(dev_frames), "min_eval_frames": MIN_FRAMES,
                       "tol_deg": TOL_DEG, "bootstrap_iterations": int(bootstrap_iterations),
                       "block_length": BLOCK_LEN, "seed": int(seed)},
        "frames": [_compact(r) for r in records], "joint": joint, "gates": [], "conclusion": {},
        "quality": {"mae_claimed": False, "accuracy_claimed": False,
                    "confidence_interval_claimed": False, "independent_verification": False}}
    summary["gates"] = gates(summary)
    accept = [g for g in summary["gates"] if g["role"] == "accept"]
    conditional_result = conditional_stability(summary["gates"])
    conditional = conditional_result["accepted"]
    common = all(g["pass"] for g in accept)
    failed = [g["name"] for g in accept if not g["pass"]]
    summary["conclusion"] = {
        "accepted_conditional_stability": bool(conditional),
        "accepted_common_needle_direction": bool(common), "accepted_overall": False,
        "reason": (
            "joint conditional stability is not accepted: required evidence is "
            "incomplete or a required gate failed ({}); the joint direction is not "
            "a trusted overall reading".format(", ".join(conditional_result["failed"]))
            if not conditional else
            "joint conditional precision passes but the consistent near/far bend "
            "and/or another common-direction gate fails; the joint direction is an "
            "average over a bent body, not a trusted overall reading"
            if failed else
            "joint conditional direction meets the listed gates, but overall RH is "
            "still unresolved by the uncalibrated roll/reference envelope"),
        "failed_gates": failed,
        "conditional_failed_gates": conditional_result["failed"],
        "no_improvement_claimed": not common, "stop": True,
        "next": "Lead independent review; no further model or threshold expansion"}
    return {"summary": summary,
            "overlays": [(r["index"], images[r["index"]], r, joint) for r in records]}


def run_cli(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Offline joint needle-direction burst study")
    parser.add_argument("--frames-dir", required=True)
    parser.add_argument("--calib", required=True)
    parser.add_argument("--allow-candidate", action="store_true")
    parser.add_argument("--calib-sha256", default=CANDIDATE_SHA)
    parser.add_argument("--output", required=True)
    parser.add_argument("--dev-frames", type=int, default=DEV_FRAMES)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--bootstrap", type=int, default=BOOTSTRAP_ITERS)
    parser.add_argument("--seed", type=int, default=SEED)
    args = parser.parse_args(argv)
    command = "python3 {} {}".format(Path(__file__).resolve(), " ".join(sys.argv[1:]))
    try:
        if not args.allow_candidate:
            raise ExperimentConfigError("loading a candidate calibration requires --allow-candidate")
        verify_r5_binding()
        output = ensure_output(Path(args.output))
        calibration = load_calibration(args.calib, True, args.calib_sha256)
        reference_path = Path(calibration["raw"]["source_image"]["path"])
        if not reference_path.is_file() or _sha(reference_path.read_bytes()) != calibration.get("source_sha256"):
            raise ExperimentConfigError("reference image missing or SHA-256 mismatch")
        reference_image = cv2.imread(str(reference_path), cv2.IMREAD_COLOR)
        if reference_image is None:
            raise ExperimentConfigError("cannot decode reference image")
        reference = r5.reference_signature(reference_image, calibration)
        if not reference.get("ok"):
            raise ExperimentConfigError("reference signature failed")
        result = run_study(Path(args.frames_dir), calibration, reference, output,
                           calibration_path=str(Path(args.calib).resolve()),
                           dev_frames=args.dev_frames, limit=args.limit,
                           bootstrap_iterations=args.bootstrap, seed=args.seed)
        summary = result["summary"]
        summary["command"] = command
        write_outputs(output, summary, result["overlays"])
    except (ExperimentConfigError, OSError, ValueError) as error:
        print("experiment failed: {}".format(error), file=sys.stderr)
        return 2
    joint = summary.get("joint") or {}
    print(json.dumps({"experimental": True, "value": None, "trusted": False,
                      "output": str(output), "frames": len(summary["frames"]),
                      "included_eval_frames": summary["input"]["included_eval_frames"],
                      "joint_angle_deg": joint.get("angle_deg"),
                      "loo_span_deg": joint.get("loo_span_deg"),
                      "near_far_difference_deg": (joint.get("near_far") or {}).get("difference_deg"),
                      "accepted_conditional_stability":
                          summary["conclusion"]["accepted_conditional_stability"],
                      "accepted_common_needle_direction":
                          summary["conclusion"]["accepted_common_needle_direction"],
                      "accepted_overall": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(run_cli())
