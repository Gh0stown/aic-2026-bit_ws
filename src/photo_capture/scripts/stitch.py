#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""区域扫描的全景/拼图拼接。

优先用 OpenCV 自带的 Stitcher：
  * 旋转全景 -> Stitcher_PANORAMA
  * 平移扫描 -> Stitcher_SCANS
失败时退化为相邻帧 ORB + RANSAC 单应，链到同一画布。

不依赖 ROS / Tk；可以被 UI、CLI 和单元测试直接调用。
"""

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional, Sequence

import cv2
import numpy as np


@dataclass
class StitchResult:
    ok: bool
    path: Optional[Path] = None
    width: int = 0
    height: int = 0
    method: str = ""
    message: str = ""
    frame_count: int = 0
    image: Optional[np.ndarray] = field(default=None, repr=False)


def _emit(progress, stage: str, value: float) -> None:
    if progress is None:
        return
    try:
        progress(stage, max(0.0, min(1.0, float(value))))
    except Exception:
        pass


def _resize(frame, max_width: int):
    if max_width <= 0:
        return frame
    height, width = frame.shape[:2]
    if width <= max_width:
        return frame
    scale = float(max_width) / width
    return cv2.resize(
        frame, (max_width, max(1, int(round(height * scale)))),
        interpolation=cv2.INTER_AREA)


def _load_frames(paths: Sequence[Path], max_width: int) -> List[np.ndarray]:
    frames = []
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        frames.append(_resize(image, max_width))
    return frames


def _crop_borders(image: np.ndarray) -> np.ndarray:
    if image is None or image.size == 0:
        return image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = (gray > 1).astype(np.uint8)
    points = cv2.findNonZero(mask)
    if points is None:
        return image
    x, y, width, height = cv2.boundingRect(points)
    if width < 2 or height < 2:
        return image
    return image[y:y + height, x:x + width]


def _try_opencv(frames: List[np.ndarray], mode: str):
    """返回 (pano, method, message)。"""
    if mode == "strafe":
        modes = [("SCANS", cv2.Stitcher_SCANS), ("PANORAMA", cv2.Stitcher_PANORAMA)]
    else:
        modes = [("PANORAMA", cv2.Stitcher_PANORAMA), ("SCANS", cv2.Stitcher_SCANS)]

    messages = []
    for name, flag in modes:
        try:
            stitcher = cv2.Stitcher_create(flag)
            status, pano = stitcher.stitch(frames)
        except Exception as error:  # pragma: no cover - 取决于 OpenCV 构建
            messages.append("{}: {}".format(name, error))
            continue
        if status == cv2.Stitcher_OK and pano is not None and pano.size:
            return pano, name, ""
        messages.append("{}: status={}".format(name, status))
    return None, "", "；".join(messages)


def _orb_homography(image_a: np.ndarray, image_b: np.ndarray):
    """估计 b -> a 的单应。"""
    gray_a = cv2.cvtColor(image_a, cv2.COLOR_BGR2GRAY)
    gray_b = cv2.cvtColor(image_b, cv2.COLOR_BGR2GRAY)

    orb = cv2.ORB_create(nfeatures=2500)
    keypoints_a, desc_a = orb.detectAndCompute(gray_a, None)
    keypoints_b, desc_b = orb.detectAndCompute(gray_b, None)
    if desc_a is None or desc_b is None or len(keypoints_a) < 12 or len(keypoints_b) < 12:
        return None

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    matches = matcher.knnMatch(desc_b, desc_a, k=2)
    good = []
    for pair in matches:
        if len(pair) < 2:
            continue
        first, second = pair
        if first.distance < 0.75 * second.distance:
            good.append(first)
    if len(good) < 12:
        return None

    src = np.float32([keypoints_b[match.queryIdx].pt for match in good]).reshape(-1, 1, 2)
    dst = np.float32([keypoints_a[match.trainIdx].pt for match in good]).reshape(-1, 1, 2)
    homography, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    if homography is None:
        return None
    inliers = int(mask.sum()) if mask is not None else 0
    if inliers < 10:
        return None
    return homography


def _orb_fallback(frames: List[np.ndarray], progress=None):
    """相邻帧两两估计单应，链到第一帧画布。失败返回 None。"""
    transforms = [np.eye(3, dtype=np.float64)]
    for index in range(1, len(frames)):
        _emit(progress, "特征匹配", 0.55 + 0.25 * index / max(1, len(frames) - 1))
        homography = _orb_homography(frames[index - 1], frames[index])
        if homography is None:
            return None
        transforms.append(transforms[-1] @ homography.astype(np.float64))

    corners = []
    for index, frame in enumerate(frames):
        height, width = frame.shape[:2]
        points = np.float32([
            [0, 0], [width, 0], [width, height], [0, height]
        ]).reshape(-1, 1, 2)
        warped = cv2.perspectiveTransform(points, transforms[index])
        corners.append(warped.reshape(-1, 2))
    corners = np.concatenate(corners, axis=0)
    min_xy = corners.min(axis=0)
    max_xy = corners.max(axis=0)
    margin = 8.0
    offset = np.array([
        [1.0, 0.0, -min_xy[0] + margin],
        [0.0, 1.0, -min_xy[1] + margin],
        [0.0, 0.0, 1.0],
    ])
    canvas_width = int(math.ceil(max_xy[0] - min_xy[0] + 2 * margin))
    canvas_height = int(math.ceil(max_xy[1] - min_xy[1] + 2 * margin))
    if canvas_width < 2 or canvas_height < 2:
        return None

    max_dimension = 16000
    scale = min(1.0, float(max_dimension) / max(canvas_width, canvas_height))
    if scale < 1.0:
        canvas_width = max(1, int(canvas_width * scale))
        canvas_height = max(1, int(canvas_height * scale))
        scale_matrix = np.array([
            [scale, 0.0, 0.0],
            [0.0, scale, 0.0],
            [0.0, 0.0, 1.0],
        ])
    else:
        scale_matrix = np.eye(3)

    accumulator = np.zeros((canvas_height, canvas_width, 3), dtype=np.float32)
    weights = np.zeros((canvas_height, canvas_width, 1), dtype=np.float32)
    for index, frame in enumerate(frames):
        _emit(progress, "融合", 0.82 + 0.15 * index / len(frames))
        transform = scale_matrix @ offset @ transforms[index]
        warped = cv2.warpPerspective(
            frame, transform, (canvas_width, canvas_height),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        mask = cv2.warpPerspective(
            np.ones(frame.shape[:2], dtype=np.float32), transform,
            (canvas_width, canvas_height), flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT)
        accumulator += warped.astype(np.float32) * mask[:, :, None]
        weights += mask[:, :, None]

    output = accumulator / np.maximum(weights, 1e-6)
    output = np.clip(output, 0, 255).astype(np.uint8)
    return _crop_borders(output)


def stitch_frames(paths: Sequence[Path], mode: str = "rotate",
                  max_width: int = 1600, output_path: Optional[Path] = None,
                  progress: Optional[Callable[[str, float], None]] = None
                  ) -> StitchResult:
    """把一组带重叠的帧拼成全景/拼图。

    mode: rotate / strafe / manual（manual 按 rotate 处理）
    """
    paths = [Path(path) for path in paths]
    if len(paths) < 2:
        return StitchResult(False, frame_count=len(paths),
                            message="至少需要 2 帧才能拼接")

    _emit(progress, "读取帧", 0.02)
    frames = _load_frames(paths, max_width)
    if len(frames) < 2:
        return StitchResult(False, frame_count=len(frames),
                            message="能读取的帧不足 2 张")

    _emit(progress, "OpenCV 拼接", 0.15)
    pano, method, message = _try_opencv(frames, mode)
    if pano is None:
        _emit(progress, "特征匹配兜底", 0.55)
        pano = _orb_fallback(frames, progress)
        if pano is not None:
            method = "ORB-HOMOGRAPHY"
            message = "OpenCV Stitcher 失败，使用 ORB 单应兜底"

    if pano is None or pano.size == 0:
        return StitchResult(False, frame_count=len(frames),
                            method="",
                            message="拼接失败：{}".format(message or "没有足够重叠"))

    pano = _crop_borders(pano)
    height, width = pano.shape[:2]
    if width < 2 or height < 2:
        return StitchResult(False, frame_count=len(frames),
                            method=method, message="拼接结果为空")

    if output_path is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(
                str(output_path), pano,
                [int(cv2.IMWRITE_JPEG_QUALITY), 92]):
            return StitchResult(False, frame_count=len(frames),
                                method=method,
                                message="全景写入失败：{}".format(output_path))
    _emit(progress, "完成", 1.0)
    return StitchResult(
        True, path=output_path, width=width, height=height,
        method=method, message=message, frame_count=len(frames),
        image=pano)
