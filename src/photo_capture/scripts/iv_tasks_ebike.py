#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""task=ebike（电动车：街区违停 / 停车区正常-倒伏）。

两种模式：
- 街区（默认）：单帧看当前视角，`people.pt` 的 `cycle` 类判断本视角有没有违停
  （每方向只报 0/1，跨视角汇总由导航做，不做多视角相加）；
- 停车区（sub_id 含“停车区”或 params.mode=parking）：
  * 默认单帧 `best.pt` 数当前画面的 倒伏/正常；
  * `params.pan=true` 时录 `duration` 秒视频（平移由导航完成，节点不发 /cmd_vel），
    再交给 `count_video` 用 `Cycle.pt` 跟踪去重，数 normal/fall。

节点只返回当前视角/本次视频的计数，街区内汇总与最终播报措辞由导航决定。
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from iv_common import Detection, TaskResult, VisionContext, VisionError

_PARKING_WORDS = ("停车区", "停车位", "17", "18")
_FALL_NAMES = ("DOWN", "倒伏", "fall")
_NORMAL_NAMES = ("UP", "正常", "normal")


# ---------------------------------------------------------------------------
# 公共
# ---------------------------------------------------------------------------

def _block_label(context: VisionContext) -> str:
    text = str(context.sub_id or "")
    for word in ("A街区", "B街区", "A 街", "B 街"):
        if word in text:
            return "A" if "A" in word else "B"
    return text or "该"


def _is_parking_mode(context: VisionContext, params: Dict[str, Any]) -> bool:
    mode = str(params.get("mode") or "").strip().lower()
    if mode in ("parking", "park"):
        return True
    if mode in ("street", "block"):
        return False
    return any(word in str(context.sub_id or "") for word in _PARKING_WORDS)


def _classify(name: str) -> Optional[str]:
    if name in _NORMAL_NAMES or name.upper() in _NORMAL_NAMES:
        return "normal"
    if name in _FALL_NAMES or name.upper() in _FALL_NAMES:
        return "fall"
    return None


def _street_model_choice(context: VisionContext,
                         params: Dict[str, Any]) -> Tuple[str, List[str]]:
    config = context.config.get("ebike", {})
    model_key = str(params.get("street_model")
                    or config.get("street_model", "crowd")).lower()
    if model_key in ("cycle", "bike", "bike_count"):
        return "bike_count", ["UP", "DOWN"]
    if model_key in ("crowd", "people", "person"):
        return "crowd", [str(item) for item in (config.get("street_classes") or ["cycle"])]
    # 默认 best.pt：倒伏/正常两类，任何一辆都算该方向有违停
    return "bike_state", []


# ---------------------------------------------------------------------------
# 街区违停（单帧）
# ---------------------------------------------------------------------------

def _handle_street(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    conf = context.threshold("ebike", 0.30)
    model_key, classes = _street_model_choice(context, params)
    frames = context.grab()
    frame = frames[-1]
    model = context.model(context.model_path(model_key), imgsz=640, conf=conf)
    detections = model.predict(frame, conf=conf, classes=classes)
    context.check_deadline("电动车检测")
    block = _block_label(context)
    count = len(detections)
    illegal = 1 if count > 0 else 0
    # 细则 19：每个街区方向至多 1 辆违停，节点只报“有/无”（0/1），
    # 原始检测框数放 extra.count 供复盘，不写进播报。
    if illegal:
        text = "{} 街区有 {} 辆电动车违停".format(block, illegal)
    else:
        text = "{} 街区无违停".format(block)
    image = context.save_frame(
        frame, tag="ebike", detections=detections, caption=text,
        extra_lines=["模型 {} / 类别 {}".format(Path(model.path).name,
                                                ",".join(classes))])
    extra = {"mode": "street", "block": block, "illegal": illegal,
             "count": count, "model": Path(model.path).name,
             "detections": [det.as_dict() for det in detections]}
    raw = context.save_raw(frame)
    if raw:
        extra["raw_image"] = raw
    return context.ok(text, extra, [image])


# ---------------------------------------------------------------------------
# 停车区单帧
# ---------------------------------------------------------------------------

def _handle_parking_frame(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    conf = context.threshold("ebike", 0.30)
    frames = context.grab()
    frame = frames[-1]
    # 2026 实拍：best.pt 会把直立车判成倒伏，Cycle.pt 的 UP/DOWN 更准；
    # 两者都可通过 bike_state / bike_count 配置切换。
    model = context.model(context.model_path("bike_count"), imgsz=640, conf=conf)
    detections = model.predict(frame, conf=conf)
    context.check_deadline("停车区检测")
    normal = fall = 0
    normal_dets: List[Detection] = []
    fall_dets: List[Detection] = []
    for det in detections:
        kind = _classify(det.name)
        if kind == "normal":
            normal += 1
            normal_dets.append(det)
        elif kind == "fall":
            fall += 1
            fall_dets.append(det)
    text = "停车区内电动车正常 {} 辆，倒伏 {} 辆".format(normal, fall)
    image = context.save_frame(
        frame, tag="ebike_parking", detections=detections, caption=text,
        extra_lines=["模型 {}（单帧，未跨视角去重）".format(Path(model.path).name)])
    extra = {"mode": "parking", "normal": normal, "fall": fall,
             "frame_count": 1, "model": Path(model.path).name,
             "detections": [det.as_dict() for det in detections]}
    raw = context.save_raw(frame)
    if raw:
        extra["raw_image"] = raw
    return context.ok(text, extra, [image])


# ---------------------------------------------------------------------------
# 停车区平移视频跟踪
# ---------------------------------------------------------------------------

def _record_video(context: VisionContext, duration: float, fps: float) -> Tuple[Path, int, Tuple[int, int]]:
    """录 duration 秒视频；平移由导航负责，节点只保证连续取帧。"""
    fps = max(1.0, min(30.0, float(fps)))
    duration = max(0.5, float(duration))
    deadline = time.monotonic() + duration
    frames: List[np.ndarray] = []
    interval = 1.0 / fps
    next_at = time.monotonic()
    size: Optional[Tuple[int, int]] = None
    while time.monotonic() < deadline:
        context.check_deadline("平移录像")
        frame = context.frames.read()
        if frame is None:
            raise VisionError("平移录像期间相机断流")
        if size is None:
            size = (int(frame.shape[1]), int(frame.shape[0]))
        if (frame.shape[1], frame.shape[0]) != size:
            frame = cv2.resize(frame, size)
        frames.append(frame)
        next_at += interval
        sleep = next_at - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_at = time.monotonic()
    if not frames:
        raise VisionError("平移录像没有取到画面")
    path = context.run_dir / ("ebike_pan_%s.avi" % time.strftime("%H%M%S"))
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"),
                             fps, size)
    if not writer.isOpened():
        raise VisionError("无法写入平移视频：{}".format(path))
    try:
        for frame in frames:
            writer.write(frame)
    finally:
        writer.release()
    return path, len(frames), size


def _handle_parking_pan(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    config = context.config.get("ebike", {}).get("parking_pan", {}) or {}
    duration = float(params.get("duration") or config.get("duration", 8.0))
    fps = float(params.get("fps") or config.get("fps", 15.0))
    min_frames = int(config.get("min_frames", 5))
    video, frame_count, size = _record_video(context, duration, fps)
    context.check_deadline("视频计数")

    try:
        from count_video import count_video
    except Exception as error:  # noqa: BLE001
        raise VisionError("无法导入 count_video：{}".format(error))
    model_path = context.models.resolve(context.model_path("bike_count"))
    if not model_path.is_file():
        raise VisionError("计数模型不存在：{}".format(model_path))
    result = count_video(
        video=str(video),
        model=str(model_path),
        conf=context.threshold("ebike", 0.30),
        imgsz=640,
        device="cpu",
        min_frames=min_frames,
        classes="UP,DOWN",
        output_dir=str(context.run_dir / "ebike_count"),
        save_video=False,
    )
    context.check_deadline("视频计数")
    if not result.ok:
        raise VisionError(
            "停车区视频计数失败：{}（视频 {} 帧，{}）".format(
                result.message or "未知原因", frame_count, video))
    by_class = dict(result.by_class or {})
    normal = int(by_class.get("UP", 0))
    fall = int(by_class.get("DOWN", 0))
    text = "停车区内电动车正常 {} 辆，倒伏 {} 辆".format(normal, fall)
    images: List[str] = []
    for candidate in (result.contact_sheet,):
        if candidate and Path(candidate).is_file():
            images.append(str(candidate))
    for track in (result.tracks or []):
        for key in ("crop", "image", "crop_path"):
            path = track.get(key)
            if path and Path(path).is_file():
                images.append(str(path))
                break
    if not images and video.is_file():
        # 视频不是图片，GUI 显示不了；给一张首帧带框缩略图
        cap = cv2.VideoCapture(str(video))
        ok, frame = cap.read()
        cap.release()
        if ok and frame is not None:
            images.append(context.save_frame(frame, tag="ebike_pan_first",
                                             caption=text))
    extra = {
        "mode": "parking_pan",
        "normal": normal,
        "fall": fall,
        "track_count": int(result.count),
        "by_class": by_class,
        "total_frames": int(result.total_frames or frame_count),
        "video_fps": float(result.fps or fps),
        "video": str(video),
        "summary": str(result.summary_path or ""),
        "min_frames": min_frames,
    }
    return context.ok(text, extra, images)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def handle(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    if not _is_parking_mode(context, params):
        return _handle_street(context, params)
    pan = str(params.get("pan", "")).lower() in ("1", "true", "yes", "on")
    if pan:
        return _handle_parking_pan(context, params)
    # 服务里如果要求平移录像，但导航没同时移动，宁可返回单帧结果并注明
    return _handle_parking_frame(context, params)
