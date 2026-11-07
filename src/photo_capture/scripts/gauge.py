#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仪表（温度/湿度表）Pose 识别。

模型输出 4 个关键点：
    0 = 指针旋转轴
    1 = 指针尖端
    2 = 量程最小值刻度
    3 = 量程最大值刻度
类别：temperature / humidity。

读数：以旋转轴为圆心，把指针角度投影到 最小刻度 -> 最大刻度 的圆弧上，
得到 0~1 的比例，再按用户设定的量程换算成温度/湿度。
"""

import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    from yolo_detect import ModelError, resolve_device
except Exception:  # pragma: no cover - 正常运行时不会发生
    class ModelError(RuntimeError):
        pass

    def resolve_device(preference="auto"):
        return "cpu"


@dataclass
class GaugeSettings:
    temp_min: float = 0.0
    temp_max: float = 100.0
    humidity_min: float = 0.0
    humidity_max: float = 100.0
    kpt_conf: float = 0.3
    arc: str = "long"        # long / short / auto
    imgsz: int = 640
    conf: float = 0.5

    def validate(self) -> "GaugeSettings":
        self.temp_min = float(self.temp_min)
        self.temp_max = float(self.temp_max)
        self.humidity_min = float(self.humidity_min)
        self.humidity_max = float(self.humidity_max)
        if self.temp_max <= self.temp_min:
            raise ValueError("温度上限必须大于下限")
        if self.humidity_max <= self.humidity_min:
            raise ValueError("湿度上限必须大于下限")
        self.kpt_conf = max(0.0, min(1.0, float(self.kpt_conf)))
        self.arc = self.arc if self.arc in ("long", "short", "auto") else "long"
        self.imgsz = max(64, int(self.imgsz))
        self.conf = max(0.01, min(0.99, float(self.conf)))
        return self

    def range_for(self, class_name: str):
        name = str(class_name).lower()
        if "temp" in name or "温度" in name:
            return self.temp_min, self.temp_max, "°C"
        if "humid" in name or "湿度" in name:
            return self.humidity_min, self.humidity_max, "%RH"
        return 0.0, 100.0, "%"


@dataclass
class GaugeDetection:
    class_id: int
    class_name: str
    box: Tuple[float, float, float, float]
    conf: float
    keypoints: np.ndarray
    keypoint_conf: np.ndarray
    fraction: Optional[float] = None
    value: Optional[float] = None
    unit: str = ""
    angle_deg: Optional[float] = None
    on_scale: bool = False
    message: str = ""


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compute_fraction(axis, tip, min_point, max_point, arc: str = "long"):
    """计算指针在最小->最大刻度弧上的位置。

    返回 (fraction 或 None, on_scale, 指针角度°, 最小角°, 最大角°, 弧方向, 弧角度°, 提示)。
    """
    axis = np.asarray(axis, dtype=float).reshape(2)
    tip = np.asarray(tip, dtype=float).reshape(2)
    min_point = np.asarray(min_point, dtype=float).reshape(2)
    max_point = np.asarray(max_point, dtype=float).reshape(2)

    if np.linalg.norm(min_point - axis) < 1.0 or np.linalg.norm(max_point - axis) < 1.0:
        return None, False, None, None, None, 0, 0.0, "最小/最大刻度离旋转轴太近"
    if np.linalg.norm(tip - axis) < 1.0:
        return None, False, None, None, None, 0, 0.0, "指针尖端离旋转轴太近"

    a_min = math.atan2(min_point[1] - axis[1], min_point[0] - axis[0])
    a_max = math.atan2(max_point[1] - axis[1], max_point[0] - axis[0])
    a_tip = math.atan2(tip[1] - axis[1], tip[0] - axis[0])

    short = _wrap(a_max - a_min)
    if abs(short) < math.radians(5.0):
        return None, False, math.degrees(a_tip) % 360.0, None, None, 0, 0.0, "最小和最大刻度几乎重合"
    short_dir = 1 if short >= 0 else -1
    short_sweep = abs(short)
    long_dir = -short_dir
    long_sweep = 2.0 * math.pi - short_sweep

    def fraction_for(direction: int, sweep: float) -> float:
        if direction > 0:
            delta = (a_tip - a_min) % (2.0 * math.pi)
        else:
            delta = (a_min - a_tip) % (2.0 * math.pi)
        return delta / sweep if sweep > 1e-9 else 0.0

    long_fraction = fraction_for(long_dir, long_sweep)
    short_fraction = fraction_for(short_dir, short_sweep)

    if arc == "short":
        fraction, direction, sweep = short_fraction, short_dir, short_sweep
    elif arc == "auto":
        if -0.02 <= long_fraction <= 1.02:
            fraction, direction, sweep = long_fraction, long_dir, long_sweep
        elif -0.02 <= short_fraction <= 1.02:
            fraction, direction, sweep = short_fraction, short_dir, short_sweep
        else:
            fraction, direction, sweep = long_fraction, long_dir, long_sweep
    else:
        fraction, direction, sweep = long_fraction, long_dir, long_sweep

    on_scale = -0.02 <= fraction <= 1.02
    message = "" if on_scale else "指针不在刻度弧上"
    return (fraction, on_scale, math.degrees(a_tip) % 360.0,
            math.degrees(a_min) % 360.0, math.degrees(a_max) % 360.0,
            direction, math.degrees(sweep), message)


def keypoint_radius(points) -> float:
    """4 个关键点到旋转轴的最小距离；用来排除挤成一团的误检。"""
    points = np.asarray(points, dtype=float).reshape(4, 2)
    axis = points[0]
    return float(min(np.linalg.norm(points[i] - axis) for i in (1, 2, 3)))


def reading_text(det: GaugeDetection) -> str:
    if det.value is None:
        value = "--"
    else:
        value = "{:.2f} {}".format(det.value, det.unit)
    if det.fraction is None:
        percent = "--"
    else:
        percent = "{:.0f}%".format(max(0.0, min(1.0, det.fraction)) * 100.0)
    return "{}: {} ({}) {:.2f}".format(det.class_name, value, percent, det.conf)


class PoseDetector:
    """后台线程跑 pose 模型，主线程取最新仪表读数。"""

    def __init__(self, model_path, settings: GaugeSettings,
                 device_pref: str = "auto"):
        self.model_path = str(model_path)
        self.settings = settings
        self.device = resolve_device(device_pref)
        self.last_error = ""
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._pending = None
        self._result = None
        self._running = True
        self._thread = None

        try:
            from ultralytics import YOLO
        except Exception as error:
            raise ModelError("无法导入 ultralytics：{}".format(error))

        try:
            self.model = YOLO(self.model_path)
        except Exception as error:
            raise ModelError("姿态模型加载失败：{}".format(error))
        if getattr(self.model, "task", "") != "pose":
            raise ModelError(
                "所选模型不是 pose 模型（task={}）。\n请选择 Pose.pt 这类姿态权重。".format(
                    getattr(self.model, "task", "?")))
        raw_names = getattr(self.model, "names", {}) or {}
        self.names: Dict[int, str] = (
            {int(k): str(v) for k, v in raw_names.items()}
            if isinstance(raw_names, dict)
            else {i: str(v) for i, v in enumerate(raw_names)})

        self._thread = threading.Thread(
            target=self._infer_loop, name="pose-infer", daemon=True)
        self._thread.start()

    @property
    def class_count(self) -> int:
        return len(self.names)

    def submit(self, frame, frame_id: int) -> None:
        with self._condition:
            if not self._running:
                return
            self._pending = (int(frame_id), frame)
            self._condition.notify()

    def latest(self):
        with self._lock:
            return self._result

    def close(self) -> None:
        with self._condition:
            self._running = False
            self._pending = None
            self._condition.notify()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _infer_loop(self) -> None:
        while True:
            with self._condition:
                while self._running and self._pending is None:
                    self._condition.wait(0.5)
                if not self._running:
                    return
                frame_id, frame = self._pending
                self._pending = None

            started = time.monotonic()
            try:
                result = self.model.predict(
                    frame,
                    imgsz=self.settings.imgsz,
                    conf=self.settings.conf,
                    device=self.device,
                    verbose=False,
                    max_det=50,
                )[0]
                detections = self._extract(result)
                self.last_error = ""
            except Exception as error:  # pragma: no cover - 取决于模型/硬件
                detections = []
                self.last_error = str(error)
            infer_ms = (time.monotonic() - started) * 1000.0
            with self._lock:
                if self._running:
                    self._result = (frame_id, detections, infer_ms)

    def _extract(self, result) -> List[GaugeDetection]:
        if getattr(result, "keypoints", None) is None:
            return []
        try:
            keypoints = result.keypoints.xy.cpu().numpy()
            keypoint_conf = result.keypoints.conf
            keypoint_conf = (keypoint_conf.cpu().numpy()
                             if keypoint_conf is not None else None)
        except Exception:
            return []
        if keypoints is None or len(keypoints) == 0:
            return []
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []
        try:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy().astype(int)
        except Exception:
            return []

        detections = []
        for index in range(min(len(keypoints), len(confs))):
            points = np.asarray(keypoints[index], dtype=float)
            if points.shape != (4, 2):
                continue
            if keypoint_conf is not None and index < len(keypoint_conf):
                kconf = np.asarray(keypoint_conf[index], dtype=float).reshape(-1)
            else:
                kconf = np.ones(4, dtype=float)
            class_id = int(classes[index])
            class_name = self.names.get(class_id, str(class_id))
            box = (float(xyxy[index][0]), float(xyxy[index][1]),
                   float(xyxy[index][2]), float(xyxy[index][3]))
            box_min = min(max(1.0, box[2] - box[0]), max(1.0, box[3] - box[1]))
            radius = keypoint_radius(points)
            low_conf = float(kconf.min()) < self.settings.kpt_conf
            if low_conf:
                fraction = None
                on_scale = False
                angle_deg = None
                message = "关键点置信度低"
            elif radius < max(12.0, 0.10 * box_min):
                fraction = None
                on_scale = False
                angle_deg = None
                message = "关键点过于集中，可能不是仪表"
            else:
                fraction, on_scale, angle_deg, _amin, _amax, _dir, _sweep, message = \
                    compute_fraction(points[0], points[1], points[2], points[3],
                                     self.settings.arc)
            vmin, vmax, unit = self.settings.range_for(class_name)
            value = None
            if fraction is not None and on_scale:
                value = vmin + max(0.0, min(1.0, fraction)) * (vmax - vmin)
            detections.append(GaugeDetection(
                class_id=class_id,
                class_name=class_name,
                box=box,
                conf=float(confs[index]),
                keypoints=points,
                keypoint_conf=kconf,
                fraction=fraction,
                value=value,
                unit=unit,
                angle_deg=angle_deg,
                on_scale=on_scale,
                message=message,
            ))
        return detections
