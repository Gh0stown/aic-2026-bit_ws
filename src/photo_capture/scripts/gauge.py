#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仪表（温度/湿度表）Pose 识别。

gauge.pt 输出 7 个关键点：
    0 = 指针旋转轴（静态）
    1 = 指针尖端（动态）
    2 = 量程最小值刻度（静态）
    3/4/5 = 三个中间刻度（静态）
    6 = 量程最大值刻度（静态）
temperature: min -30, scales -10/10/30, max 50
humidity:    min 0,   scales 20/40/70, max 100

6 个静态点（轴 + 5 个刻度）做单应 warp 到 512x512 正视空间，tip 不参与 warp；
正视空间用 Radon（tip 作搜索先验）求指针角度，再按三个中间刻度做分段线性插值。
每类表第一次识别时自动标定模板，模板保存在输出目录的 templates/<类别>.json，
可按 T 重标。旧 4 点 Pose.pt 自动退化为 3 点仿射 + 首尾刻度。
"""

import json
import math
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
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
    temp_min: float = -30.0
    temp_max: float = 50.0
    humidity_min: float = 0.0
    humidity_max: float = 100.0
    kpt_conf: float = 0.3
    arc: str = "long"        # long / short / auto
    pointer: str = "radon"   # radon / tip
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
        self.pointer = self.pointer if self.pointer in ("radon", "tip") else "radon"
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


@dataclass(frozen=True)
class GaugeSpec:
    class_name: str
    static_indices: Tuple[int, ...]
    tick_indices: Tuple[int, ...]
    tick_values: Tuple[float, ...]
    labels: Tuple[str, ...]
    unit: str


def gauge_spec_for(class_name: str, n_points: int,
                   settings: GaugeSettings) -> GaugeSpec:
    """按模型输出点数/类别给出静态配准点、刻度点和刻度值。"""
    name = str(class_name).lower()
    is_temp = "temp" in name or "温度" in name
    is_humid = "humid" in name or "湿度" in name
    vmin, vmax, unit = settings.range_for(class_name)
    if n_points >= 7:
        static = (0, 2, 3, 4, 5, 6)
        ticks = (2, 3, 4, 5, 6)
        if is_humid:
            values = (vmin, 20.0, 40.0, 70.0, vmax)
            labels = ("axis", "tip", "min_value", "scale_20", "scale_40",
                      "scale_70", "max_value")
        elif is_temp:
            values = (vmin, -10.0, 10.0, 30.0, vmax)
            labels = ("axis", "tip", "min_value", "scale_-10", "scale_10",
                      "scale_30", "max_value")
        else:
            values = (vmin, vmin + (vmax - vmin) * 0.25,
                      vmin + (vmax - vmin) * 0.5,
                      vmin + (vmax - vmin) * 0.75, vmax)
            labels = ("axis", "tip", "min_value", "scale_1", "scale_2",
                      "scale_3", "max_value")
        return GaugeSpec(class_name, static, ticks, tuple(float(v) for v in values),
                         labels, unit)
    if n_points >= 4:
        labels = ("axis", "tip", "min_value", "max_value")
        return GaugeSpec(class_name, (0, 2, 3), (2, 3), (float(vmin), float(vmax)),
                         labels, unit)
    return GaugeSpec(class_name, (0, 2, 3), (2, 3), (float(vmin), float(vmax)),
                     ("axis", "tip", "min_value", "max_value"), unit)


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
    static_indices: Tuple[int, ...] = (0, 2, 3)
    tick_indices: Tuple[int, ...] = (2, 3)
    tick_values: Tuple[float, ...] = (0.0, 100.0)
    labels: Tuple[str, ...] = ()
    n_keypoints: int = 4


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


def compute_value_from_ticks(axis, pointer, tick_points, tick_values,
                             arc: str = "long"):
    """用多个已知刻度点插值读数。

    比只用 min/max 更准：温度表用 -10/10/30 三个中间刻度，湿度表用 20/40/70，
    指针落在哪两个刻度之间就按这两个刻度线性插值。

    返回 (value 或 None, 归一化比例或 None, 是否在刻度内, 指针角度°, 提示)。
    """
    axis = np.asarray(axis, dtype=float).reshape(2)
    pointer = np.asarray(pointer, dtype=float).reshape(2)
    points = np.asarray(tick_points, dtype=float)
    values = [float(v) for v in tick_values]
    if points.ndim != 2 or points.shape[1] != 2 or len(points) != len(values):
        return None, None, False, None, "刻度点数量不匹配"
    if len(points) < 2:
        return None, None, False, None, "刻度点不足"
    if any(values[i] > values[i + 1] for i in range(len(values) - 1)):
        return None, None, False, None, "刻度值顺序错误"
    if np.linalg.norm(points[0] - axis) < 1.0 or np.linalg.norm(points[-1] - axis) < 1.0:
        return None, None, False, None, "最小/最大刻度离旋转轴太近"
    if np.linalg.norm(pointer - axis) < 1.0:
        return None, None, False, None, "指针尖端离旋转轴太近"

    a_min = math.atan2(points[0][1] - axis[1], points[0][0] - axis[0])
    a_max = math.atan2(points[-1][1] - axis[1], points[-1][0] - axis[0])
    a_tip = math.atan2(pointer[1] - axis[1], pointer[0] - axis[0])
    short = _wrap(a_max - a_min)
    if abs(short) < math.radians(5.0):
        return None, None, False, math.degrees(a_tip) % 360.0, "最小和最大刻度几乎重合"
    short_dir = 1 if short >= 0 else -1
    short_sweep = abs(short)
    long_dir = -short_dir
    long_sweep = 2.0 * math.pi - short_sweep

    def fraction_for(point, direction, sweep):
        angle = math.atan2(point[1] - axis[1], point[0] - axis[0])
        if direction > 0:
            delta = (angle - a_min) % (2.0 * math.pi)
        else:
            delta = (a_min - angle) % (2.0 * math.pi)
        return delta / sweep if sweep > 1e-9 else 0.0

    def evaluate(direction, sweep):
        fracs = [fraction_for(point, direction, sweep) for point in points]
        pointer_frac = fraction_for(pointer, direction, sweep)
        monotonic = all(fracs[i] <= fracs[i + 1] + 0.02
                        for i in range(len(fracs) - 1))
        inside = all(-0.05 <= frac <= 1.05 for frac in fracs)
        valid = monotonic and inside and -0.02 <= pointer_frac <= 1.02
        return fracs, pointer_frac, monotonic, valid

    long_fracs, long_pointer, long_mono, long_valid = evaluate(long_dir, long_sweep)
    short_fracs, short_pointer, short_mono, short_valid = evaluate(short_dir, short_sweep)
    if arc == "short":
        fracs, pointer_frac, monotonic, valid = (
            short_fracs, short_pointer, short_mono, short_valid)
    elif arc == "auto" and not long_valid and short_valid:
        fracs, pointer_frac, monotonic, valid = (
            short_fracs, short_pointer, short_mono, short_valid)
    else:
        fracs, pointer_frac, monotonic, valid = (
            long_fracs, long_pointer, long_mono, long_valid)

    angle_deg = math.degrees(a_tip) % 360.0
    if not monotonic or not valid:
        # 刻度顺序异常时退回首尾刻度，至少给出一个近似值
        if arc == "short":
            direction, sweep = short_dir, short_sweep
        else:
            direction, sweep = long_dir, long_sweep
        p_frac = fraction_for(pointer, direction, sweep)
        if -0.02 <= p_frac <= 1.02:
            p_frac = max(0.0, min(1.0, p_frac))
            value = values[0] + p_frac * (values[-1] - values[0])
            norm = (value - values[0]) / (values[-1] - values[0])                 if values[-1] != values[0] else 0.0
            return value, norm, True, angle_deg, "刻度点顺序异常，按首尾刻度估算"
        return None, None, False, angle_deg, "指针不在刻度弧上"

    if pointer_frac < fracs[0] - 0.02 or pointer_frac > fracs[-1] + 0.02:
        message = "指针不在刻度弧上"
        return None, None, False, angle_deg, message
    pointer_frac = max(0.0, min(1.0, pointer_frac))
    value = None
    for index in range(len(fracs) - 1):
        left, right = fracs[index], fracs[index + 1]
        if right - left < 1e-9:
            continue
        if left - 1e-9 <= pointer_frac <= right + 1e-9:
            ratio = (pointer_frac - left) / (right - left)
            value = values[index] + ratio * (values[index + 1] - values[index])
            break
    if value is None:
        return None, None, False, angle_deg, "指针不在刻度区间内"
    norm = (value - values[0]) / (values[-1] - values[0])         if values[-1] != values[0] else 0.0
    return value, max(0.0, min(1.0, norm)), True, angle_deg, ""


def keypoint_radius(points) -> float:
    """所有关键点到旋转轴的最小距离；用来排除挤成一团的误检。"""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
        return 0.0
    axis = points[0]
    return float(min(np.linalg.norm(points[i] - axis)
                     for i in range(1, len(points))))


def reading_text(item) -> str:
    det = getattr(item, "detection", item)
    fraction = getattr(item, "fraction", None)
    value_num = getattr(item, "value", None)
    unit = getattr(item, "unit", "")
    message = getattr(item, "message", "")
    if value_num is None:
        value = "--"
    else:
        value = "{:.2f} {}".format(value_num, unit)
    if fraction is None:
        percent = "--"
    else:
        percent = "{:.0f}%".format(max(0.0, min(1.0, fraction)) * 100.0)
    text = "{}: {} ({}) {:.2f}".format(det.class_name, value, percent, det.conf)
    if message:
        text += " " + message
    return text


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
                "所选模型不是 pose 模型（task={}）。\n请选择 gauge.pt 这类姿态权重。".format(
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
            if points.ndim != 2 or points.shape[1] != 2 or points.shape[0] < 4:
                continue
            n_points = points.shape[0]
            if keypoint_conf is not None and index < len(keypoint_conf):
                kconf = np.asarray(keypoint_conf[index], dtype=float).reshape(-1)
                if len(kconf) < n_points:
                    kconf = np.pad(kconf, (0, n_points - len(kconf)),
                                   constant_values=0.0)
                kconf = kconf[:n_points]
            else:
                kconf = np.ones(n_points, dtype=float)
            class_id = int(classes[index])
            class_name = self.names.get(class_id, str(class_id))
            spec = gauge_spec_for(class_name, n_points, self.settings)
            box = (float(xyxy[index][0]), float(xyxy[index][1]),
                   float(xyxy[index][2]), float(xyxy[index][3]))
            box_min = min(max(1.0, box[2] - box[0]), max(1.0, box[3] - box[1]))
            radius = keypoint_radius(points)
            low_conf = float(kconf.min()) < self.settings.kpt_conf
            if low_conf:
                fraction = None
                value = None
                on_scale = False
                angle_deg = None
                message = "关键点置信度低"
            elif radius < max(12.0, 0.10 * box_min):
                fraction = None
                value = None
                on_scale = False
                angle_deg = None
                message = "关键点过于集中，可能不是仪表"
            else:
                tick_points = points[list(spec.tick_indices)]
                value, fraction, on_scale, angle_deg, message = \
                    compute_value_from_ticks(
                        points[0], points[1], tick_points,
                        spec.tick_values, self.settings.arc)
            detections.append(GaugeDetection(
                class_id=class_id,
                class_name=class_name,
                box=box,
                conf=float(confs[index]),
                keypoints=points,
                keypoint_conf=kconf,
                fraction=fraction,
                value=value,
                unit=spec.unit,
                angle_deg=angle_deg,
                on_scale=on_scale,
                message=message,
                static_indices=spec.static_indices,
                tick_indices=spec.tick_indices,
                tick_values=spec.tick_values,
                labels=spec.labels,
                n_keypoints=n_points,
            ))
        return detections

def _safe_name(text) -> str:
    text = re.sub(r'[\\/:*?"<>|\x00-\x1f]', "_", str(text)).strip(" .")
    return text[:40] or "gauge"


@dataclass
class GaugeTemplate:
    class_name: str
    static_indices: Tuple[int, ...]
    tick_indices: Tuple[int, ...]
    tick_values: Tuple[float, ...]
    static_points: np.ndarray
    radius: float = 190.0
    created_at: str = ""

    @property
    def axis(self) -> np.ndarray:
        return self.static_points[self.static_indices.index(0)]

    @property
    def tick_points(self) -> np.ndarray:
        positions = [self.static_indices.index(index) for index in self.tick_indices]
        return self.static_points[positions]

    @property
    def min_point(self) -> np.ndarray:
        return self.tick_points[0]

    @property
    def max_point(self) -> np.ndarray:
        return self.tick_points[-1]

    def to_dict(self):
        return {
            "class_name": self.class_name,
            "static_indices": [int(v) for v in self.static_indices],
            "tick_indices": [int(v) for v in self.tick_indices],
            "tick_values": [float(v) for v in self.tick_values],
            "static_points": [[round(float(x), 3), round(float(y), 3)]
                              for x, y in self.static_points],
            "radius": float(self.radius),
            "created_at": self.created_at or datetime.now().isoformat(timespec="seconds"),
        }

    @classmethod
    def from_dict(cls, data):
        if "static_points" in data:
            static_points = np.asarray(data["static_points"], dtype=float)
            static_indices = tuple(int(v) for v in data["static_indices"])
            tick_indices = tuple(int(v) for v in data["tick_indices"])
            tick_values = tuple(float(v) for v in data["tick_values"])
            if static_points.ndim != 2 or static_points.shape[1] != 2:
                raise ValueError("模板点格式错误")
            if len(static_points) != len(static_indices):
                raise ValueError("模板点数量不匹配")
            return cls(str(data.get("class_name", "gauge")), static_indices,
                       tick_indices, tick_values, static_points,
                       float(data.get("radius", 190.0)),
                       str(data.get("created_at", "")))
        # 兼容旧版三点模板
        axis = np.asarray(data["axis"], dtype=float)
        min_point = np.asarray(data["min_point"], dtype=float)
        max_point = np.asarray(data["max_point"], dtype=float)
        if axis.shape != (2,) or min_point.shape != (2,) or max_point.shape != (2,):
            raise ValueError("模板点格式错误")
        return cls(str(data.get("class_name", "gauge")), (0, 2, 3), (2, 3),
                   (0.0, 100.0), np.stack([axis, min_point, max_point]),
                   float(data.get("radius", 190.0)),
                   str(data.get("created_at", "")))


class GaugeTemplateStore:
    def __init__(self, output_root, model_path):
        stem = _safe_name(Path(model_path).stem)
        self.directory = Path(output_root) / "仪表识别" / stem / "templates"
        self._cache = {}

    def path_for(self, class_name: str) -> Path:
        return self.directory / (_safe_name(class_name) + ".json")

    def load(self, class_name: str) -> Optional[GaugeTemplate]:
        if class_name in self._cache:
            return self._cache[class_name]
        path = self.path_for(class_name)
        if not path.is_file():
            return None
        try:
            with path.open("r", encoding="utf-8") as handle:
                template = GaugeTemplate.from_dict(json.load(handle))
            self._cache[class_name] = template
            return template
        except Exception:
            return None

    def save(self, template: GaugeTemplate) -> Optional[Path]:
        self._cache[template.class_name] = template
        path = self.path_for(template.class_name)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", encoding="utf-8") as handle:
                json.dump(template.to_dict(), handle, ensure_ascii=False, indent=2)
            return path
        except OSError:
            return None

    def clear(self) -> None:
        self._cache.clear()
        try:
            for path in self.directory.glob("*.json"):
                path.unlink()
        except OSError:
            pass


def _unit_angle(axis, point) -> float:
    return math.atan2(point[1] - axis[1], point[0] - axis[0])


def _template_point(center, radius, angle):
    return center + radius * np.array([math.cos(angle), math.sin(angle)], dtype=float)


@dataclass
class GaugeReading:
    detection: GaugeDetection
    warped: np.ndarray
    matrix: np.ndarray
    template: GaugeTemplate
    fraction: Optional[float] = None
    value: Optional[float] = None
    unit: str = ""
    angle_deg: Optional[float] = None
    on_scale: bool = False
    message: str = ""


def estimate_pointer_angle(warped, center=(256.0, 256.0),
                           prior: float = 0.0, radius: float = 190.0,
                           search_deg: float = 22.0, samples: int = 91):
    """在正视空间里沿射线找最暗的指针方向（Radon 式），prior 为模型尖端方向。"""
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    if float(gray.std()) < 1.0:
        return prior, 0.0
    center = np.asarray(center, dtype=np.float32)
    radii = np.arange(radius * 0.15, radius * 0.95, 1.0, dtype=np.float32)
    if radii.size < 10:
        return prior, 0.0
    prior = _wrap(prior)
    angles = np.linspace(prior - math.radians(search_deg),
                         prior + math.radians(search_deg), samples)
    def ray_score(angle):
        xs = center[0] + radii * math.cos(angle)
        ys = center[1] + radii * math.sin(angle)
        values = cv2.remap(
            gray, xs.reshape(1, -1), ys.reshape(1, -1),
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE)
        return float(np.mean(255.0 - values))

    best_angle = prior
    best_score = -1.0
    prior_score = ray_score(prior)
    for angle in angles:
        try:
            score = ray_score(angle)
        except cv2.error:
            continue
        if score > best_score:
            best_score = score
            best_angle = angle
    # 只有明显比模型尖端方向更暗时才采用 Radon 结果，避免被刻度线带偏
    if best_score >= prior_score + 1.0:
        return _wrap(best_angle), best_score
    return prior, prior_score


class GaugeRectifier:
    """用静态关键点做正视化：4 个以上用单应，3 个用仿射；tip 不参与 warp。"""

    SIZE = 512
    CENTER = np.array([256.0, 256.0], dtype=float)
    RADIUS = 190.0

    def __init__(self, output_root, model_path, settings: GaugeSettings):
        self.settings = settings
        self.store = GaugeTemplateStore(output_root, model_path)
        self.last_message = ""

    def _calibrate(self, detection: GaugeDetection) -> GaugeTemplate:
        axis = np.asarray(detection.keypoints[0], dtype=float)
        static_indices = tuple(int(v) for v in detection.static_indices)
        tick_indices = tuple(int(v) for v in detection.tick_indices)
        tick_values = tuple(float(v) for v in detection.tick_values)
        if 0 not in static_indices or len(static_indices) < 3:
            raise ValueError("静态关键点不足，无法标定模板")
        if len(tick_indices) < 2:
            raise ValueError("刻度关键点不足")
        box = detection.box
        box_min = min(max(1.0, box[2] - box[0]), max(1.0, box[3] - box[1]))
        if keypoint_radius(detection.keypoints) < max(12.0, 0.10 * box_min):
            raise ValueError("关键点过于集中，无法标定模板")
        static_points = []
        for index in static_indices:
            if index == 0:
                static_points.append(self.CENTER.copy())
            else:
                angle = _unit_angle(axis, detection.keypoints[index])
                static_points.append(_template_point(self.CENTER, self.RADIUS, angle))
        template = GaugeTemplate(
            class_name=detection.class_name,
            static_indices=static_indices,
            tick_indices=tick_indices,
            tick_values=tick_values,
            static_points=np.asarray(static_points, dtype=float),
            radius=self.RADIUS,
            created_at=datetime.now().isoformat(timespec="seconds"),
        )
        self.store.save(template)
        self.last_message = "已自动标定模板：{}".format(template.class_name)
        return template

    def template_for(self, detection: GaugeDetection) -> GaugeTemplate:
        template = self.store.load(detection.class_name)
        if template is None:
            template = self._calibrate(detection)
        return template

    def recalibrate(self, detections) -> int:
        count = 0
        for detection in detections:
            if float(detection.keypoint_conf.min()) < self.settings.kpt_conf:
                continue
            if detection.message and "关键点" in detection.message:
                continue
            try:
                self._calibrate(detection)
                count += 1
            except Exception:
                continue
        if count:
            self.last_message = "已重新标定 {} 类模板".format(count)
        return count

    def rectify(self, frame, detection: GaugeDetection) -> Optional[GaugeReading]:
        if detection.keypoints.ndim != 2 or detection.keypoints.shape[1] != 2:
            return None
        if len(detection.keypoints) != len(detection.keypoint_conf):
            return None
        if float(detection.keypoint_conf.min()) < self.settings.kpt_conf:
            return None
        invalid_prefixes = (
            "关键点置信度低", "关键点过于集中",
            "最小/最大刻度离旋转轴太近", "指针尖端离旋转轴太近",
            "最小和最大刻度几乎重合",
        )
        if detection.message and any(
                detection.message.startswith(prefix) for prefix in invalid_prefixes):
            return None
        try:
            template = self.template_for(detection)
        except Exception as error:
            self.last_message = str(error)
            return None
        static_indices = list(template.static_indices)
        if 0 not in static_indices or len(static_indices) < 3:
            return None
        src = np.float32(detection.keypoints[static_indices])
        dst = np.float32(template.static_points)
        matrix = None
        warped = None
        tip = None
        try:
            if len(static_indices) >= 4:
                matrix, _mask = cv2.findHomography(src, dst, 0)
                if matrix is not None:
                    warped = cv2.warpPerspective(
                        frame, matrix, (self.SIZE, self.SIZE),
                        flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                    tip = cv2.perspectiveTransform(
                        np.array([[detection.keypoints[1]]], dtype=np.float32),
                        matrix)[0][0]
            if matrix is None:
                # 三点仿射兜底（旧 4 点模型或单应退化）
                axis_pos = static_indices.index(0)
                min_pos = static_indices.index(detection.tick_indices[0])
                max_pos = static_indices.index(detection.tick_indices[-1])
                src3 = np.float32([src[axis_pos], src[min_pos], src[max_pos]])
                dst3 = np.float32([dst[axis_pos], dst[min_pos], dst[max_pos]])
                matrix = cv2.getAffineTransform(src3, dst3)
                warped = cv2.warpAffine(
                    frame, matrix, (self.SIZE, self.SIZE),
                    flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                tip = cv2.transform(
                    np.array([[detection.keypoints[1]]], dtype=np.float32),
                    matrix)[0][0]
        except (cv2.error, ValueError):
            return None
        if warped is None or tip is None:
            return None
        axis = template.axis
        prior = math.atan2(tip[1] - axis[1], tip[0] - axis[0])
        angle = prior
        if self.settings.pointer == "radon":
            angle, _score = estimate_pointer_angle(
                warped, tuple(axis), prior, template.radius)
        tip_point = _template_point(axis, template.radius, angle)
        tick_values = template.tick_values
        if len(detection.tick_values) == len(template.tick_indices):
            # 量程/中间刻度以当前模型配置为准，模板只提供几何
            tick_values = detection.tick_values
        value, fraction, on_scale, angle_deg, message = \
            compute_value_from_ticks(
                axis, tip_point, template.tick_points,
                tick_values, self.settings.arc)
        unit = detection.unit or self.settings.range_for(detection.class_name)[2]
        return GaugeReading(
            detection=detection,
            warped=warped,
            matrix=matrix,
            template=template,
            fraction=fraction,
            value=value,
            unit=unit,
            angle_deg=angle_deg,
            on_scale=on_scale,
            message=message,
        )
