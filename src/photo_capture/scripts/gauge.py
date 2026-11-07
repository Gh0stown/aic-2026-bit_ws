#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仪表（温度/湿度表）Pose 识别。

模型输出 4 个关键点：
    0 = 指针旋转轴
    1 = 指针尖端
    2 = 量程最小值刻度
    3 = 量程最大值刻度
类别：temperature / humidity。

当前 Pose.pt 的 4 个点里只有 3 个是静态的（轴、最小、最大），tip 会随读数动。
所以配准只用 3 个静态点做仿射 warp 到 512x512 正视空间，tip 不参与 warp；
在正视空间里用 Radon（tip 作为搜索先验）求指针角度，再投影到最小->最大刻度弧，
按量程换算读数。每类表第一次识别时自动标定模板，模板保存在输出目录的
templates/<类别>.json，可按 T 重标。

如果以后模型增加第 5 个静态关键点，可把 4 个静态点换成 homography。
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
    temp_min: float = 0.0
    temp_max: float = 100.0
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


def _safe_name(text) -> str:
    text = re.sub(r'[\\/:*?"<>|\x00-\x1f]', "_", str(text)).strip(" .")
    return text[:40] or "gauge"


@dataclass
class GaugeTemplate:
    class_name: str
    axis: np.ndarray
    min_point: np.ndarray
    max_point: np.ndarray
    radius: float = 190.0
    created_at: str = ""

    def to_dict(self):
        return {
            "class_name": self.class_name,
            "axis": [round(float(v), 3) for v in self.axis],
            "min_point": [round(float(v), 3) for v in self.min_point],
            "max_point": [round(float(v), 3) for v in self.max_point],
            "radius": float(self.radius),
            "created_at": self.created_at or datetime.now().isoformat(timespec="seconds"),
        }

    @classmethod
    def from_dict(cls, data):
        axis = np.asarray(data["axis"], dtype=float)
        min_point = np.asarray(data["min_point"], dtype=float)
        max_point = np.asarray(data["max_point"], dtype=float)
        if axis.shape != (2,) or min_point.shape != (2,) or max_point.shape != (2,):
            raise ValueError("模板点格式错误")
        return cls(str(data.get("class_name", "gauge")), axis, min_point, max_point,
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
    """用 3 个静态点（轴、最小、最大）做仿射正视化，tip 只参与指针角度。"""

    SIZE = 512
    CENTER = np.array([256.0, 256.0], dtype=float)
    RADIUS = 190.0

    def __init__(self, output_root, model_path, settings: GaugeSettings):
        self.settings = settings
        self.store = GaugeTemplateStore(output_root, model_path)
        self.last_message = ""

    def _calibrate(self, detection: GaugeDetection) -> GaugeTemplate:
        axis = np.asarray(detection.keypoints[0], dtype=float)
        min_point = np.asarray(detection.keypoints[2], dtype=float)
        max_point = np.asarray(detection.keypoints[3], dtype=float)
        box = detection.box
        box_min = min(max(1.0, box[2] - box[0]), max(1.0, box[3] - box[1]))
        if keypoint_radius(detection.keypoints) < max(12.0, 0.10 * box_min):
            raise ValueError("关键点过于集中，无法标定模板")
        min_angle = _unit_angle(axis, min_point)
        max_angle = _unit_angle(axis, max_point)
        if abs(_wrap(max_angle - min_angle)) < math.radians(5.0):
            raise ValueError("最小/最大刻度角度过小，无法标定模板")
        template = GaugeTemplate(
            class_name=detection.class_name,
            axis=self.CENTER.copy(),
            min_point=_template_point(self.CENTER, self.RADIUS, min_angle),
            max_point=_template_point(self.CENTER, self.RADIUS, max_angle),
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
        if detection.keypoints.shape != (4, 2):
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
            # 关键点/几何无效，不用它标定或读数；刻度弧外仍允许正视化后重算
            return None
        try:
            template = self.template_for(detection)
        except Exception as error:
            self.last_message = str(error)
            return None
        src = np.float32([
            detection.keypoints[0], detection.keypoints[2], detection.keypoints[3]])
        dst = np.float32([
            template.axis, template.min_point, template.max_point])
        try:
            matrix = cv2.getAffineTransform(src, dst)
            warped = cv2.warpAffine(
                frame, matrix, (self.SIZE, self.SIZE),
                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            tip = cv2.transform(
                np.array([[detection.keypoints[1]]], dtype=np.float32),
                matrix)[0][0]
        except cv2.error:
            return None
        axis = template.axis
        prior = math.atan2(tip[1] - axis[1], tip[0] - axis[0])
        angle = prior
        if self.settings.pointer == "radon":
            angle, _score = estimate_pointer_angle(
                warped, tuple(axis), prior, template.radius)
        tip_point = _template_point(axis, template.radius, angle)
        fraction, on_scale, angle_deg, _amin, _amax, _dir, _sweep, message = \
            compute_fraction(axis, tip_point, template.min_point,
                             template.max_point, self.settings.arc)
        vmin, vmax, unit = self.settings.range_for(detection.class_name)
        value = None
        if fraction is not None and on_scale:
            value = vmin + max(0.0, min(1.0, fraction)) * (vmax - vmin)
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
