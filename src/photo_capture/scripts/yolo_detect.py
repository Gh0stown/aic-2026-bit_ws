#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO 模型加载和异步推理的轻量封装，供 capture_photos.py 的模型测试模式使用。

这个模块本身不依赖 ROS，也不强制在导入时加载 ultralytics：
只有真正创建 YoloDetector 时才会 import torch / ultralytics，
所以拍照模式不会因为没装 YOLO 而变慢或启动失败。
"""

import importlib.util
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


class ModelError(RuntimeError):
    """模型加载 / 推理环境错误。"""


def ultralytics_available() -> bool:
    """只检查包是否存在，不真正 import（避免拖慢启动）。"""
    try:
        return importlib.util.find_spec("ultralytics") is not None
    except Exception:
        return False


def list_model_files(model_dir) -> List[Path]:
    """列出模型目录下的 .pt 权重。"""
    root = Path(model_dir).expanduser()
    if not root.is_dir():
        return []
    return sorted(
        (path for path in root.glob("*.pt") if path.is_file()),
        key=lambda path: path.name.lower(),
    )


def resolve_model_path(value, model_dir) -> Optional[Path]:
    """把 --model 的值解析成真实路径：支持绝对/相对路径，也支持只写文件名。"""
    if not value:
        return None
    path = Path(value).expanduser()
    if path.is_file():
        return path
    candidate = Path(model_dir).expanduser() / value
    if candidate.is_file():
        return candidate
    return None


def human_size(path) -> str:
    try:
        size = Path(path).stat().st_size
    except OSError:
        return "?"
    return "%.1f MB" % (size / 1024.0 / 1024.0)


def resolve_device(preference: str = "auto"):
    """返回 ultralytics 能接受的 device 值。"""
    if preference == "cpu":
        return "cpu"
    try:
        import torch

        has_cuda = torch.cuda.is_available()
    except Exception:
        has_cuda = False
    if preference == "cuda":
        if not has_cuda:
            raise ModelError(
                "CUDA 不可用：torch.cuda.is_available() 为 False，"
                "请改用 --infer-device cpu。"
            )
        return 0
    return 0 if has_cuda else "cpu"


def device_label(device) -> str:
    return "CPU" if device == "cpu" else "GPU"


class YoloDetector:
    """在后台线程里持续对最新一帧做推理，主线程只负责取结果。

    submit() 只保留最新的一帧：如果推理比摄像头慢，中间帧会被丢掉，
    这样界面预览不会被推理卡住，检测结果也始终是最新画面附近的结果。
    """

    def __init__(self, model_path, device_pref: str = "auto",
                 imgsz: int = 640, conf: float = 0.25):
        self.model_path = str(model_path)
        self.imgsz = max(64, int(imgsz))
        self.conf = max(0.01, min(0.99, float(conf)))
        self.device = resolve_device(device_pref)
        self.last_error = ""

        try:
            from ultralytics import YOLO
        except Exception as error:  # pragma: no cover - 取决于环境
            raise ModelError(
                "无法导入 ultralytics：{}\n"
                "可尝试：pip3 install ultralytics".format(error)
            )

        started = time.monotonic()
        try:
            self.model = YOLO(self.model_path)
        except Exception as error:
            raise ModelError(
                "模型加载失败：{}\n"
                "文件可能损坏、没下载完整，或者不是 YOLOv8/v11 权重。".format(error)
            )
        self.load_seconds = time.monotonic() - started

        raw_names = getattr(self.model, "names", {}) or {}
        if isinstance(raw_names, dict):
            self.names: Dict[int, str] = {
                int(key): str(value) for key, value in raw_names.items()
            }
        else:
            self.names = {index: str(name) for index, name in enumerate(raw_names)}

        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._pending = None  # (frame_id, frame)
        self._result = None  # (frame_id, boxes, infer_ms)
        self._running = True
        self._thread = threading.Thread(
            target=self._infer_loop, name="yolo-infer", daemon=True)
        self._thread.start()

    @property
    def class_count(self) -> int:
        return len(self.names)

    def submit(self, frame, frame_id: int) -> None:
        """提交一帧给后台推理；调用方保证 frame 之后不再修改。"""
        with self._condition:
            if not self._running:
                return
            self._pending = (int(frame_id), frame)
            self._condition.notify()

    def latest(self):
        """返回 (frame_id, boxes, infer_ms)，还没结果时返回 None。"""
        with self._lock:
            return self._result

    def set_conf(self, value: float) -> None:
        self.conf = max(0.01, min(0.99, float(value)))

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
                    imgsz=self.imgsz,
                    conf=self.conf,
                    device=self.device,
                    verbose=False,
                    max_det=100,
                )[0]
                boxes = self._extract_boxes(result)
                self.last_error = ""
            except Exception as error:  # pragma: no cover - 取决于模型/硬件
                boxes = []
                self.last_error = str(error)
            infer_ms = (time.monotonic() - started) * 1000.0

            with self._lock:
                if self._running:
                    self._result = (frame_id, boxes, infer_ms)

    @staticmethod
    def _extract_boxes(result) -> List[Tuple[float, float, float, float, float, int]]:
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []
        try:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy().astype(int)
        except Exception:
            return []
        output = []
        for index in range(len(confs)):
            x1, y1, x2, y2 = [float(value) for value in xyxy[index][:4]]
            output.append((x1, y1, x2, y2, float(confs[index]), int(classes[index])))
        return output

    def close(self) -> None:
        with self._condition:
            self._running = False
            self._pending = None
            self._condition.notify()
        self._thread.join(timeout=2.0)


@dataclass
class TiledResult:
    ok: bool
    boxes: List[Tuple[float, float, float, float, float, int]] = field(default_factory=list)
    names: Dict[int, str] = field(default_factory=dict)
    message: str = ""
    tiles: int = 0


def _iou(box_a, box_b) -> float:
    ax1, ay1, ax2, ay2 = box_a[:4]
    bx1, by1, bx2, by2 = box_b[:4]
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter = max(0.0, inter_x2 - inter_x1) * max(0.0, inter_y2 - inter_y1)
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def nms_class_aware(boxes, iou_threshold: float = 0.45):
    """按类别做 NMS，boxes 为 (x1,y1,x2,y2,conf,cls)。"""
    grouped: Dict[int, list] = {}
    for box in boxes:
        grouped.setdefault(int(box[5]), []).append(box)
    kept = []
    for _class_id, items in grouped.items():
        items = sorted(items, key=lambda item: item[4], reverse=True)
        while items:
            best = items.pop(0)
            kept.append(best)
            items = [item for item in items if _iou(best, item) <= iou_threshold]
    return sorted(kept, key=lambda item: item[4], reverse=True)


def detect_image_tiled(image, model_path, conf: float = 0.25, imgsz: int = 640,
                       device_pref: str = "auto", tile: int = 640,
                       overlap: float = 0.2, progress=None) -> TiledResult:
    """把大图切块后逐块 YOLO，再按类别 NMS 合并回原图坐标。"""
    if image is None or getattr(image, "size", 0) == 0:
        return TiledResult(False, message="图片为空")
    height, width = image.shape[:2]
    if height < 8 or width < 8:
        return TiledResult(False, message="图片太小")
    tile = max(128, min(1280, int(tile)))
    overlap = max(0.0, min(0.8, float(overlap)))
    step = max(1, int(round(tile * (1.0 - overlap))))
    xs = sorted(set(
        list(range(0, max(1, width - tile + 1), step)) +
        ([max(0, width - tile)] if width > tile else [0])))
    ys = sorted(set(
        list(range(0, max(1, height - tile + 1), step)) +
        ([max(0, height - tile)] if height > tile else [0])))
    total = len(xs) * len(ys)

    try:
        from ultralytics import YOLO
        model = YOLO(str(model_path))
    except Exception as error:
        raise ModelError("模型加载失败：{}".format(error))
    try:
        device = resolve_device(device_pref)
    except ModelError:
        raise
    raw_names = getattr(model, "names", {}) or {}
    names = ({int(key): str(value) for key, value in raw_names.items()}
             if isinstance(raw_names, dict)
             else {index: str(name) for index, name in enumerate(raw_names)})

    boxes = []
    done = 0
    for y in ys:
        for x in xs:
            crop = image[y:y + tile, x:x + tile]
            if crop.size == 0:
                done += 1
                continue
            try:
                result = model.predict(
                    crop, imgsz=imgsz, conf=conf, device=device,
                    verbose=False, max_det=100)[0]
            except Exception as error:
                return TiledResult(False, names=names, tiles=done,
                                   message="推理失败：{}".format(error))
            for x1, y1, x2, y2, score, class_id in YoloDetector._extract_boxes(result):
                x1 = max(0.0, min(float(width), x1 + x))
                x2 = max(0.0, min(float(width), x2 + x))
                y1 = max(0.0, min(float(height), y1 + y))
                y2 = max(0.0, min(float(height), y2 + y))
                if x2 > x1 and y2 > y1:
                    boxes.append((x1, y1, x2, y2, float(score), int(class_id)))
            done += 1
            if progress is not None:
                try:
                    progress(done, total)
                except Exception:
                    pass
    return TiledResult(True, boxes=nms_class_aware(boxes), names=names,
                       tiles=total, message="")
