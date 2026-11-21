#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""inspection_vision 节点的公共设施。

这个模块只依赖 cv2 / numpy / Pillow，不 import ROS、Tk 或 ultralytics
（ultralytics 只在真正加载模型时延迟 import），所以单元测试可以在没有
相机、模型和 ROS master 的环境里直接 import。

职责：
- 读取并合并 `config/inspection_vision.yaml` 配置；
- 相机/图片/视频帧源（单一采集进程，直接 cv2.VideoCapture）；
- YOLO 模型缓存（同步推理）；
- 在图上画框、写中文说明并落盘；
- 单次请求的上下文（截止时间、请求参数、结果日志）。
"""

from __future__ import annotations

import json
import os
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:  # Pillow 只用于画中文标签；没有时退化为无文字框
    from PIL import Image, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except Exception:  # pragma: no cover - 取决于运行环境
    Image = ImageDraw = ImageFont = None
    PIL_AVAILABLE = False


MODULE_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = MODULE_DIR.parent
DEFAULT_CONFIG_PATH = PACKAGE_DIR / "config" / "inspection_vision.yaml"
DEFAULT_MODEL_DIR = Path.home() / "AI视觉"
DEFAULT_OUTPUT_DIR = Path.home() / "Pictures" / "inspection_2026"


def default_gauge_model() -> Path:
    """源码布局在 models/ 下，catkin install 布局在脚本同级；两个都找。"""
    candidates = (PACKAGE_DIR / "models" / "gauge_exp16.pt",
                  PACKAGE_DIR / "gauge_exp16.pt")
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[0]


DEFAULT_GAUGE_MODEL = default_gauge_model()

CJK_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "/usr/share/fonts/truetype/arphic/ukai.ttc",
)

VIDEO_SUFFIXES = (".avi", ".mp4", ".mkv", ".mov", ".webm", ".h264")
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


# ---------------------------------------------------------------------------
# 异常
# ---------------------------------------------------------------------------

class VisionError(RuntimeError):
    """识别失败的统一基类；message 会写进 result/extra 供现场排查。"""


class VisionTimeout(VisionError):
    """本次请求的截止时间已到。"""


class CameraUnavailable(VisionError):
    """相机打不开或读不到画面。"""


class ModelUnavailable(VisionError):
    """模型文件缺失 / 推理环境不可用。"""


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------

DEFAULTS: Dict[str, Any] = {
    "camera": {
        "device": 0,
        "width": 2592,
        "height": 1944,
        "fps": 15,
        "pixel_format": "mjpg",
        "mirror": False,
        "sharpness": 32,
        "read_warmup": 2,
    },
    "output": {
        "dir": str(DEFAULT_OUTPUT_DIR),
        "jpeg_quality": 95,
        "save_raw": True,
    },
    "models": {
        "dir": str(DEFAULT_MODEL_DIR),
        "crowd": "people.pt",
        "sign": "Signal.pt",
        "sign_confirm": "stop.pt",
        "sign_alt": "模型/jinzhi.pt",
        "light": "light.pt",
        "trash_bins": "trash_brest.pt",
        "trash_items": "trash.pt",
        "fire": "fire.pt",
        "nameplate": "house.pt",
        "plate": "plate.pt",
        "bike_state": "best.pt",
        "bike_count": "Cycle.pt",
        "gauge_exp16": "",
    },
    "thresholds": {
        "default_conf": 0.25,
        "crowd": 0.25,
        "light": 0.45,
        "sign": 0.60,
        "trash_bins": 0.30,
        "trash_items": 0.20,
        "fire": 0.25,
        "nameplate": 0.35,
        "plate": 0.35,
        "ebike": 0.30,
    },
    "timeouts": {
        "default": 6.0,
        "crowd": 6.0,
        "sign": 6.0,
        "light": 3.0,
        "trash": 8.0,
        "fire": 8.0,
        "plate": 8.0,
        "temperature": 8.0,
        "meter": 15.0,
        "ebike": 8.0,
        "ebike_pan": 150.0,
    },
    "frames": {
        "default": 2,
        "crowd": 2,
        "sign": 3,
        "light": 3,
        "trash": 2,
        "fire": 2,
        "plate": 2,
        "temperature": 2,
        "ebike": 2,
    },
    "sign": {
        "route_map": {"A": "left", "B": "straight"},
        "use_confirm_model": True,
        "accept_alt_model": False,
    },
    "crowd": {
        "classes": ["people", "bad"],
    },
    "trash": {
        "bins_per_frame": 4,
        "crop_recheck_below": 0.5,
    },
    "fire": {
        "buildings": {
            "7": {"name": "A", "candidates": []},
            "8": {"name": "C", "candidates": []},
            "17": {"name": "B", "candidates": []},
        },
    },
    "plate": {
        "ocr_mode": "generic",
        "text_pattern": "",
    },
    "thermal": {
        "port": "/dev/thermal",
        "capture": "",
        "floor_table": "4F:0-5,3F:6-11,2F:12-17,1F:18-23",
        "delta": 2.0,
        "scale": 12,
        "capture_timeout": 3.0,
    },
    "meter": {
        "model": "",
        "temp_auto_ring": True,
        "humidity_auto_scale": False,
        "min_frames": 10,
        "max_frames": 20,
        "frame_interval": 0.12,
        "sha256": "5415e9abcc92ad1d5c0e62a380d861c51bb2a4931edaa0f55099b7134d824cc3",
    },
    "ebike": {
        "street_model": "crowd",
        "street_classes": [],
        "parking_pan": {
            "duration": 8.0,
            "fps": 15,
            "min_frames": 5,
        },
    },
}


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    merged = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def default_config_candidates() -> List[Path]:
    """源码运行 / catkin install 两种布局下的配置位置。"""
    return [
        DEFAULT_CONFIG_PATH,
        PACKAGE_DIR.parent.parent / "share" / "photo_capture" / "config"
        / "inspection_vision.yaml",
    ]


def load_config(path: Optional[Any] = None) -> Dict[str, Any]:
    """读取 YAML 配置并合并默认值；文件不存在时只用默认值。"""
    config = json.loads(json.dumps(DEFAULTS))  # 深拷贝，避免模块级默认值被改
    if path:
        config_path = Path(path).expanduser()
    else:
        config_path = next((candidate for candidate in default_config_candidates()
                            if candidate.is_file()), DEFAULT_CONFIG_PATH)
    if config_path.is_file():
        try:
            import yaml
            with open(str(config_path), "r", encoding="utf-8") as handle:
                loaded = yaml.safe_load(handle) or {}
            if not isinstance(loaded, dict):
                raise ValueError("YAML 顶层必须是映射（dict）")
            config = _deep_merge(config, loaded)
        except Exception as error:  # noqa: BLE001
            raise VisionError("读取配置失败 {}：{}".format(config_path, error))
    config["_config_path"] = str(config_path)
    config["_config_found"] = config_path.is_file()
    return config


def as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def as_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def safe_name(text: Any, limit: int = 40) -> str:
    """把点位/子任务名清理成安全文件名片段（保留中文）。"""
    cleaned = re.sub(r"[\s/\\:*?\"<>|\x00-\x1f]+", "_", str(text or "")).strip("_")
    return (cleaned[:limit] or "task")


def parse_params(text: Any) -> Tuple[Dict[str, Any], str]:
    """解析 srv 里的 params JSON 字符串。

    返回 (params, error)：JSON 非法时返回 ({}, 原因)，由调用方决定是回退到
    默认值还是直接判失败。空字符串/None 视为 {}。
    """
    raw = str(text or "").strip()
    if not raw:
        return {}, ""
    try:
        parsed = json.loads(raw)
    except Exception as error:  # noqa: BLE001
        return {}, "params 不是合法 JSON：{}".format(error)
    if not isinstance(parsed, dict):
        return {}, "params 必须是 JSON 对象"
    return parsed, ""


# ---------------------------------------------------------------------------
# 相机 / 帧源
# ---------------------------------------------------------------------------

def parse_camera_source(text: Any) -> Any:
    """把 0 / '0' / /dev/video0 / 文件路径 统一成 VideoCapture 能吃的参数。"""
    if isinstance(text, int):
        return text
    value = str(text).strip()
    if not value:
        return 0
    if value.isdigit():
        return int(value)
    match = re.fullmatch(r"/dev/video(\d+)", value)
    if match:
        return int(match.group(1))
    return value


class FrameSource:
    """单进程帧源：摄像头、图片文件夹、单张图片、视频文件都能读。

    摄像头与文件共用 cv2.VideoCapture；文件夹按文件名排序循环取图。
    摄像头保持常开，避免每次请求重新协商分辨率；文件源每次 read 取下一帧。
    """

    def __init__(self, source: Any = 0, width: int = 1280, height: int = 720,
                 fps: float = 30.0, mirror: bool = False, sharpness: int = 32,
                 pixel_format: str = "mjpg", loop: bool = True,
                 warmup: int = 2, logger=None):
        self.source = parse_camera_source(source)
        self.width = int(width or 0)
        self.height = int(height or 0)
        self.fps = float(fps or 0.0)
        self.mirror = bool(mirror)
        self.sharpness = int(sharpness)
        self.pixel_format = str(pixel_format or "auto").lower()
        self.loop = bool(loop)
        self.warmup = max(0, int(warmup))
        self.log = logger or (lambda _text: None)

        self._cap: Optional[cv2.VideoCapture] = None
        self._mode = ""
        self._images: List[Path] = []
        self._image_index = 0
        self._single_image: Optional[np.ndarray] = None
        self._original_sharpness: Optional[float] = None
        self.frame_size: Tuple[int, int] = (0, 0)
        self.description = ""
        self._lock = threading.Lock()

    # -- 打开 / 探测 ---------------------------------------------------
    def _prepare(self) -> None:
        source = self.source
        if isinstance(source, str):
            path = Path(source).expanduser()
            if path.is_dir():
                self._images = sorted(
                    p for p in path.iterdir()
                    if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES)
                if not self._images:
                    raise CameraUnavailable("图片文件夹里没有图片：{}".format(path))
                self._mode = "dir"
                self.description = "图片文件夹 {}（{} 张）".format(path, len(self._images))
                return
            if not path.exists():
                raise CameraUnavailable("帧源不存在：{}".format(path))
            if path.suffix.lower() in VIDEO_SUFFIXES:
                self._mode = "video"
                self.description = "视频文件 {}".format(path)
            elif path.suffix.lower() in IMAGE_SUFFIXES:
                image = cv2.imread(str(path))
                if image is None:
                    raise CameraUnavailable("读不了图片：{}".format(path))
                self._single_image = image
                self._mode = "image"
                self.description = "单张图片 {}".format(path)
                return
            else:
                raise CameraUnavailable("不支持的帧源：{}".format(path))
        else:
            self._mode = "camera"
            self.description = "摄像头 /dev/video{}".format(source)

        if self._mode in ("camera", "video"):
            self._open_capture()

    def _open_capture(self) -> None:
        cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            cap.release()
            raise CameraUnavailable(
                "无法打开帧源（{}）。检查相机是否插好/被占用，"
                "或用 ls /dev/video* 确认设备号。".format(self.source))
        try:
            if self._mode == "camera":
                fourcc = {"mjpg": "MJPG", "yuyv": "YUYV"}.get(self.pixel_format)
                if fourcc is None and (self.width >= 1280 or self.height >= 720):
                    fourcc = "MJPG"
                if fourcc:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                if self.width > 0:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                if self.height > 0:
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                if self.fps > 0:
                    cap.set(cv2.CAP_PROP_FPS, self.fps)
                if self.sharpness >= 0:
                    try:
                        self._original_sharpness = cap.get(cv2.CAP_PROP_SHARPNESS)
                        cap.set(cv2.CAP_PROP_SHARPNESS, self.sharpness)
                    except Exception:  # noqa: BLE001
                        self._original_sharpness = None
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except cv2.error:
                    pass
        except cv2.error:
            pass

        frame = None
        for _ in range(max(1, self.warmup + 1)):
            ok, current = cap.read()
            if ok and current is not None:
                frame = current
        if frame is None:
            cap.release()
            raise CameraUnavailable(
                "帧源已打开但读不到画面（{}），可能被其他程序占用。".format(self.source))
        self._cap = cap
        self.frame_size = (int(frame.shape[1]), int(frame.shape[0]))
        if self._mode == "camera":
            self.description = "摄像头 /dev/video{}（{}x{}）".format(
                self.source, self.frame_size[0], self.frame_size[1])
        else:
            self.description = "视频文件 {}（{}x{}）".format(
                self.source, self.frame_size[0], self.frame_size[1])

    def open(self) -> "FrameSource":
        with self._lock:
            if self._cap is not None or self._mode in ("dir", "image"):
                return self
            self._prepare()
            self.log("[vision] 帧源：%s" % self.description)
            return self

    # -- 读帧 ---------------------------------------------------------
    def _read_locked(self) -> Optional[np.ndarray]:
        if self._mode == "image":
            return None if self._single_image is None else self._single_image.copy()
        if self._mode == "dir":
            if not self._images:
                return None
            path = self._images[self._image_index % len(self._images)]
            return cv2.imread(str(path))
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        if not ok or frame is None:
            if self._mode == "video" and self.loop:
                try:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                except cv2.error:
                    pass
                ok, frame = self._cap.read()
            if not ok or frame is None:
                return None
        if self._mode == "camera":
            self.frame_size = (int(frame.shape[1]), int(frame.shape[0]))
        if self.mirror:
            frame = cv2.flip(frame, 1)
        return frame

    def read(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._mode == "":
                self.open()
            return self._read_locked()

    def advance(self) -> None:
        """图片文件夹模式：一次请求结束后换下一张，避免一次识别混用多张图。"""
        with self._lock:
            if self._mode == "dir" and self._images:
                self._image_index += 1

    def grab(self, count: int = 1, interval: float = 0.1,
             warmup: Optional[int] = None) -> List[np.ndarray]:
        """连续取 count 帧；先丢 warmup 帧，避免拿到相机缓冲里的旧画面。"""
        count = max(1, int(count))
        interval = max(0.0, float(interval))
        frames: List[np.ndarray] = []
        if warmup is None:
            warmup = self.warmup
        for _ in range(max(0, int(warmup))):
            self.read()
        for index in range(count):
            frame = self.read()
            if frame is None:
                raise CameraUnavailable("读取画面失败（第 {} 帧）".format(index + 1))
            frames.append(frame)
            if index + 1 < count and interval > 0:
                time.sleep(interval)
        return frames

    # -- 关闭 ---------------------------------------------------------
    def close(self) -> None:
        with self._lock:
            if self._cap is not None:
                if self._original_sharpness is not None:
                    try:
                        self._cap.set(cv2.CAP_PROP_SHARPNESS, self._original_sharpness)
                    except Exception:  # noqa: BLE001
                        pass
                    self._original_sharpness = None
                try:
                    self._cap.release()
                except Exception:  # noqa: BLE001
                    pass
            self._cap = None
            self._single_image = None
            self._images = []


# ---------------------------------------------------------------------------
# 检测框 / 模型
# ---------------------------------------------------------------------------

@dataclass
class Detection:
    """一个检测框（原图坐标）。"""

    x1: float
    y1: float
    x2: float
    y2: float
    conf: float
    cls_id: int
    name: str

    @property
    def box(self) -> Tuple[int, int, int, int]:
        return (int(round(self.x1)), int(round(self.y1)),
                int(round(self.x2)), int(round(self.y2)))

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def center(self) -> Tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    @property
    def bottom_center(self) -> Tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, self.y2)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "box": [round(self.x1, 1), round(self.y1, 1),
                    round(self.x2, 1), round(self.y2, 1)],
            "conf": round(float(self.conf), 4),
            "class": self.name,
        }

    def iou(self, other: "Detection") -> float:
        ix1 = max(self.x1, other.x1)
        iy1 = max(self.y1, other.y1)
        ix2 = min(self.x2, other.x2)
        iy2 = min(self.y2, other.y2)
        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        union = self.area + other.area - inter
        return inter / union if union > 0 else 0.0

    def overlap_ratio(self, other: "Detection") -> float:
        """本框面积落在 other 里的比例，用来把垃圾分配给垃圾桶。"""
        ix1 = max(self.x1, other.x1)
        iy1 = max(self.y1, other.y1)
        ix2 = min(self.x2, other.x2)
        iy2 = min(self.y2, other.y2)
        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        return inter / self.area if self.area > 0 else 0.0


class YoloModel:
    """同步 YOLO 封装：模型只加载一次，predict 可覆盖 conf/imgsz/classes。"""

    def __init__(self, path: Path, device: Any = "cpu", imgsz: int = 640,
                 conf: float = 0.25, max_det: int = 100):
        try:
            from ultralytics import YOLO
        except Exception as error:  # noqa: BLE001
            raise ModelUnavailable(
                "无法导入 ultralytics：{}（请用 ~/baidu-aip-env 的 python 运行）".format(error))
        if not Path(path).is_file():
            raise ModelUnavailable("模型文件不存在：{}".format(path))
        started = time.monotonic()
        try:
            self.model = YOLO(str(path))
        except Exception as error:  # noqa: BLE001
            raise ModelUnavailable("模型加载失败 {}：{}".format(path, error))
        self.path = Path(path)
        self.load_seconds = time.monotonic() - started
        self.device = device
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.max_det = int(max_det)
        raw_names = getattr(self.model, "names", {}) or {}
        if isinstance(raw_names, dict):
            self.names = {int(key): str(value) for key, value in raw_names.items()}
        else:
            self.names = {index: str(name) for index, name in enumerate(raw_names)}

    def predict(self, frame: np.ndarray, conf: Optional[float] = None,
                imgsz: Optional[int] = None, classes: Optional[Iterable[str]] = None,
                max_det: Optional[int] = None) -> List[Detection]:
        if frame is None or getattr(frame, "size", 0) == 0:
            raise VisionError("推理输入是空图")
        kwargs: Dict[str, Any] = {
            "imgsz": int(imgsz or self.imgsz),
            "conf": float(self.conf if conf is None else conf),
            "verbose": False,
            "max_det": int(self.max_det if max_det is None else max_det),
        }
        if self.device not in (None, "", "auto"):
            kwargs["device"] = self.device
        try:
            result = self.model.predict(frame, **kwargs)[0]
        except Exception as error:  # noqa: BLE001
            raise VisionError("推理失败 {}：{}".format(self.path.name, error))
        return self._extract(result, classes)

    def _extract(self, result, classes: Optional[Iterable[str]]) -> List[Detection]:
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []
        wanted = None
        if classes:
            wanted = {str(name) for name in classes}
        try:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            class_ids = boxes.cls.cpu().numpy().astype(int)
        except Exception:  # noqa: BLE001
            return []
        detections: List[Detection] = []
        for index in range(len(confs)):
            cls_id = int(class_ids[index])
            name = self.names.get(cls_id, str(cls_id))
            if wanted is not None and name not in wanted:
                continue
            x1, y1, x2, y2 = [float(value) for value in xyxy[index][:4]]
            if x2 <= x1 or y2 <= y1:
                continue
            detections.append(Detection(x1, y1, x2, y2, float(confs[index]), cls_id, name))
        return detections


class ModelCache:
    """按 (路径, imgsz) 缓存模型；服务本身串行，所以不需要推理锁。"""

    def __init__(self, model_dir: Optional[Any] = None, device: Any = "cpu",
                 logger=None):
        self.model_dir = Path(model_dir).expanduser() if model_dir else DEFAULT_MODEL_DIR
        self.device = device
        self.log = logger or (lambda _text: None)
        self._models: Dict[Tuple[str, int], YoloModel] = {}
        self._lock = threading.Lock()

    def resolve(self, value: Any) -> Path:
        if value is None or str(value).strip() == "":
            raise ModelUnavailable("没有配置模型路径")
        path = Path(str(value)).expanduser()
        if not path.is_absolute():
            path = self.model_dir / path
        return path

    def get(self, value: Any, imgsz: int = 640, conf: float = 0.25) -> YoloModel:
        path = self.resolve(value)
        key = (str(path), int(imgsz))
        with self._lock:
            model = self._models.get(key)
            if model is not None:
                return model
        self.log("[vision] 加载模型 %s（imgsz=%d）..." % (path.name, imgsz))
        model = YoloModel(path, device=self.device, imgsz=imgsz, conf=conf)
        self.log("[vision] 模型 %s 就绪（%.2f s，%d 类）"
                 % (path.name, model.load_seconds, len(model.names)))
        with self._lock:
            self._models[key] = model
        return model

    def loaded(self) -> List[str]:
        with self._lock:
            return sorted({Path(key[0]).name for key in self._models})


# ---------------------------------------------------------------------------
# 画图 / 存图
# ---------------------------------------------------------------------------

_FONT_CACHE: Dict[int, Any] = {}
_PALETTE = (
    (230, 70, 70), (70, 160, 230), (80, 190, 120), (240, 170, 60),
    (170, 110, 220), (240, 100, 180), (90, 200, 200), (200, 200, 80),
)


def find_cjk_font() -> Optional[str]:
    for candidate in CJK_FONT_CANDIDATES:
        if os.path.isfile(candidate):
            return candidate
    return None


def _font(size: int = 24):
    if not PIL_AVAILABLE:
        return None
    size = max(8, int(size))
    if size in _FONT_CACHE:
        return _FONT_CACHE[size]
    font = None
    font_path = find_cjk_font()
    if font_path:
        try:
            font = ImageFont.truetype(font_path, size)
        except Exception:  # noqa: BLE001
            font = None
    if font is None:
        try:
            font = ImageFont.load_default()
        except Exception:  # noqa: BLE001
            font = None
    _FONT_CACHE[size] = font
    return font


def annotate(frame: np.ndarray, detections: Sequence[Detection] = (),
             labels: Optional[Sequence[str]] = None,
             caption: Optional[str] = None,
             extra_lines: Sequence[str] = (),
             draw_boxes: bool = True) -> np.ndarray:
    """在 BGR 图上画检测框和中文说明，返回新图（不修改输入）。"""
    if frame is None:
        raise VisionError("要标注的图片是空的")
    output = frame.copy()
    if not PIL_AVAILABLE:
        if draw_boxes:
            for det in detections:
                x1, y1, x2, y2 = det.box
                cv2.rectangle(output, (x1, y1), (x2, y2),
                              _PALETTE[det.cls_id % len(_PALETTE)], 2)
        return output

    pil = Image.fromarray(cv2.cvtColor(output, cv2.COLOR_BGR2RGB))
    drawer = ImageDraw.Draw(pil, "RGBA")
    width, height = pil.size
    box_font = _font(max(12, min(30, int(height / 32))))
    text_font = _font(max(16, min(38, int(height / 24))))

    if draw_boxes:
        for index, det in enumerate(detections):
            x1, y1, x2, y2 = det.box
            color = _PALETTE[det.cls_id % len(_PALETTE)]
            drawer.rectangle([x1, y1, x2, y2], outline=color, width=3)
            if box_font is None:
                continue
            text = labels[index] if labels is not None and index < len(labels) \
                else "{} {:.2f}".format(det.name, det.conf)
            bbox = drawer.textbbox((0, 0), text, font=box_font)
            tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
            tx = min(max(0, x1), max(0, width - tw - 8))
            ty = y1 - th - 6
            if ty < 0:
                ty = min(max(0, y1 + 2), max(0, height - th - 6))
            drawer.rectangle([tx, ty, tx + tw + 8, ty + th + 6], fill=color + (235,))
            text_color = (0, 0, 0) if sum(color) > 420 else (255, 255, 255)
            drawer.text((tx + 3, ty + 3 - bbox[1]), text, font=box_font,
                        fill=text_color)

    lines = [line for line in list(extra_lines) + ([caption] if caption else [])
             if line]
    if lines and text_font is not None:
        pad = 8
        line_height = 0
        widths = []
        for line in lines:
            bbox = drawer.textbbox((0, 0), line, font=text_font)
            widths.append(bbox[2] - bbox[0])
            line_height = max(line_height, bbox[3] - bbox[1] + 6)
        block_w = min(width - 2 * pad, max(widths) + 2 * pad)
        block_h = line_height * len(lines) + pad
        block_h = min(block_h, height)
        drawer.rectangle([0, 0, block_w, block_h], fill=(0, 0, 0, 165))
        y = pad
        for line, text_w in zip(lines, widths):
            if y + line_height > height:
                break
            drawer.text((pad, y), line, font=text_font, fill=(255, 255, 255))
            y += line_height
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def encode_jpeg(frame: np.ndarray, quality: int = 95) -> Optional[bytes]:
    ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY),
                                              int(quality)])
    return buffer.tobytes() if ok else None


def save_image(path: Path, frame: np.ndarray, quality: int = 95) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    return bool(cv2.imwrite(str(path), frame,
                            [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]))


# ---------------------------------------------------------------------------
# 单次请求的结果与上下文
# ---------------------------------------------------------------------------

@dataclass
class TaskResult:
    ok: bool = False
    result: str = ""
    extra: Dict[str, Any] = field(default_factory=dict)
    images: List[str] = field(default_factory=list)
    error: str = ""


class VisionContext:
    """一次 /inspection_vision 调用的上下文（请求级状态都挂在这里）。"""

    def __init__(self, config: Dict[str, Any], models: ModelCache,
                 frames: FrameSource, output_root: Optional[Path] = None,
                 log=None, package_dir: Optional[Path] = None):
        self.config = config
        self.models = models
        self.frames = frames
        self.log = log or (lambda _text: None)
        self.package_dir = package_dir or PACKAGE_DIR
        base = Path(output_root or config["output"]["dir"]).expanduser().resolve()
        self.run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = base / self.run_stamp
        self.jsonl_path = self.run_dir / "recognitions.jsonl"
        self.jpeg_quality = int(config["output"].get("jpeg_quality", 95))
        self.save_raw_enabled = bool(config["output"].get("save_raw", True))

        # 请求级
        self.request_id = ""
        self.task = ""
        self.point_id = 0
        self.sub_id = ""
        self.params: Dict[str, Any] = {}
        self.deadline: Optional[float] = None
        self._request_started: Optional[float] = None
        self._request_index = 0
        self._models_used: List[str] = []
        self.metrics: List[Dict[str, Any]] = []
        self._log_lock = threading.Lock()

    # -- 请求生命周期 -------------------------------------------------
    def begin_request(self, task: str, point_id: int, sub_id: str,
                      params: Dict[str, Any], request_id: str,
                      timeout: float) -> None:
        self.request_id = request_id
        self.task = task
        self.point_id = int(point_id or 0)
        self.sub_id = str(sub_id or "")
        self.params = dict(params or {})
        self._request_started = time.monotonic()
        self.deadline = self._request_started + max(0.5, float(timeout))
        self._request_index += 1
        self._models_used = []

    def started(self) -> float:
        return self._request_started if self._request_started is not None else time.monotonic()

    def elapsed(self) -> float:
        return time.monotonic() - self.started()

    def remaining(self) -> float:
        if self.deadline is None:
            return 1e9
        return self.deadline - time.monotonic()

    def check_deadline(self, what: str = "") -> None:
        if self.deadline is not None and time.monotonic() > self.deadline:
            raise VisionTimeout(
                "请求超时{}{}".format("（" + what + "）" if what else "",
                                     "，已用 %.1fs" % self.elapsed()))

    def note_model(self, model: Any) -> None:
        self.note_model_name(Path(getattr(model, "path", "")).name)

    def note_model_name(self, name: str) -> None:
        if name and name not in self._models_used:
            self._models_used.append(str(name))

    # -- 取帧 / 模型 --------------------------------------------------
    def grab(self, count: Optional[int] = None, interval: Optional[float] = None,
             warmup: Optional[int] = None) -> List[np.ndarray]:
        override = self.params.get("frames")
        if override is not None:
            try:
                count = max(1, min(30, int(override)))
            except (TypeError, ValueError):
                pass
        count = int(count or self.config["frames"].get(self.task,
                                                       self.config["frames"]["default"]))
        if interval is None:
            interval = 0.1
        frames = self.frames.grab(count, interval, warmup=warmup)
        self.frames.advance()
        self.check_deadline("取帧")
        return frames

    def first_frame(self) -> np.ndarray:
        return self.grab(1)[0]

    def model(self, value: Any, imgsz: int = 640, conf: Optional[float] = None) -> YoloModel:
        if conf is None:
            conf = self.config["thresholds"].get(self.task,
                                                 self.config["thresholds"]["default_conf"])
        model = self.models.get(value, imgsz=imgsz, conf=conf)
        self.note_model(model)
        return model

    def model_path(self, key: str) -> str:
        return self.config["models"].get(key, "")

    def threshold(self, key: str, default: float) -> float:
        """阈值：params.conf 可覆盖本任务的配置值（现场调参不用改 YAML）。"""
        value = self.params.get("conf")
        if value is None:
            value = self.config["thresholds"].get(
                key, self.config["thresholds"].get("default_conf", default))
        return as_float(value, default)

    # -- 输出 ---------------------------------------------------------
    def _file_stem(self, tag: str) -> str:
        stamp = datetime.now().strftime("%H%M%S_%f")[:-3]
        sub = safe_name(self.sub_id or "-", 16)
        return "p%02d_%s_%s_%s" % (self.point_id, self.task, sub, tag) + "_" + stamp

    def save_frame(self, frame: np.ndarray, tag: str = "frame",
                   detections: Sequence[Detection] = (),
                   labels: Optional[Sequence[str]] = None,
                   caption: Optional[str] = None,
                   extra_lines: Sequence[str] = (),
                   annotate_frame: bool = True) -> str:
        """把（带框的）图写到本轮目录并返回绝对路径。"""
        self.check_deadline("存图")
        if annotate_frame:
            image = annotate(frame, detections, labels=labels,
                             caption=caption, extra_lines=extra_lines)
        else:
            image = frame
        path = self.run_dir / (self._file_stem(tag) + ".jpg")
        if not save_image(path, image, self.jpeg_quality):
            raise VisionError("图片写入失败：{}".format(path))
        return str(path)

    def save_raw(self, frame: np.ndarray, tag: str = "raw") -> Optional[str]:
        if not self.save_raw_enabled:
            return None
        path = self.run_dir / (self._file_stem(tag) + ".jpg")
        if not save_image(path, frame, self.jpeg_quality):
            return None
        return str(path)

    def ok(self, result: str, extra: Optional[Dict[str, Any]] = None,
           images: Optional[Sequence[str]] = None) -> TaskResult:
        payload = dict(extra or {})
        payload.setdefault("request_id", self.request_id)
        return TaskResult(True, result, payload,
                          [path for path in (images or []) if path])

    def fail(self, error: str, result: str = "",
             extra: Optional[Dict[str, Any]] = None) -> TaskResult:
        payload = dict(extra or {})
        payload.setdefault("request_id", self.request_id)
        payload["error"] = error
        return TaskResult(False, result or "", payload, [], error=error)

    # -- 日志 ---------------------------------------------------------
    def finish(self, result: TaskResult) -> TaskResult:
        elapsed = self.elapsed()
        result.extra.setdefault("elapsed", round(elapsed, 3))
        entry = {
            "stamp": datetime.now().isoformat(timespec="milliseconds"),
            "run": self.run_stamp,
            "request_id": self.request_id,
            "point": self.point_id,
            "task": self.task,
            "sub_id": self.sub_id,
            "ok": bool(result.ok),
            "result": result.result,
            "extra": result.extra,
            "images": list(result.images),
            "models": list(self._models_used),
            "elapsed": round(elapsed, 3),
            "error": result.error,
        }
        self.metrics.append(entry)
        try:
            self.run_dir.mkdir(parents=True, exist_ok=True)
            with self._log_lock:
                with open(str(self.jsonl_path), "a", encoding="utf-8") as handle:
                    handle.write(json.dumps(entry, ensure_ascii=False,
                                             default=str) + "\n")
        except OSError as error:  # noqa: BLE001
            self.log("[vision] 结果日志写入失败：%s" % error)
        tag = "OK" if result.ok else "FAIL"
        self.log("[识别] %s point=%s task=%s sub=%s req=%s %.2fs → %s%s"
                 % (tag, self.point_id, self.task, self.sub_id or "-",
                    self.request_id or "-", elapsed, result.result or "(空)",
                    "" if result.ok else " | " + (result.error or "未识别")))
        return result

    def summary(self) -> Dict[str, Any]:
        by_task: Dict[str, Dict[str, Any]] = {}
        for entry in self.metrics:
            bucket = by_task.setdefault(entry["task"], {"count": 0, "ok": 0,
                                                        "elapsed": 0.0})
            bucket["count"] += 1
            bucket["ok"] += 1 if entry["ok"] else 0
            bucket["elapsed"] += entry["elapsed"]
        for bucket in by_task.values():
            bucket["avg_elapsed"] = round(bucket["elapsed"] / max(1, bucket["count"]), 3)
            bucket["success_rate"] = round(bucket["ok"] / max(1, bucket["count"]), 3)
        return {"run": self.run_stamp, "log": str(self.jsonl_path),
                "total": len(self.metrics), "by_task": by_task}
