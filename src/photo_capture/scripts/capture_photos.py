#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本机摄像头手动拍照采集脚本。

特点：
  * 启动时询问本次要拍的照片类型，照片保存到 <输出目录>/<类型>/ 下；
  * 一次拍照可以连拍 N 张（N 随时可改），也可以单张拍；
  * 运行中随时可以切换类型，换类型后自动建对应的文件夹；
  * 启动时可以选择「拍照模式」「模型测试模式」「区域扫描模式」「仪表识别模式」
    或「红外热像模式」；
    模型测试模式会加载 ~/AI视觉 下的 .pt 权重，实时画框显示识别结果；
    区域扫描模式控制麦轮底盘横向平移，逐帧拍摄并拼成横向大图；
    仪表识别模式加载 Pose.pt，识别温度/湿度表并计算指针读数；
    红外热像模式通过 mlx90640_ros.thermal_session 显示 MLX90640 热图，支持
    capture.bin 回放、串口连接、楼层标定和快照；
  * 有桌面环境时用 tkinter 做预览窗口（OpenCV 只负责读摄像头和写图片），
    没有图形界面时自动退化为命令行交互模式。

拍照模式快捷键（图形界面）：
  空格 / 回车  拍照（按设定张数连拍）
  T            更换类型
  N            修改一次连拍张数
  Esc          退出

模型测试模式快捷键（图形界面）：
  M            更换模型
  S            保存当前帧（带检测框）
  Esc          退出

区域扫描模式（图形界面）：
  开始/停止    平移采集或手动录制
  Esc          停止并退出
  拼接完成后可点「检测全景」做分块 YOLO

仪表识别模式（图形界面）：
  M            更换 Pose 模型
  S            保存当前帧（带关键点/读数）
  R            保存当前读数（JSONL）
  Esc          退出

命令行模式（--no-gui）：
  拍照模式：回车拍照，t 换类型，n 改张数，q 退出
  模型测试：实时打印检测结果，Ctrl+C 退出
  区域扫描：自动平移或手动录制，Ctrl+C/回车停止并拼接
  仪表识别：实时打印温度/湿度表读数，Ctrl+C 退出
"""

import argparse
import json
import os
import queue
import re
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple, Union

import cv2
import numpy as np

# 让脚本通过绝对路径 / rosrun 运行时都能 import 同目录的 yolo_detect
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

try:  # YOLO 检测模块；缺失时只影响模型测试模式
    import yolo_detect

    YOLO_MODULE_AVAILABLE = True
except Exception:  # pragma: no cover - 取决于运行环境
    yolo_detect = None
    YOLO_MODULE_AVAILABLE = False

try:  # 区域扫描模块；缺失时只影响区域扫描模式
    import scan as scan_module

    SCAN_MODULE_AVAILABLE = True
except Exception:  # pragma: no cover - 取决于运行环境
    scan_module = None
    SCAN_MODULE_AVAILABLE = False

try:  # 仪表 Pose 识别模块；缺失时只影响仪表模式
    import gauge as gauge_module

    GAUGE_MODULE_AVAILABLE = True
except Exception:  # pragma: no cover - 取决于运行环境
    gauge_module = None
    GAUGE_MODULE_AVAILABLE = False

try:  # 红外热像；缺失时只影响红外热像模式
    from mlx90640_ros.thermal_session import ThermalSession, FloorTable

    THERMAL_AVAILABLE = True
except Exception:  # pragma: no cover - 取决于运行环境
    ThermalSession = None
    FloorTable = None
    THERMAL_AVAILABLE = False

# find_spec 只查包在不在，不真正 import，启动不会被拖慢
YOLO_RUNTIME_AVAILABLE = bool(
    YOLO_MODULE_AVAILABLE and yolo_detect.ultralytics_available())

try:  # tkinter 可能没装，此时退回命令行模式
    import tkinter as tk
    import tkinter.font as tkfont
    from tkinter import filedialog, messagebox, ttk

    TK_AVAILABLE = True
except Exception:  # pragma: no cover - 取决于运行环境
    tk = None
    tkfont = None
    ttk = None
    filedialog = None
    messagebox = None
    TK_AVAILABLE = False

try:  # 预览和中文叠加文字依赖 Pillow
    from PIL import Image, ImageDraw, ImageFont, ImageTk

    PIL_AVAILABLE = True
except Exception:  # pragma: no cover - 取决于运行环境
    Image = ImageDraw = ImageFont = ImageTk = None
    PIL_AVAILABLE = False


APP_TITLE = "拍照采集"
DEFAULT_OUTPUT_DIR = Path.home() / "Pictures" / "photo_capture"
MAX_PREVIEW_W = 960
MAX_PREVIEW_H = 720
MAX_BURST = 99
MAX_TYPE_LEN = 40
DEFAULT_MODEL_DIR = Path.home() / "AI视觉"
DEFAULT_POSE_MODEL = DEFAULT_MODEL_DIR / "Pose.pt"
MODE_PHOTO = "photo"
MODE_DETECT = "detect"
MODE_SCAN = "scan"
MODE_GAUGE = "gauge"
MODE_THERMAL = "thermal"

CJK_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Light.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "/usr/share/fonts/truetype/arphic/ukai.ttc",
)


class CameraError(RuntimeError):
    """摄像头打开 / 读取失败。"""


# ---------------------------------------------------------------------------
# 通用小工具
# ---------------------------------------------------------------------------

def find_cjk_font() -> Optional[str]:
    """找一个能显示中文的字体文件，找不到就返回 None。"""
    for path in CJK_FONT_CANDIDATES:
        if os.path.isfile(path):
            return path
    return None


def sanitize_type(name: object) -> str:
    """把用户输入的类型名清理成安全的文件夹名。"""
    if name is None:
        return ""
    text = str(name).strip()
    text = text.replace("/", "_").replace("\\", "_")
    text = re.sub(r'[\x00-\x1f:*?"<>|]', "_", text)
    text = re.sub(r"\s+", " ", text).strip(" .")
    if len(text) > MAX_TYPE_LEN:
        text = text[:MAX_TYPE_LEN].rstrip(" .")
    return text


def list_existing_types(root: Union[str, Path]) -> List[str]:
    """列出输出目录下已有的类型文件夹（隐藏目录除外）。"""
    root = Path(root)
    try:
        entries = sorted(root.iterdir(), key=lambda p: p.name)
    except OSError:
        return []
    return [p.name for p in entries if p.is_dir() and not p.name.startswith(".")]


def parse_camera_source(text: object) -> Union[int, str]:
    """把 0 / /dev/video0 / 视频文件路径 统一成 VideoCapture 能吃的参数。"""
    source = str(text).strip()
    if source.isdigit():
        return int(source)
    match = re.fullmatch(r"/dev/video(\d+)", source)
    if match:
        return int(match.group(1))
    return source


def parse_count(text: object, default: int = 1) -> int:
    try:
        count = int(str(text).strip())
    except (TypeError, ValueError):
        return default
    return max(1, min(MAX_BURST, count))


def display_path(path: Union[str, Path]) -> str:
    """把家目录缩成 ~，让界面上的路径短一点。"""
    text = str(path)
    home = str(Path.home())
    if text == home:
        return "~"
    if text.startswith(home + os.sep):
        return "~" + text[len(home):]
    return text


def shorten(text: str, limit: int) -> str:
    if len(text) <= limit:
        return text
    return text[:max(1, limit - 1)] + "…"


def annotate_detections(image, boxes, names, font_size: int = 24):
    """在 BGR 图上画检测框，返回新的 BGR 图；没有 PIL 时原样返回。"""
    if not PIL_AVAILABLE or image is None:
        return image
    try:
        pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil)
        width, height = pil.size
        font = None
        font_path = find_cjk_font()
        if font_path:
            try:
                font = ImageFont.truetype(font_path, font_size)
            except Exception:
                font = None
        if font is None:
            try:
                font = ImageFont.load_default()
            except Exception:
                font = None
        palette = (
            (230, 70, 70), (70, 160, 230), (80, 190, 120), (240, 170, 60),
            (170, 110, 220), (240, 100, 180), (90, 200, 200), (200, 200, 80),
        )
        for box in boxes:
            x1, y1, x2, y2, conf, class_id = box
            x1, y1, x2, y2 = [int(round(v)) for v in (x1, y1, x2, y2)]
            color = palette[int(class_id) % len(palette)]
            draw.rectangle([x1, y1, x2, y2], outline=color, width=3)
            if font is None:
                continue
            label = "{} {:.2f}".format(names.get(int(class_id), str(class_id)), conf)
            bbox = draw.textbbox((0, 0), label, font=font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
            tx = min(max(0, x1), max(0, width - text_width - 8))
            ty = y1 - text_height - 6
            if ty < 0:
                ty = min(max(0, y1 + 2), max(0, height - text_height - 6))
            draw.rectangle([tx, ty, tx + text_width + 8, ty + text_height + 6],
                           fill=color)
            text_color = (0, 0, 0) if sum(color) > 420 else (255, 255, 255)
            draw.text((tx + 3, ty + 3 - bbox[1]), label,
                      font=font, fill=text_color)
        return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
    except Exception:
        return image


def save_detection_crops(frame, boxes, names, model_dir, prefix: str = "crop",
                         quality: int = 95, pad: int = 4, source: str = ""):
    """把检测框裁成单张小图，按类别建子文件夹。

    model_dir 由调用方给出（已经包含模型名），目录结构：
        <model_dir>/<类别名>/<prefix>_<时间戳>_<序号>.jpg
    同时在 model_dir 下写一份 manifest_<时间戳>.json。

    返回 (保存数量, manifest 路径或 None)。
    """
    if frame is None or not boxes:
        return 0, None
    model_dir = Path(model_dir)
    height, width = frame.shape[:2]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    entries = []
    for index, box in enumerate(boxes):
        x1, y1, x2, y2, conf, class_id = box
        x1 = max(0, min(width, int(round(x1)) - pad))
        y1 = max(0, min(height, int(round(y1)) - pad))
        x2 = max(0, min(width, int(round(x2)) + pad))
        y2 = max(0, min(height, int(round(y2)) + pad))
        if x2 <= x1 or y2 <= y1:
            continue
        class_name = names.get(int(class_id), str(class_id))
        folder = sanitize_type(class_name) or "class_{}".format(int(class_id))
        class_dir = model_dir / folder
        class_dir.mkdir(parents=True, exist_ok=True)
        filename = "{}_{}_{:03d}.jpg".format(prefix, stamp, index + 1)
        path = class_dir / filename
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0 or not cv2.imwrite(
                str(path), crop, [int(cv2.IMWRITE_JPEG_QUALITY), quality]):
            continue
        entries.append({
            "file": "{}/{}".format(folder, filename),
            "class_id": int(class_id),
            "class_name": class_name,
            "conf": round(float(conf), 4),
            "box": [x1, y1, x2, y2],
        })
    if not entries:
        return 0, None
    manifest = model_dir / "manifest_{}.json".format(stamp)
    try:
        with manifest.open("w", encoding="utf-8") as handle:
            json.dump({
                "source": source,
                "created_at": datetime.now().isoformat(timespec="seconds"),
                "frame_size": [width, height],
                "count": len(entries),
                "detections": entries,
            }, handle, ensure_ascii=False, indent=2)
    except OSError:
        manifest = None
    return len(entries), manifest


def has_display() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


# ---------------------------------------------------------------------------
# 摄像头
# ---------------------------------------------------------------------------

class Camera:
    def __init__(self, source: Union[int, str], width: int = 0, height: int = 0,
                 fps: float = 0.0, mirror: bool = False, sharpness: int = 3,
                 pixel_format: str = "auto"):
        self.source = source
        self.width = int(width or 0)
        self.height = int(height or 0)
        self.fps = float(fps or 0.0)
        self.mirror = bool(mirror)
        self.sharpness = int(sharpness)
        self.pixel_format = str(pixel_format).lower()
        self._original_sharpness = None
        self.cap = None
        self.frame_size = (0, 0)
        self.actual_fourcc = None
        self.actual_fps = None

    def open(self) -> None:
        """打开摄像头并确认能读到画面，失败时抛 CameraError。"""
        self.release()
        cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            cap.release()
            raise CameraError(
                "无法打开摄像头（设备：{}）。\n"
                "请检查摄像头是否插好、是否被其他程序占用；\n"
                "可以用 ls /dev/video* 查看设备号，再用 --device 1 指定。".format(self.source)
            )

        try:
            # 1080p/720p 在 USB2 UVC 上通常只有 MJPG 能跑 30fps；
            # YUYV 1920x1080 往往只有 5fps，既卡又容易糊。
            # 也允许强制 YUYV，对比不同电脑上的取流差异。
            if isinstance(self.source, int):
                if self.pixel_format == "mjpg":
                    cap.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*"MJPG"))
                elif self.pixel_format == "yuyv":
                    cap.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*"YUYV"))
                elif self.width >= 1280 or self.height >= 720:
                    cap.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*"MJPG"))
            if self.width > 0:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            if self.height > 0:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if self.fps > 0:
                cap.set(cv2.CAP_PROP_FPS, self.fps)
            if self.sharpness >= 0 and isinstance(self.source, int):
                try:
                    self._original_sharpness = cap.get(cv2.CAP_PROP_SHARPNESS)
                    cap.set(cv2.CAP_PROP_SHARPNESS, self.sharpness)
                except Exception:
                    self._original_sharpness = None
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except cv2.error:
                pass
        except cv2.error:
            pass

        frame = None
        for _ in range(6):
            ok, current = cap.read()
            if ok and current is not None:
                frame = current
        if frame is None:
            cap.release()
            raise CameraError(
                "摄像头已经打开，但读不到画面（设备：{}）。\n"
                "可能被其他程序占用，或设备号不对。".format(self.source)
            )

        self.cap = cap
        self.frame_size = (int(frame.shape[1]), int(frame.shape[0]))
        self.actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        self.actual_fps = cap.get(cv2.CAP_PROP_FPS)

    def read(self):
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        if self.mirror:
            frame = cv2.flip(frame, 1)
        return frame

    def release(self) -> None:
        if self.cap is not None:
            if self._original_sharpness is not None:
                try:
                    self.cap.set(cv2.CAP_PROP_SHARPNESS,
                                 self._original_sharpness)
                except Exception:
                    pass
                self._original_sharpness = None
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None


# ---------------------------------------------------------------------------
# 照片会话：类型 + 保存
# ---------------------------------------------------------------------------

class Session:
    def __init__(self, output_root: Union[str, Path], type_name: str,
                 count: int = 1, image_format: str = "jpg", quality: int = 95,
                 interval: float = 0.4):
        self.root = Path(output_root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.image_format = "png" if str(image_format).lower() == "png" else "jpg"
        self.quality = max(1, min(100, int(quality)))
        self.interval = max(0.0, float(interval))
        self.count = parse_count(count, 1)
        self.total = 0
        self.seq = 0
        self.type_name = ""
        self.set_type(type_name)

    @property
    def type_dir(self) -> Path:
        return self.root / self.type_name

    def set_type(self, name: str) -> str:
        cleaned = sanitize_type(name)
        if not cleaned:
            raise ValueError("照片类型不能为空")
        target = self.root / cleaned
        if target.exists() and not target.is_dir():
            raise OSError("{} 已存在且不是文件夹".format(target))
        target.mkdir(parents=True, exist_ok=True)
        self.type_name = cleaned
        return cleaned

    def save(self, frame) -> Path:
        """保存一帧，返回文件路径。"""
        self.seq += 1
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        ext = self.image_format
        path = self.type_dir / "{}_{}_{:04d}.{}".format(self.type_name, stamp, self.seq, ext)
        if ext == "jpg":
            params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        else:
            params = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
        if not cv2.imwrite(str(path), frame, params):
            raise OSError("写入图片失败：{}".format(path))
        self.total += 1
        return path


# ---------------------------------------------------------------------------
# 启动时 / 运行中的类型输入对话框（tkinter）
# ---------------------------------------------------------------------------

# 共用的 Tk 展示组件：不持有设备、任务或设置状态。
GUI_COLORS = {
    "bg": "#edf2f6", "card": "#ffffff", "ink": "#203247",
    "muted": "#687b8e", "line": "#dbe4ec", "navy": "#172b40",
    "accent": "#087f8c", "danger": "#b83d4c",
}


def _setup_gui_theme(widget):
    """仅配置 Tk 字体和本脚本使用的样式，可由任一入口独立调用。"""
    families = set(tkfont.families(widget))
    family = tkfont.Font(root=widget, name="TkDefaultFont", exists=True).actual("family")
    font_path = find_cjk_font()
    if font_path and PIL_AVAILABLE:
        try:
            # 与已有预览叠字共用字体发现入口；只选 Tk 实际安装的字体族。
            candidate = ImageFont.truetype(font_path, 12).getname()[0]
            if candidate in families:
                family = candidate
        except (OSError, ValueError):
            pass
    for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
        tkfont.Font(root=widget, name=name, exists=True).configure(family=family, size=10)
    c = GUI_COLORS
    style = ttk.Style(widget)
    if style.theme_use() != "clam":
        style.theme_use("clam")
    style.configure("TFrame", background=c["card"])
    style.configure("TLabel", background=c["card"], foreground=c["ink"])
    style.configure("Capture.Background.TFrame", background=c["bg"])
    style.configure("Capture.Card.TFrame", background=c["card"])
    style.configure("Capture.Section.TLabel", font=(family, 11, "bold"))
    style.configure("Capture.Muted.TLabel", foreground=c["muted"])
    style.configure("TButton", padding=(10, 5), background="#f1f5f9",
                    bordercolor=c["line"], focuscolor=c["accent"])
    style.map("TButton", background=[("active", "#e3edf4")],
              foreground=[("disabled", "#9ba9b6")])
    for name, color, hover in (("Accent", c["accent"], "#066b77"),
                                ("Danger", c["danger"], "#9f2f3d")):
        key = "Capture." + name + ".TButton"
        style.configure(key, background=color, foreground="white", bordercolor=color,
                        font=(family, 10, "bold"))
        style.map(key, background=[("disabled", "#e6ecf1"), ("active", hover)],
                  foreground=[("disabled", "#9ba9b6"), ("!disabled", "white")],
                  bordercolor=[("disabled", c["line"])])
    for key in ("TEntry", "TSpinbox", "TCombobox"):
        style.configure(key, padding=4, fieldbackground="white", bordercolor=c["line"])
    style.map("TCombobox", fieldbackground=[("readonly", "white")])
    for key in ("TCheckbutton", "TRadiobutton"):
        style.configure(key, background="white")
        style.map(key, background=[("active", "white")])
    style.configure("Horizontal.TProgressbar", background=c["accent"],
                    troughcolor="#e7eef3", borderwidth=0, thickness=5)
    widget.configure(background=c["bg"])
    return family


def _gui_header(parent, title, subtitle=""):
    family = _setup_gui_theme(parent)
    c = GUI_COLORS
    header = tk.Frame(parent, background=c["navy"], padx=18, pady=10)
    header.pack(fill="x")
    tk.Label(header, text=title, background=c["navy"], foreground="white",
             font=(family, 17, "bold")).pack(side="left")
    if subtitle:
        tk.Label(header, text=subtitle, background=c["navy"], foreground="#9fb5c9",
                 font=(family, 9)).pack(side="left", padx=14)
    return header


def _dialog_body(dialog, title, padding=(20, 14)):
    _gui_header(dialog, title, "VISUAL TOOLS / SETTINGS")
    body = ttk.Frame(dialog, style="Capture.Card.TFrame", padding=padding)
    body.pack(fill="both", expand=True, padx=12, pady=12)
    return body


class _CapturePreview(tk.Label if TK_AVAILABLE else object):
    """只缩放已经画好的预览图；原始帧、检测坐标和保存图像不受影响。"""

    def __init__(self, parent):
        super().__init__(parent, background=GUI_COLORS["navy"], foreground="#a5bdcf",
                         text="正在打开摄像头…", takefocus=True, borderwidth=0)
        self._source_image = None
        self._display_photo = None
        self.bind("<Configure>", self._fit_on_resize)
        self.bind("<Button-1>", lambda event: self.focus_set())

    def fit_image(self, image):
        self._source_image = image
        width, height = self.winfo_width(), self.winfo_height()
        if width <= 1 or height <= 1:
            width, height = MAX_PREVIEW_W, MAX_PREVIEW_H
        scale = min(1.0, max(1, width - 4) / image.width,
                    max(1, height - 4) / image.height)
        if scale < 1.0:
            image = image.resize((max(1, int(image.width * scale)),
                                  max(1, int(image.height * scale))), Image.BILINEAR)
        self._display_photo = ImageTk.PhotoImage(image, master=self)
        return self._display_photo

    def _fit_on_resize(self, _event=None):
        if self._source_image is not None:
            photo = self.fit_image(self._source_image)
            self.configure(image=photo, text="")


def _workspace(app, title, subtitle, hint):
    """主操作常驻，次级工具可滚动；预览与侧栏可拖动调宽。"""
    root = app.root
    header = _gui_header(root, title, subtitle)
    app._bold_font.configure(family=tkfont.Font(
        root=root, name="TkDefaultFont", exists=True).actual("family"))
    if hasattr(app, "fps_var"):
        tk.Label(header, textvariable=app.fps_var, background=GUI_COLORS["navy"],
                 foreground="#b8d4e2").pack(side="right")
    root.geometry("{}x{}".format(min(1140, root.winfo_screenwidth() - 60),
                                min(800, root.winfo_screenheight() - 100)))
    footer = ttk.Frame(root, style="Capture.Card.TFrame", padding=(14, 8))
    footer.pack(side="bottom", fill="x", padx=12, pady=(0, 12))
    app.status_label = ttk.Label(footer, textvariable=app.status_var, foreground="#444444",
                                wraplength=1000, justify="left")
    app.status_label.pack(fill="x")
    footer.bind("<Configure>", lambda event: app.status_label.configure(
        wraplength=max(100, event.width - 28)))
    body = ttk.Panedwindow(root, orient="horizontal")
    body.pack(fill="both", expand=True, padx=12, pady=12)
    preview_card = ttk.Frame(body, style="Capture.Card.TFrame", padding=12)
    sidebar = ttk.Frame(body, style="Capture.Card.TFrame", padding=12)
    body.add(preview_card, weight=3)
    body.add(sidebar, weight=1)
    preview_card.columnconfigure(0, weight=1)
    preview_card.rowconfigure(1, weight=1)
    ttk.Label(preview_card, text="实时预览", style="Capture.Section.TLabel").grid(
        row=0, column=0, sticky="w", pady=(0, 9))
    # 禁用图像的请求尺寸传播，避免大图把侧栏或底部状态挤到窗口之外。
    viewport = ttk.Frame(preview_card, width=640, height=480)
    viewport.grid(row=1, column=0, sticky="nsew")
    viewport.grid_propagate(False)
    viewport.rowconfigure(0, weight=1)
    viewport.columnconfigure(0, weight=1)
    app.preview = _CapturePreview(viewport)
    app.preview.grid(row=0, column=0, sticky="nsew")
    ttk.Label(preview_card, text=hint, style="Capture.Muted.TLabel", wraplength=500,
              justify="left").grid(row=2, column=0, sticky="ew", pady=(9, 0))
    app.preview_hint = preview_card.winfo_children()[-1]
    preview_card.bind("<Configure>", lambda event: app.preview_hint.configure(
        wraplength=max(100, event.width - 24)))
    primary = ttk.Frame(sidebar, style="Capture.Card.TFrame")
    primary.pack(fill="x", pady=(0, 10))
    ttk.Label(primary, text="操作控制", style="Capture.Section.TLabel").pack(
        anchor="w", pady=(0, 8))
    # 内容不足一屏时仍可滚动，且只在侧栏自身响应滚轮，不绑定全局按键。
    scroll_area = ttk.Frame(sidebar)
    scroll_area.pack(fill="both", expand=True)
    canvas = tk.Canvas(scroll_area, width=250, highlightthickness=0,
                       background=GUI_COLORS["card"])
    scrollbar = ttk.Scrollbar(scroll_area, orient="vertical", command=canvas.yview)
    scrollbar.pack(side="right", fill="y")
    canvas.pack(side="left", fill="both", expand=True)
    canvas.configure(yscrollcommand=scrollbar.set)
    content = ttk.Frame(canvas, style="Capture.Card.TFrame")
    item = canvas.create_window(0, 0, anchor="nw", window=content)
    canvas.bind("<Configure>", lambda event: canvas.itemconfigure(item, width=event.width))
    content.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas.bind("<Button-4>", lambda event: canvas.yview_scroll(-1, "units"))
    canvas.bind("<Button-5>", lambda event: canvas.yview_scroll(1, "units"))
    canvas.bind("<MouseWheel>", lambda event: canvas.yview_scroll(
        -1 if event.delta > 0 else 1, "units"))
    root.after_idle(lambda: _wire_sidebar_scroll(canvas, content))
    app.sidebar_canvas = canvas
    app.workspace = body
    body.bind("<Configure>", lambda event: _fit_workspace(body, event.width))
    return primary, content


def _wire_sidebar_scroll(canvas, content):
    """为这一个侧栏附加局部滚轮标签，不抢 Entry/Spinbox 的原生行为。"""
    tag = "CaptureSidebar:" + str(canvas)

    def scroll(event):
        direction = -1 if getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0 else 1
        canvas.yview_scroll(direction, "units")
        return "break"

    for event in ("<Button-4>", "<Button-5>", "<MouseWheel>"):
        canvas.bind_class(tag, event, scroll)
    text_inputs = (tk.Entry, tk.Text, tk.Spinbox, ttk.Entry, ttk.Spinbox, ttk.Combobox)

    def attach(widget):
        if not isinstance(widget, text_inputs):
            tags = widget.bindtags()
            widget.bindtags((tags[0], tag) + tags[1:])
        for child in widget.winfo_children():
            attach(child)

    attach(content)


def _fit_workspace(panes, width):
    if width <= 1:
        return
    if not getattr(panes, "_initial_sash_set", False):
        panes.sashpos(0, max(140, width - 300))
        panes._initial_sash_set = True
    else:
        # 窗口变小时，预览和常驻控制至少各保留一块可用区域。
        panes.sashpos(0, max(140, min(panes.sashpos(0), width - 240)))


def _section(parent, title):
    section = ttk.Frame(parent, style="Capture.Card.TFrame")
    section.pack(fill="x", pady=(2, 12))
    ttk.Label(section, text=title, style="Capture.Section.TLabel").pack(
        anchor="w", pady=(0, 6))
    return section


def _action(parent, text, command, primary=False, danger=False, state="normal"):
    style = ("Capture.Danger.TButton" if danger else "Capture.Accent.TButton"
             if primary else "TButton")
    button = ttk.Button(parent, text=text, command=command, state=state, style=style)
    button.pack(fill="x", pady=(0, 6))
    return button


class AskTypeDialog(tk.Toplevel if TK_AVAILABLE else object):
    """询问照片类型（可选连拍张数）的模态小窗口。"""

    def __init__(self, parent, output_root: Path, initial_type: str = "",
                 initial_count: int = 1, with_count: bool = True,
                 title: str = APP_TITLE):
        if not TK_AVAILABLE:
            raise RuntimeError("tkinter 不可用")
        super().__init__(parent)
        _setup_gui_theme(self)
        self.result = None
        self.with_count = bool(with_count)
        self.output_root = Path(output_root)
        self.type_var = tk.StringVar(value=initial_type)
        self.count_var = tk.StringVar(value=str(parse_count(initial_count)))
        self.error_var = tk.StringVar(value="")
        self._title_font = tkfont.Font(root=self, size=13, weight="bold")
        self._small_font = tkfont.Font(root=self, size=10)

        self.title(title)
        self.resizable(False, False)
        try:
            if parent.winfo_viewable():
                self.transient(parent)
        except tk.TclError:
            pass

        self._build()
        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.bind("<Return>", lambda event: self._ok())
        self.bind("<Escape>", lambda event: self._cancel())
        self.update_idletasks()
        self._center()
        self.entry.focus_set()

    # -- 界面 ---------------------------------------------------------
    def _build(self) -> None:
        body = _dialog_body(self, self.title())
        body.columnconfigure(1, weight=1)

        if self.with_count:
            headline = "本次要拍哪种类型的照片？"
        else:
            headline = "要切换到哪种类型的照片？"
        ttk.Label(body, text=headline, font=self._title_font).grid(
            row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(
            body,
            text="保存位置：{}/<类型>/".format(display_path(self.output_root)),
            foreground="#666666",
            font=self._small_font,
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(3, 12))

        row = 2
        types = list_existing_types(self.output_root)
        if types:
            ttk.Label(body, text="已有类型（双击选择）：").grid(
                row=row, column=0, sticky="nw", pady=(4, 0))
            box = ttk.Frame(body)
            box.grid(row=row, column=1, sticky="we", pady=(4, 0))
            self.listbox = tk.Listbox(
                box, height=min(5, len(types)), exportselection=False,
                activestyle="dotbox", font="TkDefaultFont",
                background="#f4f8fb", foreground="#203247", relief="flat",
                selectbackground="#d8eeef", selectforeground="#075b65",
            )
            self.listbox.pack(side="left", fill="both", expand=True)
            scrollbar = ttk.Scrollbar(box, orient="vertical", command=self.listbox.yview)
            scrollbar.pack(side="right", fill="y")
            self.listbox.configure(yscrollcommand=scrollbar.set)
            for name in types:
                self.listbox.insert("end", name)
            self.listbox.bind("<<ListboxSelect>>", self._on_pick)
            self.listbox.bind("<Double-Button-1>", self._on_pick)
            row += 1

        ttk.Label(body, text="照片类型：").grid(row=row, column=0, sticky="w", pady=(10, 0))
        self.entry = ttk.Entry(body, textvariable=self.type_var, width=30)
        self.entry.grid(row=row, column=1, sticky="we", pady=(10, 0))
        row += 1

        if self.with_count:
            ttk.Label(body, text="一次连拍张数：").grid(
                row=row, column=0, sticky="w", pady=(8, 0))
            self.spin = ttk.Spinbox(
                body, from_=1, to=MAX_BURST, width=6, textvariable=self.count_var)
            self.spin.grid(row=row, column=1, sticky="w", pady=(8, 0))
            row += 1

        ttk.Label(body, textvariable=self.error_var, foreground="#b00020").grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(8, 0))
        row += 1

        buttons = ttk.Frame(body)
        buttons.grid(row=row, column=0, columnspan=2, sticky="e", pady=(8, 0))
        ttk.Button(buttons, text="取消", command=self._cancel).pack(side="right")
        self.ok_button = ttk.Button(buttons, text="开始拍照", command=self._ok,
                                    style="Capture.Accent.TButton")
        self.ok_button.pack(side="right", padx=(0, 8))

        ttk.Label(
            body,
            text="提示：类型可以是“正面/侧面/苹果”等任意中文名称，会作为文件夹名。",
            foreground="#888888",
            font=self._small_font,
        ).grid(row=row + 1, column=0, columnspan=2, sticky="w", pady=(10, 0))

    def _center(self) -> None:
        self.update_idletasks()
        width = self.winfo_width()
        height = self.winfo_height()
        x = max(0, (self.winfo_screenwidth() - width) // 2)
        y = max(0, (self.winfo_screenheight() - height) // 3)
        self.geometry("+{}+{}".format(x, y))

    # -- 事件 ---------------------------------------------------------
    def _on_pick(self, _event=None) -> None:
        selection = self.listbox.curselection()
        if selection:
            self.type_var.set(self.listbox.get(selection[0]))

    def _error(self, text: str) -> None:
        self.error_var.set(text)

    def _ok(self) -> None:
        raw = self.type_var.get().strip()
        cleaned = sanitize_type(raw)
        if not cleaned:
            self._error("请输入照片类型（不能为空）")
            return
        if cleaned != raw:
            # 斜杠等特殊字符已替换成下划线，直接采用处理后的名字
            self.type_var.set(cleaned)

        count = None
        if self.with_count:
            try:
                count = int(self.count_var.get().strip())
            except ValueError:
                self._error("连拍张数请填 1-{} 的整数".format(MAX_BURST))
                return
            if not 1 <= count <= MAX_BURST:
                self._error("连拍张数请在 1-{} 之间".format(MAX_BURST))
                return

        self.result = (cleaned, count)
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class ModeDialog(tk.Toplevel if TK_AVAILABLE else object):
    """启动时选择「拍照模式」或「模型测试模式」。"""

    def __init__(self, parent, title: str = APP_TITLE):
        if not TK_AVAILABLE:
            raise RuntimeError("tkinter 不可用")
        super().__init__(parent)
        _setup_gui_theme(self)
        self.result = None
        self.title(title)
        self.resizable(False, False)
        try:
            if parent.winfo_viewable():
                self.transient(parent)
        except tk.TclError:
            pass

        self._title_font = tkfont.Font(root=self, size=15, weight="bold")
        self._button_font = tkfont.Font(root=self, size=12, weight="bold")
        self._sub_font = tkfont.Font(root=self, size=10)

        body = _dialog_body(self, self.title(), padding=(20, 16))
        ttk.Label(body, text="请选择启动模式", font=self._title_font).pack(anchor="w")
        ttk.Label(
            body,
            text="拍照采集训练照片；模型测试做实时 YOLO 检测；区域扫描用平移拼图；仪表识别读温度/湿度表；红外热像看 MLX90640 热图。",
            wraplength=560, justify="left", foreground="#687b8e",
            font=self._sub_font,
        ).pack(anchor="w", pady=(2, 18))

        buttons = ttk.Frame(body)
        buttons.pack()
        self.photo_button = tk.Button(
            buttons, text="拍照模式\n\n按类型采集照片",
            width=14, height=4, justify="center", font=self._button_font,
            relief="flat", bd=0, cursor="hand2", background="#087f8c",
            foreground="white", activebackground="#066b77", activeforeground="white",
            command=lambda: self._choose(MODE_PHOTO),
        )
        self.photo_button.grid(row=0, column=0, sticky="nsew", padx=(0, 10), pady=(0, 10))

        if YOLO_RUNTIME_AVAILABLE:
            self.detect_button = tk.Button(
                buttons, text="模型测试\n\nYOLO 实时检测",
                width=14, height=4, justify="center", font=self._button_font,
                relief="flat", bd=0, cursor="hand2", background="#087f8c",
                foreground="white", activebackground="#066b77", activeforeground="white",
                command=lambda: self._choose(MODE_DETECT),
            )
            self.detect_button.grid(row=0, column=1, sticky="nsew", pady=(0, 10))
        else:
            ttk.Label(
                buttons, text="模型测试不可用\n（没装 ultralytics）",
                justify="center", foreground="#b00020", width=16,
            ).grid(row=0, column=1, sticky="nsew", pady=(0, 10))

        if SCAN_MODULE_AVAILABLE:
            self.scan_button = tk.Button(
                buttons, text="区域扫描\n\n平移拼大图",
                width=14, height=4, justify="center", font=self._button_font,
                relief="flat", bd=0, cursor="hand2", background="#087f8c",
                foreground="white", activebackground="#066b77", activeforeground="white",
                command=lambda: self._choose(MODE_SCAN),
            )
            self.scan_button.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        else:
            ttk.Label(
                buttons, text="区域扫描不可用\n（缺少 scan.py）",
                justify="center", foreground="#b00020", width=16,
            ).grid(row=1, column=0, sticky="nsew", padx=(0, 10))

        if GAUGE_MODULE_AVAILABLE and YOLO_RUNTIME_AVAILABLE:
            self.gauge_button = tk.Button(
                buttons, text="仪表识别\n\nPose 读表计",
                width=14, height=4, justify="center", font=self._button_font,
                relief="flat", bd=0, cursor="hand2", background="#087f8c",
                foreground="white", activebackground="#066b77", activeforeground="white",
                command=lambda: self._choose(MODE_GAUGE),
            )
            self.gauge_button.grid(row=1, column=1, sticky="nsew")
        else:
            ttk.Label(
                buttons, text="仪表识别不可用\n（缺少 gauge.py/ultralytics）",
                justify="center", foreground="#b00020", width=16,
            ).grid(row=1, column=1, sticky="nsew")

        if THERMAL_AVAILABLE:
            self.thermal_button = tk.Button(
                buttons, text="红外热像\n\nMLX90640 热图",
                width=14, height=4, justify="center", font=self._button_font,
                relief="flat", bd=0, cursor="hand2", background="#087f8c",
                foreground="white", activebackground="#066b77", activeforeground="white",
                command=lambda: self._choose(MODE_THERMAL),
            )
            self.thermal_button.grid(row=2, column=0, sticky="nsew", padx=(0, 10))
        else:
            ttk.Label(
                buttons, text="红外热像不可用\n（缺少 mlx90640_ros）",
                justify="center", foreground="#b00020", width=16,
            ).grid(row=2, column=0, sticky="nsew", padx=(0, 10))

        ttk.Label(
            body,
            text="提示：按数字键 1 / 2 / 3 / 4 / 5 也可以选择，Esc 取消。",
            foreground="#888888",
            font=self._sub_font,
        ).pack(pady=(16, 0))

        self.bind("<KeyPress-1>", lambda event: self._choose(MODE_PHOTO))
        self.bind("<KeyPress-2>", lambda event: self._choose(MODE_DETECT))
        self.bind("<KeyPress-3>", lambda event: self._choose(MODE_SCAN))
        self.bind("<KeyPress-4>", lambda event: self._choose(MODE_GAUGE))
        self.bind("<KeyPress-5>", lambda event: self._choose(MODE_THERMAL))
        self.bind("<Escape>", lambda event: self._cancel())
        self.protocol("WM_DELETE_WINDOW", self._cancel)

        self.update_idletasks()
        self._center()
        self.photo_button.focus_set()

    def _center(self) -> None:
        self.update_idletasks()
        width = self.winfo_width()
        height = self.winfo_height()
        x = max(0, (self.winfo_screenwidth() - width) // 2)
        y = max(0, (self.winfo_screenheight() - height) // 3)
        self.geometry("+{}+{}".format(x, y))

    def _choose(self, mode: str) -> None:
        if mode == MODE_DETECT and not YOLO_RUNTIME_AVAILABLE:
            return
        if mode == MODE_SCAN and not SCAN_MODULE_AVAILABLE:
            return
        if mode == MODE_GAUGE and not (GAUGE_MODULE_AVAILABLE and YOLO_RUNTIME_AVAILABLE):
            return
        if mode == MODE_THERMAL and not THERMAL_AVAILABLE:
            return
        self.result = mode
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class ModelDialog(tk.Toplevel if TK_AVAILABLE else object):
    """选择模型测试模式要用的 .pt 权重。"""

    def __init__(self, parent, model_dir, initial_model=None,
                 title: str = "选择 YOLO 模型"):
        if not TK_AVAILABLE:
            raise RuntimeError("tkinter 不可用")
        if not YOLO_MODULE_AVAILABLE:
            raise RuntimeError("yolo_detect 模块不可用")
        super().__init__(parent)
        _setup_gui_theme(self)
        self.result = None
        self.model_dir = Path(model_dir).expanduser()
        self.selected = Path(initial_model).expanduser() if initial_model else None
        if self.selected is not None and not self.selected.is_file():
            self.selected = None

        try:
            self._models = yolo_detect.list_model_files(self.model_dir)
        except Exception:
            self._models = []
        if self.selected is not None and self.selected not in self._models:
            self._models.insert(0, self.selected)

        self.path_var = tk.StringVar(value="")
        self.error_var = tk.StringVar(value="")
        self.title(title)
        self.resizable(False, False)
        try:
            if parent.winfo_viewable():
                self.transient(parent)
        except tk.TclError:
            pass

        self._title_font = tkfont.Font(root=self, size=14, weight="bold")
        self._sub_font = tkfont.Font(root=self, size=10)
        self._build()
        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.bind("<Return>", lambda event: self._ok())
        self.bind("<Escape>", lambda event: self._cancel())

        if self._models:
            index = 0
            if self.selected in self._models:
                index = self._models.index(self.selected)
            self.listbox.selection_set(index)
            self.listbox.see(index)
            self._on_select()
        if self.selected is not None:
            self.path_var.set(str(self.selected))
        self.update_idletasks()
        self._center()

    def _build(self) -> None:
        body = _dialog_body(self, self.title())

        ttk.Label(body, text="选择要测试的 YOLO 模型",
                  font=self._title_font).pack(anchor="w")
        ttk.Label(
            body,
            text="模型目录：{}    双击列表项可直接开始".format(
                display_path(self.model_dir)),
            foreground="#666666",
            font=self._sub_font,
        ).pack(anchor="w", pady=(2, 10))

        box = ttk.Frame(body)
        box.pack(fill="both", expand=True)
        self.listbox = tk.Listbox(
            box, height=min(9, max(3, len(self._models))),
            exportselection=False, width=46, font="TkDefaultFont",
            background="#f4f8fb", foreground="#203247", relief="flat",
            selectbackground="#d8eeef", selectforeground="#075b65",
        )
        self.listbox.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(
            box, orient="vertical", command=self.listbox.yview)
        scrollbar.pack(side="right", fill="y")
        self.listbox.configure(yscrollcommand=scrollbar.set)
        self.listbox.bind("<<ListboxSelect>>", self._on_select)
        self.listbox.bind("<Double-Button-1>", lambda event: self._ok())
        self._refresh()
        if not self._models:
            ttk.Label(
                body,
                text="这个目录里没有 .pt 文件，点「浏览…」选择其他位置的模型。",
                foreground="#b00020",
                justify="left",
            ).pack(anchor="w", pady=(6, 0))

        ttk.Label(body, text="已选择：", font=self._sub_font).pack(
            anchor="w", pady=(10, 0))
        ttk.Label(
            body, textvariable=self.path_var, foreground="#333333",
            wraplength=560, justify="left",
        ).pack(anchor="w")
        ttk.Label(body, textvariable=self.error_var,
                  foreground="#b00020").pack(anchor="w", pady=(4, 0))

        buttons = ttk.Frame(body)
        buttons.pack(fill="x", pady=(12, 0))
        ttk.Button(buttons, text="浏览…", command=self._browse).pack(side="left")
        ttk.Button(buttons, text="开始检测", command=self._ok,
                   style="Capture.Accent.TButton").pack(side="right")
        ttk.Button(buttons, text="取消", command=self._cancel).pack(
            side="right", padx=(0, 8))

    def _refresh(self) -> None:
        self._models = sorted(set(self._models), key=lambda p: p.name.lower())
        self.listbox.delete(0, "end")
        for path in self._models:
            self.listbox.insert(
                "end", "{}    ({})".format(path.name, yolo_detect.human_size(path)))

    def _on_select(self, _event=None) -> None:
        selection = self.listbox.curselection()
        if not selection:
            return
        self.selected = self._models[selection[0]]
        self.path_var.set(str(self.selected))
        self.error_var.set("")

    def _browse(self) -> None:
        initial_dir = self.model_dir if self.model_dir.is_dir() else Path.home()
        path = filedialog.askopenfilename(
            parent=self,
            title="选择 YOLO 权重",
            initialdir=str(initial_dir),
            filetypes=[("PyTorch 权重", "*.pt"), ("所有文件", "*")],
        )
        if not path:
            return
        chosen = Path(path).expanduser()
        if chosen not in self._models:
            self._models.insert(0, chosen)
        self._refresh()
        index = self._models.index(chosen)
        self.listbox.selection_clear(0, "end")
        self.listbox.selection_set(index)
        self.listbox.see(index)
        self._on_select()

    def _ok(self) -> None:
        if self.selected is None or not self.selected.is_file():
            self.error_var.set("请先选择一个 .pt 模型文件")
            return
        self.result = self.selected
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()

    def _center(self) -> None:
        self.update_idletasks()
        width = self.winfo_width()
        height = self.winfo_height()
        x = max(0, (self.winfo_screenwidth() - width) // 2)
        y = max(0, (self.winfo_screenheight() - height) // 3)
        self.geometry("+{}+{}".format(x, y))


# ---------------------------------------------------------------------------
# 图形界面主窗口
# ---------------------------------------------------------------------------

class PhotoApp(object):
    def __init__(self, root, camera: Camera, session: Session, output_root: Path):
        self.root = root
        self.camera = camera
        self.session = session
        self.output_root = Path(output_root)

        self._photo = None
        self._dialog_open = False
        self._closed = False
        self._font_path = find_cjk_font() if PIL_AVAILABLE else None
        self._font_cache = {}
        self._flash_until = 0.0
        self._burst_left = 0
        self._burst_total = 0
        self._next_shot = 0.0
        self._failures = 0
        self._next_reopen = 0.0
        self._after_id = None
        self._bold_font = tkfont.Font(root=root, size=12, weight="bold")

        self.type_var = tk.StringVar(value=session.type_name)
        self.count_var = tk.StringVar(value=str(session.count))
        self.status_var = tk.StringVar(value="就绪 · 保存到 {}".format(display_path(session.type_dir)))

        self._build_ui()
        self._bind_keys()
        self.root.title("{} - {}".format(APP_TITLE, session.type_name))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.minsize(520, 400)
        self._tick()

    # -- 界面 ---------------------------------------------------------
    def _build_ui(self) -> None:
        primary, content = _workspace(
            self, "拍照采集", "PHOTO CAPTURE",
            "空格 / 回车 拍照  ·  T 更换类型  ·  N 连拍张数  ·  Esc 退出\n"
            "输入框聚焦时不接管拍照快捷键；点击预览回到操作区。")
        self.shoot_button = _action(primary, "拍照（空格）", self.shoot, primary=True)
        settings = _section(content, "采集设置")
        ttk.Label(settings, text="当前类型", style="Capture.Muted.TLabel").pack(anchor="w")
        ttk.Label(settings, textvariable=self.type_var, font=self._bold_font,
                  wraplength=230).pack(anchor="w", pady=(2, 6))
        _action(settings, "更换类型 (T)", self.change_type)
        count_row = ttk.Frame(settings)
        count_row.pack(fill="x", pady=(4, 0))
        ttk.Label(count_row, text="一次连拍 / 张").pack(side="left")
        self.count_spin = ttk.Spinbox(
            count_row, from_=1, to=MAX_BURST, width=4, textvariable=self.count_var,
            command=self._on_count_change)
        self.count_spin.pack(side="right")
        files = _section(content, "文件与记录")
        _action(files, "打开保存目录", self.open_folder)
        ttk.Label(files, text=display_path(self.output_root),
                  style="Capture.Muted.TLabel", wraplength=225, justify="left").pack(anchor="w")

    def _bind_keys(self) -> None:
        self.root.bind("<space>", self._on_space)
        self.root.bind("<Return>", self._on_return)
        self.root.bind("<KeyPress-t>", self._on_key_t)
        self.root.bind("<KeyPress-T>", self._on_key_t)
        self.root.bind("<KeyPress-n>", self._on_key_n)
        self.root.bind("<KeyPress-N>", self._on_key_n)
        self.root.bind("<Escape>", lambda event: self.close())

    def _focus_consumes_keys(self) -> bool:
        """焦点在输入框 / 按钮上时，不抢空格、回车、字母键。"""
        widget = self.root.focus_get()
        if widget is None:
            return False
        text_widgets = (tk.Entry, tk.Text, tk.Spinbox, ttk.Entry, ttk.Spinbox,
                        ttk.Combobox)
        if isinstance(widget, text_widgets):
            return True
        if isinstance(widget, (tk.Button, ttk.Button)):
            return True
        return False

    def _on_space(self, event=None):
        if self._focus_consumes_keys():
            return None
        self.shoot()
        return "break"

    def _on_return(self, event=None):
        if self._focus_consumes_keys():
            return None
        self.shoot()
        return "break"

    def _on_key_t(self, event=None):
        if self._focus_consumes_keys() or self._dialog_open:
            return None
        self.change_type()
        return "break"

    def _on_key_n(self, event=None):
        if self._focus_consumes_keys() or self._dialog_open:
            return None
        self.count_spin.focus_set()
        try:
            self.count_spin.selection_range(0, "end")
        except tk.TclError:
            pass
        return "break"

    # -- 状态 ---------------------------------------------------------
    def _set_status(self, text: str, error: bool = False) -> None:
        self.status_var.set(text)
        self.status_label.configure(foreground="#b00020" if error else "#444444")

    # -- 类型 / 张数 ---------------------------------------------------
    def _current_count(self) -> int:
        try:
            count = int(self.count_var.get())
        except (ValueError, tk.TclError):
            count = self.session.count
        count = max(1, min(MAX_BURST, count))
        if str(count) != self.count_var.get().strip():
            self.count_var.set(str(count))
        return count

    def _on_count_change(self) -> None:
        count = self._current_count()
        self.session.count = count
        self._set_status("一次连拍 {} 张".format(count))

    def change_type(self) -> None:
        if self._dialog_open:
            return
        self._dialog_open = True
        try:
            dialog = AskTypeDialog(
                self.root, self.output_root,
                initial_type=self.session.type_name,
                with_count=False,
                title="{} - 更换类型".format(APP_TITLE),
            )
            self.root.wait_window(dialog)
            if not dialog.result:
                return
            new_type = dialog.result[0]
            try:
                self.session.set_type(new_type)
            except (OSError, ValueError) as error:
                messagebox.showerror("无法切换类型", str(error), parent=self.root)
                return
            self.type_var.set(self.session.type_name)
            self.root.title("{} - {}".format(APP_TITLE, self.session.type_name))
            self._set_status("已切换到「{}」· 保存到 {}".format(
                self.session.type_name, display_path(self.session.type_dir)))
        finally:
            self._dialog_open = False
            if not self._closed:
                try:
                    self.preview.focus_set()
                except tk.TclError:
                    pass

    # -- 拍照 ---------------------------------------------------------
    def shoot(self) -> None:
        if self._dialog_open or self._closed:
            return
        if self._burst_left > 0:
            return
        if self.camera.cap is None:
            self._try_reopen()
            if self.camera.cap is None:
                self._set_status("摄像头未连接，无法拍照", error=True)
                return

        count = self._current_count()
        self.session.count = count
        self._burst_total = count
        self._burst_left = count
        self._next_shot = time.monotonic()
        self._set_status("开始连拍 {} 张…".format(count))

    def _save_frame(self, frame, now: float) -> None:
        index = self._burst_total - self._burst_left + 1
        try:
            path = self.session.save(frame)
        except OSError as error:
            self._burst_left = 0
            self._burst_total = 0
            self._set_status("保存失败：{}".format(error), error=True)
            return

        self._burst_left -= 1
        self._flash_until = now + 0.12
        if self._burst_left > 0:
            self._set_status("连拍中 {}/{} · 已保存 {}".format(
                index, self._burst_total, display_path(path)))
            self._next_shot = now + max(0.05, self.session.interval)
        else:
            self._set_status("完成：本次保存 {} 张 · 累计 {} 张 · {}".format(
                self._burst_total, self.session.total,
                display_path(path.parent)))
            self._burst_total = 0

    def _try_reopen(self) -> None:
        now = time.monotonic()
        if now < self._next_reopen:
            return
        self._next_reopen = now + 2.0
        try:
            self.camera.open()
        except CameraError as error:
            first_line = str(error).splitlines()[0]
            self._set_status("摄像头连接失败：{}".format(first_line), error=True)
            return
        width, height = self.camera.frame_size
        self._set_status("摄像头已连接：{}x{}".format(width, height))
        self._failures = 0

    # -- 主循环 -------------------------------------------------------
    def _tick(self) -> None:
        if self._closed:
            return
        now = time.monotonic()

        if self.camera.cap is None:
            self._try_reopen()

        frame = self.camera.read() if self.camera.cap is not None else None
        if frame is None:
            self._failures += 1
            if self._failures == 12:
                self._set_status("摄像头没有画面，正在尝试重连…", error=True)
                self.camera.release()
            if self._burst_left > 0 and self._failures > 60:
                self._burst_left = 0
                self._burst_total = 0
                self._set_status("摄像头没有画面，连拍已取消", error=True)
        else:
            if self._failures >= 12:
                self._set_status("摄像头已恢复")
            self._failures = 0
            if self._burst_left > 0 and now >= self._next_shot:
                self._save_frame(frame, now)
            self._render(frame, now)

        try:
            self._after_id = self.root.after(20, self._tick)
        except tk.TclError:
            self._after_id = None

    # -- 预览绘制 -----------------------------------------------------
    def _render(self, frame, now: float) -> None:
        if not PIL_AVAILABLE:
            return
        height, width = frame.shape[:2]
        scale = min(1.0, float(MAX_PREVIEW_W) / width, float(MAX_PREVIEW_H) / height)
        if scale < 1.0:
            frame = cv2.resize(
                frame,
                (max(1, int(width * scale)), max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        try:
            image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            image = self._decorate(image, now)
            photo = self.preview.fit_image(image)
        except Exception:
            return
        self._photo = photo
        self.preview.configure(image=photo, text="")

    def _decorate(self, image, now: float):
        if not self._font_path:
            return image
        try:
            font = self._get_font(20)
            draw = ImageDraw.Draw(image)
            width, height = image.size

            line = "类型：{}    本次连拍：{} 张    已拍：{} 张".format(
                shorten(self.session.type_name, 16),
                self.session.count,
                self.session.total,
            )
            if self._burst_left > 0 and self._burst_total > 0:
                line += "    拍摄中 {}/{}".format(
                    self._burst_total - self._burst_left + 1, self._burst_total)

            bbox = draw.textbbox((0, 0), line, font=font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
            pad = 9
            left, top = 10, 10
            right = min(width, left + text_width + pad * 2)
            bottom = min(height, top + text_height + pad * 2)
            if right <= left or bottom <= top:
                return image

            region = image.crop((left, top, right, bottom)).convert("RGBA")
            layer = Image.new("RGBA", region.size, (0, 0, 0, 0))
            layer_draw = ImageDraw.Draw(layer)
            layer_draw.rectangle(
                [0, 0, region.size[0] - 1, region.size[1] - 1],
                fill=(0, 0, 0, 150),
            )
            layer_draw.text((pad - bbox[0], pad - bbox[1]), line,
                            font=font, fill=(255, 255, 255, 255))
            image.paste(Image.alpha_composite(region, layer).convert("RGB"),
                        (left, top))

            if now < self._flash_until:
                draw.rectangle([0, 0, width - 1, height - 1],
                               outline=(70, 220, 120), width=8)
        except Exception:
            # 画字失败就关掉叠加层，不要影响预览
            self._font_path = None
        return image

    def _get_font(self, size: int):
        if size not in self._font_cache:
            self._font_cache[size] = ImageFont.truetype(self._font_path, size)
        return self._font_cache[size]

    # -- 其他 ---------------------------------------------------------
    def open_folder(self) -> None:
        path = self.session.type_dir
        try:
            subprocess.Popen(
                ["xdg-open", str(path)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except OSError as error:
            self._set_status("无法打开文件夹：{}".format(error), error=True)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        self.camera.release()
        self.root.destroy()


class DetectApp(object):
    """模型测试模式的主窗口：预览 + 实时画框 + 推理信息。"""

    BOX_COLORS = (
        (230, 70, 70), (70, 160, 230), (80, 190, 120), (240, 170, 60),
        (170, 110, 220), (240, 100, 180), (90, 200, 200), (200, 200, 80),
        (140, 150, 240), (240, 140, 100),
    )

    def __init__(self, root, camera, detector, model_path, output_root,
                 model_dir, infer_device="auto", imgsz=640):
        self.root = root
        self.camera = camera
        self.detector = detector
        self.model_path = Path(model_path)
        self.model_dir = Path(model_dir)
        self.infer_device = infer_device
        self.imgsz = int(imgsz)
        self.snapshot_root = Path(output_root) / "模型测试"

        self._photo = None
        self._last_frame = None
        self._frame_id = 0
        self._recent_frames = {}
        self._last_result_id = -1
        self._last_error = ""
        self._dialog_open = False
        self._closed = False
        self._after_id = None
        self._failures = 0
        self._next_reopen = 0.0
        self._fps = 0.0
        self._fps_count = 0
        self._fps_start = time.monotonic()
        self._font_path = find_cjk_font() if PIL_AVAILABLE else None
        self._font_cache = {}
        self._bold_font = tkfont.Font(root=root, size=12, weight="bold")

        self.model_var = tk.StringVar(value=shorten(self.model_path.name, 26))
        self.conf_var = tk.StringVar(value="{:.2f}".format(detector.conf))
        self.infer_var = tk.StringVar(value="推理：--")
        self.det_var = tk.StringVar(value="目标：0")
        self.fps_var = tk.StringVar(value="预览：--")
        self.status_var = tk.StringVar(value=self._model_summary())

        self._build_ui()
        self._bind_keys()
        self.root.title("模型测试 - {}".format(self.model_path.name))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.minsize(560, 420)
        self._tick()

    # -- 界面 ---------------------------------------------------------
    def _model_summary(self) -> str:
        names = [self.detector.names[key] for key in sorted(self.detector.names)]
        preview = "、".join(names[:6])
        if len(names) > 6:
            preview += " 等"
        return "模型：{} · {} 类（{}）".format(
            display_path(self.model_path), len(names), preview)

    def _build_ui(self) -> None:
        primary, content = _workspace(
            self, "模型测试", "DETECTION WORKBENCH",
            "M 更换模型  ·  S 保存当前帧  ·  C 保存裁剪图  ·  Esc 退出\n"
            "拖动分隔线调整预览 / 侧栏；输入框聚焦时保留文字输入。")
        _action(primary, "保存当前帧 (S)", self.save_snapshot, primary=True)
        _action(primary, "保存裁剪图 (C)", self.save_crops)
        stats = _section(content, "实时信息")
        ttk.Label(stats, textvariable=self.infer_var).pack(anchor="w")
        ttk.Label(stats, textvariable=self.det_var).pack(anchor="w", pady=(3, 0))
        settings = _section(content, "模型与阈值")
        ttk.Label(settings, textvariable=self.model_var, font=self._bold_font,
                  wraplength=230).pack(anchor="w", pady=(0, 6))
        _action(settings, "更换模型 (M)", self.change_model)
        row = ttk.Frame(settings)
        row.pack(fill="x", pady=(4, 0))
        ttk.Label(row, text="置信度").pack(side="left")
        self.conf_spin = ttk.Spinbox(
            row, from_=0.05, to=0.95, increment=0.05, width=5,
            textvariable=self.conf_var, command=self._sync_conf)
        self.conf_spin.pack(side="right")
        self.conf_spin.bind("<Return>", lambda event: self._sync_conf())
        self.conf_spin.bind("<FocusOut>", lambda event: self._sync_conf())
        files = _section(content, "保存位置")
        ttk.Label(files, text=display_path(self.snapshot_root),
                  style="Capture.Muted.TLabel", wraplength=225, justify="left").pack(anchor="w")

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress-m>", self._on_key_m)
        self.root.bind("<KeyPress-M>", self._on_key_m)
        self.root.bind("<KeyPress-s>", self._on_key_s)
        self.root.bind("<KeyPress-S>", self._on_key_s)
        self.root.bind("<KeyPress-c>", self._on_key_c)
        self.root.bind("<KeyPress-C>", self._on_key_c)
        self.root.bind("<Escape>", lambda event: self.close())

    def _focus_is_text(self) -> bool:
        widget = self.root.focus_get()
        if widget is None:
            return False
        return isinstance(widget, (tk.Entry, tk.Text, tk.Spinbox,
                                   ttk.Entry, ttk.Spinbox))

    def _on_key_m(self, event=None):
        if self._focus_is_text() or self._dialog_open:
            return None
        self.change_model()
        return "break"

    def _on_key_s(self, event=None):
        if self._focus_is_text() or self._dialog_open:
            return None
        self.save_snapshot()
        return "break"

    def _on_key_c(self, event=None):
        if self._focus_is_text() or self._dialog_open:
            return None
        self.save_crops()
        return "break"

    # -- 参数 / 状态 ---------------------------------------------------
    def _current_conf(self) -> float:
        try:
            value = float(self.conf_var.get())
        except (TypeError, ValueError):
            return self.detector.conf
        return max(0.01, min(0.99, value))

    def _sync_conf(self) -> None:
        try:
            value = float(self.conf_var.get())
        except (TypeError, ValueError):
            self.conf_var.set("{:.2f}".format(self.detector.conf))
            return
        value = max(0.01, min(0.99, value))
        self.detector.set_conf(value)
        text = "{:.2f}".format(value)
        if self.conf_var.get().strip() != text:
            self.conf_var.set(text)

    def _set_status(self, text: str, error: bool = False) -> None:
        self.status_var.set(text)
        self.status_label.configure(foreground="#b00020" if error else "#444444")

    # -- 主循环 -------------------------------------------------------
    def _tick(self) -> None:
        if self._closed:
            return
        now = time.monotonic()

        if self.camera.cap is None:
            self._try_reopen()

        frame = self.camera.read() if self.camera.cap is not None else None
        if frame is None:
            self._failures += 1
            if self._failures == 12:
                self._set_status("摄像头没有画面，正在尝试重连…", error=True)
                self.camera.release()
        else:
            if self._failures >= 12:
                self._set_status("摄像头已恢复")
            self._failures = 0
            self._last_frame = frame
            self._frame_id += 1
            self._recent_frames[self._frame_id] = frame
            cutoff = self._frame_id - 20
            for key in list(self._recent_frames):
                if key < cutoff:
                    del self._recent_frames[key]
            self.detector.submit(frame, self._frame_id)

            latest = self.detector.latest()
            self._render(frame, latest)
            self._update_stats(latest, now)

        try:
            self._after_id = self.root.after(20, self._tick)
        except tk.TclError:
            self._after_id = None

    def _update_stats(self, latest, now: float) -> None:
        self._fps_count += 1
        elapsed = now - self._fps_start
        if elapsed >= 0.5:
            self._fps = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_start = now
            self.fps_var.set("预览：{:.1f} fps".format(self._fps))

        if latest is None:
            return
        frame_id, boxes, infer_ms = latest
        if frame_id != self._last_result_id:
            self._last_result_id = frame_id
            self.infer_var.set("推理：{} ms".format(int(round(infer_ms))))
            self.det_var.set("目标：{}".format(len(boxes)))

        error = getattr(self.detector, "last_error", "")
        if not error:
            if self._last_error:
                self._last_error = ""
                self._set_status(self._model_summary())
        elif error != self._last_error:
            self._last_error = error
            self._set_status("推理出错：{}".format(error[:100]), error=True)

    def _try_reopen(self) -> None:
        now = time.monotonic()
        if now < self._next_reopen:
            return
        self._next_reopen = now + 2.0
        try:
            self.camera.open()
        except CameraError as error:
            self._set_status(
                "摄像头连接失败：{}".format(str(error).splitlines()[0]), error=True)
            return
        width, height = self.camera.frame_size
        self._set_status("摄像头已连接：{}x{}".format(width, height))
        self._failures = 0

    # -- 绘制 ---------------------------------------------------------
    def _render(self, frame, latest) -> None:
        if not PIL_AVAILABLE:
            return
        boxes = latest[1] if latest else []
        infer_ms = latest[2] if latest else 0.0
        height, width = frame.shape[:2]
        scale = min(1.0, float(MAX_PREVIEW_W) / width, float(MAX_PREVIEW_H) / height)
        if scale < 1.0:
            display = cv2.resize(
                frame, (max(1, int(width * scale)), max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA)
        else:
            display = frame
        try:
            image = Image.fromarray(cv2.cvtColor(display, cv2.COLOR_BGR2RGB))
            image = self._annotate(image, boxes, scale, infer_ms=infer_ms)
            photo = self.preview.fit_image(image)
        except Exception:
            return
        self._photo = photo
        self.preview.configure(image=photo, text="")

    def _annotate(self, image, boxes, scale: float, infer_ms: float = 0.0,
                  font_size: int = 18):
        draw = ImageDraw.Draw(image)
        width, height = image.size

        for box in boxes:
            x1, y1, x2, y2, conf, class_id = box
            x1, y1, x2, y2 = [int(round(value * scale)) for value in (x1, y1, x2, y2)]
            color = self.BOX_COLORS[class_id % len(self.BOX_COLORS)]
            draw.rectangle([x1, y1, x2, y2], outline=color, width=3)

            font = self._get_font(font_size)
            if font is None:
                continue
            name = self.detector.names.get(class_id, str(class_id))
            label = "{} {:.2f}".format(name, conf)
            bbox = draw.textbbox((0, 0), label, font=font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
            tx = min(max(0, x1), max(0, width - text_width - 8))
            ty = y1 - text_height - 6
            if ty < 0:
                ty = min(y1 + 2, max(0, height - text_height - 6))
            draw.rectangle(
                [tx, ty, tx + text_width + 8, ty + text_height + 6], fill=color)
            text_color = (0, 0, 0) if sum(color) > 420 else (255, 255, 255)
            draw.text((tx + 3, ty + 3 - bbox[1]), label,
                      font=font, fill=text_color)

        header = "{}  |  目标 {}  |  推理 {} ms  |  {:.0f} fps".format(
            self.model_var.get(), len(boxes), int(round(infer_ms)), self._fps)
        self._draw_header(draw, header, width, height)
        return image

    def _draw_header(self, draw, text: str, width: int, height: int) -> None:
        font = self._get_font(20)
        if font is None:
            return
        bbox = draw.textbbox((0, 0), text, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]
        draw.rectangle(
            [10, 10, min(width, 10 + text_width + 18),
             min(height, 10 + text_height + 18)],
            fill=(0, 0, 0))
        draw.text((19, 19 - bbox[1]), text, font=font, fill=(255, 255, 255))

    def _get_font(self, size: int):
        if size in self._font_cache:
            return self._font_cache[size]
        font = None
        if self._font_path:
            try:
                font = ImageFont.truetype(self._font_path, size)
            except Exception:
                self._font_path = None
        if font is None:
            try:
                font = ImageFont.load_default()
            except Exception:
                font = None
        self._font_cache[size] = font
        return font

    # -- 模型 ---------------------------------------------------------
    def change_model(self) -> None:
        if self._dialog_open or self._closed:
            return
        self._dialog_open = True
        try:
            dialog = ModelDialog(
                self.root, self.model_dir,
                initial_model=str(self.model_path),
                title="更换 YOLO 模型",
            )
            self.root.wait_window(dialog)
            if not dialog.result:
                return
            new_path = dialog.result
            self._set_status("正在加载模型：{}…".format(new_path.name))
            self.root.update_idletasks()
            try:
                new_detector = yolo_detect.YoloDetector(
                    new_path, self.infer_device, self.imgsz, self._current_conf())
            except yolo_detect.ModelError as error:
                messagebox.showerror("模型加载失败", str(error), parent=self.root)
                self._set_status("模型加载失败，仍使用原模型", error=True)
                return

            old_detector = self.detector
            self.detector = new_detector
            self.model_path = new_path
            self.model_var.set(shorten(new_path.name, 26))
            self.root.title("模型测试 - {}".format(new_path.name))
            self._last_result_id = -1
            self._last_error = ""
            old_detector.close()
            self._set_status(self._model_summary())
        finally:
            self._dialog_open = False
            if not self._closed:
                try:
                    self.preview.focus_set()
                except tk.TclError:
                    pass

    # -- 截图 ---------------------------------------------------------
    def save_snapshot(self) -> None:
        if self._last_frame is None:
            self._set_status("还没有摄像头画面", error=True)
            return
        latest = self.detector.latest()
        boxes = latest[1] if latest else []
        infer_ms = latest[2] if latest else 0.0
        try:
            image = Image.fromarray(
                cv2.cvtColor(self._last_frame, cv2.COLOR_BGR2RGB))
            image = self._annotate(image, boxes, 1.0, infer_ms, font_size=24)
            stem = sanitize_type(Path(self.detector.model_path).stem) or "model"
            folder = self.snapshot_root / stem
            folder.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            path = folder / "{}_{}.jpg".format(stem, stamp)
            image.save(str(path), quality=92)
        except Exception as error:
            self._set_status("保存截图失败：{}".format(error), error=True)
            return
        self._set_status("已保存截图（{} 个目标）：{}".format(
            len(boxes), display_path(path)))

    def save_crops(self) -> None:
        latest = self.detector.latest()
        if latest is None:
            self._set_status("还没有检测结果", error=True)
            return
        frame_id, boxes, _infer_ms = latest
        if not boxes:
            self._set_status("当前没有检测目标", error=True)
            return
        frame = self._recent_frames.get(frame_id)
        if frame is None:
            frame = self._last_frame
        if frame is None:
            self._set_status("没有可裁剪的画面", error=True)
            return
        model_stem = sanitize_type(Path(self.detector.model_path).stem) or "model"
        model_dir = self.snapshot_root / model_stem / "裁剪"
        try:
            count, manifest = save_detection_crops(
                frame, boxes, self.detector.names, model_dir,
                prefix=model_stem, source="live")
        except Exception as error:
            self._set_status("保存裁剪图失败：{}".format(error), error=True)
            return
        if count <= 0:
            self._set_status("没有有效的检测框可保存", error=True)
            return
        self._set_status("已保存 {} 张裁剪图（按类别）→ {}".format(
            count, display_path(model_dir)))

    # -- 关闭 ---------------------------------------------------------
    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        try:
            self.detector.close()
        except Exception:
            pass
        self.camera.release()
        self.root.destroy()


class ScanDialog(tk.Toplevel if TK_AVAILABLE else object):
    """区域扫描参数设置。"""

    def __init__(self, parent, defaults, title: str = "区域扫描设置"):
        if not TK_AVAILABLE:
            raise RuntimeError("tkinter 不可用")
        if not SCAN_MODULE_AVAILABLE:
            raise RuntimeError("scan 模块不可用")
        super().__init__(parent)
        _setup_gui_theme(self)
        self.result = None
        self.title(title)
        self.resizable(False, False)
        try:
            if parent.winfo_viewable():
                self.transient(parent)
        except tk.TclError:
            pass

        self._title_font = tkfont.Font(root=self, size=14, weight="bold")
        self._sub_font = tkfont.Font(root=self, size=10)
        self._quality = int(getattr(defaults, "quality", 95))
        self.mode_var = tk.StringVar(value=defaults.mode)
        self.label_var = tk.StringVar(value=defaults.label)
        self.frames_var = tk.StringVar(value=str(defaults.frames))
        self.distance_var = tk.StringVar(value="%.2f" % defaults.distance_m)
        self.direction_var = tk.StringVar(value=defaults.direction)
        self.speed_var = tk.StringVar(value="%.2f" % defaults.speed)
        self.settle_var = tk.StringVar(value="%.1f" % defaults.settle)
        self.fps_var = tk.StringVar(value="%.0f" % defaults.fps)
        self.auto_stitch_var = tk.BooleanVar(value=bool(defaults.auto_stitch))
        self.record_video_var = tk.BooleanVar(value=bool(defaults.record_video))
        self.error_var = tk.StringVar(value="")
        self.summary_var = tk.StringVar(value="")

        body = _dialog_body(self, self.title(), padding=(20, 12))
        body.columnconfigure(1, weight=1)

        ttk.Label(body, text="区域扫描设置", font=self._title_font).grid(
            row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(
            body,
            text="自动平移需要 newt.py；手动录制只录画面，不控制底盘（现有键盘遥控不支持横移）。",
            wraplength=560, justify="left", foreground="#687b8e",
            font=self._sub_font,
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(2, 10))

        row = 2
        ttk.Label(body, text="区域名称：").grid(row=row, column=0, sticky="w", pady=3)
        ttk.Entry(body, textvariable=self.label_var, width=28).grid(
            row=row, column=1, sticky="we", pady=3)
        row += 1

        ttk.Label(body, text="扫描方式：").grid(row=row, column=0, sticky="nw", pady=3)
        modes = ttk.Frame(body)
        modes.grid(row=row, column=1, sticky="w", pady=3)
        self.auto_radio = ttk.Radiobutton(
            modes, text="自动平移扫描", variable=self.mode_var,
            value="strafe", command=self._update_state)
        self.auto_radio.pack(side="left")
        self.manual_radio = ttk.Radiobutton(
            modes, text="手动录制（自己移动）", variable=self.mode_var,
            value="manual", command=self._update_state)
        self.manual_radio.pack(side="left", padx=(16, 0))
        row += 1

        ttk.Label(body, text="总距离（米）：").grid(row=row, column=0, sticky="w", pady=3)
        self.distance_spin = ttk.Spinbox(
            body, from_=0.1, to=5.0, increment=0.1, width=7,
            textvariable=self.distance_var)
        self.distance_spin.grid(row=row, column=1, sticky="w", pady=3)
        row += 1

        ttk.Label(body, text="张数：").grid(row=row, column=0, sticky="w", pady=3)
        self.frames_spin = ttk.Spinbox(
            body, from_=2, to=20, increment=1, width=7,
            textvariable=self.frames_var)
        self.frames_spin.grid(row=row, column=1, sticky="w", pady=3)
        row += 1

        ttk.Label(body, text="平移方向：").grid(row=row, column=0, sticky="w", pady=3)
        directions = ttk.Frame(body)
        directions.grid(row=row, column=1, sticky="w", pady=3)
        self.left_radio = ttk.Radiobutton(
            directions, text="向左", variable=self.direction_var, value="left")
        self.left_radio.pack(side="left")
        self.right_radio = ttk.Radiobutton(
            directions, text="向右", variable=self.direction_var, value="right")
        self.right_radio.pack(side="left", padx=(16, 0))
        row += 1

        ttk.Label(body, text="平移速度（m/s）：").grid(row=row, column=0, sticky="w", pady=3)
        self.speed_spin = ttk.Spinbox(
            body, from_=0.02, to=0.15, increment=0.01, width=7,
            textvariable=self.speed_var)
        self.speed_spin.grid(row=row, column=1, sticky="w", pady=3)
        row += 1

        ttk.Label(body, text="停稳时间（秒）：").grid(row=row, column=0, sticky="w", pady=3)
        self.settle_spin = ttk.Spinbox(
            body, from_=0.1, to=3.0, increment=0.1, width=7,
            textvariable=self.settle_var)
        self.settle_spin.grid(row=row, column=1, sticky="w", pady=3)
        row += 1

        ttk.Label(body, text="手动抽帧（fps）：").grid(row=row, column=0, sticky="w", pady=3)
        self.fps_spin = ttk.Spinbox(
            body, from_=1, to=10, increment=1, width=7,
            textvariable=self.fps_var)
        self.fps_spin.grid(row=row, column=1, sticky="w", pady=3)
        row += 1

        ttk.Checkbutton(body, text="采集后自动拼接",
                        variable=self.auto_stitch_var).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(6, 0))
        row += 1
        ttk.Checkbutton(body, text="保存扫掠视频 sweep.avi",
                        variable=self.record_video_var).grid(
            row=row, column=0, columnspan=2, sticky="w")
        row += 1

        ttk.Label(body, textvariable=self.summary_var, foreground="#0066aa").grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(8, 0))
        row += 1
        ttk.Label(body, textvariable=self.error_var, foreground="#b00020").grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(4, 0))
        row += 1

        buttons = ttk.Frame(body)
        buttons.grid(row=row, column=0, columnspan=2, sticky="e", pady=(10, 0))
        ttk.Button(buttons, text="取消", command=self._cancel).pack(side="right")
        ttk.Button(buttons, text="开始扫描", command=self._ok,
                   style="Capture.Accent.TButton").pack(
            side="right", padx=(0, 8))

        for var in (self.mode_var, self.label_var, self.frames_var,
                    self.distance_var, self.direction_var, self.speed_var,
                    self.settle_var, self.fps_var):
            var.trace_add("write", lambda *args: self._update_state())
        self._update_state()
        self.update_idletasks()
        self._center()

    def _center(self) -> None:
        self.update_idletasks()
        width = self.winfo_width()
        height = self.winfo_height()
        x = max(0, (self.winfo_screenwidth() - width) // 2)
        y = max(0, (self.winfo_screenheight() - height) // 3)
        self.geometry("+{}+{}".format(x, y))

    def _collect(self, validate: bool = True):
        mode = self.mode_var.get()
        settings = scan_module.ScanSettings(
            mode=mode,
            frames=int(self.frames_var.get()) if mode == "strafe" else 5,
            distance_m=float(self.distance_var.get()) if mode == "strafe" else 0.0,
            direction=self.direction_var.get(),
            speed=float(self.speed_var.get()),
            settle=float(self.settle_var.get()),
            label=self.label_var.get(),
            auto_stitch=bool(self.auto_stitch_var.get()),
            record_video=bool(self.record_video_var.get()),
            fps=float(self.fps_var.get()),
            quality=self._quality,
        )
        if validate:
            settings.validate()
        return settings

    def _update_state(self) -> None:
        automatic = self.mode_var.get() == "strafe"
        for widget in (self.frames_spin, self.distance_spin, self.speed_spin,
                       self.settle_spin, self.left_radio, self.right_radio):
            widget.configure(state="normal" if automatic else "disabled")
        self.fps_spin.configure(state="normal" if not automatic else "disabled")
        try:
            settings = self._collect(validate=False)
            if automatic:
                text = settings.summary()
            else:
                text = "手动录制，抽帧 {:.0f} fps；自己移动，停止后拼接".format(
                    settings.fps)
            self.summary_var.set(text)
        except Exception:
            self.summary_var.set("请检查参数")

    def _ok(self) -> None:
        try:
            settings = self._collect(validate=True)
        except Exception as error:
            self.error_var.set(str(error))
            return
        self.result = settings
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class ScanApp(object):
    """区域扫描窗口：预览、开始/停止、拼接结果。"""

    def __init__(self, root, camera, output_root, settings,
                 chassis=None, stitch_max_width: int = 1600,
                 model_dir=None, conf: float = 0.25, imgsz: int = 640,
                 infer_device: str = "auto"):
        self.root = root
        self.camera = camera
        self.output_root = Path(output_root)
        self.settings = settings
        self.chassis = chassis
        self.stitch_max_width = int(stitch_max_width)
        self.model_dir = (Path(model_dir).expanduser() if model_dir
                          else Path.home() / "AI视觉")
        self.conf = float(conf)
        self.imgsz = int(imgsz)
        self.infer_device = infer_device

        self._photo = None
        self._pano_bgr = None
        self._annotated_bgr = None
        self._annotated_path = None
        self._pano_result = None
        self._detect_model = None
        self._detect_thread = None
        self._detect_events = queue.Queue()
        self._detection_boxes = []
        self._detection_names = {}
        self._session = None
        self._job = None
        self._after_id = None
        self._closed = False
        self._close_when_done = False
        self._fps = 0.0
        self._fps_count = 0
        self._fps_start = time.monotonic()
        self._font_path = find_cjk_font() if PIL_AVAILABLE else None
        self._font_cache = {}
        self._bold_font = tkfont.Font(root=root, size=12, weight="bold")

        self.status_var = tk.StringVar(value=settings.summary())
        self.progress_var = tk.StringVar(value="")
        self.fps_var = tk.StringVar(value="预览：--")
        self.mode_var = tk.StringVar(
            value="自动平移" if settings.mode == "strafe" else "手动录制")

        self._build_ui()
        self._bind_keys()
        self.root.title("区域扫描 - {}".format(self.mode_var.get()))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.minsize(560, 420)
        self._tick()

    def _build_ui(self) -> None:
        primary, content = _workspace(
            self, "区域扫描", "SCAN & STITCH",
            "C 保存裁剪图  ·  Esc 停止并退出\n"
            "自动平移涉及底盘运动；停止按钮始终保留在侧栏顶部。")
        ttk.Label(primary, textvariable=self.mode_var, font=self._bold_font).pack(
            anchor="w", pady=(0, 8))
        self.start_button = _action(
            primary, "开始扫描" if self.settings.mode == "strafe" else "开始录制",
            self.start_scan, primary=True)
        self.stop_button = _action(primary, "停止", self.stop_scan,
                                   danger=True, state="disabled")
        progress = _section(content, "扫描进度")
        self.progress = ttk.Progressbar(
            progress, orient="horizontal", mode="determinate", maximum=100)
        self.progress.pack(fill="x", pady=(2, 6))
        ttk.Label(progress, textvariable=self.progress_var, foreground="#0066aa",
                  wraplength=225, justify="left").pack(fill="x")
        results = _section(content, "全景与检测")
        self.view_button = _action(results, "查看全景", self.view_panorama, state="disabled")
        self.detect_button = _action(results, "检测全景", self.detect_panorama, state="disabled")
        self.crop_button = _action(results, "保存裁剪 (C)", self.save_crops, state="disabled")
        self.open_button = _action(results, "打开目录", self.open_folder, state="disabled")
        details = _section(content, "本次设置")
        ttk.Label(details, text=self.settings.summary(), style="Capture.Muted.TLabel",
                  wraplength=225, justify="left").pack(anchor="w")

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress-c>", self._on_key_c)
        self.root.bind("<KeyPress-C>", self._on_key_c)
        self.root.bind("<Escape>", lambda event: self.close())

    def _focus_is_text(self) -> bool:
        widget = self.root.focus_get()
        return isinstance(widget, (tk.Entry, tk.Text, tk.Spinbox,
                                   ttk.Entry, ttk.Spinbox))

    def _on_key_c(self, event=None):
        if self._focus_is_text():
            return None
        self.save_crops()
        return "break"

    # -- 预览 ---------------------------------------------------------
    def _tick(self) -> None:
        if self._closed:
            return
        if self._job is not None:
            self._poll_job()
        if self._closed:
            return
        self._poll_detection()
        if self._job is None or not self._job.running:
            if self._pano_bgr is None:
                frame = self.camera.read()
                if frame is not None:
                    self._render_frame(frame)
                    self._count_fps()

        try:
            self._after_id = self.root.after(30, self._tick)
        except tk.TclError:
            self._after_id = None

    def _count_fps(self) -> None:
        self._fps_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_start
        if elapsed >= 0.5:
            self._fps = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_start = now
            self.fps_var.set("预览：{:.1f} fps".format(self._fps))

    def _render_frame(self, frame, header: Optional[str] = None) -> None:
        if not PIL_AVAILABLE:
            return
        height, width = frame.shape[:2]
        scale = min(1.0, float(MAX_PREVIEW_W) / width, float(MAX_PREVIEW_H) / height)
        if scale < 1.0:
            frame = cv2.resize(
                frame, (max(1, int(width * scale)), max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA)
        try:
            image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(image)
            font = self._get_font(20)
            if header is None:
                header = "区域扫描 · {}".format(self.mode_var.get())
            if font is not None:
                bbox = draw.textbbox((0, 0), header, font=font)
                text_width = bbox[2] - bbox[0]
                text_height = bbox[3] - bbox[1]
                draw.rectangle(
                    [10, 10, min(image.size[0], 10 + text_width + 18),
                     min(image.size[1], 10 + text_height + 18)],
                    fill=(0, 0, 0))
                draw.text((19, 19 - bbox[1]), header, font=font,
                          fill=(255, 255, 255))
            photo = self.preview.fit_image(image)
        except Exception:
            return
        self._photo = photo
        self.preview.configure(image=photo, text="")

    def _get_font(self, size: int):
        if size in self._font_cache:
            return self._font_cache[size]
        font = None
        if self._font_path:
            try:
                font = ImageFont.truetype(self._font_path, size)
            except Exception:
                self._font_path = None
        if font is None:
            try:
                font = ImageFont.load_default()
            except Exception:
                font = None
        self._font_cache[size] = font
        return font

    # -- 采集 ---------------------------------------------------------
    def start_scan(self) -> None:
        if self._job is not None and self._job.running:
            return
        if self._detect_thread is not None and self._detect_thread.is_alive():
            self._set_status("正在检测全景，请等检测完成", error=True)
            return
        if not SCAN_MODULE_AVAILABLE:
            messagebox.showerror("区域扫描不可用", "缺少 scan.py 模块", parent=self.root)
            return
        if self.settings.mode == "strafe" and self.chassis is None:
            messagebox.showerror("没有底盘", "自动平移需要先连接底盘", parent=self.root)
            return
        try:
            session = scan_module.ScanSession(
                self.output_root, self.settings,
                camera_info={"device": "camera", "width": self.camera.frame_size[0],
                             "height": self.camera.frame_size[1]})
        except Exception as error:
            messagebox.showerror("创建扫描目录失败", str(error), parent=self.root)
            return

        self._session = session
        self._pano_bgr = None
        self._annotated_bgr = None
        self._annotated_path = None
        self._pano_result = None
        self._detection_boxes = []
        self._detection_names = {}
        self.view_button.configure(state="disabled")
        self.crop_button.configure(state="disabled")
        self.detect_button.configure(state="disabled")
        self.open_button.configure(state="normal")
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.progress.configure(mode="determinate", maximum=100, value=0)
        self.progress_var.set("")

        self._job = scan_module.ScanJob(
            self.camera, session, self.settings, chassis=self.chassis,
            stitch_max_width=self.stitch_max_width)
        self._job.start()
        self._set_status("开始采集…")

    def stop_scan(self) -> None:
        if self._job is not None and self._job.running:
            self._set_status("正在停止…")
            self._job.stop()
            self.stop_button.configure(state="disabled")

    def _poll_job(self) -> None:
        if self._job is None:
            return
        for event in self._job.poll():
            if event.kind in ("preview", "frame"):
                if event.frame is not None:
                    header = None
                    if event.kind == "frame" and event.total:
                        header = "区域扫描 · 第 {}/{} 帧".format(
                            event.index, event.total)
                    self._render_frame(event.frame, header)
            elif event.kind == "progress":
                self.progress_var.set(event.message)
                if event.total and event.total > 0:
                    value = min(100.0, 100.0 * event.index / event.total)
                    self.progress.configure(value=value)
            elif event.kind == "status":
                self._set_status(event.message)
            elif event.kind == "error":
                self._set_status(event.message, error=True)
            elif event.kind == "done":
                self._on_done(event.result)

    def _poll_detection(self) -> None:
        while True:
            try:
                event = self._detect_events.get_nowait()
            except queue.Empty:
                break
            kind = event[0]
            if kind == "progress":
                done, total = event[1], event[2]
                self._set_status("检测全景中：{}/{} 块".format(done, total))
                if total:
                    self.progress.configure(value=100.0 * done / total)
            elif kind == "done":
                _kind, result, annotated, path = event
                self._detect_thread = None
                self._annotated_bgr = annotated
                self._annotated_path = path
                self._detection_boxes = list(result.boxes)
                self._detection_names = dict(result.names)
                self.detect_button.configure(state="normal")
                self.crop_button.configure(
                    state="normal" if self._detection_boxes else "disabled")
                self._render_frame(
                    annotated, "检测结果：{} 个目标".format(len(result.boxes)))
                self._set_status("检测完成：{} 个目标 → {}".format(
                    len(result.boxes), display_path(path)))
            elif kind == "error":
                self._detect_thread = None
                self.detect_button.configure(state="normal")
                self._set_status("检测失败：{}".format(event[1]), error=True)

    def _on_done(self, result) -> None:
        job = self._job
        self._job = None
        if job is not None:
            job.join(1.0)
        self.stop_button.configure(state="disabled")
        self.start_button.configure(state="normal")
        self.progress.configure(value=100)
        self._pano_result = result
        if result is not None and getattr(result, "ok", False):
            self._show_panorama(result)
            self._set_status("拼接完成：{}x{}".format(result.width, result.height))
        elif result is not None:
            self._set_status("拼接失败：{}".format(result.message), error=True)
        else:
            self._set_status("采集结束（未拼接）")
        if self._close_when_done:
            self._destroy()

    def _show_panorama(self, result) -> None:
        if PIL_AVAILABLE and result.image is not None:
            image = result.image
        elif result.path is not None:
            image = cv2.imread(str(result.path))
        else:
            image = None
        if image is None:
            return
        self._pano_bgr = image
        header = "全景 {}x{}".format(image.shape[1], image.shape[0])
        self._render_frame(image, header)
        self.view_button.configure(state="normal")
        self.open_button.configure(state="normal")
        if YOLO_RUNTIME_AVAILABLE:
            self.detect_button.configure(state="normal")

    # -- 其他 ---------------------------------------------------------
    def _set_status(self, text: str, error: bool = False) -> None:
        self.status_var.set(text)
        self.status_label.configure(foreground="#b00020" if error else "#444444")

    def open_folder(self) -> None:
        if self._session is None:
            return
        self._open_path(self._session.directory)

    def view_panorama(self) -> None:
        if self._annotated_path is not None:
            self._open_path(self._annotated_path)
        elif self._pano_result is not None and self._pano_result.path is not None:
            self._open_path(self._pano_result.path)

    def save_crops(self) -> None:
        if not self._detection_boxes or self._pano_bgr is None or self._session is None:
            self._set_status("还没有可保存的检测裁剪图", error=True)
            return
        if not self._detect_model:
            self._set_status("没有记录检测模型", error=True)
            return
        model_stem = sanitize_type(Path(self._detect_model).stem) or "model"
        model_dir = self._session.directory / "裁剪" / model_stem
        try:
            count, _manifest = save_detection_crops(
                self._pano_bgr, self._detection_boxes, self._detection_names,
                model_dir, prefix=model_stem, source="panorama")
        except Exception as error:
            self._set_status("保存裁剪图失败：{}".format(error), error=True)
            return
        if count <= 0:
            self._set_status("没有有效的检测框可保存", error=True)
            return
        self._set_status("已保存 {} 张裁剪图（按类别）→ {}".format(
            count, display_path(model_dir)))

    def detect_panorama(self) -> None:
        if self._pano_bgr is None or self._session is None:
            self._set_status("还没有可检测的全景", error=True)
            return
        if self._detect_thread is not None and self._detect_thread.is_alive():
            return
        if not YOLO_RUNTIME_AVAILABLE:
            self._set_status("没有可用的 YOLO 环境", error=True)
            return
        dialog = ModelDialog(
            self.root, self.model_dir,
            initial_model=self._detect_model,
            title="选择全景检测模型")
        self.root.wait_window(dialog)
        if not dialog.result:
            return
        self._detect_model = dialog.result
        self._detection_boxes = []
        self._detection_names = {}
        self.detect_button.configure(state="disabled")
        self.crop_button.configure(state="disabled")
        self._set_status("正在检测全景（分块推理）…")
        self._detect_thread = threading.Thread(
            target=self._run_detection, args=(dialog.result,), daemon=True)
        self._detect_thread.start()

    def _run_detection(self, model_path) -> None:
        try:
            if not YOLO_RUNTIME_AVAILABLE or yolo_detect is None:
                raise RuntimeError("缺少 yolo_detect / ultralytics")
            result = yolo_detect.detect_image_tiled(
                self._pano_bgr, model_path,
                conf=self.conf, imgsz=self.imgsz,
                device_pref=self.infer_device,
                tile=640, overlap=0.2,
                progress=lambda done, total: self._detect_events.put(
                    ("progress", done, total)))
            if not result.ok:
                raise RuntimeError(result.message)
            annotated = annotate_detections(
                self._pano_bgr, result.boxes, result.names)
            path = self._session.directory / "panorama_detected.jpg"
            if not cv2.imwrite(str(path), annotated,
                               [int(cv2.IMWRITE_JPEG_QUALITY), 92]):
                raise RuntimeError("写入检测结果失败")
            detections = []
            for x1, y1, x2, y2, score, class_id in result.boxes:
                detections.append({
                    "class_id": int(class_id),
                    "name": result.names.get(int(class_id), str(class_id)),
                    "conf": round(float(score), 4),
                    "box": [round(float(x1), 1), round(float(y1), 1),
                            round(float(x2), 1), round(float(y2), 1)],
                })
            self._session.set_detections(detections, path)
            self._detect_events.put(("done", result, annotated, path))
        except Exception as error:
            self._detect_events.put(("error", str(error)))

    def _open_path(self, path: Path) -> None:
        try:
            subprocess.Popen(["xdg-open", str(path)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as error:
            self._set_status("无法打开：{}".format(error), error=True)

    def close(self) -> None:
        if self._closed or self._close_when_done:
            return
        if self._job is not None and self._job.running:
            if not messagebox.askyesno(
                    "正在扫描", "扫描还没结束，确定停止并退出吗？", parent=self.root):
                return
            self._close_when_done = True
            self._job.stop()
            self._set_status("正在停止，稍后退出…")
            return
        self._destroy()

    def _destroy(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        if self._job is not None:
            self._job.stop()
            self._job.join(2.0)
            self._job = None
        if self._session is not None:
            self._session.close()
        if self.chassis is not None:
            try:
                self.chassis.close()
            except Exception:
                pass
        self.camera.release()
        self.root.destroy()


class GaugeSettingsDialog(tk.Toplevel if TK_AVAILABLE else object):
    """仪表识别的量程/关键点设置。"""

    def __init__(self, parent, defaults,
                 title: str = "仪表识别设置"):
        if not TK_AVAILABLE:
            raise RuntimeError("tkinter 不可用")
        if not GAUGE_MODULE_AVAILABLE:
            raise RuntimeError("gauge 模块不可用")
        super().__init__(parent)
        _setup_gui_theme(self)
        self.result = None
        self.title(title)
        self.resizable(False, False)
        try:
            if parent.winfo_viewable():
                self.transient(parent)
        except tk.TclError:
            pass

        self._title_font = tkfont.Font(root=self, size=14, weight="bold")
        self._sub_font = tkfont.Font(root=self, size=10)
        self._imgsz = int(getattr(defaults, "imgsz", 640))
        self._conf = float(getattr(defaults, "conf", 0.5))
        self.temp_min_var = tk.StringVar(value="%.1f" % defaults.temp_min)
        self.temp_max_var = tk.StringVar(value="%.1f" % defaults.temp_max)
        self.humidity_min_var = tk.StringVar(value="%.1f" % defaults.humidity_min)
        self.humidity_max_var = tk.StringVar(value="%.1f" % defaults.humidity_max)
        self.kpt_conf_var = tk.StringVar(value="%.2f" % defaults.kpt_conf)
        self.arc_var = tk.StringVar(value={
            "long": "长弧（常见表计）", "short": "短弧", "auto": "自动",
        }.get(getattr(defaults, "arc", "long"), "长弧（常见表计）"))
        self.pointer_var = tk.StringVar(value={
            "radon": "Radon（推荐）", "tip": "模型尖端",
        }.get(getattr(defaults, "pointer", "radon"), "Radon（推荐）"))
        self.error_var = tk.StringVar(value="")

        body = _dialog_body(self, self.title(), padding=(20, 12))
        body.columnconfigure(1, weight=1)

        ttk.Label(body, text="仪表识别设置", font=self._title_font).grid(
            row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(
            body,
            text="关键点：轴 / 尖端 / 最小刻度 / 最大刻度；按量程换算读数。",
            foreground="#666666", font=self._sub_font,
        ).grid(row=1, column=0, columnspan=4, sticky="w", pady=(2, 10))

        row = 2
        ttk.Label(body, text="温度范围：").grid(row=row, column=0, sticky="w", pady=3)
        ttk.Spinbox(body, from_=-100, to=1000, increment=1, width=8,
                    textvariable=self.temp_min_var).grid(row=row, column=1, sticky="w")
        ttk.Label(body, text="～").grid(row=row, column=2)
        ttk.Spinbox(body, from_=-100, to=1000, increment=1, width=8,
                    textvariable=self.temp_max_var).grid(row=row, column=3, sticky="w")
        row += 1
        ttk.Label(body, text="湿度范围：").grid(row=row, column=0, sticky="w", pady=3)
        ttk.Spinbox(body, from_=0, to=100, increment=1, width=8,
                    textvariable=self.humidity_min_var).grid(row=row, column=1, sticky="w")
        ttk.Label(body, text="～").grid(row=row, column=2)
        ttk.Spinbox(body, from_=0, to=100, increment=1, width=8,
                    textvariable=self.humidity_max_var).grid(row=row, column=3, sticky="w")
        row += 1
        ttk.Label(body, text="关键点置信度：").grid(row=row, column=0, sticky="w", pady=3)
        ttk.Spinbox(body, from_=0.0, to=1.0, increment=0.05, width=8,
                    textvariable=self.kpt_conf_var).grid(row=row, column=1, sticky="w")
        row += 1
        ttk.Label(body, text="刻度弧方向：").grid(row=row, column=0, sticky="w", pady=3)
        self.arc_combo = ttk.Combobox(
            body, textvariable=self.arc_var, width=16, state="readonly",
            values=("长弧（常见表计）", "短弧", "自动"))
        self.arc_combo.grid(row=row, column=1, columnspan=3, sticky="w")
        row += 1
        ttk.Label(body, text="指针角度：").grid(row=row, column=0, sticky="w", pady=3)
        self.pointer_combo = ttk.Combobox(
            body, textvariable=self.pointer_var, width=16, state="readonly",
            values=("Radon（推荐）", "模型尖端"))
        self.pointer_combo.grid(row=row, column=1, columnspan=3, sticky="w")
        row += 1
        ttk.Label(body, text="提示：长弧适合大多数圆形温度/湿度表；拿不准时先用长弧。",
                  foreground="#888888", font=self._sub_font).grid(
            row=row, column=0, columnspan=4, sticky="w", pady=(4, 0))
        row += 1
        ttk.Label(body, textvariable=self.error_var,
                  foreground="#b00020").grid(
            row=row, column=0, columnspan=4, sticky="w", pady=(4, 0))
        row += 1

        buttons = ttk.Frame(body)
        buttons.grid(row=row, column=0, columnspan=4, sticky="e", pady=(10, 0))
        ttk.Button(buttons, text="取消", command=self._cancel).pack(side="right")
        ttk.Button(buttons, text="开始识别", command=self._ok,
                   style="Capture.Accent.TButton").pack(
            side="right", padx=(0, 8))

        self.update_idletasks()
        self._center()

    def _center(self) -> None:
        self.update_idletasks()
        width = self.winfo_width()
        height = self.winfo_height()
        x = max(0, (self.winfo_screenwidth() - width) // 2)
        y = max(0, (self.winfo_screenheight() - height) // 3)
        self.geometry("+{}+{}".format(x, y))

    def _arc(self) -> str:
        return {"长弧（常见表计）": "long", "短弧": "short", "自动": "auto"}.get(
            self.arc_var.get(), "long")

    def _pointer(self) -> str:
        return {"Radon（推荐）": "radon", "模型尖端": "tip"}.get(
            self.pointer_var.get(), "radon")

    def _ok(self) -> None:
        try:
            settings = gauge_module.GaugeSettings(
                temp_min=float(self.temp_min_var.get()),
                temp_max=float(self.temp_max_var.get()),
                humidity_min=float(self.humidity_min_var.get()),
                humidity_max=float(self.humidity_max_var.get()),
                kpt_conf=float(self.kpt_conf_var.get()),
                arc=self._arc(),
                pointer=self._pointer(),
                imgsz=self._imgsz,
                conf=self._conf,
            ).validate()
        except Exception as error:
            self.error_var.set(str(error))
            return
        self.result = settings
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class GaugeApp(object):
    """仪表识别窗口：Pose 关键点 + 读数。"""

    BOX_COLORS = (
        (230, 70, 70), (70, 160, 230), (80, 190, 120), (240, 170, 60),
        (170, 110, 220), (240, 100, 180),
    )

    def __init__(self, root, camera, detector, model_path, output_root,
                 model_dir, infer_device="auto", imgsz=640):
        self.root = root
        self.camera = camera
        self.detector = detector
        self.model_path = Path(model_path)
        self.model_dir = Path(model_dir)
        self.infer_device = infer_device
        self.imgsz = int(imgsz)
        self.settings = detector.settings
        self.output_root = Path(output_root)
        self.rectifier = gauge_module.GaugeRectifier(
            self.output_root, self.model_path, self.settings)

        self._photo = None
        self._readings = []
        self._readings_frame_id = -1
        self._last_template_message = ""
        self._rectified_photo = None
        self._rectified_window = None
        self._last_frame = None
        self._frame_id = 0
        self._recent_frames = {}
        self._last_result_id = -1
        self._last_error = ""
        self._dialog_open = False
        self._closed = False
        self._after_id = None
        self._failures = 0
        self._next_reopen = 0.0
        self._fps = 0.0
        self._fps_count = 0
        self._fps_start = time.monotonic()
        self._font_path = find_cjk_font() if PIL_AVAILABLE else None
        self._font_cache = {}
        self._bold_font = tkfont.Font(root=root, size=12, weight="bold")

        self.model_var = tk.StringVar(value=shorten(self.model_path.name, 26))
        self.infer_var = tk.StringVar(value="推理：--")
        self.det_var = tk.StringVar(value="仪表：0")
        self.fps_var = tk.StringVar(value="预览：--")
        self.status_var = tk.StringVar(value=self._model_summary())

        self._build_ui()
        self._bind_keys()
        self.root.title("仪表识别 - {}".format(self.model_path.name))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.minsize(560, 420)
        self._tick()

    # -- 界面 ---------------------------------------------------------
    def _model_summary(self) -> str:
        names = [self.detector.names[key] for key in sorted(self.detector.names)]
        return "模型：{} · 类别：{}".format(
            display_path(self.model_path), "、".join(names) or "?")

    def _reading_root(self) -> Path:
        stem = sanitize_type(Path(self.detector.model_path).stem) or "pose"
        return self.output_root / "仪表识别" / stem

    def _build_ui(self) -> None:
        primary, content = _workspace(
            self, "仪表识别", "GAUGE READER",
            "M 更换模型  ·  S 保存帧  ·  R 保存读数\n"
            "T 重新标定  ·  V 正视化  ·  Esc 退出；输入框内不抢快捷键。")
        _action(primary, "保存读数 (R)", self.save_readings, primary=True)
        _action(primary, "保存当前帧 (S)", self.save_snapshot)
        stats = _section(content, "实时信息")
        ttk.Label(stats, textvariable=self.infer_var).pack(anchor="w")
        ttk.Label(stats, textvariable=self.det_var).pack(anchor="w", pady=(3, 0))
        tools = _section(content, "标定与查看")
        _action(tools, "正视化 (V)", self.show_rectified)
        _action(tools, "重新标定 (T)", self.recalibrate)
        _action(tools, "打开记录目录", self.open_folder)
        settings = _section(content, "模型设置")
        ttk.Label(settings, textvariable=self.model_var, font=self._bold_font,
                  wraplength=230).pack(anchor="w", pady=(0, 6))
        _action(settings, "更换模型 (M)", self.change_model)

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress-m>", self._on_key_m)
        self.root.bind("<KeyPress-M>", self._on_key_m)
        self.root.bind("<KeyPress-s>", self._on_key_s)
        self.root.bind("<KeyPress-S>", self._on_key_s)
        self.root.bind("<KeyPress-r>", self._on_key_r)
        self.root.bind("<KeyPress-R>", self._on_key_r)
        self.root.bind("<KeyPress-t>", self._on_key_t)
        self.root.bind("<KeyPress-T>", self._on_key_t)
        self.root.bind("<KeyPress-v>", self._on_key_v)
        self.root.bind("<KeyPress-V>", self._on_key_v)
        self.root.bind("<Escape>", lambda event: self.close())

    def _focus_is_text(self) -> bool:
        widget = self.root.focus_get()
        return isinstance(widget, (tk.Entry, tk.Text, tk.Spinbox,
                                   ttk.Entry, ttk.Spinbox))

    def _on_key_m(self, event=None):
        if self._focus_is_text() or self._dialog_open:
            return None
        self.change_model()
        return "break"

    def _on_key_s(self, event=None):
        if self._focus_is_text() or self._dialog_open:
            return None
        self.save_snapshot()
        return "break"

    def _on_key_r(self, event=None):
        if self._focus_is_text() or self._dialog_open:
            return None
        self.save_readings()
        return "break"

    def _on_key_t(self, event=None):
        if self._focus_is_text() or self._dialog_open:
            return None
        self.recalibrate()
        return "break"

    def _on_key_v(self, event=None):
        if self._focus_is_text() or self._dialog_open:
            return None
        self.show_rectified()
        return "break"

    def _set_status(self, text: str, error: bool = False) -> None:
        self.status_var.set(text)
        self.status_label.configure(foreground="#b00020" if error else "#444444")

    # -- 主循环 -------------------------------------------------------
    def _tick(self) -> None:
        if self._closed:
            return
        now = time.monotonic()
        if self.camera.cap is None:
            self._try_reopen()
        frame = self.camera.read() if self.camera.cap is not None else None
        if frame is None:
            self._failures += 1
            if self._failures == 12:
                self._set_status("摄像头没有画面，正在尝试重连…", error=True)
                self.camera.release()
        else:
            if self._failures >= 12:
                self._set_status("摄像头已恢复")
            self._failures = 0
            self._last_frame = frame
            self._frame_id += 1
            self._recent_frames[self._frame_id] = frame
            cutoff = self._frame_id - 25
            for key in list(self._recent_frames):
                if key < cutoff:
                    del self._recent_frames[key]
            self.detector.submit(frame, self._frame_id)
            latest = self.detector.latest()
            readings = self._process_readings(frame, latest)
            self._render(frame, latest, readings)
            self._update_stats(latest, now, readings)
        try:
            self._after_id = self.root.after(20, self._tick)
        except tk.TclError:
            self._after_id = None

    def _process_readings(self, frame, latest):
        if latest is None:
            return []
        if latest[0] == self._readings_frame_id:
            return self._readings
        result_frame = self._recent_frames.get(latest[0], frame)
        if result_frame is None:
            return []
        readings = []
        for detection in latest[1]:
            reading = self.rectifier.rectify(result_frame, detection)
            if reading is not None:
                readings.append(reading)
        self._readings = readings
        self._readings_frame_id = latest[0]
        message = getattr(self.rectifier, "last_message", "")
        if message and message != self._last_template_message:
            self._last_template_message = message
            self._set_status(message)
        return readings

    def _update_stats(self, latest, now: float, readings=None) -> None:
        self._fps_count += 1
        elapsed = now - self._fps_start
        if elapsed >= 0.5:
            self._fps = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_start = now
            self.fps_var.set("预览：{:.1f} fps".format(self._fps))
        if latest is None:
            return
        frame_id, detections, infer_ms = latest
        if frame_id != self._last_result_id:
            self._last_result_id = frame_id
            self.infer_var.set("推理：{} ms".format(int(round(infer_ms))))
            count = len(readings) if readings is not None else len(detections)
            self.det_var.set("仪表：{}".format(count))
        error = getattr(self.detector, "last_error", "")
        if not error:
            if self._last_error:
                self._last_error = ""
                self._set_status(self._model_summary())
        elif error != self._last_error:
            self._last_error = error
            self._set_status("姿态推理出错：{}".format(error[:100]), error=True)

    def _try_reopen(self) -> None:
        now = time.monotonic()
        if now < self._next_reopen:
            return
        self._next_reopen = now + 2.0
        try:
            self.camera.open()
        except CameraError as error:
            self._set_status(
                "摄像头连接失败：{}".format(str(error).splitlines()[0]), error=True)
            return
        width, height = self.camera.frame_size
        self._set_status("摄像头已连接：{}x{}".format(width, height))
        self._failures = 0

    # -- 绘制 ---------------------------------------------------------
    def _render(self, frame, latest, readings=None) -> None:
        if not PIL_AVAILABLE:
            return
        if readings is None:
            readings = self._readings
        infer_ms = latest[2] if latest else 0.0
        height, width = frame.shape[:2]
        scale = min(1.0, float(MAX_PREVIEW_W) / width, float(MAX_PREVIEW_H) / height)
        if scale < 1.0:
            display = cv2.resize(
                frame, (max(1, int(width * scale)), max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA)
        else:
            display = frame
        try:
            image = Image.fromarray(cv2.cvtColor(display, cv2.COLOR_BGR2RGB))
            image = self._annotate(image, readings, scale, infer_ms)
            photo = self.preview.fit_image(image)
        except Exception:
            return
        self._photo = photo
        self.preview.configure(image=photo, text="")

    def _annotate(self, image, readings, scale: float, infer_ms: float = 0.0):
        draw = ImageDraw.Draw(image)
        width, height = image.size
        font = self._get_font(18)
        small = self._get_font(14)

        for reading in readings:
            det = reading.detection
            x1, y1, x2, y2 = [int(round(v * scale)) for v in det.box]
            color = self.BOX_COLORS[det.class_id % len(self.BOX_COLORS)]
            draw.rectangle([x1, y1, x2, y2], outline=color, width=3)

            points = det.keypoints * scale
            axis, tip, min_pt, max_pt = points[0], points[1], points[2], points[3]
            draw.line([tuple(axis), tuple(tip)], fill=(240, 60, 60), width=4)
            draw.line([tuple(axis), tuple(min_pt)], fill=(80, 130, 220), width=1)
            draw.line([tuple(axis), tuple(max_pt)], fill=(70, 180, 100), width=1)
            for point, point_color in (
                    (axis, (255, 255, 255)), (tip, (240, 60, 60)),
                    (min_pt, (80, 130, 220)), (max_pt, (70, 180, 100))):
                px, py = int(round(point[0])), int(round(point[1]))
                draw.ellipse([px - 5, py - 5, px + 5, py + 5],
                             fill=point_color, outline=(0, 0, 0))

            if small is not None and len(readings) == 1:
                for point, label in ((axis, "轴"), (tip, "尖"),
                                     (min_pt, "最小"), (max_pt, "最大")):
                    px, py = int(round(point[0])), int(round(point[1]))
                    draw.text((px + 7, py - 16), label, font=small,
                              fill=(255, 255, 255))

            if font is not None:
                text = self._reading_text(reading)
                bbox = draw.textbbox((0, 0), text, font=font)
                text_width = bbox[2] - bbox[0]
                text_height = bbox[3] - bbox[1]
                tx = min(max(0, x1), max(0, width - text_width - 8))
                ty = y1 - text_height - 6
                if ty < 0:
                    ty = min(y1 + 2, max(0, height - text_height - 6))
                draw.rectangle([tx, ty, tx + text_width + 8, ty + text_height + 6],
                               fill=color)
                text_color = (0, 0, 0) if sum(color) > 420 else (255, 255, 255)
                draw.text((tx + 3, ty + 3 - bbox[1]), text,
                          font=font, fill=text_color)

        header = "{}  |  仪表 {}  |  推理 {} ms  |  {:.0f} fps".format(
            self.model_var.get(), len(readings),
            int(round(infer_ms)), self._fps)
        self._draw_header(draw, header, width, height)
        return image

    @staticmethod
    def _reading_text(reading) -> str:
        det = reading.detection
        if reading.value is None:
            value = "--"
        else:
            value = "{:.2f} {}".format(reading.value, reading.unit)
        if reading.fraction is None:
            percent = "--"
        else:
            percent = "{:.0f}%".format(max(0.0, min(1.0, reading.fraction)) * 100.0)
        text = "{} {} ({})".format(det.class_name, value, percent)
        if reading.message:
            text += " " + reading.message
        return text

    def _draw_header(self, draw, text: str, width: int, height: int) -> None:
        font = self._get_font(20)
        if font is None:
            return
        bbox = draw.textbbox((0, 0), text, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]
        draw.rectangle(
            [10, 10, min(width, 10 + text_width + 18),
             min(height, 10 + text_height + 18)], fill=(0, 0, 0))
        draw.text((19, 19 - bbox[1]), text, font=font, fill=(255, 255, 255))

    def _get_font(self, size: int):
        if size in self._font_cache:
            return self._font_cache[size]
        font = None
        if self._font_path:
            try:
                font = ImageFont.truetype(self._font_path, size)
            except Exception:
                self._font_path = None
        if font is None:
            try:
                font = ImageFont.load_default()
            except Exception:
                font = None
        self._font_cache[size] = font
        return font

    # -- 保存 ---------------------------------------------------------
    def _latest_frame(self):
        latest = self.detector.latest()
        if latest is not None:
            frame = self._recent_frames.get(latest[0])
            if frame is not None:
                return frame, latest
        return self._last_frame, latest

    def save_snapshot(self) -> None:
        frame, latest = self._latest_frame()
        if frame is None:
            self._set_status("还没有摄像头画面", error=True)
            return
        readings = self._process_readings(frame, latest) if latest else []
        try:
            image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            image = self._annotate(image, readings, 1.0,
                                   latest[2] if latest else 0.0)
            folder = self._reading_root() / "snapshots"
            folder.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            path = folder / "{}_{}.jpg".format(
                sanitize_type(Path(self.detector.model_path).stem) or "pose", stamp)
            image.save(str(path), quality=92)
        except Exception as error:
            self._set_status("保存截图失败：{}".format(error), error=True)
            return
        self._set_status("已保存截图（{} 个仪表）：{}".format(
            len(readings), display_path(path)))

    def save_readings(self) -> None:
        frame, latest = self._latest_frame()
        readings = self._process_readings(frame, latest) if latest else []
        if not readings:
            self._set_status("当前没有仪表读数", error=True)
            return
        folder = self._reading_root()
        try:
            folder.mkdir(parents=True, exist_ok=True)
            path = folder / "readings.jsonl"
            record = {
                "time": datetime.now().isoformat(timespec="seconds"),
                "model": Path(self.detector.model_path).name,
                "frame_id": int(latest[0]),
                "frame_size": [int(frame.shape[1]), int(frame.shape[0])]
                              if frame is not None else None,
                "detections": [{
                    "class_id": int(reading.detection.class_id),
                    "class_name": reading.detection.class_name,
                    "conf": round(float(reading.detection.conf), 4),
                    "value": None if reading.value is None else round(float(reading.value), 3),
                    "unit": reading.unit,
                    "fraction": None if reading.fraction is None else round(float(reading.fraction), 4),
                    "angle_deg": None if reading.angle_deg is None else round(float(reading.angle_deg), 2),
                    "on_scale": bool(reading.on_scale),
                    "message": reading.message,
                    "template": reading.template.class_name,
                    "box": [round(float(v), 1) for v in reading.detection.box],
                    "keypoints": [[round(float(x), 1), round(float(y), 1)]
                                  for x, y in reading.detection.keypoints],
                    "keypoint_conf": [round(float(v), 3)
                                      for v in reading.detection.keypoint_conf],
                } for reading in readings],
            }
            with path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception as error:
            self._set_status("保存读数失败：{}".format(error), error=True)
            return
        self._set_status("已保存 {} 条仪表读数 → {}".format(
            len(readings), display_path(path)))

    def open_folder(self) -> None:
        try:
            folder = self._reading_root()
            folder.mkdir(parents=True, exist_ok=True)
            subprocess.Popen(["xdg-open", str(folder)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as error:
            self._set_status("无法打开目录：{}".format(error), error=True)

    def recalibrate(self) -> None:
        latest = self.detector.latest()
        if latest is None or not latest[1]:
            self._set_status("当前没有可标定的仪表检测", error=True)
            return
        count = self.rectifier.recalibrate(latest[1])
        if count <= 0:
            self._set_status("重新标定失败：关键点无效", error=True)
            return
        self._readings = []
        self._readings_frame_id = -1
        self._last_template_message = ""
        self._set_status("已重新标定 {} 类模板（只用了静态关键点）".format(count))

    def show_rectified(self) -> None:
        if not self._readings:
            self._set_status("还没有正视化结果", error=True)
            return
        reading = self._readings[0]
        try:
            image = Image.fromarray(cv2.cvtColor(reading.warped, cv2.COLOR_BGR2RGB))
            photo = ImageTk.PhotoImage(image)
        except Exception as error:
            self._set_status("显示正视化失败：{}".format(error), error=True)
            return
        self._rectified_photo = photo
        if self._rectified_window is not None and self._rectified_window.winfo_exists():
            label = self._rectified_window.label
            label.configure(image=photo)
            self._rectified_window.title(
                "正视化 - {}".format(reading.detection.class_name))
            self._rectified_window.lift()
            return
        window = tk.Toplevel(self.root)
        window.title("正视化 - {}".format(reading.detection.class_name))
        window.resizable(False, False)
        label = tk.Label(window, image=photo, bg="#141414")
        label.pack(padx=8, pady=8)
        window.label = label
        window.protocol("WM_DELETE_WINDOW", self._close_rectified)
        self._rectified_window = window

    def _close_rectified(self) -> None:
        if self._rectified_window is not None:
            try:
                self._rectified_window.destroy()
            except tk.TclError:
                pass
        self._rectified_window = None

    # -- 模型 ---------------------------------------------------------
    def change_model(self) -> None:
        if self._dialog_open or self._closed:
            return
        self._dialog_open = True
        try:
            dialog = ModelDialog(
                self.root, self.model_dir, initial_model=str(self.model_path),
                title="更换仪表 Pose 模型")
            self.root.wait_window(dialog)
            if not dialog.result:
                return
            new_path = dialog.result
            self._set_status("正在加载姿态模型：{}…".format(new_path.name))
            self.root.update_idletasks()
            try:
                new_detector = gauge_module.PoseDetector(
                    new_path, self.settings, self.infer_device)
            except gauge_module.ModelError as error:
                messagebox.showerror("姿态模型加载失败", str(error), parent=self.root)
                self._set_status("模型加载失败，仍使用原模型", error=True)
                return
            old_detector = self.detector
            self.detector = new_detector
            self.model_path = new_path
            self._close_rectified()
            self.rectifier = gauge_module.GaugeRectifier(
                self.output_root, self.model_path, self.settings)
            self._readings = []
            self._readings_frame_id = -1
            self._last_template_message = ""
            self.model_var.set(shorten(new_path.name, 26))
            self.root.title("仪表识别 - {}".format(new_path.name))
            self._last_result_id = -1
            self._last_error = ""
            old_detector.close()
            self._set_status(self._model_summary())
        finally:
            self._dialog_open = False
            if not self._closed:
                try:
                    self.preview.focus_set()
                except tk.TclError:
                    pass

    # -- 关闭 ---------------------------------------------------------
    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        try:
            self.detector.close()
        except Exception:
            pass
        self._close_rectified()
        self.camera.release()
        self.root.destroy()


class ThermalApp(object):
    """红外热像窗口：MLX90640 热图、楼层标定、热点读数和快照。"""

    def __init__(self, root, session, output_root, source_label="",
                 port="/dev/thermal", capture="", scale=12, delta=2.0,
                 replay_rate=2.0, floor_table_text="", camera=None,
                 camera_error=""):
        self.root = root
        self.session = session
        self.camera = camera
        self._camera_error = str(camera_error or "")
        self.output_root = Path(output_root)
        self.thermal_dir = self.output_root / "红外热像"
        self._scale = int(scale)
        self._delta = float(delta)
        self._replay_rate = float(replay_rate)
        self._capture = str(capture or "")
        self._source_label = source_label or "未连接"
        self._closed = False
        self._after_id = None
        self._shown = None
        self._last_error = None
        self._floor_samples = {}
        self._thermal_bgr = None
        self._rgb_bgr = None
        self._rgb_failures = 0
        self._last_camera_error = ""
        self._bold_font = tkfont.Font(root=root, size=12, weight="bold")

        table_text = floor_table_text or "4F:0-5,3F:6-11,2F:12-17,1F:18-23"
        if session is not None:
            try:
                current = session.floor_table.to_text()
                if current:
                    table_text = current
            except Exception:
                pass
        self.peak_var = tk.StringVar(value="--")
        self.row_var = tk.StringVar(value="--")
        self.floor_now_var = tk.StringVar(value="—")
        self.ambient_var = tk.StringVar(value="--")
        self.source_var = tk.StringVar(value=self._source_label)
        self.capture_var = tk.StringVar(value=self._capture or "未选择回放文件")
        self.port_var = tk.StringVar(value=port or "")
        self.floor_table_var = tk.StringVar(value=table_text)
        self.delta_var = tk.StringVar(value="{:.1f}".format(self._delta))
        self.view_var = tk.StringVar(value="both")
        self.status_var = tk.StringVar(
            value="等待热像帧…" if session is not None else "未连接")

        self._build_ui()
        self._bind_keys()
        self.root.title("红外热像 - MLX90640")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.minsize(700, 500)
        if self._camera_error:
            self._set_status(self._camera_error, error=True)
        self._tick()

    # -- 界面 ---------------------------------------------------------
    def _build_ui(self) -> None:
        primary, content = _workspace(
            self, "红外热像", "MLX90640 THERMAL",
            "S 保存快照  ·  Esc 退出\n"
            "热像只给出画面峰值/楼层，不提供世界坐标；回放抓包不需要硬件。")
        ttk.Label(primary, textvariable=self.source_var, font=self._bold_font,
                  wraplength=230, justify="left").pack(anchor="w", pady=(0, 8))
        _action(primary, "保存快照 (S)", self.save_snapshot, primary=True)
        ttk.Label(primary, text="预览画面", style="Capture.Section.TLabel").pack(
            anchor="w", pady=(4, 4))
        view_row = ttk.Frame(primary)
        view_row.pack(fill="x", pady=(0, 8))
        for text, value in (("双画面", "both"), ("热像", "thermal"), ("白光", "rgb")):
            ttk.Radiobutton(
                view_row, text=text, value=value, variable=self.view_var,
                command=self._render_preview).pack(side="left", padx=(0, 6))
        if self.camera is None and self._camera_error:
            ttk.Label(primary, text="白光摄像头不可用，热像仍可用",
                      style="Capture.Muted.TLabel", wraplength=225).pack(anchor="w")

        source = _section(content, "数据源")
        ttk.Label(source, text="串口（/dev/serial/by-id/...）",
                  style="Capture.Muted.TLabel", wraplength=225).pack(anchor="w")
        self.port_entry = ttk.Entry(source, textvariable=self.port_var)
        self.port_entry.pack(fill="x", pady=(2, 5))
        _action(source, "连接串口", self.connect_serial)
        _action(source, "回放抓包…", self.choose_capture)
        ttk.Label(source, textvariable=self.capture_var,
                  style="Capture.Muted.TLabel", wraplength=225,
                  justify="left").pack(anchor="w", pady=(4, 0))

        reading = _section(content, "当前读数")
        for label, var in (("峰值温度", self.peak_var),
                           ("峰值位置", self.row_var),
                           ("楼层", self.floor_now_var),
                           ("环境温度", self.ambient_var)):
            row = ttk.Frame(reading)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=label, style="Capture.Muted.TLabel").pack(side="left")
            ttk.Label(row, textvariable=var, font=self._bold_font).pack(side="right")

        calibration = _section(content, "楼层标定")
        ttk.Label(calibration, text="楼层表（row 0 是图像顶部；默认四层等分，现场必须重录）",
                  style="Capture.Muted.TLabel", wraplength=225).pack(anchor="w")
        self.floor_entry = ttk.Entry(calibration, textvariable=self.floor_table_var)
        self.floor_entry.pack(fill="x", pady=(2, 5))
        _action(calibration, "应用楼层表", self.apply_floor_table)
        row = ttk.Frame(calibration)
        row.pack(fill="x", pady=(2, 0))
        for name in ("1F", "2F", "3F", "4F"):
            ttk.Button(row, text="记录 " + name,
                       command=lambda value=name: self.record_floor(value)
                       ).pack(side="left", expand=True, fill="x", padx=1)

        params = _section(content, "热点参数")
        ttk.Label(params, text="热点框温度差 Δ℃",
                  style="Capture.Muted.TLabel").pack(anchor="w")
        self.delta_spin = ttk.Spinbox(
            params, from_=0.1, to=20.0, increment=0.1, width=8,
            textvariable=self.delta_var)
        self.delta_spin.pack(anchor="w", pady=(2, 5))
        _action(params, "应用热点参数", self.apply_delta)
        ttk.Label(params, text="保存位置", style="Capture.Muted.TLabel").pack(
            anchor="w", pady=(8, 1))
        ttk.Label(params, text=display_path(self.thermal_dir),
                  style="Capture.Muted.TLabel", wraplength=225,
                  justify="left").pack(anchor="w")

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress-s>", self._on_key_s)
        self.root.bind("<KeyPress-S>", self._on_key_s)
        self.root.bind("<Escape>", lambda event: self.close())

    def _focus_is_text(self) -> bool:
        widget = self.root.focus_get()
        return isinstance(widget, (tk.Entry, tk.Text, tk.Spinbox,
                                   ttk.Entry, ttk.Spinbox, ttk.Combobox))

    def _on_key_s(self, event=None):
        if self._focus_is_text():
            return None
        self.save_snapshot()
        return "break"

    def _set_status(self, text, error=False):
        self.status_var.set(text)
        try:
            self.status_label.configure(
                foreground="#b00020" if error else "#444444")
        except tk.TclError:
            pass

    # -- 生命周期 -----------------------------------------------------
    def _stop_session(self) -> None:
        session = self.session
        self.session = None
        self._shown = None
        if session is not None:
            try:
                session.stop()
            except Exception:
                pass

    def _start_session(self, session, source_label, port="", capture=""):
        self._stop_session()
        try:
            session.start()
        except Exception as error:
            try:
                session.stop()
            except Exception:
                pass
            self._set_status("热像启动失败：{}".format(error), error=True)
            return False
        self.session = session
        self._shown = None
        self._last_error = None
        self._floor_samples.clear()
        self._source_label = source_label
        self.source_var.set(source_label)
        if port:
            self.port_var.set(port)
        if capture:
            self._capture = capture
            self.capture_var.set(capture)
        self._set_status("已连接：{}".format(source_label))
        return True

    def connect_serial(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            self._set_status("请填写 /dev/serial/by-id/... 串口", error=True)
            return
        if not THERMAL_AVAILABLE:
            self._set_status("缺少 mlx90640_ros", error=True)
            return
        # 厂商库在进程内只能有一个 Decoder，先停掉旧 session 再建新的
        self._stop_session()
        try:
            session = ThermalSession(
                port=port, scale=self._scale, delta_c=self._current_delta(),
                floor_table=self.floor_table_var.get(),
                replay_rate=self._replay_rate)
        except Exception as error:
            self._set_status("创建热像串口会话失败：{}".format(error), error=True)
            return
        self._start_session(session, "串口：{}".format(port), port=port)

    def choose_capture(self) -> None:
        path = filedialog.askopenfilename(
            parent=self.root, title="选择 MLX90640 capture.bin",
            filetypes=[("capture.bin", "*.bin"), ("所有文件", "*")])
        if not path:
            return
        self._stop_session()
        try:
            session = ThermalSession(
                capture=path, loop=True, scale=self._scale,
                delta_c=self._current_delta(),
                floor_table=self.floor_table_var.get(),
                replay_rate=self._replay_rate)
        except Exception as error:
            self._set_status("创建热像回放会话失败：{}".format(error), error=True)
            return
        self._start_session(session, "回放：{}".format(Path(path).name),
                            capture=path)

    # -- 参数 ---------------------------------------------------------
    def _current_delta(self) -> float:
        try:
            value = float(self.delta_var.get())
        except (TypeError, ValueError, tk.TclError):
            value = self._delta
        value = max(0.1, min(20.0, value))
        self.delta_var.set("{:.1f}".format(value))
        self._delta = value
        return value

    def apply_delta(self) -> None:
        value = self._current_delta()
        if self.session is not None:
            self.session.delta_c = value
        self._set_status("热点框 Δ℃ = {:.1f}".format(value))

    def apply_floor_table(self) -> None:
        text = self.floor_table_var.get().strip()
        if self.session is None:
            self._set_status("尚未连接热像；楼层表将在连接时使用")
            return
        try:
            table = self.session.set_floor_table(text)
            self.floor_table_var.set(table.to_text())
            self._floor_samples.clear()
        except Exception as error:
            self._set_status("楼层表无效：{}".format(error), error=True)
            return
        self._set_status("楼层表：{}".format(self.floor_table_var.get()))

    def record_floor(self, name: str) -> None:
        if self.session is None:
            self._set_status("尚未连接热像", error=True)
            return
        try:
            frame = self.session.latest()
            if frame is None:
                self._set_status("还没有热像帧，等画面出来再记录", error=True)
                return
            row = float(frame.hotspot.row)
            samples = self._floor_samples.setdefault(name, [])
            samples.append(row)
            ordered = sorted(samples)
            middle = len(ordered) // 2
            if len(ordered) % 2:
                centre = ordered[middle]
            else:
                centre = (ordered[middle - 1] + ordered[middle]) / 2.0
            table = self.session.record_floor(name, centre)
            self.floor_table_var.set(table.to_text())
        except Exception as error:
            self._set_status("记录 {} 失败：{}".format(name, error), error=True)
            return
        self._set_status("{}：已记录 {} 次，row 中位数 {:.1f}；{}".format(
            name, len(samples), centre, self.floor_table_var.get()))

    # -- 显示/保存 -----------------------------------------------------
    @staticmethod
    def _to_pil(bgr):
        return Image.fromarray(bgr[:, :, ::-1])

    def _render_preview(self) -> None:
        if self._closed:
            return
        thermal = self._thermal_bgr
        rgb = self._rgb_bgr
        mode = self.view_var.get()
        if mode == "rgb" and rgb is None:
            mode = "thermal"
        if mode == "thermal" and thermal is None:
            mode = "rgb"
        if mode == "both" and (thermal is None or rgb is None):
            mode = "thermal" if thermal is not None else "rgb"

        if mode == "both" and thermal is not None and rgb is not None:
            height = 360
            t = self._to_pil(thermal)
            r = self._to_pil(rgb)
            t = t.resize((max(1, round(t.width * height / t.height)), height),
                         Image.LANCZOS)
            r = r.resize((max(1, round(r.width * height / r.height)), height),
                         Image.LANCZOS)
            gap = 8
            canvas = Image.new("RGB", (t.width + r.width + gap, height + 26),
                               (20, 20, 20))
            canvas.paste(t, (0, 26))
            canvas.paste(r, (t.width + gap, 26))
            try:
                font = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
            except Exception:
                font = ImageFont.load_default()
            draw = ImageDraw.Draw(canvas)
            draw.text((4, 3), "IR", font=font, fill=(255, 255, 255))
            draw.text((t.width + gap + 4, 3), "RGB", font=font,
                      fill=(255, 255, 255))
            image = canvas
        elif mode == "rgb" and rgb is not None:
            image = self._to_pil(rgb)
        elif thermal is not None:
            image = self._to_pil(thermal)
        elif rgb is not None:
            image = self._to_pil(rgb)
        else:
            self.preview.configure(image="", text="等待画面…")
            return
        try:
            photo = self.preview.fit_image(image)
            self.preview.configure(image=photo, text="")
        except Exception as error:
            self._set_status("显示画面失败：{}".format(error), error=True)

    def _tick(self) -> None:
        if self._closed:
            return
        changed = False
        session = self.session
        if session is not None:
            try:
                frame = session.latest()
            except Exception:
                frame = None
            if frame is not None and frame is not self._shown:
                self._shown = frame
                self._thermal_bgr = frame.bgr
                changed = True
                try:
                    summary = frame.summary()
                    self.peak_var.set("{:.2f} ℃".format(summary["peak_c"]))
                    self.row_var.set("row {:.1f} / col {:.1f}".format(
                        summary["row"], summary["col"]))
                    self.floor_now_var.set(frame.floor or "—")
                    self.ambient_var.set("{:.2f} ℃".format(summary["ambient"]))
                except Exception:
                    pass
            error = session.error()
            if error is not None and str(error) != self._last_error:
                self._last_error = str(error)
                self._set_status("热像异常：{}".format(error), error=True)

        if self.camera is not None:
            try:
                frame = self.camera.read()
            except Exception:
                frame = None
            if frame is not None:
                self._rgb_bgr = frame
                self._rgb_failures = 0
                changed = True
            else:
                self._rgb_failures += 1
                if self._rgb_failures == 12:
                    try:
                        self.camera.release()
                    except Exception:
                        pass
                    self.camera = None
                    self._last_camera_error = "白光摄像头没有画面（已停止读取）"
                    self._set_status(self._last_camera_error, error=True)

        if changed:
            self._render_preview()
        try:
            self._after_id = self.root.after(50, self._tick)
        except tk.TclError:
            self._after_id = None

    def save_snapshot(self) -> None:
        if self.session is None:
            self._set_status("尚未连接热像", error=True)
            return
        try:
            png, meta = self.session.save_snapshot(
                directory=str(self.thermal_dir), prefix="thermal", temps=False)
        except Exception as error:
            self._set_status("保存热像快照失败：{}".format(error), error=True)
            return
        self._set_status("已保存：{}（{}）".format(
            display_path(png), Path(meta).name))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        self._stop_session()
        if self.camera is not None:
            try:
                self.camera.release()
            except Exception:
                pass
            self.camera = None
        self.root.destroy()


# ---------------------------------------------------------------------------
# 图形界面启动流程
# ---------------------------------------------------------------------------

def _destroy_quietly(widget) -> None:
    try:
        if widget is not None and widget.winfo_exists():
            widget.destroy()
    except Exception:
        pass


def _load_detector_gui(root, model_path, args):
    """显示加载提示并创建检测器；失败时弹窗并返回 None。"""
    loading = tk.Toplevel(root)
    loading.title("加载模型")
    loading.resizable(False, False)
    ttk.Label(
        loading,
        text="正在加载模型：\n{}\n请稍候…".format(model_path.name),
        justify="center",
    ).pack(padx=26, pady=20)
    loading.update()
    try:
        detector = yolo_detect.YoloDetector(
            model_path, args.infer_device, args.imgsz, args.conf)
    except yolo_detect.ModelError as error:
        _destroy_quietly(loading)
        messagebox.showerror("模型加载失败", str(error))
        return None
    _destroy_quietly(loading)
    return detector


def run_gui_detect(args, output_root: Path, root) -> int:
    if not YOLO_MODULE_AVAILABLE:
        messagebox.showerror(
            "模型测试不可用",
            "缺少 yolo_detect.py 模块，请确认它和 capture_photos.py 在同一个目录。")
        root.destroy()
        return 1
    if not yolo_detect.ultralytics_available():
        messagebox.showerror(
            "模型测试不可用",
            "当前 Python 没有安装 ultralytics。\n可执行：pip3 install ultralytics")
        root.destroy()
        return 1

    model_dir = Path(args.model_dir).expanduser()
    model_path = None
    if args.model:
        model_path = yolo_detect.resolve_model_path(args.model, model_dir)
        if model_path is None:
            messagebox.showwarning(
                "找不到模型",
                "指定的模型不存在：{}\n请从列表里重新选择。".format(args.model))

    detector = None
    if model_path is not None:
        detector = _load_detector_gui(root, model_path, args)

    while detector is None:
        dialog = ModelDialog(root, model_dir, initial_model=model_path)
        root.wait_window(dialog)
        if not dialog.result:
            root.destroy()
            print("[photo_capture] 已取消")
            return 0
        model_path = dialog.result
        detector = _load_detector_gui(root, model_path, args)

    camera = Camera(
        parse_camera_source(args.device),
        width=args.width, height=args.height, fps=args.fps, mirror=args.mirror,
        sharpness=args.sharpness, pixel_format=args.pixel_format,
    )
    try:
        camera.open()
    except CameraError as error:
        detector.close()
        messagebox.showerror("摄像头不可用", str(error))
        root.destroy()
        return 1

    print("[photo_capture] 模型测试模式")
    print("[photo_capture] 模型：{}".format(model_path))
    print("[photo_capture] 设备：{}，imgsz={}，conf={:.2f}，类别 {} 类".format(
        yolo_detect.device_label(detector.device), detector.imgsz, detector.conf,
        detector.class_count))
    print("[photo_capture] 操作：M 换模型，S 保存当前帧，Esc 退出")

    try:
        app = DetectApp(
            root, camera, detector, model_path, output_root, model_dir,
            infer_device=args.infer_device, imgsz=args.imgsz,
        )
        root.deiconify()
        root.lift()
        app.preview.focus_set()
        root.mainloop()
    finally:
        camera.release()
        detector.close()
    return 0


def run_gui_scan(args, output_root: Path, root) -> int:
    if not SCAN_MODULE_AVAILABLE:
        messagebox.showerror(
            "区域扫描不可用",
            "缺少 scan.py / stitch.py 模块，请确认它们和 capture_photos.py 在同一目录。")
        root.destroy()
        return 1

    defaults = scan_module.ScanSettings(
        mode=args.scan_mode,
        frames=args.scan_frames,
        distance_m=args.scan_distance,
        direction=args.scan_direction,
        speed=args.scan_speed,
        settle=args.scan_settle,
        label=args.scan_label,
        auto_stitch=args.scan_stitch,
        record_video=args.scan_record_video,
        feedback=args.scan_feedback,
        clearance=args.scan_clearance,
        fps=args.scan_fps,
        video_fps=args.scan_video_fps,
        quality=args.quality,
    )
    dialog = ScanDialog(root, defaults)
    root.wait_window(dialog)
    if not dialog.result:
        root.destroy()
        print("[photo_capture] 已取消")
        return 0
    settings = dialog.result

    camera = Camera(
        parse_camera_source(args.device),
        width=args.width, height=args.height, fps=args.fps, mirror=args.mirror,
        sharpness=args.sharpness, pixel_format=args.pixel_format,
    )
    try:
        camera.open()
    except CameraError as error:
        messagebox.showerror("摄像头不可用", str(error))
        root.destroy()
        return 1

    chassis = None
    if settings.mode == "strafe":
        try:
            import chassis as chassis_module
            chassis = chassis_module.ChassisController(
                feedback=args.scan_feedback,
                clearance=args.scan_clearance,
                force=args.force,
                dry_run=args.scan_dry_run,
            )
            chassis.connect()
        except Exception as error:
            if chassis is not None:
                try:
                    chassis.close()
                except Exception:
                    pass
            camera.release()
            messagebox.showerror("底盘不可用", str(error))
            root.destroy()
            return 1

    print("[photo_capture] 区域扫描：{}".format(settings.summary()))
    print("[photo_capture] 输出目录：{}".format(
        display_path(Path(output_root) / "区域扫描")))

    try:
        app = ScanApp(
            root, camera, output_root, settings, chassis=chassis,
            stitch_max_width=args.stitch_max_width,
            model_dir=args.model_dir, conf=args.conf,
            imgsz=args.imgsz, infer_device=args.infer_device)
        root.deiconify()
        root.lift()
        app.preview.focus_set()
        root.mainloop()
    finally:
        camera.release()
        if chassis is not None:
            try:
                chassis.close()
            except Exception:
                pass
    return 0


def _load_pose_detector_gui(root, model_path, settings, infer_device):
    loading = tk.Toplevel(root)
    loading.title("加载仪表姿态模型")
    loading.resizable(False, False)
    ttk.Label(
        loading,
        text="正在加载姿态模型：\n{}\n请稍候…".format(Path(model_path).name),
        justify="center",
    ).pack(padx=26, pady=20)
    loading.update()
    try:
        detector = gauge_module.PoseDetector(
            model_path, settings, infer_device)
    except gauge_module.ModelError as error:
        _destroy_quietly(loading)
        messagebox.showerror("姿态模型加载失败", str(error))
        return None
    _destroy_quietly(loading)
    return detector


def run_gui_gauge(args, output_root: Path, root) -> int:
    if not GAUGE_MODULE_AVAILABLE:
        messagebox.showerror(
            "仪表识别不可用",
            "缺少 gauge.py 模块，请确认它和 capture_photos.py 在同一目录。")
        root.destroy()
        return 1
    if not YOLO_RUNTIME_AVAILABLE:
        messagebox.showerror(
            "仪表识别不可用",
            "当前 Python 没有安装 ultralytics。\n可执行：pip3 install ultralytics")
        root.destroy()
        return 1

    model_dir = Path(args.model_dir).expanduser()
    model_path = Path(args.pose_model).expanduser() if args.pose_model else None
    if model_path is None or not model_path.is_file():
        if args.pose_model:
            messagebox.showwarning(
                "找不到姿态模型",
                "指定的 Pose 模型不存在：{}\n请从列表里选择。".format(args.pose_model))
        dialog = ModelDialog(
            root, model_dir, initial_model=None,
            title="选择仪表 Pose 模型")
        root.wait_window(dialog)
        if not dialog.result:
            root.destroy()
            print("[photo_capture] 已取消")
            return 0
        model_path = dialog.result

    try:
        settings = gauge_module.GaugeSettings(
            temp_min=args.gauge_temp_min,
            temp_max=args.gauge_temp_max,
            humidity_min=args.gauge_humidity_min,
            humidity_max=args.gauge_humidity_max,
            kpt_conf=args.gauge_kpt_conf,
            arc=args.gauge_arc,
            pointer=args.gauge_pointer,
            imgsz=args.imgsz,
            conf=args.gauge_conf,
        ).validate()
    except Exception as error:
        messagebox.showerror("仪表参数错误", str(error))
        root.destroy()
        return 1

    dialog = GaugeSettingsDialog(root, settings)
    root.wait_window(dialog)
    if not dialog.result:
        root.destroy()
        print("[photo_capture] 已取消")
        return 0
    settings = dialog.result

    detector = _load_pose_detector_gui(
        root, model_path, settings, args.infer_device)
    if detector is None:
        root.destroy()
        return 1

    camera = Camera(
        parse_camera_source(args.device),
        width=args.width, height=args.height, fps=args.fps, mirror=args.mirror,
        sharpness=args.sharpness, pixel_format=args.pixel_format,
    )
    try:
        camera.open()
    except CameraError as error:
        detector.close()
        messagebox.showerror("摄像头不可用", str(error))
        root.destroy()
        return 1

    print("[photo_capture] 仪表识别模式")
    print("[photo_capture] 模型：{}（{}）".format(
        model_path, "、".join(detector.names.values())))
    print("[photo_capture] 温度范围：{:g} ~ {:g}，湿度范围：{:g} ~ {:g}".format(
        settings.temp_min, settings.temp_max,
        settings.humidity_min, settings.humidity_max))
    print("[photo_capture] 操作：M 换模型，S 存截图，R 存读数，Esc 退出")

    try:
        app = GaugeApp(
            root, camera, detector, model_path, output_root, model_dir,
            infer_device=args.infer_device, imgsz=args.imgsz)
        root.deiconify()
        root.lift()
        app.preview.focus_set()
        root.mainloop()
    finally:
        camera.release()
        detector.close()
    return 0


def run_gui_thermal(args, output_root: Path, root) -> int:
    if not THERMAL_AVAILABLE:
        messagebox.showerror(
            "红外热像不可用",
            "缺少 mlx90640_ros。请先 source devel/setup.bash，并确认包已 catkin_make。")
        root.destroy()
        return 1
    capture = str(getattr(args, "thermal_capture", "") or "").strip()
    port = str(getattr(args, "thermal_port", "") or "").strip()
    # --capture 优先；默认串口 /dev/thermal 只在没有回放时使用
    if capture:
        port = ""

    session = None
    source = ""
    try:
        if capture:
            session = ThermalSession(
                capture=capture, loop=True, scale=args.thermal_scale,
                delta_c=args.thermal_delta,
                floor_table=args.thermal_floor_table,
                replay_rate=args.thermal_replay_rate)
            source = "回放：{}".format(Path(capture).name)
        elif port:
            session = ThermalSession(
                port=port, scale=args.thermal_scale,
                delta_c=args.thermal_delta,
                floor_table=args.thermal_floor_table,
                replay_rate=args.thermal_replay_rate)
            source = "串口：{}".format(port)
        if session is not None:
            session.start()
    except Exception as error:
        if session is not None:
            try:
                session.stop()
            except Exception:
                pass
        messagebox.showerror("红外热像启动失败", str(error))
        root.destroy()
        return 1

    camera = None
    camera_error = ""
    try:
        camera = Camera(
            parse_camera_source(args.device),
            width=args.width, height=args.height, fps=args.fps,
            mirror=args.mirror, sharpness=args.sharpness,
            pixel_format=args.pixel_format)
        camera.open()
    except CameraError as error:
        camera_error = "白光摄像头不可用：{}".format(str(error).splitlines()[0])
        if camera is not None:
            try:
                camera.release()
            except Exception:
                pass
        camera = None

    try:
        app = ThermalApp(
            root, session, output_root, source_label=source or "未连接",
            port=(args.thermal_port or "/dev/thermal"), capture=capture,
            scale=args.thermal_scale, delta=args.thermal_delta,
            replay_rate=args.thermal_replay_rate,
            floor_table_text=args.thermal_floor_table,
            camera=camera, camera_error=camera_error)
        root.deiconify()
        root.lift()
        app.preview.focus_set()
        root.mainloop()
    finally:
        try:
            if 'app' in locals() and not app._closed:
                app.close()
            elif session is not None:
                session.stop()
        except Exception:
            pass
        if camera is not None:
            try:
                camera.release()
            except Exception:
                pass
    return 0


def run_gui_photo(args, output_root: Path, root) -> int:
    initial_type = args.type or ""
    while True:
        dialog = AskTypeDialog(
            root, output_root,
            initial_type=initial_type,
            initial_count=parse_count(args.count),
            with_count=True,
            title="{} - 拍摄设置".format(APP_TITLE),
        )
        root.wait_window(dialog)
        if not dialog.result:
            root.destroy()
            print("[photo_capture] 已取消")
            return 0
        type_name, count = dialog.result
        try:
            session = Session(
                output_root, type_name, count or args.count,
                image_format=args.format, quality=args.quality,
                interval=args.interval,
            )
            break
        except (OSError, ValueError) as error:
            messagebox.showerror("无法创建保存目录", str(error))
            initial_type = type_name

    camera = Camera(
        parse_camera_source(args.device),
        width=args.width, height=args.height, fps=args.fps, mirror=args.mirror,
        sharpness=args.sharpness, pixel_format=args.pixel_format,
    )
    try:
        camera.open()
    except CameraError as error:
        messagebox.showerror("摄像头不可用", str(error))
        root.destroy()
        return 1

    print_summary(session, camera)
    print("[photo_capture] 操作：空格/回车 拍照，T 换类型，N 改张数，Esc 退出")

    try:
        app = PhotoApp(root, camera, session, output_root)
        root.deiconify()
        root.lift()
        app.preview.focus_set()
        root.mainloop()
    finally:
        camera.release()
    return 0


def run_gui(args, output_root: Path) -> int:
    root = tk.Tk()
    root.withdraw()
    root.title(APP_TITLE)

    mode = args.mode
    if mode == "ask":
        dialog = ModeDialog(root, "{} - 选择模式".format(APP_TITLE))
        root.wait_window(dialog)
        mode = dialog.result
        if not mode:
            root.destroy()
            print("[photo_capture] 已取消")
            return 0

    if mode == MODE_DETECT:
        return run_gui_detect(args, output_root, root)
    if mode == MODE_SCAN:
        return run_gui_scan(args, output_root, root)
    if mode == MODE_GAUGE:
        return run_gui_gauge(args, output_root, root)
    if mode == MODE_THERMAL:
        return run_gui_thermal(args, output_root, root)
    return run_gui_photo(args, output_root, root)

# ---------------------------------------------------------------------------
# 命令行模式
# ---------------------------------------------------------------------------

def prompt_type_cli(output_root: Path, default: Optional[str] = None) -> str:
    types = list_existing_types(output_root)
    if types:
        print("已有类型：")
        for index, name in enumerate(types, 1):
            print("  {}) {}".format(index, name))
    while True:
        hint = "请选择序号或输入新的类型名称"
        if default:
            hint += "（直接回车使用 {}）".format(default)
        raw = input("{}：".format(hint)).strip()
        if not raw:
            if default:
                return default
            print("类型不能为空，请重新输入")
            continue
        if raw.isdigit() and types:
            index = int(raw)
            if 1 <= index <= len(types):
                return types[index - 1]
            print("序号超出范围（1-{}）".format(len(types)))
            continue
        cleaned = sanitize_type(raw)
        if not cleaned:
            print("类型名称无效，请重新输入")
            continue
        if cleaned != raw:
            print("[photo_capture] 类型名称已处理为：{}".format(cleaned))
        return cleaned


def shoot_cli(camera: Camera, session: Session) -> int:
    saved = 0
    for index in range(session.count):
        frame = None
        for _ in range(3):
            frame = camera.read()
            if frame is not None:
                break
        if frame is None:
            print("[photo_capture] 摄像头没有画面，本次拍照中止", file=sys.stderr)
            break
        try:
            path = session.save(frame)
        except OSError as error:
            print("[photo_capture] 保存失败：{}".format(error), file=sys.stderr)
            break
        saved += 1
        print("  已保存 {}/{}：{}".format(saved, session.count, display_path(path)))
        if index != session.count - 1 and session.interval > 0:
            time.sleep(session.interval)
    if saved:
        print("[photo_capture] 本次保存 {} 张，累计 {} 张".format(saved, session.total))
    return saved


def _fourcc_text(value) -> str:
    try:
        text = struct.pack("<I", int(value)).decode("ascii", errors="replace")
        return "".join(ch for ch in text if ch.isprintable()).strip() or "?"
    except Exception:
        return "?"


def print_summary(session: Session, camera: Camera) -> None:
    width, height = camera.frame_size
    print("[photo_capture] 类型：{}".format(session.type_name))
    print("[photo_capture] 一次连拍：{} 张，间隔 {:g} 秒".format(
        session.count, session.interval))
    print("[photo_capture] 保存目录：{}".format(session.type_dir))
    if width and height:
        print("[photo_capture] 摄像头画面：{}x{}，格式 {}，{} fps，锐度 {}".format(
            width, height, _fourcc_text(camera.actual_fourcc),
            int(camera.actual_fps or 0), camera.sharpness))


def print_cli_help() -> None:
    print("操作说明：")
    print("  直接回车 / p    按当前张数拍一次")
    print("  t [类型]        更换类型，例如：t 侧面")
    print("  n [张数]        修改一次连拍张数，例如：n 3")
    print("  q               退出")


def prompt_model_cli(model_dir: Path) -> Path:
    models = yolo_detect.list_model_files(model_dir)
    if models:
        print("可用模型（{}）：".format(display_path(model_dir)))
        for index, path in enumerate(models, 1):
            print("  {}) {}  ({})".format(
                index, path.name, yolo_detect.human_size(path)))
    elif not Path(model_dir).is_dir():
        print("模型目录不存在：{}".format(display_path(model_dir)))
    while True:
        raw = input("请选择模型序号或输入 .pt 路径：").strip()
        if not raw:
            print("不能为空，请重新输入")
            continue
        if raw.isdigit() and models:
            index = int(raw)
            if 1 <= index <= len(models):
                return models[index - 1]
            print("序号超出范围（1-{}）".format(len(models)))
            continue
        path = yolo_detect.resolve_model_path(raw, model_dir)
        if path is not None:
            return path
        print("找不到模型：{}".format(raw))


def run_detect_cli(args, output_root: Path) -> int:
    if not YOLO_MODULE_AVAILABLE:
        print("[photo_capture] 缺少 yolo_detect.py 模块", file=sys.stderr)
        return 1
    if not yolo_detect.ultralytics_available():
        print("[photo_capture] 未安装 ultralytics，请先 pip3 install ultralytics",
              file=sys.stderr)
        return 1

    model_dir = Path(args.model_dir).expanduser()
    model_path = None
    if args.model:
        model_path = yolo_detect.resolve_model_path(args.model, model_dir)
        if model_path is None:
            print("[photo_capture] 找不到模型：{}".format(args.model),
                  file=sys.stderr)
    if model_path is None:
        try:
            model_path = prompt_model_cli(model_dir)
        except (EOFError, KeyboardInterrupt):
            print("\n[photo_capture] 已取消")
            return 0

    try:
        detector = yolo_detect.YoloDetector(
            model_path, args.infer_device, args.imgsz, args.conf)
    except yolo_detect.ModelError as error:
        print("[photo_capture] {}".format(error), file=sys.stderr)
        return 1

    camera = Camera(
        parse_camera_source(args.device),
        width=args.width, height=args.height, fps=args.fps, mirror=args.mirror,
        sharpness=args.sharpness, pixel_format=args.pixel_format,
    )
    try:
        camera.open()
    except CameraError as error:
        detector.close()
        print("[photo_capture] {}".format(error), file=sys.stderr)
        return 1

    width, height = camera.frame_size
    print("[photo_capture] 模型测试模式（命令行）")
    print("[photo_capture] 模型：{}（{} 类）".format(
        model_path, detector.class_count))
    print("[photo_capture] 设备：{}，imgsz={}，conf={:.2f}".format(
        yolo_detect.device_label(detector.device), detector.imgsz, detector.conf))
    print("[photo_capture] 摄像头画面：{}x{}；Ctrl+C 退出".format(width, height))

    frames = 0
    failures = 0
    last_result_id = -1
    result_count = 0
    window_start = time.monotonic()
    last_boxes = []
    last_ms = 0.0
    try:
        while True:
            frame = camera.read()
            if frame is None:
                failures += 1
                if failures >= 20:
                    print("[photo_capture] 摄像头没有画面，退出", file=sys.stderr)
                    break
                time.sleep(0.05)
                continue
            failures = 0
            frames += 1
            detector.submit(frame, frames)

            latest = detector.latest()
            if latest is not None and latest[0] != last_result_id:
                last_result_id = latest[0]
                result_count += 1
                last_boxes = latest[1]
                last_ms = latest[2]

            now = time.monotonic()
            if now - window_start >= 1.0:
                elapsed = now - window_start
                if result_count == 0:
                    summary = "推理中…"
                elif last_boxes:
                    parts = []
                    for _x1, _y1, _x2, _y2, conf, class_id in last_boxes[:6]:
                        name = detector.names.get(class_id, str(class_id))
                        parts.append("{} {:.2f}".format(name, conf))
                    summary = "，".join(parts)
                    if len(last_boxes) > 6:
                        summary += " 等 {} 个".format(len(last_boxes))
                else:
                    summary = "无目标"
                print("[{}] 推理 {:.1f} fps | {:d} ms | {}".format(
                    datetime.now().strftime("%H:%M:%S"),
                    result_count / elapsed if elapsed > 0 else 0.0,
                    int(round(last_ms)), summary))
                result_count = 0
                window_start = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        print()
    finally:
        camera.release()
        detector.close()

    print("[photo_capture] 退出，共处理 {} 帧".format(frames))
    return 0


def run_scan_cli(args, output_root: Path) -> int:
    if not SCAN_MODULE_AVAILABLE:
        print("[photo_capture] 缺少 scan.py / stitch.py 模块", file=sys.stderr)
        return 1
    try:
        settings = scan_module.ScanSettings(
            mode=args.scan_mode,
            frames=args.scan_frames,
            distance_m=args.scan_distance,
            direction=args.scan_direction,
            speed=args.scan_speed,
            settle=args.scan_settle,
            label=args.scan_label,
            auto_stitch=args.scan_stitch,
            record_video=args.scan_record_video,
            feedback=args.scan_feedback,
            clearance=args.scan_clearance,
            fps=args.scan_fps,
            video_fps=args.scan_video_fps,
            quality=args.quality,
        ).validate()
    except Exception as error:
        print("[photo_capture] 参数错误：{}".format(error), file=sys.stderr)
        return 1

    camera = Camera(
        parse_camera_source(args.device),
        width=args.width, height=args.height, fps=args.fps, mirror=args.mirror,
        sharpness=args.sharpness, pixel_format=args.pixel_format,
    )
    try:
        camera.open()
    except CameraError as error:
        print("[photo_capture] {}".format(error), file=sys.stderr)
        return 1

    chassis = None
    if settings.mode == "strafe":
        try:
            import chassis as chassis_module
            chassis = chassis_module.ChassisController(
                feedback=args.scan_feedback,
                clearance=args.scan_clearance,
                force=args.force,
                dry_run=args.scan_dry_run,
            )
            chassis.connect()
        except Exception as error:
            camera.release()
            if chassis is not None:
                try:
                    chassis.close()
                except Exception:
                    pass
            print("[photo_capture] 底盘不可用：{}".format(error), file=sys.stderr)
            return 1

    try:
        session = scan_module.ScanSession(
            output_root, settings,
            camera_info={"device": args.device, "width": camera.frame_size[0],
                         "height": camera.frame_size[1]})
    except Exception as error:
        camera.release()
        if chassis is not None:
            chassis.close()
        print("[photo_capture] 创建扫描目录失败：{}".format(error), file=sys.stderr)
        return 1

    print("[photo_capture] 区域扫描：{}".format(settings.summary()))
    print("[photo_capture] 保存目录：{}".format(session.directory))
    if settings.mode == "strafe" and not args.yes:
        try:
            input("回车开始自动平移（Ctrl+C 取消）…")
        except (EOFError, KeyboardInterrupt):
            print("\n[photo_capture] 已取消")
            session.close()
            camera.release()
            if chassis is not None:
                chassis.close()
            return 0

    job = scan_module.ScanJob(
        camera, session, settings, chassis=chassis,
        stitch_max_width=args.stitch_max_width)
    job.start()
    if settings.mode == "manual":
        print("[photo_capture] 手动录制中：用遥控平移，按回车停止")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print()
        job.stop()
    else:
        print("[photo_capture] 自动扫描中，Ctrl+C 可停止")

    done = False
    result = None
    printed_progress = 0
    try:
        while not done:
            try:
                for event in job.poll():
                    if event.kind == "status":
                        print("[photo_capture] {}".format(event.message))
                    elif event.kind == "error":
                        print("[photo_capture] {}".format(event.message), file=sys.stderr)
                    elif event.kind == "progress":
                        if event.total and event.index > printed_progress:
                            printed_progress = event.index
                            print("[photo_capture] 已拍 {}/{}".format(
                                event.index, event.total))
                    elif event.kind == "done":
                        done = True
                        result = event.result
            except KeyboardInterrupt:
                print("\n[photo_capture] 正在停止…")
                job.stop()
                continue
            if not done and not job.running:
                time.sleep(0.05)
                continue
            if not done:
                time.sleep(0.1)
    except KeyboardInterrupt:
        job.stop()
        print("\n[photo_capture] 正在停止…")
        while job.running:
            for event in job.poll():
                if event.kind == "done":
                    result = event.result
            time.sleep(0.1)
        for event in job.poll():
            if event.kind == "done":
                result = event.result
    finally:
        job.stop()
        job.join(3.0)
        session.close()
        camera.release()
        if chassis is not None:
            try:
                chassis.close()
            except Exception:
                pass

    print("[photo_capture] 采集帧数：{}".format(len(session.frame_paths)))
    if result is not None:
        if result.ok:
            print("[photo_capture] 全景：{}（{}x{}，{}）".format(
                result.path, result.width, result.height, result.method))
        else:
            print("[photo_capture] 拼接失败：{}".format(result.message), file=sys.stderr)
    elif settings.auto_stitch:
        if len(session.frame_paths) < 2:
            print("[photo_capture] 有效帧不足 2 张，未拼接", file=sys.stderr)
    if args.detect_panorama and result is not None and result.ok:
        if not YOLO_RUNTIME_AVAILABLE:
            print("[photo_capture] 没有 YOLO 环境，跳过全景检测", file=sys.stderr)
        else:
            model_path = None
            if args.model:
                model_path = yolo_detect.resolve_model_path(args.model, args.model_dir)
            if model_path is None:
                try:
                    model_path = prompt_model_cli(Path(args.model_dir).expanduser())
                except (EOFError, KeyboardInterrupt):
                    model_path = None
            if model_path is not None:
                try:
                    print("[photo_capture] 全景检测：{}".format(model_path))
                    def progress(done, total):
                        if done == total or done % 5 == 0:
                            print("[photo_capture] 检测 {}/{} 块".format(done, total))
                    det = yolo_detect.detect_image_tiled(
                        result.image, model_path, conf=args.conf,
                        imgsz=args.imgsz, device_pref=args.infer_device,
                        tile=640, overlap=0.2, progress=progress)
                    if not det.ok:
                        print("[photo_capture] 检测失败：{}".format(det.message),
                              file=sys.stderr)
                    else:
                        annotated = annotate_detections(
                            result.image, det.boxes, det.names)
                        out_path = session.directory / "panorama_detected.jpg"
                        cv2.imwrite(str(out_path), annotated,
                                    [int(cv2.IMWRITE_JPEG_QUALITY), 92])
                        detections = [{
                            "class_id": int(cid),
                            "name": det.names.get(int(cid), str(cid)),
                            "conf": round(float(score), 4),
                            "box": [round(float(x1), 1), round(float(y1), 1),
                                    round(float(x2), 1), round(float(y2), 1)],
                        } for x1, y1, x2, y2, score, cid in det.boxes]
                        session.set_detections(detections, out_path)
                        print("[photo_capture] 检测到 {} 个目标：{}".format(
                            len(det.boxes), display_path(out_path)))
                except Exception as error:
                    print("[photo_capture] 全景检测失败：{}".format(error),
                          file=sys.stderr)

    print("[photo_capture] 目录：{}".format(session.directory))
    return 0 if session.frame_paths else 1


def run_gauge_cli(args, output_root: Path) -> int:
    if not GAUGE_MODULE_AVAILABLE:
        print("[photo_capture] 缺少 gauge.py 模块", file=sys.stderr)
        return 1
    if not YOLO_RUNTIME_AVAILABLE:
        print("[photo_capture] 未安装 ultralytics，请先 pip3 install ultralytics",
              file=sys.stderr)
        return 1
    try:
        settings = gauge_module.GaugeSettings(
            temp_min=args.gauge_temp_min,
            temp_max=args.gauge_temp_max,
            humidity_min=args.gauge_humidity_min,
            humidity_max=args.gauge_humidity_max,
            kpt_conf=args.gauge_kpt_conf,
            arc=args.gauge_arc,
            pointer=args.gauge_pointer,
            imgsz=args.imgsz,
            conf=args.gauge_conf,
        ).validate()
    except Exception as error:
        print("[photo_capture] 仪表参数错误：{}".format(error), file=sys.stderr)
        return 1

    model_dir = Path(args.model_dir).expanduser()
    model_path = Path(args.pose_model).expanduser() if args.pose_model else None
    if model_path is None or not model_path.is_file():
        if args.pose_model:
            print("[photo_capture] 找不到姿态模型：{}".format(args.pose_model),
                  file=sys.stderr)
        try:
            model_path = prompt_model_cli(model_dir)
        except (EOFError, KeyboardInterrupt):
            print("\n[photo_capture] 已取消")
            return 0

    try:
        detector = gauge_module.PoseDetector(
            model_path, settings, args.infer_device)
    except gauge_module.ModelError as error:
        print("[photo_capture] {}".format(error), file=sys.stderr)
        return 1
    rectifier = gauge_module.GaugeRectifier(output_root, model_path, settings)

    camera = Camera(
        parse_camera_source(args.device),
        width=args.width, height=args.height, fps=args.fps, mirror=args.mirror,
        sharpness=args.sharpness, pixel_format=args.pixel_format,
    )
    try:
        camera.open()
    except CameraError as error:
        detector.close()
        print("[photo_capture] {}".format(error), file=sys.stderr)
        return 1

    print("[photo_capture] 仪表识别模式（命令行）")
    print("[photo_capture] 模型：{}（{}）".format(
        model_path, "、".join(detector.names.values())))
    print("[photo_capture] 温度范围：{:g} ~ {:g}，湿度范围：{:g} ~ {:g}，弧：{}，指针：{}".format(
        settings.temp_min, settings.temp_max,
        settings.humidity_min, settings.humidity_max,
        settings.arc, settings.pointer))
    print("[photo_capture] Ctrl+C 退出")

    frames = 0
    failures = 0
    last_print = time.monotonic()
    last_result_id = -1
    last_readings = []
    try:
        while True:
            frame = camera.read()
            if frame is None:
                failures += 1
                if failures >= 20:
                    print("[photo_capture] 摄像头没有画面，退出", file=sys.stderr)
                    break
                time.sleep(0.05)
                continue
            failures = 0
            frames += 1
            detector.submit(frame, frames)
            latest = detector.latest()
            if latest is not None and latest[0] != last_result_id:
                last_result_id = latest[0]
                readings = []
                for detection in latest[1]:
                    reading = rectifier.rectify(frame, detection)
                    if reading is not None:
                        readings.append(reading)
                last_readings = readings
            else:
                readings = last_readings
            if rectifier.last_message:
                print("[photo_capture] {}".format(rectifier.last_message))
                rectifier.last_message = ""
            now = time.monotonic()
            if now - last_print >= 1.0:
                if readings:
                    text = " | ".join(
                        gauge_module.reading_text(reading) for reading in readings)
                    print("[{}] {} 个仪表 | {}".format(
                        datetime.now().strftime("%H:%M:%S"), len(readings), text))
                else:
                    print("[{}] 未识别到仪表".format(
                        datetime.now().strftime("%H:%M:%S")))
                last_print = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        print()
    finally:
        camera.release()
        detector.close()
    print("[photo_capture] 退出，共处理 {} 帧".format(frames))
    return 0


def run_thermal_cli(args, output_root: Path) -> int:
    if not THERMAL_AVAILABLE:
        print("[photo_capture] 缺少 mlx90640_ros；请先 source devel/setup.bash",
              file=sys.stderr)
        return 1
    capture = str(getattr(args, "thermal_capture", "") or "").strip()
    port = str(getattr(args, "thermal_port", "") or "").strip()
    if capture:
        port = ""
    if not capture and not port:
        print("[photo_capture] 红外热像需要 --capture 或 --thermal-port",
              file=sys.stderr)
        return 1
    try:
        session = ThermalSession(
            capture=capture or None, port=None if capture else port,
            loop=True, scale=args.thermal_scale, delta_c=args.thermal_delta,
            floor_table=args.thermal_floor_table,
            replay_rate=args.thermal_replay_rate)
    except Exception as error:
        print("[photo_capture] 创建热像会话失败：{}".format(error), file=sys.stderr)
        return 1
    try:
        session.start()
    except Exception as error:
        session.stop()
        print("[photo_capture] 启动热像失败：{}".format(error), file=sys.stderr)
        return 1

    source = "回放：{}".format(capture) if capture else "串口：{}".format(port)
    print("[photo_capture] 红外热像（命令行）：{}".format(source))
    print("[photo_capture] Ctrl+C 退出")
    last = None
    try:
        while True:
            frame = session.latest()
            if frame is not None and frame is not last:
                last = frame
                summary = frame.summary()
                print("[{}] 峰值 {:.2f}℃ row {:.1f} col {:.1f} | "
                      "楼层 {} | 环境 {:.2f}℃".format(
                          datetime.now().strftime("%H:%M:%S"),
                          summary["peak_c"], summary["row"], summary["col"],
                          frame.floor or "—", summary["ambient"]))
            error = session.error()
            if error is not None:
                print("[photo_capture] 热像异常：{}".format(error),
                      file=sys.stderr)
                return 1
            time.sleep(0.2)
    except KeyboardInterrupt:
        print()
    finally:
        session.stop()
        print("[photo_capture] 红外热像已退出")
    return 0


def run_cli(args, output_root: Path) -> int:
    print("[photo_capture] 输出根目录：{}".format(display_path(output_root)))
    print("[photo_capture] 命令行模式（没有图形界面时使用）")

    mode = args.mode
    if mode == "ask":
        print("启动模式：")
        print("  1) 拍照模式")
        print("  2) 模型测试模式")
        print("  3) 区域扫描（平移拼图）")
        print("  4) 仪表识别（Pose 读温度/湿度表）")
        print("  5) 红外热像（MLX90640）")
        while True:
            try:
                choice = input("请选择 [1/2/3/4/5]（默认 1）：").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n[photo_capture] 已取消")
                return 0
            if choice in ("", "1"):
                mode = MODE_PHOTO
                break
            if choice == "2":
                mode = MODE_DETECT
                break
            if choice == "3":
                mode = MODE_SCAN
                break
            if choice == "4":
                mode = MODE_GAUGE
                break
            if choice == "5":
                mode = MODE_THERMAL
                break
            print("请输入 1 / 2 / 3 / 4 / 5")

    if mode == MODE_DETECT:
        return run_detect_cli(args, output_root)
    if mode == MODE_SCAN:
        return run_scan_cli(args, output_root)
    if mode == MODE_GAUGE:
        return run_gauge_cli(args, output_root)
    if mode == MODE_THERMAL:
        return run_thermal_cli(args, output_root)

    try:
        type_name = args.type or prompt_type_cli(output_root)
    except (EOFError, KeyboardInterrupt):
        print("\n[photo_capture] 已取消")
        return 0

    try:
        session = Session(
            output_root, type_name, args.count,
            image_format=args.format, quality=args.quality, interval=args.interval,
        )
    except (OSError, ValueError) as error:
        print("[photo_capture] 无法创建保存目录：{}".format(error), file=sys.stderr)
        return 1

    camera = Camera(
        parse_camera_source(args.device),
        width=args.width, height=args.height, fps=args.fps, mirror=args.mirror,
        sharpness=args.sharpness, pixel_format=args.pixel_format,
    )
    try:
        camera.open()
    except CameraError as error:
        print("[photo_capture] {}".format(error), file=sys.stderr)
        return 1

    print_summary(session, camera)
    print_cli_help()

    try:
        while True:
            try:
                line = input("拍照> ").strip()
            except EOFError:
                print()
                break
            except KeyboardInterrupt:
                print()
                break

            lowered = line.lower()
            if lowered in ("q", "quit", "exit"):
                break
            if lowered in ("h", "help", "?"):
                print_cli_help()
                continue
            if lowered == "t" or lowered.startswith("t "):
                rest = line[1:].strip()
                try:
                    new_type = rest or prompt_type_cli(output_root, session.type_name)
                except (EOFError, KeyboardInterrupt):
                    print("\n[photo_capture] 已取消更换类型")
                    continue
                try:
                    session.set_type(new_type)
                except (OSError, ValueError) as error:
                    print("[photo_capture] 切换类型失败：{}".format(error))
                    continue
                print("[photo_capture] 已切换到「{}」，保存到 {}".format(
                    session.type_name, session.type_dir))
                continue
            if lowered == "n" or lowered.startswith("n "):
                rest = line[1:].strip()
                if rest:
                    session.count = parse_count(rest, session.count)
                else:
                    try:
                        raw = input("一次连拍张数（1-{}）[当前 {}]：".format(
                            MAX_BURST, session.count)).strip()
                    except (EOFError, KeyboardInterrupt):
                        print()
                        continue
                    session.count = parse_count(raw, session.count)
                print("[photo_capture] 一次连拍：{} 张".format(session.count))
                continue
            if line == "" or lowered in ("p", "s", "shoot"):
                shoot_cli(camera, session)
                continue
            print("未知指令，输入 h 查看帮助")
    finally:
        camera.release()

    print("[photo_capture] 退出，本次共保存 {} 张（{}）".format(
        session.total, display_path(session.type_dir)))
    return 0


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="调用本机摄像头拍照，或加载 YOLO 模型做实时检测。",
    )
    parser.add_argument("--mode", choices=("ask", "photo", "detect", "scan", "gauge", "thermal"),
                        default="ask",
                        help="启动模式：ask 每次询问，photo 拍照，detect 模型测试，"
                             "scan 区域扫描，gauge 仪表识别，thermal 红外热像")
    parser.add_argument("-d", "--output-dir", default=str(DEFAULT_OUTPUT_DIR),
                        help="照片根目录（默认：~/Pictures/photo_capture）")
    parser.add_argument("-t", "--type", default=None,
                        help="预填照片类型；命令行模式下不再询问")
    parser.add_argument("-c", "--count", type=int, default=1,
                        help="一次连拍张数，1-{}（默认 1）".format(MAX_BURST))
    parser.add_argument("--device", default="0",
                        help="摄像头编号 / 设备路径 / 视频文件（默认 0，即 /dev/video0）")
    parser.add_argument("--model", default=None,
                        help="YOLO 权重路径或文件名；模型测试模式不填则启动时选择")
    parser.add_argument("--model-dir", default=str(DEFAULT_MODEL_DIR),
                        help="模型目录（默认：~/AI视觉）")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="检测置信度阈值 0.01-0.99（默认 0.25）")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="推理输入尺寸（默认 640；CPU 上可用 416 提速）")
    parser.add_argument("--infer-device", choices=("auto", "cpu", "cuda"),
                        default="auto",
                        help="推理设备（默认 auto：有 CUDA 用 GPU，否则 CPU）")
    parser.add_argument("--width", type=int, default=1280,
                        help="采集宽度（默认 1280；0 表示用摄像头默认值）")
    parser.add_argument("--height", type=int, default=720,
                        help="采集高度（默认 720；0 表示用摄像头默认值）")
    parser.add_argument("--fps", type=float, default=0, help="采集帧率，0 表示用摄像头默认值")
    parser.add_argument("--sharpness", type=int, default=3,
                        help="摄像头锐度 0-6（默认 3；6 更硬但噪点更明显；-1 保持原值）")
    parser.add_argument("--pixel-format", choices=("auto", "mjpg", "yuyv"),
                        default="auto",
                        help="取流格式：auto 默认高分辨率用 MJPG；yuyv 未压缩但帧率低")
    parser.add_argument("--format", choices=("jpg", "png"), default="jpg",
                        help="图片格式（默认 jpg）")
    parser.add_argument("--quality", type=int, default=95,
                        help="JPEG 质量 1-100（默认 95，仅 jpg 有效）")
    parser.add_argument("--interval", type=float, default=0.4,
                        help="连拍时两张之间的间隔秒数（默认 0.4）")
    parser.add_argument("--mirror", action="store_true",
                        help="水平镜像（自拍用，预览和保存都镜像）")
    parser.add_argument("--no-gui", "--headless", dest="no_gui", action="store_true",
                        help="不打开预览窗口，用命令行交互")

    # ---- 区域扫描（平移拼图） ----
    parser.add_argument("--scan-mode", choices=("strafe", "manual"), default="strafe",
                        help="区域扫描方式：strafe 自动平移（默认），manual 手动录制")
    parser.add_argument("--scan-frames", type=int, default=5,
                        help="平移扫描总帧数（默认 5）")
    parser.add_argument("--scan-distance", type=float, default=1.0,
                        help="平移总距离，米（默认 1.0）")
    parser.add_argument("--scan-direction", choices=("left", "right"), default="left",
                        help="平移方向（默认 left）")
    parser.add_argument("--scan-speed", type=float, default=0.15,
                        help="平移速度 m/s（默认 0.15，上限 0.15）")
    parser.add_argument("--scan-settle", type=float, default=0.5,
                        help="每步停稳时间秒（默认 0.5）")
    parser.add_argument("--scan-fps", type=float, default=3.0,
                        help="手动录制抽帧帧率（默认 3）")
    parser.add_argument("--scan-video-fps", type=float, default=15.0,
                        help="sweep.avi 连续录制帧率（默认 15）")
    parser.add_argument("--scan-label", default="",
                        help="区域名称（默认时间戳）")
    parser.add_argument("--scan-feedback", choices=("auto", "odom", "time"),
                        default="auto",
                        help="平移距离反馈：auto / odom / time（默认 auto）")
    parser.add_argument("--scan-clearance", type=float, default=0.25,
                        help="平移前雷达净空，米；0 关闭（默认 0.25）")
    parser.add_argument("--stitch-max-width", type=int, default=1600,
                        help="拼接时每帧最大宽度（默认 1600）")
    parser.add_argument("--scan-record-video", action="store_true",
                        help="同时保存 sweep.avi")
    parser.add_argument("--no-stitch", dest="scan_stitch", action="store_false",
                        help="只采集，不自动拼接")
    parser.add_argument("--scan-dry-run", action="store_true",
                        help="区域扫描干跑：不连接 ROS、不产生真实运动（测试用）")
    parser.add_argument("--detect-panorama", action="store_true",
                        help="区域扫描拼接完成后用 YOLO 分块检测全景")
    parser.add_argument("--yes", action="store_true",
                        help="命令行自动扫描不再询问确认")
    parser.add_argument("--force", action="store_true",
                        help="忽略 /cmd_vel 上其他发布者的冲突警告")

    # ---- 仪表识别（Pose） ----
    parser.add_argument("--pose-model", default=str(DEFAULT_POSE_MODEL),
                        help="仪表 Pose 权重路径（默认：~/AI视觉/Pose.pt）")
    parser.add_argument("--gauge-conf", type=float, default=0.5,
                        help="仪表目标置信度阈值（默认 0.5）")
    parser.add_argument("--gauge-kpt-conf", type=float, default=0.3,
                        help="4 个关键点的最低置信度（默认 0.3）")
    parser.add_argument("--gauge-arc", choices=("long", "short", "auto"),
                        default="long",
                        help="指针刻度弧：long 长弧（默认，常见表计）/ short / auto")
    parser.add_argument("--gauge-pointer", choices=("radon", "tip"),
                        default="radon",
                        help="指针角度：radon 正视化后用射线搜索（默认）/ tip 模型尖端")
    parser.add_argument("--gauge-temp-min", type=float, default=0.0,
                        help="温度量程下限（默认 0）")
    parser.add_argument("--gauge-temp-max", type=float, default=100.0,
                        help="温度量程上限（默认 100）")
    parser.add_argument("--gauge-humidity-min", type=float, default=0.0,
                        help="湿度量程下限（默认 0）")
    parser.add_argument("--gauge-humidity-max", type=float, default=100.0,
                        help="湿度量程上限（默认 100）")

    # ---- 红外热像（MLX90640） ----
    parser.add_argument("--thermal-capture", "--capture", dest="thermal_capture", default="",
                        help="红外热像回放 capture.bin（无硬件测试用）")
    parser.add_argument("--thermal-port", default="/dev/thermal",
                        help="红外热像串口（默认 /dev/thermal，实机用）")
    parser.add_argument("--thermal-scale", type=int, default=12,
                        help="热像伪彩放大倍数（默认 12）")
    parser.add_argument("--thermal-delta", type=float, default=2.0,
                        help="热点框温度差（默认 2.0℃）")
    parser.add_argument("--thermal-floor-table", default="4F:0-5,3F:6-11,2F:12-17,1F:18-23",
                        help="楼层表，默认四层等分 4F:0-5,3F:6-11,2F:12-17,1F:18-23（现场必须重录）")
    parser.add_argument("--thermal-replay-rate", type=float, default=2.0,
                        help="capture.bin 回放帧率（默认 2Hz）")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    output_root = Path(args.output_dir).expanduser().resolve()
    try:
        output_root.mkdir(parents=True, exist_ok=True)
    except OSError as error:
        print("[photo_capture] 无法创建输出目录 {}：{}".format(output_root, error),
              file=sys.stderr)
        return 1

    use_gui = (not args.no_gui) and has_display() and TK_AVAILABLE and PIL_AVAILABLE
    if not use_gui:
        if not args.no_gui and has_display():
            print("[photo_capture] 缺少 tkinter 或 Pillow，使用命令行模式")
        return run_cli(args, output_root)

    try:
        return run_gui(args, output_root)
    except tk.TclError as error:
        print("[photo_capture] 图形界面启动失败（{}），改用命令行模式".format(error))
        return run_cli(args, output_root)


if __name__ == "__main__":
    sys.exit(main())
