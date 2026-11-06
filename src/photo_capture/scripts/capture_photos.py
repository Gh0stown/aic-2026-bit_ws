#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本机摄像头手动拍照采集脚本。

特点：
  * 启动时询问本次要拍的照片类型，照片保存到 <输出目录>/<类型>/ 下；
  * 一次拍照可以连拍 N 张（N 随时可改），也可以单张拍；
  * 运行中随时可以切换类型，换类型后自动建对应的文件夹；
  * 有桌面环境时用 tkinter 做预览窗口（OpenCV 只负责读摄像头和写图片），
    没有图形界面时自动退化为命令行交互模式。

快捷键（图形界面）：
  空格 / 回车  拍照（按设定张数连拍）
  T            更换类型
  N            修改一次连拍张数
  Esc          退出

命令行模式（--no-gui）：
  直接回车 / p  拍照
  t [类型]      更换类型
  n [张数]      修改连拍张数
  q             退出
"""

import argparse
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple, Union

import cv2

try:  # tkinter 可能没装，此时退回命令行模式
    import tkinter as tk
    import tkinter.font as tkfont
    from tkinter import messagebox, ttk

    TK_AVAILABLE = True
except Exception:  # pragma: no cover - 取决于运行环境
    tk = None
    tkfont = None
    ttk = None
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


def has_display() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


# ---------------------------------------------------------------------------
# 摄像头
# ---------------------------------------------------------------------------

class Camera:
    def __init__(self, source: Union[int, str], width: int = 0, height: int = 0,
                 fps: float = 0.0, mirror: bool = False):
        self.source = source
        self.width = int(width or 0)
        self.height = int(height or 0)
        self.fps = float(fps or 0.0)
        self.mirror = bool(mirror)
        self.cap = None
        self.frame_size = (0, 0)

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
            if self.width > 0:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            if self.height > 0:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if self.fps > 0:
                cap.set(cv2.CAP_PROP_FPS, self.fps)
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

class AskTypeDialog(tk.Toplevel if TK_AVAILABLE else object):
    """询问照片类型（可选连拍张数）的模态小窗口。"""

    def __init__(self, parent, output_root: Path, initial_type: str = "",
                 initial_count: int = 1, with_count: bool = True,
                 title: str = APP_TITLE):
        if not TK_AVAILABLE:
            raise RuntimeError("tkinter 不可用")
        super().__init__(parent)
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
        body = ttk.Frame(self, padding=(20, 16))
        body.pack(fill="both", expand=True)
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
                activestyle="dotbox", font=("", 11),
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
        self.ok_button = ttk.Button(buttons, text="开始拍照", command=self._ok)
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
        top = ttk.Frame(self.root, padding=(14, 12, 14, 4))
        top.pack(fill="x")

        ttk.Label(top, text="类型：").pack(side="left")
        ttk.Label(top, textvariable=self.type_var, font=self._bold_font).pack(side="left")
        ttk.Button(top, text="更换类型", command=self.change_type).pack(
            side="left", padx=(10, 0))

        ttk.Label(top, text="一次连拍：").pack(side="left", padx=(18, 2))
        self.count_spin = ttk.Spinbox(
            top, from_=1, to=MAX_BURST, width=4, textvariable=self.count_var,
            command=self._on_count_change)
        self.count_spin.pack(side="left")
        ttk.Label(top, text="张").pack(side="left", padx=(2, 0))

        self.shoot_button = ttk.Button(top, text="拍照（空格）", command=self.shoot)
        self.shoot_button.pack(side="right")

        self.preview = tk.Label(
            self.root, bg="#141414", fg="#bbbbbb",
            text="正在打开摄像头…", takefocus=True,
        )
        self.preview.pack(fill="both", expand=True, padx=14, pady=6)
        self.preview.bind("<Button-1>", lambda event: self.preview.focus_set())

        bottom = ttk.Frame(self.root, padding=(14, 0, 14, 10))
        bottom.pack(fill="x")
        self.status_label = ttk.Label(
            bottom, textvariable=self.status_var, foreground="#444444")
        self.status_label.pack(side="left")
        ttk.Button(bottom, text="打开保存目录", command=self.open_folder).pack(side="right")

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
            photo = ImageTk.PhotoImage(image)
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


# ---------------------------------------------------------------------------
# 图形界面启动流程
# ---------------------------------------------------------------------------

def run_gui(args, output_root: Path) -> int:
    root = tk.Tk()
    root.withdraw()
    root.title(APP_TITLE)

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


def print_summary(session: Session, camera: Camera) -> None:
    width, height = camera.frame_size
    print("[photo_capture] 类型：{}".format(session.type_name))
    print("[photo_capture] 一次连拍：{} 张，间隔 {:g} 秒".format(
        session.count, session.interval))
    print("[photo_capture] 保存目录：{}".format(session.type_dir))
    if width and height:
        print("[photo_capture] 摄像头画面：{}x{}".format(width, height))


def print_cli_help() -> None:
    print("操作说明：")
    print("  直接回车 / p    按当前张数拍一次")
    print("  t [类型]        更换类型，例如：t 侧面")
    print("  n [张数]        修改一次连拍张数，例如：n 3")
    print("  q               退出")


def run_cli(args, output_root: Path) -> int:
    print("[photo_capture] 输出根目录：{}".format(display_path(output_root)))
    print("[photo_capture] 命令行模式（没有图形界面时使用）")

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
        description="调用本机摄像头手动拍照：按类型分文件夹保存，可设置一次连拍张数。",
    )
    parser.add_argument("-d", "--output-dir", default=str(DEFAULT_OUTPUT_DIR),
                        help="照片根目录（默认：~/Pictures/photo_capture）")
    parser.add_argument("-t", "--type", default=None,
                        help="预填照片类型；命令行模式下不再询问")
    parser.add_argument("-c", "--count", type=int, default=1,
                        help="一次连拍张数，1-{}（默认 1）".format(MAX_BURST))
    parser.add_argument("--device", default="0",
                        help="摄像头编号 / 设备路径 / 视频文件（默认 0，即 /dev/video0）")
    parser.add_argument("--width", type=int, default=0, help="采集宽度，0 表示用摄像头默认值")
    parser.add_argument("--height", type=int, default=0, help="采集高度，0 表示用摄像头默认值")
    parser.add_argument("--fps", type=float, default=0, help="采集帧率，0 表示用摄像头默认值")
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
