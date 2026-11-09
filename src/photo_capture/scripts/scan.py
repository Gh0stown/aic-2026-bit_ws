#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""区域扫描（平移拼图）的采集与拼接。

只做平移场景：
  * strafe：控制底盘向左/右横移，停一下拍一张，最后拼成横向大图；
  * manual：不控制底盘，用户用遥控器平移，脚本按固定帧率抽帧，停止后拼接。

不依赖 Tk；UI 通过事件队列拿预览帧和进度。
"""

import json
import math
import queue
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import cv2

import stitch


class ScanError(RuntimeError):
    """采集流程错误。"""


class ScanAborted(ScanError):
    """用户主动停止。"""


def sanitize_label(name: object) -> str:
    if name is None:
        return ""
    text = str(name).strip()
    text = text.replace("/", "_").replace("\\", "_")
    text = re.sub(r'[\x00-\x1f:*?"<>|]', "_", text)
    text = re.sub(r"\s+", " ", text).strip(" .")
    return text[:40].rstrip(" .")


@dataclass
class ScanSettings:
    mode: str = "strafe"          # strafe / manual
    frames: int = 5
    distance_m: float = 1.0
    direction: str = "left"       # left / right
    speed: float = 0.15           # m/s，与巡航脚本一致
    settle: float = 0.5
    label: str = ""
    auto_stitch: bool = True
    record_video: bool = False
    feedback: str = "auto"        # auto / odom / time
    clearance: float = 0.25
    fps: float = 3.0              # manual 模式抽帧帧率
    quality: int = 95

    def validate(self) -> "ScanSettings":
        if self.mode not in ("strafe", "manual"):
            raise ValueError("扫描方式必须是 strafe 或 manual")
        self.frames = max(2, min(30, int(self.frames)))
        self.distance_m = max(0.02, min(5.0, float(self.distance_m)))
        self.speed = max(0.02, min(0.15, float(self.speed)))
        self.settle = max(0.1, min(3.0, float(self.settle)))
        self.direction = "right" if str(self.direction).lower() == "right" else "left"
        self.fps = max(1.0, min(10.0, float(self.fps)))
        self.quality = max(50, min(100, int(self.quality)))
        self.label = sanitize_label(self.label)
        if self.mode == "strafe" and self.frames > 1:
            step = self.distance_m / (self.frames - 1)
            if step > 0.35:
                raise ValueError("步长 {:.2f}m 太大，请增加帧数或减小总距离".format(step))
        if self.mode == "manual":
            self.distance_m = 0.0
        return self

    @property
    def step_m(self) -> float:
        if self.mode != "strafe" or self.frames < 2:
            return 0.0
        return self.distance_m / (self.frames - 1)

    def summary(self) -> str:
        if self.mode == "manual":
            return "手动平移录制，抽帧 {:.0f} fps".format(self.fps)
        return "{} 平移 {:.2f}m，{} 帧，每步 {:.2f}m".format(
            "向左" if self.direction == "left" else "向右",
            self.distance_m, self.frames, self.step_m)


class ScanSession:
    """一次扫描的目录、帧和 meta.json。"""

    def __init__(self, output_root, settings: ScanSettings,
                 camera_info: Optional[dict] = None):
        self.settings = settings
        self.root = Path(output_root) / "区域扫描"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = sanitize_label(settings.label) or "scan"
        self.directory = self.root / "{}_{}".format(name, stamp)
        self.frames_dir = self.directory / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.frame_paths: List[Path] = []
        self._video = None
        self._video_size = None
        self._meta = {
            "version": 1,
            "mode": settings.mode,
            "label": settings.label or name,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "camera": camera_info or {},
            "scan": {
                "frames": settings.frames,
                "distance_m": settings.distance_m,
                "step_m": round(settings.step_m, 4),
                "direction": settings.direction,
                "speed": settings.speed,
                "settle": settings.settle,
                "feedback": settings.feedback,
                "fps": settings.fps,
            },
            "frames": [],
            "stitch": None,
        }

    @property
    def panorama_path(self) -> Path:
        return self.directory / "panorama.jpg"

    @property
    def meta_path(self) -> Path:
        return self.directory / "meta.json"

    def add_frame(self, frame, pose: Optional[dict] = None,
                  stamp: Optional[float] = None) -> Path:
        index = len(self.frame_paths) + 1
        path = self.frames_dir / "{:04d}.jpg".format(index)
        params = [int(cv2.IMWRITE_JPEG_QUALITY), self.settings.quality]
        if not cv2.imwrite(str(path), frame, params):
            raise ScanError("写入帧失败：{}".format(path))
        self.frame_paths.append(path)
        self._meta["frames"].append({
            "index": index,
            "file": "frames/{:04d}.jpg".format(index),
            "time": round(stamp if stamp is not None else time.time(), 3),
            "pose": pose,
        })
        self._write_meta()
        if self.settings.record_video:
            self._write_video(frame)
        return path

    def _write_video(self, frame) -> None:
        if self._video is None:
            height, width = frame.shape[:2]
            self._video_size = (width, height)
            fps = max(2.0, min(15.0, self.settings.fps if self.settings.mode == "manual" else 2.0))
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self._video = cv2.VideoWriter(
                str(self.directory / "sweep.avi"), fourcc, fps, self._video_size)
        if self._video_size != (frame.shape[1], frame.shape[0]):
            frame = cv2.resize(frame, self._video_size)
        self._video.write(frame)

    def set_stitch(self, result: Optional[stitch.StitchResult]) -> None:
        if result is None:
            self._meta["stitch"] = None
        else:
            self._meta["stitch"] = {
                "ok": bool(result.ok),
                "method": result.method,
                "width": result.width,
                "height": result.height,
                "message": result.message,
                "frame_count": result.frame_count,
                "file": result.path.name if result.path else None,
            }
        self._write_meta()

    def set_detections(self, detections, annotated_path: Optional[Path] = None) -> None:
        self._meta["detections"] = detections or []
        if annotated_path is not None:
            self._meta["detections_file"] = annotated_path.name
        self._write_meta()

    def _write_meta(self) -> None:
        try:
            tmp = self.meta_path.with_suffix(".json.tmp")
            with tmp.open("w", encoding="utf-8") as handle:
                json.dump(self._meta, handle, ensure_ascii=False, indent=2)
            tmp.replace(self.meta_path)
        except OSError:
            pass

    def close(self) -> None:
        if self._video is not None:
            try:
                self._video.release()
            except Exception:
                pass
            self._video = None


@dataclass
class ScanEvent:
    kind: str                 # status / preview / frame / progress / error / done
    index: int = 0
    total: int = 0
    frame: object = None
    message: str = ""
    result: object = None


class ScanJob:
    """在线程里跑采集/拼接，UI/CLI 轮询 events。"""

    def __init__(self, camera, session: ScanSession, settings: ScanSettings,
                 chassis=None, stitch_max_width: int = 1600):
        self.camera = camera
        self.session = session
        self.settings = settings
        self.chassis = chassis
        self.stitch_max_width = stitch_max_width
        self.events = queue.Queue()
        self._stop = threading.Event()
        self._thread = None
        self.running = False
        self._last_preview = 0.0
        self._last_progress = 0.0

    # -- 对外 ---------------------------------------------------------
    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()
        self.running = True
        self._thread = threading.Thread(
            target=self._run, name="scan-job", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self.chassis is not None:
            try:
                self.chassis.stop()
            except Exception:
                pass

    def poll(self) -> List[ScanEvent]:
        events = []
        while True:
            try:
                events.append(self.events.get_nowait())
            except queue.Empty:
                break
        return events

    def join(self, timeout: Optional[float] = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    # -- 内部 ---------------------------------------------------------
    def _put(self, kind: str, **kwargs) -> None:
        self.events.put(ScanEvent(kind=kind, **kwargs))

    def _status(self, text: str, error: bool = False) -> None:
        self._put("status", message=text)

    def _run(self) -> None:
        error = ""
        try:
            if self.settings.mode == "strafe":
                self._run_strafe()
            else:
                self._run_manual()
        except ScanAborted:
            self._status("已停止采集，准备拼接已拍帧…")
        except Exception as exc:
            if self._stop.is_set():
                self._status("已停止采集，准备拼接已拍帧…")
            else:
                error = str(exc)
                self._put("error", message=error)
        finally:
            if self.chassis is not None:
                try:
                    self.chassis.stop()
                except Exception:
                    pass
            result = None
            if self.settings.auto_stitch and len(self.session.frame_paths) >= 2:
                try:
                    result = self._stitch()
                except Exception as exc:
                    error = error or str(exc)
                    self._put("error", message="拼接异常：{}".format(exc))
            elif len(self.session.frame_paths) < 2:
                error = error or "有效帧不足 2 张，无法拼接"
                self._put("error", message=error)
            self.session.close()
            self.running = False
            self._put("done", result=result, message=error)

    def _sleep(self, seconds: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end:
            if self._stop.is_set():
                raise ScanAborted()
            time.sleep(0.05)

    def _read_frame(self):
        for _ in range(5):
            frame = self.camera.read()
            if frame is not None:
                return frame
            time.sleep(0.05)
            if self._stop.is_set():
                raise ScanAborted()
        raise ScanError("摄像头没有画面")

    def _pose(self):
        if self.chassis is None:
            return None
        try:
            return self.chassis.last_pose()
        except Exception:
            return None

    def _capture(self, index: int, total: int, note: str = "") -> None:
        if self._stop.is_set():
            raise ScanAborted()
        frame = self._read_frame()
        pose = self._pose()
        self.session.add_frame(frame, pose=pose)
        self._put("frame", index=index, total=total, frame=frame,
                  message=note, result=pose)
        self._put("progress", index=index, total=total,
                  message="已拍 {}/{}".format(index, total))

    def _preview(self, frame) -> None:
        now = time.monotonic()
        if now - self._last_preview >= 0.06:
            self._last_preview = now
            self._put("preview", frame=frame)

    def _run_strafe(self) -> None:
        if self.chassis is None:
            raise ScanError("自动平移需要底盘控制，当前没有连接底盘")
        total = self.settings.frames
        self._status("3 秒后开始平移扫描，随时可以按停止")
        for remaining in (3, 2, 1):
            self._put("status", message="{} 秒后开始…".format(remaining))
            self._sleep(1.0)

        self._capture(1, total, "起始位置")
        step = self.settings.step_m
        for index in range(1, total):
            if self._stop.is_set():
                raise ScanAborted()
            self._put("status", message="平移第 {}/{} 步（{:.2f}m）".format(
                index, total - 1, step))
            self.chassis.strafe(
                step, self.settings.speed, self.settings.direction,
                stop_event=self._stop,
                clearance=self.settings.clearance,
                feedback=self.settings.feedback,
            )
            self._sleep(self.settings.settle)
            self._capture(index + 1, total, "第 {} 步".format(index))
        self._status("采集完成，正在拼接…")

    def _run_manual(self) -> None:
        self._status("手动录制中：请用遥控平移，停止后自动拼接")
        interval = 1.0 / max(1.0, self.settings.fps)
        next_capture = time.monotonic()
        preview_at = 0.0
        index = 0
        while not self._stop.is_set():
            frame = self.camera.read()
            now = time.monotonic()
            if frame is None:
                time.sleep(0.05)
                continue
            if now - preview_at >= 0.05:
                preview_at = now
                self._preview(frame)
            if now >= next_capture:
                index += 1
                self.session.add_frame(frame, pose=None)
                self._put("frame", index=index, total=0, frame=frame,
                          message="录制帧 {}".format(index))
                self._put("progress", index=index, total=0,
                          message="已录制 {} 帧".format(index))
                next_capture = now + interval
            time.sleep(0.01)
        self._status("停止录制，正在拼接…")

    def _stitch(self):
        self._put("status", message="正在拼接 {} 帧…".format(
            len(self.session.frame_paths)))
        self._put("progress", index=0, total=0, message="拼接中…")

        def progress(stage: str, value: float) -> None:
            now = time.monotonic()
            if now - self._last_progress >= 0.15 or value >= 1.0:
                self._last_progress = now
                self._put("progress", index=0, total=0,
                          message="拼接：{} {:.0f}%".format(stage, value * 100))

        result = stitch.stitch_frames(
            self.session.frame_paths, mode="strafe",
            max_width=self.stitch_max_width,
            output_path=self.session.panorama_path,
            progress=progress,
        )
        self.session.set_stitch(result)
        if result.ok:
            self._put("status", message="拼接完成：{}x{}".format(
                result.width, result.height))
        else:
            self._put("error", message="拼接失败：{}".format(result.message))
        return result
