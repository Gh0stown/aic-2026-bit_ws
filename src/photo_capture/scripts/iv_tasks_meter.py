#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""task=meter（站房温湿度表，Exp16 识别链路）。

复用 `gauge_exp16.Exp16Reader`（YOLO 定位 wendu/shidu + OpenCV 读数）和
`gauge_exp16_stability.StabilityBatch`（10 帧起、最多 20 帧、主簇 ≥5 帧且
≥70%、前后半段各 ≥2 帧），与 GUI 模式 7 是同一套算法。

与 GUI 的差异：节点必须自己执行截止时间，不无限续轮；`reference` 不算成功，
缺数/超时不填 0、不拿部分冒充完整（只把部分读数如实放进 extra）。
"""

from __future__ import annotations

import hashlib
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from iv_common import default_gauge_model, TaskResult, VisionContext, VisionError

_READER = None
_READER_KEY: Optional[str] = None


# ---------------------------------------------------------------------------
# 模型与 SHA 校验
# ---------------------------------------------------------------------------

def _model_path(context: VisionContext) -> Path:
    configured = str(context.config.get("meter", {}).get("model") or "").strip()
    path = Path(configured).expanduser() if configured else default_gauge_model()
    if not path.is_absolute() and configured:
        path = context.models.model_dir / path
    return path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(str(path), "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _get_reader(context: VisionContext):
    global _READER, _READER_KEY
    meter = context.config.get("meter", {})
    path = _model_path(context)
    if not path.is_file():
        raise VisionError(
            "Exp16 仪表模型不存在：{}（交接包 best.pt，不在 Git；"
            "放到 src/photo_capture/models/gauge_exp16.pt 或用 meter.model 指定）"
            .format(path))
    key = str(path)
    if _READER is not None and _READER_KEY == key:
        return _READER
    expected = str(meter.get("sha256") or "").strip().lower()
    if expected and bool(meter.get("verify_sha256", True)):
        actual = _sha256(path)
        if actual != expected:
            raise VisionError(
                "Exp16 模型 SHA-256 不匹配：期望 {}…，实际 {}…；"
                "按交接文档不要用其他权重，如确认要换请改 meter.sha256 或关掉 verify_sha256"
                .format(expected[:12], actual[:12]))
    try:
        from gauge_exp16 import Exp16Reader
    except Exception as error:  # noqa: BLE001
        raise VisionError("无法导入 gauge_exp16：{}".format(error))
    device = "cpu"
    _READER = Exp16Reader(str(path), device=device)
    _READER_KEY = key
    return _READER


def warmup(context: VisionContext) -> None:
    """启动预热：加载 Exp16 权重（约 2.7 s），避免第一次识别吃掉截止时间。"""
    _get_reader(context)


# ---------------------------------------------------------------------------
# 单帧读数
# ---------------------------------------------------------------------------

def _reading_value(reading) -> Optional[float]:
    if getattr(reading, "status", "") == "ok" and getattr(reading, "value", None) is not None:
        try:
            value = float(reading.value)
        except (TypeError, ValueError):
            return None
        return value if np.isfinite(value) else None
    return None


def _reading_dict(reading, dial) -> Dict[str, Any]:
    payload = dict(dial.as_dict()) if dial is not None else {}
    payload.setdefault("value", getattr(reading, "value", None))
    payload.setdefault("status", getattr(dial, "status", "missing"))
    payload["status"] = payload.get("status") or getattr(dial, "status", "missing")
    payload["valid_count"] = int(getattr(dial, "valid_count", 0) or 0)
    payload["inlier_count"] = int(getattr(dial, "inlier_count", 0) or 0)
    payload["value"] = payload.get("value")
    return payload


def _sample_frames(context: VisionContext, reader, batch):
    """按截止时间采样并喂 StabilityBatch；返回 (BatchResult 或 None, 帧数, 代表帧)。"""
    meter = context.config.get("meter", {})
    min_frames = int(meter.get("min_frames", 10))
    max_frames = int(meter.get("max_frames", 20))
    interval = float(meter.get("frame_interval", 0.12))
    temp_auto_ring = bool(meter.get("temp_auto_ring", True))
    humidity_auto_scale = bool(meter.get("humidity_auto_scale", False))

    measured = None
    representative = None
    attempts = 0
    valid = 0
    while attempts < max_frames:
        context.check_deadline("仪表采样")
        frame = context.frames.read()
        if frame is None:
            raise VisionError("仪表采样期间相机断流")
        attempts += 1
        try:
            result = reader.process(
                frame,
                temp_auto_ring=temp_auto_ring,
                humidity_auto_scale=humidity_auto_scale,
            )
        except ValueError as error:
            # 空帧/坏帧：记账后继续下一帧，不结束整次请求
            context.log("[vision] meter 单帧处理失败（第 %d 帧）：%s" % (attempts, error))
            continue
        representative = result
        temperature = _reading_value(result.temperature)
        humidity = _reading_value(result.humidity)
        if temperature is not None or humidity is not None:
            valid += 1
        measured = batch.add(temperature, humidity)
        if measured is not None:
            break
        if interval > 0:
            time.sleep(interval)
    return measured, attempts, valid, representative


# ---------------------------------------------------------------------------
# task
# ---------------------------------------------------------------------------

def handle(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    reader = _get_reader(context)
    context.note_model_name(_model_path(context).name)

    try:
        from gauge_exp16_stability import StabilityBatch
    except Exception as error:  # noqa: BLE001
        raise VisionError("无法导入 gauge_exp16_stability：{}".format(error))

    batch = StabilityBatch()
    started = time.monotonic()
    measured, attempts, valid, representative = _sample_frames(context, reader, batch)
    elapsed_ms = int((time.monotonic() - started) * 1000)
    if measured is None:
        return context.fail(
            "在截止时间内没有稳定读数（采样 {}/{} 帧，有数 {} 帧）".format(
                attempts, int(context.config["meter"].get("max_frames", 20)), valid),
            result="仪表读数未稳定",
            extra={"frame_count": attempts, "valid_frames": valid,
                   "elapsed_ms": elapsed_ms,
                   "log": str(context.jsonl_path)})

    temperature = measured.temperature
    humidity = measured.humidity
    temp_ok = getattr(temperature, "status", "") == "ok"
    humidity_ok = getattr(humidity, "status", "") == "ok"
    extra: Dict[str, Any] = {
        "temperature": _reading_dict(None, temperature),
        "humidity": _reading_dict(None, humidity),
        "frame_count": int(getattr(measured, "frame_count", attempts) or attempts),
        "sampled_frames": attempts,
        "valid_frames": valid,
        "elapsed_ms": elapsed_ms,
        "log": str(context.jsonl_path),
        "model": _model_path(context).name,
    }

    images: List[str] = []
    if representative is not None and representative.annotated is not None:
        caption_lines = []
        if temp_ok:
            caption_lines.append("温度表 %g 度" % temperature.value)
        if humidity_ok:
            caption_lines.append("湿度表 %g%%" % humidity.value)
        caption = "、".join(caption_lines) or "仪表读数未稳定"
        images.append(context.save_frame(
            representative.annotated, tag="meter", caption=caption,
            extra_lines=["温度 {} / 湿度 {}".format(temperature.status,
                                                    humidity.status)]))

    if not (temp_ok and humidity_ok):
        missing = []
        if not temp_ok:
            missing.append("温度表={}".format(getattr(temperature, "status", "")))
        if not humidity_ok:
            missing.append("湿度表={}".format(getattr(humidity, "status", "")))
        result = "仪表未读全（{}）".format("，".join(missing))
        return context.fail(result, result=result, extra=extra)

    text = "温度表读数为 %g 度。湿度表度数为 %g%%。" % (temperature.value, humidity.value)
    return context.ok(text, extra, images)
