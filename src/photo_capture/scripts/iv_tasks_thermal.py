#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""task=temperature（楼宇 D 高温楼层）。

链路（与 photo_capture 的红外模式一致）：
- `mlx90640_ros.thermal_session.ThermalSession` 后台取红外帧（24x32 摄氏度矩阵）；
- 在 ROI 内取峰值行，按 `thermal.floor_table`（行号从图像顶部数）映射到楼层；
- 用 `core.render_image(draw_box=True)` 自己渲染带绿框热点图（session 自带的
  bgr 默认不画框）；
- 多帧取峰值行中位数，抗单帧抖动；无新帧或楼层表外一律判失败，不猜楼层。

注意：`building_d.pt` 是楼体四角 pose 模型，当前现有标定与窗模板对不齐，
不作为判层依据（详见 docs/inspection_vision_node.md）。
"""

from __future__ import annotations

import re
import time
from typing import Any, Dict, List, Optional, Tuple

from iv_common import TaskResult, VisionContext, VisionError

_SESSION = None
_SESSION_KEY: Optional[Tuple[Any, ...]] = None


# ---------------------------------------------------------------------------
# 会话管理（一个进程只能有一个 Decoder）
# ---------------------------------------------------------------------------

def _session_key(config: Dict[str, Any]) -> Tuple[Any, ...]:
    thermal = config.get("thermal", {})
    return (str(thermal.get("port") or ""), str(thermal.get("capture") or ""),
            int(thermal.get("scale", 12)), float(thermal.get("delta", 2.0)),
            str(thermal.get("floor_table") or ""))


def get_session(context: VisionContext):
    global _SESSION, _SESSION_KEY
    thermal = context.config.get("thermal", {})
    key = _session_key(context.config)
    if _SESSION is not None and _SESSION_KEY == key:
        return _SESSION
    if _SESSION is not None:
        close_session()
    try:
        from mlx90640_ros.thermal_session import ThermalSession
    except Exception as error:  # noqa: BLE001
        raise VisionError(
            "无法导入 mlx90640_ros（先 source devel/setup.bash 且用系统 python3）：{}"
            .format(error))
    port = str(thermal.get("port") or "").strip()
    capture = str(thermal.get("capture") or "").strip()
    if not port and not capture:
        raise VisionError("thermal.port 和 thermal.capture 必须设置一个"
                          "（实机用 /dev/serial/by-id/...，回放用 capture.bin）")
    if port and capture:
        raise VisionError("thermal.port 和 thermal.capture 只能设置一个")
    kwargs: Dict[str, Any] = {
        "scale": int(thermal.get("scale", 12)),
        "delta_c": float(thermal.get("delta", 2.0)),
        "floor_table": str(thermal.get("floor_table") or ""),
        "tmin": 20.0,
        "tmax": 35.0,
    }
    if capture:
        kwargs["capture"] = capture
        kwargs["loop"] = True
        kwargs["replay_rate"] = 2.0
    else:
        kwargs["port"] = port
    try:
        session = ThermalSession(**kwargs).start()
    except Exception as error:  # noqa: BLE001
        raise VisionError("启动红外相机失败（{}）：{}".format(port or capture, error))
    _SESSION = session
    _SESSION_KEY = key
    return session


def close_session() -> None:
    global _SESSION, _SESSION_KEY
    if _SESSION is not None:
        try:
            _SESSION.stop()
        except Exception:  # noqa: BLE001
            pass
    _SESSION = None
    _SESSION_KEY = None


# ---------------------------------------------------------------------------
# 取帧 / 判层
# ---------------------------------------------------------------------------

def _fresh_frames(session, context: VisionContext,
                  count: int = 3) -> List[Any]:
    """在截止时间内收集 count 个“新”帧（同一帧只取一次）。"""
    thermal = context.config.get("thermal", {})
    timeout = min(float(thermal.get("capture_timeout", 3.0)),
                  max(0.5, context.remaining() - 0.5))
    deadline = time.monotonic() + timeout
    frames: List[Any] = []
    last_stamp = None
    while time.monotonic() < deadline:
        context.check_deadline("等红外帧")
        error = session.error()
        if error is not None:
            raise VisionError("红外相机异常：{}".format(error))
        frame = session.latest()
        if frame is not None and time.monotonic() - frame.monotonic <= 3.0:
            if last_stamp is None or frame.monotonic > last_stamp:
                frames.append(frame)
                last_stamp = frame.monotonic
                if len(frames) >= count:
                    break
        time.sleep(0.15)
    if not frames:
        raise VisionError("在 %.1f s 内没有收到新的红外帧（检查 /dev/thermal 与串口权限）"
                          % timeout)
    return frames


def _floor_number(context: VisionContext, session, row: float) -> Tuple[str, Optional[int]]:
    table = session.floor_table
    bands = list(table.bands)
    raw = table.floor_for(float(row))
    for name, first, last in bands:
        if first <= row <= last:
            break
    else:
        return raw or "", None
    match = re.search(r"(\d+)", str(raw or ""))
    return str(raw or ""), (int(match.group(1)) if match else None)


# ---------------------------------------------------------------------------
# task 处理
# ---------------------------------------------------------------------------

def handle(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    thermal = context.config.get("thermal", {})
    building = str(params.get("building") or thermal.get("building") or "D").strip()

    session = get_session(context)
    frames = _fresh_frames(session, context, count=3)
    rows = sorted(float(frame.hotspot.row) for frame in frames)
    row = rows[len(rows) // 2]
    frame = frames[-1]
    floor_raw, floor_number = _floor_number(context, session, row)
    summary = frame.summary()

    if floor_number is None:
        return context.fail(
            "热点行 {:.1f} 不在楼层表（{}）内，请现场重录 thermal.floor_table"
            .format(row, session.floor_table.to_text() or "(空表)"),
            result="{}大楼高温楼层未标定".format(building),
            extra={"building": building, "floor": None, "floor_raw": floor_raw,
                   "row": round(row, 3), "peak_c": summary.get("peak_c"),
                   "frame_count": len(frames)})

    context.check_deadline("绘制热像图")
    try:
        from mlx90640_ros import core
        rendered = core.render_image(
            core.Sample(frame.temps, frame.ambient, 0, 1),
            scale=int(thermal.get("scale", 12)),
            tmin=float(thermal.get("tmin", 20.0)),
            tmax=float(thermal.get("tmax", 35.0)),
            hotspot=frame.hotspot,
            floor=frame.floor,
            draw_box=True,
        )
    except Exception as error:  # noqa: BLE001
        raise VisionError("渲染热像图失败：{}".format(error))

    text = "{}大楼在 {} 楼处发现高温情况".format(building, floor_number)
    image = context.save_frame(
        rendered, tag="thermal",
        caption=text,
        extra_lines=["热点 {:.1f}°C 行 {:.1f} → {}（多帧 {}）".format(
            float(summary.get("peak_c", 0.0)), row, floor_raw, len(frames))])
    extra = {
        "building": building,
        "floor": floor_number,
        "floor_raw": floor_raw,
        "row": round(row, 3),
        "peak_c": summary.get("peak_c"),
        "ambient": summary.get("ambient"),
        "frame_count": len(frames),
        "floor_table": session.floor_table.to_text(),
    }
    return context.ok(text, extra, [image])
