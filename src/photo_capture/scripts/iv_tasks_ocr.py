#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""task=fire（楼宇火灾）与 task=plate（停车位车牌）。

复用 `ocr_engine` 的 PaddleOCR 单例：
- fire：`fire.pt` 检测火源计数，`house.pt` 裁楼宇名牌后 PaddleOCR 识名，
  识别结果先和配置里的候选名单做模糊匹配，匹配不上再用配置默认名兜底；
- plate：`plate.pt` 定位车牌后 PaddleOCR 读字（沙盘牌照必须用 generic 模式，
  HyperLPR3 会把首字“纠正”成省份）。
"""

from __future__ import annotations

import difflib
import re
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from iv_common import Detection, TaskResult, VisionContext, VisionError, safe_name

_CN_OCR = None
_PLATE_OCR = None


# ---------------------------------------------------------------------------
# OCR 引擎（懒加载 + 进程内复用，避免每次 +1~3 s）
# ---------------------------------------------------------------------------

def chinese_ocr():
    global _CN_OCR
    if _CN_OCR is None:
        try:
            import ocr_engine
        except Exception as error:  # noqa: BLE001
            raise VisionError("无法导入 ocr_engine：{}".format(error))
        _CN_OCR = ocr_engine.ChineseOcr()
    return _CN_OCR


def plate_ocr():
    global _PLATE_OCR
    if _PLATE_OCR is None:
        try:
            import ocr_engine
        except Exception as error:  # noqa: BLE001
            raise VisionError("无法导入 ocr_engine：{}".format(error))
        _PLATE_OCR = ocr_engine.PlateOcr()
    return _PLATE_OCR


def _detector_path(context: VisionContext, key: str) -> str:
    path = context.models.resolve(context.model_path(key))
    if not path.is_file():
        raise VisionError("模型不存在：{}".format(path))
    return str(path)


def _clean_text(text: str) -> str:
    """去掉 OCR 常见的空白和分隔符，保留中文/字母/数字。"""
    return re.sub(r"[\s·•・_—\-]+", "", str(text or ""))


# ---------------------------------------------------------------------------
# task=fire
# ---------------------------------------------------------------------------

def _building_config(context: VisionContext) -> Tuple[str, List[str]]:
    buildings = context.config.get("fire", {}).get("buildings", {}) or {}
    entry = buildings.get(str(context.point_id), {}) or {}
    default_name = str(entry.get("name") or "").strip()
    if not default_name:
        # sub_id 形如 “楼宇A”，去掉“楼宇”两字作为兜底名
        default_name = re.sub(r"^(楼宇|大厦)", "", context.sub_id or "").strip()
    if not default_name:
        default_name = context.sub_id or "该"
    candidates = [str(item) for item in (entry.get("candidates") or []) if str(item).strip()]
    return default_name, candidates


def _match_building(ocr_text: str, default_name: str,
                    candidates: List[str]) -> Tuple[str, str]:
    """返回 (楼宇名, 来源)；来源用于 extra 里区分 OCR/候选/默认。"""
    cleaned = _clean_text(ocr_text)
    if candidates:
        if cleaned in candidates:
            return cleaned, "ocr"
        matched = difflib.get_close_matches(cleaned, candidates, n=1, cutoff=0.4)
        if matched:
            return matched[0], "matched"
        return default_name, "default"
    # 没有候选名单：只接受“像名字”的短文本，避免把背景编号/广告字当楼宇名。
    # 规则：不含数字，且（有中文 或 只有 1-2 个字母）。
    if cleaned and len(cleaned) <= 8 and not re.search(r"\d", cleaned):
        if re.search(r"[\u4e00-\u9fff]", cleaned) or re.fullmatch(r"[A-Za-z]{1,2}", cleaned):
            return cleaned, "ocr"
    return default_name, "default"


def handle_fire(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    fire_conf = context.threshold("fire", 0.25)
    nameplate_conf = context.threshold("nameplate", 0.35)
    frames = context.grab()
    frame = frames[-1]

    fire_model = context.model(context.model_path("fire"), imgsz=640, conf=fire_conf)
    fire_boxes = fire_model.predict(frame, conf=fire_conf)
    context.check_deadline("火源检测")

    building_name, candidates = _building_config(context)
    building_source = "default"
    ocr_image: Optional[np.ndarray] = None
    ocr_message = ""
    raw_ocr_text = ""
    # 始终尝试 OCR；失败就用默认名，不因 OCR 失败判整个任务失败
    try:
        result = chinese_ocr().recognize(
            frame,
            detector_model=_detector_path(context, "nameplate"),
            detector_conf=nameplate_conf,
        )
        context.check_deadline("楼宇名 OCR")
        raw_ocr_text = "".join(line.text for line in (result.lines or []))
        ocr_image = result.annotated
        ocr_message = result.message or ""
        if raw_ocr_text.strip() or candidates:
            building_name, building_source = _match_building(
                raw_ocr_text, building_name, candidates)
    except Exception as error:  # noqa: BLE001
        # 火源数仍然有效；楼宇名用配置兜底，但要如实记录 OCR 失败原因
        ocr_message = "楼宇名 OCR 失败：{}".format(error)

    count = len(fire_boxes)
    text = "{}大厦发现火灾隐患 {} 个".format(building_name, count)
    extra = {
        "building": building_name,
        "count": count,
        "building_source": building_source,
        "ocr_message": ocr_message,
        "nameplate_ocr": _clean_text(raw_ocr_text),
        "detections": [box.as_dict() for box in fire_boxes],
    }
    images: List[str] = []
    main = context.save_frame(frame, tag="fire", detections=fire_boxes,
                              caption=text,
                              extra_lines=["楼宇名：{}（{}）".format(
                                  building_name, building_source)])
    images.append(main)
    if ocr_image is not None:
        images.append(context.save_frame(ocr_image, tag="nameplate",
                                         caption="楼宇名牌 OCR：{}".format(
                                             _clean_text(raw_ocr_text) or "未识别")))
    raw = context.save_raw(frame)
    if raw:
        extra["raw_image"] = raw
    return context.ok(text, extra, images)


# ---------------------------------------------------------------------------
# task=plate
# ---------------------------------------------------------------------------

def _parking_number(context: VisionContext) -> str:
    """从 sub_id（1号停车位）/点位号（14/15/16）推断“X 号停车场”。"""
    match = re.search(r"(\d+)", context.sub_id or "")
    if match:
        return match.group(1)
    mapping = {14: "1", 15: "2", 16: "3"}
    return mapping.get(context.point_id, str(context.point_id))


def handle_plate(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    config = context.config
    ocr_mode = str(params.get("ocr_mode") or config["plate"].get("ocr_mode", "generic"))
    ocr_mode = ocr_mode if ocr_mode in ("generic", "hyperlpr") else "generic"
    frames = context.grab()
    frame = frames[-1]
    number = _parking_number(context)

    detector = plate_ocr()
    ocr = detector.recognize(
        frame, mode=ocr_mode,
        detector_model=_detector_path(context, "plate"))
    context.check_deadline("车牌 OCR")

    lines = list(ocr.lines or [])
    raw_text = "".join(line.text for line in lines) if lines else str(ocr.text or "")
    plate = _clean_text(raw_text)
    # hyperlpr 的 text 形如 “宁R00136 蓝牌”，只取号码部分
    plate = re.sub(r"(蓝牌|黄牌单层|白牌单层|绿牌新能源|黑牌港澳|"
                   r"香港单层|香港双层|澳门单层|澳门双层|黄牌双层)$", "", plate)

    pattern = str(config["plate"].get("text_pattern") or "").strip()
    pattern_ok = None
    if pattern and plate:
        try:
            pattern_ok = bool(re.search(pattern, plate))
        except re.error:
            pattern_ok = None

    images: List[str] = []
    if ocr.annotated is not None:
        images.append(context.save_frame(
            ocr.annotated, tag="plate",
            caption="{}号停车场 车牌：{}".format(number, plate or "未识别"),
            extra_lines=["OCR：{}".format(ocr.message or "")]))
    raw = context.save_raw(frame)

    if not plate:
        return context.fail(
            "未识别到车牌（{}）".format(ocr.message or "OCR 无结果"),
            result="{}号停车场车牌未识别".format(number),
            extra={"parking": number, "ocr_message": ocr.message or "",
                   "raw_image": raw})
    text = "{}号停车场车牌号为 {}".format(number, plate)
    extra = {
        "parking": number,
        "plate": plate,
        "ocr_mode": ocr_mode,
        "ocr_confidence": round(float(lines[0].confidence), 3) if lines else None,
        "ocr_message": ocr.message or "",
    }
    if pattern_ok is not None:
        extra["pattern_ok"] = pattern_ok
    if raw:
        extra["raw_image"] = raw
    return context.ok(text, extra, images)
