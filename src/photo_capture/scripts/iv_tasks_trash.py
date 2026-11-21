#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""task=trash（垃圾桶：类型 / 开闭 / 垃圾类别 / 投放对错）。

- `trash_brest.pt` 8 类 = {可回收, 厨余, 有害, 其他} × {打开, 关闭}，
  一个框同时给出桶类型与开闭状态；
- `catalog.pt` 36 类，中文类别名自带“可回收垃圾-塑料瓶”这样的前缀，
  前缀就是该垃圾应投放的桶类型；
- 对每个桶**单独裁一块区域再跑垃圾模型**（向上多留桶口），这样垃圾只可能
  属于它所在的桶，避免旧实现“按类型全局匹配”把垃圾分错桶的问题；
- 类别前缀与桶类型一致记“正确”，不一致记“错误”，没识别到垃圾记“未判定”。
"""

from __future__ import annotations

import re
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from iv_common import Detection, TaskResult, VisionContext

# 四分类规范名（用户 2026-11 确认口径：厨余垃圾 / 可回收物 / 其他垃圾 / 有害垃圾）
CAT_KITCHEN = "厨余"
CAT_RECYCLE = "可回收物"
CAT_OTHER = "其他"
CAT_HAZARD = "有害"
BIN_TYPES = (CAT_KITCHEN, CAT_RECYCLE, CAT_OTHER, CAT_HAZARD)
# 官方全称（播报/上报 extra 用）；result 里用短名 + “垃圾桶”，避免“厨余垃圾垃圾桶”
CAT_FULL = {
    CAT_KITCHEN: "厨余垃圾",
    CAT_RECYCLE: "可回收物",
    CAT_OTHER: "其他垃圾",
    CAT_HAZARD: "有害垃圾",
}
BIN_STATES = {"打开": "open", "关闭": "close"}
_BIN_RE = re.compile(r"(.+?)垃圾状态为(打开|关闭)")

# trash.pt（24 类，类名是纯数字 "1".."24"）的类别映射：
# 1-6 厨余垃圾，7-12 可回收物，13-18 其他垃圾，19-24 有害垃圾。
# 用户 2026-11 给出的 24 种垃圾与标签对应关系；缺它时数字类名无法播报。
TRASH24_MAP = {
    "1": ("厨余", "果核"),
    "2": ("厨余", "鱼骨"),
    "3": ("厨余", "菜叶"),
    "4": ("厨余", "面包"),
    "5": ("厨余", "茶渣"),
    "6": ("厨余", "剩饭"),
    "7": ("可回收物", "塑料瓶"),
    "8": ("可回收物", "报纸"),
    "9": ("可回收物", "玻璃瓶"),
    "10": ("可回收物", "易拉罐"),
    "11": ("可回收物", "旧衣物"),
    "12": ("可回收物", "纸箱"),
    "13": ("其他", "烟头"),
    "14": ("其他", "用过纸巾"),
    "15": ("其他", "破陶瓷"),
    "16": ("其他", "贝壳"),
    "17": ("其他", "用过的一次性筷子"),
    "18": ("其他", "废弃眼镜"),
    "19": ("有害", "纽扣电池"),
    "20": ("有害", "荧光灯"),
    "21": ("有害", "废胶卷"),
    "22": ("有害", "油漆"),
    "23": ("有害", "药片"),
    "24": ("有害", "水银温度计"),
}


def _normalize_type(text: str) -> str:
    """把各种写法归一成四分类规范名：厨余 / 可回收物 / 其他 / 有害。"""
    cleaned = str(text or "").replace("其它", "其他").replace("垃圾", "").strip()
    if "可回收" in cleaned:
        return CAT_RECYCLE
    if "厨余" in cleaned:
        return CAT_KITCHEN
    if "有害" in cleaned:
        return CAT_HAZARD
    if "其他" in cleaned:
        return CAT_OTHER
    return cleaned


def _parse_bin(detection: Detection) -> Optional[Tuple[str, str]]:
    match = _BIN_RE.search(detection.name)
    if not match:
        return None
    return _normalize_type(match.group(1)), match.group(2)


def _parse_item(detection: Detection,
                item_map: Optional[Dict[str, Any]] = None) -> Tuple[Optional[str], str]:
    """垃圾类别名 -> (应投放类型, 垃圾名)。

    - trash.pt：类名是 "1".."24"，查 TRASH24_MAP（config trash.item_map 可覆盖）；
    - catalog.pt：类名形如 “可回收垃圾-塑料瓶”，前缀即桶类型；
    无法解析时返回 (None, 原名)。
    """
    name = str(detection.name or "").strip()
    mapping = item_map or TRASH24_MAP
    entry = mapping.get(name)
    if entry is not None:
        if isinstance(entry, (list, tuple)) and len(entry) >= 2:
            return _normalize_type(entry[0]), str(entry[1])
        return None, str(entry)
    if "-" in name:
        prefix, item = name.split("-", 1)
    else:
        prefix, item = name, name
    category = _normalize_type(prefix)
    for bin_type in BIN_TYPES:
        if bin_type in category:
            return bin_type, item.strip()
    return None, item.strip() or name


def _expanded(detection: Detection, ratio: float = 0.15,
              top_ratio: float = 0.6) -> Tuple[float, float, float, float]:
    """桶框向外扩一圈；上方多扩，垃圾通常堆在开口处/桶沿以上。"""
    width = detection.width
    height = detection.height
    return (detection.x1 - width * ratio,
            detection.y1 - height * top_ratio,
            detection.x2 + width * ratio,
            detection.y2 + height * ratio)


def _assign_items(bins: List[Detection],
                  items: List[Detection]) -> Dict[int, List[Detection]]:
    """按空间位置把垃圾框分给桶：中心落在扩框内取面积最小者，否则取重叠最大者。"""
    assigned: Dict[int, List[Detection]] = {index: [] for index in range(len(bins))}
    expanded = [_expanded(bin_det) for bin_det in bins]
    for item in items:
        cx, cy = item.center
        candidates = []
        for index, (x1, y1, x2, y2) in enumerate(expanded):
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                candidates.append(((x2 - x1) * (y2 - y1), index))
        if candidates:
            candidates.sort()
            assigned[candidates[0][1]].append(item)
            continue
        best_index, best_ratio = -1, 0.0
        for index, bin_det in enumerate(bins):
            ratio = item.overlap_ratio(bin_det)
            if ratio > best_ratio:
                best_ratio, best_index = ratio, index
        if best_index >= 0 and best_ratio >= 0.2:
            assigned[best_index].append(item)
    return assigned


def _crop_bin(frame: np.ndarray, detection: Detection,
              ratio: float = 0.5, top_ratio: float = 1.2,
              bottom_ratio: float = 0.3):
    """裁出桶附近区域（比桶框宽松），用于全图漏检时的补检。"""
    height, width = frame.shape[:2]
    x1 = detection.x1 - detection.width * ratio
    y1 = detection.y1 - detection.height * top_ratio
    x2 = detection.x2 + detection.width * ratio
    y2 = detection.y2 + detection.height * bottom_ratio
    x1 = max(0, int(round(x1)))
    y1 = max(0, int(round(y1)))
    x2 = min(width, int(round(x2)))
    y2 = min(height, int(round(y2)))
    if x2 <= x1 or y2 <= y1:
        return None, (0, 0)
    return frame[y1:y2, x1:x2], (x1, y1)


def _shift(detection: Detection, dx: int, dy: int) -> Detection:
    return Detection(detection.x1 + dx, detection.y1 + dy,
                     detection.x2 + dx, detection.y2 + dy,
                     detection.conf, detection.cls_id, detection.name)


def _describe(item: Optional[Tuple[str, str]], bin_type: str) -> Tuple[str, Optional[bool]]:
    if item is None:
        return "未识别", None
    category, name = item
    if category is None:
        return name, None
    return name, category == bin_type


def handle(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    config = context.config.get("trash", {})
    item_map = config.get("item_map") or TRASH24_MAP
    bins_conf = context.threshold("trash_bins", 0.30)
    items_conf = context.threshold("trash_items", 0.20)
    frames = context.grab()
    frame = frames[-1]

    bins_model = context.model(context.model_path("trash_bins"), imgsz=640,
                               conf=bins_conf)
    items_model = context.model(context.model_path("trash_items"), imgsz=640,
                                conf=items_conf)
    bin_detections = bins_model.predict(frame, conf=bins_conf)
    # 先整图找垃圾：小目标在整图里有上下文，通常比裁图更稳
    full_items = items_model.predict(frame, conf=items_conf)
    context.check_deadline("垃圾桶检测")

    # 同一类型只保留置信度最高的一个框；按从左到右排序，方便播报/复盘
    by_type: Dict[str, Detection] = {}
    for detection in bin_detections:
        parsed = _parse_bin(detection)
        if parsed is None:
            continue
        bin_type, _state = parsed
        current = by_type.get(bin_type)
        if current is None or detection.conf > current.conf:
            by_type[bin_type] = detection
    bins = sorted(by_type.values(), key=lambda det: det.center[0])[:int(
        config.get("bins_per_frame", 4) or 4)]
    if not bins:
        return context.fail(
            "没有检测到垃圾桶（trash_bins 类：%s）" % ", ".join(
                sorted(set(bins_model.names.values()))[:8]),
            result="未识别到垃圾桶")

    assigned = _assign_items(bins, full_items)
    entries: List[Dict[str, Any]] = []
    sentences: List[str] = []
    draw_detections: List[Detection] = []
    draw_labels: List[str] = []
    for index, detection in enumerate(bins):
        parsed = _parse_bin(detection)
        assert parsed is not None
        bin_type, state = parsed
        state_code = BIN_STATES.get(state, state)

        items: List[Detection] = list(assigned.get(index) or [])
        best_so_far = max(items, key=lambda det: det.conf) if items else None
        recheck_below = float(config.get("crop_recheck_below", 0.5))
        if best_so_far is None or best_so_far.conf < recheck_below:
            # 整图漏检/低置信时，对这个桶的宽松裁图补检；小目标在裁图里更大更稳
            crop, offset = _crop_bin(frame, detection)
            if crop is not None and crop.size > 0:
                context.check_deadline("垃圾补检")
                raw_items = items_model.predict(crop, conf=items_conf)
                # 补检候选必须真的落在本桶框内（防止把邻桶的垃圾算进来）
                inside = []
                for raw in raw_items:
                    candidate = _shift(raw, offset[0], offset[1])
                    cx, cy = candidate.center
                    if (detection.x1 <= cx <= detection.x2
                            and detection.y1 - 0.2 * detection.height <= cy
                            <= detection.y2):
                        inside.append(candidate)
                if inside:
                    items.append(max(inside, key=lambda det: det.conf))
        items.sort(key=lambda det: det.conf, reverse=True)
        best_item = items[0] if items else None
        item = _parse_item(best_item, item_map) if best_item is not None else None
        item_name, correct = _describe(item, bin_type)

        entries.append({
            "type": bin_type,
            "category": CAT_FULL.get(bin_type, bin_type),
            "state": state_code,
            "state_text": state,
            "item": item_name,
            "items": [_parse_item(det, item_map)[1] for det in items],
            "correct": correct,
            "conf": round(float(detection.conf), 3),
            "item_conf": round(float(best_item.conf), 3) if best_item is not None else None,
            "box": [round(value, 1) for value in detection.box],
        })
        draw_detections.append(detection)
        draw_labels.append("{}垃圾桶-{}".format(bin_type, state))
        if best_item is not None:
            draw_detections.append(best_item)
            draw_labels.append(item_name)

        if correct is True:
            verdict = "正确"
        elif correct is False:
            verdict = "错误"
        else:
            verdict = "未判定"
        sentences.append("{}垃圾桶状态为{}，垃圾桶里的垃圾为{}，投放{}".format(
            bin_type, state, item_name, verdict))

    text = "；".join(sentences)
    expected = int(config.get("bins_per_frame", 4) or 4)
    extra_lines = ["{}垃圾桶 {}".format(entry["type"], entry["state_text"])
                   for entry in entries]
    if len(entries) < expected:
        extra_lines.append("只检出 {}/{} 个桶".format(len(entries), expected))
    image = context.save_frame(frame, tag="trash", detections=draw_detections,
                               labels=draw_labels, caption=text,
                               extra_lines=extra_lines)
    raw = context.save_raw(frame)
    extra: Dict[str, Any] = {
        "count": len(entries),
        "detected": len(entries),
        "expected": expected,
        "bins": entries,
        "frames": len(frames),
    }
    if raw:
        extra["raw_image"] = raw
    return context.ok(text, extra, [image])
