#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""task=crowd / sign / light 的检测类识别。

- crowd：`people.pt` 三类 people(社区人员) / bad(非社区人员) / cycle(违停电动车)；
  节点只返回当前视角的计数（多帧取每类最大值，不做多视角相加，汇总由导航做）。
- sign：`Signal.pt` 两类 A/B。A=正面→left，B=侧面→straight（暂定，需实测），
  多帧投票，票数不足或缺帧判失败，绝不猜一个方向。
- light：`light.pt` 红/黄/绿，多帧投票；黄灯按等待处理。红绿灯不上报裁判。
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from iv_common import Detection, TaskResult, VisionContext, VisionError

LIGHT_COLORS = {
    "红灯": ("red", "前方红灯，停车等待"),
    "黄灯": ("yellow", "前方黄灯，等待"),
    "绿灯": ("green", "前方绿灯，通行"),
}


# ---------------------------------------------------------------------------
# 公共
# ---------------------------------------------------------------------------

def _block_label(context: VisionContext) -> str:
    text = str(context.sub_id or "")
    for word in ("A街区", "A 街", "B街区", "B 街"):
        if word in text:
            return "A" if "A" in word else "B"
    return text or "该"


CROWD_ROLES = {
    "people": "people", "社区人员": "people", "person": "people",
    "bad": "bad", "非社区人员": "bad",
    "cycle": "cycle", "电动车": "cycle", "非机动车": "cycle",
}


def _crowd_role(name: str) -> Optional[str]:
    return CROWD_ROLES.get(str(name or "").strip())


def _frame_vote_class(detections: List[Detection]) -> Optional[Tuple[str, float]]:
    """一帧内取置信度最高的类别，返回 (类别名, 置信度)。"""
    if not detections:
        return None
    best = max(detections, key=lambda det: det.conf)
    return best.name, float(best.conf)


# ---------------------------------------------------------------------------
# task=light
# ---------------------------------------------------------------------------

def handle_light(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    conf = context.threshold("light", 0.45)
    frames = context.grab()
    model = context.model(context.model_path("light"), imgsz=640, conf=conf)
    votes: Counter = Counter()
    confidences: Dict[str, float] = {}
    best_detection: Dict[str, Detection] = {}
    frame_for: Dict[str, Any] = {}
    last_frame = None
    for frame in frames:
        context.check_deadline("红绿灯检测")
        detections = model.predict(frame, conf=conf)
        last_frame = frame
        vote = _frame_vote_class(detections)
        if vote is None:
            continue
        name, score = vote
        votes[name] += 1
        if score > confidences.get(name, 0.0):
            same = [det for det in detections if det.name == name]
            if same:
                best_detection[name] = max(same, key=lambda det: det.conf)
                frame_for[name] = frame
        confidences[name] = max(confidences.get(name, 0.0), score)
        context.check_deadline("红绿灯检测")
    if not votes:
        return context.fail("连续 %d 帧没有检测到红绿灯" % len(frames),
                            result="未识别到红绿灯")
    color_name, count = votes.most_common(1)[0]
    need = max(2, (len(frames) + 1) // 2)
    if count < need:
        return context.fail(
            "灯色不一致（%s），票数 %d/%d，不足 %d 票"
            % (dict(votes), count, len(frames), need),
            result="红绿灯识别不稳定",
            extra={"votes": dict(votes), "frames": len(frames)})
    color, sentence = LIGHT_COLORS.get(color_name, ("unknown", "未识别到红绿灯"))
    det = best_detection.get(color_name)
    image = None
    save_frame = frame_for.get(color_name, last_frame)
    if save_frame is not None:
        image = context.save_frame(
            save_frame, tag="light",
            detections=([det] if det is not None else []),
            caption="%s（%d/%d 票）" % (sentence, count, len(frames)))
    # handoff 规定 result 就是“红/黄/绿”短词，导航用 is_green() 判断
    extra = {"color": color, "text": sentence, "votes": dict(votes),
             "frames": len(frames), "required_votes": need,
             "confidence": round(confidences.get(color_name, 0.0), 3)}
    return context.ok(color_name, extra, [image] if image else [])


# ---------------------------------------------------------------------------
# task=sign
# ---------------------------------------------------------------------------

def handle_sign(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    config = context.config.get("sign", {})
    conf = context.threshold("sign", 0.60)
    route_map = {str(key).upper(): str(value)
                 for key, value in (config.get("route_map") or {}).items()}
    frames = context.grab()
    model = context.model(context.model_path("sign"), imgsz=640, conf=conf)
    votes: Counter = Counter()
    confidences: Dict[str, float] = {}
    per_frame: List[Dict[str, Any]] = []
    best_detections: Dict[str, Detection] = {}
    frame_for: Dict[str, Any] = {}
    last_frame = None
    for frame in frames:
        context.check_deadline("指示牌检测")
        detections = model.predict(frame, conf=conf)
        last_frame = frame
        vote = _frame_vote_class(detections)
        if vote is None:
            per_frame.append({"side": None})
            continue
        side, score = vote
        votes[side] += 1
        if score > confidences.get(side, 0.0):
            same = [det for det in detections if det.name == side]
            if same:
                best_detections[side] = max(same, key=lambda det: det.conf)
                frame_for[side] = frame
        confidences[side] = max(confidences.get(side, 0.0), score)
        per_frame.append({"side": side, "conf": round(score, 3)})

    confirm_detections: List[Detection] = []
    if bool(config.get("use_confirm_model", True)):
        confirm_key = context.model_path("sign_confirm")
        if confirm_key and last_frame is not None:
            try:
                confirm_model = context.model(confirm_key, imgsz=640, conf=conf)
                confirm_detections = confirm_model.predict(last_frame, conf=conf)
            except Exception as error:  # noqa: BLE001
                context.log("[vision] sign 复核模型失败：%s" % error)

    alt_used = False
    if not votes and bool(config.get("accept_alt_model", False)):
        alt_key = context.model_path("sign_alt")
        if alt_key:
            try:
                alt_model = context.model(alt_key, imgsz=640, conf=conf)
                alt_detections = alt_model.predict(last_frame, conf=conf) if last_frame is not None else []
                if alt_detections:
                    alt_used = True
                    best_detections["nogo"] = alt_detections[0]
            except Exception as error:  # noqa: BLE001
                context.log("[vision] sign 备用模型失败：%s" % error)

    if not votes and not alt_used:
        if confirm_detections:
            text = "前方是禁止直行指示牌"
            image = context.save_frame(
                last_frame, tag="sign", detections=confirm_detections,
                caption=text + "（stop.pt 复核到牌，A/B 未分出）")
            return context.ok(text, {"side": "unknown", "route": "unknown",
                                     "model": "sign_confirm",
                                     "confirm_conf": round(
                                         float(confirm_detections[0].conf), 3),
                                     "frames": len(frames)},
                              [image])
        return context.fail("连续 %d 帧没有检测到指示牌" % len(frames),
                            result="未识别到指示牌",
                            extra={"per_frame": per_frame, "frames": len(frames)})
    if alt_used:
        text = "前方是禁止直行指示牌"
        image = context.save_frame(last_frame, tag="sign",
                                   detections=[best_detections["nogo"]],
                                   caption=text + "（备用模型，方向未知）")
        return context.ok(text, {"side": "unknown", "route": "unknown",
                                 "model": "sign_alt", "frames": len(frames)},
                          [image])

    side, count = votes.most_common(1)[0]
    need = max(2, (len(frames) + 1) // 2)
    if count < need or len(votes) > 1 and votes[side] == votes.most_common(2)[1][1]:
        return context.fail(
            "指示牌识别不稳定（%s），票数 %d/%d" % (dict(votes), count, len(frames)),
            result="指示牌识别不稳定",
            extra={"votes": dict(votes), "per_frame": per_frame, "frames": len(frames)})
    texts = {
        "A": "前方是禁止直行指示牌",
        "B": "前方是禁止直行指示牌",
    }
    text = texts.get(side, "前方是禁止直行指示牌")
    route = route_map.get(side.upper(), "unknown")
    save_frame = frame_for.get(side, last_frame)
    image = context.save_frame(
        save_frame, tag="sign",
        detections=[best_detections[side]],
        caption="%s（%s 面，%d/%d 票，route=%s）" % (text, side, count, len(frames), route))
    raw = context.save_raw(save_frame)
    extra = {"side": side, "route": route, "votes": dict(votes),
             "frames": len(frames), "required_votes": need,
             "confidence": round(confidences.get(side, 0.0), 3),
             "confirm_hit": bool(confirm_detections),
             "confirm_conf": round(float(confirm_detections[0].conf), 3)
             if confirm_detections else None,
             "per_frame": per_frame}
    if raw:
        extra["raw_image"] = raw
    return context.ok(text, extra, [image])


# ---------------------------------------------------------------------------
# task=crowd
# ---------------------------------------------------------------------------

def handle_crowd(context: VisionContext, params: Dict[str, Any]) -> TaskResult:
    config = context.config.get("crowd", {})
    roles = [str(item) for item in (config.get("classes") or ["people", "bad"])]
    conf = context.threshold("crowd", 0.25)
    frames = context.grab()
    model = context.model(context.model_path("crowd"), imgsz=640, conf=conf)

    best_counts: Dict[str, int] = {name: 0 for name in roles}
    best_detections: List[Detection] = []
    best_frame = None
    best_conf_sum = -1.0
    last_frame = None
    per_frame: List[Dict[str, Any]] = []
    for frame in frames:
        context.check_deadline("人员计数")
        detections = model.predict(frame, conf=conf)
        last_frame = frame
        counts: Counter = Counter()
        kept: List[Detection] = []
        for det in detections:
            role = _crowd_role(det.name)
            if role is None:
                continue
            kept.append(det)
            counts[role] += 1
        detections = kept
        for name in roles:
            best_counts[name] = max(best_counts[name], int(counts.get(name, 0)))
        conf_sum = sum(det.conf for det in detections)
        if conf_sum > best_conf_sum:
            best_conf_sum = conf_sum
            best_detections = detections
            best_frame = frame
        per_frame.append({name: int(counts.get(name, 0)) for name in roles})

    person = int(best_counts.get("people", 0))
    bad = int(best_counts.get("bad", 0))
    cycle = int(best_counts.get("cycle", 0))
    block = _block_label(context)
    text = "{} 街区当前视角：社区人员 {} 人，非社区人员 {} 人".format(
        block, person, bad)
    save_frame = best_frame if best_frame is not None else last_frame
    image = context.save_frame(
        save_frame, tag="crowd", detections=best_detections, caption=text,
        extra_lines=["多帧取每类最大值，不做多视角相加"])
    raw = context.save_raw(save_frame) if save_frame is not None else None
    extra = {"block": block, "person": person, "bad": bad,
             "frames": len(frames), "per_frame": per_frame,
             "model": Path(model.path).name}
    if "cycle" in best_counts:
        extra["cycle"] = cycle
    if raw:
        extra["raw_image"] = raw
    return context.ok(text, extra, [image])
