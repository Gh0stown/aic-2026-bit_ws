#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OCR 引擎封装：PaddleOCR 中文文字 + HyperLPR3 车牌。

中文模式可以先用任意 Ultralytics YOLO 模型裁剪区域，再对裁剪图做 PaddleOCR；
车牌模式支持 HyperLPR3，也支持“沙盘/非标车牌”的通用 OCR 读取。
两个引擎都懒加载，GUI 必须把识别放到工作线程。
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np


class OcrError(RuntimeError):
    pass


@dataclass
class OcrLine:
    text: str
    confidence: float
    points: List[List[int]] = field(default_factory=list)


@dataclass
class OcrResult:
    ok: bool
    kind: str
    text: str = ""
    lines: List[OcrLine] = field(default_factory=list)
    annotated: Optional[np.ndarray] = None
    message: str = ""


PLATE_TYPE_NAMES = {
    0: "蓝牌",
    1: "黄牌单层",
    2: "白牌单层",
    3: "绿牌新能源",
    4: "黑牌港澳",
    5: "香港单层",
    6: "香港双层",
    7: "澳门单层",
    8: "澳门双层",
    9: "黄牌双层",
}


def _ensure_bgr(image):
    if image is None:
        raise OcrError("没有可识别的画面")
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


def _offset_lines(lines: List[OcrLine], dx: int, dy: int) -> List[OcrLine]:
    shifted = []
    for line in lines:
        shifted.append(OcrLine(
            text=line.text,
            confidence=line.confidence,
            points=[[x + dx, y + dy] for x, y in line.points],
        ))
    return shifted


def _text_of(lines: List[OcrLine]) -> str:
    return "\n".join(
        "#{} {} ({:.2f})".format(i, line.text, line.confidence)
        for i, line in enumerate(lines, 1))


def _polygon_size(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return max(1, max(xs) - min(xs)), max(1, max(ys) - min(ys))


class ChineseOcr:
    """PaddleOCR 中文文字；可选 YOLO 模型先裁剪候选区域。"""

    def __init__(self):
        self._ocr = None
        self._detectors = {}

    def _engine(self):
        if self._ocr is None:
            try:
                from paddleocr import PaddleOCR
            except Exception as error:
                raise OcrError(
                    "PaddleOCR 未安装或导入失败：{}".format(error))
            try:
                self._ocr = PaddleOCR(
                    use_angle_cls=True, lang="ch", show_log=False,
                    use_gpu=False)
            except Exception as error:
                raise OcrError("PaddleOCR 初始化失败：{}".format(error))
        return self._ocr

    def _run_paddle(self, image) -> List[OcrLine]:
        engine = self._engine()
        try:
            raw = engine.ocr(image, cls=True)
        except Exception as error:
            raise OcrError("中文 OCR 失败：{}".format(error))
        lines = []
        first = raw[0] if raw else None
        for item in first or []:
            try:
                points = [[int(round(x)), int(round(y))]
                          for x, y in item[0]]
                text, score = item[1]
                lines.append(OcrLine(str(text), float(score), points))
            except Exception:
                continue
        return lines

    def _load_detector(self, model_path: str):
        key = str(Path(model_path).expanduser())
        model = self._detectors.get(key)
        if model is None:
            try:
                from ultralytics import YOLO
            except Exception as error:
                raise OcrError(
                    "ultralytics 不可用，无法加载裁剪模型：{}".format(error))
            try:
                model = YOLO(key)
            except Exception as error:
                raise OcrError("裁剪模型加载失败：{}".format(error))
            self._detectors[key] = model
        return model

    def _detect_boxes(self, image, model_path: str,
                      conf: float = 0.25, imgsz: int = 640):
        model = self._load_detector(model_path)
        try:
            results = model.predict(
                source=image, imgsz=imgsz, conf=conf,
                device="cpu", verbose=False)
        except Exception as error:
            raise OcrError("裁剪模型推理失败：{}".format(error))
        boxes = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes.xyxy:
                try:
                    x1, y1, x2, y2 = [int(round(float(v)))
                                      for v in box[:4]]
                except Exception:
                    continue
                if x2 > x1 and y2 > y1:
                    boxes.append((x1, y1, x2, y2))
        boxes.sort(key=lambda b: (b[2] - b[0]) * (b[3] - b[1]), reverse=True)
        return boxes[:8]

    @staticmethod
    def _crop(image, box):
        height, width = image.shape[:2]
        x1, y1, x2, y2 = box
        pad_x = max(2, int((x2 - x1) * 0.06))
        pad_y = max(2, int((y2 - y1) * 0.06))
        x1 = max(0, x1 - pad_x)
        y1 = max(0, y1 - pad_y)
        x2 = min(width, x2 + pad_x)
        y2 = min(height, y2 + pad_y)
        if x2 - x1 < 8 or y2 - y1 < 8:
            return None, (x1, y1, x2, y2)
        return image[y1:y2, x1:x2].copy(), (x1, y1, x2, y2)

    @staticmethod
    def _annotate(image, lines, crop_boxes=None, color=(255, 200, 0)):
        annotated = image.copy()
        for (x1, y1, x2, y2) in crop_boxes or []:
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 200, 255), 2)
        for index, line in enumerate(lines, 1):
            if not line.points:
                continue
            pts = np.asarray(line.points, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(annotated, [pts], True, color, 2)
            x = min(p[0] for p in line.points)
            y = max(8, min(p[1] for p in line.points) - 4)
            cv2.putText(annotated, "#{}".format(index), (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        return annotated

    def recognize(self, image, detector_model: str = "",
                  detector_conf: float = 0.25,
                  detector_imgsz: int = 640) -> OcrResult:
        image = _ensure_bgr(image)
        detector_model = str(detector_model or "").strip()
        if detector_model:
            boxes = self._detect_boxes(
                image, detector_model, detector_conf, detector_imgsz)
            if boxes:
                lines = []
                used_boxes = []
                for box in boxes:
                    crop, clipped = self._crop(image, box)
                    used_boxes.append(clipped)
                    if crop is None:
                        continue
                    local = self._run_paddle(crop)
                    lines.extend(_offset_lines(local, clipped[0], clipped[1]))
                if lines:
                    annotated = self._annotate(image, lines, used_boxes)
                    return OcrResult(
                        ok=True, kind="chinese", text=_text_of(lines),
                        lines=lines, annotated=annotated,
                        message="YOLO 裁剪 {} 个区域".format(len(used_boxes)))
                # 有轮廓但裁出来没文字：回退整图，避免空手而归
                lines = self._run_paddle(image)
                annotated = self._annotate(
                    image, lines, used_boxes) if lines else image.copy()
                return OcrResult(
                    ok=bool(lines), kind="chinese", text=_text_of(lines),
                    lines=lines, annotated=annotated,
                    message="裁剪区域未识别到文字，已回退全图" if lines
                    else "裁剪区域未识别到文字")
            lines = self._run_paddle(image)
            annotated = self._annotate(image, lines) if lines else image.copy()
            return OcrResult(
                ok=bool(lines), kind="chinese", text=_text_of(lines),
                lines=lines, annotated=annotated,
                message="检测模型未找到区域，已回退全图" if lines
                else "未识别到文字")

        lines = self._run_paddle(image)
        annotated = self._annotate(image, lines) if lines else image.copy()
        return OcrResult(
            ok=bool(lines), kind="chinese", text=_text_of(lines),
            lines=lines, annotated=annotated,
            message="" if lines else "未识别到文字")


class PlateOcr:
    """HyperLPR3 车牌；沙盘/非标车牌可用通用 OCR 替代省份识别。"""

    def __init__(self):
        self._catcher = None
        self._ocr = ChineseOcr()

    def _engine(self):
        if self._catcher is None:
            try:
                from hyperlpr3 import LicensePlateCatcher
            except Exception as error:
                raise OcrError(
                    "HyperLPR3 未安装或导入失败：{}".format(error))
            try:
                self._catcher = LicensePlateCatcher()
            except Exception as error:
                raise OcrError("HyperLPR3 初始化失败：{}".format(error))
        return self._catcher

    @staticmethod
    def _plate_like(line: OcrLine) -> bool:
        if not (5 <= len(line.text) <= 12):
            return False
        if not line.points:
            return False
        width, height = _polygon_size(line.points)
        return 1.8 <= width / float(height) <= 8.0

    @staticmethod
    def _pad_crop(crop, ratio=0.35):
        height, width = crop.shape[:2]
        pad_x = max(8, int(width * ratio))
        pad_y = max(6, int(height * ratio))
        padded = cv2.copyMakeBorder(
            crop, pad_y, pad_y, pad_x, pad_x, cv2.BORDER_CONSTANT,
            value=(80, 80, 80))
        return padded, pad_x, pad_y

    def _generic_full(self, image) -> OcrResult:
        annotated = image.copy()
        # HyperLPR3 只用来找车牌框；文字由通用 PaddleOCR 读，避免省份字典限制
        try:
            detections = self._engine()(image) or []
        except Exception:
            detections = []
        lines = []
        used_boxes = []
        for item in detections:
            try:
                x1, y1, x2, y2 = [int(round(float(v))) for v in item[3]]
            except Exception:
                continue
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(image.shape[1], x2), min(image.shape[0], y2)
            if x2 - x1 < 8 or y2 - y1 < 4:
                continue
            crop = image[y1:y2, x1:x2]
            local = self._ocr._run_paddle(crop)
            lines.extend(_offset_lines(local, x1, y1))
            used_boxes.append((x1, y1, x2, y2))
        if lines:
            annotated = self._ocr._annotate(
                image, lines, used_boxes, color=(0, 128, 255))
        else:
            # 没有车牌框或框内没文字时，整图 OCR 后按车牌长宽比过滤
            all_lines = self._ocr._run_paddle(image)
            filtered = [line for line in all_lines if self._plate_like(line)]
            lines = filtered or all_lines
            annotated = self._ocr._annotate(
                image, lines, color=(0, 128, 255)) if lines else image.copy()
        return OcrResult(
            ok=bool(lines), kind="plate", text=_text_of(lines),
            lines=lines, annotated=annotated,
            message="" if lines else "未识别到车牌文字")

    def _generic(self, image, detector_model="") -> OcrResult:
        detector_model = str(detector_model or "").strip()
        if detector_model:
            boxes = self._ocr._detect_boxes(image, detector_model)
            if boxes:
                lines = []
                used_boxes = []
                for box in boxes:
                    crop, clipped = self._ocr._crop(image, box)
                    used_boxes.append(clipped)
                    if crop is None:
                        continue
                    local = self._ocr._run_paddle(crop)
                    lines.extend(_offset_lines(local, clipped[0], clipped[1]))
                if lines:
                    annotated = self._ocr._annotate(
                        image, lines, used_boxes, color=(0, 128, 255))
                    return OcrResult(
                        ok=True, kind="plate", text=_text_of(lines),
                        lines=lines, annotated=annotated,
                        message="车牌裁剪 {} 个区域".format(len(used_boxes)))
                # 裁剪框内没有文字，继续回退整图检测
        return self._generic_full(image)

    def _hyperlpr_crop(self, image, detector_model="") -> Optional[OcrResult]:
        detector_model = str(detector_model or "").strip()
        if not detector_model:
            return None
        boxes = self._ocr._detect_boxes(image, detector_model)
        if not boxes:
            return None
        lines = []
        used_boxes = []
        for box in boxes:
            crop, clipped = self._ocr._crop(image, box)
            used_boxes.append(clipped)
            if crop is None:
                continue
            # HyperLPR3 对纯车牌图容易漏检；外扩一块背景再识别
            padded, pad_x, pad_y = self._pad_crop(crop)
            try:
                raw = self._engine()(padded) or []
            except Exception:
                raw = []
            for item in raw:
                try:
                    code = str(item[0])
                    score = float(item[1])
                    plate_type = int(item[2])
                    px1, py1, px2, py2 = [int(round(v)) for v in item[3]]
                except Exception:
                    continue
                x1 = clipped[0] + px1 - pad_x
                y1 = clipped[1] + py1 - pad_y
                x2 = clipped[0] + px2 - pad_x
                y2 = clipped[1] + py2 - pad_y
                type_name = PLATE_TYPE_NAMES.get(plate_type, str(plate_type))
                lines.append(OcrLine(
                    text="{} {}".format(code, type_name),
                    confidence=score,
                    points=[[x1, y1], [x2, y1], [x2, y2], [x1, y2]]))
        if not lines:
            return None
        annotated = self._ocr._annotate(
            image, lines, used_boxes, color=(0, 128, 255))
        return OcrResult(
            ok=True, kind="plate", text=_text_of(lines),
            lines=lines, annotated=annotated,
            message="HyperLPR3 裁剪加背景")

    def _hyperlpr(self, image, crop_boxes=None) -> OcrResult:
        catcher = self._engine()
        try:
            raw = catcher(image)
        except Exception as error:
            raise OcrError("车牌识别失败：{}".format(error))
        lines = []
        for item in raw or []:
            try:
                code = str(item[0])
                score = float(item[1])
                plate_type = int(item[2])
                box = [int(round(v)) for v in item[3]]
            except Exception:
                continue
            type_name = PLATE_TYPE_NAMES.get(plate_type, str(plate_type))
            x1, y1, x2, y2 = box
            lines.append(OcrLine(
                text="{} {}".format(code, type_name),
                confidence=score,
                points=[[x1, y1], [x2, y1], [x2, y2], [x1, y2]]))
        annotated = self._ocr._annotate(
            image, lines, crop_boxes, color=(0, 128, 255)) if lines else image.copy()
        return OcrResult(
            ok=bool(lines), kind="plate", text=_text_of(lines),
            lines=lines, annotated=annotated,
            message="" if lines else "未识别到车牌")

    def recognize(self, image, mode: str = "generic",
                  detector_model: str = "") -> OcrResult:
        image = _ensure_bgr(image)
        detector_model = str(detector_model or "").strip()
        if str(mode or "").lower() == "hyperlpr":
            if detector_model:
                result = self._hyperlpr_crop(image, detector_model)
                if result is not None:
                    return result
            return self._hyperlpr(image)
        return self._generic(image, detector_model)
