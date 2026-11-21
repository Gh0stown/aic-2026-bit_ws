"""Exp16 test-mode backend: one YOLO prediction and two same-frame OpenCV reads."""

from collections.abc import Mapping
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

from gauge_exp16_vendor import UnreadableGauge
from gauge_exp16_vendor.read_gauge_rect import TEMP_MAX, TEMP_MIN, read_temperature
from gauge_exp16_vendor.read_humidity_ticks_ordered import read_humidity


BBox = Tuple[int, int, int, int]


@dataclass(frozen=True)
class GaugeReading:
    status: str  # ok, missing (no class detection), unreadable, error
    value: Optional[float] = None
    confidence: Optional[float] = None  # YOLO box confidence, not reading confidence.
    bbox: Optional[BBox] = None  # Inclusive xyxy coordinates in the source frame.
    detail: Optional[str] = None
    tick_count: Optional[int] = None
    method: Optional[str] = None

    def as_dict(self):
        return {
            "status": self.status,
            "value": self.value,
            "confidence": self.confidence,
            "bbox": list(self.bbox) if self.bbox is not None else None,
            "detail": self.detail,
            "tick_count": self.tick_count,
            "method": self.method,
        }


@dataclass(frozen=True)
class Exp16Result:
    annotated: np.ndarray
    temperature: GaugeReading
    humidity: GaugeReading

    def as_dict(self):
        """JSON-safe metadata; the annotated image stays separate."""
        return {"temperature": self.temperature.as_dict(), "humidity": self.humidity.as_dict()}


def _as_array(data):
    if hasattr(data, "cpu"):
        data = data.cpu()
    if hasattr(data, "numpy"):
        data = data.numpy()
    return np.asarray(data)


def _select_boxes(result, width, height, minimum_confidence):
    selected = {}
    if result is None or result.boxes is None:
        return selected
    boxes = result.boxes
    for xyxy, class_id, confidence in zip(
        _as_array(boxes.xyxy), _as_array(boxes.cls), _as_array(boxes.conf),
    ):
        name = str(result.names[int(class_id)]).lower()
        if name not in ("wendu", "shidu"):
            continue
        score = float(confidence)
        # The detector normally applies `conf`; tolerate a float32 ulp here.
        if not 0.0 <= score <= 1.0 or score + 1e-7 < minimum_confidence:
            continue
        if name in selected and selected[name][1] >= score:
            continue
        if not np.all(np.isfinite(xyxy)):
            continue
        # Match the offline detector's flooring, clipping and inclusive end.
        x1, y1, x2, y2 = (math.floor(value) for value in xyxy)
        bbox = max(0, x1), max(0, y1), min(width - 1, x2), min(height - 1, y2)
        if bbox[0] > bbox[2] or bbox[1] > bbox[3]:
            continue
        selected[name] = bbox, score
    return selected


def _read_crop(frame, detection, read, limits):
    if detection is None:
        return GaugeReading("missing"), None
    bbox, confidence = detection
    x1, y1, x2, y2 = bbox
    try:
        # The handoff thresholded decoded JPEG ROIs at native camera scale.
        # Reproduce the codec roundtrip without a temporary file or resize.
        ok, jpeg = cv2.imencode(".jpg", frame[y1 : y2 + 1, x1 : x2 + 1])
        if not ok:
            raise RuntimeError("could not encode gauge crop")
        crop = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
        if crop is None:
            raise RuntimeError("could not decode gauge crop")
        measurement = read(crop)
        value = float(measurement.value)
        if not limits[0] <= value <= limits[1]:
            raise UnreadableGauge("reading outside valid range")
    except UnreadableGauge as exc:
        return GaugeReading("unreadable", confidence=confidence, bbox=bbox, detail=str(exc)), None
    except Exception as exc:
        detail = "{}: {}".format(type(exc).__name__, exc)
        return GaugeReading("error", confidence=confidence, bbox=bbox, detail=detail), None
    return (
        GaugeReading("ok", round(value, 3), confidence, bbox,
                     tick_count=measurement.tick_count, method=measurement.method),
        measurement,
    )


def _annotate(output, label, unit, color, reading, measurement):
    if reading.bbox is None:
        return
    x1, y1, x2, y2 = reading.bbox
    if measurement is not None:
        # Draw on the crop view so the pointer cannot extend to the other dial.
        roi = output[y1 : y2 + 1, x1 : x2 + 1]
        cx, cy = measurement.center
        angle = math.radians(measurement.angle)
        radius = measurement.radius * (0.88 if label == "T" else 0.98)
        tip = (round(cx + radius * math.cos(angle)), round(cy - radius * math.sin(angle)))
        cv2.arrowedLine(roi, (cx, cy), tip, (0, 0, 255), 2, cv2.LINE_AA, tipLength=0.08)
    cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
    text = (
        "{} {:.1f}{}".format(label, reading.value, unit)
        if reading.status == "ok" else "{} {}".format(label, reading.status)
    )
    text += " ({:.2f})".format(reading.confidence)
    font = cv2.FONT_HERSHEY_SIMPLEX
    (text_width, text_height), _ = cv2.getTextSize(text, font, 0.54, 1)
    box_height = text_height + 9
    left = min(x1, max(0, output.shape[1] - text_width - 8))
    top = y1 - box_height if y1 >= box_height else y1
    cv2.rectangle(output, (left, top), (left + text_width + 8, top + box_height), (0, 0, 0), -1)
    cv2.putText(output, text, (left + 4, top + text_height + 3), font, 0.54, color, 1, cv2.LINE_AA)


class Exp16Reader:
    """Load the detector once; call process with uint8 BGR camera frames.

    `model` may be injected as a test double. Call from a single inference
    worker. The reader holds no frame history and never mutates its input.
    """

    def __init__(self, model_path, device="auto", imgsz=960, conf=0.05, *, model=None):
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.device = device
        if self.imgsz <= 0 or not 0.0 <= self.conf <= 1.0:
            raise ValueError("imgsz must be positive and conf must be in [0, 1]")
        if model is None:
            path = Path(model_path).expanduser()
            if not path.is_file():
                raise FileNotFoundError("Exp16 weights not found: {}".format(path))
            from ultralytics import YOLO

            # Let the checkpoint declare its task rather than forcing detection.
            model = YOLO(str(path))
            if model.task != "detect":
                raise ValueError("Exp16 requires a detection model; got task {!r}".format(model.task))
            names = model.names
            labels = names.values() if isinstance(names, Mapping) else (names or ())
            if not {"wendu", "shidu"}.issubset(str(name).lower() for name in labels):
                raise ValueError("Exp16 weights must contain both `wendu` and `shidu` classes")
        self.model = model

    def process(self, frame):
        """Return readings and a BGR overlay on a copy of this exact frame."""
        if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or
                frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0):
            raise ValueError("Exp16Reader.process expects a nonempty uint8 BGR image")
        output = frame.copy()
        try:
            kwargs = {"imgsz": self.imgsz, "conf": self.conf, "verbose": False}
            if self.device not in (None, "auto"):
                kwargs["device"] = self.device
            results = self.model.predict(frame, **kwargs)
            height, width = frame.shape[:2]
            selected = _select_boxes(results[0] if len(results) else None, width, height, self.conf)
        except Exception as exc:
            error = GaugeReading("error", detail="{}: {}".format(type(exc).__name__, exc))
            return Exp16Result(output, error, error)

        temperature, temp_geometry = _read_crop(frame, selected.get("wendu"), read_temperature, (TEMP_MIN, TEMP_MAX))
        humidity, humidity_geometry = _read_crop(frame, selected.get("shidu"), read_humidity, (0.0, 100.0))
        _annotate(output, "T", " C", (0, 255, 0), temperature, temp_geometry)
        _annotate(output, "RH", "%", (255, 255, 0), humidity, humidity_geometry)
        return Exp16Result(output, temperature, humidity)
