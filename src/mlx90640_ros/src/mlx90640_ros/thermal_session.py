"""GUI-friendly thermal session: background capture, latest frame, floor table.

Designed for the ``photo_capture`` Tk GUI, but deliberately free of tkinter,
PIL and ROS: the GUI only calls :class:`ThermalSession`, gets a BGR frame plus
a structured hotspot, and renders it with its existing preview widget.

One process may own only one :class:`~mlx90640_ros.core.Decoder` at a time
(the vendor library keeps calibration in process globals), so do not create
two sessions in the same process.

Typical use::

    session = ThermalSession(port="/dev/serial/by-id/...").start()
    try:
        frame = session.latest()          # poll from a Tk ``after`` callback
        if frame is not None:
            show(frame.bgr)               # annotated colour image
            print(frame.summary())        # peak/row/col/box/floor
    finally:
        session.stop()

Offline replay for GUI development::

    ThermalSession(capture="/path/capture.bin", loop=True).start()
"""

import json
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time

import numpy as np

from . import core
from .hotspot import find_hotspot, floor_for_row, parse_floor_bands
from .transport import ReplayTransport, SerialTransport


@dataclass(frozen=True)
class ThermalFrame:
    """One complete thermal image, already annotated for display."""

    temps: np.ndarray
    bgr: np.ndarray
    hotspot: object
    floor: str
    ambient: float
    monotonic: float

    def summary(self):
        """JSON-safe dict; same shape as the ROS ``hotspot`` topic."""
        return {
            "peak_c": round(float(self.hotspot.peak_c), 3),
            "row": round(float(self.hotspot.row), 3),
            "col": round(float(self.hotspot.col), 3),
            "row_index": int(self.hotspot.row_index),
            "col_index": int(self.hotspot.col_index),
            "box": [int(value) for value in self.hotspot.box],
            "area": int(self.hotspot.area),
            "mean_c": round(float(self.hotspot.mean_c), 3),
            "floor": str(self.floor or ""),
            "ambient": round(float(self.ambient), 3),
        }


class FloorTable:
    """Mutable row-to-floor table for on-site calibration.

    ``record()`` stores a floor's exact centre row and rebuilds contiguous
    bands from 0 to 23 by splitting the gap between neighbouring centres.
    Recording 1F=20, 2F=12, 3F=4 therefore yields
    ``3F:0-7,2F:8-15,1F:16-23``.  Centres are kept in float, so repeated
    recording does not drift the way deriving them from band midpoints would.
    """

    def __init__(self, bands=(), centres=None):
        self._centres = {str(name): float(row)
                         for name, row in (centres or {}).items()}
        if not self._centres:
            self._centres = {name: (first + last) / 2.0
                             for name, first, last in bands}
        self._bands = tuple(bands) if bands else self._rebuild()

    @classmethod
    def from_text(cls, text):
        return cls(parse_floor_bands(text))

    def copy(self):
        return FloorTable(self._bands, self._centres)

    def __iter__(self):
        return iter(self._bands)

    @property
    def bands(self):
        return self._bands

    def to_text(self):
        return ",".join("{}:{}-{}".format(name, first, last)
                        for name, first, last in self._bands)

    def floor_for(self, row):
        return floor_for_row(row, self._bands)

    def _rebuild(self):
        ordered = sorted(self._centres.items(), key=lambda item: item[1])
        bands = []
        for index, (floor, centre) in enumerate(ordered):
            first = 0 if index == 0 else int(round(
                (ordered[index - 1][1] + centre) / 2.0))
            last = 23 if index == len(ordered) - 1 else int(round(
                (centre + ordered[index + 1][1]) / 2.0)) - 1
            bands.append((floor, max(0, first), min(23, last)))
        return tuple(bands)

    def record(self, name, row):
        """Record or replace one floor's centre row, then rebuild the bands."""
        label = str(name).strip()
        if not label:
            raise ValueError("floor name must not be empty")
        value = int(round(float(row)))
        if not 0 <= value <= 23:
            raise ValueError("row must be within 0..23")
        self._centres[label] = float(value)
        self._bands = self._rebuild()
        return self


class ThermalSession:
    """Own one MLX90640 stream in a background thread.

    ``port`` runs the real sensor; ``capture`` replays a recorded capture.bin
    and is the supported way to develop the GUI without hardware.
    """

    def __init__(self, port=None, capture=None, baud=460800, scale=12,
                 tmin=None, tmax=None, delta_c=2.0, roi=None,
                 library_path=None, floor_table=None, replay_rate=2.0,
                 loop=True):
        port = str(port or "").strip()
        capture = str(capture or "").strip()
        if bool(port) == bool(capture):
            raise ValueError("set exactly one of port or capture")
        if not isinstance(scale, int) or scale < 1:
            raise ValueError("scale must be a positive integer")
        for value in (tmin, tmax):
            if value is not None and not math.isfinite(float(value)):
                raise ValueError("display limits must be finite")
        if tmin is not None and tmax is not None and float(tmin) >= float(tmax):
            raise ValueError("tmin must be less than tmax")
        delta = float(delta_c)
        if not math.isfinite(delta) or delta < 0:
            raise ValueError("delta_c must be finite and >= 0")
        self.port = port
        self.capture = capture
        self.baud = int(baud)
        self.scale = scale
        self.tmin = None if tmin is None else float(tmin)
        self.tmax = None if tmax is None else float(tmax)
        self.delta_c = delta
        self.roi = roi
        self.replay_rate = replay_rate
        self.loop = bool(loop)
        self._table = (floor_table if isinstance(floor_table, FloorTable)
                       else FloorTable.from_text(floor_table or ""))
        self._decoder = core.Decoder(library_path=library_path,
                                     emissivity=0.95, tr_shift=8.0,
                                     bad_pixels=False)
        self._lock = threading.Lock()
        self._latest = None
        self._error = None
        self._thread = None
        self._transport = None
        self._stop = threading.Event()

    # -- calibration -----------------------------------------------------
    @property
    def floor_table(self):
        with self._lock:
            return self._table

    def set_floor_table(self, table):
        """Replace the table from a FloorTable or a ``1F:16-23,...`` string."""
        new = table if isinstance(table, FloorTable) else FloorTable.from_text(
            table or "")
        with self._lock:
            self._table = new
        return new

    def record_floor(self, name, row=None):
        """Record a floor using the latest frame's peak row (or an explicit row)."""
        if row is None:
            frame = self.latest()
            if frame is None:
                raise RuntimeError("no frame received yet")
            row = frame.hotspot.row
        with self._lock:
            self._table = self._table.copy().record(name, row)
            return self._table

    # -- data path -------------------------------------------------------
    def process(self, sample):
        """Turn one Sample into a ThermalFrame; no I/O, safe to call directly."""
        hotspot = find_hotspot(sample.temps, delta_c=self.delta_c, roi=self.roi)
        with self._lock:
            floor = self._table.floor_for(hotspot.row)
        color = core.render_image(sample, scale=self.scale, tmin=self.tmin,
                                  tmax=self.tmax, hotspot=hotspot, floor=floor)
        return ThermalFrame(temps=np.array(sample.temps, copy=True), bgr=color,
                            hotspot=hotspot, floor=floor,
                            ambient=float(sample.ambient),
                            monotonic=time.monotonic())

    def _run(self):
        transport = None
        try:
            if self.capture:
                transport = ReplayTransport(self.capture, self._decoder,
                                            replay_rate=self.replay_rate,
                                            loop=self.loop,
                                            stop=self._stop.is_set)
            else:
                transport = SerialTransport(self.port, self._decoder,
                                            baud=self.baud,
                                            stop=self._stop.is_set)
            with self._lock:
                self._transport = transport
            for sample in transport.samples():
                if self._stop.is_set():
                    break
                frame = self.process(sample)
                with self._lock:
                    self._latest = frame
        except Exception as exc:  # surfaced through error(), never raised in Tk
            with self._lock:
                self._error = exc
        finally:
            transport = self._transport
            if transport is not None:
                transport.close()

    def start(self):
        if self._thread is not None:
            raise RuntimeError("session already started")
        self._thread = threading.Thread(target=self._run, name="thermal-session",
                                        daemon=True)
        self._thread.start()
        return self

    def latest(self):
        with self._lock:
            return self._latest

    def error(self):
        with self._lock:
            return self._error

    def stop(self, timeout=2.0):
        self._stop.set()
        transport = self._transport
        if transport is not None:
            transport.close()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None

    def save_snapshot(self, directory=None, prefix="thermal", temps=False):
        """Write ``<prefix>_<timestamp>.png`` plus a ``.json`` sidecar.

        Returns ``(png_path, json_path)``; raises if no frame or write failed.
        """
        import cv2

        frame = self.latest()
        if frame is None:
            raise RuntimeError("no frame received yet")
        folder = Path(directory or "~/Pictures/mlx90640").expanduser()
        folder.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S") + "_{:03d}".format(
            int((time.time() % 1) * 1000))
        png = folder / "{}_{}.png".format(prefix, stamp)
        json_path = folder / "{}_{}.json".format(prefix, stamp)
        if not cv2.imwrite(str(png), frame.bgr):
            raise IOError("failed to write {}".format(png))
        payload = frame.summary()
        if temps:
            payload["temps"] = np.round(frame.temps, 3).tolist()
        json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                             encoding="utf-8")
        return png, json_path
