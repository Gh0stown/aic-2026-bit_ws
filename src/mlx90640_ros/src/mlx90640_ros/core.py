"""ROS-free MLX90640 decoding, capture replay and optional visualization.

The vendor wrapper owns process-global calibration (g_params/g_ready): use
one active Decoder per process, not concurrent independent sensor instances.
"""

import ctypes
from dataclasses import dataclass
import math
import operator
from pathlib import Path
import struct

import numpy as np

from .protocol import TYPE_EEPROM, TYPE_FRAME, TYPE_STATUS

EEPROM_BYTES = 832 * 2
FRAME_BYTES = 834 * 2
FRAME_PAYLOAD_BYTES = 6 + FRAME_BYTES
CAPTURE_MAGIC = 0x314C584D


@dataclass(frozen=True)
class Sample:
    temps: np.ndarray
    ambient: float
    device_ms: int
    subpage: int


class Decoder:
    """Join two fresh, different subpages in the same native float buffer."""

    def __init__(self, library_path=None, emissivity=.95, tr_shift=8,
                 bad_pixels=False):
        self.emissivity = float(emissivity)
        self.tr_shift = float(tr_shift)
        if not math.isfinite(self.emissivity) or not 0 < self.emissivity <= 1:
            raise ValueError("emissivity must be finite and in (0, 1]")
        if not math.isfinite(self.tr_shift):
            raise ValueError("tr_shift must be finite")
        self.bad_pixels = bool(bad_pixels)
        self._lib = ctypes.CDLL(library_path or "libmlx90640.so")
        u16p = ctypes.POINTER(ctypes.c_uint16)
        fp = ctypes.POINTER(ctypes.c_float)
        self._lib.mlx_init.argtypes = [u16p]
        self._lib.mlx_init.restype = ctypes.c_int
        self._lib.mlx_frame_to_temp.argtypes = [u16p, ctypes.c_float,
                                               ctypes.c_float, fp, fp]
        self._lib.mlx_frame_to_temp.restype = ctypes.c_int
        self._lib.mlx_bad_pixels.argtypes = [fp, ctypes.c_uint16]
        self._lib.mlx_bad_pixels.restype = None
        self._temps = np.full((24, 32), np.nan, dtype=np.float32)
        rows, cols = np.indices((24, 32))
        self._pixel_subpages = (rows & 1, (rows ^ cols) & 1)
        self._out = self._temps.ctypes.data_as(fp)
        self.reset()

    @property
    def ready(self):
        return self._ready

    def _clear_subpages(self):
        self._seen = set()
        self._mode = None
        self._temps.fill(np.nan)

    def reset(self):
        """Forget calibration and all temperatures; wait for another EEPROM.

        The C global is deliberately not touched: Python's readiness gate
        prevents its use until the next successful mlx_init.
        """
        self._ready = False
        self._last_ms = None
        self._clear_subpages()

    def load_eeprom(self, payload):
        self.reset()
        if len(payload) != EEPROM_BYTES:
            raise ValueError("EEPROM payload must contain exactly 1664 bytes")
        if payload == bytes(EEPROM_BYTES) or payload == b"\xff" * EEPROM_BYTES:
            raise ValueError("EEPROM calibration cannot be all zero or all 0xff")
        words = (ctypes.c_uint16 * 832)(*struct.unpack("<832H", payload))
        result = self._lib.mlx_init(words)
        if result != 0:
            raise ValueError("EEPROM calibration failed (mlx_init={})".format(result))
        self._ready = True

    def process_frame(self, payload):
        if len(payload) != FRAME_PAYLOAD_BYTES:
            self._clear_subpages()
            raise ValueError("frame payload must contain exactly 1674 bytes")
        device_ms, subpage = struct.unpack_from("<IB", payload)
        words = (ctypes.c_uint16 * 834)(*struct.unpack_from("<834H", payload, 6))
        if subpage not in (0, 1) or words[833] != subpage:
            self._clear_subpages()
            raise ValueError("frame subpage must be 0 or 1 and match words[833]")
        if not self._ready:
            return None
        if self._last_ms is not None:
            delta = (device_ms - self._last_ms) & 0xFFFFFFFF
            if delta >= 0x80000000:
                # Serial-number arithmetic accepts natural UINT32 wrap, not
                # an ESP32 reboot/time reversal. Calibration must be resent.
                self.reset()
                return None
            if delta == 0:
                return None
            if delta > 3000:
                self._clear_subpages()
        self._last_ms = device_ms
        mode = (words[832] >> 12) & 1
        if self._mode is not None and self._mode != mode:
            self._clear_subpages()
        self._mode = mode
        ambient = ctypes.c_float()
        result = self._lib.mlx_frame_to_temp(words, self.emissivity, self.tr_shift,
                                              self._out, ctypes.byref(ambient))
        if result != 0:
            self._clear_subpages()
            raise ValueError("temperature conversion failed (mlx_frame_to_temp={})".format(result))
        if not math.isfinite(ambient.value):
            self._clear_subpages()
            raise ValueError("non-finite ambient temperature")
        # The unreceived half is intentionally NaN. Without correction, reject
        # an invalid received half immediately. With correction enabled, allow
        # the vendor's known bad pixels to be repaired after BOTH halves arrive;
        # all remaining non-finite values are still rejected before publishing.
        selected = self._pixel_subpages[mode] == subpage
        if not self.bad_pixels and not np.isfinite(self._temps[selected]).all():
            self._clear_subpages()
            raise ValueError("non-finite pixel temperature")
        self._seen.add(subpage)
        if len(self._seen) != 2:
            return None
        if self.bad_pixels:
            self._lib.mlx_bad_pixels(self._out, words[832])
        if not np.isfinite(self._temps).all():
            self._clear_subpages()
            raise ValueError("non-finite pixel temperature")
        sample = Sample(self._temps.copy(), float(ambient.value), device_ms, subpage)
        # Every output consumes BOTH subpages. Never reuse the other half of
        # the preceding image, and never invalidate a previously returned copy.
        self._clear_subpages()
        return sample


def load_capture(path):
    """Read ONLY the documented capture format: <III + 832H + N*834H.

    Return EEPROM bytes and a list of raw 1668-byte frames (no UART headers).
    """
    data = Path(path).read_bytes()
    if len(data) < 12:
        raise ValueError("capture header is truncated")
    magic, ee_words, count = struct.unpack_from("<III", data)
    if magic != CAPTURE_MAGIC:
        raise ValueError("invalid capture magic (expected 0x314c584d)")
    if ee_words != 832:
        raise ValueError("capture EEPROM word count must be 832")
    if count == 0:
        raise ValueError("capture must contain at least one frame")
    expected = 12 + EEPROM_BYTES + count * FRAME_BYTES
    if len(data) != expected:
        raise ValueError("capture length does not match its declared frame count")
    frames_start = 12 + EEPROM_BYTES
    return (data[12:frames_start],
            [data[i:i + FRAME_BYTES] for i in range(frames_start, expected, FRAME_BYTES)])


def capture_payload(words, index, interval_ms=500):
    """Wrap one raw 834-word little-endian frame as an ESP32 FRAME payload."""
    if len(words) != FRAME_BYTES:
        raise ValueError("capture frame must contain exactly 1668 bytes")
    try:
        index = operator.index(index)
        interval_ms = operator.index(interval_ms)
    except TypeError as exc:
        raise ValueError("replay index and interval_ms must be integers") from exc
    if index < 0 or interval_ms <= 0:
        raise ValueError("replay index must be nonnegative and interval_ms positive")
    subpage = struct.unpack_from("<H", words, FRAME_BYTES - 2)[0]
    if subpage not in (0, 1):
        raise ValueError("capture words[833] must be 0 or 1")
    return struct.pack("<IBB", (index * interval_ms) & 0xFFFFFFFF, subpage, 0) + bytes(words)


def iter_capture_payloads(frames, interval_ms=500):
    for index, words in enumerate(frames):
        yield capture_payload(words, index, interval_ms)


def render_image(sample, scale=12, tmin=None, tmax=None):
    """Return an annotated BGR uint8 image, without modifying temperatures.

    cv2 is imported only here; decoding/replay requires no GUI dependencies.
    Display limits affect colors only, never the raw temperature Sample.
    """
    import cv2

    if not isinstance(scale, int) or scale < 1:
        raise ValueError("scale must be a positive integer")
    temps = np.asarray(sample.temps)
    if temps.shape != (24, 32) or not np.isfinite(temps).all():
        raise ValueError("rendering requires a finite 24x32 temperature array")
    lo = float(temps.min()) if tmin is None else float(tmin)
    hi = float(temps.max()) if tmax is None else float(tmax)
    if not math.isfinite(lo) or not math.isfinite(hi) or hi < lo:
        raise ValueError("display limits must be finite and tmax >= tmin")
    normalized = np.clip((temps - lo) / max(hi - lo, 1e-6), 0, 1)
    colors = cv2.applyColorMap((normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)
    image = cv2.resize(colors, (32 * scale, 24 * scale), interpolation=cv2.INTER_NEAREST)
    row, col = np.unravel_index(int(np.argmax(temps)), temps.shape)
    center = (int(col * scale + scale // 2), int(row * scale + scale // 2))
    cv2.drawMarker(image, center, (255, 255, 255), cv2.MARKER_CROSS,
                   max(5, scale), 1)
    # Separate caption avoids covering the actual hotspot/data area.
    image = cv2.copyMakeBorder(image, 0, 42, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    caption = "Max {:.2f}  Min {:.2f} C".format(float(temps.max()), float(temps.min()))
    cv2.putText(image, caption, (4, 24 * scale + 16), cv2.FONT_HERSHEY_SIMPLEX,
                .40, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, "Ambient {:.2f} C".format(sample.ambient), (4, 24 * scale + 34),
                cv2.FONT_HERSHEY_SIMPLEX, .40, (255, 255, 255), 1, cv2.LINE_AA)
    return image
