"""Noise rejection and adaptive sampling for Exp16 dial readings (no I/O)."""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple


BATCH_SIZE = 10
MAX_BATCH_SIZE = 20
MIN_VALID_COUNT = 6
_HALF_SIZE = BATCH_SIZE // 2
_TEMP_WIDTH = 1.0  # °C
_HUMIDITY_WIDTH = 2.0  # RH percentage points
_WIDTH_EPSILON = 1e-9  # absolute allowance for subtraction at a width boundary


@dataclass(frozen=True)
class DialResult:
    """The inlier count describes the strongest cluster even on rejection."""

    value: Optional[float]
    status: str  # ok, reference, insufficient, skipped (unstable during evaluation)
    valid_count: int
    inlier_count: int

    def as_dict(self):
        return {
            "value": self.value,
            "status": self.status,
            "valid_count": self.valid_count,
            "inlier_count": self.inlier_count,
        }


@dataclass(frozen=True)
class BatchResult:
    frame_count: int
    temperature: DialResult
    humidity: DialResult

    def as_dict(self):
        return {
            "frame_count": self.frame_count,
            "temperature": self.temperature.as_dict(),
            "humidity": self.humidity.as_dict(),
        }


class StabilityBatch:
    """Collect at least ten attempts, extending for any unresolved requested dial."""

    def __init__(self, *, collect_temperature: bool = True, collect_humidity: bool = True):
        self._temperatures = _DialCollection(collect_temperature, _TEMP_WIDTH)
        self._humidities = _DialCollection(collect_humidity, _HUMIDITY_WIDTH)
        self._count = 0
        self._complete = False

    @property
    def count(self) -> int:
        """Number of attempts, including frames with neither dial readable."""
        return self._count

    @property
    def temperature_valid_count(self) -> int:
        """Number of collected finite temperature readings in the round so far."""
        return self._temperatures.valid_count

    @property
    def humidity_valid_count(self) -> int:
        """Number of collected finite humidity readings in the round so far."""
        return self._humidities.valid_count

    def add(self, temperature: Optional[float], humidity: Optional[float]) -> Optional[BatchResult]:
        """Return one result at completion, then raise on further attempts."""
        if self._complete:
            raise RuntimeError("stability batch is already complete")
        self._temperatures.append(temperature)
        self._humidities.append(humidity)
        self._count += 1
        if self.count < BATCH_SIZE:
            return None

        temperature_result = self._temperatures.resolve()
        humidity_result = self._humidities.resolve()
        if temperature_result is None or humidity_result is None:
            return None

        self._complete = True
        return BatchResult(self.count, temperature_result, humidity_result)


class _DialCollection:
    def __init__(self, collect: bool, width: float):
        self._collect = collect
        self._width = width
        self._readings: List[Optional[float]] = []
        self._result = None if collect else DialResult(None, "skipped", 0, 0)

    @property
    def valid_count(self) -> int:
        return sum(value is not None for value in self._readings)

    def append(self, value: Optional[float]) -> None:
        if self._collect:
            self._readings.append(_finite_value(value))

    def resolve(self) -> Optional[DialResult]:
        if self._result is not None:
            # Lock the first stable result, including its counts, for this round.
            return self._result

        readings = tuple(self._readings)
        split_index = _HALF_SIZE
        if len(readings) > BATCH_SIZE:
            # Halve the inclusive span of valid attempts, retaining interior gaps.
            valid_frames = [index for index, value in enumerate(readings) if value is not None]
            if valid_frames:
                first, last = valid_frames[0], valid_frames[-1]
                split_index = (first + last + 1) // 2
        candidate = _summarize(readings, self._width, split_index)
        if candidate.status == "ok":
            self._result = candidate
        elif len(readings) == MAX_BATCH_SIZE:
            self._result = _reference(readings, self._width, candidate)
        return self._result


def _finite_value(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _summarize(readings: Tuple[Optional[float], ...], width: float,
               split_index: int = _HALF_SIZE) -> DialResult:
    # Sort by value, retaining the original attempted-frame index (not valid rank).
    samples = sorted((value, frame) for frame, value in enumerate(readings) if value is not None)
    valid_count = len(samples)
    best_cluster = []
    best_score = (0, 0, 0.0, 0.0)
    limit = width + _WIDTH_EPSILON
    # Any admissible cluster is a contiguous window in value order. Prefer more
    # inliers, then more in the weaker half, then a tighter and finally lower window.
    for left, (low, _) in enumerate(samples):
        first_half = 0
        for right in range(left, valid_count):
            high, frame = samples[right]
            span = high - low
            if span > limit:
                break
            first_half += frame < split_index
            size = right - left + 1
            score = (size, min(first_half, size - first_half), -span, -low)
            if score > best_score:
                best_score = score
                best_cluster = samples[left:right + 1]

    inlier_count, weaker_half, _, _ = best_score
    if valid_count < MIN_VALID_COUNT:
        return DialResult(None, "insufficient", valid_count, inlier_count)
    if (inlier_count < 5 or 10 * inlier_count < 7 * valid_count or weaker_half < 2
            or _is_clean_drift(readings, limit)):
        return DialResult(None, "unstable", valid_count, inlier_count)

    # For an even cluster, average the middle pair without adding two large values.
    lower_middle = best_cluster[(inlier_count - 1) // 2][0]
    upper_middle = best_cluster[inlier_count // 2][0]
    median = lower_middle + (upper_middle - lower_middle) / 2.0
    return DialResult(median, "ok", valid_count, inlier_count)


def _reference(readings: Tuple[Optional[float], ...], width: float,
               candidate: DialResult) -> DialResult:
    if candidate.inlier_count < 2:
        return DialResult(None, "insufficient", candidate.valid_count, candidate.inlier_count)

    samples = sorted((value, frame) for frame, value in enumerate(readings) if value is not None)
    limit = width + _WIDTH_EPSILON
    left = 0
    best_support = ()
    best_cluster = []
    for right, (high, _) in enumerate(samples):
        while high - samples[left][0] > limit:
            left += 1
        # Only the largest admissible window ending here can be the most populated.
        if right - left + 1 != candidate.inlier_count:
            continue
        cluster = samples[left:right + 1]
        support = tuple(sorted((frame for _, frame in cluster), reverse=True))
        if support > best_support:
            best_support = support
            best_cluster = cluster

    # For an even cluster, the middle pair are equally near the median. Use
    # the newer observation instead of averaging into an unobserved reading.
    lower = (len(best_cluster) - 1) // 2
    upper = len(best_cluster) // 2
    value, _ = max(best_cluster[lower:upper + 1], key=lambda sample: sample[1])
    return DialResult(value, "reference", candidate.valid_count, candidate.inlier_count)


def _is_clean_drift(readings: Tuple[Optional[float], ...], limit: float) -> bool:
    # A monotone progression can put its center in one cluster. Trim one
    # sample from either end so an isolated endpoint cannot trigger the veto.
    values = [value for value in readings if value is not None][1:-1]
    if abs(values[-1] - values[0]) <= limit:
        return False
    differences = [after - before for before, after in zip(values, values[1:])]
    return (all(0.0 < delta <= limit for delta in differences)
            or all(-limit <= delta < 0.0 for delta in differences))
