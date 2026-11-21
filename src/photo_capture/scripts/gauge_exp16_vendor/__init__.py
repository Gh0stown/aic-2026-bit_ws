"""Runtime-only port of the 2026-11-21 Exp16 ROS deployment handoff.

The batch CLI, file I/O and cross-image consensus are intentionally omitted.
All geometry returned by a crop reader is relative to its input crop.
"""

from dataclasses import dataclass
from typing import Tuple


class UnreadableGauge(Exception):
    """A detection exists, but its crop does not support a valid reading."""


@dataclass(frozen=True)
class CropReading:
    value: float
    center: Tuple[int, int]
    radius: int
    angle: float  # Degrees counter-clockwise from right, y increasing down.
    tick_count: int
    method: str
