#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dial angle math and nonlinear scale lookup (G1a, pure functions).

This module deliberately contains no image processing, camera, ROS, model,
GUI or file access.  Angles are in degrees with the image convention used by
the handoff document: x to the right, y downwards, clockwise positive.

The 11-point table accepted here is only a *structure* check.  A valid table
does not prove that a real calibration exists or that a humidity reading is
accurate.
"""

import bisect
import math
import numbers
from typing import List, Optional, Tuple


TABLE_SIZE = 11
ENDPOINT_EPS_DEG = 1e-9


def _finite_real(value, name: str) -> float:
    """Return *value* as a finite float, or raise ValueError.

    bool is rejected explicitly because it is a subclass of int.  Strings,
    containers, complex numbers and non-finite values are also rejected.
    """
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError("{} must be a finite real number".format(name))
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError):
        raise ValueError("{} must be a finite real number".format(name))
    if not math.isfinite(result):
        raise ValueError("{} must be finite".format(name))
    return result


def _table_values(table, name: str) -> List[float]:
    if not isinstance(table, (list, tuple)):
        raise ValueError("{} must be a one-dimensional list or tuple".format(name))
    if len(table) != TABLE_SIZE:
        raise ValueError("{} must contain exactly {} values".format(name, TABLE_SIZE))
    return [
        _finite_real(item, "{}[{}]".format(name, index))
        for index, item in enumerate(table)
    ]


def _validated_scale(table_deg, table_val) -> Tuple[List[float], List[float]]:
    deg = _table_values(table_deg, "table_deg")
    val = _table_values(table_val, "table_val")

    for index in range(1, TABLE_SIZE):
        if not deg[index] > deg[index - 1]:
            raise ValueError("table_deg must be strictly increasing")

    span = deg[-1] - deg[0]
    if not math.isfinite(span):
        raise ValueError("table_deg span overflowed")
    if span <= 0.0 or span >= 360.0:
        raise ValueError(
            "table_deg span must be greater than 0 and less than 360 degrees")

    expected_values = [float(value) for value in range(0, 101, 10)]
    for index, expected in enumerate(expected_values):
        if val[index] != expected:
            raise ValueError("table_val must be exactly 0, 10, ..., 100")

    return deg, val


def validate_scale(table_deg, table_val) -> None:
    """Validate the structure of an 11-point degree/value table.

    Requirements: two one-dimensional list/tuple objects, 11 finite real
    numbers each, strictly increasing degrees with a total span in (0, 360),
    and values exactly 0, 10, ..., 100.  Invalid input raises ValueError.

    This is a structure-only check; it does not prove a real calibration.
    """
    _validated_scale(table_deg, table_val)
    return None


def wrap_deg(angle_deg) -> float:
    """Wrap a finite angle into [0, 360).  Invalid input raises ValueError."""
    angle = _finite_real(angle_deg, "angle_deg")
    wrapped = math.fmod(angle, 360.0)
    if wrapped < 0.0:
        wrapped += 360.0
    # Avoid returning 360.0 after adding a tiny negative value and avoid -0.0.
    if wrapped == 360.0 or wrapped == 0.0:
        wrapped = 0.0
    return wrapped


def relative_angle(needle_deg, roll_deg, zero_deg) -> float:
    """Return zero_deg + wrap_deg(needle_deg - roll_deg - zero_deg).

    roll_deg is the observed roll difference relative to calibration; this
    function does not detect the blue segment.  All input angles use the
    image-clockwise convention.  Invalid input or intermediate overflow
    raises ValueError.
    """
    needle = _finite_real(needle_deg, "needle_deg")
    roll = _finite_real(roll_deg, "roll_deg")
    zero = _finite_real(zero_deg, "zero_deg")

    delta = needle - roll
    if not math.isfinite(delta):
        raise ValueError("angle difference overflowed")
    delta = delta - zero
    if not math.isfinite(delta):
        raise ValueError("angle difference overflowed")

    result = zero + wrap_deg(delta)
    if not math.isfinite(result):
        raise ValueError("relative angle overflowed")
    return result


def lookup_value(angle_deg, table_deg, table_val) -> Optional[float]:
    """Linearly interpolate an already-unwrapped angle on a validated table.

    The angle must already have roll removed and be expanded relative to the
    zero mark.  This function never wraps the angle and never clamps an
    out-of-range angle to the first/last value; outside the closed table
    interval it returns None.  Only the two table endpoints absorb a floating
    rounding error of at most 1e-9 degrees.  Invalid input raises ValueError;
    normal out-of-range input returns None.
    """
    deg, val = _validated_scale(table_deg, table_val)
    angle = _finite_real(angle_deg, "angle_deg")

    first = deg[0]
    last = deg[-1]

    if angle < first - ENDPOINT_EPS_DEG or angle > last + ENDPOINT_EPS_DEG:
        return None
    if angle <= first:
        return val[0]
    if angle >= last:
        return val[-1]

    index = bisect.bisect_right(deg, angle) - 1
    if angle == deg[index]:
        return val[index]

    low_deg = deg[index]
    high_deg = deg[index + 1]
    low_val = val[index]
    high_val = val[index + 1]
    result = low_val + (high_val - low_val) * (
        (angle - low_deg) / (high_deg - low_deg))
    if not math.isfinite(result):
        raise ValueError("interpolated value is not finite")
    return float(result)
