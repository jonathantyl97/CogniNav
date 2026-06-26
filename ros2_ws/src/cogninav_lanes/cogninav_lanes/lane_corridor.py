"""Test whether image points lie between detected lane lines."""

from __future__ import annotations

from typing import Optional, Tuple


def x_on_segment_at_y(
    line: Tuple[float, float, float, float],
    y: float,
    extrapolate: float = 0.15,
) -> Optional[float]:
    """Return x on the segment (or slight extrapolation) at image row y."""
    x1, y1, x2, y2 = line
    if abs(y2 - y1) < 1e-6:
        return None
    t = (y - y1) / (y2 - y1)
    if t < -extrapolate or t > 1.0 + extrapolate:
        return None
    return x1 + t * (x2 - x1)


def point_in_lane_corridor(
    u: float,
    v: float,
    left: Optional[Tuple[float, float, float, float]],
    right: Optional[Tuple[float, float, float, float]],
    margin_px: float = 5.0,
) -> bool:
    """True if pixel (u, v) lies between left and right lane lines at row v."""
    if left is None or right is None:
        return False
    x_left = x_on_segment_at_y(left, v)
    x_right = x_on_segment_at_y(right, v)
    if x_left is None or x_right is None:
        return False
    if x_left > x_right:
        x_left, x_right = x_right, x_left
    return (x_left - margin_px) <= u <= (x_right + margin_px)
