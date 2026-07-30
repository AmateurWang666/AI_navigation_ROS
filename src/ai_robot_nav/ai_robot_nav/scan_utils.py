"""Geometry-agnostic helpers for reading sector distances out of a LaserScan.

Angles follow REP-103: 0 rad points straight ahead and positive angles rotate
counter-clockwise, so the robot's left side has positive angles.

Sector indices are derived from ``angle_min``/``angle_increment`` instead of
assuming one sample per degree, so these helpers work with any scanner
resolution, start angle or field of view.
"""

import math
from typing import Optional, Tuple

from sensor_msgs.msg import LaserScan


def normalize_angle(angle: float) -> float:
    """Wrap an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def sector_min_distance(
    scan: LaserScan,
    center_deg: float,
    half_width_deg: float,
) -> Optional[float]:
    """Return the closest obstacle inside a sector, in metres.

    A reading beyond ``range_max`` (including ``inf``) means "nothing out
    there", so a sector containing only such readings reports ``range_max``.
    ``None`` is reserved for a sector holding no usable measurement at all -
    every sample was NaN or a below-``range_min`` dropout - which lets callers
    tell "nothing is there" apart from "I cannot see".
    """
    if scan is None or not len(scan.ranges) or scan.angle_increment == 0.0:
        return None

    center = normalize_angle(math.radians(center_deg))
    half_width = math.radians(abs(half_width_deg))

    closest: Optional[float] = None
    saw_clear = False

    for index, distance in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if abs(normalize_angle(angle - center)) > half_width:
            continue

        if math.isnan(distance):
            continue
        if distance > scan.range_max:
            saw_clear = True
            continue
        if distance < scan.range_min:
            continue

        if closest is None or distance < closest:
            closest = distance

    if closest is not None:
        return closest
    return scan.range_max if saw_clear else None


def describe_environment(
    scan: LaserScan,
    front_half_deg: float,
    side_center_deg: float,
    side_half_deg: float,
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """Return the closest obstacle ahead, to the left and to the right."""
    front = sector_min_distance(scan, 0.0, front_half_deg)
    left = sector_min_distance(scan, side_center_deg, side_half_deg)
    right = sector_min_distance(scan, -side_center_deg, side_half_deg)
    return front, left, right


def format_distance(distance: Optional[float]) -> str:
    """Render a sector distance for the prompt, spelling out the blind case."""
    if distance is None:
        return '未知(无有效回波)'
    return f'{distance:.2f}m'
