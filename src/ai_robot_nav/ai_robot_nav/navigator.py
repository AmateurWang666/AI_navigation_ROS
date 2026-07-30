"""Deterministic reactive navigation policy.

Velocities are computed from LiDAR geometry alone. The vision model contributes
only a semantic hint, and the hint is wired so it can exclusively make the robot
more conservative: it can force the path ahead to count as blocked, scale the
cruise speed down, or break a left/right tie. Nothing it returns can raise a
speed or send the robot toward a direction the LiDAR reports as closed.

Kept free of rclpy so the policy can be unit tested directly.
"""

from typing import NamedTuple, Optional

BLOCKED_AHEAD = 0.0


class NavConfig(NamedTuple):
    cruise_speed: float = 0.18
    turn_speed: float = 0.5
    reverse_speed: float = 0.08
    forward_clearance: float = 0.6
    trapped_distance: float = 0.4
    tie_threshold: float = 0.3
    caution_scale: float = 0.5


class Plan(NamedTuple):
    action: str
    linear_x: float
    angular_z: float
    reason: str


def _usable(distance: Optional[float]) -> float:
    """A sector with no valid return counts as blocked, never as open."""
    return BLOCKED_AHEAD if distance is None else distance


def plan(front, left, right, config: NavConfig, hint=None) -> Plan:
    """Choose an action from sector distances, optionally tempered by a vision hint."""
    ahead = _usable(front)
    to_left = _usable(left)
    to_right = _usable(right)

    hazard = hint.hazard if hint is not None else 'NONE'
    preference = hint.preferred_direction if hint is not None else 'NONE'

    if hazard == 'BLOCKED':
        # Vision sees something the scanner misses - glass, a drop-off, a closed
        # door. Override the LiDAR's optimism and let the cascade below react.
        ahead = BLOCKED_AHEAD

    if (ahead < config.trapped_distance
            and to_left < config.trapped_distance
            and to_right < config.trapped_distance):
        return Plan(
            'REVERSE', -config.reverse_speed, 0.0,
            f'boxed in (front {ahead:.2f}m, left {to_left:.2f}m, right {to_right:.2f}m)')

    if ahead > config.forward_clearance:
        speed = config.cruise_speed
        reason = f'clear ahead ({ahead:.2f}m)'
        if hazard == 'CAUTION':
            speed *= config.caution_scale
            reason += '; vision advises caution'
        return Plan('FORWARD', speed, 0.0, reason)

    if abs(to_left - to_right) < config.tie_threshold and preference in ('LEFT', 'RIGHT'):
        go_left = preference == 'LEFT'
        basis = f'sides within {config.tie_threshold:.2f}m, vision prefers {preference.lower()}'
    else:
        go_left = to_left >= to_right
        basis = f'more open side (left {to_left:.2f}m, right {to_right:.2f}m)'

    return Plan(
        'TURN_LEFT' if go_left else 'TURN_RIGHT',
        0.0,
        config.turn_speed if go_left else -config.turn_speed,
        f'obstacle ahead ({ahead:.2f}m); turning toward {basis}')
