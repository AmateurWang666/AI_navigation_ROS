"""Velocity limiting helpers shared by the AI and safety nodes."""

from geometry_msgs.msg import Twist


def clamp(value: float, limit: float) -> float:
    """Constrain ``value`` to +/-``limit``."""
    bound = abs(limit)
    return max(-bound, min(bound, value))


def clamp_twist(source: Twist, max_linear: float, max_angular: float) -> Twist:
    """Copy the driven axes of ``source`` into a fresh Twist, limited in magnitude."""
    limited = Twist()
    limited.linear.x = clamp(source.linear.x, max_linear)
    limited.angular.z = clamp(source.angular.z, max_angular)
    return limited
