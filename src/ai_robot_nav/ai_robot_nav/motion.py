"""速度限幅工具，AI 节点与安全节点共用。

单独抽成一个模块，是因为限幅在系统里出现两次且含义不同：ai_nav_node 做的是
第一道钳位（防止策略参数配错就把整数值发出去），safety_node 做的是最终生效的
权威钳位。两处必须用同一套逻辑，否则"权威上限"就名不副实了。
"""

from geometry_msgs.msg import Twist


def clamp(value: float, limit: float) -> float:
    """把 ``value`` 约束到 ±``limit`` 区间内。

    ``limit`` 先取绝对值，所以调用方误传负的上限也不会让区间反转成空集。
    """
    bound = abs(limit)
    return max(-bound, min(bound, value))


def clamp_twist(source: Twist, max_linear: float, max_angular: float) -> Twist:
    """把 ``source`` 中实际驱动的两个轴复制到一条新的 Twist 并限幅。

    差速底盘只用 linear.x 和 angular.z。这里刻意新建 Twist 而不是原地修改
    ``source``：其余分量一律保持为零，上游即使在别的轴上写了值也传不下去。
    """
    limited = Twist()
    limited.linear.x = clamp(source.linear.x, max_linear)
    limited.angular.z = clamp(source.angular.z, max_angular)
    return limited
