"""确定性反应式导航策略。

速度完全由激光雷达的几何关系算出。视觉模型只提供一个语义提示，而且接线方式
保证它**只能让机器人更保守**：可以把前方强制判为不可通行、可以按倍率压低巡航
速度、可以在左右空间相近时打破平局。它返回的任何内容都无法提高速度，也无法把
机器人导向雷达判定为封闭的方向。模型幻觉的后果因此被限制在"过于谨慎"这一侧，
而这一侧是安全的。

模块不依赖 rclpy，策略可以直接单元测试；test_navigator.py 把上面这条不变式
写成了断言。
"""

from typing import NamedTuple, Optional

# 判定为"前方不可通行"时代入的距离。用 0.0 而不是 None，是为了让下面的距离
# 比较链保持单一类型，不必到处插入 None 判断。
BLOCKED_AHEAD = 0.0


class NavConfig(NamedTuple):
    """策略的全部可调参数。默认值对应 TurtleBot3 在 Gazebo 中的实测表现。"""

    cruise_speed: float = 0.18        # 前方开阔时的直行速度（m/s）
    turn_speed: float = 0.5           # 前方受阻时的原地转向角速度（rad/s）
    reverse_speed: float = 0.08       # 三面受困时的后退速度（m/s），刻意最慢
    forward_clearance: float = 0.6    # 维持直行所需的前方净空（m）
    trapped_distance: float = 0.4     # 三面均低于此值即判定被困（m）
    tie_threshold: float = 0.3        # 左右差值小于此值才允许视觉打破平局（m）
    caution_scale: float = 0.5        # 视觉报 CAUTION 时的巡航降速倍率，恒小于 1


class Plan(NamedTuple):
    """一次决策的结果。``reason`` 只进日志，用于事后复盘机器人为什么这么走。"""

    action: str        # FORWARD / TURN_LEFT / TURN_RIGHT / REVERSE
    linear_x: float
    angular_z: float
    reason: str


def _usable(distance: Optional[float]) -> float:
    """把"盲区"折算成"不可通行"，绝不折算成"空旷"。

    ``sector_min_distance`` 用 ``None`` 表示该方向没有任何可用回波。这种情况
    只能按最坏结果处理：当成空旷会让机器人径直撞上雷达看不见的东西。
    """
    return BLOCKED_AHEAD if distance is None else distance


def plan(front, left, right, config: NavConfig, hint=None) -> Plan:
    """根据三个扇区距离选定动作，视觉提示仅用于收紧结果。

    下面的判断顺序本身就是优先级：先处理被困（唯一会给出负向速度的分支），
    再看能否直行，最后才决定往哪一侧转。
    """
    ahead = _usable(front)
    to_left = _usable(left)
    to_right = _usable(right)

    # 没有提示时代入中性值，效果与"模型认为一切正常"完全一致。因此模型缺席、
    # 超时或被拒绝时，本函数的行为就是纯激光导航，不需要另一条代码路径。
    hazard = hint.hazard if hint is not None else 'NONE'
    preference = hint.preferred_direction if hint is not None else 'NONE'

    if hazard == 'BLOCKED':
        # 视觉看到了雷达测不到的东西：玻璃隔断、下沉台阶、关着的门。覆盖雷达的
        # 乐观读数，让下面的判断链自然做出反应——包括必要时判定为被困。
        ahead = BLOCKED_AHEAD

    # 三面都贴近障碍：转向也转不出去，只能先退出来再重新决策。
    if (ahead < config.trapped_distance
            and to_left < config.trapped_distance
            and to_right < config.trapped_distance):
        return Plan(
            'REVERSE', -config.reverse_speed, 0.0,
            f'boxed in (front {ahead:.2f}m, left {to_left:.2f}m, right {to_right:.2f}m)')

    # 前方净空足够：直行。这是唯一给出正向速度的分支，也是唯一受 CAUTION 影响
    # 的分支——视觉能压低速度的入口只有这一处。
    if ahead > config.forward_clearance:
        speed = config.cruise_speed
        reason = f'clear ahead ({ahead:.2f}m)'
        if hazard == 'CAUTION':
            speed *= config.caution_scale   # 倍率恒小于 1，所以只可能变慢
            reason += '; vision advises caution'
        return Plan('FORWARD', speed, 0.0, reason)

    # 前方受阻，必须转向。左右差距明显时一律听雷达；只有当两侧空间相当
    # （差值小于 tie_threshold）、雷达本身也无从取舍时，才让视觉的方向偏好
    # 来打破平局。正是这道门槛使模型无法把机器人导向明显更封闭的一侧。
    if abs(to_left - to_right) < config.tie_threshold and preference in ('LEFT', 'RIGHT'):
        go_left = preference == 'LEFT'
        basis = f'sides within {config.tie_threshold:.2f}m, vision prefers {preference.lower()}'
    else:
        go_left = to_left >= to_right
        basis = f'more open side (left {to_left:.2f}m, right {to_right:.2f}m)'

    # 动作名与角速度符号在同一个表达式里产生，两者结构上不可能不一致。
    # （曾经把方向交给模型输出时，正是这里出现过"说左转、却给了右转角速度"。）
    return Plan(
        'TURN_LEFT' if go_left else 'TURN_RIGHT',
        0.0,
        config.turn_speed if go_left else -config.turn_speed,   # REP-103：左正右负
        f'obstacle ahead ({ahead:.2f}m); turning toward {basis}')
