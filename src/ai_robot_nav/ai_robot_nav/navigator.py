"""确定性反应式导航策略。

速度完全由激光雷达的几何关系算出。视觉模型只提供一个语义提示，而且接线方式
保证它**只能让机器人更保守**：可以把前方强制判为不可通行、可以按倍率压低巡航
速度、可以在左右空间相近时打破平局。它返回的任何内容都无法提高速度，也无法把
机器人导向雷达判定为封闭的方向。模型幻觉的后果因此被限制在"过于谨慎"这一侧，
而这一侧是安全的。

模块不依赖 rclpy，策略可以直接单元测试；test_navigator.py 把上面这条不变式
写成了断言。
"""

from typing import NamedTuple, Optional, Tuple

# 判定为"前方不可通行"时代入的距离。用 0.0 而不是 None，是为了让下面的距离
# 比较链保持单一类型，不必到处插入 None 判断。
BLOCKED_AHEAD = 0.0

# 动作名与转向侧的对应表。ESCAPE_* 是"边退边转"的脱困动作，方向语义与同侧的
# TURN_* 完全一致，因此在需要判断"上次往哪边转"的地方两者可以互换。
LEFT_ACTIONS = ('TURN_LEFT', 'ESCAPE_LEFT')
RIGHT_ACTIONS = ('TURN_RIGHT', 'ESCAPE_RIGHT')
TURNING_ACTIONS = LEFT_ACTIONS + RIGHT_ACTIONS


class NavConfig(NamedTuple):
    """策略的全部可调参数。实车与仿真共用，差异只体现在 yaml 里的取值。"""

    cruise_speed: float = 0.18        # 前方开阔时的直行速度（m/s）
    turn_speed: float = 0.5           # 前方受阻时的原地转向角速度（rad/s）
    reverse_speed: float = 0.08       # 后退速度（m/s），刻意最慢
    reverse_clearance: float = 0.4    # 后方净空不足时不盲退（m）
    forward_clearance: float = 0.6    # 恢复直行所需的前方净空（m）
    turn_clearance: float = 0.5       # 开始转向所需的前方净空（m），与上行形成迟滞带
    trapped_distance: float = 0.4     # 三面均低于此值即判定被困（m）
    tie_threshold: float = 0.3        # 左右差值小于此值视为平局，启用方向保持（m）
    caution_scale: float = 0.5        # 视觉报 CAUTION 时的巡航降速倍率，恒小于 1
    escape_commit_time: float = 4.0   # 连续转向超过此时长后锁死方向（s），0 关闭
    escape_reverse_time: float = 9.0  # 再超过此时长改为边退边转（s），0 关闭


class Plan(NamedTuple):
    """一次决策的结果。``reason`` 只进日志，用于事后复盘机器人为什么这么走。"""

    action: str        # FORWARD / TURN_LEFT / TURN_RIGHT / ESCAPE_* / REVERSE
    linear_x: float
    angular_z: float
    reason: str


def _usable(distance: Optional[float]) -> float:
    """把"盲区"折算成"不可通行"，绝不折算成"空旷"。

    ``sector_min_distance`` 用 ``None`` 表示该方向没有任何可用回波。这种情况
    只能按最坏结果处理：当成空旷会让机器人径直撞上雷达看不见的东西。
    """
    return BLOCKED_AHEAD if distance is None else distance


def _effective_tie_threshold(ahead: float, config: NavConfig) -> float:
    """前方越近，转向方向锁定越紧，避免贴墙时左右来回摆。"""
    if ahead <= config.turn_clearance:
        return max(config.tie_threshold * 3.0, 1.0)
    return config.tie_threshold


def _latched_side(action: Optional[str]) -> Optional[bool]:
    """把上一次的动作名折算成"左/右"。非转向动作返回 None。"""
    if action in LEFT_ACTIONS:
        return True
    if action in RIGHT_ACTIONS:
        return False
    return None


def _choose_turn_side(
    to_left: float,
    to_right: float,
    config: NavConfig,
    preference: str,
    last_turn: Optional[str],
    tie_threshold: Optional[float] = None,
    committed: bool = False,
) -> Tuple[bool, str]:
    """在需要转向时选定左右，带死区与方向保持。

    明显更优的一侧（差距超过 tie_threshold）始终优先；落在死区内时不再用
    ``>=`` 裸比较，而是保持上次转向、听视觉偏好，或确定性默认左转。

    ``committed`` 为真时完全跳过左右比较，直接沿用已锁定的方向。这条路径只在
    机器人已经连续转了很久（见 ``escape_commit_time``）时启用：此时"哪边更开阔"
    显然没能把它带出来，继续跟着这个量走只会让它在两侧之间反复改主意。
    """
    if committed:
        latched = _latched_side(last_turn)
        if latched is not None:
            side = 'left' if latched else 'right'
            return latched, f'committed {side} (escape in progress)'

    threshold = config.tie_threshold if tie_threshold is None else tie_threshold
    diff = to_left - to_right

    if diff > threshold:
        return True, f'more open side (left {to_left:.2f}m, right {to_right:.2f}m)'
    if diff < -threshold:
        return False, f'more open side (left {to_left:.2f}m, right {to_right:.2f}m)'

    if preference == 'LEFT':
        return True, f'sides within {threshold:.2f}m, vision prefers left'
    if preference == 'RIGHT':
        return False, f'sides within {threshold:.2f}m, vision prefers right'

    latched = _latched_side(last_turn)
    if latched is not None:
        side = 'left' if latched else 'right'
        return latched, f'sides within {threshold:.2f}m, holding {side}'

    return True, f'sides within {threshold:.2f}m, defaulting left'


def _effective_last_action(
    last_action: Optional[str],
    last_turn: Optional[str],
) -> Optional[str]:
    """合并上一周期的动作状态，兼容只传 last_turn 的调用方式。"""
    if last_action is not None:
        return last_action
    if last_turn in TURNING_ACTIONS:
        return last_turn
    return None


def _should_drive_forward(
    ahead: float,
    config: NavConfig,
    last_action: Optional[str],
) -> bool:
    """判断是否应直行，带 forward_clearance / turn_clearance 迟滞。

    明显开阔（> forward_clearance）或明显受阻（<= turn_clearance）时直接判定；
    落在迟滞带内则保持上一动作，避免 FORWARD 与 TURN 在边界上来回切换。
    """
    if ahead > config.forward_clearance:
        return True
    if ahead <= config.turn_clearance:
        return False
    if last_action == 'FORWARD':
        return True
    if last_action in TURNING_ACTIONS:
        return False
    return False


def _forward_plan(ahead: float, config: NavConfig, hazard: str, extra: str = '') -> Plan:
    speed = config.cruise_speed
    reason = f'clear ahead ({ahead:.2f}m)'
    if hazard == 'CAUTION':
        speed *= config.caution_scale
        reason += '; vision advises caution'
    if extra:
        reason += extra
    return Plan('FORWARD', speed, 0.0, reason)


def _rear_safe(rear, config: NavConfig) -> bool:
    """后方是否有足够空间安全后退。盲区视为不可退。"""
    if rear is None:
        return False
    return _usable(rear) >= config.reverse_clearance


def _turn_plan(
    go_left: bool,
    basis: str,
    config: NavConfig,
    turning_for: float,
    prefix: str,
) -> Plan:
    """生成转向动作；转了太久就升级为"边退边转"。

    纯原地转向对一台差速+万向轮底盘几乎总能脱困，除非车头已经顶在障碍上——
    此时轮子只会空转。给一个很小的负向线速度把车头拽离障碍，同时保持角速度，
    机器人就会沿倒车弧线退出来，而方向仍是已锁定的那一侧。
    """
    angular = config.turn_speed if go_left else -config.turn_speed   # REP-103：左正右负

    if config.escape_reverse_time > 0.0 and turning_for >= config.escape_reverse_time:
        return Plan(
            'ESCAPE_LEFT' if go_left else 'ESCAPE_RIGHT',
            -config.reverse_speed,
            angular,
            f'{prefix}; turning for {turning_for:.1f}s without clearing, '
            f'backing out while turning toward {basis}')

    # 动作名与角速度符号在同一个表达式里产生，两者结构上不可能不一致。
    # （曾经把方向交给模型输出时，正是这里出现过"说左转、却给了右转角速度"。）
    return Plan(
        'TURN_LEFT' if go_left else 'TURN_RIGHT',
        0.0,
        angular,
        f'{prefix}; turning toward {basis}')


def plan(
    front,
    left,
    right,
    config: NavConfig,
    hint=None,
    last_turn: Optional[str] = None,
    last_action: Optional[str] = None,
    rear=None,
    turn_elapsed: float = 0.0,
) -> Plan:
    """根据三个扇区距离选定动作，视觉提示仅用于收紧结果。

    下面的判断顺序本身就是优先级：先处理被困（唯一会给出负向速度的分支），
    再看能否直行，最后才决定往哪一侧转。

    ``turn_elapsed`` 是调用方测得的"已经连续转向多少秒"。时间不在本模块里读取，
    策略因此保持纯函数、可复现、可单测；它只是又一个输入量。
    """
    ahead = _usable(front)
    to_left = _usable(left)
    to_right = _usable(right)
    turning_for = max(0.0, turn_elapsed)

    # 没有提示时代入中性值，效果与"模型认为一切正常"完全一致。因此模型缺席、
    # 超时或被拒绝时，本函数的行为就是纯激光导航，不需要另一条代码路径。
    hazard = hint.hazard if hint is not None else 'NONE'
    preference = hint.preferred_direction if hint is not None else 'NONE'
    prior_action = _effective_last_action(last_action, last_turn)
    if last_turn is None and prior_action in TURNING_ACTIONS:
        last_turn = prior_action

    # 转了很久还没转出去，说明"哪边更开阔"这个量在当前位置没有指导意义，
    # 锁死已选方向直到脱困为止。
    committed = (config.escape_commit_time > 0.0
                 and turning_for >= config.escape_commit_time)

    if config.turn_clearance >= config.forward_clearance:
        config = config._replace(turn_clearance=config.forward_clearance - 0.05)

    if hazard == 'BLOCKED':
        # 视觉看到了雷达测不到的东西：玻璃隔断、下沉台阶、关着的门。覆盖雷达的
        # 乐观读数，让下面的判断链自然做出反应——包括必要时判定为被困。
        ahead = BLOCKED_AHEAD

    # 三面都贴近障碍：后方空间足够才后退，否则转向脱困，避免盲退撞墙。
    if (ahead < config.trapped_distance
            and to_left < config.trapped_distance
            and to_right < config.trapped_distance):
        if _rear_safe(rear, config):
            return Plan(
                'REVERSE', -config.reverse_speed, 0.0,
                f'boxed in (front {ahead:.2f}m, left {to_left:.2f}m, right {to_right:.2f}m); '
                f'rear clear ({_usable(rear):.2f}m)')
        go_left, basis = _choose_turn_side(
            to_left, to_right, config, preference, last_turn,
            tie_threshold=_effective_tie_threshold(ahead, config),
            committed=committed)
        rear_note = 'blind' if rear is None else f'{_usable(rear):.2f}m'
        return _turn_plan(
            go_left, basis, config, turning_for,
            f'boxed in but rear blocked ({rear_note})')

    if _should_drive_forward(ahead, config, prior_action):
        extra = ''
        if (config.turn_clearance < ahead <= config.forward_clearance
                and prior_action == 'FORWARD'):
            extra = '; holding forward in clearance hysteresis'
        return _forward_plan(ahead, config, hazard, extra)

    # 前方受阻，必须转向。tie_threshold 对纯激光同样生效：落在死区内时保持
    # 上次方向，避免 10 Hz 控制回路因测量噪声在左右之间来回切换。
    go_left, basis = _choose_turn_side(
        to_left, to_right, config, preference, last_turn,
        tie_threshold=_effective_tie_threshold(ahead, config),
        committed=committed)

    return _turn_plan(
        go_left, basis, config, turning_for, f'obstacle ahead ({ahead:.2f}m)')
