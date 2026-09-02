"""速度指令仲裁：在「目标点导航」与「反应式漫游」之间选择一路输出。

引入 move_base 后，系统里出现了两个速度来源：

    move_base   ──► /nav_cmd_vel       有目标点时的全局规划结果
    ai_nav_node ──► /explore_cmd_vel   无目标点时的反应式漫游

它们不能同时驱动底盘，必须选一路。本模块就是这个选择器。

**为什么不让 move_base 直接发 /cmd_vel**：本仓库有一条贯穿始终的不变式——
``safety_node`` 是 ``/cmd_vel`` 的唯一发布者，它独立于上层逻辑，靠激光做急停。
move_base 默认发 ``/cmd_vel``，接进来就等于在安全层旁边开了一个后门：
move_base 崩溃、卡在恢复行为、或者规划出一条贴墙的路径时，都没有任何东西
能拦住它。所以 move_base 的输出被重映射到 ``/nav_cmd_vel``，经本模块汇流到
``/ai_cmd_vel``，仍然要过 ``safety_node`` 这一关才能到底盘。急停权不下放。

模块是纯函数，不依赖 rospy，可以直接单元测试。
"""

from typing import NamedTuple, Optional, Tuple

# 指令用 (linear_x, angular_z) 二元组表示，不引入 geometry_msgs，
# 这样策略层可以脱离 ROS 消息类型单独测试。
Command = Tuple[float, float]

ZERO: Command = (0.0, 0.0)

# 无目标点时的两种行为。explore 保留本项目原有的反应式漫游能力，
# stop 则是「没给目标就别动」，实车调试阶段通常更希望这样。
IDLE_EXPLORE = 'explore'
IDLE_STOP = 'stop'


class MuxConfig(NamedTuple):
    """仲裁器的全部可调参数。"""

    goal_cmd_timeout: float = 0.5     # /nav_cmd_vel 超过此时长未更新即视为失效（s）
    explore_cmd_timeout: float = 0.7  # /explore_cmd_vel 的同上（s）
    idle_behavior: str = IDLE_EXPLORE  # 无目标点时漫游还是停车


class MuxDecision(NamedTuple):
    """一次仲裁的结果。``reason`` 只进日志，用于事后复盘。"""

    source: str        # 'goal' / 'explore' / 'stop'
    linear_x: float
    angular_z: float
    reason: str

    @property
    def command(self) -> Command:
        return (self.linear_x, self.angular_z)


def _stop(reason: str) -> MuxDecision:
    return MuxDecision('stop', 0.0, 0.0, reason)


def select_command(
    goal_active: bool,
    goal_cmd: Optional[Command],
    goal_cmd_age: float,
    explore_cmd: Optional[Command],
    explore_cmd_age: float,
    config: MuxConfig,
) -> MuxDecision:
    """选定本周期要下发的速度。

    ``*_age`` 是调用方测得的「该指令距今多少秒」。时间不在本模块里读取，
    仲裁因此保持纯函数、可复现、可单测；它只是又一个输入量。

    判断顺序即优先级：目标点导航一旦激活就独占控制权，漫游必须让出，否则两路
    指令会在同一个话题上互相覆盖，机器人表现为方向来回跳变。
    """
    if goal_active:
        # move_base 在规划、执行恢复行为、或刚接到目标还没算完时，会有一小段
        # 时间不发速度。这时必须输出零速而不是沿用上一条指令：上一条是针对旧位置
        # 算出来的，继续执行等于闭着眼睛走，而恢复行为本身往往就是因为前方有问题。
        if goal_cmd is None:
            return _stop('目标点已激活，但尚未收到 move_base 的速度指令')
        if goal_cmd_age > config.goal_cmd_timeout:
            return _stop(
                f'move_base 速度指令已过期 {goal_cmd_age:.1f}s '
                f'(>{config.goal_cmd_timeout:.1f}s)，可能正在规划或执行恢复行为')
        return MuxDecision('goal', goal_cmd[0], goal_cmd[1], '目标点导航中')

    if config.idle_behavior == IDLE_STOP:
        return _stop('无目标点，idle_behavior=stop')

    if explore_cmd is None:
        return _stop('无目标点，且尚未收到漫游指令')
    if explore_cmd_age > config.explore_cmd_timeout:
        return _stop(
            f'无目标点，漫游指令已过期 {explore_cmd_age:.1f}s '
            f'(>{config.explore_cmd_timeout:.1f}s)')

    return MuxDecision('explore', explore_cmd[0], explore_cmd[1], '无目标点，反应式漫游')


# move_base 的 actionlib 状态码（actionlib_msgs/GoalStatus）。
# 只有 PENDING/ACTIVE 表示目标仍在执行；其余都是终态，不该继续占用控制权。
GOAL_PENDING = 0
GOAL_ACTIVE = 1
GOAL_PREEMPTED = 2
GOAL_SUCCEEDED = 3
GOAL_ABORTED = 4
GOAL_REJECTED = 5

RUNNING_GOAL_STATES = (GOAL_PENDING, GOAL_ACTIVE)


def has_active_goal(status_codes) -> bool:
    """从 move_base 的状态数组判断当前是否有目标在执行。

    move_base 的 /move_base/status 会保留一段时间的历史条目，其中既有正在执行的
    目标，也有刚刚结束的。因此不能看「数组非空」，必须逐个看状态码——否则目标
    到达之后仲裁器会一直以为还在导航，永远不把控制权还给漫游层。
    """
    if not status_codes:
        return False
    return any(code in RUNNING_GOAL_STATES for code in status_codes)
