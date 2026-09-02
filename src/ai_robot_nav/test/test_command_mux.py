"""指令仲裁的不变式：目标点导航独占控制权，过期指令一律降级为停车。

其中最关键的是「过期即停」这一组：仲裁器绝不能沿用上一条指令。旧指令是针对
旧位置算出来的，而 move_base 断流往往正是因为它在规划或执行恢复行为——恰恰是
最不该盲目前进的时刻。改动仲裁逻辑时必须保持这些断言通过。

不需要 ROS 图。
"""

import pytest

from ai_robot_nav.command_mux import (
    GOAL_ABORTED, GOAL_ACTIVE, GOAL_PENDING, GOAL_PREEMPTED, GOAL_REJECTED,
    GOAL_SUCCEEDED, IDLE_EXPLORE, IDLE_STOP, MuxConfig, has_active_goal,
    select_command,
)

CFG = MuxConfig()
GOAL_CMD = (0.18, 0.3)
EXPLORE_CMD = (0.15, -0.4)


def select(goal_active=False, goal_cmd=None, goal_age=0.0,
           explore_cmd=None, explore_age=0.0, config=CFG):
    return select_command(goal_active, goal_cmd, goal_age,
                          explore_cmd, explore_age, config)


# ---------------------------------------------------------------- 目标点优先
def test_active_goal_takes_control():
    result = select(goal_active=True, goal_cmd=GOAL_CMD,
                    explore_cmd=EXPLORE_CMD)
    assert result.source == 'goal'
    assert result.command == GOAL_CMD


def test_active_goal_silences_exploration():
    """有目标点时漫游必须彻底让出，否则两路指令会互相覆盖。"""
    result = select(goal_active=True, goal_cmd=GOAL_CMD,
                    explore_cmd=EXPLORE_CMD, explore_age=0.0)
    assert result.command != EXPLORE_CMD


def test_goal_command_is_forwarded_unmodified():
    """仲裁器只做选择，不做限幅——限幅是 safety_node 的职责，不能有两套上限。"""
    for command in [(0.0, 0.0), (0.2, -1.2), (-0.05, 0.9)]:
        result = select(goal_active=True, goal_cmd=command)
        assert result.command == command


# ---------------------------------------------------------------- 过期即停
def test_stale_goal_command_stops_instead_of_repeating():
    """move_base 断流时输出零速，绝不沿用上一条指令。"""
    result = select(goal_active=True, goal_cmd=GOAL_CMD,
                    goal_age=CFG.goal_cmd_timeout + 0.01)
    assert result.source == 'stop'
    assert result.command == (0.0, 0.0)


def test_active_goal_without_any_command_stops():
    """刚下发目标、move_base 还没算完时同样必须停住。"""
    result = select(goal_active=True, goal_cmd=None, explore_cmd=EXPLORE_CMD)
    assert result.source == 'stop'
    assert result.command == (0.0, 0.0)


def test_stale_goal_does_not_fall_back_to_exploration():
    """目标点期间 move_base 断流，不能悄悄改由漫游层接管。

    那会让机器人在导航途中突然改走反应式路线，方向大幅跳变，
    而使用者以为它还在按规划的路径走。
    """
    result = select(goal_active=True, goal_cmd=GOAL_CMD,
                    goal_age=99.0, explore_cmd=EXPLORE_CMD, explore_age=0.0)
    assert result.source == 'stop'


def test_stale_explore_command_stops():
    result = select(goal_active=False, explore_cmd=EXPLORE_CMD,
                    explore_age=CFG.explore_cmd_timeout + 0.01)
    assert result.source == 'stop'
    assert result.command == (0.0, 0.0)


def test_fresh_command_exactly_at_timeout_is_still_accepted():
    """边界取值不应判为过期，避免在阈值上反复抖动。"""
    result = select(goal_active=True, goal_cmd=GOAL_CMD,
                    goal_age=CFG.goal_cmd_timeout)
    assert result.source == 'goal'


# ---------------------------------------------------------------- 空闲行为
def test_no_goal_falls_back_to_exploration():
    result = select(goal_active=False, explore_cmd=EXPLORE_CMD)
    assert result.source == 'explore'
    assert result.command == EXPLORE_CMD


def test_idle_stop_never_explores():
    """idle_behavior=stop 时即使漫游指令新鲜也不能动。"""
    config = CFG._replace(idle_behavior=IDLE_STOP)
    result = select(goal_active=False, explore_cmd=EXPLORE_CMD, config=config)
    assert result.source == 'stop'
    assert result.command == (0.0, 0.0)


def test_no_goal_and_no_explore_command_stops():
    result = select(goal_active=False, explore_cmd=None)
    assert result.source == 'stop'


def test_move_base_absent_behaves_like_before_its_introduction():
    """不启动 navigation.launch 时，链路行为必须与引入 move_base 之前一致。

    此时 /move_base/status 永远没有消息，goal_active 恒为 False，
    漫游指令应当原样透传。
    """
    result = select(goal_active=False, explore_cmd=EXPLORE_CMD,
                    config=MuxConfig(idle_behavior=IDLE_EXPLORE))
    assert result.source == 'explore'
    assert result.command == EXPLORE_CMD


# ---------------------------------------------------------------- 目标状态判定
def test_only_pending_and_active_count_as_running():
    assert has_active_goal([GOAL_PENDING])
    assert has_active_goal([GOAL_ACTIVE])
    for terminal in (GOAL_SUCCEEDED, GOAL_ABORTED, GOAL_REJECTED, GOAL_PREEMPTED):
        assert not has_active_goal([terminal])


def test_finished_goal_history_does_not_keep_control():
    """move_base 会保留刚结束的目标条目；只看数组非空会让控制权永远交不回去。"""
    assert not has_active_goal([GOAL_SUCCEEDED, GOAL_ABORTED])


def test_running_goal_among_finished_ones_still_counts():
    assert has_active_goal([GOAL_SUCCEEDED, GOAL_ACTIVE, GOAL_ABORTED])


def test_empty_status_means_no_goal():
    assert not has_active_goal([])
    assert not has_active_goal(None)
