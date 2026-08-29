"""策略必须完全由激光雷达决定；视觉只允许让它更保守。

这组测试把 navigator 的核心不变式写成断言，其中最关键的是最后几条：视觉提示
不能提速、不能把机器人导向明显更封闭的一侧。这些正是"模型幻觉不会造成危险"
这一设计承诺的可执行形式，改动策略时必须保持它们通过。

不需要 ROS 图，也不需要 Ollama 服务。
"""

import pytest

from ai_robot_nav.llm_client import Assessment
from ai_robot_nav.navigator import NavConfig, plan

CFG = NavConfig()


def hint(hazard='NONE', direction='NONE'):
    """构造一个视觉提示。默认值等价于"模型认为一切正常"。"""
    return Assessment(hazard=hazard, preferred_direction=direction, description='')


def test_clear_path_drives_forward():
    result = plan(3.0, 3.0, 3.0, CFG)
    assert result.action == 'FORWARD'
    assert result.linear_x == pytest.approx(CFG.cruise_speed)
    assert result.angular_z == pytest.approx(0.0)


def test_blocked_front_turns_toward_the_open_side():
    left = plan(0.4, 2.0, 0.5, CFG)
    assert left.action == 'TURN_LEFT'
    assert left.angular_z > 0.0          # REP-103 下正值为左
    assert left.linear_x == pytest.approx(0.0)

    right = plan(0.4, 0.5, 2.0, CFG)
    assert right.action == 'TURN_RIGHT'
    assert right.angular_z < 0.0


def test_turn_sign_always_matches_the_action_name():
    """曾在 llava:7b 上观察到的"说左转却右转"现在结构上不可能出现。

    动作名和角速度符号由同一个表达式产生，所以这里遍历各种左右组合，
    只要动作名确定，符号就必须与之一致。
    """
    for left_dist, right_dist in [(2.0, 0.5), (0.5, 2.0), (1.0, 1.0), (0.45, 0.44)]:
        result = plan(0.3, left_dist, right_dist, CFG)
        if result.action == 'TURN_LEFT':
            assert result.angular_z > 0.0
        elif result.action == 'TURN_RIGHT':
            assert result.angular_z < 0.0


def test_boxed_in_reverses_when_rear_is_clear():
    """三面都贴近障碍且后方开阔时才后退。"""
    result = plan(0.2, 0.2, 0.2, CFG, rear=3.0)
    assert result.action == 'REVERSE'
    assert result.linear_x < 0.0


def test_boxed_in_turns_when_rear_is_blocked():
    """后方贴近障碍时不盲退，改转向脱困。"""
    result = plan(0.2, 0.2, 0.2, CFG, rear=0.2)
    assert result.action in ('TURN_LEFT', 'TURN_RIGHT')
    assert result.linear_x == pytest.approx(0.0)


def test_blind_sector_counts_as_blocked_not_open():
    """None 表示该方向没有有效回波，必须按不可通行处理，不能按空旷处理。"""
    assert plan(None, 3.0, 3.0, CFG).action != 'FORWARD'
    assert plan(None, None, None, CFG, rear=3.0).action == 'REVERSE'


def test_caution_only_scales_speed_down():
    """CAUTION 只改变速度大小，不改变动作，而且只能改小。"""
    baseline = plan(3.0, 3.0, 3.0, CFG)
    cautious = plan(3.0, 3.0, 3.0, CFG, hint('CAUTION'))
    assert cautious.action == 'FORWARD'
    assert cautious.linear_x == pytest.approx(baseline.linear_x * CFG.caution_scale)
    assert cautious.linear_x < baseline.linear_x


def test_blocked_hazard_overrides_an_open_lidar_reading():
    """视觉能看到雷达报"空旷"的玻璃和下沉台阶，此时必须以视觉为准。"""
    result = plan(3.0, 3.0, 0.5, CFG, hint('BLOCKED'))
    assert result.action != 'FORWARD'
    assert result.linear_x <= 0.0


def test_blocked_hazard_with_no_way_out_reverses():
    """BLOCKED 会一路传导到被困判定：前方作废、两侧又都很窄，就该后退。"""
    assert plan(3.0, 0.2, 0.2, CFG, hint('BLOCKED'), rear=3.0).action == 'REVERSE'


def test_hint_breaks_a_tie_between_comparable_sides():
    """左右空间相当时，方向偏好才允许起作用——这是它唯一的用武之地。"""
    sides = 1.0
    assert plan(0.4, sides, sides, CFG, hint(direction='RIGHT')).action == 'TURN_RIGHT'
    assert plan(0.4, sides, sides, CFG, hint(direction='LEFT')).action == 'TURN_LEFT'


def test_hint_cannot_override_a_clearly_more_open_side():
    """差距明显时必须听雷达，否则视觉就能把机器人指向墙。"""
    result = plan(0.4, 2.5, 0.45, CFG, hint(direction='RIGHT'))
    assert result.action == 'TURN_LEFT'


def test_hint_never_raises_speed():
    """遍历提示的所有取值组合，速度都不得超过无提示时的基线。

    这是"模型只能让机器人更保守"这条不变式最直接的证明。
    """
    baseline = plan(3.0, 3.0, 3.0, CFG)
    for hazard in ['NONE', 'CAUTION', 'BLOCKED']:
        for direction in ['LEFT', 'RIGHT', 'STRAIGHT', 'NONE']:
            result = plan(3.0, 3.0, 3.0, CFG, hint(hazard, direction))
            assert result.linear_x <= baseline.linear_x + 1e-9
            assert abs(result.angular_z) <= CFG.turn_speed + 1e-9


def test_missing_hint_behaves_like_a_clean_assessment():
    """模型缺席与模型报"一切正常"必须完全等价，纯激光才算真正的降级路径。"""
    assert plan(3.0, 3.0, 3.0, CFG, None) == plan(3.0, 3.0, 3.0, CFG, hint())


def test_dead_zone_holds_last_turn_despite_noise_flip():
    """纯激光模式下，死区内测量噪声不应让转向方向来回切换。"""
    left = plan(0.4, 1.01, 1.00, CFG, last_turn='TURN_LEFT')
    assert left.action == 'TURN_LEFT'

    right = plan(0.4, 1.00, 1.01, CFG, last_turn='TURN_RIGHT')
    assert right.action == 'TURN_RIGHT'


def test_dead_zone_without_history_defaults_left():
    """首次进入死区且无视觉偏好时，确定性默认左转。"""
    result = plan(0.4, 1.0, 1.0, CFG)
    assert result.action == 'TURN_LEFT'
    assert result.angular_z > 0.0


def test_clear_winner_overrides_last_turn():
    """明显更优的一侧始终优先，不受方向保持影响。"""
    result = plan(0.4, 2.0, 0.5, CFG, last_turn='TURN_RIGHT')
    assert result.action == 'TURN_LEFT'


def test_vision_still_breaks_tie_in_dead_zone():
    """死区内视觉偏好仍可打破平局，且不能覆盖明显更优侧（见下一测试）。"""
    assert plan(0.4, 1.0, 1.0, CFG, hint(direction='RIGHT')).action == 'TURN_RIGHT'
    assert plan(0.4, 1.0, 1.0, CFG, hint(direction='LEFT')).action == 'TURN_LEFT'


def test_clearance_hysteresis_holds_forward_in_band():
    """迟滞带内保持直行，避免 0.59m/0.60m 在 FORWARD 与 TURN 之间抖动。"""
    result = plan(0.59, 2.0, 0.5, CFG, last_action='FORWARD')
    assert result.action == 'FORWARD'
    assert 'hysteresis' in result.reason


def test_clearance_hysteresis_holds_turn_in_band():
    """迟滞带内保持转向，直到前方真正开阔。"""
    result = plan(0.59, 2.0, 0.5, CFG, last_action='TURN_LEFT')
    assert result.action == 'TURN_LEFT'


def test_clearance_above_band_always_forwards():
    assert plan(0.61, 0.4, 0.4, CFG, last_action='TURN_LEFT').action == 'FORWARD'


def test_clearance_below_band_always_turns():
    assert plan(0.49, 2.0, 0.5, CFG, last_action='FORWARD').action == 'TURN_LEFT'


def test_near_wall_locks_turn_direction():
    """贴近墙壁时扩大转向死区，避免左右小幅噪声导致来回摆。"""
    left = plan(0.30, 1.05, 1.00, CFG, last_turn='TURN_LEFT')
    assert left.action == 'TURN_LEFT'

    right = plan(0.30, 1.00, 1.05, CFG, last_turn='TURN_RIGHT')
    assert right.action == 'TURN_RIGHT'


def test_long_turn_commits_to_the_latched_side():
    """转了很久之后，方向锁死，连"另一侧明显更开阔"也不再改变它。

    这一条针对的正是贴墙摆头：只要还允许换边，两侧读数在转动过程中不断变化，
    机器人就会一直改主意。转过 escape_commit_time 之后，把"选边"这件事整个关掉。
    """
    elapsed = CFG.escape_commit_time + 0.5
    result = plan(0.30, 0.4, 3.0, CFG, last_turn='TURN_LEFT', turn_elapsed=elapsed)
    assert result.action == 'TURN_LEFT'
    assert result.angular_z > 0.0
    assert 'committed' in result.reason


def test_commit_needs_a_latched_side_to_act_on():
    """没有历史方向时，锁定无从生效，仍按左右比较来选边。"""
    result = plan(0.30, 0.4, 3.0, CFG, turn_elapsed=CFG.escape_commit_time + 0.5)
    assert result.action == 'TURN_RIGHT'


def test_stuck_turning_backs_out_while_turning():
    """转到 escape_reverse_time 还没脱困，说明车头已经顶住了，必须退出来。

    纯原地转向对差速底盘几乎总能脱困，唯一的例外是车头已经贴在障碍上、轮子只在
    空转。此时给一个很小的负线速度把车头拽离障碍，角速度保持不变。
    """
    result = plan(0.30, 0.4, 0.4, CFG,
                  last_turn='TURN_LEFT', turn_elapsed=CFG.escape_reverse_time + 0.5)
    assert result.action == 'ESCAPE_LEFT'
    assert result.linear_x == pytest.approx(-CFG.reverse_speed)
    assert result.angular_z > 0.0


def test_escape_action_keeps_the_direction_sign_invariant():
    """ESCAPE_* 同样遵守"动作名与角速度符号一致"这条不变式。"""
    elapsed = CFG.escape_reverse_time + 0.5
    left = plan(0.30, 3.0, 0.4, CFG, last_turn='TURN_LEFT', turn_elapsed=elapsed)
    right = plan(0.30, 0.4, 3.0, CFG, last_turn='TURN_RIGHT', turn_elapsed=elapsed)
    assert (left.action, left.angular_z > 0.0) == ('ESCAPE_LEFT', True)
    assert (right.action, right.angular_z < 0.0) == ('ESCAPE_RIGHT', True)


def test_escape_history_is_interchangeable_with_turn_history():
    """ESCAPE_* 也要能当作"上次往哪边转"，否则脱困动作自己会打断方向锁存。"""
    result = plan(0.30, 1.00, 1.05, CFG, last_turn='ESCAPE_LEFT')
    assert result.action == 'TURN_LEFT'

    held = plan(0.59, 1.0, 1.0, CFG, last_action='ESCAPE_RIGHT')
    assert held.action == 'TURN_RIGHT'          # 迟滞带内不得跳回 FORWARD


def test_escape_stays_off_until_the_timers_expire():
    """计时未到时行为与改动前完全一致，脱困逻辑不影响正常避障。"""
    assert plan(0.30, 3.0, 0.4, CFG, last_turn='TURN_RIGHT').action == 'TURN_LEFT'
    assert plan(0.30, 3.0, 0.4, CFG, turn_elapsed=CFG.escape_commit_time - 0.1).linear_x \
        == pytest.approx(0.0)


def test_escape_timers_can_be_disabled():
    """两个计时器置 0 即完全关闭，实车调试初期可以先要求纯原地转向。"""
    cfg = CFG._replace(escape_commit_time=0.0, escape_reverse_time=0.0)
    result = plan(0.30, 0.4, 3.0, cfg, last_turn='TURN_LEFT', turn_elapsed=60.0)
    assert result.action == 'TURN_RIGHT'
    assert result.linear_x == pytest.approx(0.0)
