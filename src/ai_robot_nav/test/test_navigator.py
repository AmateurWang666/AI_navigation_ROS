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


def test_boxed_in_reverses():
    """三面都贴近障碍时只能后退——转向出不去。"""
    result = plan(0.2, 0.2, 0.2, CFG)
    assert result.action == 'REVERSE'
    assert result.linear_x < 0.0


def test_blind_sector_counts_as_blocked_not_open():
    """None 表示该方向没有有效回波，必须按不可通行处理，不能按空旷处理。"""
    assert plan(None, 3.0, 3.0, CFG).action != 'FORWARD'
    assert plan(None, None, None, CFG).action == 'REVERSE'


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
    assert plan(3.0, 0.2, 0.2, CFG, hint('BLOCKED')).action == 'REVERSE'


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
