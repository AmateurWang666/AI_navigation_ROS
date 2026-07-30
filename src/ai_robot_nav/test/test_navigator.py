"""The policy must be fully determined by LiDAR; vision may only add caution."""

import pytest

from ai_robot_nav.llm_client import Assessment
from ai_robot_nav.navigator import NavConfig, plan

CFG = NavConfig()


def hint(hazard='NONE', direction='NONE'):
    return Assessment(hazard=hazard, preferred_direction=direction, description='')


def test_clear_path_drives_forward():
    result = plan(3.0, 3.0, 3.0, CFG)
    assert result.action == 'FORWARD'
    assert result.linear_x == pytest.approx(CFG.cruise_speed)
    assert result.angular_z == pytest.approx(0.0)


def test_blocked_front_turns_toward_the_open_side():
    left = plan(0.4, 2.0, 0.5, CFG)
    assert left.action == 'TURN_LEFT'
    assert left.angular_z > 0.0          # positive is left under REP-103
    assert left.linear_x == pytest.approx(0.0)

    right = plan(0.4, 0.5, 2.0, CFG)
    assert right.action == 'TURN_RIGHT'
    assert right.angular_z < 0.0


def test_turn_sign_always_matches_the_action_name():
    """The failure mode observed from llava:7b is now structurally impossible."""
    for left_dist, right_dist in [(2.0, 0.5), (0.5, 2.0), (1.0, 1.0), (0.45, 0.44)]:
        result = plan(0.3, left_dist, right_dist, CFG)
        if result.action == 'TURN_LEFT':
            assert result.angular_z > 0.0
        elif result.action == 'TURN_RIGHT':
            assert result.angular_z < 0.0


def test_boxed_in_reverses():
    result = plan(0.2, 0.2, 0.2, CFG)
    assert result.action == 'REVERSE'
    assert result.linear_x < 0.0


def test_blind_sector_counts_as_blocked_not_open():
    assert plan(None, 3.0, 3.0, CFG).action != 'FORWARD'
    assert plan(None, None, None, CFG).action == 'REVERSE'


def test_caution_only_scales_speed_down():
    baseline = plan(3.0, 3.0, 3.0, CFG)
    cautious = plan(3.0, 3.0, 3.0, CFG, hint('CAUTION'))
    assert cautious.action == 'FORWARD'
    assert cautious.linear_x == pytest.approx(baseline.linear_x * CFG.caution_scale)
    assert cautious.linear_x < baseline.linear_x


def test_blocked_hazard_overrides_an_open_lidar_reading():
    """Vision can see glass and drop-offs that the scanner reports as clear."""
    result = plan(3.0, 3.0, 0.5, CFG, hint('BLOCKED'))
    assert result.action != 'FORWARD'
    assert result.linear_x <= 0.0


def test_blocked_hazard_with_no_way_out_reverses():
    assert plan(3.0, 0.2, 0.2, CFG, hint('BLOCKED')).action == 'REVERSE'


def test_hint_breaks_a_tie_between_comparable_sides():
    sides = 1.0
    assert plan(0.4, sides, sides, CFG, hint(direction='RIGHT')).action == 'TURN_RIGHT'
    assert plan(0.4, sides, sides, CFG, hint(direction='LEFT')).action == 'TURN_LEFT'


def test_hint_cannot_override_a_clearly_more_open_side():
    """A wide gap must beat the model's preference, so vision cannot steer into a wall."""
    result = plan(0.4, 2.5, 0.45, CFG, hint(direction='RIGHT'))
    assert result.action == 'TURN_LEFT'


def test_hint_never_raises_speed():
    baseline = plan(3.0, 3.0, 3.0, CFG)
    for hazard in ['NONE', 'CAUTION', 'BLOCKED']:
        for direction in ['LEFT', 'RIGHT', 'STRAIGHT', 'NONE']:
            result = plan(3.0, 3.0, 3.0, CFG, hint(hazard, direction))
            assert result.linear_x <= baseline.linear_x + 1e-9
            assert abs(result.angular_z) <= CFG.turn_speed + 1e-9


def test_missing_hint_behaves_like_a_clean_assessment():
    assert plan(3.0, 3.0, 3.0, CFG, None) == plan(3.0, 3.0, 3.0, CFG, hint())
