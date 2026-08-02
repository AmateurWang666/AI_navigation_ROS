"""模型输出属于不可信输入；解析必须"静默失败"，而不是抛给控制回路。

这里覆盖两类场景：一是小模型常见的格式毛病（代码块包裹、夹杂客套话），要能容忍；
二是语义上不合法的内容（未知 hazard、夹带速度字段），必须拒绝或降级。

不需要 ROS 图，也不需要 Ollama 服务。
"""

import math

import pytest

from ai_robot_nav.llm_client import (
    Assessment, DecisionError, apply_caution_debounce, extract_json, parse_assessment,
)
from ai_robot_nav.motion import clamp

GOOD_JSON = '{"hazard": "NONE", "preferred_direction": "STRAIGHT", "description": "clear"}'


def test_plain_json():
    assert extract_json(GOOD_JSON)['hazard'] == 'NONE'


def test_markdown_fenced_json():
    """即使要求只输出 JSON，小模型仍常把它裹在 markdown 代码块里。"""
    assert extract_json(f'```json\n{GOOD_JSON}\n```')['hazard'] == 'NONE'
    assert extract_json(f'```\n{GOOD_JSON}\n```')['hazard'] == 'NONE'


def test_json_wrapped_in_prose():
    """前后夹带的客套话不该让整次推理作废。"""
    text = f'Sure! Here is my assessment:\n{GOOD_JSON}\nHope that helps.'
    assert extract_json(text)['hazard'] == 'NONE'


def test_empty_and_garbage_are_rejected():
    """空串、纯文字、以及顶层为数组的合法 JSON，都取不出字段，必须拒绝。"""
    for text in ['', '   ', 'I cannot help with that.', '[1, 2, 3]']:
        with pytest.raises(DecisionError):
            extract_json(text)


def test_valid_assessment_round_trip():
    assessment = parse_assessment(GOOD_JSON)
    assert assessment.hazard == 'NONE'
    assert assessment.preferred_direction == 'STRAIGHT'
    assert assessment.description == 'clear'


def test_hazard_is_case_insensitive():
    """大小写不一致属于格式毛病而非语义错误，统一转成大写即可。"""
    assert parse_assessment('{"hazard": "caution"}').hazard == 'CAUTION'


def test_unknown_hazard_is_rejected():
    """hazard 直接决定是否降速或停车，非法值不能猜默认值，只能拒绝。"""
    for payload in ['{"hazard": "EXPLODE"}', '{"hazard": ""}', '{"description": "hi"}']:
        with pytest.raises(DecisionError):
            parse_assessment(payload)


def test_unknown_direction_degrades_to_none_instead_of_failing():
    """非法方向只损失一次平局裁决，不值得为它触发一次以秒计的重试。"""
    assert parse_assessment('{"hazard": "NONE", "preferred_direction": "NORTH"}') \
        .preferred_direction == 'NONE'
    assert parse_assessment('{"hazard": "NONE", "preferred_direction": 42}') \
        .preferred_direction == 'NONE'


def test_missing_direction_and_description_default():
    """只给 hazard 是合法的最小输出，其余字段走默认值。"""
    assessment = parse_assessment('{"hazard": "BLOCKED"}')
    assert assessment.preferred_direction == 'NONE'
    assert assessment.description == ''


def test_model_cannot_smuggle_a_velocity_through():
    """速度是 navigator.py 的职责；模型多写的数值字段一律进不来。"""
    assessment = parse_assessment(
        '{"hazard": "NONE", "linear_x": 99.0, "angular_z": -99.0}')
    assert not hasattr(assessment, 'linear_x')
    assert assessment._fields == ('hazard', 'preferred_direction', 'description')


def test_clamp_is_symmetric_and_sign_preserving():
    """限幅要对称、保号，且负的上限（配置写错）不会让区间反转。"""
    assert clamp(5.0, 2.0) == pytest.approx(2.0)
    assert clamp(-5.0, 2.0) == pytest.approx(-2.0)
    assert clamp(1.0, 2.0) == pytest.approx(1.0)
    assert clamp(-3.0, -2.0) == pytest.approx(-2.0)
    assert not math.isnan(clamp(0.0, 0.0))


def test_single_caution_is_debounced():
    assessment = Assessment('CAUTION', 'NONE', 'dim hallway')
    effective, streak = apply_caution_debounce(assessment, 0, 2)
    assert effective.hazard == 'NONE'
    assert streak == 1


def test_consecutive_caution_applies_after_threshold():
    first = Assessment('CAUTION', 'NONE', 'dim hallway')
    second = Assessment('CAUTION', 'NONE', 'dim hallway')
    mid, streak = apply_caution_debounce(first, 0, 2)
    final, streak = apply_caution_debounce(second, streak, 2)
    assert mid.hazard == 'NONE'
    assert final.hazard == 'CAUTION'
    assert streak == 2


def test_none_resets_caution_streak():
    caution = Assessment('CAUTION', 'NONE', 'dim hallway')
    none = Assessment('NONE', 'NONE', 'clear')
    _, streak = apply_caution_debounce(caution, 0, 2)
    effective, streak = apply_caution_debounce(none, streak, 2)
    assert effective.hazard == 'NONE'
    assert streak == 0


def test_blocked_is_not_debounced():
    blocked = Assessment('BLOCKED', 'NONE', 'wall')
    effective, streak = apply_caution_debounce(blocked, 5, 2)
    assert effective.hazard == 'BLOCKED'
    assert streak == 0
