"""Model output is untrusted input; parsing must fail closed, never loudly."""

import math

import pytest

from ai_robot_nav.llm_client import DecisionError, extract_json, parse_assessment
from ai_robot_nav.motion import clamp

GOOD_JSON = '{"hazard": "NONE", "preferred_direction": "STRAIGHT", "description": "clear"}'


def test_plain_json():
    assert extract_json(GOOD_JSON)['hazard'] == 'NONE'


def test_markdown_fenced_json():
    assert extract_json(f'```json\n{GOOD_JSON}\n```')['hazard'] == 'NONE'
    assert extract_json(f'```\n{GOOD_JSON}\n```')['hazard'] == 'NONE'


def test_json_wrapped_in_prose():
    text = f'Sure! Here is my assessment:\n{GOOD_JSON}\nHope that helps.'
    assert extract_json(text)['hazard'] == 'NONE'


def test_empty_and_garbage_are_rejected():
    for text in ['', '   ', 'I cannot help with that.', '[1, 2, 3]']:
        with pytest.raises(DecisionError):
            extract_json(text)


def test_valid_assessment_round_trip():
    assessment = parse_assessment(GOOD_JSON)
    assert assessment.hazard == 'NONE'
    assert assessment.preferred_direction == 'STRAIGHT'
    assert assessment.description == 'clear'


def test_hazard_is_case_insensitive():
    assert parse_assessment('{"hazard": "caution"}').hazard == 'CAUTION'


def test_unknown_hazard_is_rejected():
    for payload in ['{"hazard": "EXPLODE"}', '{"hazard": ""}', '{"description": "hi"}']:
        with pytest.raises(DecisionError):
            parse_assessment(payload)


def test_unknown_direction_degrades_to_none_instead_of_failing():
    """A bad direction only costs a tie-break, so it must not trigger a retry."""
    assert parse_assessment('{"hazard": "NONE", "preferred_direction": "NORTH"}') \
        .preferred_direction == 'NONE'
    assert parse_assessment('{"hazard": "NONE", "preferred_direction": 42}') \
        .preferred_direction == 'NONE'


def test_missing_direction_and_description_default():
    assessment = parse_assessment('{"hazard": "BLOCKED"}')
    assert assessment.preferred_direction == 'NONE'
    assert assessment.description == ''


def test_model_cannot_smuggle_a_velocity_through():
    """Velocities are navigator.py's job; nothing numeric survives parsing."""
    assessment = parse_assessment(
        '{"hazard": "NONE", "linear_x": 99.0, "angular_z": -99.0}')
    assert not hasattr(assessment, 'linear_x')
    assert assessment._fields == ('hazard', 'preferred_direction', 'description')


def test_clamp_is_symmetric_and_sign_preserving():
    assert clamp(5.0, 2.0) == pytest.approx(2.0)
    assert clamp(-5.0, 2.0) == pytest.approx(-2.0)
    assert clamp(1.0, 2.0) == pytest.approx(1.0)
    assert clamp(-3.0, -2.0) == pytest.approx(-2.0)
    assert not math.isnan(clamp(0.0, 0.0))
