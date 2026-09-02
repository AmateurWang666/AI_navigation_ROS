"""目的地解析：名字与坐标两条输入路径必须归一成同一个位姿。

最关键的一条是「名字优先于坐标」：把目的地命名成 "3" 之类的数字字符串时，
仍应按名字取，而不是被静默当成坐标。静默走错分支会让机器人开到一个
完全无关的地方，且没有任何报错。

不需要 ROS 图。
"""

import pytest

from ai_robot_nav.goals import (
    Destination, GoalError, format_destinations, parse_destination,
    parse_destinations, resolve_destination,
)

TABLE = {
    '前台': Destination(2.5, 3.0, 0.0),
    '厨房': Destination(-1.0, 4.5, 1.57),
}


# ---------------------------------------------------------------- 单条解析
def test_two_element_list_defaults_yaw_to_zero():
    assert parse_destination([1.0, 2.0], 'ctx') == Destination(1.0, 2.0, 0.0)


def test_three_element_list_keeps_yaw():
    assert parse_destination([1.0, 2.0, 1.57], 'ctx').yaw == pytest.approx(1.57)


def test_mapping_form_is_accepted():
    assert parse_destination({'x': 1.0, 'y': 2.0, 'yaw': 0.5}, 'ctx') == \
        Destination(1.0, 2.0, 0.5)


def test_integers_are_accepted():
    assert parse_destination([1, 2], 'ctx') == Destination(1.0, 2.0, 0.0)


def test_wrong_length_is_rejected():
    for bad in ([1.0], [1.0, 2.0, 3.0, 4.0]):
        with pytest.raises(GoalError):
            parse_destination(bad, 'ctx')


def test_non_numeric_component_is_rejected():
    with pytest.raises(GoalError, match='期望数字'):
        parse_destination([1.0, 'left'], 'ctx')


def test_booleans_are_not_silently_treated_as_numbers():
    """bool 是 int 的子类，不显式排除会让 True 变成 1.0 被静默接受。"""
    with pytest.raises(GoalError, match='期望数字'):
        parse_destination([True, 2.0], 'ctx')


def test_infinite_and_nan_are_rejected():
    for bad in (float('inf'), float('nan')):
        with pytest.raises(GoalError, match='有限'):
            parse_destination([bad, 1.0], 'ctx')


# ---------------------------------------------------------------- 整表解析
def test_empty_table_is_valid():
    """只用坐标导航时不需要配任何名字。"""
    assert parse_destinations(None) == {}
    assert parse_destinations({}) == {}


def test_table_parses_every_entry():
    table = parse_destinations({'a': [1.0, 2.0], 'b': [3.0, 4.0, 0.5]})
    assert table['a'] == Destination(1.0, 2.0, 0.0)
    assert table['b'].yaw == pytest.approx(0.5)


def test_table_must_be_a_mapping():
    with pytest.raises(GoalError, match='映射'):
        parse_destinations([[1.0, 2.0]])


def test_bad_entry_names_the_offending_key():
    with pytest.raises(GoalError, match='厨房'):
        parse_destinations({'厨房': [1.0, 'x']})


# ---------------------------------------------------------------- 命令行解析
def test_single_argument_resolves_a_name():
    destination, source = resolve_destination(['前台'], TABLE)
    assert destination == TABLE['前台']
    assert '前台' in source


def test_two_arguments_resolve_coordinates():
    destination, source = resolve_destination(['2.5', '3.0'], TABLE)
    assert destination == Destination(2.5, 3.0, 0.0)
    assert '坐标' in source


def test_three_arguments_include_yaw():
    destination, _ = resolve_destination(['2.5', '3.0', '1.57'], TABLE)
    assert destination.yaw == pytest.approx(1.57)


def test_name_wins_over_coordinate_interpretation():
    """目的地命名成数字时仍按名字取，不能被静默当作坐标。"""
    table = {'3': Destination(9.0, 9.0, 0.0)}
    destination, source = resolve_destination(['3', '5'], table)
    assert destination == table['3']
    assert '3' in source


def test_negative_coordinates_are_accepted():
    destination, _ = resolve_destination(['-1.5', '-2.5'], TABLE)
    assert destination == Destination(-1.5, -2.5, 0.0)


def test_unknown_name_lists_the_available_ones():
    """报错必须告诉使用者有哪些可选值，否则只能去翻配置文件。"""
    with pytest.raises(GoalError) as excinfo:
        resolve_destination(['不存在的地方'], TABLE)
    assert '前台' in str(excinfo.value)
    assert '厨房' in str(excinfo.value)


def test_no_arguments_explains_the_usage():
    with pytest.raises(GoalError, match='用法'):
        resolve_destination([], TABLE)


def test_too_many_arguments_is_rejected():
    with pytest.raises(GoalError, match='参数过多'):
        resolve_destination(['1', '2', '3', '4'], TABLE)


def test_unparseable_arguments_are_rejected():
    with pytest.raises(GoalError):
        resolve_destination(['去', '厨房'], TABLE)


def test_format_destinations_handles_the_empty_table():
    assert '没有配置' in format_destinations({})
