"""扇区提取必须对任意雷达几何成立，而不只是 360 线 1 度间隔那一种。

这里刻意用三种雷达布局做同样的方向断言：TurtleBot3（360 线、从正前方起算）、
高分辨率雷达（1440 线）、以及 angle_min 为 -pi 的 RPLidar 风格布局。如果哪天
有人把"度数当下标"写回代码里，这几条会立刻失败。

另一组断言区分 inf（量程内空旷）与 None（盲区）——两者对导航含义相反。

不需要 ROS 图，只用到 LaserScan 消息类型。
"""

import math

import pytest
from sensor_msgs.msg import LaserScan

from ai_robot_nav.scan_utils import describe_environment, format_distance, sector_min_distance


def make_scan(ranges, angle_min=0.0, angle_increment=None, range_min=0.12, range_max=3.5):
    """构造一个从 angle_min 起算、采样点铺满一整圈的 LaserScan。

    不传 angle_increment 时按"点数均分 2pi"自动推算，这样改点数就等于改分辨率。
    """
    scan = LaserScan()
    scan.angle_min = angle_min
    scan.angle_increment = (
        2.0 * math.pi / len(ranges) if angle_increment is None else angle_increment)
    scan.angle_max = angle_min + scan.angle_increment * (len(ranges) - 1)
    scan.range_min = range_min
    scan.range_max = range_max
    scan.ranges = [float(value) for value in ranges]
    return scan


def turtlebot_scan(fill=3.0):
    """360 个采样点、间隔 1 度、从正前方起算——TurtleBot3 的布局。"""
    return make_scan([fill] * 360)


def test_front_sector_spans_the_zero_degree_wrap():
    """前方扇区跨越序列首尾接缝：350 度与 10 度都必须算在前方。"""
    scan = turtlebot_scan()
    scan.ranges[350] = 0.5   # -10 度
    scan.ranges[10] = 0.4    # +10 度
    assert sector_min_distance(scan, 0.0, 30.0) == pytest.approx(0.4)


def test_front_sector_ignores_targets_outside_it():
    """扇区外的近距离障碍不能污染读数，否则侧面的墙会让机器人永远不敢直行。"""
    scan = turtlebot_scan()
    scan.ranges[90] = 0.2
    scan.ranges[180] = 0.1
    assert sector_min_distance(scan, 0.0, 30.0) == pytest.approx(3.0)


def test_left_is_positive_and_right_is_negative():
    """REP-103 的角度符号约定：左正右负。搞反会让机器人朝错误的一侧转。"""
    scan = turtlebot_scan()
    scan.ranges[90] = 1.0     # +90 度
    scan.ranges[270] = 2.0    # -90 度
    assert sector_min_distance(scan, 90.0, 30.0) == pytest.approx(1.0)
    assert sector_min_distance(scan, -90.0, 30.0) == pytest.approx(2.0)


def test_high_resolution_scanner_reads_the_same_directions():
    """1440 线、0.25 度间隔必须解析出与 360 线相同的方位。"""
    scan = make_scan([3.0] * 1440)
    scan.ranges[4 * 90] = 1.0
    scan.ranges[4 * 270] = 2.0
    assert sector_min_distance(scan, 90.0, 30.0) == pytest.approx(1.0)
    assert sector_min_distance(scan, -90.0, 30.0) == pytest.approx(2.0)


def test_scanner_starting_at_negative_pi_reads_the_same_directions():
    """RPLidar 风格布局：angle_min 为 -pi，下标 0 指向正后方。"""
    scan = make_scan([3.0] * 360, angle_min=-math.pi)
    scan.ranges[180] = 0.5    # 0 度
    scan.ranges[270] = 1.0    # +90 度
    assert sector_min_distance(scan, 0.0, 10.0) == pytest.approx(0.5)
    assert sector_min_distance(scan, 90.0, 10.0) == pytest.approx(1.0)


def test_out_of_range_returns_read_as_clear_not_blind():
    """inf 表示量程内没有东西，属于空旷而不是故障。"""
    scan = make_scan([math.inf] * 360)
    assert sector_min_distance(scan, 0.0, 30.0) == pytest.approx(scan.range_max)


def test_all_invalid_returns_none():
    """NaN 与低于 range_min 的丢点都不可用，整个扇区因此是盲区。"""
    scan = make_scan([math.nan] * 360)
    assert sector_min_distance(scan, 0.0, 30.0) is None

    scan = make_scan([0.0] * 360)
    assert sector_min_distance(scan, 0.0, 30.0) is None


def test_readings_beyond_range_max_do_not_mask_a_real_obstacle():
    """一片 inf 中夹着一个真实障碍时，必须报出那个障碍，而不是报空旷。"""
    scan = turtlebot_scan(fill=math.inf)
    scan.ranges[5] = 0.8
    assert sector_min_distance(scan, 0.0, 30.0) == pytest.approx(0.8)


def test_empty_scan_is_blind():
    """空消息和 None 都要安全返回盲区，不能抛异常——调用方在控制回路里。"""
    assert sector_min_distance(make_scan([1.0]), 0.0, 30.0) is not None
    empty = LaserScan()
    assert sector_min_distance(empty, 0.0, 30.0) is None
    assert sector_min_distance(None, 0.0, 30.0) is None


def test_describe_environment_orders_front_left_right():
    """返回顺序固定为 (前, 左, 右)，调用方按位置解包。"""
    scan = turtlebot_scan()
    scan.ranges[0] = 0.6
    scan.ranges[90] = 1.2
    scan.ranges[270] = 2.4
    scan.ranges[180] = 1.0
    front, left, right, rear = describe_environment(scan, 30.0, 90.0, 30.0)
    assert front == pytest.approx(0.6)
    assert left == pytest.approx(1.2)
    assert right == pytest.approx(2.4)
    assert rear == pytest.approx(1.0)


def test_format_distance_spells_out_the_blind_case():
    """盲区必须在提示词里写成文字，不能伪装成一个数字距离。"""
    assert format_distance(1.234) == '1.23m'
    assert '未知' in format_distance(None)
