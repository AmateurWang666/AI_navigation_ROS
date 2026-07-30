"""Sector extraction must hold for any scanner geometry, not just 360x1deg."""

import math

import pytest
from sensor_msgs.msg import LaserScan

from ai_robot_nav.scan_utils import describe_environment, format_distance, sector_min_distance


def make_scan(ranges, angle_min=0.0, angle_increment=None, range_min=0.12, range_max=3.5):
    """Build a LaserScan whose samples span a full turn starting at angle_min."""
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
    """360 samples, 1 degree apart, starting straight ahead - the TurtleBot3 layout."""
    return make_scan([fill] * 360)


def test_front_sector_spans_the_zero_degree_wrap():
    scan = turtlebot_scan()
    scan.ranges[350] = 0.5   # -10 degrees
    scan.ranges[10] = 0.4    # +10 degrees
    assert sector_min_distance(scan, 0.0, 30.0) == pytest.approx(0.4)


def test_front_sector_ignores_targets_outside_it():
    scan = turtlebot_scan()
    scan.ranges[90] = 0.2
    scan.ranges[180] = 0.1
    assert sector_min_distance(scan, 0.0, 30.0) == pytest.approx(3.0)


def test_left_is_positive_and_right_is_negative():
    scan = turtlebot_scan()
    scan.ranges[90] = 1.0     # +90 degrees
    scan.ranges[270] = 2.0    # -90 degrees
    assert sector_min_distance(scan, 90.0, 30.0) == pytest.approx(1.0)
    assert sector_min_distance(scan, -90.0, 30.0) == pytest.approx(2.0)


def test_high_resolution_scanner_reads_the_same_directions():
    """1440 samples at 0.25deg must resolve the same bearings as the 360 case."""
    scan = make_scan([3.0] * 1440)
    scan.ranges[4 * 90] = 1.0
    scan.ranges[4 * 270] = 2.0
    assert sector_min_distance(scan, 90.0, 30.0) == pytest.approx(1.0)
    assert sector_min_distance(scan, -90.0, 30.0) == pytest.approx(2.0)


def test_scanner_starting_at_negative_pi_reads_the_same_directions():
    """RPLidar-style geometry: angle_min is -pi, so index 0 points backwards."""
    scan = make_scan([3.0] * 360, angle_min=-math.pi)
    scan.ranges[180] = 0.5    # 0 degrees
    scan.ranges[270] = 1.0    # +90 degrees
    assert sector_min_distance(scan, 0.0, 10.0) == pytest.approx(0.5)
    assert sector_min_distance(scan, 90.0, 10.0) == pytest.approx(1.0)


def test_out_of_range_returns_read_as_clear_not_blind():
    """inf means nothing within range_max, which is open space rather than a fault."""
    scan = make_scan([math.inf] * 360)
    assert sector_min_distance(scan, 0.0, 30.0) == pytest.approx(scan.range_max)


def test_all_invalid_returns_none():
    """NaN and below-range_min dropouts leave nothing usable, so the sector is blind."""
    scan = make_scan([math.nan] * 360)
    assert sector_min_distance(scan, 0.0, 30.0) is None

    scan = make_scan([0.0] * 360)
    assert sector_min_distance(scan, 0.0, 30.0) is None


def test_readings_beyond_range_max_do_not_mask_a_real_obstacle():
    scan = turtlebot_scan(fill=math.inf)
    scan.ranges[5] = 0.8
    assert sector_min_distance(scan, 0.0, 30.0) == pytest.approx(0.8)


def test_empty_scan_is_blind():
    assert sector_min_distance(make_scan([1.0]), 0.0, 30.0) is not None
    empty = LaserScan()
    assert sector_min_distance(empty, 0.0, 30.0) is None
    assert sector_min_distance(None, 0.0, 30.0) is None


def test_describe_environment_orders_front_left_right():
    scan = turtlebot_scan()
    scan.ranges[0] = 0.6
    scan.ranges[90] = 1.2
    scan.ranges[270] = 2.4
    front, left, right = describe_environment(scan, 30.0, 90.0, 30.0)
    assert front == pytest.approx(0.6)
    assert left == pytest.approx(1.2)
    assert right == pytest.approx(2.4)


def test_format_distance_spells_out_the_blind_case():
    assert format_distance(1.234) == '1.23m'
    assert '未知' in format_distance(None)
