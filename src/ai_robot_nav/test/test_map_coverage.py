"""地图覆盖率与连通性校验。

核心不变式：allow_unknown=false 时，全局规划器只在 FREE 格上寻路。
若起点到目的地之间隔着 UNKNOWN，规划必然失败——这不是 bug，是安全策略。
map_manager 在启动时应把这一点说清楚，而不是让 move_base 静默 abort。

不需要 ROS 图。
"""

import pytest

from ai_robot_nav.map_utils import (
    FREE, OCCUPIED, UNKNOWN, MapError, analyze_coverage, check_reachability,
    format_navigation_readiness, load_map_info, world_to_cell,
)
from ai_robot_nav.occupancy_grid import (
    FREE as OG_FREE, OCCUPIED as OG_OCCUPIED, UNKNOWN as OG_UNKNOWN,
    to_pgm_bytes,
)

ORIGIN = [-1.0, -1.0, 0.0]
RES = 0.5   # 粗栅格，测试用小图


def write_grid_map(tmp_path, width, height, states, name='map'):
    """states 按 OccupancyGrid 行序（row 0 = origin 侧）。"""
    data = []
    for state in states:
        if state == FREE:
            data.append(OG_FREE)
        elif state == OCCUPIED:
            data.append(OG_OCCUPIED)
        else:
            data.append(OG_UNKNOWN)
    pgm_path = tmp_path / f'{name}.pgm'
    pgm_path.write_bytes(to_pgm_bytes(data, width, height))
    yaml_path = tmp_path / f'{name}.yaml'
    yaml_path.write_text(
        f'image: {name}.pgm\n'
        f'resolution: {RES}\n'
        f'origin: {ORIGIN}\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.196\n',
        encoding='utf-8',
    )
    return load_map_info(str(yaml_path))


def test_analyze_coverage_counts_each_state(tmp_path):
    # 2x2: 空闲、占据、未知、空闲
    info = write_grid_map(tmp_path, 2, 2,
                          [FREE, OCCUPIED, UNKNOWN, FREE])
    cov = analyze_coverage(info)
    assert cov.free == 2
    assert cov.occupied == 1
    assert cov.unknown == 1
    assert cov.known_ratio == pytest.approx(0.75)


def test_reachable_through_free_corridor(tmp_path):
    # 3x1 全空闲，起点左，终点右
    info = write_grid_map(tmp_path, 3, 1, [FREE, FREE, FREE])
    report = check_reachability(
        info, spawn_xy=(-0.75, -0.75),
        destinations={'goal': (0.75, -0.75)},
        allow_unknown=False)
    assert report.unreachable == []


def test_unreachable_when_unknown_blocks_path(tmp_path):
    # [FREE, UNKNOWN, FREE] —— 中间未知，allow_unknown=false 时不可达
    info = write_grid_map(tmp_path, 3, 1, [FREE, UNKNOWN, FREE])
    report = check_reachability(
        info, spawn_xy=(-0.75, -0.75),
        destinations={'far': (0.25, -0.75)},   # 右端空闲格，中间隔着 UNKNOWN
        allow_unknown=False)
    assert 'far' in report.unreachable


def test_unknown_becomes_traversable_when_allowed(tmp_path):
    info = write_grid_map(tmp_path, 3, 1, [FREE, UNKNOWN, FREE])
    report = check_reachability(
        info, spawn_xy=(-0.75, -0.75),
        destinations={'far': (0.25, -0.75)},
        allow_unknown=True)
    assert report.unreachable == []


def test_destination_on_obstacle_is_blocked(tmp_path):
    info = write_grid_map(tmp_path, 2, 1, [FREE, OCCUPIED])
    report = check_reachability(
        info, spawn_xy=(-0.75, -0.75),
        destinations={'wall': (0.25, -0.75)},
        allow_unknown=False)
    assert 'wall' in report.blocked


def test_readiness_report_warns_about_unreachable_destinations(tmp_path):
    info = write_grid_map(tmp_path, 3, 1, [FREE, UNKNOWN, FREE])
    text = format_navigation_readiness(
        info, spawn_xy=(-0.75, -0.75),
        destinations={'hall': (0.25, -0.75)},
        allow_unknown=False)
    assert 'hall' in text
    assert 'allow_unknown=False' in text
    assert '补建地图' in text


def test_readiness_report_does_not_warn_when_reachable(tmp_path):
    info = write_grid_map(tmp_path, 3, 1, [FREE, FREE, FREE])
    text = format_navigation_readiness(
        info, spawn_xy=(-0.75, -0.75),
        destinations={'near': (0.25, -0.75)},
        allow_unknown=False)
    assert '!!!' not in text


def test_invalid_pgm_is_rejected(tmp_path):
    (tmp_path / 'bad.yaml').write_text(
        'image: bad.pgm\nresolution: 0.05\norigin: [0,0,0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
        encoding='utf-8')
    (tmp_path / 'bad.pgm').write_bytes(b'not a pgm')
    info = load_map_info(str(tmp_path / 'bad.yaml'))
    with pytest.raises(MapError):
        analyze_coverage(info)
