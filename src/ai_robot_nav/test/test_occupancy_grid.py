"""占据栅格建图的核心不变式。

其中两条直接决定地图能不能用：

- **超量程的射线只清空、不标记障碍**。标上去等于凭空造一堵墙，
  而那堵墙会让 move_base 永远规划不出穿过该处的路径。
- **无效读数（NaN）整条丢弃**。当成超量程去清空，会把从未观测过的区域
  错误地标成空闲，机器人于是敢往一片其实一无所知的地方走。

不需要 ROS 图。
"""

import math

import pytest

from ai_robot_nav.occupancy_grid import (
    FREE, GridSpec, OCCUPIED, OccupancyGridMapper, PGM_FREE, PGM_OCCUPIED,
    PGM_UNKNOWN, UNKNOWN, bresenham, spec_covering, to_map_yaml, to_pgm_bytes,
)

# 20x20 格 @ 0.1 m = 2x2 m，原点在左下角 (-1, -1)，因此世界原点落在栅格正中。
SPEC = GridSpec(width=20, height=20, resolution=0.1, origin_x=-1.0, origin_y=-1.0)


def mapper():
    return OccupancyGridMapper(SPEC)


# ---------------------------------------------------------------- Bresenham
def test_line_includes_both_endpoints():
    cells = bresenham(0, 0, 3, 0)
    assert cells[0] == (0, 0)
    assert cells[-1] == (3, 0)


def test_single_point_line():
    assert bresenham(2, 2, 2, 2) == [(2, 2)]


def test_line_is_connected_without_gaps():
    """相邻格之间的步进不能超过 1，否则射线会从墙上的孔里漏过去。"""
    cells = bresenham(0, 0, 7, 3)
    for (x0, y0), (x1, y1) in zip(cells, cells[1:]):
        assert max(abs(x1 - x0), abs(y1 - y0)) == 1


def test_line_has_no_duplicate_cells():
    """重复格会让同一次观测在该格上被计入多次，人为放大占据几率。"""
    cells = bresenham(0, 0, 9, 4)
    assert len(cells) == len(set(cells))


@pytest.mark.parametrize('target', [(5, 2), (-5, 2), (5, -2), (-5, -2),
                                    (2, 5), (0, 4), (4, 0)])
def test_line_reaches_the_target_in_every_direction(target):
    assert bresenham(0, 0, *target)[-1] == target


# ---------------------------------------------------------------- 坐标换算
def test_world_to_grid_and_back_round_trips():
    grid = mapper()
    cell = grid.world_to_grid(0.05, 0.05)
    assert cell is not None
    x, y = grid.grid_to_world(*cell)
    assert grid.world_to_grid(x, y) == cell


def test_points_outside_the_map_return_none():
    grid = mapper()
    for point in [(-2.0, 0.0), (2.0, 0.0), (0.0, -2.0), (0.0, 2.0)]:
        assert grid.world_to_grid(*point) is None


def test_origin_corner_maps_to_the_first_cell():
    assert mapper().world_to_grid(-1.0, -1.0) == (0, 0)


# ---------------------------------------------------------------- 射线融合
def test_hit_marks_the_endpoint_occupied():
    grid = mapper()
    grid.integrate_ray(0.0, 0.0, 0.5, 0.0, hit=True)
    cell = grid.world_to_grid(0.5, 0.0)
    assert grid.log_odds_at(*cell) > 0.0


def test_hit_clears_the_cells_along_the_way():
    grid = mapper()
    grid.integrate_ray(0.0, 0.0, 0.5, 0.0, hit=True)
    cell = grid.world_to_grid(0.25, 0.0)
    assert grid.log_odds_at(*cell) < 0.0


def test_out_of_range_ray_never_marks_an_obstacle():
    """超量程只说明「这条线上量程内没东西」，末端并未观测到障碍。"""
    grid = mapper()
    grid.integrate_ray(0.0, 0.0, 0.5, 0.0, hit=False)
    cell = grid.world_to_grid(0.5, 0.0)
    assert grid.log_odds_at(*cell) < 0.0


def test_repeated_observations_accumulate_confidence():
    grid = mapper()
    cell = grid.world_to_grid(0.5, 0.0)
    grid.integrate_ray(0.0, 0.0, 0.5, 0.0, hit=True)
    once = grid.log_odds_at(*cell)
    grid.integrate_ray(0.0, 0.0, 0.5, 0.0, hit=True)
    assert grid.log_odds_at(*cell) > once


def test_confidence_is_capped_so_the_map_stays_correctable():
    """不封顶的话，被观测上千次的墙搬走后需要同样多次反向观测才能翻转。"""
    grid = mapper()
    cell = grid.world_to_grid(0.5, 0.0)
    for _ in range(200):
        grid.integrate_ray(0.0, 0.0, 0.5, 0.0, hit=True)
    assert grid.log_odds_at(*cell) <= grid.l_max


def test_a_removed_obstacle_can_be_cleared_again():
    grid = mapper()
    cell = grid.world_to_grid(0.5, 0.0)
    for _ in range(50):
        grid.integrate_ray(0.0, 0.0, 0.5, 0.0, hit=True)
    for _ in range(50):
        grid.integrate_ray(0.0, 0.0, 0.9, 0.0, hit=False)
    assert grid.log_odds_at(*cell) < 0.0


def test_sensor_outside_the_map_is_ignored():
    grid = mapper()
    grid.integrate_ray(99.0, 99.0, 100.0, 99.0, hit=True)
    assert grid.coverage() == pytest.approx(0.0)


# ---------------------------------------------------------------- 整帧融合
def test_nan_beams_are_dropped_entirely():
    """NaN 既不是障碍也不是空旷，当成超量程会把未知区域错标为空闲。"""
    grid = mapper()
    grid.integrate_scan(0.0, 0.0, 0.0, [(0.0, float('nan'))], 0.12, 1.0)
    assert grid.coverage() == pytest.approx(0.0)


def test_beams_below_min_range_are_dropped():
    grid = mapper()
    grid.integrate_scan(0.0, 0.0, 0.0, [(0.0, 0.01)], 0.12, 1.0)
    assert grid.coverage() == pytest.approx(0.0)


def test_infinite_beams_clear_up_to_max_range():
    grid = mapper()
    grid.integrate_scan(0.0, 0.0, 0.0, [(0.0, float('inf'))], 0.12, 0.5)
    cell = grid.world_to_grid(0.3, 0.0)
    assert grid.log_odds_at(*cell) < 0.0


def test_sensor_yaw_rotates_the_beam():
    """扫描角要叠加传感器朝向，否则整帧观测会绕原点整体转错角度。"""
    grid = mapper()
    grid.integrate_scan(0.0, 0.0, math.pi / 2.0, [(0.0, 0.5)], 0.12, 1.0)
    assert grid.log_odds_at(*grid.world_to_grid(0.0, 0.5)) > 0.0
    assert grid.log_odds_at(*grid.world_to_grid(0.5, 0.0)) == pytest.approx(0.0)


# ---------------------------------------------------------------- 导出
def test_untouched_cells_export_as_unknown():
    assert set(mapper().to_occupancy_data()) == {UNKNOWN}


def test_export_uses_ros_occupancy_values():
    grid = mapper()
    grid.integrate_ray(0.0, 0.0, 0.5, 0.0, hit=True)
    data = grid.to_occupancy_data()
    assert data[grid._index(*grid.world_to_grid(0.5, 0.0))] == OCCUPIED
    assert data[grid._index(*grid.world_to_grid(0.25, 0.0))] == FREE
    assert set(data) <= {UNKNOWN, FREE, OCCUPIED}


def test_export_length_matches_the_grid_size():
    assert len(mapper().to_occupancy_data()) == SPEC.size


# ---------------------------------------------------------------- 尺寸推算
def test_spec_covering_includes_the_margin():
    spec = spec_covering(0.0, 0.0, 1.0, 1.0, resolution=0.1, margin=0.5)
    assert spec.origin_x == pytest.approx(-0.5)
    assert spec.width * spec.resolution >= 2.0


def test_spec_covering_rejects_invalid_input():
    with pytest.raises(ValueError):
        spec_covering(0.0, 0.0, 1.0, 1.0, resolution=0.0)
    with pytest.raises(ValueError):
        spec_covering(1.0, 0.0, 0.0, 1.0, resolution=0.1)


# ---------------------------------------------------------------- 存盘格式
def test_pgm_has_a_valid_p5_header():
    blob = to_pgm_bytes([UNKNOWN] * 6, 3, 2)
    assert blob.startswith(b'P5\n3 2\n255\n')
    assert len(blob) == len(b'P5\n3 2\n255\n') + 6


def test_pgm_maps_each_state_to_the_expected_grey():
    blob = to_pgm_bytes([FREE, OCCUPIED, UNKNOWN], 3, 1)
    assert blob[-3:] == bytes([PGM_FREE, PGM_OCCUPIED, PGM_UNKNOWN])


def test_pgm_flips_rows_vertically():
    """OccupancyGrid 第 0 行在底部，PGM 第 0 行在顶部，必须翻转。

    不翻转的话地图上下镜像。它看起来仍然像一张地图、不报任何错，
    但机器人会把左右完全走反——属于最难发现的一类错误。
    """
    # 底行全占据，顶行全空闲。
    data = [OCCUPIED, OCCUPIED, FREE, FREE]
    pixels = to_pgm_bytes(data, width=2, height=2)[-4:]
    # PGM 第一行应当是「地图的顶行」，也就是 FREE。
    assert pixels[:2] == bytes([PGM_FREE, PGM_FREE])
    assert pixels[2:] == bytes([PGM_OCCUPIED, PGM_OCCUPIED])


def test_pgm_rejects_mismatched_dimensions():
    with pytest.raises(ValueError, match='不符'):
        to_pgm_bytes([FREE] * 5, 3, 2)


def test_pgm_rejects_invalid_size():
    with pytest.raises(ValueError, match='尺寸无效'):
        to_pgm_bytes([], 0, 0)


def test_map_yaml_contains_every_required_field():
    text = to_map_yaml('map.pgm', SPEC)
    for key in ('image', 'resolution', 'origin', 'negate',
                'occupied_thresh', 'free_thresh'):
        assert key in text


def test_map_yaml_uses_a_relative_image_name():
    """写相对文件名，整个地图目录才能随意搬动或提交进仓库。"""
    assert 'image: map.pgm' in to_map_yaml('map.pgm', SPEC)


def test_map_yaml_round_trips_through_the_map_loader(tmp_path):
    """自己生成的地图必须能通过自己的校验器——两端在此对齐。"""
    from ai_robot_nav.map_utils import load_map_info

    (tmp_path / 'm.pgm').write_bytes(to_pgm_bytes([UNKNOWN] * SPEC.size,
                                                  SPEC.width, SPEC.height))
    (tmp_path / 'm.yaml').write_text(to_map_yaml('m.pgm', SPEC), encoding='utf-8')

    info = load_map_info(str(tmp_path / 'm.yaml'))
    assert info.resolution == pytest.approx(SPEC.resolution)
    assert info.origin[0] == pytest.approx(SPEC.origin_x)
    assert info.origin[1] == pytest.approx(SPEC.origin_y)
