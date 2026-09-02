"""用户地图的校验：坏地图必须在启动前就被挡下，并说清该改哪里。

这组测试的意义在于「地图错误的代价很高」——map_server 遇到坏地图时给的是
SDL 底层错误或直接 exit(-1)，而下游的 amcl、move_base 会把这种静默失败表现成
「机器人莫名不动」或「定位一直发散」，排查要绕很大一圈。

不需要 ROS 图。
"""

import pytest
import yaml

from ai_robot_nav.map_utils import MapError, load_map_info, resolve_image_path

VALID = {
    'image': 'map.pgm',
    'resolution': 0.05,
    'origin': [-10.0, -10.0, 0.0],
    'negate': 0,
    'occupied_thresh': 0.65,
    'free_thresh': 0.196,
}


def write_map(tmp_path, data=None, image_name='map.pgm', create_image=True):
    """在临时目录里造一张地图，返回 yaml 路径。"""
    payload = dict(VALID if data is None else data)
    yaml_path = tmp_path / 'map.yaml'
    yaml_path.write_text(yaml.safe_dump(payload), encoding='utf-8')
    if create_image:
        (tmp_path / image_name).write_bytes(b'P5\n1 1\n255\n\x00')
    return str(yaml_path)


# ---------------------------------------------------------------- 正常路径
def test_valid_map_loads(tmp_path):
    info = load_map_info(write_map(tmp_path))
    assert info.resolution == pytest.approx(0.05)
    assert info.origin == [-10.0, -10.0, 0.0]
    assert info.negate == 0
    assert info.image_path.endswith('map.pgm')


def test_negate_defaults_to_zero_when_absent(tmp_path):
    """negate 在 map_server 里有默认值，缺失不应视为错误。"""
    data = dict(VALID)
    del data['negate']
    assert load_map_info(write_map(tmp_path, data)).negate == 0


def test_origin_with_extra_components_is_accepted(tmp_path):
    """有些工具会写出 6 个分量的 origin，只取前三个即可。"""
    data = dict(VALID, origin=[1.0, 2.0, 0.5, 0.0, 0.0, 0.0])
    assert load_map_info(write_map(tmp_path, data)).origin == [1.0, 2.0, 0.5]


# ---------------------------------------------------------------- 路径解析
def test_relative_image_resolves_against_the_yaml_directory(tmp_path):
    """相对路径按 YAML 所在目录解析，而不是当前工作目录。

    这一点必须与 map_server 的规则一致，否则「从不同目录启动结果不同」，
    表现为昨天能跑今天不能跑。
    """
    resolved = resolve_image_path('map.pgm', str(tmp_path / 'map.yaml'))
    assert resolved == str(tmp_path / 'map.pgm')


def test_absolute_image_path_is_kept(tmp_path):
    absolute = str(tmp_path / 'elsewhere.pgm')
    assert resolve_image_path(absolute, str(tmp_path / 'map.yaml')) == absolute


# ---------------------------------------------------------------- 拒绝坏地图
def test_missing_yaml_is_rejected(tmp_path):
    with pytest.raises(MapError, match='不存在'):
        load_map_info(str(tmp_path / 'nope.yaml'))


def test_empty_path_is_rejected():
    with pytest.raises(MapError, match='未指定地图'):
        load_map_info('')


def test_missing_image_file_is_rejected(tmp_path):
    with pytest.raises(MapError, match='图片文件不存在'):
        load_map_info(write_map(tmp_path, create_image=False))


@pytest.mark.parametrize('key', ['image', 'resolution', 'origin',
                                 'occupied_thresh', 'free_thresh'])
def test_each_required_key_is_enforced(tmp_path, key):
    data = dict(VALID)
    del data[key]
    with pytest.raises(MapError, match=key):
        load_map_info(write_map(tmp_path, data))


def test_non_positive_resolution_is_rejected(tmp_path):
    for bad in (0.0, -0.05):
        with pytest.raises(MapError, match='resolution'):
            load_map_info(write_map(tmp_path, dict(VALID, resolution=bad)))


def test_thresholds_must_not_be_inverted(tmp_path):
    """free_thresh 高于 occupied_thresh 会让空闲与占据的判定区间重叠。

    map_server 不会报错，但栅格语义已经错了，最终表现为规划穿墙
    或整张图不可通行——属于最难定位的那类问题。
    """
    data = dict(VALID, free_thresh=0.9, occupied_thresh=0.65)
    with pytest.raises(MapError, match='free_thresh'):
        load_map_info(write_map(tmp_path, data))


def test_origin_must_have_three_components(tmp_path):
    with pytest.raises(MapError, match='origin'):
        load_map_info(write_map(tmp_path, dict(VALID, origin=[1.0, 2.0])))


def test_origin_components_must_be_numbers(tmp_path):
    with pytest.raises(MapError, match='origin'):
        load_map_info(write_map(tmp_path, dict(VALID, origin=[1.0, 'x', 0.0])))


def test_resolution_must_be_a_number(tmp_path):
    with pytest.raises(MapError, match='resolution'):
        load_map_info(write_map(tmp_path, dict(VALID, resolution='fast')))


def test_negate_must_be_zero_or_one(tmp_path):
    with pytest.raises(MapError, match='negate'):
        load_map_info(write_map(tmp_path, dict(VALID, negate=7)))


def test_malformed_yaml_is_rejected(tmp_path):
    yaml_path = tmp_path / 'map.yaml'
    yaml_path.write_text('image: [unclosed\n', encoding='utf-8')
    with pytest.raises(MapError, match='YAML'):
        load_map_info(str(yaml_path))


def test_non_mapping_yaml_is_rejected(tmp_path):
    yaml_path = tmp_path / 'map.yaml'
    yaml_path.write_text('- just\n- a\n- list\n', encoding='utf-8')
    with pytest.raises(MapError, match='键值映射'):
        load_map_info(str(yaml_path))


def test_unknown_image_suffix_is_rejected(tmp_path):
    data = dict(VALID, image='map.txt')
    yaml_path = tmp_path / 'map.yaml'
    yaml_path.write_text(yaml.safe_dump(data), encoding='utf-8')
    (tmp_path / 'map.txt').write_bytes(b'nope')
    with pytest.raises(MapError, match='扩展名'):
        load_map_info(str(yaml_path))
