"""用户自定义地图的解析与校验。

本项目的地图沿用 ROS 标准的 map_server 格式（一个 YAML 描述 + 一张 PGM/PNG 图），
不自造格式。这样使用者手上已有的地图——无论来自 gmapping、cartographer、还是
别的工程——都能直接拿来用，不需要转换。

为什么要在启动前单独校验一遍，而不是直接把路径丢给 map_server：
map_server 遇到坏地图时的行为对使用者很不友好。文件不存在它直接 exit(-1)，
YAML 少一个键它抛 C++ 异常，图片路径写错时报的是 SDL 的底层错误。这些信息都
不指向真正该改的那一行。导航链路又长（map_server → amcl → move_base），
上游静默失败会表现成下游莫名其妙的症状：机器人不动、或者定位一直发散，
排查要绕很大一圈。所以这里提前把话说清楚，把错误挡在启动之前。

模块不依赖 rospy，可以脱离 ROS 图直接单元测试。
"""

import os
from typing import Any, Dict, List, NamedTuple, Optional

import yaml

# map_server 要求必须存在的键。缺任何一个，地图都无法被正确解释：
#   image           栅格图片路径
#   resolution      米/像素，决定地图尺度
#   origin          地图左下角在世界坐标中的位姿 [x, y, yaw]
#   occupied_thresh 高于此占据概率视为障碍
#   free_thresh     低于此占据概率视为空闲
# negate 有默认值 0，故不强制。
REQUIRED_KEYS = ('image', 'resolution', 'origin', 'occupied_thresh', 'free_thresh')

# map_server 通过 SDL_image 读图，实际支持的格式比这几种多，但常见的就这些。
# 列出来只用于给出「扩展名看着不像地图」的提示，不作为硬性拒绝条件。
KNOWN_IMAGE_SUFFIXES = ('.pgm', '.png', '.bmp', '.jpg', '.jpeg')


class MapError(Exception):
    """地图不可用。异常信息直接面向使用者，应当指出改哪里。"""


class MapInfo(NamedTuple):
    """一张校验通过的地图。"""

    yaml_path: str        # 地图 YAML 的绝对路径
    image_path: str       # 图片的绝对路径（已由 YAML 所在目录解析）
    resolution: float     # 米/像素
    origin: List[float]   # [x, y, yaw]
    occupied_thresh: float
    free_thresh: float
    negate: int


def _require_number(data: Dict[str, Any], key: str, yaml_path: str) -> float:
    value = data[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MapError(
            f'{yaml_path}: {key} 必须是数字，当前是 {type(value).__name__} ({value!r})')
    return float(value)


def resolve_image_path(image_field: str, yaml_path: str) -> str:
    """把 YAML 里的 image 字段解析成绝对路径。

    map_server 的规则是「相对路径相对于 YAML 文件所在目录」，而不是相对于当前
    工作目录。这一点很容易踩：从不同目录 roslaunch 会得到不同结果，表现为
    「昨天还能跑，今天换个目录启动就找不到图片」。这里按同样的规则解析，保证
    校验结果与 map_server 实际读取的是同一个文件。
    """
    if os.path.isabs(image_field):
        return os.path.normpath(image_field)
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                                         image_field))


def load_map_info(yaml_path: str) -> MapInfo:
    """读取并校验地图 YAML，返回解析结果；任何问题都抛 MapError。

    校验项覆盖了实际会导致 map_server 失败或导航行为异常的全部情况：文件缺失、
    YAML 格式错误、必需键缺失、类型不对、数值不合理、图片文件不存在。
    """
    if not yaml_path:
        raise MapError('未指定地图文件。请用 map_file:=/路径/到/你的地图.yaml 指定。')

    yaml_path = os.path.expanduser(yaml_path)
    if not os.path.isfile(yaml_path):
        raise MapError(
            f'地图 YAML 不存在: {yaml_path}\n'
            '请检查 map_file 参数。地图应为 ROS 标准格式（一个 .yaml 加一张 .pgm/.png）。')

    try:
        with open(yaml_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        raise MapError(f'{yaml_path}: YAML 解析失败: {exc}')
    except OSError as exc:
        raise MapError(f'{yaml_path}: 无法读取: {exc}')

    if not isinstance(data, dict):
        raise MapError(
            f'{yaml_path}: 顶层应为键值映射，实际解析出 {type(data).__name__}。')

    missing = [key for key in REQUIRED_KEYS if key not in data]
    if missing:
        raise MapError(
            f'{yaml_path}: 缺少必需字段 {", ".join(missing)}。\n'
            'ROS 标准地图 YAML 形如:\n'
            '  image: my_map.pgm\n'
            '  resolution: 0.05\n'
            '  origin: [-10.0, -10.0, 0.0]\n'
            '  negate: 0\n'
            '  occupied_thresh: 0.65\n'
            '  free_thresh: 0.196')

    if not isinstance(data['image'], str) or not data['image'].strip():
        raise MapError(f'{yaml_path}: image 必须是非空字符串。')

    resolution = _require_number(data, 'resolution', yaml_path)
    if resolution <= 0.0:
        raise MapError(
            f'{yaml_path}: resolution 必须为正数，当前 {resolution}。'
            '它的含义是「米/像素」，室内地图常用 0.05。')

    origin = data['origin']
    if not isinstance(origin, (list, tuple)) or len(origin) < 3:
        raise MapError(
            f'{yaml_path}: origin 必须是形如 [x, y, yaw] 的三元列表，当前 {origin!r}。')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in origin[:3]):
        raise MapError(f'{yaml_path}: origin 的三个分量都必须是数字，当前 {origin!r}。')

    occupied = _require_number(data, 'occupied_thresh', yaml_path)
    free = _require_number(data, 'free_thresh', yaml_path)
    # 两个阈值定义的是同一条概率轴上的上下界，倒挂会让「空闲」与「占据」的判定
    # 区间重叠，map_server 不报错但栅格语义已经错了，最终表现为规划穿墙或全图不可通行。
    if free >= occupied:
        raise MapError(
            f'{yaml_path}: free_thresh({free}) 必须小于 occupied_thresh({occupied})，'
            '否则空闲与占据的判定区间会重叠。常用取值 0.196 / 0.65。')

    negate = data.get('negate', 0)
    if isinstance(negate, bool):
        negate = int(negate)
    if negate not in (0, 1):
        raise MapError(f'{yaml_path}: negate 只能是 0 或 1，当前 {negate!r}。')

    image_path = resolve_image_path(data['image'], yaml_path)
    if not os.path.isfile(image_path):
        raise MapError(
            f'{yaml_path}: 图片文件不存在: {image_path}\n'
            f'（YAML 里写的是 image: {data["image"]}，相对路径按 YAML 所在目录解析）')

    suffix = os.path.splitext(image_path)[1].lower()
    if suffix not in KNOWN_IMAGE_SUFFIXES:
        raise MapError(
            f'{yaml_path}: 图片扩展名 {suffix or "(无)"} 不是常见的地图格式。'
            f'支持的有 {", ".join(KNOWN_IMAGE_SUFFIXES)}。')

    return MapInfo(
        yaml_path=os.path.abspath(yaml_path),
        image_path=image_path,
        resolution=resolution,
        origin=[float(v) for v in origin[:3]],
        occupied_thresh=occupied,
        free_thresh=free,
        negate=int(negate),
    )


def describe_map(info: MapInfo, image_size: Optional[tuple] = None) -> str:
    """把地图信息渲染成一行启动日志，方便确认「加载的确实是我要的那张图」。"""
    line = (f'地图 {os.path.basename(info.yaml_path)}: '
            f'{info.resolution:.3f} m/px, '
            f'原点 ({info.origin[0]:.2f}, {info.origin[1]:.2f}, {info.origin[2]:.2f})')
    if image_size:
        width, height = image_size
        line += (f', {width}x{height} px '
                 f'= {width * info.resolution:.1f}x{height * info.resolution:.1f} m')
    return line
