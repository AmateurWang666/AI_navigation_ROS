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


# map_server / PGM 灰度值约定（与 occupancy_grid.py 一致）
PGM_FREE = 254
PGM_OCCUPIED = 0
PGM_UNKNOWN = 205

FREE = 1
OCCUPIED = 2
UNKNOWN = 0


class MapCoverage(NamedTuple):
    """栅格地图的覆盖率统计。"""

    width: int
    height: int
    free: int
    occupied: int
    unknown: int

    @property
    def total(self) -> int:
        return self.free + self.occupied + self.unknown

    @property
    def known_ratio(self) -> float:
        return (self.free + self.occupied) / self.total if self.total else 0.0

    @property
    def unknown_ratio(self) -> float:
        return self.unknown / self.total if self.total else 0.0


class ReachabilityReport(NamedTuple):
    """从起点出发、在指定 allow_unknown 策略下能到达的目的地。"""

    coverage: MapCoverage
    spawn_cell: Optional[tuple]
    unreachable: List[str]       # 在 allow_unknown=False 时走不通的目的地名字
    unknown_only: List[str]      # 目的地落在未知格上
    blocked: List[str]           # 目的地落在障碍或地图外


def _read_pgm(path: str) -> tuple:
    """读取 P5 二进制 PGM，返回 (width, height, pixels bytearray)。"""
    with open(path, 'rb') as handle:
        blob = handle.read()
    parts = blob.split(b'\n', 3)
    if parts[0] != b'P5':
        raise MapError(f'{path}: 只支持 P5 二进制 PGM，当前为 {parts[0]!r}')
    width, height = map(int, parts[1].split())
    pixels = parts[3]
    if len(pixels) != width * height:
        raise MapError(
            f'{path}: 像素数 {len(pixels)} 与尺寸 {width}x{height} 不符')
    return width, height, pixels


def _pgm_value_to_state(value: int) -> int:
    if value == PGM_OCCUPIED:
        return OCCUPIED
    if value == PGM_FREE:
        return FREE
    return UNKNOWN


def load_map_grid(info: MapInfo) -> tuple:
    """把地图图片读成按 OccupancyGrid 行序排列的状态网格。

    第 0 行对应 origin 处的 +y 方向（地图底部），与 map_server 一致。
    返回 (width, height, states)，states[row * width + col] 为 FREE/OCCUPIED/UNKNOWN。
    """
    width, height, pixels = _read_pgm(info.image_path)
    states = [UNKNOWN] * (width * height)
    for pgm_row in range(height):
        grid_row = height - 1 - pgm_row
        base = pgm_row * width
        target = grid_row * width
        for col in range(width):
            states[target + col] = _pgm_value_to_state(pixels[base + col])
    return width, height, states


def analyze_coverage(info: MapInfo) -> MapCoverage:
    """统计地图中空闲/占据/未知栅格的数量。"""
    width, height, states = load_map_grid(info)
    counts = {FREE: 0, OCCUPIED: 0, UNKNOWN: 0}
    for state in states:
        counts[state] += 1
    return MapCoverage(width, height, counts[FREE], counts[OCCUPIED], counts[UNKNOWN])


def world_to_cell(info: MapInfo, width: int, height: int, x: float, y: float):
    """世界坐标 -> 栅格 (col, row)。越界返回 None。"""
    col = int((x - info.origin[0]) / info.resolution)
    row = int((y - info.origin[1]) / info.resolution)
    if not (0 <= col < width and 0 <= row < height):
        return None
    return col, row


def _cell_state(states, width, col, row) -> int:
    return states[row * width + col]


def _is_traversable(state: int, allow_unknown: bool) -> bool:
    if state == OCCUPIED:
        return False
    if state == FREE:
        return True
    return allow_unknown


def _bfs_reachable(states, width, height, start, allow_unknown: bool) -> set:
    """从 start=(col,row) 出发，8 邻域 BFS，返回可达格集合。"""
    if start is None:
        return set()
    col0, row0 = start
    if not _is_traversable(_cell_state(states, width, col0, row0), allow_unknown):
        return set()

    seen = {start}
    frontier = [start]
    while frontier:
        next_frontier = []
        for col, row in frontier:
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    if dc == 0 and dr == 0:
                        continue
                    nc, nr = col + dc, row + dr
                    if not (0 <= nc < width and 0 <= nr < height):
                        continue
                    if (nc, nr) in seen:
                        continue
                    if not _is_traversable(
                            _cell_state(states, width, nc, nr), allow_unknown):
                        continue
                    seen.add((nc, nr))
                    next_frontier.append((nc, nr))
        frontier = next_frontier
    return seen


def check_reachability(
    info: MapInfo,
    spawn_xy: tuple,
    destinations: Dict[str, tuple],
    allow_unknown: bool,
) -> ReachabilityReport:
    """检查从起点到各命名目的地的栅格连通性。

    allow_unknown=False 时只在 FREE 格上 BFS——与 GlobalPlanner allow_unknown=false
    的行为一致：路径不能穿越未知区域。若某目的地不可达，全局规划几乎必然失败。
    """
    width, height, states = load_map_grid(info)
    coverage = analyze_coverage(info)
    spawn_cell = world_to_cell(info, width, height, spawn_xy[0], spawn_xy[1])

    reachable = _bfs_reachable(states, width, height, spawn_cell, allow_unknown)

    unreachable, unknown_only, blocked = [], [], []
    for name, dest in destinations.items():
        cell = world_to_cell(info, width, height, dest[0], dest[1])
        if cell is None:
            blocked.append(name)
            continue
        state = _cell_state(states, width, *cell)
        if state == OCCUPIED:
            blocked.append(name)
        elif state == UNKNOWN:
            unknown_only.append(name)
            if cell not in reachable:
                unreachable.append(name)
        elif cell not in reachable:
            unreachable.append(name)

    return ReachabilityReport(coverage, spawn_cell, unreachable, unknown_only, blocked)


# 未知区域超过此比例时，在 allow_unknown=false 下几乎必然规划失败。
SPARSE_MAP_UNKNOWN_RATIO = 0.40


def format_navigation_readiness(
    info: MapInfo,
    spawn_xy: tuple,
    destinations: Dict[str, tuple],
    allow_unknown: bool,
) -> str:
    """生成启动时的地图/nav 就绪报告，含覆盖率与连通性警告。"""
    if not destinations:
        coverage = analyze_coverage(info)
        lines = [
            f'地图覆盖率: 已知 {coverage.known_ratio * 100:.1f}% '
            f'(空闲 {coverage.free / coverage.total * 100:.1f}%, '
            f'占据 {coverage.occupied / coverage.total * 100:.1f}%, '
            f'未知 {coverage.unknown_ratio * 100:.1f}%)',
        ]
        if coverage.unknown_ratio > SPARSE_MAP_UNKNOWN_RATIO and not allow_unknown:
            lines.append(
                '警告: 未知区域占比高且 allow_unknown=false。'
                '若规划失败，请先补建地图或显式设置 allow_unknown:=true。')
        return '\n'.join(lines)

    report = check_reachability(info, spawn_xy, destinations, allow_unknown)
    cov = report.coverage
    lines = [
        f'地图覆盖率: 已知 {cov.known_ratio * 100:.1f}% '
        f'(空闲 {cov.free / cov.total * 100:.1f}%, '
        f'占据 {cov.occupied / cov.total * 100:.1f}%, '
        f'未知 {cov.unknown_ratio * 100:.1f}%)',
        f'allow_unknown={allow_unknown}, '
        f'起点 ({spawn_xy[0]:.2f}, {spawn_xy[1]:.2f}) '
        f'-> 栅格 {report.spawn_cell}',
    ]

    if report.unreachable:
        lines.append('')
        lines.append('!!! 以下命名目的地在 allow_unknown=false 时不可达 !!!')
        lines.append('    （全局规划器不会穿越未知区域，send_goal 将报 Failed to find a valid plan）')
        for name in report.unreachable:
            dest = destinations[name]
            lines.append(f'    - {name}: ({dest[0]:.2f}, {dest[1]:.2f})')
        lines.append('')
        lines.append('推荐做法（按优先级）:')
        lines.append('  1. 补建地图，让起点到目标点之间的走廊变为「已知空闲」——这是正确做法')
        lines.append('     roslaunch ai_robot_nav mapping.launch')
        lines.append('     rosservice call /mapper/save_map')
        lines.append('  2. 若暂时无法补建，可显式放宽（有安全风险，机器人可能进入未探索区域）:')
        lines.append('     roslaunch ai_robot_nav sim.launch allow_unknown:=true')
        lines.append('')
        lines.append('!!! 请勿把 allow_unknown 永久设为 true —— 见 README「地图与导航安全」 !!!')

    if report.blocked:
        lines.append(f'警告: 以下目的地落在障碍或地图外: {", ".join(report.blocked)}')

    if (cov.unknown_ratio > SPARSE_MAP_UNKNOWN_RATIO
            and not allow_unknown
            and not report.unreachable):
        lines.append(
            '提示: 未知区域占比高，部分坐标目标点可能规划失败。'
            '建议补建地图后再导航。')

    return '\n'.join(lines)


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
