"""占据栅格建图的核心数学。

用途是为一个新环境生成 ROS 标准地图（map_server 格式），供 amcl 定位与
move_base 全局规划使用。本机 Ubuntu 24.04 上装不到 gmapping（社区 PPA 不提供），
因此这里自带一份最小实现。

它与 gmapping 的关键区别：**不做扫描匹配，直接信任里程计给出的位姿**。
所以严格说这是「已知位姿的建图」（mapping with known poses），不是 SLAM。
这个取舍对本项目是划算的：

- 仿真里 diff_drive 插件发布的 odom 精度很高，建出来的图足够干净；
- 实车上里程计会漂移，长回环会糊掉，届时应改用真正的 SLAM 方案。
  docs/mapping.md 写明了这一点，也说明了用户完全可以拿别处建好的图直接用。

算法是标准的对数几率（log-odds）栅格占据建图：
每条激光射线沿途的栅格减少占据几率（打空），末端命中的栅格增加占据几率。
用对数几率而不是直接存概率，是因为多次观测的融合在对数域里就是简单相加，
既避免了反复乘除带来的数值下溢，也让「反复观测到同一堵墙」自然地累积成确信。

模块不依赖 rospy，可以脱离 ROS 图直接单元测试。
"""

import math
from typing import Iterable, List, NamedTuple, Optional, Tuple

# ROS 的 nav_msgs/OccupancyGrid 用这三类取值表达栅格状态。
UNKNOWN = -1
FREE = 0
OCCUPIED = 100


class GridSpec(NamedTuple):
    """栅格地图的几何定义，与 map_server 的 yaml 字段一一对应。"""

    width: int            # 列数（x 方向格数）
    height: int           # 行数（y 方向格数）
    resolution: float     # 米/格
    origin_x: float       # 栅格 (0,0) 左下角在世界坐标中的 x
    origin_y: float       # 同上的 y

    @property
    def size(self) -> int:
        return self.width * self.height


def spec_covering(
    min_x: float,
    min_y: float,
    max_x: float,
    max_y: float,
    resolution: float,
    margin: float = 1.0,
) -> GridSpec:
    """构造一个能覆盖指定世界范围的栅格定义，四周留出 ``margin`` 米余量。

    留余量是因为机器人常会贴着边界走，而射线可能打到范围之外；没有余量时
    这些观测会被整片丢弃，地图边缘于是出现一圈本不该有的未知区域。
    """
    if resolution <= 0.0:
        raise ValueError(f'resolution 必须为正数，得到 {resolution}')
    if max_x <= min_x or max_y <= min_y:
        raise ValueError(
            f'范围无效: x [{min_x}, {max_x}], y [{min_y}, {max_y}]')

    origin_x = min_x - margin
    origin_y = min_y - margin
    width = int(math.ceil((max_x + margin - origin_x) / resolution))
    height = int(math.ceil((max_y + margin - origin_y) / resolution))
    return GridSpec(width, height, resolution, origin_x, origin_y)


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """整数直线光栅化，返回从 (x0,y0) 到 (x1,y1) 沿途的所有格子（含两端）。

    用 Bresenham 而不是「按固定步长采样再取整」：后者步长偏大时会跳过格子，
    在墙上留下一个个孔，射线于是从孔里漏过去，把墙后面的区域错误地清成空闲。
    步长偏小又会在同一个格子上重复更新，让该格的占据几率被人为放大。
    整数算法两个问题都没有，而且不含浮点运算，结果完全可复现。
    """
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    step_x = 1 if x1 > x0 else -1
    step_y = 1 if y1 > y0 else -1

    if dx >= dy:
        error = dx // 2
        for _ in range(dx + 1):
            cells.append((x, y))
            error -= dy
            if error < 0:
                y += step_y
                error += dx
            x += step_x
    else:
        error = dy // 2
        for _ in range(dy + 1):
            cells.append((x, y))
            error -= dx
            if error < 0:
                x += step_x
                error += dy
            y += step_y

    return cells


class OccupancyGridMapper:
    """对数几率占据栅格。

    ``l_occ`` / ``l_free`` 是单次观测带来的对数几率增量，``l_min`` / ``l_max``
    是累积上下界。设上下界是为了让地图保持可修改：不封顶的话，一堵被观测过
    上千次的墙会累积到极大的确信度，之后即使它被搬走了，也需要同样上千次的
    反向观测才能翻转过来，地图于是「记住」了早已不存在的障碍。
    """

    def __init__(
        self,
        spec: GridSpec,
        l_occ: float = 0.85,
        l_free: float = -0.4,
        l_min: float = -4.0,
        l_max: float = 4.0,
    ):
        if l_occ <= 0.0:
            raise ValueError('l_occ 必须为正（命中应当提高占据几率）')
        if l_free >= 0.0:
            raise ValueError('l_free 必须为负（打空应当降低占据几率）')

        self.spec = spec
        self.l_occ = l_occ
        self.l_free = l_free
        self.l_min = l_min
        self.l_max = l_max
        # 初值 0 表示「占据与空闲各半」，也就是未知。
        self._log_odds = [0.0] * spec.size

    # ---------------------------------------------------------- 坐标换算
    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """世界坐标 -> 栅格下标。落在地图之外返回 None。"""
        col = int(math.floor((x - self.spec.origin_x) / self.spec.resolution))
        row = int(math.floor((y - self.spec.origin_y) / self.spec.resolution))
        if 0 <= col < self.spec.width and 0 <= row < self.spec.height:
            return col, row
        return None

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """栅格下标 -> 该格中心的世界坐标。"""
        return (
            self.spec.origin_x + (col + 0.5) * self.spec.resolution,
            self.spec.origin_y + (row + 0.5) * self.spec.resolution,
        )

    def _index(self, col: int, row: int) -> int:
        # OccupancyGrid 的 data 是行优先（row-major），行号沿 +y 增长。
        return row * self.spec.width + col

    def log_odds_at(self, col: int, row: int) -> float:
        return self._log_odds[self._index(col, row)]

    def _update(self, col: int, row: int, delta: float):
        if not (0 <= col < self.spec.width and 0 <= row < self.spec.height):
            return
        index = self._index(col, row)
        value = self._log_odds[index] + delta
        self._log_odds[index] = max(self.l_min, min(self.l_max, value))

    # ---------------------------------------------------------- 观测融合
    def integrate_ray(
        self,
        sensor_x: float,
        sensor_y: float,
        end_x: float,
        end_y: float,
        hit: bool,
    ):
        """融合一条激光射线。``hit`` 为 False 表示超出量程，末端不标记为障碍。

        超量程的射线必须只清空、不标记：那条射线上「没测到东西」是确定的，
        但末端那个点并没有观测到障碍，标上去等于凭空造出一堵墙。
        """
        start = self.world_to_grid(sensor_x, sensor_y)
        if start is None:
            # 传感器本身不在地图内，这条射线无从落笔。
            return

        end_col = int(math.floor((end_x - self.spec.origin_x) / self.spec.resolution))
        end_row = int(math.floor((end_y - self.spec.origin_y) / self.spec.resolution))

        cells = bresenham(start[0], start[1], end_col, end_row)
        # 末端单独处理，其余一律按「射线穿过 = 空闲」更新。
        for col, row in cells[:-1]:
            self._update(col, row, self.l_free)

        if cells:
            col, row = cells[-1]
            self._update(col, row, self.l_occ if hit else self.l_free)

    def integrate_scan(
        self,
        sensor_x: float,
        sensor_y: float,
        sensor_yaw: float,
        beams: Iterable[Tuple[float, float]],
        range_min: float,
        range_max: float,
    ):
        """融合一帧扫描。``beams`` 是 (角度, 距离) 序列，角度为传感器坐标系下的弧度。

        距离为 NaN 的射线整条丢弃：那既不是「测到障碍」也不是「确认空旷」，
        没有任何信息可用。当成超量程去清空会把未知区域错误地标成空闲。
        """
        for angle, distance in beams:
            if distance is None or math.isnan(distance):
                continue

            hit = True
            if math.isinf(distance) or distance >= range_max:
                distance = range_max
                hit = False
            elif distance < range_min:
                continue        # 低于最小量程的丢点不可信

            world_angle = sensor_yaw + angle
            self.integrate_ray(
                sensor_x, sensor_y,
                sensor_x + distance * math.cos(world_angle),
                sensor_y + distance * math.sin(world_angle),
                hit,
            )

    # ---------------------------------------------------------- 输出
    def to_occupancy_data(
        self,
        occupied_log_odds: float = 0.85,
        free_log_odds: float = -0.4,
    ) -> List[int]:
        """导出 nav_msgs/OccupancyGrid 的 data 字段（-1 未知 / 0 空闲 / 100 占据）。

        两个阈值之间的格子保持未知而不是强行二选一：观测证据不足时如实报告
        「不知道」，move_base 的 allow_unknown=false 会据此拒绝把路径规划到
        这些地方去，比猜一个值安全。
        """
        data = []
        for value in self._log_odds:
            if value >= occupied_log_odds:
                data.append(OCCUPIED)
            elif value <= free_log_odds:
                data.append(FREE)
            else:
                data.append(UNKNOWN)
        return data

    def coverage(self) -> float:
        """已确定（非未知）栅格的占比，用来判断建图跑得够不够久。"""
        known = sum(1 for value in self._log_odds if value != 0.0)
        return known / self.spec.size if self.spec.size else 0.0


# map_server 约定的 PGM 灰度值。它读图时按 occupied_thresh / free_thresh
# 把灰度换算回占据概率，所以这三个值必须落在阈值的正确一侧。
PGM_FREE = 254
PGM_OCCUPIED = 0
PGM_UNKNOWN = 205


def to_pgm_bytes(data: List[int], width: int, height: int) -> bytes:
    """把 OccupancyGrid 的 data 渲染成 map_server 能读的 PGM（P5 二进制）。

    自带存盘能力是有意为之：标准做法是 ``rosrun map_server map_saver``，
    但那要求导航栈已经装好，而建图往往正是「还没有导航栈」时要做的第一步。
    格式本身很简单，与其引入一个先有鸡还是先有蛋的依赖，不如自己写。

    **必须上下翻转**：OccupancyGrid 的第 0 行在地图底部（沿 +y 增长），
    而 PGM 的第 0 行在图像顶部。不翻转的话地图会上下镜像——它看起来仍然
    「像一张地图」，不会报任何错，但机器人会把左右完全走反。
    """
    if width <= 0 or height <= 0:
        raise ValueError(f'尺寸无效: {width}x{height}')
    if len(data) != width * height:
        raise ValueError(
            f'data 长度 {len(data)} 与尺寸 {width}x{height}={width * height} 不符')

    pixels = bytearray(width * height)
    for row in range(height):
        source = (height - 1 - row) * width      # 翻转
        target = row * width
        for col in range(width):
            value = data[source + col]
            if value == OCCUPIED:
                pixels[target + col] = PGM_OCCUPIED
            elif value == FREE:
                pixels[target + col] = PGM_FREE
            else:
                pixels[target + col] = PGM_UNKNOWN

    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    return header + bytes(pixels)


def to_map_yaml(
    image_name: str,
    spec: GridSpec,
    occupied_thresh: float = 0.65,
    free_thresh: float = 0.196,
) -> str:
    """生成配套的地图 YAML，字段与 map_server 的要求一一对应。

    image 写成相对文件名而不是绝对路径：map_server 按 YAML 所在目录解析相对
    路径，这样整个地图目录可以随便搬动、也可以直接提交进仓库，换台机器仍然能用。
    """
    return (
        f'image: {image_name}\n'
        f'resolution: {spec.resolution}\n'
        f'origin: [{spec.origin_x}, {spec.origin_y}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: {occupied_thresh}\n'
        f'free_thresh: {free_thresh}\n'
    )
