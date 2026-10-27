"""从 LaserScan 中提取扇区距离的工具，与雷达型号无关。

角度约定遵循 REP-103：0 弧度指向正前方，逆时针为正，因此机器人左侧对应正角度、
右侧对应负角度。

扇区对应的下标一律由 ``angle_min`` / ``angle_increment`` 反算，而不是假设
"一个采样点等于一度"。TurtleBot3 恰好是 360 线、1 度间隔、从正前方起算，但
RPLidar 之类的雷达分辨率、起始角、视场角都不同。把度数当下标写死，换雷达后会
静默地读错方向——不报错，只是机器人朝着错误的一侧转。

本模块只用到消息类型，不依赖 rclpy，可以脱离 ROS 图直接单元测试。
"""

import math
from typing import List, Optional, Tuple

from sensor_msgs.msg import LaserScan


def _valid_range(distance: float, scan: LaserScan) -> bool:
    if math.isnan(distance) or math.isinf(distance):
        return False
    if distance < scan.range_min or distance > scan.range_max:
        return False
    return True


def is_obstacle_reading(distance: Optional[float], range_max: float) -> bool:
    """区分「量程内命中障碍」与「量程内空旷（range_max / inf）」。"""
    if distance is None:
        return False
    return distance < range_max - 1e-3


def copy_laser_scan(scan: LaserScan) -> LaserScan:
    out = LaserScan()
    for attr in (
        'header', 'angle_min', 'angle_max', 'angle_increment',
        'time_increment', 'scan_time', 'range_min', 'range_max',
    ):
        setattr(out, attr, getattr(scan, attr))
    out.ranges = list(scan.ranges)
    if scan.intensities:
        out.intensities = list(scan.intensities)
    return out


def inflate_laser_scan(
    scan: LaserScan,
    inflation_radius: float,
    max_range: float = 0.0,
    center_deg: Optional[float] = None,
    half_width_deg: Optional[float] = None,
) -> LaserScan:
    """把有效回波在角度上展宽，等效于给点状障碍加「体积」。

    ``max_range`` 大于 0 时，只对近距离回波展宽，远处侧向桌腿不参与。

    ``center_deg`` 与 ``half_width_deg`` 同时给出时，只从已经落在该扇区内的
    回波向外摊。扇区外的墙不能写进扇区，否则近距离侧墙的展宽角会盖过窄通路扇区。
    """
    if scan is None or inflation_radius <= 0.0 or not len(scan.ranges):
        return scan
    if scan.angle_increment == 0.0:
        return scan

    out = copy_laser_scan(scan)
    inflated: List[float] = list(out.ranges)
    count = len(inflated)
    increment = abs(scan.angle_increment)
    limit_to_sector = center_deg is not None and half_width_deg is not None
    if limit_to_sector:
        center = normalize_angle(math.radians(center_deg))
        half_width = math.radians(abs(half_width_deg))

    for index, distance in enumerate(scan.ranges):
        if limit_to_sector:
            angle = scan.angle_min + index * scan.angle_increment
            if abs(normalize_angle(angle - center)) > half_width:
                continue
        if not _valid_range(distance, scan):
            continue
        if max_range > 0.0 and distance > max_range:
            continue

        half_angle = math.atan(inflation_radius / max(distance, scan.range_min + 1e-6))
        half_bins = max(1, int(math.ceil(half_angle / increment)))

        for offset in range(-half_bins, half_bins + 1):
            target = index + offset
            if target < 0 or target >= count:
                continue
            if _valid_range(inflated[target], scan):
                inflated[target] = min(inflated[target], distance)
            else:
                inflated[target] = distance

    out.ranges = inflated
    return out


class SectorDistanceFilter:
    """跨帧保留最近扇区距离，避免稀疏点状障碍在 miss 帧被当成空旷。

    ``decay_rate`` 让记忆距离随时间增大（模拟已经绕过障碍）；超过 ``release_distance``
    或 ``persistence_sec`` 后释放，避免空旷地带长期保持「高度警戒」。
    """

    def __init__(
        self,
        persistence_sec: float = 0.0,
        decay_rate: float = 0.0,
        release_distance: float = 0.0,
    ):
        self._persistence = max(0.0, persistence_sec)
        self._decay_rate = max(0.0, decay_rate)
        self._release_distance = max(0.0, release_distance)
        self._stored: List[Optional[float]] = [None, None, None, None]
        self._times: List[float] = [0.0, 0.0, 0.0, 0.0]

    def apply(
        self,
        front: Optional[float],
        left: Optional[float],
        right: Optional[float],
        rear: Optional[float],
        now: float,
        range_max: float,
    ) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        if self._persistence <= 0.0:
            return front, left, right, rear

        values = [front, left, right, rear]
        out: List[Optional[float]] = []
        for index, value in enumerate(values):
            if is_obstacle_reading(value, range_max):
                self._stored[index] = value
                self._times[index] = now
                out.append(value)
                continue

            # 盲区不是空旷。用上一帧的距离盖掉 None，安全层会把无效雷达当成
            # 前方还有数，扫描仍在到达，超时停车也不会触发。记忆留着，
            # 下一帧若恢复成真实障碍或空旷读数，仍按原规则接上。
            if value is None:
                out.append(None)
                continue

            stored = self._stored[index]
            if stored is None:
                out.append(value)
                continue

            elapsed = now - self._times[index]
            aged = stored + self._decay_rate * elapsed
            release = (
                elapsed > self._persistence
                or (self._release_distance > 0.0 and aged >= self._release_distance)
            )
            if release:
                self._stored[index] = None
                out.append(value)
                continue

            out.append(aged)

        return out[0], out[1], out[2], out[3]


def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]。

    走 atan2(sin, cos) 而不是手写取模，边界情况和符号都交给标准库处理。
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def sector_min_distance(
    scan: LaserScan,
    center_deg: float,
    half_width_deg: float,
) -> Optional[float]:
    """返回以 ``center_deg`` 为中心、半宽 ``half_width_deg`` 的扇区内最近障碍距离（米）。

    返回值刻意区分三种情况，因为"那里没东西"和"我看不见"对导航的含义正好相反：

    - 具体数值：扇区内测到了有效障碍，取最近的那一个。
    - ``range_max``：所有读数都超出量程（含 ``inf``），说明量程内空无一物。
    - ``None``：扇区内没有任何可用测量——全是 NaN，或全是低于 ``range_min``
      的丢点。调用方据此把该方向视为盲区，navigator 会按不可通行处理。
    """
    # 空消息，或 angle_increment 为 0（下标无法映射到角度），都无从判断。
    # 这里按盲区返回而不是抛异常：调用方位于控制回路中，宁可降级也不要崩。
    if scan is None or not len(scan.ranges) or scan.angle_increment == 0.0:
        return None

    center = normalize_angle(math.radians(center_deg))
    half_width = math.radians(abs(half_width_deg))

    closest: Optional[float] = None
    saw_clear = False   # 是否见过"超出量程"的读数，用来区分空旷与盲区

    for index, distance in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        # 用归一化后的角度差判断是否落在扇区内。正前方扇区会跨越采样序列的首尾
        # 接缝（例如 350 度与 10 度同属前方），直接比较角度大小会漏掉一半采样点。
        if abs(normalize_angle(angle - center)) > half_width:
            continue

        if math.isnan(distance):
            continue            # 无效读数：既不算障碍，也不能算空旷
        if distance > scan.range_max:
            saw_clear = True    # 超出量程说明这条射线上、量程以内没有东西
            continue
        if distance < scan.range_min:
            continue            # 低于最小量程的丢点同样不可信

        if closest is None or distance < closest:
            closest = distance

    if closest is not None:
        return closest
    # 一个有效障碍都没测到：见过超量程读数才是真空旷，否则整个扇区都是盲区。
    return scan.range_max if saw_clear else None


def sector_min_distance_inflated(
    scan: LaserScan,
    center_deg: float,
    half_width_deg: float,
    inflation_radius: float = 0.0,
    max_range: float = 0.0,
) -> Optional[float]:
    """扇区最近距离。展宽只从本扇区已有的回波向外摊。"""
    if inflation_radius <= 0.0 or scan is None:
        return sector_min_distance(scan, center_deg, half_width_deg)
    inflated = inflate_laser_scan(
        scan, inflation_radius, max_range,
        center_deg=center_deg, half_width_deg=half_width_deg)
    return sector_min_distance(inflated, center_deg, half_width_deg)


def describe_environment(
    scan: LaserScan,
    front_half_deg: float,
    side_center_deg: float,
    side_half_deg: float,
    front_center_deg: float = 0.0,
    path_half_deg: float = 0.0,
    inflation_radius: float = 0.0,
    inflation_max_range: float = 0.0,
    sector_filter: Optional[SectorDistanceFilter] = None,
    filter_time: Optional[float] = None,
) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
    """一次取出前、左、右、后四个方向的最近障碍距离。

    返回顺序固定为 (前, 左, 右, 后)。

    ``front_center_deg`` 是**机器人正前方在扫描角坐标下的读数**，也就是雷达相对
    车体的安装偏转角。四个扇区全部由它推算：机器人坐标下的方位 φ 对应扫描角
    ``φ + front_center_deg``。tjark_agv 的雷达绕 z 轴装反了 180°（tf 实测
    ``R_base_laser = Rz(pi)``），因此正前方落在扫描角 180°，左侧落在 270°、
    右侧落在 90°。

    只把偏移加到前方、侧向仍按 ±90° 取，会让左右整体互换——机器人于是朝更封闭
    的一侧转，贴墙时几何关系反复改变符号，表现为原地左右摆头且无法脱困。

    前提：雷达 z 轴与车体 z 轴同向（未倒装）。倒装会额外镜像扫描角的正负号，
    此处不做处理。

    ``path_half_deg`` 若大于 0，则「前方」取更窄的通路扇区（默认 ±12°），用于直行/
    转向决策。宽扇区 ``front_half_deg`` 会把侧墙斜射进前方读数，贴墙巡航时
    ``ahead`` 会被误判为 0.55 m 而长期卡在迟滞带。

    ``inflation_radius`` 与 ``sector_filter`` 用于稀疏点状障碍（细桌腿等）。
    每个扇区只展宽自己内部的回波，侧向的墙不会被摊进前方通路。
    """
    path_half = path_half_deg if path_half_deg > 0.0 else front_half_deg
    front = sector_min_distance_inflated(
        scan, front_center_deg, path_half, inflation_radius, inflation_max_range)
    left = sector_min_distance_inflated(
        scan, front_center_deg + side_center_deg, side_half_deg,
        inflation_radius, inflation_max_range)
    right = sector_min_distance_inflated(
        scan, front_center_deg - side_center_deg, side_half_deg,
        inflation_radius, inflation_max_range)
    rear = sector_min_distance_inflated(
        scan, front_center_deg + 180.0, front_half_deg,
        inflation_radius, inflation_max_range)

    if sector_filter is not None and filter_time is not None:
        return sector_filter.apply(
            front, left, right, rear, filter_time, scan.range_max)

    return front, left, right, rear


def format_distance(distance: Optional[float]) -> str:
    """把扇区距离渲染成提示词里的一行，盲区必须明确写出来。

    不能用 "0.00m" 之类的数字代替盲区：那会让模型以为前方贴着障碍物，而实际
    情况是这个方向完全没有信息，两者该引出的判断并不一样。
    """
    if distance is None:
        return '未知(无有效回波)'
    return f'{distance:.2f}m'
