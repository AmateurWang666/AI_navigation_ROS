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
from typing import Optional, Tuple

from sensor_msgs.msg import LaserScan


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


def describe_environment(
    scan: LaserScan,
    front_half_deg: float,
    side_center_deg: float,
    side_half_deg: float,
) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
    """一次取出前、左、右、后四个方向的最近障碍距离。

    返回顺序固定为 (前, 左, 右, 后)。后方扇区取 ±180°，半宽与前方相同。
    """
    front = sector_min_distance(scan, 0.0, front_half_deg)
    left = sector_min_distance(scan, side_center_deg, side_half_deg)
    right = sector_min_distance(scan, -side_center_deg, side_half_deg)
    rear = sector_min_distance(scan, 180.0, front_half_deg)
    return front, left, right, rear


def format_distance(distance: Optional[float]) -> str:
    """把扇区距离渲染成提示词里的一行，盲区必须明确写出来。

    不能用 "0.00m" 之类的数字代替盲区：那会让模型以为前方贴着障碍物，而实际
    情况是这个方向完全没有信息，两者该引出的判断并不一样。
    """
    if distance is None:
        return '未知(无有效回波)'
    return f'{distance:.2f}m'
