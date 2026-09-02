"""目的地表的解析与校验。

使用者指定目的地有两种方式，本模块把两者归一成同一个 (x, y, yaw)：

1. **命名目的地** —— 在地图配套的 yaml 里写好 ``前台: [2.5, 3.0, 0.0]``，
   之后只说名字。实际使用时几乎总是这一种：目的地是固定的几个点，
   每次去查坐标既麻烦又容易记错。
2. **直接给坐标** —— 调试和标定新地图时用，省去先改配置再启动的往返。

坐标一律是地图坐标系（``map``）下的米与弧度，与 RViz 里 2D Nav Goal 发出的
``/move_base_simple/goal`` 完全一致，因此两条输入路径在下游没有任何差别。

模块不依赖 rospy，可以直接单元测试。
"""

import math
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple


class GoalError(Exception):
    """目的地不可用。异常信息直接面向使用者，应当指出改哪里。"""


class Destination(NamedTuple):
    """地图坐标系下的一个目标位姿。"""

    x: float
    y: float
    yaw: float = 0.0   # 弧度，机器人抵达后的朝向


def _as_number(value, context: str) -> float:
    # bool 是 int 的子类，不排除的话 True 会被当成 1.0 静默接受。
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GoalError(f'{context}: 期望数字，得到 {type(value).__name__} ({value!r})')
    result = float(value)
    if math.isnan(result) or math.isinf(result):
        raise GoalError(f'{context}: 数值必须是有限的，得到 {value!r}')
    return result


def parse_destination(raw, context: str) -> Destination:
    """把一条 ``[x, y]`` 或 ``[x, y, yaw]`` 解析成 Destination。

    yaw 允许省略：很多目的地只关心「到那个位置」，不关心停下来朝哪。
    省略时按 0.0 处理，与 ROS 里「未指定朝向」的惯例一致。
    """
    if isinstance(raw, dict):
        missing = [key for key in ('x', 'y') if key not in raw]
        if missing:
            raise GoalError(f'{context}: 缺少字段 {", ".join(missing)}')
        return Destination(
            _as_number(raw['x'], f'{context}.x'),
            _as_number(raw['y'], f'{context}.y'),
            _as_number(raw.get('yaw', 0.0), f'{context}.yaw'),
        )

    if not isinstance(raw, (list, tuple)):
        raise GoalError(
            f'{context}: 应为 [x, y] 或 [x, y, yaw] 列表，得到 {type(raw).__name__}')
    if len(raw) not in (2, 3):
        raise GoalError(f'{context}: 应为 2 或 3 个数字，得到 {len(raw)} 个')

    yaw = _as_number(raw[2], f'{context}[2] (yaw)') if len(raw) == 3 else 0.0
    return Destination(
        _as_number(raw[0], f'{context}[0] (x)'),
        _as_number(raw[1], f'{context}[1] (y)'),
        yaw,
    )


def parse_destinations(raw) -> Dict[str, Destination]:
    """解析整张目的地表。空表是合法的——只用坐标导航时不需要配任何名字。"""
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise GoalError(
            f'destinations 应为「名字: [x, y, yaw]」的映射，得到 {type(raw).__name__}')

    table = {}
    for name, value in raw.items():
        key = str(name)
        table[key] = parse_destination(value, f'destinations["{key}"]')
    return table


def format_destinations(table: Dict[str, Destination]) -> str:
    """把目的地表渲染成可读的一段文本，用于报错时提示有哪些可选值。"""
    if not table:
        return '（当前没有配置任何命名目的地）'
    return '\n'.join(
        f'  {name}: x={dest.x:.2f} y={dest.y:.2f} yaw={dest.yaw:.2f}'
        for name, dest in sorted(table.items()))


def resolve_destination(
    args: Sequence[str],
    table: Optional[Dict[str, Destination]] = None,
) -> Tuple[Destination, str]:
    """把命令行参数解析成目标点，返回 (目的地, 来源描述)。

    支持三种写法：

        send_goal 前台            按名字取
        send_goal 2.5 3.0         给坐标，朝向不限
        send_goal 2.5 3.0 1.57    给坐标与朝向

    名字优先于坐标：先查表，查不到再尝试按数字解析。这样即使有人把目的地
    命名成 "3"，也仍然能按名字取到，不会被静默当成坐标。
    """
    table = table or {}
    if not args:
        raise GoalError(
            '未指定目的地。用法:\n'
            '  send_goal <目的地名字>\n'
            '  send_goal <x> <y> [yaw]\n'
            f'可用的命名目的地:\n{format_destinations(table)}')

    if len(args) == 1:
        name = args[0]
        if name in table:
            return table[name], f'命名目的地 "{name}"'
        raise GoalError(
            f'找不到名为 "{name}" 的目的地，也无法按坐标解析（坐标需要至少两个数字）。\n'
            f'可用的命名目的地:\n{format_destinations(table)}')

    if args[0] in table:
        return table[args[0]], f'命名目的地 "{args[0]}"'

    if len(args) > 3:
        raise GoalError(f'参数过多（{len(args)} 个）。用法: send_goal <x> <y> [yaw]')

    try:
        numbers: List[float] = [float(value) for value in args]
    except ValueError:
        raise GoalError(
            f'无法把 {" ".join(args)} 解析为坐标，也不是已知的目的地名字。\n'
            f'可用的命名目的地:\n{format_destinations(table)}')

    return parse_destination(numbers, '命令行坐标'), '命令行坐标'
