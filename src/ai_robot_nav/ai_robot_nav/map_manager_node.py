"""地图校验与目的地加载节点（ROS 1 / rospy 版）。

它在导航链路启动时做两件事：

1. **校验用户地图**，把错误挡在 map_server 之前。地图有问题时 map_server 的报错
   （SDL 底层错误、C++ 异常、直接 exit(-1)）都不指向真正该改的那一行，而下游的
   amcl 和 move_base 又会把上游的静默失败表现成「机器人莫名不动」。这里提前
   检查并给出可操作的中文提示，本节点标了 ``required="true"``，校验不过就让
   整个 launch 立刻停下，而不是留一堆半死不活的节点。

2. **加载命名目的地**到全局参数 ``/destinations``，供 ``send_goal`` 按名字取用。
   目的地表默认放在地图旁边（``我的地图.destinations.yaml``），跟着地图走：
   换一张地图就换一套目的地，两者天然对应，不会出现「用 A 地图的坐标去 B 地图
   导航」这种既不报错、结果又完全不对的情况。
"""

import os
import sys

import rospy
import yaml

from ai_robot_nav.goals import GoalError, format_destinations, parse_destinations
from ai_robot_nav.lifecycle import install_shutdown_signals
from ai_robot_nav.map_utils import MapError, describe_map, load_map_info
from ai_robot_nav.ros_params import param

# 目的地表的默认位置：把地图 yaml 的扩展名换成 .destinations.yaml。
DESTINATIONS_SUFFIX = '.destinations.yaml'


def default_destinations_path(map_file: str) -> str:
    """由地图路径推出目的地表的默认路径。"""
    return os.path.splitext(map_file)[0] + DESTINATIONS_SUFFIX


def load_destinations(path: str):
    """读取目的地表。文件不存在是正常情况——只用坐标导航时不需要它。"""
    if not os.path.isfile(path):
        return {}, f'未找到目的地表 {path}（可选，只用坐标导航时不需要）'

    try:
        with open(path, 'r', encoding='utf-8') as handle:
            raw = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        raise GoalError(f'{path}: YAML 解析失败: {exc}')
    except OSError as exc:
        raise GoalError(f'{path}: 无法读取: {exc}')

    # 允许两种写法：顶层直接是目的地表，或者包在 destinations: 键下面。
    # 后者便于把目的地和别的配置放在同一个文件里。
    if isinstance(raw, dict) and 'destinations' in raw:
        raw = raw['destinations']

    table = parse_destinations(raw)
    return table, f'已从 {path} 加载 {len(table)} 个命名目的地'


def main():
    install_shutdown_signals()
    rospy.init_node('map_manager')

    map_file = param('map_file', '')

    try:
        info = load_map_info(map_file)
    except MapError as exc:
        # logfatal 进 rosout，print 进终端。roslaunch 的日志很长，只写 rosout
        # 的话这条最关键的信息很容易被淹没在后续的节点退出刷屏里。
        rospy.logfatal(f'地图不可用:\n{exc}')
        print(f'\n[地图错误] {exc}\n', file=sys.stderr)
        sys.exit(1)

    rospy.loginfo(describe_map(info))
    rospy.loginfo(f'地图图片: {info.image_path}')

    destinations_file = param('destinations_file', '') or \
        default_destinations_path(info.yaml_path)

    try:
        table, message = load_destinations(destinations_file)
    except GoalError as exc:
        rospy.logfatal(f'目的地表不可用:\n{exc}')
        print(f'\n[目的地表错误] {exc}\n', file=sys.stderr)
        sys.exit(1)

    rospy.loginfo(message)
    if table:
        # 存成普通列表而不是 Destination，ROS 参数服务器只接受基本类型。
        rospy.set_param('/destinations',
                        {name: [d.x, d.y, d.yaw] for name, d in table.items()})
        rospy.loginfo(f'可用目的地:\n{format_destinations(table)}')

    # 校验与加载都是一次性的，但本节点标了 required="true"，退出会让 roslaunch
    # 关停整个链路。因此这里必须保持存活，靠 spin 挂住。
    rospy.spin()


if __name__ == '__main__':
    main()
