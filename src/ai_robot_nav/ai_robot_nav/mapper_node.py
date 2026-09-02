"""占据栅格建图节点：为新环境生成一张 ROS 标准地图（ROS 1 / rospy 版）。

    roslaunch ai_robot_nav mapping.launch          # 建图 + 让机器人自己漫游
    rosrun map_server map_saver -f 我的地图         # 满意后存盘

存出来的就是标准的 map_server 格式（.yaml + .pgm），之后用
``roslaunch ai_robot_nav sim.launch map_file:=我的地图.yaml`` 即可导航。

位姿直接取自 tf（默认 ``odom`` 坐标系），不做扫描匹配，因此这是「已知位姿的
建图」而不是 SLAM。仿真里 odom 精度足够，实车上长距离会漂移——取舍的理由与
局限写在 ``occupancy_grid`` 模块的文档里。

建图跑的是本项目原有的反应式漫游层：机器人自己避障游走，扫过的地方逐步变成
地图。不需要遥控，也不需要额外的探索算法。
"""

import math
import os

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse

from ai_robot_nav.lifecycle import install_shutdown_signals
from ai_robot_nav.log_throttle import LogThrottle
from ai_robot_nav.occupancy_grid import (
    GridSpec, OccupancyGridMapper, to_map_yaml, to_pgm_bytes,
)
from ai_robot_nav.ros_params import param, param_float, param_int


def yaw_from_quaternion(q) -> float:
    """从四元数取偏航角。

    只需要绕 z 的分量，用 atan2 直接算比引入完整的欧拉角转换更省事，
    也避免了万向锁相关的边界情况——地面机器人的 roll/pitch 恒为零。
    """
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class MapperNode:
    """订阅激光，按 tf 给出的位姿累积占据栅格，并周期性发布 /map。"""

    def __init__(self):
        self._load_parameters()
        self._log = LogThrottle()

        spec = GridSpec(
            width=self._width,
            height=self._height,
            resolution=self._resolution,
            origin_x=self._origin_x,
            origin_y=self._origin_y,
        )
        self._mapper = OccupancyGridMapper(spec)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._publisher = rospy.Publisher(
            self._map_topic, OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber(self._scan_topic, LaserScan, self._scan_callback, queue_size=1)

        self._scan_count = 0
        self._timer = rospy.Timer(
            rospy.Duration(self._publish_period), self._publish_map)

        # 自带存盘服务。标准做法是 rosrun map_server map_saver，但建图往往正是
        # 导航栈还没装好时要做的第一步，那时 map_saver 还不存在。
        self._save_service = rospy.Service('~save_map', Trigger, self._handle_save)

        rospy.loginfo(
            f'建图节点已启动: {self._width}x{self._height} 格 @ {self._resolution} m/格 '
            f'= {self._width * self._resolution:.1f}x{self._height * self._resolution:.1f} m, '
            f'坐标系 {self._map_frame}')
        rospy.loginfo(f'地图发布到 {self._map_topic}')
        rospy.loginfo(f'存盘: rosservice call /mapper/save_map  ->  {self._save_path}.yaml')

    def _load_parameters(self):
        self._scan_topic = param('scan_topic', '/scan')
        self._map_topic = param('map_topic', '/map')
        # 建图阶段没有定位，位姿只能来自里程计，所以默认挂在 odom 上。
        self._map_frame = param('map_frame', 'odom')
        self._resolution = param_float('resolution', 0.05)
        self._width = param_int('width', 800)
        self._height = param_int('height', 800)
        # 默认把栅格中心对准坐标原点：机器人从 odom 原点出发，四周都能建图。
        self._origin_x = param_float(
            'origin_x', -0.5 * self._width * self._resolution)
        self._origin_y = param_float(
            'origin_y', -0.5 * self._height * self._resolution)
        self._publish_period = max(0.5, param_float('publish_period', 2.0))
        self._transform_timeout = param_float('transform_timeout', 0.2)
        # 存盘路径不带扩展名，与 map_saver -f 的约定一致：会写出 .pgm 和 .yaml 两个文件。
        self._save_path = os.path.expanduser(param('save_path', '/tmp/my_map'))

    def _scan_callback(self, msg: LaserScan):
        try:
            # 按扫描时刻而不是「当前时刻」查变换：机器人在动，两者对应的位姿
            # 不同，用当前位姿去解释一帧稍早的扫描会让障碍物整体错位，
            # 建出来的墙是糊的。
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, msg.header.frame_id, msg.header.stamp,
                rospy.Duration(self._transform_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self._log.warn('tf_missing', f'查不到激光位姿，跳过该帧: {exc}', period=5.0)
            return

        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)

        beams = (
            (msg.angle_min + index * msg.angle_increment, distance)
            for index, distance in enumerate(msg.ranges)
        )
        self._mapper.integrate_scan(
            translation.x, translation.y, yaw, beams, msg.range_min, msg.range_max)
        self._scan_count += 1

    def _publish_map(self, _event):
        if self._scan_count == 0:
            self._log.warn(
                'no_scan', f'尚未融合任何扫描，检查 {self._scan_topic} 是否在发布。', period=5.0)
            return

        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = self._map_frame
        grid.info.resolution = self._resolution
        grid.info.width = self._width
        grid.info.height = self._height
        grid.info.origin.position.x = self._origin_x
        grid.info.origin.position.y = self._origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = self._mapper.to_occupancy_data()

        self._publisher.publish(grid)
        self._log.info(
            'progress',
            f'已融合 {self._scan_count} 帧，地图确定率 {self._mapper.coverage() * 100:.1f}%',
            period=10.0)

    def _handle_save(self, _request) -> TriggerResponse:
        try:
            path = self.save_map()
        except (OSError, ValueError) as exc:
            rospy.logerr(f'地图存盘失败: {exc}')
            return TriggerResponse(success=False, message=str(exc))
        return TriggerResponse(success=True, message=path)

    def save_map(self) -> str:
        """把当前地图写成 .pgm + .yaml，返回 yaml 路径。"""
        if self._scan_count == 0:
            raise ValueError('尚未融合任何扫描，没有可存的地图')

        directory = os.path.dirname(self._save_path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        image_name = os.path.basename(self._save_path) + '.pgm'
        image_path = self._save_path + '.pgm'
        yaml_path = self._save_path + '.yaml'

        data = self._mapper.to_occupancy_data()
        with open(image_path, 'wb') as handle:
            handle.write(to_pgm_bytes(data, self._width, self._height))
        with open(yaml_path, 'w', encoding='utf-8') as handle:
            handle.write(to_map_yaml(image_name, self._mapper.spec))

        rospy.loginfo(
            f'地图已保存: {yaml_path} (融合 {self._scan_count} 帧, '
            f'确定率 {self._mapper.coverage() * 100:.1f}%)')
        return yaml_path

    def shutdown(self):
        self._timer.shutdown()


def main():
    install_shutdown_signals()
    rospy.init_node('mapper')
    node = MapperNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
