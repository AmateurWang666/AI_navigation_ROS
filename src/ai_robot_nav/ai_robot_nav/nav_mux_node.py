"""指令仲裁节点：在 move_base 目标点导航与反应式漫游之间切换（ROS 1 / rospy 版）。

系统里现在有两个速度来源，本节点是它们汇流到 ``/ai_cmd_vel`` 的唯一入口：

    move_base   ──► /nav_cmd_vel     ─┐
                                      ├─► nav_mux ──► /ai_cmd_vel ──► safety_node ──► /cmd_vel
    ai_nav_node ──► /explore_cmd_vel ─┘

move_base 的输出被重映射到 ``/nav_cmd_vel`` 而不是默认的 ``/cmd_vel``，因此
``safety_node`` 依然是 ``/cmd_vel`` 的唯一发布者，激光急停对目标点导航同样生效。
仲裁逻辑本身在 ``command_mux`` 里，是纯函数；本节点只负责接线与计时。
"""

import time

import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist

from ai_robot_nav.command_mux import (
    IDLE_EXPLORE, IDLE_STOP, MuxConfig, has_active_goal, select_command,
)
from ai_robot_nav.lifecycle import install_shutdown_signals
from ai_robot_nav.log_throttle import LogThrottle
from ai_robot_nav.motion import clamp
from ai_robot_nav.ros_params import param, param_float


class NavMuxNode:
    """把两路速度指令仲裁成一路，输出给安全层。"""

    def __init__(self):
        self._load_parameters()
        self._log = LogThrottle()

        self._goal_cmd = None
        self._goal_cmd_stamp = None
        self._explore_cmd = None
        self._explore_cmd_stamp = None
        self._goal_active = False
        self._last_source = None

        self._publisher = rospy.Publisher(self._output_topic, Twist, queue_size=10)
        rospy.Subscriber(self._goal_cmd_topic, Twist, self._goal_cmd_callback, queue_size=1)
        rospy.Subscriber(
            self._explore_cmd_topic, Twist, self._explore_cmd_callback, queue_size=1)
        rospy.Subscriber(
            self._status_topic, GoalStatusArray, self._status_callback, queue_size=1)

        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._publish_rate), self._control_loop)

        rospy.loginfo(
            f'Nav mux up: {self._goal_cmd_topic} (目标点) / '
            f'{self._explore_cmd_topic} (漫游) -> {self._output_topic}, '
            f'无目标时{"漫游" if self._config.idle_behavior == IDLE_EXPLORE else "停车"}')

    def _load_parameters(self):
        self._goal_cmd_topic = param('goal_cmd_topic', '/nav_cmd_vel')
        self._explore_cmd_topic = param('explore_cmd_topic', '/explore_cmd_vel')
        self._output_topic = param('output_topic', '/ai_cmd_vel')
        self._status_topic = param('move_base_status_topic', '/move_base/status')
        self._publish_rate = max(1.0, param_float('publish_rate', 20.0))
        self._max_linear = param_float('max_linear', 0.20)
        self._max_angular = param_float('max_angular', 1.2)

        idle_behavior = str(param('idle_behavior', IDLE_EXPLORE)).lower()
        if idle_behavior not in (IDLE_EXPLORE, IDLE_STOP):
            rospy.logwarn(
                f'idle_behavior 取值 {idle_behavior!r} 无法识别，'
                f'回退为 {IDLE_EXPLORE}。可选值: {IDLE_EXPLORE} / {IDLE_STOP}')
            idle_behavior = IDLE_EXPLORE

        self._config = MuxConfig(
            goal_cmd_timeout=param_float('goal_cmd_timeout', 0.5),
            explore_cmd_timeout=param_float('explore_cmd_timeout', 0.7),
            idle_behavior=idle_behavior,
        )

    def _goal_cmd_callback(self, msg: Twist):
        self._goal_cmd = (msg.linear.x, msg.angular.z)
        self._goal_cmd_stamp = rospy.Time.now()

    def _explore_cmd_callback(self, msg: Twist):
        self._explore_cmd = (msg.linear.x, msg.angular.z)
        self._explore_cmd_stamp = rospy.Time.now()

    def _status_callback(self, msg: GoalStatusArray):
        active = has_active_goal([status.status for status in msg.status_list])
        if active != self._goal_active:
            rospy.loginfo('目标点导航已激活。' if active else '目标点已结束，交还控制权。')
        self._goal_active = active

    def _control_loop(self, _event):
        now = rospy.Time.now()
        decision = select_command(
            goal_active=self._goal_active,
            goal_cmd=self._goal_cmd,
            goal_cmd_age=self._age(self._goal_cmd_stamp, now),
            explore_cmd=self._explore_cmd,
            explore_cmd_age=self._age(self._explore_cmd_stamp, now),
            config=self._config,
        )

        command = Twist()
        command.linear.x = clamp(decision.linear_x, self._max_linear)
        command.angular.z = clamp(decision.angular_z, self._max_angular)
        self._publisher.publish(command)

        if decision.source != self._last_source:
            rospy.loginfo(f'指令来源切换为 {decision.source}: {decision.reason}')
            self._last_source = decision.source
        elif decision.source == 'stop':
            # 持续停车往往意味着有东西没接上（move_base 没起来、漫游节点挂了）。
            # 切换日志只在状态变化时打一次，这里补一条低频提醒，避免"机器人不动
            # 且日志一片安静"这种最难排查的情况。
            self._log.warn('mux_stopped', f'仲裁器持续输出零速: {decision.reason}', period=5.0)

    @staticmethod
    def _age(stamp, now) -> float:
        if stamp is None:
            return float('inf')
        return (now - stamp).to_sec()

    def shutdown(self):
        self._timer.shutdown()
        if rospy.is_shutdown():
            rospy.logwarn('Node already shutting down; could not flush a stop.')
            return
        for _ in range(3):
            self._publisher.publish(Twist())
            time.sleep(0.02)


def main():
    install_shutdown_signals()
    rospy.init_node('nav_mux')
    node = NavMuxNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
