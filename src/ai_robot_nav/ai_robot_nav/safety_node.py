"""反应式安全层，同时是 /cmd_vel 的唯一发布者（ROS 1 / rospy 版）。"""

import time

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan

from ai_robot_nav.lifecycle import install_shutdown_signals
from ai_robot_nav.log_throttle import LogThrottle
from ai_robot_nav.motion import clamp, clamp_twist
from ai_robot_nav.ros_params import param, param_bool, param_float
from ai_robot_nav.scan_utils import sector_min_distance


class SafetyNode:
    """看门狗 AI 指令流，并强制执行硬性停车区。"""

    def __init__(self):
        self._load_parameters()
        self._log = LogThrottle()

        self._latest_ai_cmd = Twist()
        self._ai_cmd_stamp = None
        self._scan_stamp = None
        self._emergency = False

        output_type = TwistStamped if self._cmd_stamped else Twist
        self._cmd_stamped_type = output_type
        self._cmd_publisher = rospy.Publisher(self._cmd_topic, output_type, queue_size=10)
        rospy.Subscriber(self._scan_topic, LaserScan, self._scan_callback, queue_size=1)
        rospy.Subscriber(self._ai_cmd_topic, Twist, self._ai_cmd_callback, queue_size=10)

        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._control_frequency), self._control_loop)

        if self._passthrough:
            rospy.logwarn(
                'passthrough_mode is on: this node will NOT publish. '
                'The robot is unguarded - intended for manual teleoperation only.')
        rospy.loginfo(
            f'Safety node guarding {self._cmd_topic} at {self._control_frequency:.0f} Hz '
            f'as {output_type.__name__} '
            f'(stop <{self._stop_distance:.2f}m, resume >{self._clear_distance:.2f}m).')

    def _load_parameters(self):
        self._scan_topic = param('scan_topic', '/scan')
        self._ai_cmd_topic = param('ai_cmd_topic', '/ai_cmd_vel')
        self._cmd_topic = param('cmd_topic', '/cmd_vel')
        self._cmd_stamped = param_bool('cmd_stamped', False)
        self._control_frequency = max(1.0, param_float('control_frequency', 20.0))
        self._ai_cmd_timeout = param_float('ai_cmd_timeout', 0.7)
        self._scan_timeout = param_float('scan_timeout', 0.5)
        self._stop_distance = param_float('stop_distance', 0.30)
        self._clear_distance = param_float('clear_distance', 0.40)
        self._front_half_angle = param_float('front_half_angle', 20.0)
        self._front_center_angle = param_float('front_center_angle', 0.0)
        self._max_linear = param_float('max_linear', 0.22)
        self._max_angular = param_float('max_angular', 1.5)
        self._reverse_speed_limit = param_float('reverse_speed_limit', 0.1)
        self._allow_reverse = param_bool('allow_reverse_in_emergency', True)
        self._allow_rotate = param_bool('allow_rotate_in_emergency', True)
        self._passthrough = param_bool('passthrough_mode', False)

        if self._clear_distance < self._stop_distance:
            rospy.logwarn(
                'clear_distance is below stop_distance, which disables hysteresis; '
                'raising it to stop_distance.')
            self._clear_distance = self._stop_distance

    def _scan_callback(self, msg: LaserScan):
        self._scan_stamp = rospy.Time.now()

        front = sector_min_distance(msg, self._front_center_angle, self._front_half_angle)
        was_emergency = self._emergency

        if front is None:
            self._emergency = True
        elif front < self._stop_distance:
            self._emergency = True
        elif front > self._clear_distance:
            self._emergency = False

        if self._emergency and not was_emergency:
            reading = 'no valid return' if front is None else f'{front:.2f}m'
            rospy.logwarn(f'EMERGENCY STOP: obstacle ahead ({reading}).')
            self._publish(self._emergency_command())
        elif was_emergency and not self._emergency:
            rospy.loginfo(f'Path clear ({front:.2f}m); resuming AI control.')

    def _ai_cmd_callback(self, msg: Twist):
        self._latest_ai_cmd = msg
        self._ai_cmd_stamp = rospy.Time.now()

    def _control_loop(self, _event):
        now = rospy.Time.now()
        command = Twist()

        if self._is_stale(self._scan_stamp, now, self._scan_timeout):
            self._log.warn('lidar_silent', 'LiDAR silent; holding stop.', period=2.0)
        elif self._is_stale(self._ai_cmd_stamp, now, self._ai_cmd_timeout):
            self._log.warn(
                'ai_cmd_stale',
                f'No {self._ai_cmd_topic} within {self._ai_cmd_timeout:.1f}s; holding stop.',
                period=2.0)
        elif self._emergency:
            command = self._emergency_command()
        else:
            command = clamp_twist(self._latest_ai_cmd, self._max_linear, self._max_angular)

        self._publish(command)

    def _emergency_command(self) -> Twist:
        command = Twist()
        source = self._latest_ai_cmd

        if self._allow_reverse and source.linear.x < 0.0:
            command.linear.x = clamp(source.linear.x, self._reverse_speed_limit)
        if self._allow_rotate:
            command.angular.z = clamp(source.angular.z, self._max_angular)

        return command

    def _publish(self, command: Twist):
        if self._passthrough:
            return

        if self._cmd_stamped:
            stamped = TwistStamped()
            stamped.header.stamp = rospy.Time.now()
            stamped.twist = command
            self._cmd_publisher.publish(stamped)
        else:
            self._cmd_publisher.publish(command)

    @staticmethod
    def _is_stale(stamp, now, timeout: float) -> bool:
        if stamp is None:
            return True
        return (now - stamp).to_sec() > timeout

    def stop_robot(self):
        self._timer.shutdown()
        if rospy.is_shutdown():
            rospy.logwarn('Node already shutting down; could not flush a stop.')
            return
        for _ in range(3):
            self._publish(Twist())
            time.sleep(0.02)


def main():
    install_shutdown_signals()
    rospy.init_node('safety_node')
    node = SafetyNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()


if __name__ == '__main__':
    main()
