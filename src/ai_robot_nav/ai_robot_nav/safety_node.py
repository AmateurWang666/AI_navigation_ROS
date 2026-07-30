"""Reactive safety layer and the sole publisher on /cmd_vel.

The AI planner runs at roughly 0.5 Hz, far slower than the robot can get into
trouble, so this node owns the velocity output and republishes it at a fixed
rate. Anything that stops feeding it - a dead planner, a stalled Ollama call, a
dropped network link, a silent LiDAR - decays to a stop within one timeout
instead of leaving the last command latched in the base controller.
"""

import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from ai_robot_nav.lifecycle import install_shutdown_signals
from ai_robot_nav.motion import clamp, clamp_twist
from ai_robot_nav.scan_utils import sector_min_distance


class SafetyNode(Node):
    """Watchdogs the AI command stream and enforces a hard stop zone."""

    def __init__(self):
        super().__init__('safety_node')

        self._declare_parameters()
        self._load_parameters()

        self._latest_ai_cmd = Twist()
        self._ai_cmd_stamp = None
        self._scan_stamp = None
        self._emergency = False

        output_type = TwistStamped if self._cmd_stamped else Twist
        self._cmd_publisher = self.create_publisher(output_type, self._cmd_topic, 10)
        self.create_subscription(
            LaserScan, self._scan_topic, self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            Twist, self._ai_cmd_topic, self._ai_cmd_callback, 10)

        self._timer = self.create_timer(1.0 / self._control_frequency, self._control_loop)

        if self._passthrough:
            self.get_logger().warn(
                'passthrough_mode is on: this node will NOT publish. '
                'The robot is unguarded - intended for manual teleoperation only.')
        self.get_logger().info(
            f'Safety node guarding {self._cmd_topic} at {self._control_frequency:.0f} Hz '
            f'as {output_type.__name__} '
            f'(stop <{self._stop_distance:.2f}m, resume >{self._clear_distance:.2f}m).')

    def _declare_parameters(self):
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ai_cmd_topic', '/ai_cmd_vel')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('cmd_stamped', True)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('ai_cmd_timeout', 0.7)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('stop_distance', 0.30)
        self.declare_parameter('clear_distance', 0.40)
        self.declare_parameter('front_half_angle', 20.0)
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('reverse_speed_limit', 0.1)
        self.declare_parameter('allow_reverse_in_emergency', True)
        self.declare_parameter('allow_rotate_in_emergency', True)
        self.declare_parameter('passthrough_mode', False)

    def _load_parameters(self):
        get = self.get_parameter
        self._scan_topic = get('scan_topic').value
        self._ai_cmd_topic = get('ai_cmd_topic').value
        self._cmd_topic = get('cmd_topic').value
        self._cmd_stamped = bool(get('cmd_stamped').value)
        self._control_frequency = max(1.0, float(get('control_frequency').value))
        self._ai_cmd_timeout = float(get('ai_cmd_timeout').value)
        self._scan_timeout = float(get('scan_timeout').value)
        self._stop_distance = float(get('stop_distance').value)
        self._clear_distance = float(get('clear_distance').value)
        self._front_half_angle = float(get('front_half_angle').value)
        self._max_linear = float(get('max_linear').value)
        self._max_angular = float(get('max_angular').value)
        self._reverse_speed_limit = float(get('reverse_speed_limit').value)
        self._allow_reverse = bool(get('allow_reverse_in_emergency').value)
        self._allow_rotate = bool(get('allow_rotate_in_emergency').value)
        self._passthrough = bool(get('passthrough_mode').value)

        if self._clear_distance < self._stop_distance:
            self.get_logger().warn(
                'clear_distance is below stop_distance, which disables hysteresis; '
                'raising it to stop_distance.')
            self._clear_distance = self._stop_distance

    def _scan_callback(self, msg: LaserScan):
        self._scan_stamp = self.get_clock().now()

        front = sector_min_distance(msg, 0.0, self._front_half_angle)
        was_emergency = self._emergency

        if front is None:
            # No usable return ahead. Treat a blind sector as blocked, never as clear.
            self._emergency = True
        elif front < self._stop_distance:
            self._emergency = True
        elif front > self._clear_distance:
            self._emergency = False

        if self._emergency and not was_emergency:
            reading = 'no valid return' if front is None else f'{front:.2f}m'
            self.get_logger().warn(f'EMERGENCY STOP: obstacle ahead ({reading}).')
            # Brake on the scan itself rather than waiting for the next timer tick.
            self._publish(self._emergency_command())
        elif was_emergency and not self._emergency:
            self.get_logger().info(f'Path clear ({front:.2f}m); resuming AI control.')

    def _ai_cmd_callback(self, msg: Twist):
        self._latest_ai_cmd = msg
        self._ai_cmd_stamp = self.get_clock().now()

    def _control_loop(self):
        now = self.get_clock().now()
        command = Twist()

        if self._is_stale(self._scan_stamp, now, self._scan_timeout):
            self.get_logger().warn(
                'LiDAR silent; holding stop.', throttle_duration_sec=2.0)
        elif self._is_stale(self._ai_cmd_stamp, now, self._ai_cmd_timeout):
            self.get_logger().warn(
                f'No {self._ai_cmd_topic} within {self._ai_cmd_timeout:.1f}s; holding stop.',
                throttle_duration_sec=2.0)
        elif self._emergency:
            command = self._emergency_command()
        else:
            command = clamp_twist(self._latest_ai_cmd, self._max_linear, self._max_angular)

        self._publish(command)

    def _emergency_command(self) -> Twist:
        """Block forward motion while leaving the escape routes open."""
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
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.twist = command
            self._cmd_publisher.publish(stamped)
        else:
            self._cmd_publisher.publish(command)

    @staticmethod
    def _is_stale(stamp, now, timeout: float) -> bool:
        if stamp is None:
            return True
        return (now - stamp).nanoseconds * 1e-9 > timeout

    def stop_robot(self):
        """Flush zero velocity so the base controller does not latch the last command."""
        self._timer.cancel()
        if not rclpy.ok():
            # The context is already torn down, so publishing would raise. Nothing
            # can reach the base from here.
            self.get_logger().warn('Context already shut down; could not flush a stop.')
            return
        # Repeat with a short gap so the message survives a dropped packet and DDS
        # has time to flush before the process exits.
        for _ in range(3):
            self._publish(Twist())
            time.sleep(0.02)


def main(args=None):
    # rclpy's own SIGINT handler tears the context down before the finally block
    # runs, which makes the shutdown stop above impossible. Take the signal
    # ourselves so the context is still valid when we flush it.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_shutdown_signals()
    node = SafetyNode()
    try:
        # Spin in slices so SIGINT is delivered promptly instead of being swallowed
        # by a blocking wait inside rcl.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
