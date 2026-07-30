"""反应式安全层，同时是 /cmd_vel 的唯一发布者。

AI 规划器的推理周期在秒级，远慢于机器人陷入危险所需的时间，所以速度输出的所有权
必须握在这个快速回路手里，由它以固定频率持续发布。任何一路输入断流——规划器崩溃、
Ollama 卡住、网络掉线、雷达静默——都会在一个超时周期内衰减到停车，而不是把最后
一条速度指令永久锁存在底盘控制器里。

"唯一发布者"这条不变式很重要：如果别的节点也往同一个话题发，底盘收到的将是两路
指令的交错，这里的限幅与急停也就形同虚设。需要手动遥控时，请用 passthrough_mode
让本节点闭嘴，而不是再起一个发布者。
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
    """看门狗 AI 指令流，并强制执行硬性停车区。"""

    def __init__(self):
        super().__init__('safety_node')

        self._declare_parameters()
        self._load_parameters()

        # 初始状态即为停车：还没收到任何 AI 指令时，输出的就是零速度。
        self._latest_ai_cmd = Twist()
        self._ai_cmd_stamp = None
        self._scan_stamp = None
        self._emergency = False

        # 输出消息类型在启动时定下来：Gazebo Sim 8 / ROS 2 Jazzy 的桥接端订阅
        # TwistStamped，而多数实车底盘驱动仍订阅传统的 Twist。
        output_type = TwistStamped if self._cmd_stamped else Twist
        self._cmd_publisher = self.create_publisher(output_type, self._cmd_topic, 10)
        # 传感器话题用 BEST_EFFORT 的传感器 QoS：它与 RELIABLE、BEST_EFFORT 两种
        # 发布者都兼容。用默认 QoS 遇到 BEST_EFFORT 发布者会静默收不到数据——
        # 节点不报错、不动、也没有日志，很难查。
        self.create_subscription(
            LaserScan, self._scan_topic, self._scan_callback, qos_profile_sensor_data)
        # /ai_cmd_vel 是本进程组内部话题，用默认的可靠 QoS 即可。
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
        # 话题名。换机器人时通常只需要改这三个。
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ai_cmd_topic', '/ai_cmd_vel')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('cmd_stamped', True)

        # 本节点的发布频率，也就是底盘看到的指令刷新率。
        self.declare_parameter('control_frequency', 20.0)

        # 两个看门狗超时。ai_cmd_timeout 要对齐 ai_nav_node 的心跳频率
        # （command_publish_rate），而不是它的推理周期。
        self.declare_parameter('ai_cmd_timeout', 0.7)
        self.declare_parameter('scan_timeout', 0.5)

        # 带迟滞的停车区，以及用于判定的前方扇区半宽。
        self.declare_parameter('stop_distance', 0.30)
        self.declare_parameter('clear_distance', 0.40)
        self.declare_parameter('front_half_angle', 20.0)

        # 权威速度上限：无论上游要求多快，都以这里为准。
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('max_angular', 1.5)

        # 急停期间的逃生策略。
        self.declare_parameter('reverse_speed_limit', 0.1)
        self.declare_parameter('allow_reverse_in_emergency', True)
        self.declare_parameter('allow_rotate_in_emergency', True)

        # 置 true 后本节点完全不发布，用于手动遥控。
        self.declare_parameter('passthrough_mode', False)

    def _load_parameters(self):
        get = self.get_parameter
        self._scan_topic = get('scan_topic').value
        self._ai_cmd_topic = get('ai_cmd_topic').value
        self._cmd_topic = get('cmd_topic').value
        self._cmd_stamped = bool(get('cmd_stamped').value)
        # 下限兜到 1 Hz：频率被配成 0 会让 create_timer 直接除零。
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

        # 恢复阈值低于停车阈值会让迟滞失效，机器人在边界上反复启停。
        # 这里就地修正并告警，而不是让它带着坏配置跑起来。
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
            # 前方没有任何可用回波。盲区一律按不可通行处理，绝不按空旷处理。
            self._emergency = True
        elif front < self._stop_distance:
            self._emergency = True
        elif front > self._clear_distance:
            self._emergency = False
        # 落在两个阈值之间时保持原状态不变——这就是迟滞，用来抑制边界抖动。

        if self._emergency and not was_emergency:
            reading = 'no valid return' if front is None else f'{front:.2f}m'
            self.get_logger().warn(f'EMERGENCY STOP: obstacle ahead ({reading}).')
            # 直接在扫描回调里刹车，而不是等下一个定时器周期。20 Hz 下那最多也就
            # 50 ms，但这 50 ms 正好发生在离障碍物最近的时候。
            self._publish(self._emergency_command())
        elif was_emergency and not self._emergency:
            self.get_logger().info(f'Path clear ({front:.2f}m); resuming AI control.')

    def _ai_cmd_callback(self, msg: Twist):
        # 只留最新一条并记下到达时间。这里不做限幅——限幅统一在发布前做，
        # 保证急停逃生路径用的也是同一套上限。
        self._latest_ai_cmd = msg
        self._ai_cmd_stamp = self.get_clock().now()

    def _control_loop(self):
        now = self.get_clock().now()
        # 默认输出零速度：下面每个分支要么明确覆盖它，要么就地停车。
        # 这样新增分支时"忘了赋值"的后果是停车，而不是保持上一条速度。
        command = Twist()

        # 检查顺序即优先级，也让日志能指向真正的根因：雷达没数据时，AI 指令
        # 必然也是不可信的，此时报"雷达静默"比报"没有 AI 指令"更有用。
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
        """急停状态下的逃生策略：禁止前进，但保留后退与原地转向。

        完全归零会让机器人在墙角里彻底卡死——前方受阻、又不允许动，就再也出不来。
        所以这里放行两条出路，同时无论上游要求什么，前进方向永远不放行。
        """
        command = Twist()
        source = self._latest_ai_cmd

        # 只有上游本来就想后退时才放行，这里不会自己造出一个后退指令。
        # 后退另有一个更低的上限：视野里看不到的方向要走得更慢。
        if self._allow_reverse and source.linear.x < 0.0:
            command.linear.x = clamp(source.linear.x, self._reverse_speed_limit)
        if self._allow_rotate:
            command.angular.z = clamp(source.angular.z, self._max_angular)

        return command

    def _publish(self, command: Twist):
        # 透传模式下彻底不发布，把话题让给 teleop 等外部工具。
        # 注意此时机器人处于无保护状态。
        if self._passthrough:
            return

        if self._cmd_stamped:
            stamped = TwistStamped()
            # 用本节点的时钟盖时间戳（use_sim_time 为真时即仿真时钟），
            # 这样下游按时间戳做过期判断时与本节点的判断保持一致。
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.twist = command
            self._cmd_publisher.publish(stamped)
        else:
            self._cmd_publisher.publish(command)

    @staticmethod
    def _is_stale(stamp, now, timeout: float) -> bool:
        # 从未收到过（stamp 为 None）与很久没收到过，对安全而言是同一件事。
        if stamp is None:
            return True
        return (now - stamp).nanoseconds * 1e-9 > timeout

    def stop_robot(self):
        """刷出零速度，避免底盘控制器把最后一条指令一直锁存下去。"""
        # 先停掉定时器，否则它会和下面的停车指令抢着发布。
        self._timer.cancel()
        if not rclpy.ok():
            # context 已经拆掉了，再发布会抛异常。此时已经没有任何办法把指令
            # 送到底盘，只能留个日志说明为什么没刹车。
            self.get_logger().warn('Context already shut down; could not flush a stop.')
            return
        # 重复三次并留出间隔：既能扛住一次丢包，也给 DDS 留出时间在进程退出前
        # 把队列真正发出去。
        for _ in range(3):
            self._publish(Twist())
            time.sleep(0.02)


def main(args=None):
    # rclpy 自带的 SIGINT 处理器会在 finally 块运行之前就把 context 拆掉，
    # 上面 stop_robot 里的刹车动作就再也发不出去了。所以这里自己接管信号，
    # 保证刷停车指令时 context 仍然有效。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_shutdown_signals()
    node = SafetyNode()
    try:
        # 分片自旋而不是 spin()：让 SIGINT 能及时送达，不被 rcl 内部的阻塞等待吞掉。
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 顺序固定：先刹车，再销毁节点，最后关 context。
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
