"""Navigation planner: deterministic LiDAR policy tempered by a vision model.

The velocity itself is computed by navigator.py from LiDAR sector distances. The
local Ollama model is asked only for a semantic read of the camera frame, which
can slow the robot, mark the path ahead as blocked, or break a left/right tie -
never raise a speed. If the model is unavailable the robot keeps navigating on
LiDAR alone. Output is still advisory: safety_node arbitrates before /cmd_vel.
"""

import base64
import threading
import time

import cv2
import rclpy
import requests
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, LaserScan

from ai_robot_nav.lifecycle import install_shutdown_signals
from ai_robot_nav.llm_client import DecisionError, OllamaClient, parse_assessment
from ai_robot_nav.motion import clamp
from ai_robot_nav.navigator import NavConfig, plan
from ai_robot_nav.scan_utils import describe_environment, format_distance

SYSTEM_PROMPT = """你是移动机器人的视觉安全观察员。你不负责计算速度。
机器人的转向和速度由激光雷达数据确定性地计算，你的任务只是判断
摄像头画面里那些激光雷达看不到的情况。

只输出如下 JSON，不要任何多余文字：
{"hazard": "NONE", "preferred_direction": "STRAIGHT", "description": "走廊空旷"}

hazard 取值（只能三选一）：
- NONE:    画面正常，无需干预
- CAUTION: 存在应当减速的情况，例如行人、宠物、狭窄通道、反光或湿滑地面、光线昏暗
- BLOCKED: 前方明确不可通行，例如贴脸的墙面、关闭的门、玻璃隔断、楼梯口或下沉台阶

preferred_direction 取值：LEFT / RIGHT / STRAIGHT / NONE
仅在左右两侧空间相近时用作参考。不确定就填 NONE。

重要：你的判断只会让机器人更保守（减速或停止），不会让它加速，
也不会让它驶向激光雷达判定为封闭的方向。宁可保守，不要冒进。
"""


class AINavNode(Node):
    """Publishes advisory velocity commands derived from a local VLM."""

    def __init__(self):
        super().__init__('ai_nav_node')

        self._declare_parameters()
        self._load_parameters()

        self._bridge = CvBridge()
        self._client = OllamaClient(
            url=self._ollama_url,
            model=self._ollama_model,
            timeout=self._ollama_timeout,
            keep_alive=self._keep_alive,
            num_predict=self._num_predict,
            temperature=self._temperature,
        )

        self._lock = threading.Lock()
        self._scan = None
        self._scan_stamp = None
        self._image = None
        self._image_stamp = None

        self._assessment_lock = threading.Lock()
        self._assessment = None
        self._assessment_stamp = None
        self._last_log_signature = None
        self._last_log_time = 0.0

        self._cmd_publisher = self.create_publisher(Twist, self._cmd_topic, 10)
        self.create_subscription(
            LaserScan, self._scan_topic, self._scan_callback, qos_profile_sensor_data)
        if self._use_image:
            self.create_subscription(
                Image, self._image_topic, self._image_callback, qos_profile_sensor_data)

        # This fast deterministic loop never waits for Ollama. The model only
        # refreshes an optional semantic hint in the background.
        self._control_timer = self.create_timer(
            1.0 / self._command_rate, self._control_loop)

        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

        if not self._use_image and not self._lidar_only_fallback:
            self.get_logger().error(
                'use_image is off while lidar_only_fallback is also off, so no '
                'assessment can ever arrive and the robot will never move. '
                'Enable one of them.')

        self.get_logger().info(
            f'AI nav node up: model={self._ollama_model} '
            f'period={self._period:.1f}s vision={"on" if self._use_image else "off"} '
            f'-> {self._cmd_topic}')

    def _declare_parameters(self):
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_topic', '/ai_cmd_vel')
        self.declare_parameter('ollama_url', 'http://localhost:11434/api/generate')
        self.declare_parameter('ollama_model', 'llava:7b')
        self.declare_parameter('ollama_timeout', 30.0)
        self.declare_parameter('keep_alive', '30m')
        self.declare_parameter('num_predict', 128)
        self.declare_parameter('temperature', 0.1)
        self.declare_parameter('inference_period', 1.0)
        self.declare_parameter('command_publish_rate', 10.0)
        self.declare_parameter('vision_ttl', 3.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('sensor_timeout', 1.0)
        self.declare_parameter('use_image', True)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('front_half_angle', 30.0)
        self.declare_parameter('side_center_angle', 90.0)
        self.declare_parameter('side_half_angle', 30.0)
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('max_angular', 1.5)

        self.declare_parameter('cruise_speed', 0.18)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('reverse_speed', 0.08)
        self.declare_parameter('forward_clearance', 0.6)
        self.declare_parameter('trapped_distance', 0.4)
        self.declare_parameter('tie_threshold', 0.3)
        self.declare_parameter('caution_scale', 0.5)
        self.declare_parameter('lidar_only_fallback', True)

    def _load_parameters(self):
        get = self.get_parameter
        self._scan_topic = get('scan_topic').value
        self._image_topic = get('image_topic').value
        self._cmd_topic = get('cmd_topic').value
        self._ollama_url = get('ollama_url').value
        self._ollama_model = get('ollama_model').value
        self._ollama_timeout = float(get('ollama_timeout').value)
        self._keep_alive = get('keep_alive').value
        self._num_predict = int(get('num_predict').value)
        self._temperature = float(get('temperature').value)
        self._period = max(0.1, float(get('inference_period').value))
        self._command_rate = max(1.0, float(get('command_publish_rate').value))
        self._vision_ttl = max(0.0, float(get('vision_ttl').value))
        self._max_retries = max(0, int(get('max_retries').value))
        self._sensor_timeout = float(get('sensor_timeout').value)
        self._use_image = bool(get('use_image').value)
        self._image_width = int(get('image_width').value)
        self._image_height = int(get('image_height').value)
        self._jpeg_quality = int(get('jpeg_quality').value)
        self._front_half_angle = float(get('front_half_angle').value)
        self._side_center_angle = float(get('side_center_angle').value)
        self._side_half_angle = float(get('side_half_angle').value)
        self._max_linear = float(get('max_linear').value)
        self._max_angular = float(get('max_angular').value)
        self._lidar_only_fallback = bool(get('lidar_only_fallback').value)

        self._nav_config = NavConfig(
            cruise_speed=float(get('cruise_speed').value),
            turn_speed=float(get('turn_speed').value),
            reverse_speed=float(get('reverse_speed').value),
            forward_clearance=float(get('forward_clearance').value),
            trapped_distance=float(get('trapped_distance').value),
            tie_threshold=float(get('tie_threshold').value),
            caution_scale=float(get('caution_scale').value),
        )

    def _scan_callback(self, msg: LaserScan):
        with self._lock:
            self._scan = msg
            self._scan_stamp = self.get_clock().now()

    def _image_callback(self, msg: Image):
        # Only the raw message is kept here. Decoding and JPEG encoding happen in
        # the inference loop so the executor thread is not doing 30 fps of work
        # for a consumer that runs below 1 Hz, which would delay scan callbacks.
        with self._lock:
            self._image = msg
            self._image_stamp = self.get_clock().now()

    def _inference_loop(self):
        while not self._stop_event.is_set() and rclpy.ok():
            started = time.monotonic()
            try:
                self._vision_step()
            except Exception as exc:
                # Vision is optional. Never let a failed request or decoder stop
                # the deterministic LiDAR control loop.
                self.get_logger().error(f'Vision cycle failed: {exc}')

            remaining = self._period - (time.monotonic() - started)
            if remaining > 0.0:
                self._stop_event.wait(remaining)

    def _vision_step(self):
        if not self._use_image:
            return

        scan, scan_stamp, image, image_stamp = self._snapshot()
        now = self.get_clock().now()
        if scan is None or self._age(scan_stamp, now) > self._sensor_timeout:
            return
        if image is None or self._age(image_stamp, now) > self._sensor_timeout:
            self.get_logger().warn(
                'Camera frame missing or stale; running on LiDAR only.',
                throttle_duration_sec=5.0)
            return

        front, left, right = describe_environment(
            scan, self._front_half_angle, self._side_center_angle, self._side_half_angle)
        image_b64 = self._encode_image(image)
        if image_b64 is None:
            return

        assessment = self._request_assessment(
            self._build_prompt(front, left, right), image_b64)
        if assessment is not None:
            # Timestamp the hint with the frame it describes, not request
            # completion. A slow response must not make an old image look fresh.
            with self._assessment_lock:
                self._assessment = assessment
                self._assessment_stamp = image_stamp

    def _control_loop(self):
        """Compute and publish a fresh LiDAR decision without ever waiting for AI."""
        scan, scan_stamp, _, _ = self._snapshot()
        now = self.get_clock().now()

        if scan is None or self._age(scan_stamp, now) > self._sensor_timeout:
            self.get_logger().warn(
                'LiDAR data missing or stale; commanding stop.',
                throttle_duration_sec=5.0)
            self._publish_stop()
            return

        front, left, right = describe_environment(
            scan, self._front_half_angle, self._side_center_angle, self._side_half_angle)
        assessment = self._fresh_assessment(now)

        if assessment is None and not self._lidar_only_fallback:
            self.get_logger().error(
                'No fresh vision assessment and lidar_only_fallback is off; stopping.',
                throttle_duration_sec=5.0)
            self._publish_stop()
            return

        decision = plan(front, left, right, self._nav_config, assessment)

        command = Twist()
        command.linear.x = clamp(decision.linear_x, self._max_linear)
        command.angular.z = clamp(decision.angular_z, self._max_angular)
        self._cmd_publisher.publish(command)

        vision = 'lidar-only' if assessment is None else assessment.hazard
        signature = (
            decision.action, round(command.linear.x, 3),
            round(command.angular.z, 3), vision)
        wall_now = time.monotonic()
        if signature != self._last_log_signature or wall_now - self._last_log_time >= 2.0:
            self.get_logger().info(
                f'{decision.action} lin={command.linear.x:.2f} '
                f'ang={command.angular.z:.2f} [vision={vision}] | {decision.reason}')
            self._last_log_signature = signature
            self._last_log_time = wall_now

    def _snapshot(self):
        """Take a consistent copy of both sensor messages and receipt times."""
        with self._lock:
            scan, scan_stamp = self._scan, self._scan_stamp
            image, image_stamp = self._image, self._image_stamp
        return scan, scan_stamp, image, image_stamp

    def _fresh_assessment(self, now):
        with self._assessment_lock:
            assessment, stamp = self._assessment, self._assessment_stamp

        if assessment is None or self._age(stamp, now) > self._vision_ttl:
            return None
        return assessment

    @staticmethod
    def _age(stamp, now) -> float:
        if stamp is None:
            return float('inf')
        return (now - stamp).nanoseconds * 1e-9

    def _build_prompt(self, front, left, right) -> str:
        return (
            '激光雷达读数（供你参考，你不需要据此计算速度）：\n'
            f'- 前方: {format_distance(front)}\n'
            f'- 左侧: {format_distance(left)}\n'
            f'- 右侧: {format_distance(right)}\n'
            '\n请判断摄像头画面中是否存在激光雷达无法察觉的风险。'
        )

    def _encode_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.resize(frame, (self._image_width, self._image_height))
            encoded, buffer = cv2.imencode(
                '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
            if not encoded:
                raise RuntimeError('cv2.imencode reported failure')
            return base64.b64encode(buffer).decode('utf-8')
        except Exception as exc:
            self.get_logger().error(f'Image encoding failed: {exc}')
            return None

    def _request_assessment(self, prompt: str, image_b64):
        attempts = self._max_retries + 1
        for attempt in range(1, attempts + 1):
            if self._stop_event.is_set():
                return None
            try:
                raw = self._client.generate(prompt, SYSTEM_PROMPT, image_b64)
            except requests.RequestException as exc:
                self.get_logger().error(
                    f'Ollama request failed ({attempt}/{attempts}): {exc}',
                    throttle_duration_sec=5.0)
                continue
            try:
                return parse_assessment(raw)
            except DecisionError as exc:
                self.get_logger().warn(
                    f'Rejected model output ({attempt}/{attempts}): {exc}',
                    throttle_duration_sec=5.0)
        return None

    def _publish_stop(self):
        self._cmd_publisher.publish(Twist())

    def shutdown(self):
        self._stop_event.set()
        self._control_timer.cancel()
        # Short join: a worker blocked in an HTTP call would otherwise hold up
        # shutdown for the full request timeout. It is a daemon thread, so the
        # process can exit without it.
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)

        if rclpy.ok():
            # Published directly: the control timer no longer runs at this point.
            for _ in range(3):
                self._cmd_publisher.publish(Twist())
                time.sleep(0.02)
        else:
            self.get_logger().warn('Context already shut down; could not flush a stop.')

        self._client.close()


def main(args=None):
    # rclpy's own SIGINT handler tears the context down before the finally block
    # runs, which makes the shutdown stop above impossible. Take the signal
    # ourselves so the context is still valid when we flush it.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_shutdown_signals()
    node = AINavNode()
    try:
        # Spin in slices so SIGINT is delivered promptly instead of being swallowed
        # by a blocking wait inside rcl.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
