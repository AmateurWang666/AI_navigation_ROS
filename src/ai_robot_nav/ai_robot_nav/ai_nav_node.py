"""导航规划节点：确定性激光策略 + 视觉模型的收敛性修正（ROS 1 / rospy 版）。"""

import base64
import threading
import time

import cv2
import requests
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan

from ai_robot_nav.lifecycle import install_shutdown_signals
from ai_robot_nav.llm_client import (
    DecisionError, OllamaClient, apply_caution_debounce, parse_assessment,
)
from ai_robot_nav.log_throttle import LogThrottle
from ai_robot_nav.motion import clamp
from ai_robot_nav.navigator import TURNING_ACTIONS, NavConfig, plan
from ai_robot_nav.ros_params import param, param_bool, param_float, param_int
from ai_robot_nav.scan_utils import describe_environment, format_distance

SYSTEM_PROMPT = """你是移动机器人的视觉安全观察员。你不负责计算速度。
机器人的转向和速度由激光雷达数据确定性地计算，你的任务只是判断
摄像头画面里那些激光雷达看不到的情况。

只输出如下 JSON，不要任何多余文字：
{"hazard": "NONE", "preferred_direction": "STRAIGHT", "description": "走廊空旷"}

hazard 取值（只能三选一）：
- NONE:    画面正常、通道可通行，无需干预（仿真走廊等空旷场景应选此项）
- CAUTION: 仅当明确看到需要减速的对象时才用，例如行人、宠物、湿滑地面
- BLOCKED: 前方明确不可通行，例如贴脸的墙面、关闭的门、玻璃隔断、楼梯口或下沉台阶

preferred_direction 取值：LEFT / RIGHT / STRAIGHT / NONE
仅在左右两侧空间相近时用作参考。不确定就填 NONE。

重要：你的判断只会让机器人更保守（减速或停止），不会让它加速，
也不会让它驶向激光雷达判定为封闭的方向。没有明确危险时务必填 NONE，
不要把正常行驶场景标成 CAUTION。
"""


class AINavNode:
    """发布建议速度：激光确定性决策，视觉模型仅作收敛性修正。"""

    def __init__(self):
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
        self._log = LogThrottle()

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
        self._last_plan_action = None
        self._caution_streak = 0
        self._last_vision_log = None

        # 转向状态。方向要跨越 FORWARD 阶段保留，否则一次避障中间只要有一帧
        # 直行，方向就被清空、下一帧重新选边，机器人便在原地摆头。
        self._turn_side = None
        self._turn_side_time = 0.0
        self._turn_started = None

        self._cmd_publisher = rospy.Publisher(self._cmd_topic, Twist, queue_size=10)
        self._scan_sub = rospy.Subscriber(
            self._scan_topic, LaserScan, self._scan_callback, queue_size=1)
        if self._use_image:
            self._image_sub = rospy.Subscriber(
                self._image_topic, Image, self._image_callback, queue_size=1)

        self._control_timer = rospy.Timer(
            rospy.Duration(1.0 / self._command_rate), self._control_loop)

        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

        if not self._use_image and not self._lidar_only_fallback:
            rospy.logerr(
                'use_image is off while lidar_only_fallback is also off, so no '
                'assessment can ever arrive and the robot will never move. '
                'Enable one of them.')

        rospy.loginfo(
            f'AI nav node up: model={self._ollama_model} '
            f'period={self._period:.1f}s vision={"on" if self._use_image else "off"} '
            f'-> {self._cmd_topic}')

    def _load_parameters(self):
        self._scan_topic = param('scan_topic', '/scan')
        self._image_topic = param('image_topic', '/camera/image_raw')
        self._cmd_topic = param('cmd_topic', '/ai_cmd_vel')
        self._ollama_url = param('ollama_url', 'http://localhost:11434/api/generate')
        self._ollama_model = param('ollama_model', 'llava:7b')
        self._ollama_timeout = param_float('ollama_timeout', 30.0)
        self._keep_alive = param('keep_alive', '30m')
        self._num_predict = param_int('num_predict', 128)
        self._temperature = param_float('temperature', 0.1)
        self._period = max(0.1, param_float('inference_period', 1.0))
        self._command_rate = max(1.0, param_float('command_publish_rate', 10.0))
        self._vision_ttl = max(0.0, param_float('vision_ttl', 3.0))
        self._max_retries = max(0, param_int('max_retries', 2))
        self._sensor_timeout = param_float('sensor_timeout', 1.0)
        self._use_image = param_bool('use_image', True)
        self._image_width = param_int('image_width', 320)
        self._image_height = param_int('image_height', 240)
        self._jpeg_quality = param_int('jpeg_quality', 60)
        self._front_half_angle = param_float('front_half_angle', 30.0)
        self._front_center_angle = param_float('front_center_angle', 0.0)
        self._side_center_angle = param_float('side_center_angle', 90.0)
        self._side_half_angle = param_float('side_half_angle', 30.0)
        self._max_linear = param_float('max_linear', 0.22)
        self._max_angular = param_float('max_angular', 1.5)
        self._lidar_only_fallback = param_bool('lidar_only_fallback', True)
        self._caution_confirmations = max(1, param_int('caution_confirmations', 2))
        self._turn_latch_timeout = max(0.0, param_float('turn_latch_timeout', 3.0))

        forward_clearance = param_float('forward_clearance', 0.6)
        turn_clearance = param_float('turn_clearance', 0.5)
        if turn_clearance >= forward_clearance:
            rospy.logwarn(
                'turn_clearance is not below forward_clearance, which disables '
                'hysteresis; lowering turn_clearance.')
            turn_clearance = forward_clearance - 0.05

        self._nav_config = NavConfig(
            cruise_speed=param_float('cruise_speed', 0.18),
            turn_speed=param_float('turn_speed', 0.5),
            reverse_speed=param_float('reverse_speed', 0.08),
            reverse_clearance=param_float('reverse_clearance', 0.4),
            forward_clearance=forward_clearance,
            turn_clearance=turn_clearance,
            trapped_distance=param_float('trapped_distance', 0.4),
            tie_threshold=param_float('tie_threshold', 0.3),
            caution_scale=param_float('caution_scale', 0.5),
            escape_commit_time=param_float('escape_commit_time', 4.0),
            escape_reverse_time=param_float('escape_reverse_time', 9.0),
        )

    def _scan_callback(self, msg: LaserScan):
        with self._lock:
            self._scan = msg
            self._scan_stamp = rospy.Time.now()

    def _image_callback(self, msg: Image):
        with self._lock:
            self._image = msg
            self._image_stamp = rospy.Time.now()

    def _inference_loop(self):
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            started = time.monotonic()
            try:
                self._vision_step()
            except Exception as exc:
                rospy.logerr(f'Vision cycle failed: {exc}')

            remaining = self._period - (time.monotonic() - started)
            if remaining > 0.0:
                self._stop_event.wait(remaining)

    def _vision_step(self):
        if not self._use_image:
            return

        scan, scan_stamp, image, image_stamp = self._snapshot()
        now = rospy.Time.now()
        if scan is None or self._age(scan_stamp, now) > self._sensor_timeout:
            return
        if image is None or self._age(image_stamp, now) > self._sensor_timeout:
            self._log.warn(
                'camera_stale',
                'Camera frame missing or stale; running on LiDAR only.',
                period=5.0)
            return

        front, left, right, _rear = describe_environment(
            scan, self._front_half_angle, self._side_center_angle, self._side_half_angle,
            self._front_center_angle)
        image_b64 = self._encode_image(image)
        if image_b64 is None:
            return

        assessment = self._request_assessment(
            self._build_prompt(front, left, right), image_b64)
        if assessment is not None:
            assessment, self._caution_streak = apply_caution_debounce(
                assessment, self._caution_streak, self._caution_confirmations)
            vision_key = (assessment.hazard, assessment.description)
            if vision_key != self._last_vision_log:
                rospy.loginfo(
                    f'Vision assessment: {assessment.hazard} | {assessment.description}')
                self._last_vision_log = vision_key
            with self._assessment_lock:
                self._assessment = assessment
                self._assessment_stamp = image_stamp

    def _control_loop(self, _event):
        scan, scan_stamp, _, _ = self._snapshot()
        now = rospy.Time.now()

        if scan is None or self._age(scan_stamp, now) > self._sensor_timeout:
            self._log.warn(
                'lidar_stale',
                'LiDAR data missing or stale; commanding stop.',
                period=5.0)
            self._publish_stop()
            return

        front, left, right, rear = describe_environment(
            scan, self._front_half_angle, self._side_center_angle, self._side_half_angle,
            self._front_center_angle)
        assessment = self._fresh_assessment(now)

        if assessment is None and not self._lidar_only_fallback:
            self._log.error(
                'no_vision',
                'No fresh vision assessment and lidar_only_fallback is off; stopping.',
                period=5.0)
            self._publish_stop()
            return

        # 转向计时走 ROS 时间而非墙钟：仿真里 Gazebo 常慢于实时，用墙钟会让
        # escape_* 两个阈值对应的实际转角比配置的更小，仿真与实车行为就对不上了。
        ros_now = now.to_sec()
        decision = plan(
            front, left, right, self._nav_config, assessment,
            last_turn=self._turn_side, last_action=self._last_plan_action,
            rear=rear, turn_elapsed=self._turn_elapsed(ros_now))

        self._last_plan_action = decision.action
        self._track_turn_state(decision.action, ros_now)

        command = Twist()
        command.linear.x = clamp(decision.linear_x, self._max_linear)
        command.angular.z = clamp(decision.angular_z, self._max_angular)
        self._cmd_publisher.publish(command)

        vision = 'lidar-only' if assessment is None else assessment.hazard
        signature = (
            decision.action, round(command.linear.x, 3),
            round(command.angular.z, 3), vision)
        # 日志节流用墙钟：仿真暂停时 ROS 时间不走，否则日志会跟着一起冻住。
        wall_now = time.monotonic()
        if signature != self._last_log_signature or wall_now - self._last_log_time >= 2.0:
            rospy.loginfo(
                f'{decision.action} lin={command.linear.x:.2f} '
                f'ang={command.angular.z:.2f} [vision={vision}] | {decision.reason}')
            self._last_log_signature = signature
            self._last_log_time = wall_now

    def _turn_elapsed(self, ros_now: float) -> float:
        """已经**连续**转向了多少秒。中间只要有一帧不是转向就归零。

        刻意只统计连续转向：脱困动作会短暂后退，代价不小，只应在「明明一直在转却
        转不出去」时触发。转向与直行交替出现说明机器人在正常绕障、确有位移，那种
        情况不该按卡死处理。
        """
        if self._turn_started is None:
            return 0.0
        return ros_now - self._turn_started

    def _track_turn_state(self, action: str, ros_now: float):
        """维护转向计时与方向锁存。

        方向锁存与上面的计时不同，它刻意跨越直行帧，停止转向后再保留
        ``turn_latch_timeout`` 秒：一次避障往往是「转一点 → 前方够宽了就直行 →
        又不够了继续转」，锁存必须活过中间的直行帧，否则每次都重新选边，左右读数
        一变机器人就改主意，于是原地摆头。超时后清空，避免几分钟前的一次转向长期
        给策略施加偏置。
        """
        if action in TURNING_ACTIONS:
            if self._turn_started is None:
                self._turn_started = ros_now
            self._turn_side = action
            self._turn_side_time = ros_now
            return

        self._turn_started = None
        if (self._turn_side is not None
                and ros_now - self._turn_side_time > self._turn_latch_timeout):
            self._turn_side = None

    def _snapshot(self):
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
        return (now - stamp).to_sec()

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
            rospy.logerr(f'Image encoding failed: {exc}')
            return None

    def _request_assessment(self, prompt: str, image_b64):
        attempts = self._max_retries + 1
        for attempt in range(1, attempts + 1):
            if self._stop_event.is_set():
                return None
            try:
                raw = self._client.generate(prompt, SYSTEM_PROMPT, image_b64)
            except requests.RequestException as exc:
                self._log.error(
                    'ollama_fail',
                    f'Ollama request failed ({attempt}/{attempts}): {exc}. '
                    'Is "ollama serve" running and is the model pulled?',
                    period=5.0)
                continue
            try:
                return parse_assessment(raw)
            except DecisionError as exc:
                self._log.warn(
                    'bad_output',
                    f'Rejected model output ({attempt}/{attempts}): {exc}',
                    period=5.0)
        return None

    def _publish_stop(self):
        self._cmd_publisher.publish(Twist())

    def shutdown(self):
        self._stop_event.set()
        self._control_timer.shutdown()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)

        if not rospy.is_shutdown():
            for _ in range(3):
                self._cmd_publisher.publish(Twist())
                time.sleep(0.02)
        else:
            rospy.logwarn('Node already shutting down; could not flush a stop.')

        self._client.close()


def main():
    install_shutdown_signals()
    rospy.init_node('ai_nav_node')
    node = AINavNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
