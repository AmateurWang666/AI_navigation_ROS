"""导航规划节点：确定性激光策略 + 视觉模型的收敛性修正。

速度本身由 navigator.py 依据激光扇区距离算出。本地 Ollama 模型只被问一个问题：
摄像头画面里有什么是雷达看不到的。它的回答可以让机器人减速、可以把前方判为不可
通行、可以在左右空间相近时打破平局——但永远无法提高任何速度。模型不可用时机器人
照常按纯激光导航。

节点内部有两条独立的执行流，这是本文件的核心结构：

- 控制回路（ROS 定时器，command_publish_rate 驱动）：纯激光、不碰网络，因此永远
  不会被 Ollama 拖慢。
- 视觉线程（独立 Python 线程，inference_period 驱动）：异步刷新一个可选的语义
  提示；提示超过 vision_ttl 就作废，控制立刻退回纯激光。

两者必须分线程。Ollama 请求可能耗时数秒、断连，或一直等到 HTTP 超时；若与激光
规划同线程，即使开了 lidar_only_fallback，阻塞期间也无法真正降级。

本节点的输出仍然只是建议：最终由 safety_node 仲裁后才发到 /cmd_vel。
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
from ai_robot_nav.llm_client import (
    DecisionError, OllamaClient, apply_caution_debounce, parse_assessment,
)
from ai_robot_nav.motion import clamp
from ai_robot_nav.navigator import NavConfig, plan
from ai_robot_nav.scan_utils import describe_environment, format_distance

# 系统提示词。三点是刻意写进提示的：明确告诉模型它不负责算速度（降低它输出速度
# 字段的倾向）、把 hazard 限定为三个枚举值（便于严格校验）、以及说明它的判断只会
# 让机器人更保守（与代码里的接线方式一致，减少模型"想帮忙加速"的冲动）。
# 即便如此，提示词只是第一道防线，真正的保证在 llm_client 的校验和 navigator 的接线。
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


class AINavNode(Node):
    """发布建议速度：激光确定性决策，视觉模型仅作收敛性修正。"""

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

        # 传感器缓存由 _lock 保护：写在 ROS 执行器线程（回调），读在视觉线程和
        # 控制回路两处。只存最新一帧，历史数据对反应式策略没有价值。
        self._lock = threading.Lock()
        self._scan = None
        self._scan_stamp = None
        self._image = None
        self._image_stamp = None

        # 视觉提示单独一把锁：它由视觉线程写、控制回路读，与传感器缓存的争用关系
        # 不同。共用一把锁会让慢速的视觉侧和 10 Hz 的控制侧互相等待。
        self._assessment_lock = threading.Lock()
        self._assessment = None
        self._assessment_stamp = None
        # 日志去重状态：决策不变时不重复刷屏，但至少每 2 秒留一条，
        # 便于确认节点还活着。
        self._last_log_signature = None
        self._last_log_time = 0.0
        # 上一周期动作，供 navigator 做直行/转向迟滞与转向方向保持。
        self._last_plan_action = None
        self._caution_streak = 0
        self._last_vision_log = None

        self._cmd_publisher = self.create_publisher(Twist, self._cmd_topic, 10)
        # 传感器话题用 BEST_EFFORT 的传感器 QoS，与两种发布者都兼容；用默认 QoS
        # 遇到 BEST_EFFORT 发布者会静默收不到数据，表现为节点不报错也不动。
        self.create_subscription(
            LaserScan, self._scan_topic, self._scan_callback, qos_profile_sensor_data)
        # use_image 关闭时连订阅都不建立，省掉整条图像传输与解码的开销。
        if self._use_image:
            self.create_subscription(
                Image, self._image_topic, self._image_callback, qos_profile_sensor_data)

        # 这条快速确定性回路永远不等 Ollama。模型只在后台刷新一个可选的语义提示。
        self._control_timer = self.create_timer(
            1.0 / self._command_rate, self._control_loop)

        # 守护线程 + Event：Event 让线程能在等待间隔中被立刻唤醒退出，
        # daemon 则保证即使它卡在 HTTP 请求里，进程也仍然能退出。
        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

        # 这两个开关同时关掉会让机器人永远拿不到评估从而一动不动。这属于配置错误，
        # 启动时就明确报出来，而不是让用户去猜为什么机器人不走。
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
        # 话题名。换机器人时通常只需要改这三个。
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_topic', '/ai_cmd_vel')

        # Ollama 后端。模型跑在另一台机器上时只改 ollama_url，不需要改源码。
        self.declare_parameter('ollama_url', 'http://localhost:11434/api/generate')
        self.declare_parameter('ollama_model', 'llava:7b')
        self.declare_parameter('ollama_timeout', 30.0)
        self.declare_parameter('keep_alive', '30m')
        self.declare_parameter('num_predict', 128)
        self.declare_parameter('temperature', 0.1)

        # 两条执行流的节拍：inference_period 属于视觉线程，
        # command_publish_rate 属于控制回路，两者互不影响。
        self.declare_parameter('inference_period', 1.0)
        self.declare_parameter('command_publish_rate', 10.0)
        # 视觉提示的保质期。超过则丢弃，控制自动退回纯激光。
        self.declare_parameter('vision_ttl', 3.0)

        self.declare_parameter('max_retries', 2)
        self.declare_parameter('sensor_timeout', 1.0)

        # 视觉输入。分辨率和 JPEG 质量都压得很低——模型只需要看清场景类别，
        # 传大图只会拖长推理时间。
        self.declare_parameter('use_image', True)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('jpeg_quality', 60)

        # 扇区几何，单位度。0 为正前方，正值为左（REP-103）。
        self.declare_parameter('front_half_angle', 30.0)
        self.declare_parameter('side_center_angle', 90.0)
        self.declare_parameter('side_half_angle', 30.0)

        # 第一道速度钳位；权威上限在 safety_node。
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('max_angular', 1.5)

        # 确定性策略参数——决定速度的是这些，不是模型。
        self.declare_parameter('cruise_speed', 0.18)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('reverse_speed', 0.08)
        self.declare_parameter('reverse_clearance', 0.4)
        self.declare_parameter('forward_clearance', 0.6)
        self.declare_parameter('turn_clearance', 0.5)
        self.declare_parameter('trapped_distance', 0.4)
        self.declare_parameter('tie_threshold', 0.3)
        self.declare_parameter('caution_scale', 0.5)
        self.declare_parameter('caution_confirmations', 2)
        # 模型不可用时是否继续按纯激光行驶。置 false 则改为停车。
        self.declare_parameter('lidar_only_fallback', True)

    def _load_parameters(self):
        # 参数只在启动时读一次并缓存成属性：控制回路每周期都要用，
        # 每次都走 get_parameter 是没必要的开销。
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
        # 下面几处 max()/min() 都是防止配置值把节点弄成除零或死循环。
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
        self._caution_confirmations = max(1, int(get('caution_confirmations').value))

        # 策略参数打包成不可变配置，之后整个运行期不再变化。
        forward_clearance = float(get('forward_clearance').value)
        turn_clearance = float(get('turn_clearance').value)
        if turn_clearance >= forward_clearance:
            self.get_logger().warn(
                'turn_clearance is not below forward_clearance, which disables '
                'hysteresis; lowering turn_clearance.')
            turn_clearance = forward_clearance - 0.05

        self._nav_config = NavConfig(
            cruise_speed=float(get('cruise_speed').value),
            turn_speed=float(get('turn_speed').value),
            reverse_speed=float(get('reverse_speed').value),
            reverse_clearance=float(get('reverse_clearance').value),
            forward_clearance=forward_clearance,
            turn_clearance=turn_clearance,
            trapped_distance=float(get('trapped_distance').value),
            tie_threshold=float(get('tie_threshold').value),
            caution_scale=float(get('caution_scale').value),
        )

    def _scan_callback(self, msg: LaserScan):
        # 记的是本地收到的时间，不是消息头里的时间戳：过期判断要衡量的是
        # "数据到我这儿有多久了"，用发送方时钟反而会引入时钟不同步的问题。
        with self._lock:
            self._scan = msg
            self._scan_stamp = self.get_clock().now()

    def _image_callback(self, msg: Image):
        # 这里只存原始消息。解码和 JPEG 编码放到视觉线程里做，否则执行器线程会为
        # 一个跑不到 1 Hz 的消费者承担 30 fps 的图像处理量，进而拖慢扫描回调。
        with self._lock:
            self._image = msg
            self._image_stamp = self.get_clock().now()

    def _inference_loop(self):
        """视觉线程主体：按 inference_period 尽力循环，绝不影响控制回路。"""
        while not self._stop_event.is_set() and rclpy.ok():
            started = time.monotonic()
            try:
                self._vision_step()
            except Exception as exc:
                # 视觉是可选项。这里兜住所有异常，绝不让一次失败的请求或解码错误
                # 把线程搞死——线程一死，视觉就永久失效，而且没有任何提示。
                self.get_logger().error(f'Vision cycle failed: {exc}')

            # 只睡掉"周期减去推理耗时"的余量，所以模型比周期慢时循环自然变慢，
            # 不会积压请求。用 Event.wait 而非 sleep，关闭时可被立刻唤醒。
            remaining = self._period - (time.monotonic() - started)
            if remaining > 0.0:
                self._stop_event.wait(remaining)

    def _vision_step(self):
        """取一帧图像做一次评估，成功则刷新共享的语义提示。"""
        if not self._use_image:
            return

        scan, scan_stamp, image, image_stamp = self._snapshot()
        now = self.get_clock().now()
        # 雷达数据都过期了就没必要推理：此时控制回路本来就在停车。
        if scan is None or self._age(scan_stamp, now) > self._sensor_timeout:
            return
        if image is None or self._age(image_stamp, now) > self._sensor_timeout:
            self.get_logger().warn(
                'Camera frame missing or stale; running on LiDAR only.',
                throttle_duration_sec=5.0)
            return

        # 把雷达读数一并写进提示词，让模型知道自己在看什么场景；但明确告诉它
        # 这只是参考，不需要据此算速度。
        front, left, right, _rear = describe_environment(
            scan, self._front_half_angle, self._side_center_angle, self._side_half_angle)
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
                self.get_logger().info(
                    f'Vision assessment: {assessment.hazard} | {assessment.description}')
                self._last_vision_log = vision_key
            # 时间戳取它所描述的那一帧图像的时间，而不是请求完成的时间。否则一次
            # 慢响应会让一张旧图看起来很新鲜，TTL 也就失去了意义。
            with self._assessment_lock:
                self._assessment = assessment
                self._assessment_stamp = image_stamp

    def _control_loop(self):
        """计算并发布一条新的激光决策，全程不等待 AI。"""
        scan, scan_stamp, _, _ = self._snapshot()
        now = self.get_clock().now()

        # 没有雷达就不许动。这条分支在 Gazebo 还没起来时会持续触发，
        # 日志里刷 stop 是安全设计生效，不是故障。
        if scan is None or self._age(scan_stamp, now) > self._sensor_timeout:
            self.get_logger().warn(
                'LiDAR data missing or stale; commanding stop.',
                throttle_duration_sec=5.0)
            self._publish_stop()
            return

        front, left, right, rear = describe_environment(
            scan, self._front_half_angle, self._side_center_angle, self._side_half_angle)
        assessment = self._fresh_assessment(now)

        # 关掉 fallback 意味着"没有视觉就不走"，用于必须依赖视觉的场景。
        if assessment is None and not self._lidar_only_fallback:
            self.get_logger().error(
                'No fresh vision assessment and lidar_only_fallback is off; stopping.',
                throttle_duration_sec=5.0)
            self._publish_stop()
            return

        # assessment 为 None 时 plan() 按纯激光决策，不需要另一条代码路径。
        decision = plan(
            front, left, right, self._nav_config, assessment,
            last_action=self._last_plan_action, rear=rear)

        self._last_plan_action = decision.action

        command = Twist()
        # 本地先钳一道。真正的权威上限在 safety_node，这里的作用是让本节点
        # 发出的话题内容本身就是可信的（便于单独 echo 调试）。
        command.linear.x = clamp(decision.linear_x, self._max_linear)
        command.angular.z = clamp(decision.angular_z, self._max_angular)
        self._cmd_publisher.publish(command)

        # 决策内容没变就不重复刷日志，但至少每 2 秒输出一条，用来确认回路还在跑。
        # 10 Hz 全量打印会把真正的状态变化淹掉。
        vision = 'lidar-only' if assessment is None else assessment.hazard
        signature = (
            decision.action, round(command.linear.x, 3),
            round(command.angular.z, 3), vision)
        # 这里用 monotonic 墙上时钟：日志节流是给人看的，不该跟着仿真时钟变快变慢。
        wall_now = time.monotonic()
        if signature != self._last_log_signature or wall_now - self._last_log_time >= 2.0:
            self.get_logger().info(
                f'{decision.action} lin={command.linear.x:.2f} '
                f'ang={command.angular.z:.2f} [vision={vision}] | {decision.reason}')
            self._last_log_signature = signature
            self._last_log_time = wall_now

    def _snapshot(self):
        """在一次加锁内取走两路传感器消息及其接收时间。

        一次取全而不是分两次加锁，保证同一轮决策看到的是一致的状态；也让锁的
        持有时间压到最短，不会阻塞传感器回调。
        """
        with self._lock:
            scan, scan_stamp = self._scan, self._scan_stamp
            image, image_stamp = self._image, self._image_stamp
        return scan, scan_stamp, image, image_stamp

    def _fresh_assessment(self, now):
        """返回仍在保质期内的视觉提示，过期或从未有过则返回 None。"""
        with self._assessment_lock:
            assessment, stamp = self._assessment, self._assessment_stamp

        # 过期的提示直接当作不存在。这就是"模型故障不会阻塞运动控制"的落点：
        # 视觉线程卡住后，最多 vision_ttl 秒控制就完全退回纯激光。
        if assessment is None or self._age(stamp, now) > self._vision_ttl:
            return None
        return assessment

    @staticmethod
    def _age(stamp, now) -> float:
        """数据已存在多久（秒）。从未收到过时返回 inf，让所有超时比较都成立。"""
        if stamp is None:
            return float('inf')
        return (now - stamp).nanoseconds * 1e-9

    def _build_prompt(self, front, left, right) -> str:
        """拼接单次请求的用户提示词。"""
        return (
            '激光雷达读数（供你参考，你不需要据此计算速度）：\n'
            f'- 前方: {format_distance(front)}\n'
            f'- 左侧: {format_distance(left)}\n'
            f'- 右侧: {format_distance(right)}\n'
            '\n请判断摄像头画面中是否存在激光雷达无法察觉的风险。'
        )

    def _encode_image(self, msg: Image):
        """把 ROS 图像消息转成 base64 JPEG，失败返回 None。

        缩放和压缩都在这里做：模型只需要看清场景类别，原图尺寸只会增加传输和
        推理时间。任何一步失败都退化为"这一轮没有视觉"，而不是抛给调用方。
        """
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
        """请求一次评估，必要时重试；始终返回评估或 None，不向外抛异常。

        两类失败刻意分开处理：网络错误记 error（服务多半有问题），模型输出不合格
        记 warn（模型不听话，但服务是好的）。两者都不会让机器人停下——控制回路
        此时仍在按纯激光行驶。
        """
        attempts = self._max_retries + 1
        for attempt in range(1, attempts + 1):
            # 每次重试前检查关闭标志：否则 max_retries 次数秒的超时会把
            # 进程退出拖上十几秒。
            if self._stop_event.is_set():
                return None
            try:
                raw = self._client.generate(prompt, SYSTEM_PROMPT, image_b64)
            except requests.RequestException as exc:
                self.get_logger().error(
                    f'Ollama request failed ({attempt}/{attempts}): {exc}. '
                    'Is "ollama serve" running and is the model pulled?',
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
        # 全零 Twist 即停车指令。
        self._cmd_publisher.publish(Twist())

    def shutdown(self):
        """有序关闭：停掉两条执行流，再刷出停车指令。"""
        self._stop_event.set()
        self._control_timer.cancel()
        # join 只等很短的时间：卡在 HTTP 调用里的工作线程否则会把关闭流程拖满整个
        # 请求超时。它是守护线程，进程可以不等它就退出。
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)

        if rclpy.ok():
            # 直接发布：此时控制定时器已经停了，不存在竞争。重复三次并留间隔，
            # 让停车指令能扛住丢包，也给 DDS 留出发送时间。
            for _ in range(3):
                self._cmd_publisher.publish(Twist())
                time.sleep(0.02)
        else:
            self.get_logger().warn('Context already shut down; could not flush a stop.')

        self._client.close()


def main(args=None):
    # rclpy 自带的 SIGINT 处理器会在 finally 块运行之前就把 context 拆掉，
    # 上面 shutdown 里的刹车动作也就发不出去了。所以这里自己接管信号，
    # 保证刷停车指令时 context 仍然有效。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_shutdown_signals()
    node = AINavNode()
    try:
        # 分片自旋而不是 spin()：让 SIGINT 能及时送达，不被 rcl 内部的阻塞等待吞掉。
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 顺序固定：先刹车与收线程，再销毁节点，最后关 context。
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
