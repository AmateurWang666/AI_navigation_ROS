# AI Robot Navigation Project Tasks

> 说明：原 v1 单机 Python 脚本架构（`lidar_driver.py`、`camera_driver.py`、`sensor_fusion.py`、`prompt_builder.py`、`qwen_client.py`、`decision_parser.py`、`safety_guard.py`、`serial_comm.py`、`motion_controller.py`、`main.py`、`firmware/esp32_motor_driver/`、`utils/logger.py` 等）在 `src/ai_robot_nav/` 下均不存在，功能已迁移至 ROS2 节点 `ai_nav_node.py` 与 `safety_node.py`。

## 阶段一：ROS2 包结构与 Launch
- [x] 创建 `ai_robot_nav` ROS2 Python 包（`setup.py`、`package.xml`）
- [x] 注册入口点 `ai_nav_node`、`safety_node`（`setup.py` entry_points）
- [x] 编写 `launch/ai_nav.launch.py`（同时启动双节点）
- [x] launch 暴露 `params_file` / `ollama_url` / `ollama_model` / `use_image` 参数
- [x] 参数集中至 `config/ai_nav_params.yaml`
- [x] `package.xml` 补齐 `python3-requests`、`python3-opencv` 等 rosdep 依赖

## 阶段二：AI 决策节点（`ai_robot_nav/ai_nav_node.py`）
- [x] 订阅 `/scan`（LaserScan）与 `/camera/image_raw`（Image）
- [x] 图像经 CvBridge 转为 BGR，缩放至 320x240 并 JPEG Base64 编码
- [x] 计算前/左/右扇区最近障碍物距离，拼接环境描述文本
- [x] 后台线程 `ai_loop` 定时 HTTP POST 至 Ollama `/api/generate`（`llava:7b`）
- [x] 解析模型返回 JSON（含 markdown 代码块剥离），发布 `Twist` 至 `/ai_cmd_vel`；解析失败时发布零速
- [x] 为 `latest_scan` / `latest_image` 添加 `threading.Lock`，避免回调与推理线程读写竞态
- [x] 图像编码从回调移入推理线程（回调只存原始消息），避免 30fps 无用编码阻塞激光回调
- [x] 传感器新鲜度检查：`/scan` 或图像超过 `sensor_timeout` 未更新即发零速
- [x] 将 Ollama URL、模型名、推理间隔、图像尺寸提取为 ROS 参数（便于 Jetson 切换局域网 IP）
- [x] Ollama 请求增加 `format: json` / `keep_alive` / `num_predict`，并复用 HTTP 连接
- [x] 决策 schema 校验（动作白名单 + 有限数值）与失败重试（`max_retries`）
- [x] 修正 `SYSTEM_PROMPT` 角度约定（原写"顺时针"，实为 REP-103 逆时针）
- [x] 循环节拍按剩余时间补齐，替代固定 `time.sleep(1.5)`
- [x] 快速控制定时器以 `command_publish_rate` 独立计算并发布激光决策，
      使安全看门狗的超时与视觉推理速度彻底解耦
- [x] 优雅退出：`threading.Event` + join + 退出前发零速

> 已核实：原 `ranges[-30:] + ranges[:30]` 与 `get_min_dist(60,120)` / `get_min_dist(240,300)`
> 对 TurtleBot3（360 点、1 度/点、`angle_min=0`、逆时针）**是正确的**，此前记录的"扇区索引 bug"为误判。
> 真实问题是把该几何硬编码了，换雷达会静默读错方向。现已改为从
> `angle_min` / `angle_increment` 反算，并用 `range_min` / `range_max` 替代硬编码的 `0.1~10.0`。

## 阶段三：安全守卫节点（`ai_robot_nav/safety_node.py`）
- [x] 订阅 `/scan` 与 `/ai_cmd_vel`，发布 `/cmd_vel`
- [x] 检测正前方扇区，`stop_distance` 内障碍置 `emergency_stop`
- [x] 紧急状态下拦截 AI 指令
- [x] 扇区计算改用 `scan_utils`，与 `ai_nav_node` 共用同一套几何逻辑
- [x] **改为 20Hz 主动发布 `/cmd_vel`**：不再只在收到 `/ai_cmd_vel` 时才发布
- [x] AI 指令看门狗（`ai_cmd_timeout`）：规划器断流即衰减到停车
- [x] 激光看门狗（`scan_timeout`）：失去感知即停车
- [x] `scan_callback` 进入紧急状态时立刻发布刹车，不等定时器周期
- [x] 停车区迟滞（`stop_distance` / `clear_distance`），抑制阈值边界抖动
- [x] 盲区（扇区内无有效回波）按危险处理，而非按空旷处理
- [x] 速度限幅（`max_linear` / `max_angular`），防止模型幻觉出超大速度
- [x] 紧急模式下允许后退（限速）与原地转向的脱困策略
- [x] `passthrough_mode` 参数，便于手动遥控时让出 `/cmd_vel`
- [x] 退出前冲刷零速，避免底盘锁存最后指令

## 阶段三点五：决策权收归确定性策略（`navigator.py`）
> 起因：`llava:7b` 会输出与动作名矛盾的转向符号（如 `TURN_LEFT` 配负 `angular_z`），
> 让模型直接决定速度不可靠。改为速度全部由激光几何确定性算出，模型降级为语义观察员。
- [x] 新增 `navigator.py`：由前/左/右扇区距离确定性推出动作与速度
- [x] `llm_client.py` 由 `parse_decision`（动作+速度）改为 `parse_assessment`（hazard+方向偏好）
- [x] 提示词改写：明确告知模型不负责计算速度，只判断激光看不到的视觉风险
- [x] 视觉提示单向收紧：可标记前方不可通行、可降速、可打破左右平局，
      但不能提速、不能导向激光判定为封闭的一侧
- [x] 非法方向降级为 `NONE` 而非触发重试（只影响平局打破，不值得重试）
- [x] 模型不可用时按纯激光继续导航（`lidar_only_fallback`）
- [x] `use_image: false` 时完全跳过模型调用，不再让视觉模型在无图像时盲猜
- [x] 策略参数（`cruise_speed` / `turn_speed` / `forward_clearance` 等）全部 ROS 参数化
- [x] 视觉提示按 `vision_ttl` 过期；过期或请求失败时自动退回纯激光
- [x] 确定性激光控制与阻塞式 Ollama 请求分线程：
      模型断线、重试或等待 HTTP 超时均不能阻塞 `/ai_cmd_vel`
- [x] 强制阻塞测试：假 Ollama 接收请求后 120s 不响应，机器人仍以 0.18m/s
      纯激光控制前进，里程计 x 增加 0.378m，未出现 `No fresh decision`

## 阶段四：QoS 与传感器兼容性
- [x] `/scan`、`/camera/image_raw` 订阅改用 `qos_profile_sensor_data`
      （BEST_EFFORT 订阅端对 RELIABLE 与 BEST_EFFORT 发布端均兼容；
      实车雷达/相机驱动多为 BEST_EFFORT，原默认 RELIABLE 会收不到任何数据）

## 阶段四点五：进程退出与信号处理
> 起因：Ctrl-C 时两节点均以 exit code 1 崩溃于
> `RCLError: publisher's context is invalid`，"退出前发零速"这一安全特性实际从未生效。
- [x] 定位根因：rclpy 默认 SIGINT 处理器在 `finally` 之前就拆掉了 context，导致 publish 抛错
- [x] 改用 `SignalHandlerOptions.NO` 自行接管信号，使 context 在冲刷零速时仍有效
- [x] `rclpy.spin_once(timeout_sec=0.1)` 切片自旋，避免信号被 rcl 阻塞等待吞掉
- [x] 新增 `lifecycle.py::install_shutdown_signals()` 显式安装 SIGINT/SIGTERM 处理器
      （不能依赖继承的处置方式：shell 后台作业会把 SIGINT 置为 `SIG_IGN`，
      仅禁用 rclpy 处理器会导致进程完全忽略 Ctrl-C）
- [x] 同时接管 SIGTERM，使 launch 升级信号时也能刹车；二次信号恢复默认以强杀
- [x] `ai_nav_node` 的 worker join 超时由 31s 收紧至 2s，避免退出被在途 HTTP 请求拖住
- [x] 运行时验证：SIGINT 后两节点均 exit code 0、500ms 内退出、无 traceback
- [x] 运行时验证：合成传感器下 `/cmd_vel` 运行中为 0.15，SIGINT 后最后一条为 0.0

## 阶段四点六：仿真时钟
- [x] launch 新增 `use_sim_time` 参数并透传给两个节点
      （WSL2 中 Gazebo 实时率常远低于 1，墙上时钟超时会把慢仿真误判为传感器失效）

## 阶段四点七：`/cmd_vel` 消息类型兼容
> 起因：Jazzy 的 TurtleBot3 `ros_gz_bridge` 订阅
> `geometry_msgs/msg/TwistStamped`，安全节点却发布 `Twist`。同名但不同类型不会通信，
> 因而日志持续显示前进，Gazebo 底盘实际完全收不到命令。
- [x] `safety_node` 新增 `cmd_stamped` 参数，按配置创建 `TwistStamped` 或 `Twist` publisher
- [x] Gazebo/Jazzy 默认 `cmd_stamped: true`，实车传统底盘可用 `cmd_stamped:=false`
- [x] `TwistStamped.header.stamp` 使用节点时钟，兼容仿真时间
- [x] 运行时验证：`/cmd_vel` 仅剩 `TwistStamped`，与桥接订阅类型一致
- [x] 里程计验证：测试窗口内 x 从 0.4513m 增至 0.6205m，实际前进约 0.169m

## 阶段五：测试与部署
- [x] 编写节点级单元测试（扇区提取 + 确定性策略 + 模型输出解析，无需 ROS 图与 Ollama）
- [x] `colcon build` + `colcon test` 在 WSL2 / ROS 2 Jazzy 通过（34 项）
- [x] 合成传感器运行时验证：看门狗超时衰减停车、停车区拦截前进但保留转向
- [x] 双节点 + 真实 Ollama 端到端验证：心跳 10Hz、`/cmd_vel` 20Hz、无抽搐
- [x] 实测 `llava:7b` 延迟：中位数 ~0.9s，带图与纯文本无可测差异
- [x] 新增 `scripts/sync_to_wsl.sh`，解决 Windows 编辑 / WSL 构建的双副本漂移
- [x] Gazebo + TurtleBot3 控制链路验证（雷达、视觉、TwistStamped 桥接、里程计运动）
- [ ] Gazebo 障碍物闭环回归（接近障碍后的转向、后退、停车区迟滞）
- [ ] 验证 `use_image:=false` 的纯激光通路与 `lidar_only_fallback` 降级路径
- [ ] 实车话题名映射验证（确认 LiDAR / 相机话题是否为 `/scan`、`/camera/image_raw`）
- [ ] 实车速度上限按底盘重标定（现为 TurtleBot3 的 0.22 / 1.5）
- [ ] 扩大 WSL 内存/交换区：实测 `llava:7b` 与 Gazebo 并行时触发 OOM Killer
      （当前 WSL 7.3GiB RAM + 2GiB swap；16GB 宿主机建议 10GB + 8GB）
- [ ] 录制演示视频/生成运行总结报告

## 已解决：模型决策质量（催生了阶段三点五）
端到端验证中，`llava:7b` 对"前方 3.0m 开阔、右侧 0.7m 有墙"的场景持续输出
`TURN_LEFT` 且 `angular_z = -0.4`，存在两处问题：
1. 前方开阔时应输出 `FORWARD`，模型未遵循提示词的规则 1；
2. 动作名与符号自相矛盾——按 REP-103，`TURN_LEFT` 应为正角速度，模型给了负值。

结论是让 7B 级视觉模型直接产出数值控制量本就不可靠，加校验只能把错误变成拒绝，
不能变成正确行为。因此采用"把数值决策改为确定性计算、只让模型做语义判断"这条对策，
即阶段三点五的 `navigator.py`。现在转向符号由代码生成，
`test_navigator.py::test_turn_sign_always_matches_the_action_name` 锁死了这个不变式，
上述失败模式在结构上不再可能出现。
