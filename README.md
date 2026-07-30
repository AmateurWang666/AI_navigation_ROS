# ai_robot_nav

ROS 2 端到端自动驾驶：激光雷达确定性地决定怎么走，本地视觉大模型（Ollama + LLaVA）只提供
语义补充，独立的安全节点负责快速硬约束。面向单线激光雷达 + 前向摄像头的差速底盘，
在 WSL2（Ubuntu 24.04 + ROS 2 Jazzy）Gazebo 仿真验证，可迁移至 Jetson 实车。

## 快速启动

每次新开终端需要输入的完整命令见：

**[`docs/quick_start.md`](docs/quick_start.md) — 两终端快速启动指南**

## 架构

```
/scan ────────────┬──────────────────────────────────► safety_node ──► /cmd_vel ──► 底盘
                  │                                         ▲
                  └─► ai_nav_node ─► navigator (确定性策略)   │
                          │  ▲                               │
/camera/image_raw ────────┘  └─ Ollama (llava:7b) 语义提示    │
                          └──────────► /ai_cmd_vel ──────────┘
```

设计上有两条硬性分工。

**速度由激光雷达算，不由模型算。** `navigator.py` 从前/左/右扇区距离确定性地推出动作与速度。
模型只回一个语义评估 `{hazard, preferred_direction}`，而且接线方式保证它**只能让机器人更保守**：
可以把前方标记为不可通行、可以按 `caution_scale` 降速、可以在左右空间相近时打破平局，
但无法提高任何速度，也无法把机器人导向激光雷达判定为封闭的方向。这样模型幻觉的后果被限制在
"过于谨慎"这一侧。模型不可用时机器人照常按纯激光导航。

**`safety_node` 是 `/cmd_vel` 的唯一发布者**，以 20 Hz 持续发布。大模型推理耗时远长于机器人
陷入危险所需的时间，所以速度输出的所有权必须在快速回路手里。任何一路输入断流——规划器崩溃、
Ollama 卡住、网络掉线、雷达静默——都会在一个超时周期内自动衰减到停车，而不是把最后一条
速度指令永久锁存在底盘里。

| 文件 | 职责 |
|------|------|
| `ai_nav_node.py` | 编排：快速激光控制循环 + 独立的慢速视觉推理线程 |
| `navigator.py` | 确定性反应式策略，由激光扇区距离算出动作与速度 |
| `llm_client.py` | Ollama 请求、JSON 提取、语义评估校验 |
| `safety_node.py` | 看门狗、停车区、速度限幅，仲裁后发布 `/cmd_vel` |
| `scan_utils.py` | 与雷达型号无关的扇区距离提取 |
| `motion.py` | 速度钳位 |

## 依赖

- ROS 2 Jazzy（Humble 亦可）
- Ollama ≥ 0.1.24（需要 `format: "json"` 支持）+ 已拉取的 `llava:7b`
- `rosdep install --from-paths src --ignore-src -r -y`

## 开发流程（Windows 编辑 / WSL 构建）

本仓库在 Windows 侧编辑，但 ROS 2 工作空间在 WSL 的 `~/ros2_ws`，两者是**两份独立拷贝**。
改完代码必须先同步，否则 `colcon build` 构建的还是旧版本。在仓库根目录的 PowerShell 里：

```powershell
wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh --test    # 同步 + 构建 + 跑测试
wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh --build   # 同步 + 构建
wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh           # 只同步
```

脚本会顺带清掉 `__pycache__`、把 CRLF 换成 LF、并删除 `build/` `install/` 下的旧产物。
工作空间不在 `~/ros2_ws` 时用 `ROS2_WS=/path/to/ws` 覆盖。

## 构建（已在 WSL 内时）

```bash
cd ~/ros2_ws
colcon build --packages-select ai_robot_nav --symlink-install
source install/setup.bash
```

## 运行（Gazebo 仿真）

> 必须**先起仿真**再起节点。没有 `/scan` 时两个节点会正确地拒绝移动，
> 日志持续刷 `LiDAR silent; holding stop.`——这是安全设计生效，不是故障。
> 用 `ros2 topic list` 确认 `/scan` 和 `/camera/image_raw` 存在再继续。

```bash
# 终端 1：仿真
export TURTLEBOT3_MODEL=waffle
export QT_QPA_PLATFORM=xcb
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端 2：Ollama
ollama serve
ollama pull llava:7b

# 终端 3：双节点。配合 Gazebo 建议开 use_sim_time
ros2 launch ai_robot_nav ai_nav.launch.py use_sim_time:=true
```

`use_sim_time:=true` 让超时判断跟随仿真时钟。WSL2 里 Gazebo 常常跑不到实时速度，
若仍按墙上时钟计时，慢下来的仿真会被误判成传感器失效而一直停车。

Gazebo Sim 8 / ROS 2 Jazzy 的桥接端订阅
`geometry_msgs/msg/TwistStamped`，所以本项目默认 `cmd_stamped:=true`。若看到
“`/cmd_vel` contains more than one type: Twist, TwistStamped”，说明仍在运行旧构建，
请重新执行同步构建脚本并重启节点。

## 实车 / 远程算力部署

模型跑在另一台机器上时，不需要改任何源码：

```bash
ros2 launch ai_robot_nav ai_nav.launch.py \
    ollama_url:=http://192.168.1.100:11434/api/generate
```

远端主机需要 `OLLAMA_HOST=0.0.0.0 ollama serve` 才会监听局域网。

话题名与本机不一致时，改 `config/ai_nav_params.yaml` 里的 `scan_topic` / `image_topic` / `cmd_topic`，
或者传一份自己的参数文件：`params_file:=/path/to/my_robot.yaml`。

多数实车底盘驱动仍订阅传统 `geometry_msgs/msg/Twist`。部署前用
`ros2 topic info /cmd_vel --verbose` 确认类型；若是 `Twist`，启动时加：

```bash
ros2 launch ai_robot_nav ai_nav.launch.py cmd_stamped:=false \
    ollama_url:=http://192.168.1.100:11434/api/generate
```

## 关键参数

全部参数见 `config/ai_nav_params.yaml`，最常调的几个：

| 参数 | 默认 | 说明 |
|------|------|------|
| `cruise_speed` | `0.18` | 前方开阔时的直行速度 |
| `turn_speed` | `0.5` | 前方受阻时的原地转向角速度 |
| `forward_clearance` | `0.6` | 维持直行所需的前方净空 |
| `trapped_distance` | `0.4` | 三面均低于此值判定为被困，转后退 |
| `caution_scale` | `0.5` | 视觉报 `CAUTION` 时的降速倍率 |
| `use_image` | `true` | 关闭后完全跳过模型，走纯激光确定性导航 |
| `lidar_only_fallback` | `true` | 模型不可用时继续按纯激光行驶；置 `false` 则停车 |
| `inference_period` | `1.0` | 目标推理周期（秒），实际不会快过模型 |
| `command_publish_rate` | `10.0` | 确定性激光控制及 `/ai_cmd_vel` 发布频率 |
| `vision_ttl` | `3.0` | 视觉提示超过此时长后丢弃，自动降级为纯激光 |
| `ollama_timeout` | `30.0` | 单次请求超时，需大于实测推理耗时 |
| `stop_distance` / `clear_distance` | `0.30` / `0.40` | 停车区迟滞，两者之差抑制边界抖动 |
| `ai_cmd_timeout` | `0.7` | 看门狗超时，对齐心跳频率而非推理周期 |
| `cmd_stamped` | `true` | `/cmd_vel` 发布 `TwistStamped`，匹配 Gazebo Sim 8 / Jazzy |
| `max_linear` / `max_angular` | `0.22` / `1.5` | 安全层的权威速度上限 |
| `passthrough_mode` | `false` | 置 `true` 时安全节点完全不发布 |

角度约定遵循 REP-103：0 度为正前方，逆时针为正，因此左侧为正角度、右侧为负角度。

`use_image: false` 与 `lidar_only_fallback: false` 同时设置会让机器人永远拿不到评估从而不动，
节点启动时会报错提示。

### 为什么控制与视觉必须分线程

Ollama 请求可能耗时数秒、断开连接或一直等到 HTTP 超时。若激光规划和模型请求在同一个线程，
即使开启 `lidar_only_fallback`，阻塞期间也无法真正降级。现在
`command_publish_rate` 驱动一个不接触网络的确定性激光控制循环；视觉线程只异步更新可选提示。
提示超过 `vision_ttl` 自动丢弃，控制立即退回纯激光，因此模型故障不会阻塞运动控制。

### 实测延迟参考

本机（WSL2 + GPU 加速）`llava:7b` 单次决策中位数约 **0.9 秒**，带 320x240 图像与纯文本无可测差异，
因此 `use_image` 默认开启。换到 Jetson 或纯 CPU 时请重新实测，并据此调整 `inference_period`。
`vision_ttl` 与 `ai_cmd_timeout` 不需要随模型变慢而放大。

## 手动遥控

`safety_node` 以 20 Hz 持续发布 `/cmd_vel`，会覆盖 `teleop_twist_keyboard` 的指令。
手动遥控前先让它闭嘴：

```bash
ros2 param set /safety_node passthrough_mode true
```

**此时机器人处于无保护状态。** 遥控结束后记得设回 `false`。

## 测试

```bash
colcon test --packages-select ai_robot_nav
colcon test-result --verbose
```

三组单元测试，均不需要 ROS 图或 Ollama 服务：

- `test_scan_utils.py` — 扇区提取，覆盖 360/1440 线、`angle_min` 为负、inf/NaN/dropout 区分
- `test_navigator.py` — 确定性策略，覆盖转向符号与动作名一致、盲区不当作空旷、
  以及"视觉提示只能更保守"这条不变式（不能提速、不能导向更封闭的一侧）
- `test_llm_client.py` — 模型输出解析，覆盖代码块包裹、夹杂散文、非法 hazard、
  非法方向降级为 `NONE`、以及模型无法夹带速度字段

## 故障排查

| 现象 | 原因 |
|------|------|
| 持续刷 `LiDAR silent; holding stop.` | 没有 `/scan`。最常见的是忘了先起 Gazebo；用 `ros2 topic list` 确认。这是安全设计在正确工作 |
| 仿真已启动但仍报 `LiDAR silent` | Gazebo 实时率过低，墙上时钟超时被误触发。加 `use_sim_time:=true` |
| `ros2 topic hz /cmd_vel` 报含两种类型 | 旧安全节点发布 `Twist`，Jazzy 桥订阅 `TwistStamped`。重新同步构建并重启；当前默认已修复 |
| 节点无日志、不动，也不报错 | QoS 不兼容。传感器订阅已用 `qos_profile_sensor_data`（BEST_EFFORT），对 RELIABLE 和 BEST_EFFORT 发布者都兼容；先用 `ros2 topic info /scan --verbose` 确认话题存在 |
| 日志刷 `No /ai_cmd_vel within 0.7s` | 快速激光控制循环没在发布，说明节点卡死或崩溃，不是模型推理慢 |
| 日志出现 `Ollama request failed` | 视觉线程连接失败；`lidar_only_fallback: true` 时控制不受阻，提示过期后自动变为 `vision=lidar-only` |
| Ollama 报 `RemoteDisconnected` 后自动重启 | 检查 `journalctl -u ollama`。若有 `killed by the OOM killer`，说明 WSL 内存不足；16GB 宿主机建议 `.wslconfig` 设 `memory=10GB`、`swap=8GB` 后执行 `wsl --shutdown` |
| 日志刷 `Rejected model output` | 模型不听话。确认 Ollama 版本支持 `format: json`，或调低 `temperature`；期间机器人按纯激光继续行驶 |
| 机器人在空地上不走 | 前方扇区被判为盲区。用 `ros2 topic echo /scan --once --no-arr` 检查 `range_min`/`range_max` 是否合理 |
| 机器人一直原地打转 | `forward_clearance` 相对场地过大，或前方扇区 `front_half_angle` 太宽 |
| Ollama 首次请求特别慢 | 模型加载。`keep_alive: "30m"` 已避免后续重载 |
