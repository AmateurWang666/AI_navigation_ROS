# AI Navigation ROS

面向 **tjark_agv** 差速小车的 ROS 1 自主导航栈：**激光雷达负责确定性运动决策**，**Ollama 视觉模型（LLaVA）仅作安全收敛层**——可以减速或停车，不会替雷达选路，也不会让机器人加速冲向障碍。

> 仓库原名 `ROS2_AI_Robot_Workspace`，当前主分支已迁移至 **ROS 1 Noetic + Gazebo Classic**，与 tjark_agv 实车/仿真平台对齐。

---

## 项目简介

本仓库提供 `ai_robot_nav` 功能包，在 Gazebo 或实车上实现：

- 订阅 `/scan` 做前方/左右扇区测距，输出 `FORWARD` / `TURN_LEFT` / `TURN_RIGHT` / `REVERSE`
- 可选订阅相机，调用本地 [Ollama](https://ollama.com/) 上的 **llava:7b** 识别玻璃、关门、台阶等雷达盲区风险
- 经 `safety_node` 做急停、限速、超时兜底后，发布 `/cmd_vel` 驱动底盘

仿真平台为外部包 **[tjark_agv](https://github.com/)**（只读集成，不修改其 URDF/控制器源码）。开发环境推荐 **Windows + WSL2（Ubuntu 24.04）+ ROS Noetic**。

---

## 核心特性

| 特性 | 说明 |
|------|------|
| 激光优先 | 转向方向、前进/后退由 `navigator.py` 纯函数策略决定，可单元测试、可复现 |
| 视觉保守 | LLM 只输出 `NONE / CAUTION / BLOCKED` 与方向偏好；CAUTION 仅降速，BLOCKED 覆盖乐观测距 |
| 安全兜底 | `safety_node` 独立监听 `/scan`，前方过近强制停车，AI 断连时自动零速 |
| tjark 适配 | 激光视场 90°–270°、`front_center_angle: 180`、Twist 型 `/cmd_vel` |
| WSL 友好 | 一键脚本、headless 空场景（约 3 秒就绪）、可选 Gazebo GUI 两终端调试 |
| 纯激光降级 | Ollama 未启动或 `--lidar` 模式下仍可完整导航 |

---

## 设计原则

```
传感器 ──► 确定性策略（navigator）──► 建议速度 /ai_cmd_vel
                ▲                           │
                │ 仅保守修正                 ▼
           Ollama 视觉评估              safety_node ──► /cmd_vel ──► 底盘
           （可选，不选路）
```

1. **模型不能选路**：角速度与线速度符号在代码里绑定，杜绝「说左转却给右转」类幻觉。
2. **模型不能加速**：视觉提示只会让机器人更慢或停下，不会超过无提示时的基线速度。
3. **雷达测不到才靠视觉**：玻璃隔断、关着的门、下沉台阶等场景由 `BLOCKED` 触发。
4. **安全与 AI 解耦**：即使 `ai_nav_node` 崩溃，`safety_node` 仍可根据激光独立停车。

---

## 系统架构

```
                    ┌─────────────────┐
  /scan ───────────►│   ai_nav_node   │
                    │  scan_utils     │
  /my_camera/... ──►│  navigator      │──► /ai_cmd_vel (Twist)
                    │  llm_client     │         │
                    └────────┬────────┘         │
                             │ Ollama API       ▼
                             ▼            ┌─────────────┐
                      http://localhost   │ safety_node │──► /cmd_vel ──► tjark_agv
                             :11434       │ 急停/限速    │
                                         └─────────────┘
```

### 主要节点

| 节点 | 职责 |
|------|------|
| `ai_nav_node` | 激光扇区解析、调用 `plan()`、可选视觉推理、发布 `/ai_cmd_vel` |
| `safety_node` | 监听 AI 指令与 `/scan`，急停迟滞、超时零速、最终 `/cmd_vel` 输出 |

### 关键模块（`src/ai_robot_nav/ai_robot_nav/`）

| 文件 | 作用 |
|------|------|
| `navigator.py` | 纯函数导航策略：迟滞、转向死区、贴墙方向锁定 |
| `scan_utils.py` | 激光扇区最小距离，支持任意 `front_center_angle` |
| `llm_client.py` | Ollama HTTP 客户端与 JSON 解析 |
| `safety_node.py` | 最后一道安全闸 |
| `motion.py` | 速度限幅 |

---

## 技术栈

| 组件 | 版本/说明 |
|------|-----------|
| ROS | **Noetic**（ROS 1） |
| 仿真 | Gazebo Classic 11 |
| 车辆模型 | tjark_agv（外部 catkin 包） |
| 语言 | Python 3 + rospy |
| 视觉 | Ollama + llava:7b（可选） |
| 测试 | pytest（47 项，无需 Gazebo） |
| 宿主 | Windows 11 + WSL2 Ubuntu 24.04 |

---

## 环境要求

- WSL2 已安装 Ubuntu，并能 `source /opt/ros/noetic/setup.bash`
- catkin 工作空间，默认 `~/ros_ws`
- tjark_agv 源码在仓库外（默认 `../tjark_agv-master/tjark_agv-master`），由 `setup_ros1_ws.sh` 只读复制
- 可选：Ollama 本地服务 + `ollama pull llava:7b`
- 带 GUI 仿真：WSLg（Win11）或 X11

**首次安装（WSL，只需一次）：**

```bash
cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace
bash scripts/install_ros_noetic.sh
bash scripts/setup_ros1_ws.sh --build
```

---

## 快速开始

### 一条命令（无 GUI，日常推荐）

```bash
cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace
bash scripts/sim.sh
```

约 3 秒后终端出现 `FORWARD lin=0.15 ang=0.00 | clear ahead (...)` 即表示机器人在前进。  
停止：`Ctrl+C` 或 `bash scripts/sim.sh stop`。

### 两终端 + Gazebo 画面

**终端 1 — 仿真：**

```bash
source /opt/ros/noetic/setup.bash && source ~/ros_ws/devel/setup.bash
bash scripts/sim.sh --gui gazebo
```

**终端 2 — 导航（等 Gazebo 窗口或 `/scan` 就绪）：**

```bash
source /opt/ros/noetic/setup.bash && source ~/ros_ws/devel/setup.bash
roslaunch ai_robot_nav ai_nav.launch use_sim_time:=true \
  params_file:=$(rospack find ai_robot_nav)/config/tjark_agv_params.yaml
```

### 常用选项

```bash
bash scripts/sim.sh --lidar      # 纯激光，不调用 Ollama
bash scripts/sim.sh --gui         # 单终端全流程 + Gazebo 窗口
bash scripts/sim.sh gazebo        # 仅仿真（终端 1）
bash scripts/sim.sh nav           # 仅导航（终端 2，自动等 /scan）
```

完整步骤、验证命令与 FAQ 见 **[docs/quick_start.md](docs/quick_start.md)**。

---

## 目录结构

```
ROS2_AI_Robot_Workspace/
├── README.md
├── docs/
│   ├── quick_start.md        # 仿真启动详解
│   └── tjark_agv_sim.md      # tjark 话题与部署说明
├── scripts/
│   ├── sim.sh                # 仿真 + 导航一键脚本
│   ├── sync_nav.sh           # 仅同步 ai_robot_nav 并 catkin_make
│   ├── setup_ros1_ws.sh      # 首次部署（含 tjark_agv 复制）
│   ├── install_ros_noetic.sh # WSL 安装 ROS Noetic
│   ├── run_tests.sh          # pytest
│   └── ros_env.sh            # 公共环境变量
└── src/ai_robot_nav/
    ├── ai_robot_nav/         # Python 包（节点逻辑）
    ├── config/
    │   ├── tjark_agv_params.yaml   # tjark 专用参数（推荐）
    │   └── ai_nav_params.yaml      # 通用默认
    ├── launch/
    │   ├── ai_nav.launch
    │   ├── tjark_gazebo.launch
    │   └── sim_full.launch
    ├── scripts/              # ROS 节点入口
    └── test/                 # 单元测试
```

---

## 参数与话题

### tjark_agv 话题映射

| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | 2D 激光 |
| `/my_camera/color/image_raw` | `sensor_msgs/Image` | RGB 相机 |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度（非 TwistStamped） |
| `/odom` | `nav_msgs/Odometry` | 里程计 |

### 关键参数（`tjark_agv_params.yaml`）

| 参数 | 典型值 | 含义 |
|------|--------|------|
| `front_center_angle` | `180.0` | tjark 激光坐标系下正前方角度 |
| `forward_clearance` | `0.70` | 大于此距离可直行 |
| `turn_clearance` | `0.55` | 小于此距离必须转向 |
| `tie_threshold` | `0.6` | 左右差距小于此值时保持转向方向 |
| `stop_distance` | `0.24` | safety 急停距离（m） |

车辆几何：轮距 0.34 m，轮径 0.12 m（来自 tjark_agv URDF，本仓库不修改）。

---

## 开发与测试

### Windows 改代码 → WSL 编译

```powershell
wsl -d Ubuntu bash -c "cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace && bash scripts/sync_nav.sh --build"
```

WSL 内重新 `bash scripts/sim.sh` 即可验证。

### 单元测试（不需要 Gazebo / Ollama）

```bash
bash scripts/run_tests.sh
# 或
cd src/ai_robot_nav && python -m pytest test/ -q
```

测试覆盖：激光扇区解析、导航策略不变式、LLM JSON 解析边界。

---

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 只左右转、不前进 | 激光角度未映射到正前方 | 确认 `front_center_angle: 180`，执行 `sync_nav.sh --build` |
| 贴墙左右摇摆 | 转向方向在噪声下频繁切换 | 已内置贴墙锁定；可调大 `tie_threshold` 或 `turn_clearance` |
| Gazebo 启动极慢 | 使用了 cafe 大场景 | 改用 `scripts/sim.sh` 或 `tjark_gazebo.launch` 空场景 |
| 一直停车 | `/scan` 未发布或 AI 超时 | 先启仿真；检查 `rostopic echo /scan` |
| Ollama 报错 | 服务未启动 | `bash scripts/sim.sh --lidar` 纯激光模式 |

---

## 文档索引

| 文档 | 内容 |
|------|------|
| [docs/quick_start.md](docs/quick_start.md) | 一键启动、两终端 GUI、改代码流程 |
| [docs/tjark_agv_sim.md](docs/tjark_agv_sim.md) | tjark 部署、话题、与 TurtleBot3 差异 |

---

## 许可证

`ai_robot_nav` 包声明为 **Apache-2.0**（见 `package.xml`）。tjark_agv 为外部依赖，遵循其原项目许可证。

---

## 致谢

- [tjark_agv](https://github.com/) — 仿真/实车 URDF 与 Gazebo 模型
- [Ollama](https://ollama.com/) + LLaVA — 本地视觉推理
- 由 TurtleBot3 / ROS 2 原型迁移至 ROS 1，以匹配实车平台
