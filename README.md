# AI Navigation ROS

面向 **tjark_agv** 差速小车的 ROS 1 自主导航栈：**激光雷达负责确定性运动决策**，
**Ollama 视觉模型（LLaVA）仅作安全收敛层**——可以减速或停车，不会替雷达选路，
也不会让机器人加速冲向障碍。

项目处于早期阶段：**当前只在仿真中运行，后续挪到实车调试**。因此仓库刻意把
「平台」与「策略」分开，仿真与实车共用同一份 URDF、同一份参数、同一套导航节点，
迁移时不需要改导航代码（清单见 [docs/real_robot.md](docs/real_robot.md)）。

> 仓库目录名沿用了早期的 `ROS2_AI_Robot_Workspace`，但主分支已迁移到
> **ROS 1 Noetic + Gazebo Classic 11**，与 tjark_agv 实车平台对齐。

---

## 仓库内容

仓库自带全部仿真资产，克隆后**不依赖任何仓库外路径，也不需要联网下载模型**：

| 包 | 角色 |
|----|------|
| `src/tjark_agv` | 车辆描述 + 唯一仿真环境（cafe 场景与其 Gazebo 模型均已内置） |
| `src/ai_robot_nav` | 导航与安全节点（仿真、实车共用） |

仿真环境只有 **cafe** 一个，不提供空场景等其它选择：环境固定下来，两次运行之间的
差异就只来自代码本身。

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

这四条不是口号，`test/test_navigator.py` 把它们写成了断言。

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
                      http://localhost    │ safety_node │──► /cmd_vel ──► tjark_agv
                             :11434       │ 急停/限速    │
                                          └─────────────┘
```

| 节点 | 职责 |
|------|------|
| `ai_nav_node` | 激光扇区解析、调用 `plan()`、可选视觉推理、发布 `/ai_cmd_vel` |
| `safety_node` | 监听 AI 指令与 `/scan`，急停迟滞、超时零速、**唯一** `/cmd_vel` 发布者 |

| 模块（`src/ai_robot_nav/ai_robot_nav/`） | 作用 |
|------|------|
| `navigator.py` | 纯函数导航策略：净空迟滞、转向死区、方向锁存、脱困 |
| `scan_utils.py` | 激光扇区最小距离，按雷达安装角推算四个方向 |
| `llm_client.py` | Ollama HTTP 客户端与 JSON 解析 |
| `safety_node.py` | 最后一道安全闸 |
| `motion.py` | 速度限幅 |

---

## 环境要求

| 组件 | 版本 |
|------|------|
| 宿主 | Windows 11 + WSL2（Ubuntu） |
| ROS | Noetic（ROS 1） |
| 仿真 | Gazebo Classic 11 |
| 语言 | Python 3 + rospy |
| 视觉 | Ollama + `llava:7b`（可选，未启动时自动降级纯激光） |
| 测试 | pytest（57 项，不需要 Gazebo 或 Ollama） |

**首次安装（WSL，只需一次）：**

```bash
cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace
bash scripts/install_ros_noetic.sh    # 装 ROS Noetic + Gazebo，并自动构建
```

---

## 快速开始

### 一条命令跑全程（日常推荐）

```bash
cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace
bash scripts/sim.sh
```

**正常现象：**

- **没有 Gazebo 窗口** —— 默认 headless，仿真在后台跑
- 约 **3 秒**后 `/scan` 就绪，随即出现
  `FORWARD lin=0.15 ang=0.00 | clear ahead (2.60m)`，表示机器人在前进
- 开头几行 `LiDAR silent` 是在等 Gazebo 加载，可忽略
- 已默认静音 Gazebo 相机 DEBUG，需要完整日志加 `--verbose`

**另开终端确认在动：**

```bash
source /opt/ros/noetic/setup.bash && source ~/ros_ws/devel/setup.bash
rostopic echo /odom/pose/pose/position
```

停止：`Ctrl+C` 或 `bash scripts/sim.sh stop`。

### 常用选项

```bash
bash scripts/sim.sh --lidar     # 纯激光，不调用 Ollama
bash scripts/sim.sh --gui       # 单终端全流程 + Gazebo 窗口
bash scripts/sim.sh gazebo      # 仅仿真（终端 1）
bash scripts/sim.sh nav         # 仅导航（终端 2，自动等 /scan）
```

需要看画面时用两个终端：终端 1 `bash scripts/sim.sh --gui gazebo` 等窗口弹出，
终端 2 `bash scripts/sim.sh nav`。WSL2 需已启用图形（Win11 自带 WSLg）；
窗口黑屏时 `sim.sh --gui` 已自动设置 `QT_QPA_PLATFORM=xcb` 与
`LIBGL_ALWAYS_SOFTWARE=1`。

### 改代码后（Windows 编辑 → WSL 运行）

```powershell
wsl -d Ubuntu bash -c "cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace && bash scripts/sync_ws.sh --build"
```

然后在 WSL 里重新 `bash scripts/sim.sh`。只改 Python 逻辑时其实不必重新构建，
`sync_ws.sh` 不带 `--build` 同步一下即可。

### 单元测试（不需要 Gazebo / Ollama）

```powershell
wsl -d Ubuntu bash ./scripts/run_tests.sh
```

覆盖激光扇区解析（含雷达安装角）、导航策略不变式、脱困计时、LLM JSON 解析边界。

---

## 目录结构

```
ROS2_AI_Robot_Workspace/
├── README.md
├── docs/
│   ├── platform.md            # tjark_agv 车辆与 cafe 仿真环境说明、硬件清单
│   └── real_robot.md          # 挪到实车的清单与标定顺序
├── scripts/
│   ├── sim.sh                 # 仿真 + 导航一键脚本
│   ├── sync_ws.sh             # 同步 src/ 到 ~/ros_ws 并 catkin_make
│   ├── install_ros_noetic.sh  # WSL 首次安装
│   ├── run_tests.sh           # pytest
│   └── ros_env.sh             # 公共环境变量（含 Gazebo 模型路径）
├── src/ai_robot_nav/
│   ├── ai_robot_nav/          # Python 包（节点逻辑）
│   ├── config/
│   │   ├── nav_params.yaml        # 唯一参数文件，仿真与实车共用
│   │   └── rosconsole_sim.config  # 仿真日志降噪
│   ├── launch/
│   │   ├── ai_nav.launch      # 导航层（平台无关）
│   │   ├── sim.launch         # cafe 仿真 + 导航
│   │   └── real.launch        # 实车入口（驱动挂载点）
│   ├── scripts/               # ROS 节点入口
│   └── test/                  # 单元测试
└── src/tjark_agv/
    ├── urdf/ meshes/          # 车辆描述（仿真与实车共用）
    ├── worlds/ models/        # cafe 场景及其模型（离线可用）
    └── launch/
        ├── description.launch # URDF + TF（仿真与实车共用）
        └── cafe_world.launch  # Gazebo + cafe + spawn（仅仿真）
```

---

## 参数与话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | 2D 激光，180° 视场 |
| `/my_camera/color/image_raw` | `sensor_msgs/Image` | RGB 相机 |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度（非 TwistStamped） |
| `/odom` | `nav_msgs/Odometry` | 里程计 |

关键参数（`config/nav_params.yaml`，全部带中文注释）：

| 参数 | 值 | 含义 |
|------|-----|------|
| `front_center_angle` | `180.0` | 车体正前方对应的扫描角，即雷达安装偏转角 |
| `forward_clearance` | `0.70` | 大于此距离可直行 |
| `turn_clearance` | `0.55` | 小于此距离必须转向（与上行构成迟滞带） |
| `tie_threshold` | `0.6` | 左右差距小于此值时保持上次转向方向 |
| `escape_commit_time` | `4.0` | 连续转向超过此秒数即锁死方向 |
| `escape_reverse_time` | `9.0` | 再超过此秒数改为边退边转脱困；`0` 关闭 |
| `stop_distance` | `0.24` | safety 急停距离（从雷达算起） |

坐标约定、车辆几何与已知的参数不一致见 [docs/platform.md](docs/platform.md)。

---

## 常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| 只左右转、不前进 | 雷达安装角没对上 | 确认 `front_center_angle: 180`，`sync_ws.sh --build` |
| 撞墙后左右摆头 | 左右扇区映射反了 | 已修复：四个扇区统一由 `front_center_angle` 推算 |
| 转很久出不来 | 车头顶住了障碍 | 已内置：4 s 锁死方向、9 s 边退边转 |
| Gazebo 启动卡住 | 联网找模型 | 已修复：模型内置且关闭在线模型库；请用 `scripts/sim.sh` |
| 一直停车 | `/scan` 未发布或 AI 超时 | 先启仿真；`rostopic echo /scan` |
| Ollama 报错 | 服务未启动 | `bash scripts/sim.sh --lidar` 纯激光模式 |

---

## 许可证

`ai_robot_nav` 为 **Apache-2.0**，`tjark_agv` 沿用其原始的 **BSD**（见各自 `package.xml`）。

## 致谢

- tjark_agv —— 实车 URDF 与 Gazebo 模型，现已并入本仓库维护
- [Ollama](https://ollama.com/) + LLaVA —— 本地视觉推理
- 由 TurtleBot3 / ROS 2 原型迁移至 ROS 1，以匹配实车平台
