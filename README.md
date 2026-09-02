# AI Navigation ROS

面向 **tjark_agv** 差速小车的 ROS 1 自主导航栈：**激光雷达负责确定性运动决策**，
**Ollama 视觉模型（LLaVA）仅作安全收敛层**——可以减速或停车，不会替雷达选路，
也不会让机器人加速冲向障碍。

支持两种运行方式，共用同一条指令链路与同一道安全闸：

| 方式 | 说明 |
|------|------|
| **目标点导航** | 给定地图与目的地，自动规划路径并抵达（`move_base` + `amcl`） |
| **反应式漫游** | 无地图、无目标点，纯激光避障游走（本项目最初的形态） |

地图用 **ROS 标准 map_server 格式**，可换成任何环境的地图——用法见
[docs/mapping.md](docs/mapping.md)。

项目处于早期阶段：**当前只在仿真中运行，后续挪到实车调试**。因此仓库刻意把
「平台」与「策略」分开，仿真与实车共用同一份 URDF、同一份参数、同一套导航节点，
迁移时不需要改导航代码（清单见 [docs/real_robot.md](docs/real_robot.md)）。

> 仓库名里的 "ROS" 指 **ROS 1 Noetic + Gazebo Classic 11**（早期原型基于 ROS 2，
> 已迁移，以对齐 tjark_agv 实车平台）。

---

## 仓库内容

仓库自带全部仿真资产，克隆后**不依赖任何仓库外路径，也不需要联网下载模型**：

| 包 | 角色 |
|----|------|
| `src/tjark_agv` | 车辆描述 + 唯一仿真环境（cafe 场景与其 Gazebo 模型均已内置） |
| `src/ai_robot_nav` | 导航与安全节点（仿真、实车共用） |

仿真自带 **cafe** 场景与配套地图，克隆即用。换成你自己的世界与地图见
[docs/mapping.md](docs/mapping.md)。

---

## 设计原则

```
传感器 ──► 确定性策略 ──► 建议速度 ──► 仲裁 ──► safety_node ──► /cmd_vel ──► 底盘
              ▲                                    ▲
              │ 仅保守修正                          │ 唯一发布者
         Ollama 视觉评估                        激光急停
         （可选，不选路）
```

1. **模型不能选路**：角速度与线速度符号在代码里绑定，杜绝「说左转却给右转」类幻觉。
2. **模型不能加速**：视觉提示只会让机器人更慢或停下，不会超过无提示时的基线速度。
3. **雷达测不到才靠视觉**：玻璃隔断、关着的门、下沉台阶等场景由 `BLOCKED` 触发。
4. **安全与 AI 解耦**：即使上层全部崩溃，`safety_node` 仍可根据激光独立停车。

这四条不是口号，`test/` 下把它们写成了断言。第 4 条同样约束 `move_base`：
它的输出被重映射到 `/nav_cmd_vel`，不允许绕过安全闸直接驱动底盘。

---

## 系统架构

```
  地图 ──► map_server ──► /map ─┐
                                ├──► move_base ──► /nav_cmd_vel ─┐
  /scan + /odom ──► amcl ───────┘                                │
                                                                 ├─► nav_mux
  /scan ─────────►┌─────────────┐                                │
  /my_camera/... ►│ ai_nav_node │──► /explore_cmd_vel ───────────┘
                  │ + Ollama    │                                │
                  └─────────────┘                                ▼
                                                    /ai_cmd_vel ──► safety_node ──► /cmd_vel
```

| 节点 | 职责 |
|------|------|
| `ai_nav_node` | 激光扇区解析、调用 `plan()`、可选视觉推理、发布 `/explore_cmd_vel` |
| `move_base` | 全局路径规划（A\*）+ 局部避障（DWA），发布 `/nav_cmd_vel` |
| `amcl` | 在地图上定位，发布 `map → odom` 修正 |
| `nav_mux` | 在目标点导航与反应式漫游之间仲裁，输出 `/ai_cmd_vel` |
| `map_manager` | 启动前校验地图、加载命名目的地 |
| `mapper` | 为新环境建图（仅 `mapping.launch` 用） |
| `safety_node` | 急停迟滞、超时零速、**唯一** `/cmd_vel` 发布者 |

| 模块（`src/ai_robot_nav/ai_robot_nav/`） | 作用 |
|------|------|
| `navigator.py` | 纯函数反应式策略：净空迟滞、转向死区、方向锁存、脱困 |
| `command_mux.py` | 纯函数指令仲裁：目标点优先、过期即停 |
| `scan_utils.py` | 激光扇区最小距离，按雷达安装角推算四个方向 |
| `map_utils.py` | 用户地图的解析与校验 |
| `goals.py` | 目的地表解析，名字与坐标两种输入 |
| `occupancy_grid.py` | 对数几率占据栅格，含 PGM/YAML 导出 |
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
| 导航 | `move_base` / `amcl` / `map_server`（源码编译，见下） |
| 测试 | pytest（155 项，不需要 Gazebo 或 Ollama） |

**首次安装（WSL，只需一次）：**

```bash
cd /mnt/c/Users/ROG/Desktop/ROS_AI_Robot_Workspace
bash scripts/install_ros_noetic.sh    # 装 ROS Noetic + Gazebo，并自动构建
bash scripts/install_nav_stack.sh     # 装导航栈（目标点导航需要）
```

> **导航栈为什么要源码编译**：本机是 Ubuntu 24.04，而 ROS Noetic 官方只发布
> Ubuntu 20.04 的二进制包；现有 Noetic 来自社区 PPA `ros-for-jammy`，
> 那个 PPA 不含任何导航组件。换 apt 镜像源解决不了——这些包对本发行版根本不存在。
> 脚本会按内存推算并行度以避免编译时 OOM，细节与排查见
> [docs/mapping.md](docs/mapping.md)。只跑反应式漫游则不需要这一步。

---

## 快速开始

### 一条命令跑全程（日常推荐）

```bash
cd /mnt/c/Users/ROG/Desktop/ROS_AI_Robot_Workspace
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

### 导航到指定目的地

仿真起来之后，另开一个终端：

```bash
source /opt/ros/noetic/setup.bash && source ~/ros_ws/devel/setup.bash

rosrun ai_robot_nav send_goal --list      # 看有哪些命名目的地
rosrun ai_robot_nav send_goal hall        # 按名字去
rosrun ai_robot_nav send_goal 0.5 -3.5    # 按坐标去
```

命令会阻塞到有结论为止并回传结果（抵达 / 规划失败 / 目标被拒 / 被打断），
退出码非零表示未抵达，可以直接用在脚本里。RViz 里的 **2D Nav Goal** 也照常可用。

换成你自己的地图：

```bash
bash scripts/sim.sh --map /路径/到/我的地图.yaml
```

地图格式、建图方法、目的地配置见 [docs/mapping.md](docs/mapping.md)。

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
wsl -d Ubuntu bash -c "cd /mnt/c/Users/ROG/Desktop/ROS_AI_Robot_Workspace && bash scripts/sync_ws.sh --build"
```

然后在 WSL 里重新 `bash scripts/sim.sh`。只改 Python 逻辑时其实不必重新构建，
`sync_ws.sh` 不带 `--build` 同步一下即可。

### 单元测试（不需要 Gazebo / Ollama）

```powershell
wsl -d Ubuntu bash ./scripts/run_tests.sh
```

覆盖激光扇区解析（含雷达安装角）、导航策略不变式、脱困计时、LLM JSON 解析边界、
指令仲裁不变式、地图校验、目的地解析、占据栅格数学与 PGM 导出。

---

## 目录结构

```
ROS_AI_Robot_Workspace/
├── README.md
├── docs/
│   ├── platform.md            # tjark_agv 车辆与 cafe 仿真环境说明、硬件清单
│   ├── mapping.md             # 地图格式、建图、目标点导航、规划器选型
│   └── real_robot.md          # 挪到实车的清单与标定顺序
├── scripts/
│   ├── sim.sh                 # 仿真 + 导航一键脚本
│   ├── sync_ws.sh             # 同步 src/ 到 ~/ros_ws 并 catkin_make
│   ├── install_ros_noetic.sh  # WSL 首次安装
│   ├── install_nav_stack.sh   # 源码编译 move_base/amcl/map_server
│   ├── run_tests.sh           # pytest
│   └── ros_env.sh             # 公共环境变量（含导航栈 overlay）
├── src/ai_robot_nav/
│   ├── ai_robot_nav/          # Python 包（节点逻辑）
│   ├── config/
│   │   ├── nav_params.yaml        # 唯一参数文件，仿真与实车共用
│   │   ├── move_base.yaml         # 全局/局部规划器选型与参数
│   │   ├── amcl.yaml              # 定位参数
│   │   ├── costmap_*.yaml         # 代价地图（公共/全局/局部）
│   │   └── rosconsole_sim.config  # 仿真日志降噪
│   ├── maps/
│   │   ├── cafe.yaml / cafe.pgm         # 默认地图
│   │   └── cafe.destinations.yaml       # 命名目的地
│   ├── launch/
│   │   ├── ai_nav.launch      # 导航层 + 仲裁（平台无关）
│   │   ├── navigation.launch  # map_server + amcl + move_base
│   │   ├── mapping.launch     # 为新环境建图
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
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度（非 TwistStamped），**仅 `safety_node` 发布** |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `/map` | `nav_msgs/OccupancyGrid` | 静态地图 |
| `/explore_cmd_vel` | `geometry_msgs/Twist` | 反应式漫游的建议速度 |
| `/nav_cmd_vel` | `geometry_msgs/Twist` | `move_base` 的建议速度（已从 `/cmd_vel` 重映射） |
| `/ai_cmd_vel` | `geometry_msgs/Twist` | 仲裁结果，送入安全闸 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 目标点（RViz 的 2D Nav Goal） |

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
