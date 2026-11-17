# AI Navigation ROS

面向 **tjark_agv** 差速小车的 ROS 1 导航栈：**激光雷达负责确定性运动决策**，
**Ollama 视觉模型（LLaVA）仅作安全收敛层**——可以减速或停车，不会替雷达选路，
也不会让机器人加速冲向障碍。

> 仓库名里的 "ROS" 指 **ROS 1 Noetic + Gazebo Classic 11**（早期原型基于 ROS 2，
> 已迁移，以对齐 tjark_agv 实车平台）。

当前进度、还没做的事、以及建议的先后顺序写在 [docs/roadmap.md](docs/roadmap.md)。

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
| `scan_utils.py` | 激光扇区最小距离。展宽只作用于扇区内部的回波；跨帧记忆不覆盖盲区（`None`） |
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
| 测试 | pytest（`src/ai_robot_nav/test/`，不需要 Gazebo 或 Ollama） |

**首次安装（WSL，只需一次）：**

```bash
cd /mnt/c/Users/ROG/Desktop/ROS_AI_Robot_Workspace
bash scripts/install_ros_noetic.sh    # 装 ROS Noetic + Gazebo，并自动构建
bash scripts/install_nav_stack.sh     # 装导航栈（目标点导航需要）
bash scripts/align_wsl_layout.sh      # 若 WSL 里仍是旧的 ~/ros_ws 命名，一次性对齐
```

WSL 侧目录与 Windows 仓库名对齐（源码在 `/mnt/c/.../ROS_AI_Robot_Workspace`，构建在本地盘）：

| 路径 | 用途 |
|------|------|
| `/mnt/c/Users/ROG/Desktop/ROS_AI_Robot_Workspace` | 源码（Git 仓库，Windows 编辑） |
| `~/ROS_AI_Robot_Workspace_ws` | Catkin 构建副本（`sync_ws.sh` 同步到这里） |
| `~/ROS_AI_Robot_Workspace_nav_ws` | 导航栈 overlay（`install_nav_stack.sh`） |

> **导航栈为什么要源码编译**：本机是 Ubuntu 24.04，而 ROS Noetic 官方只发布
> Ubuntu 20.04 的二进制包；现有 Noetic 来自社区 PPA `ros-for-jammy`，
> 那个 PPA 不含任何导航组件。换 apt 镜像源解决不了——这些包对本发行版根本不存在。
> 脚本会按内存推算并行度以避免编译时 OOM，细节与排查见
> [docs/mapping.md](docs/mapping.md)。只跑反应式漫游则不需要这一步。

---

## 地图与导航安全

`sim.launch` 默认会加载 cafe 地图和 `move_base`，所以仿真里发目标点会走到这些限制。
换地图、建图和规划器选型见 [docs/mapping.md](docs/mapping.md)。

目标点导航能否成功，**不只看 move_base 有没有装好，更取决于地图是否完整**。
这是本项目最容易踩坑、也最容易被误判为「规划器坏了」的地方。

### 核心规则：`allow_unknown` 默认必须是 `false`

全局规划器有一个参数 `GlobalPlanner/allow_unknown`：

| 取值 | 行为 | 适用场景 |
|------|------|----------|
| **`false`（默认）** | 只在「已知空闲」栅格上规划；**未知区域视为不可通行** | 完整地图、实车、任何需要安全保证的场景 |
| `true` | 允许路径穿越未知栅格 | 仅作**临时权宜**，例如地图还没建完、又要先跑通 demo |

`allow_unknown=true` 的含义是：机器人可能被规划进**从未观测过的区域**——那里有没有墙、台阶、玻璃，地图里完全没有信息。局部代价地图只能看见当前激光扫到的范围，**挡不住地图标注错误带来的风险**。

因此：

1. **不要把 `allow_unknown` 永久写进 `move_base.yaml`。** 默认值已设为 `false`。
2. **正确做法是补建地图**：让起点到目标点之间的走廊在地图上标记为「已知空闲」。
3. **若暂时无法补建**，可以显式临时放宽（启动时传参，而不是改配置文件）：

```bash
# 临时放宽——有安全风险，仅用于地图覆盖不完整的调试
bash scripts/sim.sh --allow-unknown
# 或
roslaunch ai_robot_nav sim.launch allow_unknown:=true
```

### 内置 cafe 地图的已知限制

仓库自带的 `maps/cafe.yaml` 是用轻量建图节点生成的演示地图，**覆盖率约 17%**（大量区域仍是未知）。在此地图 + `allow_unknown=false` 下：

- 近处目标点（如 `north`）通常可以抵达
- 较远目标点（如 `hall`）往往报 `Failed to find a valid plan`——因为路径必须穿过未知区域

这不是 bug，而是安全策略在起作用。启动时 **`map_manager` 会打印连通性警告**，列出哪些命名目的地在当前策略下不可达，并给出补建地图的步骤。

**推荐流程（换环境或发现规划失败时）：**

```bash
# 1. 重新建图，直到覆盖率不再增长
roslaunch ai_robot_nav mapping.launch

# 2. 存盘
rosservice call /mapper/save_map

# 3. 用新地图启动（allow_unknown 保持默认 false）
bash scripts/sim.sh --map /path/to/my_map.yaml
```

用 gmapping / cartographer 等工具建出的完整地图，通常边界外才有少量未知，`allow_unknown=false` 即可正常工作。

---

## 快速开始

### 一条命令跑仿真（日常推荐）

`scripts/sim.sh` 走的是 cafe 仿真，默认同时有漫游和目标点导航。

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
source /opt/ros/noetic/setup.bash && source ~/ROS_AI_Robot_Workspace_ws/devel/setup.bash
rostopic echo /odom/pose/pose/position
```

停止：`Ctrl+C` 或 `bash scripts/sim.sh stop`。

### 导航到指定目的地

仿真起来之后，另开一个终端：

```bash
source /opt/ros/noetic/setup.bash && source ~/ROS_AI_Robot_Workspace_ws/devel/setup.bash

rosrun ai_robot_nav send_goal --list      # 看有哪些命名目的地
rosrun ai_robot_nav send_goal hall        # 按名字去（需地图连通，见上文「地图与导航安全」）
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
bash scripts/sim.sh --allow-unknown  # 临时允许穿越未知区域（见「地图与导航安全」）
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

## 文件说明

两个 ROS 包：`src/tjark_agv` 是车和仿真世界，`src/ai_robot_nav` 是导航与安全。
网格、贴图和 `cafe.pgm` 是二进制资产，导航代码不解析贴图文件名。

### 仓库根与文档

| 文件 | 作用 |
|------|------|
| `README.md` | 本说明：架构、怎么跑、每个文件做什么 |
| `docs/platform.md` | 车辆几何、激光坐标、cafe 世界、硬件清单 |
| `docs/mapping.md` | 地图格式、建图、目标点导航、规划器选型 |
| `docs/real_robot.md` | 从仿真接到实车时的操作和安全底线 |
| `docs/roadmap.md` | 当前进度、未完成项、建议顺序 |
| `.gitignore` | 忽略 catkin/colcon 构建目录、Python 缓存和编辑器文件 |
| `.gitattributes` | 源码强制 LF；网格、贴图、PGM 按二进制保存，避免检出时被插进回车 |

### `scripts/`

在 WSL 里跑。Windows 上编辑源码，这些脚本负责安装、同步和启动。

| 文件 | 作用 |
|------|------|
| `sim.sh` | cafe 仿真加导航的日常入口：headless / GUI、纯激光、换地图、只起仿真或只起导航、停止 |
| `sync_ws.sh` | 把 `src/` 复制到 `~/ROS_AI_Robot_Workspace_ws` 再构建；`/mnt/c` 上直接 `catkin_make` 太慢 |
| `wsl_paths.sh` | 定义源码目录、catkin 工作空间、导航栈 overlay 的路径，被其它脚本 source |
| `ros_env.sh` | source Noetic、导航栈 overlay 和本仓库工作空间，并设置 Gazebo 模型路径 |
| `install_ros_noetic.sh` | 在 Ubuntu 24.04 WSL 上安装 ROS Noetic 与 Gazebo Classic，并做首次构建 |
| `install_nav_stack.sh` | 源码编译 `move_base`、`amcl`、`map_server` 等到独立的 nav 工作空间 |
| `align_wsl_layout.sh` | 把旧的 `~/ros_ws` 目录名迁到当前命名，并清理过期的 ROS 2 工作空间 |
| `run_tests.sh` | 跑 `ai_robot_nav` 的 pytest，不需要 Gazebo 或 Ollama |

### `src/ai_robot_nav/` 包描述

| 文件 | 作用 |
|------|------|
| `package.xml` | 包元数据与依赖。`move_base_msgs` 只作运行依赖，避免导航栈没装好时本包编译失败 |
| `CMakeLists.txt` | catkin 安装 Python 节点、launch、config 和 maps |
| `setup.py` | 把 `ai_robot_nav` 注册成可导入的 Python 包 |

### `src/ai_robot_nav/ai_robot_nav/` 逻辑

节点文件带 `rospy`；`navigator`、`command_mux`、`scan_utils`、`llm_client`、`motion`、`map_utils`、`goals`、`occupancy_grid` 可以脱离 ROS 图做单元测试。

| 文件 | 作用 |
|------|------|
| `__init__.py` | 包说明 |
| `ai_nav_node.py` | 读激光和可选相机，调用 `plan()`，发布 `/explore_cmd_vel` |
| `navigator.py` | 纯函数策略：净空迟滞、转向死区、方向锁存、脱困。视觉只能让它更慢或改平局 |
| `nav_mux_node.py` | 订阅漫游速度和 `move_base` 速度，发布 `/ai_cmd_vel` |
| `command_mux.py` | 仲裁规则：有目标点时用导航速度，指令过期就停车 |
| `safety_node.py` | 激光急停、速度限幅、上游超时零速；`/cmd_vel` 的唯一发布者 |
| `motion.py` | 线速度、角速度限幅，AI 节点和安全节点共用 |
| `scan_utils.py` | 按 `front_center_angle` 取前/左/右/后扇区距离；按圆形车身算还能直行多远；近距离展宽和短时记忆 |
| `llm_client.py` | 向 Ollama 要风险等级和方向偏好，解析失败就当这次没有视觉 |
| `ros_params.py` | 从参数服务器读数字、布尔和全局参数 |
| `log_throttle.py` | 按时间间隔压缩重复日志 |
| `lifecycle.py` | 安装 SIGINT/SIGTERM，让节点能干净退出 |
| `map_manager_node.py` | 启动前校验地图，加载旁边的命名目的地，打印哪些目标不可达 |
| `map_utils.py` | 解析 map_server 的 yaml/图片，检查覆盖率和目标是否走得通 |
| `goals.py` | 把目的地名字或坐标解析成同一个位姿 |
| `send_goal.py` | 命令行下发目标并等到抵达、失败或被打断 |
| `mapper_node.py` | 边走边建占据栅格，并用服务把地图存成 yaml + pgm |
| `occupancy_grid.py` | 对数几率更新、射线、PGM/YAML 导出 |

### `src/ai_robot_nav/scripts/` 节点入口

每个文件只调用同名模块的 `main()`，给 `rosrun` / `roslaunch` 用。

| 文件 | 作用 |
|------|------|
| `ai_nav_node` | 启动 `ai_nav_node.py` |
| `safety_node` | 启动 `safety_node.py` |
| `nav_mux_node` | 启动 `nav_mux_node.py` |
| `map_manager_node` | 启动 `map_manager_node.py` |
| `mapper_node` | 启动 `mapper_node.py` |
| `send_goal` | 启动 `send_goal.py` |

### `src/ai_robot_nav/launch/`

| 文件 | 作用 |
|------|------|
| `sim.launch` | cafe 仿真加导航层；默认同开漫游和目标点导航 |
| `real.launch` | 实车入口：车辆描述 + `ai_nav.launch`，文件里留有底盘、雷达、相机驱动的挂载位置 |
| `ai_nav.launch` | 漫游、仲裁、急停。不含仿真或硬件，两边共用 |
| `navigation.launch` | `map_server` + `amcl` + `move_base`，输出重映射到 `/nav_cmd_vel` |
| `mapping.launch` | 在仿真或实车里用漫游层走一圈，配合 `mapper_node` 建图 |

### `src/ai_robot_nav/config/`

| 文件 | 作用 |
|------|------|
| `nav_params.yaml` | 唯一的导航与安全参数，仿真和实车共用 |
| `move_base.yaml` | 全局规划用 A*（`global_planner`），局部用 DWA |
| `amcl.yaml` | 粒子滤波定位；针对前向 180° 雷达放宽了粒子和噪声 |
| `costmap_common.yaml` | 车体半径和障碍膨胀，全局、局部代价地图都加载 |
| `costmap_global.yaml` | 整张地图上的静态代价，坐标系是 `map` |
| `costmap_local.yaml` | 跟着车走的滚动窗口，坐标系是 `odom`，供局部避障 |
| `rosconsole_sim.config` | 仿真时把 Gazebo 和相机日志降到 WARN |

### `src/ai_robot_nav/maps/`

| 文件 | 作用 |
|------|------|
| `cafe.yaml` | 默认地图描述：分辨率 0.05 m、原点、占据阈值 |
| `cafe.pgm` | 对应的栅格图 |
| `cafe.destinations.yaml` | 这张图上的命名目的地，`send_goal hall` 读这里 |

### `src/ai_robot_nav/test/`

pytest，不启动 ROS 图。

| 文件 | 作用 |
|------|------|
| `test_navigator.py` | 策略由激光决定；视觉不能加速，也不能在明显更空的一侧选错方向 |
| `test_scan_utils.py` | 扇区、安装角、展宽、短时记忆；盲区必须保持为盲区 |
| `test_command_mux.py` | 目标点独占控制权，过期指令变成停车 |
| `test_llm_client.py` | 模型输出当不可信输入，垃圾文本要静默失败 |
| `test_map_utils.py` | 坏地图在启动前被拒绝，并指出该改哪一项 |
| `test_map_coverage.py` | 覆盖率统计，以及未知格子是否挡住路径 |
| `test_goals.py` | 名字和坐标两种输入都得到同一个位姿 |
| `test_occupancy_grid.py` | 射线、占据更新、PGM 导出 |

### `src/tjark_agv/` 车辆与仿真世界

| 文件 | 作用 |
|------|------|
| `package.xml` | 包元数据。导出 Gazebo 模型路径；没有 Gazebo 时仍可只用 URDF |
| `CMakeLists.txt` | 安装描述文件；找到 Gazebo 时才编译行人碰撞插件 |
| `launch/description.launch` | 把 xacro 放到 `robot_description` 并发布 TF。仿真和实车共用 |
| `launch/cafe_world.launch` | 启动 Gazebo、加载 cafe、把车放到世界里。只用于仿真 |
| `worlds/cafe.world` | 唯一的仿真世界：灯光、地面、cafe 模型和两名行人 |
| `plugins/ActorCollisionsPlugin.hh` | 插件声明：重新打开 Gazebo actor 的碰撞 |
| `plugins/ActorCollisionsPlugin.cc` | 插件实现。行人默认为无碰撞，不处理的话激光和车会穿过去 |

URDF 由 `urdf/tjark_agv.xacro` 串起来。`<gazebo>` 标签只有 gzserver 读，实车驱动会忽略。

| 文件 | 作用 |
|------|------|
| `urdf/tjark_agv.xacro` | 描述入口，按顺序引入下面五个文件 |
| `urdf/tjark_agv.urdf` | SolidWorks 导出的连杆、关节和 STL 引用 |
| `urdf/tjark_agv.sensor.xacro` | Gazebo 激光与深度相机插件 |
| `urdf/tjark_agv.motor.xacro` | 左右驱动轮的 `SimpleTransmission` |
| `urdf/tjark_agv.controller.xacro` | `libgazebo_ros_diff_drive.so`：订阅 `/cmd_vel`，发布 `/odom` |
| `urdf/tjark_agv.controller.yaml` | `gazebo_ros_control` 的轮速 PID。当前底盘不走这条通路 |
| `urdf/tjark_agv.color.xacro` | Gazebo 材质和摩擦 |

`meshes/` 是车体网格，由 URDF 引用：

| 文件 | 作用 |
|------|------|
| `meshes/base_link.STL` | 根连杆网格 |
| `meshes/base_body.STL` | 车体外壳 |
| `meshes/left_wheel.STL` / `right_wheel.STL` | 两个驱动轮 |
| `meshes/LF_link.STL` / `LB_link.STL` / `RF_link.STL` / `RB_link.STL` | 四个被动万向轮 |
| `meshes/laser_link.STL` | 雷达外壳 |
| `meshes/camera_link.STL` | 相机外壳 |

`models/` 是 cafe 世界引用的 Gazebo 模型，克隆后可离线加载。

| 文件 | 作用 |
|------|------|
| `models/cafe/model.config` | 把模型名 Cafe 指到 `model.sdf` |
| `models/cafe/model.sdf` | 咖啡厅场景的模型描述 |
| `models/cafe/meshes/cafe.dae` | 场景网格 |
| `models/cafe/materials/textures/__auto_*.jpg` | `cafe.dae` 用的贴图（`__auto_2` 到 `__auto_37` 中的 15 张） |
| `models/cafe_table/model.config` | 桌子模型入口 |
| `models/cafe_table/model.sdf` | 桌子的现行 SDF |
| `models/cafe_table/model-1_4.sdf` | 同一张桌子的 SDF 1.4 描述，含桌面和桌腿碰撞盒 |
| `models/cafe_table/meshes/cafe_table.dae` | 桌子网格 |
| `models/cafe_table/materials/textures/Maple.jpg` | 桌面木纹 |
| `models/cafe_table/materials/textures/Wood_Floor_Dark.jpg` | 深色木地板贴图 |

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
| `robot_footprint/radius` | `0.246` | 圆形车体半径，圆心在 `base_link` |
| `robot_footprint/laser_x` | `0.166` | 雷达在 `base_link` 前方的距离 |
| `forward_clearance` | `0.60` | 车身外廓还能直行这么远才直行 |
| `turn_clearance` | `0.45` | 车身外廓能直行的距离小于此值就转向（与上行构成迟滞带） |
| `tie_threshold` | `0.6` | 左右差距小于此值时保持上次转向方向 |
| `escape_commit_time` | `4.0` | 连续转向超过此秒数即锁死方向 |
| `escape_reverse_time` | `9.0` | 再超过此秒数改为边退边转脱困；`0` 关闭 |
| `stop_distance` | `0.10` | safety 急停：车身外廓能直行的距离小于此值 |

前方距离不是雷达读数，而是圆形车身沿当前朝向还能走多远。斜着靠近的墙在侧前方，
也会算进来。原地旋转不会扫到新位置，所以转向不检查车身。

雷达坐标与车辆几何见 [docs/platform.md](docs/platform.md)。

---

## 常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| 只左右转、不前进 | 雷达安装角没对上 | 确认 `front_center_angle: 180`，`sync_ws.sh --build` |
| 撞墙后左右摆头 | 左右扇区映射反了 | 已修复：四个扇区统一由 `front_center_angle` 推算 |
| 斜着蹭上墙，之后左转一点、前进一点来回摇头 | 前方只看雷达正前 ±12°，侧前方的墙进不来 | 已修复：前方按圆形车身能直行多远算，见 `robot_footprint` |
| 转很久出不来 | 车头顶住了障碍 | 已内置：4 s 锁死方向、9 s 边退边转 |
| Gazebo 启动卡住 | 联网找模型 | 已修复：模型内置且关闭在线模型库；请用 `scripts/sim.sh` |
| 一直停车 | `/scan` 未发布或 AI 超时 | 先启仿真；`rostopic echo /scan` |
| `send_goal` 报规划失败 | 地图不完整且 `allow_unknown=false` | **先读「地图与导航安全」**；补建地图；临时可用 `--allow-unknown` |
| Ollama 报错 | 服务未启动 | `bash scripts/sim.sh --lidar` 纯激光模式 |

---

## 许可证

`ai_robot_nav` 为 **Apache-2.0**，`tjark_agv` 沿用其原始的 **BSD**（见各自 `package.xml`）。

## 致谢

- tjark_agv —— 实车 URDF 与 Gazebo 模型，现已并入本仓库维护
- [Ollama](https://ollama.com/) + LLaVA —— 本地视觉推理
- 由 TurtleBot3 / ROS 2 原型迁移至 ROS 1，以匹配实车平台
