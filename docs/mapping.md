# 地图与目标点导航

本文说明两件事：**怎么换成你自己的地图**，以及**怎么让机器人自动去指定目的地**。

导航依赖三样东西，缺一不可：

| 要素 | 由谁提供 | 说明 |
|------|----------|------|
| 地图 | `map_server` | 环境的栅格图，全局规划的依据 |
| 定位 | `amcl` | 估计车在地图上的位姿，修正里程计漂移 |
| 规划 | `move_base` | 全局路径 + 局部避障 |

这三个包**不在本仓库里**，需要先装。原因见下一节。

---

## 一、先决条件：安装导航栈

```bash
bash scripts/install_nav_stack.sh
```

### 为什么要源码编译，而不是 apt 安装

本机是 **Ubuntu 24.04 (noble)**，而 ROS Noetic 官方只发布 **Ubuntu 20.04 (focal)**
的二进制包。现有的 Noetic 来自社区 PPA `ros-for-jammy`，那个 PPA 只有 276 个包，
**不含任何导航组件**——`move_base`、`amcl`、`map_server`、`costmap_2d` 全都没有。

所以换 apt 镜像源是解决不了的：这些包对本发行版根本不存在。好在导航栈的依赖
（`tf2_ros`、`pcl_ros`、`laser_geometry`、`nodelet`、`dynamic_reconfigure` 等）
在 PPA 里齐全，`map_server` 需要的 `libsdl1.2-dev` 也在 Ubuntu 官方源里，
因此源码编译可行。脚本会把它们装到独立的工作空间 `~/ros_nav_ws`，
由 `scripts/ros_env.sh` 作为底层 overlay 自动加载，不干扰日常的
`sync_ws.sh` 快速构建循环。

### 编译内存不足怎么办

导航栈是 C++ 的，`costmap_2d`、`move_base` 这类重模板代码单个 `cc1plus` 峰值
可达 1–2 GB。本机 32 核但只有 10 GB 内存，`catkin_make` 默认开满 32 路必然
把内存吃穿。脚本因此**按内存而不是按核数**推算并行度（每任务留 1.5 GB，上限 6）。

若仍看到 `Killed signal terminated program cc1plus` 或 `internal compiler error`，
按这个顺序处理：

```bash
# 1. 先降并行度——这比加内存有效得多
NAV_BUILD_JOBS=2 bash scripts/install_nav_stack.sh

# 2. 仍失败再调高 WSL2 内存：编辑 %UserProfile%\.wslconfig
#      [wsl2]
#      memory=16GB
#    然后在 PowerShell 里 wsl --shutdown 重启生效
```

---

## 二、用你自己的地图

### 地图格式

采用 **ROS 标准的 map_server 格式**，不自造格式。所以你用 gmapping、
cartographer、hector_slam 或任何其它工具建的图都能直接用，不需要转换。
一张地图由两个文件组成：

```
我的地图.yaml     描述文件
我的地图.pgm      栅格图片（也可以是 png/bmp）
```

`.yaml` 的内容形如：

```yaml
image: 我的地图.pgm      # 相对路径按本 yaml 所在目录解析
resolution: 0.05         # 米/像素
origin: [-10.0, -10.0, 0.0]   # 图片左下角在世界坐标中的 [x, y, yaw]
negate: 0
occupied_thresh: 0.65    # 高于此占据概率视为障碍
free_thresh: 0.196       # 低于此视为空闲
```

### 使用

```bash
roslaunch ai_robot_nav sim.launch map_file:=/路径/到/我的地图.yaml
```

启动时 `map_manager` 节点会先校验这张图，有问题立刻停下并指出该改哪里，
而不是让 `map_server` 抛一个 SDL 底层错误、然后下游表现成「机器人莫名不动」。
被拦截的情况包括：文件不存在、YAML 格式错误、必需字段缺失、
`resolution` 非正、图片文件找不到、以及 `free_thresh` 与 `occupied_thresh` 倒挂。

> **`origin` 决定一切坐标的含义。** 它是图片左下角在世界坐标系里的位置。
> 给错了地图不会报错，但所有目标点坐标都会整体偏移，机器人会「精确地开到错误的地方」。

### 换仿真场景

地图和 Gazebo 世界是两件事，都要换，而且**必须对应同一个环境**，否则定位一直发散：

```bash
roslaunch ai_robot_nav sim.launch \
    world_name:=/路径/我的世界.world \
    map_file:=/路径/我的地图.yaml \
    spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0
```

`spawn_*` 同时用作 Gazebo 出生点和 AMCL 的初始位姿猜测——两者本就该一致，
分开写只会在改动时忘掉另一处，然后表现为「一启动就朝墙走」。

---

## 三、为新环境建图

如果你还没有地图，本仓库自带一个建图节点。

> 它**不做扫描匹配，直接信任里程计**，所以严格说是「已知位姿的建图」而不是 SLAM。
> 仿真里 `diff_drive` 插件的里程计精度很高，建出来的图足够干净；
> **实车上里程计会漂移，走大回环时地图会糊**。实车建议改用真正的 SLAM 方案，
> 或者直接拿别处建好的图来用——上一节的加载路径对任何来源的地图都通用。

```bash
# 1. 启动建图。机器人靠本项目原有的反应式漫游层自己游走避障，
#    扫过的地方逐步成图，不需要遥控。
roslaunch ai_robot_nav mapping.launch

# 2. 另开终端，看覆盖率不再增长时存盘
rosservice call /mapper/save_map
```

默认存到 `/tmp/my_map.{pgm,yaml}`，改存盘位置：

```bash
roslaunch ai_robot_nav mapping.launch save_path:=$(rospack find ai_robot_nav)/maps/我的地图
```

环境比 40×40 m 更大时要调大栅格，否则超出范围的观测会被丢弃，
表现为地图边缘出现一圈永远填不上的未知区域：

```bash
roslaunch ai_robot_nav mapping.launch width:=1200 height:=1200
```

存盘功能自带、不依赖 `map_server` 的 `map_saver`，因为建图往往正是
「导航栈还没装好」时要做的第一步。装好之后 `rosrun map_server map_saver -f 我的地图`
也一样能用。

---

## 四、导航到指定目的地

### 三种下发方式

```bash
# 1. 按坐标（地图坐标系，米/弧度）
rosrun ai_robot_nav send_goal 2.5 3.0
rosrun ai_robot_nav send_goal 2.5 3.0 1.57      # 附带抵达朝向

# 2. 按名字
rosrun ai_robot_nav send_goal 前台
rosrun ai_robot_nav send_goal --list            # 看有哪些名字

# 3. RViz 里点 "2D Nav Goal"
```

`send_goal` 走 actionlib 而不是直接发话题，因此会**阻塞到有结论为止**并回传结果：
抵达、规划失败、目标被拒、还是中途被打断。退出码非零表示未抵达，可以直接用在脚本里。

### 命名目的地

在地图旁边放一个同名的 `.destinations.yaml`，它会被自动加载：

```
maps/我的地图.yaml
maps/我的地图.pgm
maps/我的地图.destinations.yaml      <-- 自动找到
```

内容：

```yaml
前台: [2.5, 3.0, 0.0]        # [x, y, yaw]，yaw 可省略
厨房: [-1.0, 4.5, 1.57]
充电桩:
  x: 0.0
  y: 0.0
  yaw: 3.14
```

目的地跟着地图走是有意的：换一张地图就换一套目的地，两者天然对应，
不会出现「拿 A 图的坐标去 B 图导航」这种既不报错、结果又完全不对的情况。

---

## 五、它是怎么接进原有架构的

原本这个项目是**纯反应式**的：没有地图、没有定位、没有目标点，机器人只是避障漫游。
加入 `move_base` 后系统里出现了两个速度来源，必须选一路：

```
用户地图 ──► map_server ──► /map ─┐
                                   ├──► move_base ──► /nav_cmd_vel ─┐
/scan + /odom ──► amcl ──► map→odom ┘                               │
                                                                    ├──► nav_mux
ai_nav_node（反应式漫游）───────────────► /explore_cmd_vel ────────┘
                                                                    │
                                                                    ▼
                                                       /ai_cmd_vel ──► safety_node ──► /cmd_vel
```

### 为什么不让 move_base 直接发 /cmd_vel

本仓库有一条贯穿始终的不变式：**`safety_node` 是 `/cmd_vel` 的唯一发布者**，
它独立于上层逻辑，靠激光做急停。`move_base` 默认发 `/cmd_vel`，直接接进来
就等于在安全层旁边开了一个后门——它崩溃、卡在恢复行为、或者规划出一条贴墙
路径时，没有任何东西能拦住。

所以 `move_base` 的输出被重映射到 `/nav_cmd_vel`，经 `nav_mux` 汇流到
`/ai_cmd_vel`，仍然要过 `safety_node` 才能到底盘。**急停权不下放。**

### 仲裁规则

| 情况 | 输出 |
|------|------|
| 有目标点且 move_base 指令新鲜 | move_base 的速度 |
| 有目标点但 move_base 断流超时 | **零速**（不沿用旧指令） |
| 无目标点，`idle_behavior=explore` | 反应式漫游 |
| 无目标点，`idle_behavior=stop` | 零速 |

「断流即停」是刻意的：旧指令是针对旧位置算出来的，而 `move_base` 断流往往正是
因为它在规划或执行恢复行为——恰恰是最不该盲目前进的时刻。

不启动 `navigation.launch` 时，`/move_base/status` 永远没有消息，`nav_mux` 判定
无目标点并直接透传漫游指令，**行为与引入 move_base 之前完全一致**，
原有的纯反应式模式不受影响：

```bash
roslaunch ai_robot_nav sim.launch navigation:=false
```

---

## 六、规划器选型依据

### 全局规划：`global_planner/GlobalPlanner`，启用 A*

ROS 1 导航栈里有两个全局规划器，都在 `ros-planning/navigation` 仓库内：

| 实现 | 说明 |
|------|------|
| `navfn` | 2008 年的原版，Dijkstra，参数几乎不可调 |
| `global_planner` | 后来重写的版本，可选 Dijkstra 或 A*，可选路径提取方式 |

选后者。两个关键参数：

- **`use_dijkstra: false`** → 用 A*。有目标点作为启发，展开的节点数远少于
  Dijkstra，室内地图上规划耗时通常低一个量级。若发现路径明显绕远，
  改回 `true` 用 Dijkstra 求全局最优。
- **`use_grid_path: false`** → 沿势场梯度下降提取路径。这样走出来的路径不受
  栅格八方向限制，天然接近 **any-angle**（也就是 Theta\* 追求的效果），
  省掉了后处理去锯齿这一步。设成 `true` 则严格沿栅格走，会有明显的 45° 阶梯锯齿。

**为什么不用 Hybrid A\* / SBPL**：那类算法解决的是「非完整约束 + 最小转弯半径」，
典型对象是阿克曼转向的车。tjark_agv 是差速底盘，**可以原地旋转，不存在最小
转弯半径**，用它们只增加计算量和调参负担，换不来更短的路径。

### 局部规划：`dwa_local_planner/DWAPlannerROS`

DWA 在**速度空间**而不是位置空间采样，对差速底盘的加速度约束建模更直接，
低速下轨迹质量明显优于更老的 `base_local_planner`。

**为什么暂时不用 TEB**：`teb_local_planner` 不在 `navigation` 仓库里，是独立包且
依赖 g2o 图优化库，本机 Ubuntu 24.04 上要连 g2o 一起从源码编译，代价不小。
DWA 对本车这种低速室内差速小车已经够用。若将来需要更平滑的绕障，
TEB 是明确的下一步升级方向。

### 本车特有的两处约束

- **禁止倒车**（`min_vel_x: 0.0`）。本车雷达只有**前向 180° 视场**，车后完全没有
  采样点，代价地图无法清除车后的历史障碍。倒车等于闭眼后退，脱困一律用原地旋转。
- **恢复行为只保留原地旋转**。旋转能让雷达扫到原本看不见的方向，
  这恰好是本车最需要的补救手段。

---

## 七、常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| 启动就报「地图不可用」 | `map_file` 路径或格式有问题 | 按提示改；提示会指出具体是哪个字段 |
| 一启动就朝墙走 | AMCL 初始位姿不对 | 核对 `initial_pose_x/y/a`，或在 RViz 里用 2D Pose Estimate 纠正 |
| 定位慢慢发散 | 地图与实际环境不符 | 确认 world 与 map 是同一个环境；本车前向 180° 视场本就比 360° 雷达更易发散 |
| `send_goal` 等不到 move_base | 导航栈没装或没启动 | `bash scripts/install_nav_stack.sh`，确认没加 `navigation:=false` |
| 目标点被拒绝（REJECTED） | 目标落在地图外或障碍里 | 换个点；注意坐标是**地图坐标系**，受 `origin` 影响 |
| 规划失败（ABORTED） | 到目标点没有可行路径 | 检查膨胀半径是否过大把通道堵死（`inflation_radius`） |
| 机器人原地转圈不走 | 局部规划器选不出可行轨迹 | 多半是 `min_vel_trans` 过大或代价地图把车围死；先看 RViz 里的局部代价地图 |
| 报 Extrapolation Error | tf 时间戳容差不够 | 调大 `transform_tolerance`（WSL 里 Gazebo 时钟抖动大） |
