# tjark_agv 平台说明

`src/tjark_agv` 是本仓库内的车辆描述包，也是**仿真与实车共享的唯一几何/传感器
真相源**。它原本是一个独立的外部 catkin 包（SolidWorks 导出的 URDF），现已并入
本仓库统一维护，不再依赖仓库外的任何路径。

上层导航逻辑在 `src/ai_robot_nav`，与本包解耦：两者之间只通过话题与参数文件
约定往来，换到实车时导航代码不需要改动。

---

## 目录

```
src/tjark_agv/
├── urdf/          车辆描述（见下）
├── meshes/        10 个 STL，共约 7.2 MB，base_body.STL 占 5.1 MB
├── worlds/        cafe.world —— 唯一仿真世界
├── models/        cafe、cafe_table —— cafe.world 引用的 Gazebo 模型
└── launch/        description.launch（仿真+实车共用）、cafe_world.launch（仅仿真）
```

`worlds/` 与 `models/` 必须成对存在。`cafe.world` 通过 `model://cafe` 引用场景
模型，而这两个模型原本要从 Gazebo 在线模型库下载并缓存到 `~/.gazebo/models`。
把它们放进仓库并把路径加进 `GAZEBO_MODEL_PATH`（`scripts/ros_env.sh` 会做，
`package.xml` 的 `gazebo_ros` 导出也会做）之后：

- 仿真可完全离线运行，任何机器上克隆即用
- `/scan` 从启动到就绪约 3 秒，而不是等 gzserver 联网找模型的几分钟
  （`GAZEBO_MODEL_DATABASE_URI` 已被置空，彻底关掉在线查询）

`ground_plane` 由 Gazebo 11 自带，无需内置。

---

## URDF 组成

`urdf/tjark_agv.xacro` 是唯一入口，按顺序引入：

| 文件 | 内容 |
|------|------|
| `tjark_agv.urdf` | 连杆、关节、STL 引用（SolidWorks 导出的本体） |
| `tjark_agv.sensor.xacro` | Gazebo 激光与深度相机传感器及插件 |
| `tjark_agv.motor.xacro` | 两个驱动轮的 `SimpleTransmission` |
| `tjark_agv.controller.xacro` | `libgazebo_ros_diff_drive.so` 差速驱动插件 |
| `tjark_agv.color.xacro` | Gazebo 材质与摩擦参数 |

`<gazebo>` 标签只有 gzserver 会读，`robot_state_publisher` 与实车驱动会忽略，
所以实车沿用同一份 xacro 即可，不需要按平台分叉。

`motor.xacro` 的 transmission 与 `urdf/tjark_agv.controller.yaml` 的 PID 参数配成
一套 `gazebo_ros_control` 通路。当前底盘由 diff_drive 插件直接驱动，**这条通路处于
待用状态**；将来实车改走 `ros_control` 时再启用。

---

## 连杆与话题

| 连杆 | 说明 |
|------|------|
| `base_link` | 根坐标系，位于两驱动轮轴线上 |
| `base_body` | 车体外壳（5.86 kg） |
| `left_wheel` / `right_wheel` | 两个驱动轮（`continuous` 关节） |
| `laser` | RPLidar；位于 `base_link` **前方 0.166 m** |
| `camera_link` | Orbbec Astra Pro Plus 深度相机 |
| `LF/LB/RF/RB_link` | 四个被动万向轮 |

| 话题 | 类型 | 方向 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | 出 |
| `/my_camera/color/image_raw` | `sensor_msgs/Image` | 出 |
| `/my_camera/depth/points` | `sensor_msgs/PointCloud2` | 出（本项目未用） |
| `/odom` | `nav_msgs/Odometry` | 出 |
| `/cmd_vel` | `geometry_msgs/Twist`（**不是** TwistStamped） | 入 |

---

## 激光坐标约定（最容易出错的一处）

`laser` 连杆相对 `base_link` **绕 z 轴装反了 180°**。用 tf 实测可得：

```
R_base_laser = Rz(pi)
```

激光视场是传感器坐标下的 90°–270°（180° 视场、360 线、10 Hz、量程 0.12–20 m）。
换算到车体坐标：

| 扫描角 | 车体方位 | 含义 |
|--------|----------|------|
| 180° | 0° | **正前方** |
| 270°（= `angle_max`） | +90° | 左侧 |
| 90°（= `angle_min`） | −90° | 右侧 |
| 视场外 | 180° | 正后方**无任何采样点**，是盲区 |

导航侧只用一个参数表达这件事：`nav_params.yaml` 的 `front_center_angle: 180.0`，
含义是「车体正前方对应的扫描角」。前、左、右、后四个扇区全部由它推算
（见 `scan_utils.describe_environment`），所以换雷达或改安装角时只改这一个值。

> 这里曾经出过一个真实的 bug：偏移只加在前方扇区上，侧向仍按 ±90° 取，于是算出来
> 的「左」其实是车体右侧。机器人因此总朝更封闭的一侧转，贴墙时来回摆头且无法脱困。

后方是盲区，因此策略默认不会盲目倒车：`navigator.plan` 只在后方**确有**足够读数时
才输出 `REVERSE`。唯一的例外是长时间转不出去后的脱困动作（`escape_reverse_time`），
它会以最低速短暂后退，实车若无后向感知需将该项置 0。

---

## 车辆几何与已知的参数不一致

| 量 | URDF 关节几何 | diff_drive 插件 | CAD 零件名 |
|----|---------------|-----------------|-----------|
| 轮距 | ±0.192205 → **0.3844 m** | `wheelSeparation` **0.34** | — |
| 轮径 | — | `wheelDiameter` **0.12** | 125 mm |

三者互不一致。仿真里 `odom` 的尺度由插件值决定，闭环反应式导航对此不敏感，因此
**本仓库保留原值不动**，避免悄悄改变已跑通的仿真行为。但这是一笔明确的技术债：

- 实车里程计精度直接取决于真实轮距与轮径，上车前必须实测并统一三处取值；
- 标定方法见 [real_robot.md](real_robot.md)。

---

## 实车硬件清单

以下型号来自 SolidWorks 导出件号（原始 `tjark_agv.csv`，内容为 CAD 元数据、
中文已在导出时损坏，故未纳入仓库，信息提取到此表）：

| 部位 | 型号 / 规格 |
|------|-------------|
| 驱动电机 | Z4BLD60-24GN-30S（24 V 无刷） |
| 驱动轮 | 125 mm × 38 mm × 18 mm，两个 |
| 万向轮 | 70 mm（2 寸），四个 |
| 激光雷达 | RPLidar 系列，装配件号 WH-0545-XNJ04 |
| 相机 | Orbbec Astra Pro Plus |
| IMU | JY263-B |
| 电池 | 24 V 20 Ah（260 × 136 × 40 mm） |
| 路由器 | TP-LINK TL-WR842N 300M |

**本包只有仿真与描述，不含任何实车驱动**：没有串口/CAN 驱动、没有电机控制节点、
没有 IMU 与编码器里程计发布器。这些需要在实车阶段补齐，挂载点见
`ai_robot_nav/launch/real.launch`。
