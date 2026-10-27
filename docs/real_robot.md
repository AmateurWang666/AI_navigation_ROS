# 从仿真到实车

## 最终目标

实车（ROS 1 Noetic 上的这台 tjark_agv）先自主完成**激光漫游和急停**：

- 没有地图、没有目标点
- `/scan` 决定直行或转向，视觉模型只能让车更慢或停下
- `safety_node` 是 `/cmd_vel` 的唯一发布者
- 雷达按已给出的模型使用：前向 180°，360 线，10 Hz，量程 0.12–20 m，正前方对应扫描角 180°

目标点导航（建图、`amcl`、`move_base`）排在漫游验收之后。仓库里这套代码已经能在
仿真里用，`real.launch` 尚未把它挂上。

## 现在到哪一步

只在 cafe 仿真里跑。

| 入口 | 实际拉起的内容 |
|------|----------------|
| `sim.launch` | Gazebo cafe、漫游、急停，并且**默认还有**地图和 `move_base` |
| `real.launch` | 车辆描述 + 漫游层（`ai_nav.launch`）。底盘和雷达驱动的位置是空的，没有 `navigation.launch` |

差速插件仍是轮距 **0.34 m**、轮径 **0.12 m**。实车数据是轮距 **0.3844 m**、轮径 **125 mm**。

## 实现路径

### 1. 仿真对齐实车尺寸

修改 `src/tjark_agv/urdf/tjark_agv.controller.xacro`：

- `wheelSeparation` → `0.3844`
- `wheelDiameter` → `0.125`

URDF 里左右轮关节 `y = ±0.192205` 保持不动。改完后仿真 `/odom` 的尺度才和实车几何一致。
`amcl` 的 `odom_alpha*` 先留着，等第四步再按新里程计收紧。

### 2. cafe 里验收激光漫游和急停

```bash
bash scripts/sim.sh --lidar
```

看四件事：前方空就直行；近障就转向，且转向朝更开阔的一侧；贴脸急停后不再往前挤；
通路打开后能从急停恢复。细柱靠扇区内展宽和短时记忆，整扇区无效（`None`）仍必须急停。

cafe 行人带圆柱碰撞体，用来确认车会绕开或停下，而不是从身体里穿过去。

### 3. 实车挂底盘和雷达，空场标定

在 `real.launch` 里已经标出的位置接入：

| 需要 | 话题 | 类型 |
|------|------|------|
| 激光雷达驱动 | `/scan` | `sensor_msgs/LaserScan` |
| 底盘驱动 | 订阅 `/cmd_vel` | `geometry_msgs/Twist` |
| 里程计（底盘一并给出即可） | `/odom` | `nav_msgs/Odometry` |

相机（`/my_camera/color/image_raw`）不是这一步的条件。话题名对不上时改
`nav_params.yaml`，不要改节点。底盘若只接受 `TwistStamped`，把 `safety_node` 的
`cmd_stamped` 改成 `true`。

`real.launch` 目前没有 `idle_behavior` 参数，一挂上驱动就会按 `ai_nav.launch` 的默认值开始漫游。接驱动时把这个参数传进 `ai_nav.launch`。首次上电用 `stop`，确认急停之后再改成 `explore`。

空场要核对的三项：

1. **`front_center_angle`。** 给出的值是 `180`。正前方 1 m 放一个箱子，在 `/scan` 里找出最小距离的下标 `i`，核对 `degrees(angle_min + i * angle_increment)` 是不是 180。对得上就不要改；对不上左右会反，车会朝障碍转。日志里的 `turning toward more open side (left …, right …)` 应和眼睛看到的左右一致。
2. **`stop_distance` / `clear_distance`。** 仿真初值 0.24 / 0.34 m 从雷达算起，雷达在 `base_link` 前方 0.166 m。按车头外廓和实际制动距离重测，并保持 `clear_distance > stop_distance`。
3. **`escape_reverse_time` 置 `0`。** 雷达后方没有采样点。没有后向传感器或防撞条时不要边退边转，脱困只原地转。

速度从 `nav_params.yaml` 的保守初值开始：先关视觉，原地确认 `turn_speed` 转得动，再以
`cruise_speed` 直行测制动，然后才加 `cruise_speed` 和 `max_linear`。视觉最后开，它只能让车更慢。

硬件型号见 [platform.md](platform.md#实车硬件清单)。本仓库没有底盘、雷达、IMU 驱动。

### 4. 再上地图和目标点导航

漫游和急停在实车上稳定之后，再把 `navigation.launch` 加进实车启动，并准备该环境的地图。
步骤、`allow_unknown` 和规划器选型见 [mapping.md](mapping.md)。

这一步才需要可信的 `/odom`。开发机是 Ubuntu 24.04，导航栈要用
`scripts/install_nav_stack.sh` 源码编译。实车若是 Ubuntu 20.04 上的官方 Noetic，
安装 `ros-noetic-navigation` 即可，不必走那条源码路径。

## 为什么漫游节点不用改写

仓库刻意把「平台」与「策略」分开，两者只通过话题与一份参数文件往来：

```
                 sim.launch                        real.launch
                     │                                  │
        ┌────────────┴───────────┐          ┌───────────┴────────────┐
        │  tjark_agv/            │          │  tjark_agv/            │
        │  cafe_world.launch     │          │  description.launch    │
        │  （Gazebo 插件出话题） │          │  + 实车驱动（待补）    │
        └────────────┬───────────┘          └───────────┬────────────┘
                     │  /scan /my_camera/... /odom      │
                     │  ◄── /cmd_vel                    │
        ┌────────────┴──────────────────────────────────┴────────────┐
        │  ai_robot_nav/ai_nav.launch —— 仿真与实车完全相同           │
        │  ai_nav_node（决策）→ /ai_cmd_vel → safety_node → /cmd_vel  │
        └─────────────────────────────────────────────────────────────┘
```

- URDF 是同一份（`tjark_agv/launch/description.launch`）。xacro 里的 `<gazebo>`
  标签只有 gzserver 读，实车侧会忽略。
- 参数是同一份（`ai_robot_nav/config/nav_params.yaml`）。
- 决策逻辑不读时钟、不读 TF、不依赖仿真：`navigator.plan` 是纯函数，输入是四个
  扇区距离、上一周期动作和已连续转向的秒数，因此仿真里跑通的行为在实车上可复现。

## 安全底线

不要绕过这些约束：

- `safety_node` 是 `/cmd_vel` 的唯一发布者，也是最后一道闸。不要让别的节点直接发
  `/cmd_vel`。
- `passthrough_mode: true` 会让 `safety_node` 完全不发布，只用于手动遥控调试，
  自主运行时必须为 `false`。
- `ai_nav_node` 崩溃时 `safety_node` 会因 `/ai_cmd_vel` 超时而输出零速；反之
  `safety_node` 不在时没有任何东西会发 `/cmd_vel`。这个「AI 挂了车会停，安全层挂了
  车不动」的方向是刻意设计的，改动时别把它反过来。
- 实车首次自主运行请垫高驱动轮或留人手持急停。
