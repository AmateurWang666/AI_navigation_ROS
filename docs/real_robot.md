# 从仿真挪到实车

本项目当前只在仿真中运行。这份清单说明上车前需要做什么，以及为什么导航代码
本身不用改。

## 为什么导航代码不用改

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

## 一、挂上硬件驱动

在 `ai_robot_nav/launch/real.launch` 标注的位置引入三个驱动，让它们对齐
`nav_params.yaml` 里约定的话题：

| 需要 | 话题 | 类型 |
|------|------|------|
| 激光雷达驱动 | `/scan` | `sensor_msgs/LaserScan` |
| 相机驱动（可选，仅视觉层用） | `/my_camera/color/image_raw` | `sensor_msgs/Image` |
| 底盘驱动 | 订阅 `/cmd_vel` | `geometry_msgs/Twist` |
| 里程计（推荐） | `/odom` | `nav_msgs/Odometry` |

话题名对不上时**改 yaml，不要改代码**。底盘若只接受 `TwistStamped`，把
`safety_node` 的 `cmd_stamped` 改成 `true` 即可，无需改节点。

硬件型号见 [platform.md](platform.md#实车硬件清单)。本仓库不含任何实车驱动。

## 二、必须复核的三项参数

这三项在仿真里的取值来自 Gazebo 模型，实车不一定相同，配错的后果都比较严重。

### 1. `front_center_angle` —— 雷达安装角

含义是「车体正前方对应的扫描角」。仿真中雷达绕 z 轴装反了 180°，所以是 `180.0`。

**实车必须实测**，否则左右会整体反过来，机器人会朝障碍物转（这个 bug 真实发生过）。
最简单的验证方法：把车停在空地上，正前方 1 m 放一个箱子，然后

```bash
rostopic echo /scan | head -40      # 看 angle_min / angle_increment
```

找出最小距离所在的下标 `i`，则 `front_center_angle = degrees(angle_min + i * angle_increment)`。
装好后在空场地上跑一次，确认它总是朝更开阔的一侧转（日志里会打印
`turning toward more open side (left …, right …)`，把数值和眼睛看到的比一下）。

### 2. `stop_distance` / `clear_distance` —— 急停区

仿真值 0.24 / 0.34 m 是从**雷达**算起的，而雷达装在 `base_link` 前方 0.166 m。
实车需要按车头实际外廓 + 制动距离重新给值，并保证 `clear_distance > stop_distance`
（否则迟滞失效，会在停/走边界抖动）。

### 3. `escape_reverse_time` —— 脱困后退

长时间转不出去时，策略会以最低速边退边转把车头从障碍上拽出来。**本车雷达只有
前向 180° 视场，后方是完全的盲区。** 实车如果没有后向传感器或防撞条，请置 `0`
关闭该动作；此时策略退化为纯原地转向（对差速+万向轮底盘通常够用）。

## 三、速度标定顺序

`nav_params.yaml` 里的速度按较小底盘留了保守余量。建议顺序：

1. **先关视觉**：`roslaunch ai_robot_nav real.launch use_image:=false`，
   排除 Ollama 带来的变量，确认纯激光行为正常。
2. **原地测转向**：确认 `turn_speed` 下车辆能原地转、不打滑、不憋停。
3. **测制动**：以 `cruise_speed` 直行并触发急停，量实际停车距离，反推 `stop_distance`。
4. **再提速**：`cruise_speed`、`max_linear` 逐步上调，每次都重测第 3 步。
5. **最后开视觉**：确认 Ollama 只会让车更慢（这条有单元测试保证，实车再核一遍）。

## 四、里程计与轮距

`platform.md` 记录了一处已知的参数不一致：URDF 关节几何给出轮距 0.3844 m，
diff_drive 插件用的是 0.34 m。闭环反应式导航对此不敏感，所以仿真里没改；
但实车里程计精度直接取决于真实轮距与轮径，上车前需要实测：

- **轮距**：让车原地转整 10 圈，用 `/odom` 的偏航角误差反推；
- **轮径**：让车直行 5 m（卷尺实测），与 `/odom` 位移比对。

标定后把 `tjark_agv.controller.xacro` 的 `wheelSeparation` / `wheelDiameter`、
URDF 关节位置、以及实车底盘固件三处统一。

## 五、安全底线（不要绕过）

- `safety_node` 是 `/cmd_vel` 的唯一发布者，也是最后一道闸。不要让别的节点直接发
  `/cmd_vel`。
- `passthrough_mode: true` 会让 `safety_node` 完全不发布，只用于手动遥控调试，
  自主运行时必须为 `false`。
- `ai_nav_node` 崩溃时 `safety_node` 会因 `/ai_cmd_vel` 超时而输出零速；反之
  `safety_node` 不在时没有任何东西会发 `/cmd_vel`。这个「AI 挂了车会停，安全层挂了
  车不动」的方向是刻意设计的，改动时别把它反过来。
- 实车首次自主运行请垫高驱动轮或留人手持急停。
