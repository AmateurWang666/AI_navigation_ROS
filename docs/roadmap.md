# 进度与后续顺序

这里只记三件事：现在做到哪、哪些还没做、建议按什么顺序做。
车辆数据见 [platform.md](platform.md)，上车时的操作见 [real_robot.md](real_robot.md)，
建图和目标点见 [mapping.md](mapping.md)。

## 最终目标

实车在 ROS 1 上先自己完成激光漫游和急停：

- 没有地图、没有目标点
- `/scan` 决定走和转，视觉模型只能让车更慢或停下
- `safety_node` 是 `/cmd_vel` 的唯一发布者

目标点导航（地图、`amcl`、`move_base`）排在实车漫游验收之后。
尺寸和雷达以 [platform.md](platform.md) 里的实车数据为准，不另做一套仿真规格。

## 现在到哪一步

停在 cafe 仿真。下面四步里，第一步还没做。

| 入口 | 现在实际拉起的内容 |
|------|--------------------|
| `sim.launch` | Gazebo cafe、漫游、急停，并且默认还有地图和 `move_base` |
| `real.launch` | 车辆描述 + 漫游层（`ai_nav.launch`）。底盘和雷达驱动的位置是空的，没有 `navigation.launch` |

差速插件仍是轮距 **0.34 m**、轮径 **0.12 m**。实车是轮距 **0.3844 m**、轮径 **125 mm**。
因此仿真 `/odom` 的尺度和实车几何还不一致。

仓库里已经有两套能力，共用同一条指令链路和同一道安全闸。实车按下面的顺序用，不是两套一起上车：

| 方式 | 现在何处可用 | 实车顺序 |
|------|----------------|----------|
| 反应式漫游 | 仿真与 `real.launch` 的节点都有；实车驱动还没挂 | 先上车 |
| 目标点导航 | `sim.launch` 默认会拉起 `move_base` | 漫游验收之后；`real.launch` 尚未包含 |

## 已经有的

- 反应式漫游、指令仲裁、激光急停，仿真里可以跑
- 可选的 Ollama 视觉收敛层；不启动时自动退回纯激光
- 仿真里的目标点导航（`map_server`、`amcl`、`move_base`），输出被重映射到 `/nav_cmd_vel`，过安全闸
- cafe 世界、车辆描述、一份覆盖率约 17% 的演示地图
- 轻量建图节点、地图校验、命名目的地
- 不依赖 Gazebo 和 Ollama 的单元测试
- `real.launch` 骨架：描述和漫游层已接好，硬件驱动槽是空的

## 还没做

1. **仿真尺寸未对齐实车。** `tjark_agv.controller.xacro` 的 `wheelSeparation` / `wheelDiameter` 仍是 0.34 和 0.12。
2. **激光漫游和急停还没有在对齐后的 cafe 里验收。** 代码在跑，验收标准还没过一遍。
3. **实车驱动没挂。** 没有底盘、雷达、IMU、编码器里程计节点。`real.launch` 一挂上驱动就会按默认值开始漫游，接驱动时还要把 `idle_behavior` 传进去。
4. **空场标定没做。** `front_center_angle`、`stop_distance` / `clear_distance`、`escape_reverse_time` 都还是仿真初值。
5. **目标点导航还没进实车启动。** `real.launch` 不包含 `navigation.launch`。`amcl` 的 `odom_alpha*` 仍取 0.3，是为了盖住当前里程计偏差，插件改齐之后再收紧。
6. **`gazebo_ros_control` 那条通路还没用。** `motor.xacro` 和 `tjark_agv.controller.yaml` 配好了，当前底盘由差速插件直接驱动。

相机不是第 3 步的条件。

## 建议顺序

### 1. 仿真对齐实车尺寸

改 `src/tjark_agv/urdf/tjark_agv.controller.xacro`：

- `wheelSeparation` → `0.3844`
- `wheelDiameter` → `0.125`

URDF 里左右轮关节 `y = ±0.192205` 保持不动。不要改关节去迁就插件。
`amcl` 的 `odom_alpha*` 先留着，等第 4 步再按新里程计收紧。

### 2. 在 cafe 里验收激光漫游和急停

```bash
bash scripts/sim.sh --lidar
```

看四件事：前方空就直行；近障就转向，且转向朝更开阔的一侧；贴脸急停后不再往前挤；
通路打开后能从急停恢复。细柱靠扇区内展宽和短时记忆，整扇区无效（`None`）仍必须急停。
cafe 行人带圆柱碰撞体，用来确认车会绕开或停下，而不是从身体里穿过去。

### 3. 实车挂底盘和雷达，空场标定

在 `real.launch` 里已经标出的位置接入订阅 `/cmd_vel` 的底盘、发布 `/scan` 的雷达，
以及 `/odom`。话题名对不上时改 `nav_params.yaml`，不要改节点。
首次上电用 `idle_behavior:=stop`，确认急停之后再改成 `explore`。

要核对的三项、速度从哪一档加起，写在 [real_robot.md](real_robot.md)。
雷达后方没有采样点；没有后向传感器或防撞条时，把 `escape_reverse_time` 置 `0`。

### 4. 再上地图和目标点导航

漫游和急停在实车上稳定之后，再把 `navigation.launch` 加进实车启动，并准备该环境的地图。
步骤、`allow_unknown` 和规划器选型见 [mapping.md](mapping.md)。

这一步才需要可信的 `/odom`。开发机是 Ubuntu 24.04，导航栈用
`scripts/install_nav_stack.sh` 源码编译。实车若是 Ubuntu 20.04 上的官方 Noetic，
安装 `ros-noetic-navigation` 即可。

## 不要绕过的约束

改顺序或加节点时仍要守住这些，操作细节在 [real_robot.md](real_robot.md#安全底线)：

- `safety_node` 是 `/cmd_vel` 的唯一发布者
- 自主运行时 `passthrough_mode` 必须为 `false`
- AI 节点挂了，车应因超时而停；安全节点不在时，不应有别的节点接着发 `/cmd_vel`
