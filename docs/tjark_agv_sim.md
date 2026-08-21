# tjark_agv 仿真环境说明

本文档描述如何将 **tjark_agv**（实车 URDF / Gazebo 模型）作为本项目的仿真平台。
**tjark_agv 源码位于外部目录，本仓库不会修改其任何文件**（含车辆尺寸、传感器视场等参数）。

## 来源与 ROS 版本

| 项目 | 路径 | ROS 版本 | 构建系统 |
|------|------|----------|----------|
| tjark_agv（仿真模型） | `C:\Users\ROG\Desktop\tjark_agv-master\tjark_agv-master` | **ROS 1** | catkin |
| ai_robot_nav（导航栈） | 本仓库 `src/ai_robot_nav` | **ROS 1**（步骤二迁移后） | catkin |

tjark_agv 的 `package.xml` 使用 `catkin`，launch 为 `.launch`（XML），依赖 `gazebo_ros`、`rostopic` 等 ROS 1 组件，**实车同样基于 ROS 1**。

## 车辆与话题

| 项目 | 值 |
|------|-----|
| 差速轮距 | 0.34 m |
| 轮径 | 0.12 m |
| 激光话题 | `/scan` |
| 相机话题 | `/my_camera/color/image_raw` |
| 速度话题 | `/cmd_vel`（`geometry_msgs/Twist`） |
| 里程计 | `/odom` |

激光在 URDF 中视场约为传感器坐标系 90°–270°；导航代码通过 `angle_min` / `angle_increment` 动态映射扇区，不硬编码 TurtleBot3 几何。

## 一次性部署

在 WSL（Ubuntu + ROS Noetic）中，从本仓库根目录执行：

```bash
bash ./scripts/setup_ros1_ws.sh
```

脚本会：

1. 将 `ai_robot_nav` 同步到 `~/ros_ws/src/`
2. **只读复制** `tjark_agv-master` 到 `~/ros_ws/src/tjark_agv`（不修改源目录）
3. 可选 `--build` 执行 `catkin_make`

环境变量：

- `TJARK_AGV_SRC`：tjark_agv 源码路径（默认 `../tjark_agv-master/tjark_agv-master` 相对仓库）
- `ROS1_WS`：catkin 工作空间（默认 `~/ros_ws`）

## 两终端快速启动

### 终端 1：Gazebo + tjark_agv

**WSL2 推荐（无 GUI，启动快）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash
roslaunch ai_robot_nav tjark_gazebo.launch headless:=true
```

原版 `tjark_agv.launch` 使用 cafe 大场景，WSL 上 gzserver 可能要等 **3–5 分钟**，gzclient 窗口还常卡死；不推荐。

确认传感器（另开终端）：

```bash
source /opt/ros/noetic/setup.bash
rostopic list | grep -E 'scan|my_camera|cmd_vel|clock'
```

### 终端 2：AI 导航

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash
roslaunch ai_robot_nav ai_nav.launch use_sim_time:=true \
  params_file:=$(rospack find ai_robot_nav)/config/tjark_agv_params.yaml
```

纯激光测试（不调用 Ollama）：

```bash
roslaunch ai_robot_nav ai_nav.launch use_sim_time:=true use_image:=false \
  params_file:=$(rospack find ai_robot_nav)/config/tjark_agv_params.yaml
```

## 与旧 TurtleBot3 仿真的差异

| 项目 | TurtleBot3（旧） | tjark_agv（新） |
|------|------------------|-----------------|
| ROS | ROS 2 Jazzy | ROS 1 Noetic |
| Gazebo | Gazebo Sim 8 | Gazebo Classic 11 |
| 相机话题 | `/camera/image_raw` | `/my_camera/color/image_raw` |
| cmd_vel 类型 | TwistStamped | Twist |
| 参数文件 | `ai_nav_params.yaml` | `tjark_agv_params.yaml` |

## 注意事项

- 必须先启动 Gazebo，再启动导航节点；无 `/scan` 时安全节点会持续停车（设计行为）。
- `use_sim_time:=true` 在 Gazebo 慢于实时时必须开启。
- 速度上限在 `tjark_agv_params.yaml` 中按较小底盘做了保守初值，实车部署前需重标定。
