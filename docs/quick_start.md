# 两终端快速启动指南

本文档用于每次重新打开 WSL 终端后启动 TurtleBot3 Gazebo 仿真和 AI 导航。
正常运行只需要两个终端：终端 1 运行 Gazebo，终端 2 运行导航节点。

## 启动前

确认 Windows 中已经启动 WSL，并且代码已构建到：

```text
~/ros2_ws
```

如果刚修改或更新过 Windows 项目代码，先在项目根目录的 **Windows PowerShell** 执行一次：

```powershell
wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh --test
```

没有修改代码时不需要重复构建。

## 终端 1：启动 Gazebo

打开第一个 Ubuntu/WSL 终端，完整执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle
export QT_QPA_PLATFORM=xcb
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

保持这个终端运行，不要关闭。

等待 Gazebo 界面和机器人出现后，可以新开一个临时终端，用下面的命令确认传感器已建立：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep -E 'scan|camera/image_raw|clock'
```

正常应包含：

```text
/camera/image_raw
/clock
/scan
```

## 终端 2：启动 AI 导航

打开第二个 Ubuntu/WSL 终端，完整执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ai_robot_nav ai_nav.launch.py use_sim_time:=true
```

`use_sim_time:=true` 不可省略。WSL2 中 Gazebo 通常低于真实时间速度；
若使用墙上时钟，安全节点会把正常但较慢的雷达误判为静默。

启动最初出现以下告警是正常的，因为节点正在等待 `/clock` 和第一帧传感器数据：

```text
LiDAR silent; holding stop.
LiDAR data missing or stale; commanding stop.
Camera frame missing or stale; running on LiDAR only.
```

如果 `LiDAR silent` 在数秒后仍持续出现，请先确认终端 1 的 Gazebo 仍在运行，
然后执行：

```bash
ros2 topic list -t | grep -E 'scan|cmd_vel'
```

当前 Jazzy 仿真的正确类型应为：

```text
/ai_cmd_vel [geometry_msgs/msg/Twist]
/cmd_vel [geometry_msgs/msg/TwistStamped]
/scan [sensor_msgs/msg/LaserScan]
```

## Ollama 检查

Ollama 已配置为 systemd 服务，通常会随 WSL 自动启动，不需要第三个长期运行的终端。
在终端 2 启动导航前，可选执行：

```bash
systemctl is-active ollama
curl -sS --max-time 5 http://localhost:11434/api/tags
```

若服务未运行：

```bash
sudo systemctl start ollama
```

即使 Ollama 暂时断线或请求超时，机器人也会自动切换为纯激光导航。

如果只想测试激光导航、完全不调用模型：

```bash
ros2 launch ai_robot_nav ai_nav.launch.py \
  use_sim_time:=true \
  use_image:=false
```

## 停止顺序

1. 在终端 2 按 `Ctrl+C`，先停止 AI 导航；节点会在退出前发送零速。
2. 确认两个节点均显示 `process has finished cleanly`。
3. 回到终端 1 按 `Ctrl+C`，再关闭 Gazebo。

不要直接关闭终端 2 的窗口，以免跳过正常的停车和清理流程。

