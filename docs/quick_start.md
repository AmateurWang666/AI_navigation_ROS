# 仿真快速启动（ROS 1 Noetic + tjark_agv）

## 一条命令跑全程（推荐）

```bash
cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace
bash scripts/sim.sh
```

**正常现象：**

- **没有 Gazebo 窗口** — 默认无 GUI（headless），仿真在后台跑
- 启动约 **3 秒** 后出现 `FORWARD lin=0.15 ang=0.00 | clear ahead (20.00m)` — **表示机器人在前进**
- 开头几行 `LiDAR silent` 是等 Gazebo 加载，可忽略
- 已默认静音 Gazebo 相机 DEBUG；需要完整日志时加 `--verbose`

**验证机器人在动（另开终端）：**

```bash
source /opt/ros/noetic/setup.bash && source ~/ros_ws/devel/setup.bash
rostopic echo /odom/pose/pose/position/x
```

数值持续增大说明在前进。停止：`Ctrl+C` 或 `bash scripts/sim.sh stop`。

---

## 改代码后（Windows 编辑 → WSL 测试）

**PowerShell（只改了 ai_robot_nav 时，最快）：**

```powershell
wsl -d Ubuntu bash -c "cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace && bash scripts/sync_nav.sh --build"
```

然后在 WSL 里重新 `bash scripts/sim.sh`。

首次部署或改了 tjark_agv 外部包时，才需要全量：

```powershell
wsl -d Ubuntu bash ./scripts/setup_ros1_ws.sh --build
```

---

## 两终端 + Gazebo 画面（调试推荐）

需要**看到机器人在 Gazebo 里动**时，开两个 WSL 终端。终端 1 先启动仿真并等 Gazebo 窗口弹出，再在终端 2 启动导航。

**终端 1 — 仿真（带画面）：**

```bash
cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash
bash scripts/sim.sh --gui gazebo
```

等价于手动启动（空场景，启动较快）：

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash
roslaunch ai_robot_nav tjark_gazebo.launch headless:=false gui:=true
```

若要用 tjark_agv 自带的 **cafe 场景**（更真实，但加载慢，约 1–2 分钟）：

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash
roslaunch tjark_agv tjark_agv.launch
```

**终端 2 — AI 导航（等 `/scan` 出现或 Gazebo 窗口就绪后再开）：**

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash
roslaunch ai_robot_nav ai_nav.launch use_sim_time:=true \
  params_file:=$(rospack find ai_robot_nav)/config/tjark_agv_params.yaml
```

或用脚本（会自动等待 `/scan`）：

```bash
cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace
bash scripts/sim.sh nav
```

纯激光（不调用 Ollama）：终端 2 加 `--lidar`，或 `bash scripts/sim.sh nav --lidar`。

**成功标志：** 终端 2 日志出现 `FORWARD lin=0.15 ang=0.00 | clear ahead (...)`，Gazebo 里小车开始前进。

**停止：** 两个终端分别 `Ctrl+C`，或任意终端执行 `bash scripts/sim.sh stop`。

> WSL2 需已启用图形（Windows 11 自带 WSLg，或配置 X11）。窗口黑屏时可试：  
> `export QT_QPA_PLATFORM=xcb` 与 `export LIBGL_ALWAYS_SOFTWARE=1`（`sim.sh --gui` 已自动设置）。

---

## 两终端模式（无画面 / 脚本快捷方式）

终端 1：

```bash
bash scripts/sim.sh gazebo
```

终端 2（等 `/scan` 出现后再开，或脚本自动等待）：

```bash
bash scripts/sim.sh nav
```

纯激光：`bash scripts/sim.sh nav --lidar`

---

## 单元测试（不需要 Gazebo）

```powershell
wsl -d Ubuntu bash ./scripts/run_tests.sh
```

---

## 首次环境安装（只需一次）

```bash
bash scripts/install_ros_noetic.sh
```

---

## 常见问题

| 现象 | 处理 |
|------|------|
| 只左右转不走 | 确认已 `sync_nav.sh --build`，参数含 `front_center_angle: 180` |
| 启动慢 | 不要用 `tjark_agv.launch`（cafe 场景），用 `scripts/sim.sh` |
| Ollama 断线 | 加 `--lidar` 或自动降级纯激光 |

## 可选：写入 ~/.bashrc 的别名

```bash
echo 'alias sim="bash /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace/scripts/sim.sh"' >> ~/.bashrc
echo 'alias sim-sync="bash /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace/scripts/sync_nav.sh --build"' >> ~/.bashrc
```

之后任意目录执行 `sim` 即可。
