# GitHub 仓库 About 文案

复制以下内容到 GitHub 仓库 **Settings → General → About**，或仓库首页右侧 **About 编辑** 面板。

---

## Description（简短描述，≤ 350 字符）

```
ROS 1 Noetic 自主导航栈：激光雷达确定性规划 + Ollama/LLaVA 视觉安全层，适配 tjark_agv 差速小车与 Gazebo Classic 仿真。WSL 一键脚本，支持纯激光降级与双终端 GUI 调试。
```

**英文版（可选）：**

```
ROS 1 Noetic nav stack for tjark_agv: deterministic LiDAR planning with optional Ollama/LLaVA vision safety. Gazebo Classic sim, WSL-friendly scripts, lidar-only fallback.
```

---

## Website（可选）

留空，或填文档入口：

```
https://github.com/AmateurWang666/AI_navigation_ROS#readme
```

---

## Topics（建议标签）

```
ros
ros-noetic
robotics
mobile-robot
autonomous-navigation
lidar
gazebo
gazebo-classic
python
rospy
ollama
llava
computer-vision
tjark-agv
wsl
safety
```

---

## 扩展 About（README 顶部 / 个人简介用）

**中文（约 120 字）：**

> 为 tjark_agv 实车/仿真打造的 ROS 1 导航方案。激光雷达负责选路与速度，本地 Ollama 视觉模型只在雷达盲区做保守修正（减速/停车），独立 safety 节点兜底急停。含 Gazebo 仿真 launch、WSL 同步脚本与 47 项 pytest，支持无 GUI 快速迭代与双终端可视化调试。

**English (~80 words):**

> ROS 1 Noetic navigation for the tjark_agv differential-drive platform. LiDAR drives deterministic motion planning; a local Ollama LLaVA model adds conservative vision checks (slow/stop only, never route selection). A separate safety node enforces emergency stops. Includes Gazebo Classic launches, WSL sync/build scripts, pytest suite, headless and GUI two-terminal workflows.

---

## 一句话 Elevator Pitch

| 语言 | 文案 |
|------|------|
| 中文 | 激光选路、视觉刹车——面向 tjark_agv 的可测试 ROS 1 自主导航。 |
| EN | LiDAR plans, vision brakes — testable ROS 1 autonomy for tjark_agv. |

---

## 与旧版 About 的差异说明

| 旧（ROS 2 / TurtleBot3） | 新（当前主分支） |
|--------------------------|------------------|
| ROS 2 Jazzy | ROS 1 Noetic |
| TurtleBot3 + Gazebo Sim | tjark_agv + Gazebo Classic |
| TwistStamped | Twist |
| `/camera/image_raw` | `/my_camera/color/image_raw` |

更新 About 时请删除仍指向 TurtleBot3 / ROS 2 的旧描述，避免访客误解。
