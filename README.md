# ai_robot_nav

ROS 1 激光确定性导航 + Ollama 视觉辅助，仿真平台 **tjark_agv**。

## 日常仿真（一条命令）

```bash
cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace
bash scripts/sim.sh
```

纯激光 / 停止 / 带 GUI：`bash scripts/sim.sh --lidar` · `bash scripts/sim.sh stop` · `bash scripts/sim.sh --gui`

完整说明见 **[docs/quick_start.md](docs/quick_start.md)**。

## 改代码后同步

```bash
bash scripts/sync_nav.sh --build    # 仅 ai_robot_nav，最快
bash scripts/setup_ros1_ws.sh --build   # 含 tjark_agv 全量
```

## 测试

```bash
bash scripts/run_tests.sh
```

## 架构

```
/scan ──► ai_nav_node ──► /ai_cmd_vel ──► safety_node ──► /cmd_vel ──► tjark_agv
              └── Ollama (可选)
```
