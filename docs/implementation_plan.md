# 智能自动驾驶机器人：从仿真到实车的完整技术蓝图 (终版)

## 核心架构（已实现）

```
/scan --------------------------> safety_node --> /cmd_vel --> 电机
/camera/image_raw --+                  ^
/scan --------------+--> ai_nav_node --+--> /ai_cmd_vel（10Hz 心跳）
                         |  |
                         |  +--> navigator（确定性策略，决定速度）
                         +-----> Ollama (llava:7b)（只给语义提示）
```

| 文件 | 节点 | 输入 | 输出 |
|------|------|------|------|
| `ai_robot_nav/ai_nav_node.py` | `ai_nav_node` | `/scan`、`/camera/image_raw` | `/ai_cmd_vel` |
| `ai_robot_nav/navigator.py` | - | 前/左/右扇区距离 + 可选语义提示 | 动作与速度（确定性） |
| `ai_robot_nav/safety_node.py` | `safety_node` | `/scan`、`/ai_cmd_vel` | `/cmd_vel`（唯一发布者，20Hz） |
| `ai_robot_nav/scan_utils.py` | - | LaserScan | 与雷达型号无关的扇区距离 |
| `ai_robot_nav/llm_client.py` | - | 模型原始输出 | 校验后的 hazard + 方向偏好 |
| `ai_robot_nav/motion.py` | - | Twist | 限幅后的 Twist |
| `config/ai_nav_params.yaml` | - | - | 两个节点的全部参数 |
| `launch/ai_nav.launch.py` | - | - | 启动双节点并加载参数 |
| `scripts/sync_to_wsl.sh` | - | - | Windows 仓库 -> WSL 工作空间同步/构建/测试 |

> 关键约束：模型的输出只能让机器人更保守。速度全部由 `navigator.py` 从激光几何算出，
> 模型无法提高任何速度，也无法把机器人导向激光判定为封闭的方向。

> 原 v1 脚本（`lidar_driver.py`、`qwen_client.py`、`motion_controller.py`、`main.py`、`serial_comm.py`、`firmware/esp32_motor_driver/` 等）已废弃，逻辑内嵌于上述 ROS2 节点。

## 算力策略
- 仿真阶段：WSL2 本地 Ollama + `llava:7b`
- 实车阶段：Jetson 采集控制，可选笔记本局域网作云脑

## 算力策略补充
`llava:7b` 实测中位数 ~0.9s/次，带图与纯文本无可测差异。因为速度已由确定性策略产出，
模型变慢只会降低语义提示的刷新率，不会拖慢控制回路——这是把数值决策移出模型的附带收益。

## 下一步（对应 task.md 未完成项）
1. Gazebo + TurtleBot3 闭环回归（合成传感器已过，尚未在仿真世界里跑）
2. 验证 `use_image:=false` 的纯激光通路与 `lidar_only_fallback` 降级路径
3. 实车话题名与 QoS 验证，速度上限按底盘重标定

## Verification Plan
1. 单元测试：扇区提取、确定性策略、模型输出解析（`colcon test`，无需 ROS 图与 Ollama）
2. 看门狗测试：杀掉 `ai_nav_node` 后确认机器人在 `ai_cmd_timeout` 内停住
3. 解耦测试：`ros2 topic hz /ai_cmd_vel` 应稳定在 `command_publish_rate`；
   停掉 Ollama 或让请求超时也不能中断纯激光控制
4. Gazebo 停车区测试：`stop_distance` 内阻断前进，后退与转向仍可用
5. 降级测试：停掉 Ollama，确认机器人切换为纯激光导航而非停车
6. 闭环自动驾驶观察