# ROS 仿真与本地大模型落地任务清单

## 阶段一：基础环境自动化配置 (WSL & Ollama)
- [x] 检查 WSL Ubuntu 系统版本 (已确认为 Ubuntu 24.04)
- [ ] 自动化安装 ROS (根据 Ubuntu 版本选择 Noetic 或 Humble)
- [x] 在 WSL 中安装并启动 Ollama 本地服务
- [ ] 下载轻量级多模态模型 (`llava:7b`)

## 阶段二：ROS 工作空间与基础节点搭建
- [x] 创建 ROS 工作空间 (`~/ros_ws`)
- [x] 编写 ROS-LLM 通信节点 (Python)，通过 HTTP 调用 Windows 上的 Ollama
- [x] 编写 Prompt 模板节点，订阅传感器话题并拼接环境描述文本

## 阶段三：安全层与运动控制节点
- [x] 编写 安全守卫节点 (订阅 LiDAR 话题 `/scan`，发布 `/cmd_vel`)
- [x] 编写 运动解析节点 (将 LLM 的 JSON 转化为 `Twist` 消息)

## 阶段四：Gazebo 仿真测试
- [x] 下载开源机器人模型包 (如 TurtleBot3)
- [x] 在 Gazebo 中加载包含墙壁或障碍物的虚拟世界
- [x] 启动全套 ROS 节点进行端到端闭环测试 (正在进行中)
- [ ] 录制演示视频/生成运行总结报告
