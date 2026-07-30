"""ai_robot_nav：激光雷达确定性策略 + 本地视觉大模型的 ROS 2 导航包。

对外只有两个可执行节点（见 setup.py 的 console_scripts）：

- ``ai_nav_node``：激光确定性决策 + 后台视觉推理，输出建议速度到 /ai_cmd_vel。
- ``safety_node``：看门狗与急停，是 /cmd_vel 的唯一发布者。

其余模块（navigator / llm_client / scan_utils / motion / lifecycle）都不依赖
rclpy，可以脱离 ROS 图直接单元测试。
"""
