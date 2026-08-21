"""ai_robot_nav：激光雷达确定性策略 + 本地视觉大模型的 ROS 1 导航包。

核心模块（navigator、llm_client、scan_utils、motion）不依赖 rospy，
可以脱离 ROS 图直接单元测试。
"""
