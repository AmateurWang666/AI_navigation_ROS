cmake_minimum_required(VERSION 3.0.2)
project(ai_robot_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  std_msgs
  std_srvs
  nav_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ai_nav_node
  scripts/safety_node
  scripts/nav_mux_node
  scripts/map_manager_node
  scripts/mapper_node
  scripts/send_goal
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.yaml" PATTERN "*.config"
)

# 地图单独装：栅格图片是二进制，不能并进上面那条按扩展名过滤的规则里。
install(DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
