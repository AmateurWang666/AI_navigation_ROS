cmake_minimum_required(VERSION 3.0.2)
project(tjark_agv)

find_package(catkin REQUIRED)

catkin_package()

# worlds/ 与 models/ 必须一起安装：cafe.world 通过 model:// 引用 models/ 下的
# 场景模型，缺了它 Gazebo 会转去联网找模型，在 WSL 上表现为启动卡住数分钟。
install(DIRECTORY launch urdf meshes worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
