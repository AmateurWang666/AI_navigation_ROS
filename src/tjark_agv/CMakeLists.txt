cmake_minimum_required(VERSION 3.0.2)
project(tjark_agv)

find_package(catkin REQUIRED)

# 行人碰撞插件只给 cafe 仿真用。实车没有 Gazebo 时跳过，URDF 描述仍可编译。
find_package(gazebo QUIET)
if(gazebo_FOUND)
  set(_tjark_gazebo_default ON)
else()
  set(_tjark_gazebo_default OFF)
endif()
option(TJARK_BUILD_GAZEBO_PLUGINS
  "Build the Gazebo actor collision plugin" ${_tjark_gazebo_default})

catkin_package()

if(TJARK_BUILD_GAZEBO_PLUGINS)
  if(NOT gazebo_FOUND)
    message(FATAL_ERROR
      "TJARK_BUILD_GAZEBO_PLUGINS=ON but Gazebo was not found. "
      "Install Gazebo Classic, or reconfigure with -DTJARK_BUILD_GAZEBO_PLUGINS=OFF.")
  endif()
  message(STATUS "tjark_agv: building actor collision plugin")

  add_library(tjark_actor_collisions SHARED plugins/ActorCollisionsPlugin.cc)
  target_include_directories(tjark_actor_collisions PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins
  )
  target_link_libraries(tjark_actor_collisions ${GAZEBO_LIBRARIES})

  # 库放在 lib/ 而不是 lib/tjark_agv/。devel 空间的默认输出和
  # package.xml / cafe_world.launch 里的 GAZEBO_PLUGIN_PATH（.../lib）都指这里；
  # 装到包私有目录的话，catkin_make install 之后 Gazebo 会静默丢掉插件。
  install(TARGETS tjark_actor_collisions
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
else()
  message(STATUS "tjark_agv: skipping actor collision plugin (Gazebo disabled)")
endif()

# worlds/ 与 models/ 必须一起安装：cafe.world 通过 model:// 引用 models/ 下的
# 场景模型，缺了它 Gazebo 会转去联网找模型，在 WSL 上表现为启动卡住数分钟。
install(DIRECTORY launch urdf meshes worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
