#!/usr/bin/env bash
# WSL 侧 catkin 工作空间路径，与 Windows 仓库目录名对齐。
#
#   源码（Windows 编辑）  /mnt/c/.../ROS_AI_Robot_Workspace
#   构建（WSL 本地盘）    ~/ROS_AI_Robot_Workspace_ws
#   导航栈 overlay        ~/ROS_AI_Robot_Workspace_nav_ws
#
# 目录名随仓库文件夹自动推导：重命名 ROS_AI_Robot_Workspace 后，WSL 路径会跟着变。
# 可用环境变量 ROS1_WS / NAV_WS 覆盖。

_wsl_paths_init() {
    local scripts_di
    scripts_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    REPO_DIR="$(cd "$scripts_dir/.." && pwd)"
    PROJECT_NAME="$(basename "$REPO_DIR")"
    ROS1_WS="${ROS1_WS:-$HOME/${PROJECT_NAME}_ws}"
    NAV_WS="${NAV_WS:-$HOME/${PROJECT_NAME}_nav_ws}"
    if [ -f "$ROS1_WS/src/ai_robot_nav/config/nav_params.yaml" ]; then
        NAV_PARAMS="${NAV_PARAMS:-$ROS1_WS/src/ai_robot_nav/config/nav_params.yaml}"
    else
        NAV_PARAMS="${NAV_PARAMS:-$REPO_DIR/src/ai_robot_nav/config/nav_params.yaml}"
    fi
    export REPO_DIR PROJECT_NAME ROS1_WS NAV_WS NAV_PARAMS
}

_wsl_paths_init
