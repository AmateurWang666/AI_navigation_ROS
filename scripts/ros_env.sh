#!/usr/bin/env bash
# 共享 ROS 环境变量（被 sim.sh / sync_nav.sh 引用，也可手动 source）。
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS1_WS="${ROS1_WS:-$HOME/ros_ws}"
NAV_PARAMS="${NAV_PARAMS:-$ROS1_WS/src/ai_robot_nav/config/tjark_agv_params.yaml}"

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "ROS Noetic not found. Run: bash $REPO_DIR/scripts/install_ros_noetic.sh"
    return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [ -f "$ROS1_WS/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROS1_WS/devel/setup.bash"
fi

export REPO_DIR ROS1_WS NAV_PARAMS
