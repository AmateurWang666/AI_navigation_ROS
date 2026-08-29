#!/usr/bin/env bash
# 共享 ROS 环境变量（被 sim.sh / sync_ws.sh 引用，也可手动 source）。
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS1_WS="${ROS1_WS:-$HOME/ros_ws}"
NAV_PARAMS="${NAV_PARAMS:-$ROS1_WS/src/ai_robot_nav/config/nav_params.yaml}"

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

# cafe 场景引用的模型都在仓库内。工作空间副本放前面：它在 WSL 本地盘上，
# 比 /mnt/c 快得多；仓库副本兜底，让「还没 sync 就直接跑」也能用。
for models in "$ROS1_WS/src/tjark_agv/models" "$REPO_DIR/src/tjark_agv/models"; do
    if [ -d "$models" ]; then
        export GAZEBO_MODEL_PATH="$models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
    fi
done

# 关掉在线模型库。所需模型已全部内置，留着这个 URI 只会让 gzserver 在启动时
# 反复尝试联网拉取，WSL 上因此要卡好几分钟才出 /scan。
export GAZEBO_MODEL_DATABASE_URI=""

export REPO_DIR ROS1_WS NAV_PARAMS
