#!/usr/bin/env bash
# 共享 ROS 环境变量（被 sim.sh / sync_ws.sh 引用，也可手动 source）。
# shellcheck source=scripts/wsl_paths.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/wsl_paths.sh"

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "ROS Noetic not found. Run: bash $REPO_DIR/scripts/install_ros_noetic.sh"
    return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

# 导航栈（move_base / amcl / map_server）是源码编译的独立工作空间，必须在
# ROS1_WS 之前 source：catkin 的 overlay 是有序的，后 source 的叠在前面之上。
# 顺序反了，ROS1_WS 构建时就看不到 move_base_msgs 等包，编译会报找不到依赖。
# 本机 Ubuntu 24.04 没有 Noetic 官方二进制，故导航栈只能这样提供，
# 详见 scripts/install_nav_stack.sh。
if [ -f "$NAV_WS/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$NAV_WS/devel/setup.bash"
fi

if [ -f "$ROS1_WS/devel/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROS1_WS/devel/setup.bash"
fi

# 构建工作空间若在导航栈安装之前就编译过，其 setup.bash 可能不链到 _nav_ws，
# 后 source 反而会把 move_base 等包从搜索路径里盖掉。手动补回 overlay。
if [ -d "$NAV_WS/devel" ]; then
    case ":$CMAKE_PREFIX_PATH:" in
        *":$NAV_WS/devel:"*) ;;
        *) export CMAKE_PREFIX_PATH="$NAV_WS/devel${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}" ;;
    esac
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

export REPO_DIR ROS1_WS NAV_WS NAV_PARAMS PROJECT_NAME
