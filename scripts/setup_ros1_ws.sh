#!/usr/bin/env bash
#
# 将 ai_robot_nav 与 tjark_agv 部署到 ROS 1 catkin 工作空间。
# tjark_agv 以外部只读方式复制，不修改 C:\Users\ROG\Desktop\tjark_agv-master 中的源码。
#
# 用法（仓库根目录）：
#   bash ./scripts/setup_ros1_ws.sh              # 只同步
#   bash ./scripts/setup_ros1_ws.sh --build      # 同步 + catkin_make
#   bash ./scripts/setup_ros1_ws.sh --build --test
#
# 环境变量：
#   ROS1_WS       catkin 工作空间（默认 ~/ros_ws）
#   TJARK_AGV_SRC tjark_agv 包根目录（含 package.xml 的目录）

set -o pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAV_SRC="$REPO_DIR/src/ai_robot_nav"
WS="${ROS1_WS:-$HOME/ros_ws}"
TJARK_SRC="${TJARK_AGV_SRC:-$REPO_DIR/../tjark_agv-master/tjark_agv-master}"
NAV_DST="$WS/src/ai_robot_nav"
TJARK_DST="$WS/src/tjark_agv"

DO_BUILD=0
DO_TEST=0
for arg in "$@"; do
    case "$arg" in
        --build) DO_BUILD=1 ;;
        --test)  DO_BUILD=1; DO_TEST=1 ;;
        *) echo "unknown option: $arg"; exit 2 ;;
    esac
done

if [ ! -d "$NAV_SRC" ]; then
    echo "ai_robot_nav not found at $NAV_SRC"
    exit 1
fi

if [ ! -f "$TJARK_SRC/package.xml" ]; then
    echo "tjark_agv package.xml not found at $TJARK_SRC"
    echo "set TJARK_AGV_SRC to the directory containing tjark_agv package.xml"
    exit 1
fi

echo "=== syncing ai_robot_nav -> $NAV_DST ==="
rm -rf "$NAV_DST"
mkdir -p "$WS/src"
cp -r "$NAV_SRC/." "$NAV_DST/"

echo "=== copying tjark_agv (read-only) -> $TJARK_DST ==="
rm -rf "$TJARK_DST"
cp -r "$TJARK_SRC/." "$TJARK_DST/"

find "$NAV_DST" -name __pycache__ -type d -prune -exec rm -rf {} +
rm -rf "$NAV_DST/.pytest_cache"
find "$NAV_DST" -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.xml' -o -name '*.launch' \) \
    -exec sed -i 's/\r$//' {} +
chmod -R u+rwX "$NAV_DST" "$TJARK_DST"

echo "synced ai_robot_nav ($(find "$NAV_DST" -type f | wc -l) files)"
echo "copied tjark_agv from $TJARK_SRC"

if [ "$DO_BUILD" -eq 0 ]; then
    exit 0
fi

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "ROS Noetic not found under /opt/ros/noetic"
    echo "On Ubuntu 24.04 WSL, run first:"
    echo "  bash $REPO_DIR/scripts/install_ros_noetic.sh"
    exit 1
fi

source /opt/ros/noetic/setup.bash
cd "$WS" || exit 1

echo
echo "=== catkin_make (ROS $ROS_DISTRO) ==="
catkin_make -DCATKIN_WHITELIST_PACKAGES="ai_robot_nav;tjark_agv" || exit 1

if [ "$DO_TEST" -eq 1 ]; then
    echo
    bash "$REPO_DIR/scripts/run_tests.sh"
fi

echo
echo "done. In a new shell run:  source $WS/devel/setup.bash"
