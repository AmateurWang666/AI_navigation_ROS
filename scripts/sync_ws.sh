#!/usr/bin/env bash
#
# 把仓库 src/ 下的两个包同步到 WSL 的 catkin 工作空间并构建。
# 代码在 Windows 编辑、在 WSL 构建，所以要复制而不是直接在 /mnt/c 上 catkin_make：
# /mnt/c 的文件系统性能很差，且行尾需要统一成 LF。
#
#   bash scripts/sync_ws.sh              # 只同步
#   bash scripts/sync_ws.sh --build      # 同步 + catkin_make
#   bash scripts/sync_ws.sh --build --test
#
# 环境变量 ROS1_WS 指定工作空间，默认 ~/ros_ws。

set -o pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROS1_WS:-$HOME/ros_ws}"
PACKAGES="ai_robot_nav tjark_agv"

DO_BUILD=0
DO_TEST=0
for arg in "$@"; do
    case "$arg" in
        --build) DO_BUILD=1 ;;
        --test)  DO_BUILD=1; DO_TEST=1 ;;
        *) echo "unknown option: $arg"; exit 2 ;;
    esac
done

mkdir -p "$WS/src"
for pkg in $PACKAGES; do
    src="$REPO_DIR/src/$pkg"
    dst="$WS/src/$pkg"
    if [ ! -f "$src/package.xml" ]; then
        echo "package.xml not found at $src"
        exit 1
    fi
    echo "=== sync $pkg -> $dst ==="
    if command -v rsync >/dev/null 2>&1; then
        rsync -a --delete --exclude __pycache__ --exclude .pytest_cache "$src/" "$dst/"
    else
        rm -rf "$dst"
        cp -r "$src/." "$dst/"
        find "$dst" -name __pycache__ -type d -prune -exec rm -rf {} +
    fi
    find "$dst" -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.xml' \
        -o -name '*.launch' -o -name '*.xacro' -o -name '*.urdf' \) \
        -exec sed -i 's/\r$//' {} +
done

if [ "$DO_BUILD" -eq 0 ]; then
    echo "synced. Run with --build, or just: bash scripts/sim.sh"
    exit 0
fi

# shellcheck source=scripts/ros_env.sh
source "$REPO_DIR/scripts/ros_env.sh"
cd "$WS" || exit 1

echo
echo "=== catkin_make (ROS $ROS_DISTRO) ==="
catkin_make -DCATKIN_WHITELIST_PACKAGES="${PACKAGES// /;}" -DCMAKE_BUILD_TYPE=Release || exit 1

if [ "$DO_TEST" -eq 1 ]; then
    echo
    bash "$REPO_DIR/scripts/run_tests.sh"
fi

echo
echo "done. In a new shell run:  source $WS/devel/setup.bash"
