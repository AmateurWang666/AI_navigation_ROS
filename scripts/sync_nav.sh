#!/usr/bin/env bash
#
# 快速同步 ai_robot_nav 到 ~/ros_ws 并增量构建（日常改代码后用，比全量 sync 快）。
#
#   bash scripts/sync_nav.sh           # 只同步
#   bash scripts/sync_nav.sh --build   # 同步 + catkin_make（仅 ai_robot_nav）

set -o pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAV_SRC="$REPO_DIR/src/ai_robot_nav"
WS="${ROS1_WS:-$HOME/ros_ws}"
NAV_DST="$WS/src/ai_robot_nav"
DO_BUILD=0

for arg in "$@"; do
    case "$arg" in
        --build) DO_BUILD=1 ;;
        *) echo "unknown option: $arg"; exit 2 ;;
    esac
done

mkdir -p "$WS/src"
echo "=== sync ai_robot_nav -> $NAV_DST ==="
if command -v rsync >/dev/null 2>&1; then
    rsync -a --delete \
        --exclude __pycache__ --exclude .pytest_cache \
        "$NAV_SRC/" "$NAV_DST/"
else
    rm -rf "$NAV_DST"
    cp -r "$NAV_SRC/." "$NAV_DST/"
    find "$NAV_DST" -name __pycache__ -type d -prune -exec rm -rf {} +
fi
find "$NAV_DST" -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.launch' \) \
    -exec sed -i 's/\r$//' {} +

if [ "$DO_BUILD" -eq 0 ]; then
    echo "synced. Run with --build or: bash scripts/sim.sh"
    exit 0
fi

# shellcheck source=scripts/ros_env.sh
source "$REPO_DIR/scripts/ros_env.sh"
cd "$WS" || exit 1
echo "=== catkin_make ai_robot_nav only ==="
catkin_make -DCATKIN_WHITELIST_PACKAGES=ai_robot_nav -DCMAKE_BUILD_TYPE=Release
echo "done."
