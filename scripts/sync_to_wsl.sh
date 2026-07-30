#!/usr/bin/env bash
#
# Copy the package from this repo into the WSL colcon workspace and optionally
# build and test it. Sources edited on Windows are not visible to colcon until
# they land inside the Linux filesystem, and building directly off /mnt/c is
# slow, so the workspace keeps its own copy.
#
# Run from Windows PowerShell at the repo root:
#     wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh
#     wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh --build
#     wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh --build --test
#
# Override the destination workspace with ROS2_WS=/path/to/ws.

# ROS setup.bash reads unset variables, so -u must stay off.
set -o pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO_DIR/src/ai_robot_nav"
WS="${ROS2_WS:-$HOME/ros2_ws}"
DST="$WS/src/ai_robot_nav"

DO_BUILD=0
DO_TEST=0
for arg in "$@"; do
    case "$arg" in
        --build) DO_BUILD=1 ;;
        --test)  DO_BUILD=1; DO_TEST=1 ;;
        *) echo "unknown option: $arg"; exit 2 ;;
    esac
done

if [ ! -d "$SRC" ]; then
    echo "package not found at $SRC"
    exit 1
fi

echo "=== syncing $SRC -> $DST ==="
rm -rf "$DST"
mkdir -p "$DST"
cp -r "$SRC/." "$DST/"

find "$DST" -name __pycache__ -type d -prune -exec rm -rf {} +
rm -rf "$DST/.pytest_cache"

# Files authored on Windows may carry CRLF, which breaks shebangs and YAML.
find "$DST" -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.xml' -o -name '*.cfg' \) \
    -exec sed -i 's/\r$//' {} +
chmod -R u+rwX "$DST"

echo "synced $(find "$DST" -type f | wc -l) files"

if [ "$DO_BUILD" -eq 0 ]; then
    exit 0
fi

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "no ROS 2 installation under /opt/ros"
    exit 1
fi

cd "$WS" || exit 1

echo
echo "=== colcon build (ROS $ROS_DISTRO) ==="
# Drop stale artefacts so deleted modules cannot linger in the install space.
rm -rf build/ai_robot_nav install/ai_robot_nav
colcon build --packages-select ai_robot_nav --symlink-install || exit 1

if [ "$DO_TEST" -eq 1 ]; then
    echo
    echo "=== colcon test ==="
    colcon test --packages-select ai_robot_nav || true
    colcon test-result --all
fi

echo
echo "done. In a new shell run:  source $WS/install/setup.bash"
