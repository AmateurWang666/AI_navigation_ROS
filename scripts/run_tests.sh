#!/usr/bin/env bash
#
# 在本机 WSL 运行 ai_robot_nav 单元测试（不需要 ROS 图或 Ollama）。
#
# 依赖：已安装的 ROS（用于 sensor_msgs Python 绑定）+ pytest。
# Ubuntu 24.04 无 Noetic 时，会自动回退到已安装的 ROS 2 Jazzy。
#
# 用法（仓库根目录）：
#   bash ./scripts/run_tests.sh
#
# Windows PowerShell：
#   wsl -d Ubuntu bash ./scripts/run_tests.sh

set -o pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG_DIR="$REPO_DIR/src/ai_robot_nav"

if [ ! -d "$PKG_DIR/test" ]; then
    echo "test directory not found at $PKG_DIR/test"
    exit 1
fi

# 按优先级选择 ROS 环境（测试只需 sensor_msgs，不启动节点）
ROS_SETUP=""
for distro in noetic jazzy humble; do
    candidate="/opt/ros/$distro/setup.bash"
    if [ -f "$candidate" ]; then
        ROS_SETUP="$candidate"
        break
    fi
done

if [ -z "$ROS_SETUP" ]; then
    echo "No ROS installation found under /opt/ros/{noetic,jazzy,humble}."
    echo "Install one of them, then re-run. On Ubuntu 24.04, Jazzy is sufficient for tests."
    exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"

if ! python3 -m pytest --version >/dev/null 2>&1; then
    echo "Installing pytest..."
    if command -v apt-get >/dev/null 2>&1; then
        sudo apt-get update -qq
        sudo apt-get install -y python3-pytest
    else
        python3 -m pip install --user pytest
    fi
fi

export PYTHONPATH="$PKG_DIR${PYTHONPATH:+:$PYTHONPATH}"

echo "=== pytest (ROS ${ROS_DISTRO:-unknown}, no graph) ==="
cd "$PKG_DIR" || exit 1
python3 -m pytest test/ -v
