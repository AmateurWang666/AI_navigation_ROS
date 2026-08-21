#!/usr/bin/env bash
#
# 兼容入口：将 ai_robot_nav 与 tjark_agv 同步到 WSL 的 ROS 1 catkin 工作空间。
# 实际逻辑见 setup_ros1_ws.sh。
#
# 只跑单元测试（无需 catkin 构建）：
#     wsl -d Ubuntu bash ./scripts/run_tests.sh
#
# 同步 + 构建 + 测试：
#     wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh --build --test

set -o pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$REPO_DIR/scripts/setup_ros1_ws.sh" "$@"
