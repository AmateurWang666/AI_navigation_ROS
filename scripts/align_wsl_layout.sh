#!/usr/bin/env bash
#
# 一次性对齐 WSL 目录命名，并清理与本项目无关的旧文件。
#
#   bash scripts/align_wsl_layout.sh           # 迁移旧目录名 + 列出/删除冗余
#   bash scripts/align_wsl_layout.sh --dry-run # 只预览，不改动
#
# 迁移（若新名不存在、旧名存在）：
#   ~/ros_ws      -> ~/ROS_AI_Robot_Workspace_ws
#   ~/ros_nav_ws  -> ~/ROS_AI_Robot_Workspace_nav_ws
#
# 可安全删除的冗余（旧 ROS 2 工作空间、调试临时目录、过期备份）：
#   ~/ros2_ws  ~/final_check  ~/sim_check  ~/ai_robot_nav_pre_refactor.tgz

set -o pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/wsl_paths.sh
source "$REPO_DIR/scripts/wsl_paths.sh"

DRY_RUN=0
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        -h|--help)
            sed -n '2,14p' "$0"
            exit 0
            ;;
        *) echo "unknown option: $arg"; exit 2 ;;
    esac
done

run() {
    if [ "$DRY_RUN" -eq 1 ]; then
        echo "[dry-run] $*"
    else
        echo "+ $*"
        "$@"
    fi
}

migrate_dir() {
    local old_path="$1" new_path="$2"
    if [ -d "$old_path" ] && [ ! -e "$new_path" ]; then
        echo "=== 迁移 $old_path -> $new_path ==="
        run mv "$old_path" "$new_path"
    elif [ -d "$old_path" ] && [ -d "$new_path" ]; then
        echo "=== 跳过迁移：$new_path 已存在（仍保留 $old_path，请手动合并后删除旧目录）==="
    fi
}

remove_redundant() {
    local path="$1" reason="$2"
    if [ -e "$path" ]; then
        echo "=== 删除冗余: $path ($reason) ==="
        run rm -rf "$path"
    fi
}

echo "仓库目录名: $PROJECT_NAME"
echo "目标 WSL 路径:"
echo "  构建工作空间 : $ROS1_WS"
echo "  导航栈工作空间: $NAV_WS"
echo

migrate_dir "$HOME/ros_ws" "$ROS1_WS"
migrate_dir "$HOME/ros_nav_ws" "$NAV_WS"

if [ -d "$ROS1_WS/build" ] && grep -rq 'ros_ws' "$ROS1_WS/build" 2>/dev/null; then
    echo "=== 清理项目构建缓存（仍引用旧路径 ~/ros_ws）==="
    run rm -rf "$ROS1_WS/build" "$ROS1_WS/devel"
fi
if [ -d "$NAV_WS/build" ] && grep -rq 'ros_nav_ws' "$NAV_WS/build" 2>/dev/null; then
    echo "=== 清理导航栈构建缓存（仍引用旧路径 ~/ros_nav_ws）==="
    echo "    迁移后需重建导航栈: bash scripts/install_nav_stack.sh"
    run rm -rf "$NAV_WS/build" "$NAV_WS/devel"
fi

echo
echo "=== 清理冗余（与本项目 ROS 1 栈无关）==="
remove_redundant "$HOME/ros2_ws" "旧 ROS 2 工作空间，项目已迁移至 ROS 1 Noetic"
remove_redundant "$HOME/final_check" "调试临时目录（launch.log / odom.csv）"
remove_redundant "$HOME/sim_check" "调试临时目录（launch.log / odom.csv）"
remove_redundant "$HOME/ai_robot_nav_pre_refactor.tgz" "重构前备份包，源码已在 Git 仓库"

echo
if [ "$DRY_RUN" -eq 1 ]; then
    echo "预览完成。确认后执行: bash scripts/align_wsl_layout.sh"
else
    echo "完成。接下来: bash scripts/sync_ws.sh --build"
fi
