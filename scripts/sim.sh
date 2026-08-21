#!/usr/bin/env bash
# 仿真 + 导航一键启动（单终端，推荐日常使用）。
#
#   bash scripts/sim.sh              # 无 GUI，空场景，带 AI 导航
#   bash scripts/sim.sh --lidar      # 不调用 Ollama，纯激光（更少日志）
#   bash scripts/sim.sh --gui        # 带 Gazebo 窗口（WSL 较慢）
#   bash scripts/sim.sh --verbose    # 显示 Gazebo 相机 DEBUG（默认已静音）
#
#   bash scripts/sim.sh stop         # 停止仿真

set -o pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/ros_env.sh
source "$REPO_DIR/scripts/ros_env.sh"

USE_IMAGE=true
HEADLESS=true
MODE=full
VERBOSE=false

usage() {
    sed -n '2,11p' "$0"
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --lidar|--no-vision) USE_IMAGE=false ;;
        --gui)               HEADLESS=false ;;
        --verbose)           VERBOSE=true ;;
        gazebo|nav|stop|help|-h) MODE="$1" ;;
        *) echo "unknown option: $1"; usage 1 ;;
    esac
    shift
done

setup_gui_env() {
    if [ "$HEADLESS" = true ]; then
        return
    fi
    # WSL2 下 Gazebo 窗口通常需要这些环境变量
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
}

setup_logging() {
    if [ "$VERBOSE" = true ]; then
        return
    fi
    local cfg="$REPO_DIR/src/ai_robot_nav/config/rosconsole_sim.config"
    if [ -f "$cfg" ]; then
        export ROSCONSOLE_CONFIG_FILE="$cfg"
    fi
    export GAZEBO_VERBOSE=0
}

print_banner() {
    echo "=============================================="
    echo "  仿真已启动（默认无 Gazebo 窗口 = 正常）"
    echo "  成功标志：日志出现"
    echo "    FORWARD lin=0.15 ang=0.00 | clear ahead"
    echo "  另开终端看位移："
    echo "    source ~/ros_ws/devel/setup.bash"
    echo "    rostopic echo /odom/pose/pose/position/x"
    echo "  停止：Ctrl+C  或  bash scripts/sim.sh stop"
    echo "=============================================="
    echo
}

stop_sim() {
    echo "=== stopping simulation ==="
    pkill -f 'roslaunch ai_robot_nav sim_full.launch' 2>/dev/null || true
    pkill -f 'roslaunch ai_robot_nav tjark_gazebo.launch' 2>/dev/null || true
    pkill -f 'roslaunch ai_robot_nav ai_nav.launch' 2>/dev/null || true
    pkill -f gzserver 2>/dev/null || true
    pkill -f gzclient 2>/dev/null || true
    sleep 1
    echo "done"
}

wait_for_scan() {
    echo "=== waiting for /scan (max 120s) ==="
    local i=0
    while [ "$i" -lt 120 ]; do
        if rostopic list 2>/dev/null | grep -qx '/scan'; then
            echo "sensor ready"
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done
    echo "timeout: /scan not found. Is Gazebo running?"
    return 1
}

setup_logging
setup_gui_env

case "$MODE" in
    help|-h) usage 0 ;;
    stop)    stop_sim ;;
    gazebo)
        stop_sim
        if [ "$HEADLESS" = true ]; then
            print_banner
        else
            echo "=============================================="
            echo "  正在启动 Gazebo 图形界面（请等待窗口弹出）"
            echo "  首次打开可能需要 1–2 分钟"
            echo "  终端 2 请在看到窗口后再运行: bash scripts/sim.sh nav"
            echo "=============================================="
            echo
        fi
        exec roslaunch ai_robot_nav tjark_gazebo.launch \
            headless:="$HEADLESS" \
            gui:="$([ "$HEADLESS" = true ] && echo false || echo true)"
        ;;
    nav)
        wait_for_scan || exit 1
        exec roslaunch ai_robot_nav ai_nav.launch \
            use_sim_time:=true \
            use_image:="$USE_IMAGE" \
            params_file:="$NAV_PARAMS"
        ;;
    full)
        stop_sim
        print_banner
        exec roslaunch ai_robot_nav sim_full.launch \
            headless:="$HEADLESS" \
            gui:="$([ "$HEADLESS" = true ] && echo false || echo true)" \
            use_image:="$USE_IMAGE"
        ;;
esac
