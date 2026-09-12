#!/usr/bin/env bash
# cafe 仿真 + AI 导航一键启动。
#
#   bash scripts/sim.sh              # 单终端全流程，无 Gazebo 窗口
#   bash scripts/sim.sh --lidar      # 不调用 Ollama，纯激光（更少日志）
#   bash scripts/sim.sh --gui        # 带 Gazebo 窗口（WSL 较慢）
#   bash scripts/sim.sh --verbose    # 显示 Gazebo 相机 DEBUG（默认已静音）
#   bash scripts/sim.sh --map 图.yaml # 换一张地图（默认 maps/cafe.yaml）
#   bash scripts/sim.sh --no-nav     # 只跑反应式漫游，不起 move_base
#   bash scripts/sim.sh --allow-unknown  # 临时允许穿越未知区域（有安全风险，见 README）
#
#   bash scripts/sim.sh gazebo       # 只起仿真（终端 1）
#   bash scripts/sim.sh nav          # 只起导航（终端 2，自动等 /scan）
#   bash scripts/sim.sh stop         # 停止仿真

set -o pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/ros_env.sh
source "$REPO_DIR/scripts/ros_env.sh"

USE_IMAGE=true
HEADLESS=true
MODE=full
VERBOSE=false
MAP_FILE=""
NAVIGATION=auto      # auto = 装了导航栈就用，没装就退回反应式漫游
ALLOW_UNKNOWN=false

usage() {
    sed -n '2,15p' "$0"
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --lidar|--no-vision) USE_IMAGE=false ;;
        --gui)               HEADLESS=false ;;
        --verbose)           VERBOSE=true ;;
        --no-nav)            NAVIGATION=false ;;
        --allow-unknown)     ALLOW_UNKNOWN=true ;;
        --map)
            shift
            [ $# -gt 0 ] || { echo "--map 需要一个地图 yaml 路径"; exit 1; }
            MAP_FILE="$1"
            ;;
        gazebo|nav|stop|help|-h) MODE="$1" ;;
        *) echo "unknown option: $1"; usage 1 ;;
    esac
    shift
done

# 导航栈是源码编译的独立 overlay，未安装时 sim.launch 会因为找不到 move_base
# 而整体启动失败。与其抛一个 roslaunch 的 ResourceNotFound 让人去猜，
# 不如在这里探测一次并退回到反应式漫游——那是本项目原本就能跑的模式。
resolve_navigation() {
    if [ "$NAVIGATION" != auto ]; then
        return
    fi
    if rospack find move_base >/dev/null 2>&1; then
        NAVIGATION=true
        return
    fi
    NAVIGATION=false
    echo "=============================================="
    echo "  未检测到导航栈，本次只跑反应式漫游。"
    echo "  目标点导航需要先安装（约 10-25 分钟）："
    echo "      bash scripts/install_nav_stack.sh"
    echo "  说明见 docs/mapping.md"
    echo "=============================================="
    echo
}

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
    echo "  cafe 仿真已启动（默认无 Gazebo 窗口 = 正常）"
    echo "  成功标志：日志出现"
    echo "    FORWARD lin=0.15 ang=0.00 | clear ahead"
    echo "  另开终端看位移："
    echo "    source ${ROS1_WS}/devel/setup.bash"
    echo "    rostopic echo /odom/pose/pose/position/x"
    if [ "$NAVIGATION" = true ]; then
        echo "  导航到指定目的地（另开终端）："
        echo "    rosrun ai_robot_nav send_goal --list"
        echo "    rosrun ai_robot_nav send_goal hall"
    fi
    echo "  停止：Ctrl+C  或  bash scripts/sim.sh stop"
    echo "=============================================="
    echo
}

stop_sim() {
    echo "=== stopping simulation ==="
    pkill -f 'roslaunch ai_robot_nav sim.launch' 2>/dev/null || true
    pkill -f 'roslaunch ai_robot_nav ai_nav.launch' 2>/dev/null || true
    pkill -f 'roslaunch tjark_agv cafe_world.launch' 2>/dev/null || true
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
            echo "=== cafe world (headless); 终端 2 请运行: bash scripts/sim.sh nav ==="
        else
            echo "=== 正在启动 Gazebo 图形界面，等窗口弹出后再开终端 2 ==="
        fi
        echo
        exec roslaunch tjark_agv cafe_world.launch \
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
        resolve_navigation
        print_banner
        set -- \
            headless:="$HEADLESS" \
            gui:="$([ "$HEADLESS" = true ] && echo false || echo true)" \
            use_image:="$USE_IMAGE" \
            navigation:="$NAVIGATION"
        if [ -n "$MAP_FILE" ]; then
            set -- "$@" map_file:="$MAP_FILE"
        fi
        if [ "$ALLOW_UNKNOWN" = true ]; then
            set -- "$@" allow_unknown:=true
        fi
        exec roslaunch ai_robot_nav sim.launch "$@"
        ;;
esac
