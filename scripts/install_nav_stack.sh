#!/usr/bin/env bash
#
# 源码编译 ROS 1 导航栈（move_base / amcl / map_server / costmap_2d / global_planner）。
#
#   bash scripts/install_nav_stack.sh
#
# 为什么必须源码编译而不是 apt 安装：
# 本机是 Ubuntu 24.04(noble)，而 ROS Noetic 官方只发布 20.04(focal) 的二进制包。
# 现有 Noetic 来自社区 PPA ros-for-jammy，该 PPA 只有 276 个包，不含任何导航组件
# （move_base / amcl / map_server / costmap_2d 全都没有）。换 apt 镜像源解决不了，
# 因为这些包对本发行版根本不存在。好在导航栈的依赖 PPA 里齐全，源码编译可行。
#
# 装到独立工作空间而不是项目 _ws（默认 ~/ROS_AI_Robot_Workspace_ws）：
# 项目构建目录由 sync_ws.sh 管理；导航栈单独放在 ~/ROS_AI_Robot_Workspace_nav_ws，
# 由 ros_env.sh 作为底层 overlay 先 source，项目 _ws 叠在其上。
#
# 环境变量：
#   NAV_WS           导航栈工作空间，默认 ~/ROS_AI_Robot_Workspace_nav_ws
#   NAV_BUILD_JOBS   并行编译任务数，默认按内存推算（见下）

set -o pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/wsl_paths.sh
source "$REPO_DIR/scripts/wsl_paths.sh"

# 并行度按「内存」而不是「核数」推算。这台机器 32 核 / 10 GB：catkin_make 默认
# 开满 32 路，而 costmap_2d、move_base 这类重模板 C++ 单个 cc1plus 峰值可达 1-2 GB，
# 32 路必然把内存吃穿，触发 OOM killer（表现为 "c++: fatal error: Killed signal
# terminated program cc1plus"）。按每任务 1.5 GB 留量估算更稳，且比盲目加内存有效。
if [ -z "$NAV_BUILD_JOBS" ]; then
    mem_mb="$(awk '/MemTotal/ {print int($2/1024)}' /proc/meminfo 2>/dev/null || echo 4096)"
    NAV_BUILD_JOBS=$(( mem_mb / 1536 ))
    [ "$NAV_BUILD_JOBS" -lt 1 ] && NAV_BUILD_JOBS=1
    cpus="$(nproc 2>/dev/null || echo 1)"
    [ "$NAV_BUILD_JOBS" -gt "$cpus" ] && NAV_BUILD_JOBS="$cpus"
    # 再压一档上限：编译 navigation 时并行度收益很快饱和，而 OOM 的代价是整轮重来。
    [ "$NAV_BUILD_JOBS" -gt 6 ] && NAV_BUILD_JOBS=6
fi

echo "=== ROS 1 导航栈源码安装 ==="
echo "工作空间 : $NAV_WS"
echo "并行度   : $NAV_BUILD_JOBS  (内存 $(awk '/MemTotal/ {print int($2/1024)}' /proc/meminfo) MB, $(nproc) 核)"
echo

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "未找到 ROS Noetic。先执行: bash $REPO_DIR/scripts/install_ros_noetic.sh"
    exit 1
fi

# ---------------------------------------------------------------- 系统依赖
# map_server 用 SDL + SDL_image 读 PGM/PNG 地图，用 yaml-cpp 解析地图 yaml。
# 这三个在 Ubuntu 官方源里有，不受 ROS 源不可达的影响。
echo "=== 1/3 安装系统依赖（需要 sudo 密码）==="

# Ubuntu 的 unattended-upgrades 是定时任务，随时可能在后台占着 dpkg 锁，
# 此时 apt 会直接失败退出。等它跑完比让使用者反复重试更省事——它通常只需要
# 一两分钟。这里主动轮询锁，而不是失败后一走了之。
wait_for_dpkg_lock() {
    local waited=0 limit=300
    while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 ||
          sudo fuser /var/lib/dpkg/lock >/dev/null 2>&1; do
        if [ "$waited" -eq 0 ]; then
            echo "dpkg 锁被占用（多半是 unattended-upgrades 在跑），等待它结束..."
        fi
        if [ "$waited" -ge "$limit" ]; then
            echo
            echo "等待 ${limit}s 后 dpkg 锁仍被占用。查看是谁占着:"
            echo "    sudo fuser -v /var/lib/dpkg/lock-frontend"
            echo "若确认是卡死的残留进程，可以结束它后重跑本脚本。"
            return 1
        fi
        sleep 5
        waited=$(( waited + 5 ))
        echo "  ...已等待 ${waited}s"
    done
    return 0
}

wait_for_dpkg_lock || exit 1

sudo apt-get install -y --no-install-recommends \
    libsdl1.2-dev libsdl-image1.2-dev libyaml-cpp-dev || {
    echo
    echo "系统依赖安装失败。若提示找不到包，先执行: sudo apt-get update"
    exit 1
}
echo

# ---------------------------------------------------------------- 拉取源码
# 只取三个仓库：
#   navigation      导航栈本体（含 voxel_grid、nav_core 等内部依赖）
#   navigation_msgs 仅取 move_base_msgs —— PPA 缺这个包，而 move_base 依赖它
#   geometry2       仅取 tf2_sensor_msgs —— PPA 同样缺，costmap_2d 依赖它
# 后两个仓库的其余包（tf2、tf2_ros 等）已由 PPA 以 deb 形式装好，重复编译会与
# 系统版本冲突，因此克隆后立即裁剪，只保留需要的那一个包。
clone_or_update() {
    local url="$1" branch="$2" dir="$3"
    if [ -d "$dir/.git" ]; then
        echo "--- 更新 $(basename "$dir") ---"
        git -C "$dir" fetch --depth 1 origin "$branch" && \
            git -C "$dir" reset --hard "origin/$branch" || return 1
    else
        echo "--- 克隆 $(basename "$dir") ($branch) ---"
        rm -rf "$dir"
        git clone --depth 1 --branch "$branch" "$url" "$dir" || return 1
    fi
}

echo "=== 2/3 拉取源码 ==="
mkdir -p "$NAV_WS/src" || exit 1

clone_or_update https://github.com/ros-planning/navigation.git \
    noetic-devel "$NAV_WS/src/navigation" || exit 1

clone_or_update https://github.com/ros-planning/navigation_msgs.git \
    ros1 "$NAV_WS/src/navigation_msgs" || exit 1

clone_or_update https://github.com/ros/geometry2.git \
    noetic-devel "$NAV_WS/src/geometry2" || exit 1

# 裁剪：只留 PPA 缺失的那两个包，其余交给系统 deb，避免同名包双份编译。
prune_to() {
    local repo="$1" keep="$2"
    find "$repo" -mindepth 1 -maxdepth 1 -not -name "$keep" -not -name '.git' \
        -exec rm -rf {} + 2>/dev/null
}
prune_to "$NAV_WS/src/navigation_msgs" move_base_msgs
prune_to "$NAV_WS/src/geometry2" tf2_sensor_msgs

# navigation 仓库里这两个包本项目用不到，且会额外引入依赖，直接排除以缩短编译。
rm -rf "$NAV_WS/src/navigation/navigation"          # metapackage，只是依赖聚合
echo

# ---------------------------------------------------------------- 编译
echo "=== 3/3 编译（并行度 $NAV_BUILD_JOBS）==="
echo "这一步耗时较长（约 10-25 分钟），C++ 编译占绝大部分。"
echo
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
cd "$NAV_WS" || exit 1

# -j 限制 make 任务数，-l 限制负载均值，两者并用才能真正压住内存峰值：
# 只给 -j 时 catkin_make 仍可能在多个包之间并行展开。
catkin_make -DCMAKE_BUILD_TYPE=Release -j"$NAV_BUILD_JOBS" -l"$NAV_BUILD_JOBS" || {
    echo
    echo "=============================================================="
    echo "编译失败。"
    echo
    echo "若日志里出现 'Killed signal terminated program cc1plus' 或 'internal"
    echo "compiler error'，那就是内存不足。处理顺序："
    echo "  1) 先降并行度重试（比加内存更有效）："
    echo "       NAV_BUILD_JOBS=2 bash scripts/install_nav_stack.sh"
    echo "  2) 仍失败再调高 WSL2 内存（%UserProfile%\\.wslconfig 的 memory=），"
    echo "     然后 wsl --shutdown 重启生效。"
    echo "=============================================================="
    exit 1
}

echo
echo "=== 完成 ==="
echo "已安装到: $NAV_WS/devel"
echo "scripts/ros_env.sh 会自动 source 它，无需手动配置。"
echo "接着执行:  bash scripts/sync_ws.sh --build"
