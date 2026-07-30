#!/usr/bin/env bash
#
# 把本仓库里的包同步到 WSL 的 colcon 工作空间，并可选地构建与测试。
#
# 为什么需要这一步：本仓库在 Windows 侧编辑，而 ROS 2 工作空间在 WSL 的
# ~/ros2_ws，两者是两份独立拷贝。在 Windows 改完代码不同步，colcon 构建的
# 仍然是旧版本——症状往往是"改了没效果"，很难往构建路径上想。直接在 /mnt/c
# 上构建虽然可行，但跨文件系统 I/O 慢得多，所以工作空间保留自己的副本。
#
# 在仓库根目录的 Windows PowerShell 里执行：
#     wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh                  # 只同步
#     wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh --build          # 同步 + 构建
#     wsl -d Ubuntu bash ./scripts/sync_to_wsl.sh --build --test   # 同步 + 构建 + 测试
#
# 工作空间不在 ~/ros2_ws 时用 ROS2_WS=/path/to/ws 覆盖。

# 刻意不加 -u：ROS 的 setup.bash 会读取未定义变量，开了 -u 会在 source 时直接退出。
# 也不加 -e，所以下面每个关键步骤都自己带 || exit。
set -o pipefail

# 用脚本自身位置反推仓库根目录，这样从任何工作目录调用都成立。
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO_DIR/src/ai_robot_nav"
WS="${ROS2_WS:-$HOME/ros2_ws}"
DST="$WS/src/ai_robot_nav"

DO_BUILD=0
DO_TEST=0
for arg in "$@"; do
    case "$arg" in
        --build) DO_BUILD=1 ;;
        # --test 隐含 --build：没构建过的代码测不了。
        --test)  DO_BUILD=1; DO_TEST=1 ;;
        # 拼错的参数直接报错退出，而不是静默当成"只同步"。
        *) echo "unknown option: $arg"; exit 2 ;;
    esac
done

if [ ! -d "$SRC" ]; then
    echo "package not found at $SRC"
    exit 1
fi

echo "=== syncing $SRC -> $DST ==="
# 先整个删掉再拷贝，而不是覆盖：否则在 Windows 侧删除的文件会留在目标目录里，
# 继续参与构建。
rm -rf "$DST"
mkdir -p "$DST"
cp -r "$SRC/." "$DST/"

# Windows 侧 Python 生成的缓存对 Linux 无效，且会干扰 colcon 的导入。
find "$DST" -name __pycache__ -type d -prune -exec rm -rf {} +
rm -rf "$DST/.pytest_cache"

# 在 Windows 上编辑过的文件可能带 CRLF，会让 shebang 和 YAML 解析出错。
find "$DST" -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.xml' -o -name '*.cfg' \) \
    -exec sed -i 's/\r$//' {} +
# 从 /mnt/c 拷过来的文件权限位可能不对，补上属主的读写与目录进入权限。
chmod -R u+rwX "$DST"

echo "synced $(find "$DST" -type f | wc -l) files"

if [ "$DO_BUILD" -eq 0 ]; then
    exit 0
fi

# 优先 Jazzy，回退 Humble。两个都没有说明环境不对，直接报错而不是继续。
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
# 清掉旧产物，避免已删除的模块残留在 install 空间里被继续导入。
# 只删本包的目录，不影响工作空间里的其他包。
rm -rf build/ai_robot_nav install/ai_robot_nav
colcon build --packages-select ai_robot_nav --symlink-install || exit 1

if [ "$DO_TEST" -eq 1 ]; then
    echo
    echo "=== colcon test ==="
    # colcon test 在有测试失败时返回非零。这里用 || true 咽掉，好让下面的
    # test-result 有机会把具体哪条失败打出来——那才是真正需要看的信息。
    colcon test --packages-select ai_robot_nav || true
    colcon test-result --all
fi

echo
# 必须在新 shell 里 source：本脚本的环境变量不会传回调用方。
echo "done. In a new shell run:  source $WS/install/setup.bash"
