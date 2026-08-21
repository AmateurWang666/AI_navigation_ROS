#!/usr/bin/env bash
#
# 在 Ubuntu 24.04 WSL 上安装 ROS Noetic + Gazebo Classic，并构建 ros_ws。
#
# 背景：官方 Noetic 只支持 Ubuntu 20.04；24.04 需用社区 PPA。
# 若你已有 Ubuntu 20.04，可跳过 PPA，直接用官方源：
#   http://wiki.ros.org/noetic/Installation/Ubuntu
#
# 用法（WSL 终端，需 sudo 密码）：
#   cd /mnt/c/Users/ROG/Desktop/ROS2_AI_Robot_Workspace
#   bash ./scripts/install_ros_noetic.sh
#
# 安装完成后新开终端：
#   source /opt/ros/noetic/setup.bash
#   source ~/ros_ws/devel/setup.bash
#   roslaunch tjark_agv tjark_agv.launch

set -o pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"

echo "=== Detected Ubuntu $CODENAME ==="

if [ "$EUID" -ne 0 ] && ! sudo -n true 2>/dev/null; then
    echo "This script needs sudo. You will be prompted for your password."
fi

install_noetic_official_focal() {
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-get update
    sudo apt-get install -y curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt-get install -y ros-noetic-desktop-full
}

install_noetic_noble_ppa() {
    echo "=== Adding ROS Noetic PPA for Ubuntu 24.04 (noble) ==="
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl
    sudo add-apt-repository -y ppa:ros-for-jammy/noble
    sudo apt-get update

    # desktop-full 若不可用，则逐个装核心组件
    if apt-cache show ros-noetic-desktop-full >/dev/null 2>&1; then
        sudo apt-get install -y ros-noetic-desktop-full
    else
        echo "ros-noetic-desktop-full not in PPA; installing core packages..."
        sudo apt-get install -y \
            ros-noetic-ros-base \
            ros-noetic-roslaunch \
            ros-noetic-robot-state-publisher \
            ros-noetic-xacro \
            ros-noetic-cv-bridge \
            ros-noetic-gazebo-ros \
            ros-noetic-gazebo-ros-pkgs \
            ros-noetic-gazebo-plugins \
            ros-noetic-joint-state-publisher \
            ros-noetic-tf \
            ros-noetic-controller-manager
    fi
}

case "$CODENAME" in
    focal)
        install_noetic_official_focal
        ;;
    noble)
        install_noetic_noble_ppa
        ;;
    *)
        echo "Unsupported Ubuntu codename: $CODENAME"
        echo "Supported: focal (20.04) via official ROS, noble (24.04) via PPA."
        exit 1
        ;;
esac

echo
echo "=== Gazebo Classic + build tools ==="
sudo apt-get install -y \
    gazebo \
    libgazebo-dev \
    python3-rosdep \
    python3-catkin-tools \
    python3-pip \
    build-essential

if ! sudo rosdep init 2>/dev/null; then
    true  # already initialized
fi
rosdep update

echo
echo "=== Building catkin workspace ==="
# 非 root 执行同步与构建
if [ "$EUID" -eq 0 ]; then
    echo "Re-run workspace setup as normal user:"
    echo "  bash $REPO_DIR/scripts/setup_ros1_ws.sh --build"
else
    bash "$REPO_DIR/scripts/setup_ros1_ws.sh" --build
fi

echo
echo "=============================================="
echo "Installation complete."
echo
echo "Add to ~/.bashrc (optional):"
echo "  echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc"
echo "  echo 'source ~/ros_ws/devel/setup.bash' >> ~/.bashrc"
echo
echo "Start simulation (new terminal):"
echo "  source /opt/ros/noetic/setup.bash"
echo "  source ~/ros_ws/devel/setup.bash"
echo "  roslaunch tjark_agv tjark_agv.launch"
echo "=============================================="
