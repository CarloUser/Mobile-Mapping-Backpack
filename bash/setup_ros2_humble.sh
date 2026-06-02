#!/bin/bash

set -e  # stop on error

echo "=============================="
echo "ROS2 Humble Full Setup Script"
echo "=============================="

# -----------------------------
# STEP 1: Locale (recommended)
# -----------------------------
echo "[1/8] Setting locale..."

sudo apt update
sudo apt install -y locales

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

echo "Locale set to:"
locale

# -----------------------------
# STEP 2: Enable universe repo
# -----------------------------
echo "[2/8] Enabling Ubuntu universe repository..."

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# -----------------------------
# STEP 3: Add ROS2 apt source
# -----------------------------
echo "[3/8] Adding ROS2 repository..."

sudo apt update
sudo apt install -y curl

ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
"https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb

# -----------------------------
# STEP 4: Install ROS2 Humble
# -----------------------------
echo "[4/8] Installing ROS2 Humble..."

sudo apt update
sudo apt install -y ros-humble-desktop

# -----------------------------
# STEP 5: Setup ROS environment
# -----------------------------
echo "[5/8] Setting up ROS environment..."

if ! grep -q "/opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

source /opt/ros/humble/setup.bash

# -----------------------------
# STEP 6: Install dev tools
# -----------------------------
echo "[6/8] Installing development tools..."

sudo apt install -y ros-dev-tools

# -----------------------------
# STEP 7: Initialize rosdep
# -----------------------------
echo "[7/8] Initializing rosdep..."

sudo rosdep init || true
rosdep update

# -----------------------------
# STEP 8: Create workspace
# -----------------------------
echo "[8/8] Creating ROS2 workspace..."

mkdir -p ~/ros_ws/src
cd ~/ros_ws

colcon build

source ~/ros_ws/install/setup.bash

echo "=============================="
echo "SETUP COMPLETE"
echo "=============================="
echo "Run: source ~/.bashrc"
echo "Then test with: ros2"
