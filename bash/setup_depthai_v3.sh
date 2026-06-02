#!/bin/bash

set -e

echo "=============================="
echo "DepthAI ROS2 v3 Setup Script"
echo "=============================="

# -----------------------------
# STEP 1: Check ROS_DISTRO
# -----------------------------
echo "[1/5] Checking ROS distribution..."

if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS_DISTRO is not set!"
    echo "Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS_DISTRO = $ROS_DISTRO"

# -----------------------------
# STEP 2: Install testing repo
# -----------------------------
echo "[2/5] Installing ROS2 testing apt source..."

sudo apt update
sudo apt install -y ros2-testing-apt-source

# -----------------------------
# STEP 3: Update package list
# -----------------------------
echo "[3/5] Updating apt..."

sudo apt update

# -----------------------------
# STEP 4: Install DepthAI ROS v3
# -----------------------------
echo "[4/5] Installing DepthAI ROS v3..."

sudo apt install -y ros-$ROS_DISTRO-depthai-ros-v3

# -----------------------------
# STEP 5: Verify installation
# -----------------------------
echo "[5/5] Verifying installation..."

if ros2 pkg list | grep -q depthai; then
    echo "DepthAI packages found:"
    ros2 pkg list | grep depthai
else
    echo "WARNING: DepthAI package not found in ROS!"
fi

echo "=============================="
echo "DepthAI v3 installation DONE"
echo "=============================="
