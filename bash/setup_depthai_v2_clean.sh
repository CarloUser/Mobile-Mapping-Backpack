#!/bin/bash

set -e

echo "======================================"
echo "DepthAI ROS v2 CLEAN SETUP"
echo "======================================"

# -----------------------------
# STEP 1: Source ROS2
# -----------------------------
echo "[1/5] Sourcing ROS2..."
source /opt/ros/humble/setup.bash

# -----------------------------
# STEP 2: Install DepthAI core (official)
# -----------------------------
echo "[2/5] Installing DepthAI core + dependencies..."

sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash

# -----------------------------
# STEP 3: Prepare workspace
# -----------------------------
echo "[3/5] Preparing workspace..."

cd ~/ros_ws/src

# -----------------------------
# STEP 4: Clone depthai-ros (v2)
# -----------------------------
echo "[4/5] Cloning depthai-ros..."

if [ ! -d "depthai-ros" ]; then
    git clone --branch humble https://github.com/luxonis/depthai-ros.git
else
    echo "depthai-ros already exists, skipping clone"
fi

# -----------------------------
# STEP 5: Install ROS deps + build
# -----------------------------
echo "[5/5] Installing ROS deps and building..."

cd ~/ros_ws

rosdep install --from-paths src --ignore-src -r -y

MAKEFLAGS="-j4 -l4" colcon build --symlink-install --parallel-workers 4

echo "======================================"
echo "DepthAI v2 setup COMPLETE"
echo "======================================"

echo ""
echo "USAGE:"
echo "--------------------------------------"
echo "Default terminal → v3 (APT)"
echo ""
echo "To use v2:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/ros_ws/install/setup.bash"
echo ""
echo "IMPORTANT:"
echo "Run v2 and v3 in separate terminals!"
