#!/bin/bash

set -e

echo "======================================"
echo "Livox ROS2 Avia Setup"
echo "======================================"

WS=~/ros2_ws

# -----------------------------
# STEP 1: Source ROS2
# -----------------------------
echo "[1/5] Sourcing ROS2..."
source /opt/ros/humble/setup.bash

# -----------------------------
# STEP 2: Create workspace
# -----------------------------
echo "[2/5] Creating workspace..."

mkdir -p "$WS/src"
cd "$WS/src"

# -----------------------------
# STEP 3: Clone repo
# -----------------------------
echo "[3/5] Cloning livox_ros2_avia..."

if [ ! -d "livox_ros2_avia" ]; then
    git clone https://github.com/ASIG-X/livox_ros2_avia.git
else
    echo "Repo already exists, skipping clone"
fi

# -----------------------------
# STEP 4: Build
# -----------------------------
echo "[4/5] Building workspace..."

cd "$WS"

sudo rosdep init 2>/dev/null || true
rosdep update

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --parallel-workers 2

# -----------------------------
# STEP 5: Done
# -----------------------------
echo "======================================"
echo "Livox setup COMPLETE"
echo "======================================"

echo ""
echo "USAGE:"
echo "--------------------------------------"
echo "source /opt/ros/humble/setup.bash"
echo "source $WS/install/setup.bash"
echo ""
echo "Launch:"
echo "ros2 launch livox_ros2_avia livox_lidar_launch.py"