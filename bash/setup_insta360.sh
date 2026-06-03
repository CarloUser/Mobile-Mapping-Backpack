#!/bin/bash

set -e

echo "======================================"
echo "Insta360 ROS Driver Setup (NO SDK)"
echo "======================================"

ROS_DISTRO=humble
WS="$HOME/ros2_ws"

# -----------------------------
# STEP 1: Source ROS2
# -----------------------------
echo "[1/5] Sourcing ROS2..."

source "/opt/ros/$ROS_DISTRO/setup.bash"

# -----------------------------
# STEP 2: Create workspace
# -----------------------------
echo "[2/5] Creating workspace..."

mkdir -p "$WS/src"
cd "$WS/src"

# -----------------------------
# STEP 3: Clone driver
# -----------------------------
echo "[3/5] Cloning driver..."

if [ ! -d "insta360_ros_driver" ]; then
    git clone -b humble https://github.com/ai4ce/insta360_ros_driver
else
    echo "Driver already exists."
fi

# -----------------------------
# STEP 4: Install ROS deps
# -----------------------------
echo "[4/5] Installing dependencies..."

cd "$WS"

sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y

# -----------------------------
# STEP 5: Done
# -----------------------------
echo "[5/5] Driver repo ready."

echo "======================================"
echo "Insta360 ROS setup COMPLETE"
echo "======================================"

echo ""
echo "Next:"
echo "--------------------------------------"
echo "1. Run setup_insta360_sdk.sh"
echo "2. Run config_insta360.sh"
echo ""
echo "Build happens in config_insta360.sh after SDK files are copied."