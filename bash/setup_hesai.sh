#!/bin/bash

set -e

echo "======================================"
echo "Hesai ROS2 Driver Setup (JT128)"
echo "======================================"

ROS_DISTRO=humble
WS=~/ros2_ws

# -----------------------------
# STEP 1: Source ROS
# -----------------------------
echo "[1/6] Sourcing ROS2..."
source /opt/ros/$ROS_DISTRO/setup.bash

# -----------------------------
# STEP 2: Install dependencies
# -----------------------------
echo "[2/6] Installing dependencies..."

sudo apt-get update
sudo apt-get install -y \
    libboost-all-dev \
    libyaml-cpp-dev

# -----------------------------
# STEP 3: Create workspace
# -----------------------------
echo "[3/6] Creating workspace..."
if [ ! -d $WS ]; then
    mkdir -p $WS/src
else
    echo "WS already exists."
fi
cd $WS/src

# -----------------------------
# STEP 4: Clone driver
# -----------------------------
echo "[4/6] Cloning Hesai ROS driver..."

if [ ! -d "HesaiLidar_ROS_2.0" ]; then
    git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
else
    echo "Repo already exists, skipping clone"
fi

# -----------------------------
# STEP 5: Build
# -----------------------------
echo "[5/6] Building workspace..."

cd $WS

# Optional: disable PTCS (safer if not needed)


# -----------------------------
# STEP 6: Done
# -----------------------------
echo "======================================"
echo "Hesai ROS2 setup COMPLETE"
echo "======================================"

echo ""
echo "USAGE:"
echo "--------------------------------------"
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
echo "source $WS/install/setup.bash"
echo ""
echo "Run:"
echo "ros2 launch hesai_ros_driver start.py"
