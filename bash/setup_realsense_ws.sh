#!/bin/bash

set -e

echo "======================================"
echo "RealSense ROS2 (SOURCE INSTALL)"
echo "======================================"

ROS_DISTRO=humble
WS=~/ros2_ws

# -----------------------------
# STEP 1: Source ROS
# -----------------------------
echo "[1/6] Sourcing ROS2..."
source /opt/ros/$ROS_DISTRO/setup.bash

# -----------------------------
# STEP 2: Create workspace
# -----------------------------
echo "[2/6] Creating workspace..."

if [ ! -d "$WS" ]; then
	echo "Directory is missing! Creating it now..."
	mkdir -p $WS/src

cd $WS/src

# -----------------------------
# STEP 3: Clone repo
# -----------------------------
echo "[3/6] Cloning realsense-ros..."

if [ ! -d "realsense-ros" ]; then
    git clone https://github.com/realsenseai/realsense-ros.git -b ros2-master
else
    echo "Repo already exists, skipping clone"
fi

# -----------------------------
# STEP 4: Install dependencies
# -----------------------------
echo "[4/6] Installing dependencies..."

sudo apt-get update
sudo apt-get install -y python3-rosdep

# Initialize rosdep only if needed
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update

cd $WS

rosdep install -i --from-path src \
  --rosdistro $ROS_DISTRO \
  --skip-keys=librealsense2 \
  -y

# -----------------------------
# STEP 5: Build
# -----------------------------
echo "[5/6] Building workspace..."

colcon build --symlink-install --parallel-workers 2

# -----------------------------
# STEP 6: Done
# -----------------------------
echo "======================================"
echo "RealSense ROS setup COMPLETE"
echo "======================================"

echo ""
echo "USAGE:"
echo "--------------------------------------"
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
echo "source $WS/install/setup.bash"
echo ""
echo "Run:"
echo "ros2 launch realsense2_camera rs_launch.py"
