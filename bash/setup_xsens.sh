#!/bin/bash

set -e

echo "======================================"
echo "Xsens ROS2 Driver + NTRIP Setup"
echo "======================================"

ROS_DISTRO=humble
WS=~/ros2_ws

# -----------------------------
# STEP 1: Source ROS2
# -----------------------------
echo "[1/5] Sourcing ROS2..."

source /opt/ros/$ROS_DISTRO/setup.bash

# -----------------------------
# STEP 2: Install dependencies
# -----------------------------
echo "[2/5] Installing dependencies..."

sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-mavros-msgs

# -----------------------------
# STEP 3: Create workspace
# -----------------------------
echo "[3/5] Creating workspace..."

if [ ! -d $WS ]; then
    mkdir -p $WS/src
else
    echo "WS already exists."
cd $WS/src

# -----------------------------
# STEP 4: Clone ROS2 driver
# -----------------------------
echo "[4/5] Cloning ROS2 branch..."

if [ ! -d "xsens_mti_ros2" ]; then
    git clone --branch ros2 https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git xsens_mti_ros2
else
    echo "✔ Driver already exists"
fi

# -----------------------------
# STEP 5: Build
# -----------------------------
echo "[5/5] Building..."

cd $WS
colcon build

echo "======================================"
echo "Xsens ROS2 setup COMPLETE"
echo "======================================"

echo ""
echo "Next steps:"
echo "--------------------------------------"
echo "1. Edit NTRIP config:"
echo "   nano $WS/src/xsens_mti_ros2/src/ntrip/launch/ntrip_launch.py"
echo ""
echo "2. Source:"
echo "   source /opt/ros/$ROS_DISTRO/setup.bash"
echo "   source $WS/install/setup.bash"
echo ""
echo "3. Run:"
echo "   ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py"
echo ""
echo "4. In another terminal:"
echo "   ros2 launch ntrip ntrip_launch.py"


