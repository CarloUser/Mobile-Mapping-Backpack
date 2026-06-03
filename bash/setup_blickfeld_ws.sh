#!/bin/bash

set -e

echo "======================================"
echo "Blickfeld ROS2 Setup"
echo "======================================"

ROS_DISTRO=humble   # ⚠ works but officially foxy
WS=~/blickfeld_ws

# -----------------------------
# STEP 1: Source ROS
# -----------------------------
echo "[1/7] Sourcing ROS2..."
source /opt/ros/$ROS_DISTRO/setup.bash

# -----------------------------
# STEP 2: Install dependencies
# -----------------------------
echo "[2/7] Installing dependencies..."

sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-diagnostic-msgs \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions

# -----------------------------
# STEP 3: Create workspace
# -----------------------------
echo "[3/7] Creating workspace..."

if [ ! -d $WS ]; then
    mkdir -p $WS/src
else
    echo "WS already exists."
fi
cd $WS/src

# -----------------------------
# STEP 4: Add driver
# -----------------------------
echo "[4/7] Adding driver..."

echo ""
echo "⚠ IMPORTANT:"
echo "Download the Blickfeld ROS2 driver manually"
echo "and place it inside:"
echo "$WS/src/"
echo ""
read -p "Press ENTER after placing the driver..."

# -----------------------------
# STEP 5: BSL (scanner library)
# -----------------------------
echo "[5/7] Checking BSL..."

echo ""
echo "⚠ IMPORTANT:"
echo "Install Blickfeld Scanner Library (BSL) manually"
echo "System-wide (required)"
echo ""
read -p "Press ENTER after installing BSL..."

# -----------------------------
# STEP 6: Build
# -----------------------------
echo "[6/7] Building..."

cd $WS

rosdep update

rosdep install --from-paths src \
    --ignore-src \
    -r -y \
    --skip-keys "blickfeld-scanner"

colcon build --symlink-install --cmake-clean-first

# -----------------------------
# STEP 7: Done
# -----------------------------
echo "======================================"
echo "Blickfeld setup COMPLETE"
echo "======================================"

echo ""
echo "USAGE:"
echo "--------------------------------------"
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
echo "source $WS/install/setup.bash"
echo ""
echo "Set DDS:"
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo ""
echo "Run:"
echo "ros2 run blickfeld_driver blickfeld_driver_node --ros-args -p host:=<DEVICE_IP>"
