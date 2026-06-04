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
cd ~/ros2_ws

rosdep install \
  --from-paths src/insta360_ros_driver \
  --ignore-src \
  -r -y

# -----------------------------
# STEP X: Fix Jetson OpenCV
# -----------------------------
echo "Fixing Jetson OpenCV..."

sudo apt-get update

# Remove Ubuntu/mixed OpenCV packages
sudo apt-get remove -y 'libopencv*' python3-opencv || true
sudo apt-get autoremove -y || true

# Remove leftover broken CMake/OpenCV files
sudo rm -rf /usr/lib/cmake/opencv4
sudo rm -f /usr/lib/libopencv_*
sudo ldconfig

# Install NVIDIA Jetson OpenCV packages
sudo apt-get install -y \
    nvidia-opencv \
    nvidia-opencv-dev

# Reinstall ROS deps that may have been removed
sudo apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-std-srvs

sudo ldconfig

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