#!/bin/bash

set -e

echo "======================================"
echo "RealSense (Fixed Humble Setup)"
echo "======================================"

ROS_DISTRO=humble

# -----------------------------
# STEP 1: Update
# -----------------------------
echo "[1/4] Updating..."

sudo apt-get update

# -----------------------------
# STEP 2: System deps
# -----------------------------
echo "[2/4] Installing system dependencies..."

sudo apt-get install -y \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev

# -----------------------------
# STEP 3: ROS deps
# -----------------------------
echo "[3/4] Installing ROS dependencies..."

sudo apt-get install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-image-transport

# -----------------------------
# STEP 4: RealSense
# -----------------------------
echo "[4/4] Installing RealSense..."

sudo apt-get install -y \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-description

echo "======================================"
echo "RealSense setup COMPLETE"
echo "======================================"

echo ""
echo "Test:"
echo "realsense-viewer"

echo ""
echo "Run:"
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
echo "ros2 launch realsense2_camera rs_launch.py"