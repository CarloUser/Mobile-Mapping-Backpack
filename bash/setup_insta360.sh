#!/bin/bash

set -e

echo "======================================"
echo "Insta360 ROS Driver Setup (NO SDK)"
echo "======================================"

ROS_DISTRO=humble
WS=~/ros2_ws

# -----------------------------
# STEP 1: Source ROS2
# -----------------------------
echo "[1/6] Sourcing ROS2..."

source /opt/ros/$ROS_DISTRO/setup.bash

# -----------------------------
# STEP 2: Create workspace
# -----------------------------
echo "[2/6] Creating workspace..."

if [ ! -d $WS ]; then
    mkdir -p $WS/src
else
    echo "WS already exists."
cd $WS/src

# -----------------------------
# STEP 3: Clone driver
# -----------------------------
echo "[3/6] Cloning driver..."

if [ ! -d "insta360_ros_driver" ]; then
    git clone -b humble https://github.com/ai4ce/insta360_ros_driver
else
    echo "✔ Driver already exists"
fi

# -----------------------------
# STEP 4: Install ROS deps
# -----------------------------
echo "[4/6] Installing dependencies..."

cd $WS

sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y

# -----------------------------
# STEP 5: Build
# -----------------------------
echo "[5/6] Building..."

colcon build --symlink-install

# -----------------------------
# STEP 6: Setup udev (USB access)
# -----------------------------
echo "[6/6] Setting up udev rule..."

echo 'SUBSYSTEM=="usb", ATTR{manufacturer}=="Arashi Vision", SYMLINK+="insta", MODE="0777"' | \
sudo tee /etc/udev/rules.d/99-insta.rules > /dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "✔ udev rule created → /dev/insta"

# -----------------------------
# DONE
# -----------------------------
echo "======================================"
echo "Insta360 ROS setup COMPLETE"
echo "======================================"

echo ""
echo "IMPORTANT:"
echo "--------------------------------------"
echo "1. Copy SDK files manually:"
echo "   → headers → include/"
echo "   → libCameraSDK.so → lib/"
echo ""
echo "2. Set camera:"
echo "   → Dual-lens mode"
echo "   → USB mode = Android"
echo ""
echo "3. Replug camera"
echo ""

echo "Run:"
echo "--------------------------------------"
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
echo "source $WS/install/setup.bash"