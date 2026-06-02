#!/bin/bash

set -e

echo "======================================"
echo "Insta360 ROS2 Driver Setup"
echo "======================================"

ROS_DISTRO=humble
WS=~/ros2_ws
SDK_DIR=~/insta360_sdk   # where your SDK gets extracted

# -----------------------------
# STEP 1: Source ROS
# -----------------------------
echo "[1/7] Sourcing ROS2..."
source /opt/ros/$ROS_DISTRO/setup.bash

# -----------------------------
# STEP 2: Create workspace
# -----------------------------
echo "[2/7] Creating workspace..."

mkdir -p $WS/src
cd $WS/src

# -----------------------------
# STEP 3: Clone repo
# -----------------------------
echo "[3/7] Cloning insta360 driver..."

if [ ! -d "insta360_ros_driver" ]; then
    git clone -b humble https://github.com/ai4ce/insta360_ros_driver
else
    echo "Repo already exists, skipping clone"
fi

# -----------------------------
# STEP 4: Prepare SDK files
# -----------------------------
echo "[4/7] Preparing SDK files..."

DRIVER_DIR=$WS/src/insta360_ros_driver

mkdir -p $DRIVER_DIR/include
mkdir -p $DRIVER_DIR/lib

echo ""
echo "⚠ IMPORTANT:"
echo "--------------------------------------"
echo "Place the following files manually:"
echo ""
echo "Headers → $DRIVER_DIR/include/"
echo "libCameraSDK.so → $DRIVER_DIR/lib/"
echo ""
echo "From your SDK folder: $SDK_DIR"
echo ""
read -p "Press ENTER after copying files..."

# -----------------------------
# STEP 5: Install dependencies
# -----------------------------
echo "[5/7] Installing dependencies..."

cd $WS

rosdep install --from-paths src --ignore-src -r -y

# -----------------------------
# STEP 6: Build
# -----------------------------
echo "[6/7] Building workspace..."

colcon build --symlink-install --parallel-workers 2

# -----------------------------
# STEP 7: Setup udev rule
# -----------------------------
echo "[7/7] Setting up udev rule..."

echo 'SUBSYSTEM=="usb", ATTR{manufacturer}=="Arashi Vision", SYMLINK+="insta", MODE="0777"' | \
sudo tee /etc/udev/rules.d/99-insta.rules

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "======================================"
echo "Insta360 setup COMPLETE"
echo "======================================"

echo ""
echo "USAGE:"
echo "--------------------------------------"
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
echo "source $WS/install/setup.bash"
echo ""
echo "Device should appear as:"
echo "/dev/insta"
