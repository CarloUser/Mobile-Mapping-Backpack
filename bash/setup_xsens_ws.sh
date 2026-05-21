#!/bin/bash

set -e

echo "======================================"
echo "Xsens MTi ROS2 Setup"
echo "======================================"

ROS_DISTRO=humble
WS=~/xsens_ws
SDK_DEB=~/Downloads/xsens-xme-sdk_2026.4.0-1_amd64.deb

# -----------------------------
# STEP 1: Source ROS
# -----------------------------
echo "[1/7] Sourcing ROS2..."
source /opt/ros/$ROS_DISTRO/setup.bash

# -----------------------------
# STEP 2: Install SDK
# -----------------------------
echo "[2/7] Installing Xsens SDK..."

if [ ! -f "$SDK_DEB" ]; then
    echo "Downloading SDK..."
    wget -O $SDK_DEB https://www.xsens.com/hubfs/Downloads/Software/MVN/Releases/2026.4/Linux/xsens-xme-sdk_2026.4.0-1_amd64.deb
fi

sudo dpkg -i $SDK_DEB || sudo apt-get install -f -y

# cleanup deb
rm -f $SDK_DEB

# -----------------------------
# STEP 3: Create workspace
# -----------------------------
echo "[3/7] Creating workspace..."

mkdir -p $WS/src
cd $WS/src

# -----------------------------
# STEP 4: Clone driver
# -----------------------------
echo "[4/7] Cloning driver..."

if [ ! -d "ros2_xsens_mti_driver" ]; then
    git clone https://github.com/xsenssupport/ros2_xsens_mti_driver.git
else
    echo "Driver already exists"
fi

# -----------------------------
# STEP 5: Fix permissions
# -----------------------------
echo "[5/7] Fixing permissions..."

chmod -R o+rw ros2_xsens_mti_driver

# -----------------------------
# STEP 6: Build
# -----------------------------
echo "[6/7] Building..."

cd $WS

colcon build \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON

# -----------------------------
# STEP 7: Done
# -----------------------------
echo "======================================"
echo "Xsens setup COMPLETE"
echo "======================================"

echo ""
echo "USAGE:"
echo "--------------------------------------"
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
echo "source $WS/install/setup.bash"
echo ""
echo "Run:"
echo "ros2 launch ros2_xsens_mti_driver xsens_mti_node.launch.py"
