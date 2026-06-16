#!/bin/bash
set -e

echo "=============================="
echo "DepthAI ROS2 v2 Setup Script"
echo "=============================="

ROS_DISTRO=humble
WS=$HOME/ros2_ws

echo "[1/7] Source ROS..."
source /opt/ros/$ROS_DISTRO/setup.bash

echo "[2/7] Install dependencies..."
sudo apt update
sudo apt install -y \
  git cmake build-essential python3-rosdep \
  libopencv-dev libpcl-dev libusb-1.0-0-dev

echo "[3/7] Create workspace..."
mkdir -p "$WS/src"
cd "$WS/src"

echo "[4/7] Clone repositories..."
if [ ! -d depthai-ros ]; then
  git clone -b humble https://github.com/luxonis/depthai-ros.git
fi

if [ ! -d depthai-core ]; then
  git clone -b v2_stable https://github.com/luxonis/depthai-core.git
fi

echo "[5/7] Build depthai-core v2..."
cd "$WS/src/depthai-core"
git submodule update --init --recursive

rm -rf build
mkdir build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON

make -j$(nproc)

make install DESTDIR="$WS/depthai_install"

DEPTHAI_DIR="$WS/depthai_install$WS/src/depthai-core/build/install/lib/cmake/depthai"

echo "[6/7] Install ROS deps..."
cd "$WS"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "[7/7] Build depthai-ros v2..."
rm -rf build install log

echo "[6.5/7] Installing common ROS runtime dependencies..."

sudo apt install -y \
  ros-$ROS_DISTRO-diagnostic-updater \
  ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-image-transport \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-message-filters \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-tf2-geometry-msgs \
  ros-$ROS_DISTRO-rviz2

colcon build \
  --symlink-install \
  --packages-select \
    depthai \
    depthai_ros_msgs \
    depthai_descriptions \
    depthai_bridge \
    depthai_ros_driver \
    depthai_ros \
  --parallel-workers 1 \
  --cmake-args -Ddepthai_DIR="$DEPTHAI_DIR"

echo "=============================="
echo "DepthAI ROS2 v2 DONE"
echo "=============================="
echo "Run:"
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
echo "source $WS/install/setup.bash"

echo ""
echo "Verifying installation..."

ros2 pkg list | grep depthai || true

echo ""
echo "Available launch files:"
ros2 pkg executables depthai_ros_driver || true

echo ""
echo "Checking OAK permissions..."
ls /etc/udev/rules.d | grep depthai || echo "WARNING: udev rules not found"
