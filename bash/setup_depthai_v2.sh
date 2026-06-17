#!/bin/bash
set -e

echo "=============================="
echo "DepthAI ROS2 v2 Setup Script"
echo "=============================="

ROS_DISTRO=humble
WS="$HOME/ros2_ws"

echo "[1/7] Source clean ROS environment..."
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source "/opt/ros/$ROS_DISTRO/setup.bash"

echo "[2/7] Install dependencies..."
sudo apt update
sudo apt install -y \
  git \
  cmake \
  build-essential \
  python3-rosdep \
  libpcl-dev \
  libusb-1.0-0-dev \
  ros-$ROS_DISTRO-diagnostic-updater \
  ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-image-transport \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-message-filters \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-tf2-geometry-msgs \
  ros-$ROS_DISTRO-rviz2

echo "[3/7] Create workspace..."
mkdir -p "$WS/src"
cd "$WS/src"

echo "[4/7] Clone repositories..."
if [ ! -d depthai-ros ]; then
  git clone -b humble https://github.com/luxonis/depthai-ros.git
else
  echo "depthai-ros already exists"
fi

if [ ! -d depthai-core ]; then
  git clone -b v2_stable https://github.com/luxonis/depthai-core.git
else
  echo "depthai-core already exists"
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

make -j1
make install DESTDIR="$WS/depthai_install"

DEPTHAI_DIR="$WS/depthai_install$WS/src/depthai-core/build/install/lib/cmake/depthai"

echo "DepthAI CMake dir:"
echo "$DEPTHAI_DIR"

if [ ! -f "$DEPTHAI_DIR/depthaiConfig.cmake" ]; then
  echo "ERROR: depthaiConfig.cmake not found at:"
  echo "$DEPTHAI_DIR"
  exit 1
fi

echo "[6/7] Install ROS deps..."
cd "$WS"
rosdep update
rosdep install \
  --from-paths src \
  --ignore-src \
  --skip-keys "depthai depthai_examples depthai_filters" \
  -r -y

echo "[7/7] Build depthai-ros v2..."
rm -rf build install log

colcon build \
  --symlink-install \
  --parallel-workers 1 \
  --packages-select \
    depthai_ros_msgs \
    depthai_descriptions \
    depthai_bridge \
    depthai_ros_driver \
    depthai_ros \
  --packages-ignore \
    depthai \
    depthai_examples \
    depthai_filters \
  --cmake-args -Ddepthai_DIR="$DEPTHAI_DIR"

echo "=============================="
echo "DepthAI ROS2 v2 DONE"
echo "=============================="

echo "Source workspace:"
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
echo "source $WS/install/setup.bash"

echo ""
echo "Verifying installation..."
source "$WS/install/setup.bash"

ros2 pkg list | grep depthai || true

echo ""
echo "Available launch/executables:"
ros2 pkg executables depthai_ros_driver || true

echo ""
echo "Checking OAK permissions..."
ls /etc/udev/rules.d | grep -i depthai || echo "WARNING: depthai udev rules not found"