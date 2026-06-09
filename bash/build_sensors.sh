#!/bin/bash
set -e

ROS_DISTRO=humble
WS="$HOME/ros2_ws"

source /opt/ros/$ROS_DISTRO/setup.bash
cd "$WS"

# DepthAI v2 path
export depthai_DIR="$WS/depthai_install$WS/src/depthai-core/build/install/lib/cmake/depthai"

PACKAGES=(
    depthai
    depthai_ros_msgs
    depthai_descriptions
    depthai_bridge
    # depthai_filters      # skipped on Jetson: OpenCV ximgproc issue
    depthai_ros_driver
    depthai_ros
    # depthai_examples     # skipped on Jetson: heavy/RAM issue

    insta360_ros_driver

    hesai_ros_driver

    livox_interfaces
    livox_sdk_vendor
    livox_ros2_avia

    ublox_serialization
    ublox_msgs
    ublox_gps
    ublox

    ntrip

    xsens_mti_ros2_driver

    mmb_bringup
)

for pkg in "${PACKAGES[@]}"; do
    echo ""
    echo "===================================="
    echo "Building: $pkg"
    echo "===================================="

    colcon build \
        --packages-select "$pkg" \
        --symlink-install \
        --parallel-workers 1 \
        --cmake-args -Ddepthai_DIR="$depthai_DIR"

    source install/local_setup.bash 2>/dev/null || true
done

echo ""
echo "All selected packages built."
