#!/bin/bash
set -e

ROS_DISTRO=humble
WS="$HOME/ros2_ws"

source /opt/ros/$ROS_DISTRO/setup.bash
cd "$WS"

PACKAGES=(
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
        --parallel-workers 1

    source install/local_setup.bash 2>/dev/null || true
done

echo ""
echo "All selected packages built."
