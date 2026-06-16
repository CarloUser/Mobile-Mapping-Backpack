#!/bin/bash

set -u
set -o pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
PIDS=()

source_ros() {
    source "/opt/ros/$ROS_DISTRO/setup.bash"

    if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
        source "$HOME/ros2_ws/install/setup.bash"
    fi
}

start_node() {
    local name="$1"
    shift

    echo ""
    echo "Starting $name:"
    echo "  $*"

    "$@" &
    PIDS+=("$!")
    sleep 1
}

cleanup() {
    echo ""
    echo "Stopping all sensor launches..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

source_ros

start_node "Xsens IMU" ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
start_node "GNSS" ros2 launch ublox_gps ublox_gps_node-launch.py

start_node "OAK v3 camera" ros2 launch depthai_ros_driver_v3 driver.launch.py
start_node "OAK D 1 camera" ros2 launch depthai_ros_driver camera.launch.py
start_node "RealSense camera" ros2 launch realsense2_camera rs_launch.py
start_node "Insta360 camera" ros2 launch insta360_ros_driver bringup.launch.xml

start_node "Hesai lidar" ros2 launch hesai_ros_driver start.py
start_node "Livox lidar" ros2 launch livox_ros2_avia livox_lidar_launch.py

echo ""
echo "All sensor launches started. Press Ctrl-C to stop."
wait
