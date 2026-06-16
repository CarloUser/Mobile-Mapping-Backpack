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
    echo "Stopping camera launches..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

source_ros

start_node "OAK v3 camera" ros2 launch depthai_ros_driver_v3 driver.launch.py
start_node "OAK D 1 camera" ros2 launch depthai_ros_driver camera.launch.py
start_node "RealSense camera" ros2 launch realsense2_camera rs_launch.py
start_node "Insta360 camera" ros2 launch insta360_ros_driver bringup.launch.xml

echo ""
echo "Camera launches started. Press Ctrl-C to stop."
wait
