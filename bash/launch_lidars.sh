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
    echo "Stopping lidar launches..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

source_ros

start_node "Hesai lidar" ros2 launch hesai_ros_driver start.py
start_node "Livox lidar" ros2 launch livox_ros2_avia livox_lidar_launch.py

echo ""
echo "Lidar launches started. Press Ctrl-C to stop."
wait
