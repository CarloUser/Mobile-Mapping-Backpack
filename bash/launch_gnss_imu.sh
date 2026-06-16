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
    echo "Stopping GNSS/IMU launches..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

source_ros

start_node "Xsens IMU" ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
start_node "GNSS" ros2 launch ublox_gps ublox_gps_node-launch.py

echo ""
echo "GNSS/IMU launches started. Press Ctrl-C to stop."
wait
