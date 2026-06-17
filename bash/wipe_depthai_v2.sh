#!/bin/bash
set -e

echo "=============================="
echo "DepthAI v2 Wipe Script"
echo "=============================="

WS="$HOME/ros2_ws"

echo "[1/5] Remove workspace build artifacts..."
rm -rf "$WS/build"
rm -rf "$WS/install"
rm -rf "$WS/log"

echo "[2/5] Remove depthai install..."
rm -rf "$WS/depthai_install"

echo "[3/5] Remove source repositories..."
rm -rf "$WS/src/depthai-core"
rm -rf "$WS/src/depthai-ros"

echo "[4/5] Clear environment..."
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

echo "[5/5] Verify cleanup..."

echo
echo "Remaining DepthAI folders:"
find "$WS/src" -maxdepth 1 -iname "*depthai*" 2>/dev/null || true

echo
echo "Workspace status:"
ls -la "$WS" 2>/dev/null || true

echo
echo "=============================="
echo "DepthAI v2 wiped successfully"
echo "=============================="
echo
echo "ROS2 Humble was NOT removed."
echo "You can now run setup_depthai_v2.sh again."