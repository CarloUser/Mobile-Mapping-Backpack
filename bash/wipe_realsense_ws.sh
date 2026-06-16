#!/bin/bash

set -e

echo "======================================"
echo "WIPE RealSense ROS2 Workspace"
echo "======================================"

WS=~/realsense_ws

# -----------------------------
# STEP 1: Remove workspace
# -----------------------------
echo "[1/4] Removing workspace..."

rm -rf $WS

# -----------------------------
# STEP 2: Clean rosdep cache (optional)
# -----------------------------
echo "[2/4] Cleaning rosdep cache..."

rm -rf ~/.ros/rosdep

# -----------------------------
# STEP 3: Clean build leftovers (safety)
# -----------------------------
echo "[3/4] Removing possible leftover logs..."

rm -rf ~/log

# -----------------------------
# STEP 4: Done
# -----------------------------
echo "[4/4] Done"

echo "======================================"
echo "RealSense ROS workspace CLEANED"
echo "======================================"

echo ""
echo "NOTE:"
echo "--------------------------------------"
echo "SDK (librealsense2) is NOT removed."
echo "Use wipe_realsense_sdk.sh if needed."
