#!/bin/bash

set -e

echo "======================================"
echo "DepthAI ROS v2 (Inside ros_ws - FINAL FIXED)"
echo "======================================"

# -----------------------------
# STEP 1: Source ROS2
# -----------------------------
echo "[1/6] Sourcing ROS2..."
source /opt/ros/humble/setup.bash

# -----------------------------
# STEP 2: Go to workspace
# -----------------------------
echo "[2/6] Using existing workspace ~/ros_ws..."
cd ~/ros_ws/src

# -----------------------------
# STEP 3: Clone depthai-ros (v2)
# -----------------------------
echo "[3/6] Cloning depthai-ros (v2)..."
if [ ! -d "depthai-ros" ]; then
    git clone --branch humble https://github.com/luxonis/depthai-ros.git
else
    echo "depthai-ros already exists, skipping clone"
fi

# -----------------------------
# STEP 4: Clone depthai-core WITH submodules (CRITICAL)
# -----------------------------
echo "[4/6] Cloning depthai-core (with submodules)..."
if [ ! -d "depthai-core" ]; then
    git clone --recurse-submodules https://github.com/luxonis/depthai-core.git
else
    echo "depthai-core already exists, updating submodules..."
    cd depthai-core
    git submodule update --init --recursive
    cd ..
fi

# -----------------------------
# STEP 5: Install dependencies
# -----------------------------

echo "[5/6] Installing tracked DepthAI system dependencies..."

sudo apt update
sudo apt install -y $(cat deps_depthai.txt)

echo "[5/6] Installing dependencies (rosdep)..."

cd ~/ros_ws

rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="depthai"

# -----------------------------
# STEP 6: Build workspace
# -----------------------------

echo "[6/6] Building workspace (this will take time)..."

MAKEFLAGS="-j4 -l4" colcon build \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DVCPKG_FEATURE_FLAGS=manifests
echo "======================================"
echo "DepthAI v2 setup COMPLETE"
echo "======================================"

echo ""
echo "IMPORTANT USAGE:"
echo "--------------------------------------"
echo "Default terminal → uses v3 (APT)"
echo ""
echo "To use v2:"
echo "  source ~/ros_ws/install/setup.bash"
echo ""
echo "WARNING:"
echo "Do NOT add ros_ws to ~/.bashrc!"
