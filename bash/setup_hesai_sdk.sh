#!/bin/bash

set -e

echo "======================================"
echo "Hesai Lidar SDK Setup (JT128)"
echo "======================================"

SDK_DIR=~/hesai_sdk

# -----------------------------
# STEP 1: Install dependencies
# -----------------------------
echo "[1/5] Installing dependencies..."

sudo apt update
sudo apt install -y \
    libpcl-dev \
    libpcap-dev \
    libyaml-cpp-dev \
    libssl-dev \
    cmake \
    build-essential

# -----------------------------
# STEP 2: Clone repo
# -----------------------------
echo "[2/5] Cloning SDK..."

if [ ! -d "$SDK_DIR" ]; then
    git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0.git $SDK_DIR
else
    echo "SDK already exists, skipping clone"
fi

# -----------------------------
# STEP 3: Build SDK
# -----------------------------
echo "[3/5] Building SDK..."

cd $SDK_DIR

mkdir -p build
cd build

cmake -DCMAKE_BUILD_TYPE=Release ..

make -j2   # limited cores for stability

# -----------------------------
# STEP 4: Done
# -----------------------------
echo "[4/5] Build complete"

echo "======================================"
echo "Hesai SDK READY"
echo "======================================"

echo ""
echo "Location:"
echo "$SDK_DIR/build"
echo ""
echo "You can now integrate with ROS driver."
