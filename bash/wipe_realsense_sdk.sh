#!/bin/bash

set -e

echo "======================================"
echo "WIPE RealSense SDK (librealsense2)"
echo "======================================"

# -----------------------------
# STEP 1: Remove SDK packages
# -----------------------------
echo "[1/5] Removing librealsense packages..."

sudo apt-get remove -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

sudo apt-get autoremove -y

# -----------------------------
# STEP 2: Remove repository
# -----------------------------
echo "[2/5] Removing repository..."

sudo rm -f /etc/apt/sources.list.d/librealsense.list

# -----------------------------
# STEP 3: Remove GPG key
# -----------------------------
echo "[3/5] Removing key..."

sudo rm -f /etc/apt/keyrings/librealsenseai.gpg

# -----------------------------
# STEP 4: Clean apt
# -----------------------------
echo "[4/5] Cleaning apt..."

sudo apt-get update

# -----------------------------
# STEP 5: Optional udev cleanup
# -----------------------------
echo "[5/5] Reloading udev rules..."

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "======================================"
echo "RealSense SDK CLEANUP COMPLETE"
echo "======================================"

echo ""
echo "System is clean from librealsense."
