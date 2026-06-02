#!/bin/bash

set -e

echo "======================================"
echo "RealSense SDK (OFFICIAL CLEAN SETUP)"
echo "======================================"

# -----------------------------
# STEP 1: Prerequisites
# -----------------------------
echo "[1/6] Installing prerequisites..."

sudo apt update
sudo apt install -y \
    curl \
    gnupg \
    lsb-release \
    apt-transport-https

# -----------------------------
# STEP 2: Add GPG key
# -----------------------------
echo "[2/6] Adding RealSense key..."

sudo mkdir -p /etc/apt/keyrings

curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null

# -----------------------------
# STEP 3: Add repository
# -----------------------------
echo "[3/6] Adding repository..."

echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] \
https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update

# -----------------------------
# STEP 4: Install SDK
# -----------------------------
echo "[4/6] Installing SDK..."

sudo apt-get install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev

# Optional debug (commented)
sudo apt-get install -y librealsense2-dbg

# -----------------------------
# STEP 5: Verify
# -----------------------------
echo "[5/6] Verifying..."

if command -v realsense-viewer &> /dev/null; then
    echo "realsense-viewer installed ✔"
else
    echo "viewer not found (OK if headless)"
fi

# -----------------------------
# STEP 6: Done
# -----------------------------
echo "======================================"
echo "INSTALL COMPLETE"
echo "======================================"

echo ""
echo "Test:"
echo "realsense-viewer"
echo ""
echo "Kernel check:"
echo "modinfo uvcvideo | grep version"
