#!/bin/bash

set -e

echo "======================================"
echo "Insta360 SDK Setup"
echo "======================================"

# -----------------------------
# CONFIG
# -----------------------------
DOWNLOAD_DIR=~/Downloads
ZIP_NAME="Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip"
INSTALL_DIR=~/insta360_sdk
ZIP_PATH="$DOWNLOAD_DIR/$ZIP_NAME"

JETSON_TAR_PATTERN="CameraSDK-*jetson*.tar.gz"

echo "[1/6] Installing tools..."
sudo apt-get update
sudo apt-get install -y unzip tar

echo "[2/6] Using SDK zip from Downloads..."

if [ ! -f "$ZIP_PATH" ]; then
    echo "ZIP not found at: $ZIP_PATH"
    exit 1
fi

echo "Found: $ZIP_PATH"

echo "[3/6] Creating install directory..."
mkdir -p "$INSTALL_DIR"

echo "[4/6] Extracting main SDK zip..."
unzip -o "$ZIP_PATH" -d "$INSTALL_DIR"

echo "[5/6] Extracting Jetson CameraSDK tar.gz..."

JETSON_TAR=$(find "$INSTALL_DIR" -type f -name "$JETSON_TAR_PATTERN" | head -n 1)

if [ -z "$JETSON_TAR" ]; then
    echo "Jetson CameraSDK tar.gz not found!"
    echo "Expected pattern: $JETSON_TAR_PATTERN"
    exit 1
fi

echo "Found: $JETSON_TAR"

JETSON_DIR=$(dirname "$JETSON_TAR")
tar -xzf "$JETSON_TAR" -C "$JETSON_DIR"

echo "[6/6] Cleaning up..."
rm -f "$ZIP_PATH"

echo "======================================"
echo "Insta360 SDK setup COMPLETE"
echo "======================================"

echo "SDK location:"
echo "$INSTALL_DIR"

echo ""
echo "Check:"
echo "find ~/insta360_sdk -name 'libCameraSDK.so'"
echo "find ~/insta360_sdk -name '*.h' | head"