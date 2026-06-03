#!/bin/bash

set -e

echo "======================================"
echo "Insta360 SDK Setup"
echo "======================================"

# -----------------------------
# CONFIG
# -----------------------------
DOWNLOAD_DIR=~/Downloads
ZIP_NAME="Linux_CameraSDK-2.1.1_MediaSDK-3.1.1.zip"   # change if needed
INSTALL_DIR=~/insta360_sdk
ZIP_PATH="$DOWNLOAD_DIR/$ZIP_NAME"

# -----------------------------
# STEP 1: Install tools
# -----------------------------
echo "[1/5] Installing tools..."

sudo apt-get update
sudo apt-get install -y unzip

# -----------------------------
# STEP 2: Use existing ZIP
# -----------------------------
echo "[2/5] Using SDK from Downloads..."

if [ ! -f "$ZIP_PATH" ]; then
    echo "ZIP not found at: $ZIP_PATH"
    echo "Please place your SDK zip in ~/Downloads"
    exit 1
fi

echo "✔ Found: $ZIP_PATH"

# -----------------------------
# STEP 3: Create install dir
# -----------------------------
echo "[3/5] Creating install directory..."

mkdir -p "$INSTALL_DIR"

# -----------------------------
# STEP 4: Extract SDK
# -----------------------------
echo "[4/5] Extracting SDK..."

unzip -o "$ZIP_PATH" -d "$INSTALL_DIR"

# -----------------------------
# STEP 5: Cleanup
# -----------------------------
echo "[5/5] Cleaning up..."

rm -f "$ZIP_PATH"
echo "✔ ZIP removed from Downloads"

echo "======================================"
echo "Insta360 SDK setup COMPLETE"
echo "======================================"

echo ""
echo "Location:"
echo "$INSTALL_DIR">
