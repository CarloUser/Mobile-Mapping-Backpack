#!/bin/bash

set -e

echo "======================================"
echo "WIPE Insta360 SDK"
echo "======================================"

SDK_DIR=~/insta360_sdk

# -----------------------------
# STEP 1: Remove SDK folder
# -----------------------------
echo "[1/4] Removing SDK directory..."

rm -rf $SDK_DIR

# -----------------------------
# STEP 2: Remove installed libs (if copied somewhere global)
# -----------------------------
echo "[2/4] Removing global libraries (if any)..."

sudo rm -f /usr/local/lib/libCameraSDK.so
sudo rm -rf /usr/local/include/insta360

# -----------------------------
# STEP 3: Remove udev rule
# -----------------------------
echo "[3/4] Removing udev rule..."

sudo rm -f /etc/udev/rules.d/99-insta.rules

sudo udevadm control --reload-rules
sudo udevadm trigger

# -----------------------------
# STEP 4: Done
# -----------------------------
echo "[4/4] Done"

echo "======================================"
echo "Insta360 SDK CLEANED"
echo "======================================"
