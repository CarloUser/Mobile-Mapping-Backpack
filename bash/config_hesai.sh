#!/bin/bash

set -e

echo "======================================"
echo "Hesai Config Setup"
echo "======================================"

WS=~/ros2_ws
CONFIG_DIR="$WS/src/HesaiLidar_ROS_2.0/config"
CONFIG_FILE="$CONFIG_DIR/config.yaml"

# CSV file (located where this script is)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CSV_NAME="JT128_default_angle.csv"
CSV_SOURCE="$SCRIPT_DIR/$CSV_NAME"
CSV_TARGET="$CONFIG_DIR/$CSV_NAME"

# -----------------------------
# STEP 1: Check files
# -----------------------------
echo "[1/4] Checking files..."

if [ ! -f "$CSV_SOURCE" ]; then
    echo "CSV file not found: $CSV_SOURCE"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "config.yaml not found"
    exit 1
fi

# -----------------------------
# STEP 2: Copy CSV
# -----------------------------
echo "[2/4] Copying CSV to config folder..."

cp "$CSV_SOURCE" "$CSV_TARGET"

# -----------------------------
# STEP 3: Update config.yaml
# -----------------------------
echo "[3/4] Updating correction_file_path..."

# replace ALL occurrences
sed -i "s|correction_file_path:.*|correction_file_path: \"$CSV_TARGET\"|g" "$CONFIG_FILE"

# -----------------------------
# STEP 4: Done
# -----------------------------
echo "[4/4] Done"

echo "======================================"
echo "Hesai config COMPLETE"
echo "======================================"

echo ""
echo "Updated path:"
echo "$CSV_TARGET"