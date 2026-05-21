#!/bin/bash

set -e

echo "======================================"
echo "Insta360 SDK Setup"
echo "======================================"

# -----------------------------
# CONFIG
# -----------------------------
SDK_URL="https://www.insta360.com/sdk/record?auto_jump=true&_gl=1*jqebgd*_up*MQ..*_ga*MTA4MTU2ODQxMi4xNzc5MzU4NzIz*_ga_ZMDD1VX0M1*czE3NzkzNTg3MjIkbzEkZzAkdDE3NzkzNTg3MjIkajYwJGwwJGgw*_ga_7TV2BE92TS*czE3NzkzNTg3MjMkbzEkZzAkdDE3NzkzNTg3MjMkajYwJGwwJGgw"   # <-- OPTIONAL: put direct download link here
ZIP_NAME="insta360_sdk.zip"
INSTALL_DIR=~/insta360_sdk

# -----------------------------
# STEP 1: Install tools
# -----------------------------
echo "[1/5] Installing tools..."

sudo apt-get update
sudo apt-get install -y unzip wget

# -----------------------------
# STEP 2: Download (optional)
# -----------------------------
echo "[2/5] Downloading SDK..."

if [ -n "$SDK_URL" ]; then
    wget -O $ZIP_NAME "$SDK_URL"
else
    echo "⚠ No URL provided."
    echo "Please place your SDK zip as: $ZIP_NAME"
    read -p "Press ENTER to continue..."
fi

# -----------------------------
# STEP 3: Create install dir
# -----------------------------
echo "[3/5] Creating install directory..."

mkdir -p $INSTALL_DIR

# -----------------------------
# STEP 4: Unzip
# -----------------------------
echo "[4/5] Extracting SDK..."

unzip -o $ZIP_NAME -d $INSTALL_DIR

# -----------------------------
# STEP 5: Cleanup
# -----------------------------
echo "[5/5] Cleaning up..."

rm -f $ZIP_NAME

echo "======================================"
echo "Insta360 SDK setup COMPLETE"
echo "======================================"

echo ""
echo "Location:"
echo "$INSTALL_DIR"
