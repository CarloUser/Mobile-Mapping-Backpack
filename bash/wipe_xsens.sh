#!/bin/bash

set -e

echo "======================================"
echo "WIPE Xsens MTi Setup"
echo "======================================"

WS=~/xsens_ws

# -----------------------------
# STEP 1: Remove workspace
# -----------------------------
echo "[1/4] Removing workspace..."

rm -rf $WS

# -----------------------------
# STEP 2: Remove SDK
# -----------------------------
echo "[2/4] Removing SDK..."

sudo apt-get remove -y xsens-xme-sdk
sudo apt-get autoremove -y

# -----------------------------
# STEP 3: Clean leftovers
# -----------------------------
echo "[3/4] Cleaning leftovers..."

sudo rm -rf /usr/local/xsens
sudo rm -rf /opt/xsens

# -----------------------------
# STEP 4: Done
# -----------------------------
echo "[4/4] Done"

echo "======================================"
echo "Xsens CLEANED"
echo "======================================"
