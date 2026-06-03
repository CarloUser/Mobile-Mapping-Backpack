#!/bin/bash

set -e

echo "======================================"
echo "Insta360 CONFIG SETUP"
echo "======================================"

WS=~/ros2_ws
SDK_DIR=~/insta360_sdk
DRIVER_DIR=$WS/src/insta360_ros_driver

INCLUDE_DIR=$DRIVER_DIR/include
LIB_DIR=$DRIVER_DIR/lib

# -----------------------------
# CHECKS
# -----------------------------
if [ ! -d "$DRIVER_DIR" ]; then
    echo "ERROR: Insta360 driver not found: $DRIVER_DIR"
    exit 1
fi

if [ ! -d "$SDK_DIR" ]; then
    echo "ERROR: SDK folder not found: $SDK_DIR"
    exit 1
fi

mkdir -p "$INCLUDE_DIR"
mkdir -p "$LIB_DIR"

# -----------------------------
# FIND SDK FILES
# -----------------------------
echo "[1/4] Searching SDK files..."

CAMERA_HEADERS=$(find "$SDK_DIR" -type f \( -iname "*camera*.h" -o -iname "*Camera*.h" \))
STREAM_HEADERS=$(find "$SDK_DIR" -type f \( -iname "*stream*.h" -o -iname "*Stream*.h" \))
SDK_LIB=$(find "$SDK_DIR" -type f -name "libCameraSDK.so" | head -n 1)

if [ -z "$SDK_LIB" ]; then
    echo "ERROR: libCameraSDK.so not found in $SDK_DIR"
    exit 1
fi

# -----------------------------
# COPY FILES
# -----------------------------
echo "[2/4] Copying headers and library..."

cp $CAMERA_HEADERS "$INCLUDE_DIR/" 2>/dev/null || true
cp $STREAM_HEADERS "$INCLUDE_DIR/" 2>/dev/null || true
cp "$SDK_LIB" "$LIB_DIR/"

echo "Copied SDK library:"
echo "$SDK_LIB"

# -----------------------------
# UDEV RULE
# -----------------------------
echo "[3/4] Setting up udev rule..."

echo 'SUBSYSTEM=="usb", ATTR{manufacturer}=="Arashi Vision", SYMLINK+="insta", MODE="0777"' | \
sudo tee /etc/udev/rules.d/99-insta.rules > /dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger

# chmod only if device exists
if [ -e /dev/insta ]; then
    sudo chmod 777 /dev/insta
else
    echo "NOTE: /dev/insta not found yet. Replug camera after setting USB mode to Android."
fi

# -----------------------------
# BUILD INSTA360 ONLY
# -----------------------------
echo "[4/4] Building Insta360 package..."

cd "$WS"

colcon build \
  --packages-select insta360_ros_driver \
  --symlink-install \
  --parallel-workers 1

echo "======================================"
echo "Insta360 CONFIG COMPLETE"
echo "======================================"

echo ""
echo "Check:"
echo "ls $INCLUDE_DIR"
echo "ls $LIB_DIR"
echo ""
echo "Run:"
echo "source /opt/ros/humble/setup.bash"
echo "source $WS/install/setup.bash"