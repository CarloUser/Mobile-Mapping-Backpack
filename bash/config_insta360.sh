#!/bin/bash

set -e

echo "======================================"
echo "Insta360 CONFIG SETUP"
echo "======================================"

# -----------------------------
# CONFIG
# -----------------------------
WS="$HOME/ros2_ws"

DRIVER_DIR="$WS/src/insta360_ros_driver"
INCLUDE_DIR="$DRIVER_DIR/include"
LIB_DIR="$DRIVER_DIR/lib"

INSTALL_DIR="$HOME/insta360_sdk"
SDK_ROOT="$INSTALL_DIR/Linux_CameraSDK-2.1.1_MediaSDK-3.1.1"

JETSON_SDK_DIR="$SDK_ROOT/CameraSDK-20251105_112855-2.1.1-jetson-linux-9.3.0-2020.08-x86_64_aarch64_linux-gnu"

SDK_INCLUDE_DIR="$JETSON_SDK_DIR/include"
SDK_LIB_PATH="$JETSON_SDK_DIR/lib/libCameraSDK.so"

# -----------------------------
# CHECKS
# -----------------------------
if [ ! -d "$DRIVER_DIR" ]; then
    echo "ERROR: Insta360 driver not found: $DRIVER_DIR"
    exit 1
fi

if [ ! -d "$SDK_INCLUDE_DIR" ]; then
    echo "ERROR: SDK include folder not found: $SDK_INCLUDE_DIR"
    exit 1
fi

if [ ! -f "$SDK_LIB_PATH" ]; then
    echo "ERROR: libCameraSDK.so not found: $SDK_LIB_PATH"
    exit 1
fi

mkdir -p "$INCLUDE_DIR"
mkdir -p "$LIB_DIR"

# -----------------------------
# STEP 1: COPY SDK FILES
# -----------------------------
echo "[1/4] Copying SDK headers and library..."

cp -r "$SDK_INCLUDE_DIR/camera" "$INCLUDE_DIR/"
cp -r "$SDK_INCLUDE_DIR/stream" "$INCLUDE_DIR/"
cp "$SDK_LIB_PATH" "$LIB_DIR/"

echo "Copied include from:"
echo "$SDK_INCLUDE_DIR"

echo "Copied library:"
echo "$SDK_LIB_PATH"

# -----------------------------
# STEP 2: UDEV RULE
# -----------------------------
echo "[2/4] Setting up udev rule..."

echo 'SUBSYSTEM=="usb", ATTR{manufacturer}=="Arashi Vision", SYMLINK+="insta", MODE="0777"' | \
sudo tee /etc/udev/rules.d/99-insta.rules > /dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger

if [ -e /dev/insta ]; then
    sudo chmod 777 /dev/insta
else
    echo "NOTE: /dev/insta not found yet. Replug camera after setting USB mode to Android."
fi

# -----------------------------
# STEP 3: BUILD INSTA360 ONLY
# -----------------------------
echo "[3/4] Building Insta360 package..."

cd "$WS"

source /opt/ros/humble/setup.bash

colcon build \
  --packages-select insta360_ros_driver \
  --symlink-install \
  --parallel-workers 1

# -----------------------------
# STEP 4: DONE
# -----------------------------
echo "[4/4] DONE"

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