#!/bin/bash

set -e

echo "======================================"
echo "Livox CONFIG SETUP"
echo "======================================"

CONFIG_FILE=~/ros2_ws/src/livox_ros2_avia/config/livox_lidar_config.json

# -----------------------------
# CHECK FILE
# -----------------------------
if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: Config file not found!"
    exit 1
fi

echo "[1/3] Editing Livox config..."

# -----------------------------
# USER INPUT
# -----------------------------
read -p "Enter Livox broadcast code (e.g. 3JEDLB1001276511): " CODE

# -----------------------------
# APPLY CHANGES
# -----------------------------
sed -i "s/\"broadcast_code\": \".*\"/\"broadcast_code\": \"$CODE\"/" $CONFIG_FILE
sed -i "s/\"enable_connect\": false/\"enable_connect\": true/" $CONFIG_FILE

echo "[2/3] Config updated"

# -----------------------------
# SHOW RESULT
# -----------------------------
echo "[3/3] Final config:"
grep -E "broadcast_code|enable_connect" $CONFIG_FILE

echo "======================================"
echo "DONE"
echo "======================================"

echo ""
echo "Next steps:"
echo "--------------------------------------"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/ros2_ws/install/setup.bash"
echo "ros2 launch livox_ros2_avia livox_lidar_launch.py"
