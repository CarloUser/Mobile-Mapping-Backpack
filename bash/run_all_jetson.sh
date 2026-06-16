#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================="
echo "Mobile Mapping Backpack - Jetson Setup"
echo "========================================="

SCRIPTS=(
    "jetson_libopencv.sh"

    "setup_ros2_humble.sh"

    "setup_depthai_v2.sh"
    "setup_depthai_v3_auto.sh"

    "setup_realsense_sdk_jetson.sh"
    "setup_realsense_binary.sh"

    "setup_insta360_sdk_jetson.sh"
    "setup_insta360.sh"
    "config_insta360_jetson.sh"

    "setup_livox.sh"
    "setup_livox_config.sh"

    "setup_hesai_sdk.sh"
    "setup_hesai.sh"
    "config_hesai.sh"

    "setup_xsens.sh"

    "setup_gnss.sh"
    "config_gnss.sh"

    "build_sensors.sh"
)

for script in "${SCRIPTS[@]}"; do
    path="$SCRIPT_DIR/$script"

    if [ ! -f "$path" ]; then
        echo ""
        echo "WARNING: Missing script, skipping: $script"
        continue
    fi

    echo ""
    echo "========================================="
    echo "Running: $script"
    echo "========================================="

    chmod +x "$path"
    bash "$path"
done

echo ""
echo "========================================="
echo "Jetson setup finished."
echo "========================================="
