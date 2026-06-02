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
# OVERWRITE lidar_config
# -----------------------------

python3 <<EOF
import json

cfg_file = "$CONFIG_FILE"

with open(cfg_file, "r") as f:
    data = json.load(f)

data["lidar_config"] = [
    {
        "broadcast_code": "3JEDLB100127651",
        "enable_connect": True,
        "enable_fan": True,
        "return_mode": 0,
        "coordinate": 0,
        "imu_rate": 1,
        "extrinsic_parameter_source": 0
    },
    {
        "broadcast_code": "3JEDLCL0016V731",
        "enable_connect": True,
        "enable_fan": True,
        "return_mode": 0,
        "coordinate": 0,
        "imu_rate": 1,
        "extrinsic_parameter_source": 0
    }
]

with open(cfg_file, "w") as f:
    json.dump(data, f, indent=4)

print("Livox config updated.")
EOF

echo "[2/3] Config updated"

# -----------------------------
# SHOW RESULT
# -----------------------------
echo "[3/3] Final config:"
grep -E "broadcast_code|enable_connect" "$CONFIG_FILE"

echo "======================================"
echo "DONE"
echo "======================================"

echo ""
echo "Next steps:"
echo "--------------------------------------"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/ros2_ws/install/setup.bash"
echo "ros2 launch livox_ros2_avia livox_lidar_launch.py"
