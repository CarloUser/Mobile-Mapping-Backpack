#!/bin/bash

set -e#!/bin/bash

set -e

echo "======================================"
echo "GNSS CONFIG (zed_f9p.yaml COMPLETE)"
echo "======================================"

WS=~/ros2_ws
YAML_FILE=$WS/src/ublox/ublox_gps/config/zed_f9p.yaml
LAUNCH_FILE=$WS/src/ublox/ublox_gps/launch/ublox_gps_node-launch.py

# -----------------------------
# STEP 1: Check files
# -----------------------------
if [ ! -f "$YAML_FILE" ]; then
	    echo "YAML file not found"
	        exit 1
fi

if [ ! -f "$LAUNCH_FILE" ]; then
	    echo "Launch file not found"
	        exit 1
fi

# -----------------------------
# STEP 2: Fix YAML
# -----------------------------
echo "[1/5] Fixing YAML..."

# Add config_on_startup under frame_id (with 4 spaces)
sed -i '/frame_id: gps/a\    config_on_startup: false' "$YAML_FILE"

# Remove TMODE3 block completely
sed -i '/# TMODE3 Config/,/acc_lim:/d' "$YAML_FILE"

# Set device path (optional but recommended)
if grep -q "device:" "$YAML_FILE"; then
	    sed -i 's|device:.*|device: /dev/tty_Ardusimple|' "$YAML_FILE"
    else
	        echo "device: /dev/tty_Ardusimple" >> "$YAML_FILE"
fi

# -----------------------------
# STEP 3: Fix launch file
# -----------------------------
echo "[2/5] Fixing launch file..."

sed -i 's/c94_m8p_rover.yaml/zed_f9p.yaml/g' "$LAUNCH_FILE"

# -----------------------------
# STEP 4: Setup udev rule
# -----------------------------
echo "[3/5] Setting up udev rule..."

RULE_FILE="/etc/udev/rules.d/50-ardusimple.rules"

echo 'KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK="tty_Ardusimple", GROUP="dialout", MODE="0666"' | \
sudo tee $RULE_FILE > /dev/null

# FULL reload (recommended)
sudo service udev reload
sudo service udev restart
sudo udevadm trigger

echo "✔ udev rule created → /dev/tty_Ardusimple"

# STEP 5: Rebuild
# -----------------------------
echo "[4/5] Rebuilding..."

cd $WS
colcon build --symlink-install --parallel-workers 2

# -----------------------------
# DONE
# -----------------------------
echo "[5/5] DONE"

echo "======================================"
echo "GNSS CONFIG COMPLETE"
echo "======================================"

echo ""
echo "⚠ IMPORTANT:"
echo "Unplug and reconnect the GNSS device!"
echo ""

echo "Test with:"
echo "ls /dev/tty_Ardusimple"

echo ""
echo "Run:"
echo "source /opt/ros/humble/setup.bash"
echo "source $WS/install/setup.bash"
echo "ros2 launch ublox_gps ublox_gps_node.launch.py"

echo "======================================"
echo "GNSS CONFIG (zed_f9p.yaml COMPLETE)"
echo "======================================"

WS=~/ros2_ws
YAML_FILE=$WS/src/ublox/ublox_gps/config/zed_f9p.yaml
LAUNCH_FILE=$WS/src/ublox/ublox_gps/launch/ublox_gps_node-launch.py
