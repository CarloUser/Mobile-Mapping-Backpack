#!/bin/bash

set -e

echo "======================================"
echo "FULL ROS2 + DEPTHAI CLEANUP"
echo "======================================"

# -----------------------------
# STEP 1: Remove ROS2
# -----------------------------
echo "[1/7] Removing ROS2..."

sudo apt remove -y 'ros-*'
sudo apt remove -y ros2-testing-apt-source
sudo apt autoremove -y
sudo apt clean

# -----------------------------
# STEP 2: Remove DepthAI APT packages
# -----------------------------
echo "[2/7] Removing DepthAI packages..."

sudo apt remove -y 'ros-humble-depthai*'
sudo apt autoremove -y

# -----------------------------
# STEP 3: Remove ROS config
# -----------------------------
echo "[3/7] Removing ROS configs..."

rm -rf ~/.ros

# -----------------------------
# STEP 4: Remove workspaces
# -----------------------------
echo "[4/7] Removing workspaces..."

rm -rf ~/ros_ws
rm -rf ~/livox_ws

# -----------------------------
# STEP 5: Remove DepthAI core (global install)
# -----------------------------
echo "[5/7] Removing depthai-core from /usr/local..."

sudo rm -rf /usr/local/include/depthai
sudo rm -rf /usr/local/lib/libdepthai*
sudo rm -rf /usr/local/share/depthai*

# -----------------------------
# STEP 6: Remove udev rules
# -----------------------------
echo "[6/7] Removing udev rules..."

sudo rm -f /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# -----------------------------
# STEP 7: Clean bashrc
# -----------------------------
echo "[7/7] Cleaning bashrc..."

sed -i '/ros\/humble/d' ~/.bashrc
sed -i '/ros_ws\/install/d' ~/.bashrc
sed -i '/livox_ws\/install/d' ~/.bashrc

echo "======================================"
echo "CLEANUP COMPLETE"
echo "======================================"

echo ""
echo "You now have a CLEAN system."
