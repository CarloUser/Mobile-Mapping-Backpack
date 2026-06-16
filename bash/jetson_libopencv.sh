#!/bin/bash
set -e

echo "========================================="
echo "Jetson NVIDIA OpenCV Setup"
echo "========================================="

if [ ! -f /etc/nv_tegra_release ]; then
    echo "ERROR: This script is only for NVIDIA Jetson."
    exit 1
fi

cat /etc/nv_tegra_release

echo "Removing existing OpenCV packages..."
sudo apt remove --purge -y 'libopencv*' 'opencv-data' 'opencv-licenses' 'python3-opencv' || true
sudo apt autoremove -y
sudo apt --fix-broken install -y

echo "Removing leftover OpenCV config files..."
sudo rm -rf /usr/lib/cmake/opencv4
sudo rm -f /usr/lib/pkgconfig/opencv4.pc

echo "Installing NVIDIA OpenCV..."
sudo apt update
sudo apt install -y 'nvidia-opencv*'
sudo ldconfig

echo "Verification:"
dpkg -l | grep -E 'opencv|nvidia-opencv' || true
pkg-config --modversion opencv4 || true
find /usr -name "libopencv_core.so*" 2>/dev/null

echo "NVIDIA OpenCV setup done."