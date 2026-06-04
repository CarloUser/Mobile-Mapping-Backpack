echo "#------------------"
echo "# SDK installation "
echo "#------------------"

sudo apt-get install -y librealsense2-utils
sudo apt-get install -y librealsense2-dev
sudo apt-get install -y librealsense2-udev-rules
realsense-viewer --version
rs-enumerate-devices --version

echo "SDK installed successfully."