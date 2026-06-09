echo "========================================="
echo "Checking OpenCV on Jetson"
echo "========================================="

if [ -f /etc/nv_tegra_release ]; then
    echo "Jetson detected"

    sudo apt-get update

    sudo apt-get install -y \
        libopencv-core-dev \
        libopencv-imgproc-dev \
        libopencv-imgcodecs-dev \
        libopencv-calib3d-dev \
        libopencv-features2d-dev \
        libopencv-highgui-dev \
        libopencv-video-dev \
        libopencv-videoio-dev

    sudo apt --fix-broken install -y

    echo "Installed OpenCV version:"
    pkg-config --modversion opencv4 || true

else
    echo "Non-Jetson system detected"

    sudo apt-get update

    sudo apt-get install -y \
        libopencv-dev \
        libopencv-contrib-dev

    echo "Installed OpenCV version:"
    pkg-config --modversion opencv4 || true
fi
