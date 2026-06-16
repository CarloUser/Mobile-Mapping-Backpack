#!/bin/bash

set -e  # stop on error

echo "=============================="
echo "ROS2 Humble Full Setup Script"
echo "=============================="

apt_repair() {
    echo "Repairing apt/dpkg state..."
    sudo dpkg --configure -a
    sudo apt --fix-broken install -y
    sudo apt update
}

install_apt_with_retry() {
    local label="$1"
    shift

    echo "$label"
    if sudo apt install -y "$@"; then
        return 0
    fi

    echo "Initial apt install failed. Trying apt repair and retry..."
    apt_repair
    sudo apt install -y "$@"
}

install_ros_desktop() {
    if sudo apt install -y ros-humble-desktop; then
        return 0
    fi

    echo "ROS desktop install failed. Trying to fix Jammy dependency version drift..."
    apt_repair

    if . /etc/os-release && [ "${VERSION_CODENAME:-}" = "jammy" ]; then
        echo "Preinstalling Jammy dev packages that commonly mismatch on Jetson..."
        sudo apt install -y -t jammy-updates \
            libsdl2-dev \
            libusb-1.0-0-dev \
            qtbase5-dev || true
    fi

    echo "Upgrading installed packages so runtime and -dev versions match..."
    sudo apt full-upgrade -y
    apt_repair

    sudo apt install -y ros-humble-desktop
}

enable_jammy_updates_repo() {
    if ! . /etc/os-release || [ "${VERSION_CODENAME:-}" != "jammy" ]; then
        return 0
    fi

    if grep -RqsE "^[[:space:]]*deb .* jammy-updates " /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
        echo "jammy-updates repository already enabled."
        return 0
    fi

    echo "Enabling jammy-updates repository for matching Ubuntu runtime/dev packages..."

    local ubuntu_repo="http://archive.ubuntu.com/ubuntu"
    if [ "$(dpkg --print-architecture)" = "arm64" ]; then
        ubuntu_repo="http://ports.ubuntu.com/ubuntu-ports"
    fi

    echo "deb ${ubuntu_repo} jammy-updates main restricted universe multiverse" | \
        sudo tee /etc/apt/sources.list.d/jammy-updates.list > /dev/null
}

# -----------------------------
# STEP 1: Locale (recommended)
# -----------------------------
echo "[1/8] Setting locale..."

sudo apt update
install_apt_with_retry "Installing locale support..." locales

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

echo "Locale set to:"
locale

# -----------------------------
# STEP 2: Enable universe repo
# -----------------------------
echo "[2/8] Enabling Ubuntu universe repository..."

install_apt_with_retry "Installing repository tools..." software-properties-common
sudo add-apt-repository -y universe
enable_jammy_updates_repo

# -----------------------------
# STEP 3: Add ROS2 apt source
# -----------------------------
echo "[3/8] Adding ROS2 repository..."

sudo apt update
install_apt_with_retry "Installing curl..." curl

ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
"https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb

# -----------------------------
# STEP 4: Install ROS2 Humble
# -----------------------------
echo "[4/8] Installing ROS2 Humble..."

sudo apt update
apt_repair
install_ros_desktop

# -----------------------------
# STEP 5: Setup ROS environment
# -----------------------------
echo "[5/8] Setting up ROS environment..."

if ! grep -q "/opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

source /opt/ros/humble/setup.bash

# -----------------------------
# STEP 6: Install dev tools
# -----------------------------
echo "[6/8] Installing development tools..."

install_apt_with_retry "Installing ROS development tools..." ros-dev-tools

# -----------------------------
# STEP 7: Initialize rosdep
# -----------------------------
echo "[7/8] Initializing rosdep..."

sudo rosdep init || true
rosdep update

# -----------------------------
# STEP 8: Create workspace
# -----------------------------
echo "[8/8] Creating ROS2 workspace..."

mkdir -p ~/ros_ws/src
cd ~/ros_ws

colcon build

source ~/ros_ws/install/setup.bash

echo "=============================="
echo "SETUP COMPLETE"
echo "=============================="
echo "Run: source ~/.bashrc"
echo "Then test with: ros2"
