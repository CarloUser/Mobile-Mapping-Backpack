#!/bin/bash

set -euo pipefail

echo "========================================"
echo "DepthAI ROS2 v3 Automated Setup Script"
echo "========================================"

# -----------------------------
# SETTINGS
# -----------------------------
ROS_DISTRO="${ROS_DISTRO:-humble}"
DEPTHAI_V2_WS="${DEPTHAI_V2_WS:-$HOME/depthai_v2_ws}"
INSTALL_RVIZ="${INSTALL_RVIZ:-false}"

V3_META_PKG="ros-${ROS_DISTRO}-depthai-ros-v3"
ROS2CLI_PKG="ros-${ROS_DISTRO}-ros2cli"

# -----------------------------
# HELPERS
# -----------------------------
have_cmd() {
    command -v "$1" >/dev/null 2>&1
}

pkg_installed() {
    dpkg -s "$1" >/dev/null 2>&1
}

step() {
    echo ""
    echo "[$1] $2"
}

# -----------------------------
# STEP 1: Basic checks
# -----------------------------
step "1/9" "Checking ROS installation..."

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found."
    echo "Install ROS2 ${ROS_DISTRO} first."
    exit 1
fi

# shellcheck disable=SC1090
set +u  # ROS setup.bash reads unbound vars (AMENT_TRACE_SETUP_FILES) under set -u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

echo "ROS_DISTRO = ${ROS_DISTRO}"

# -----------------------------
# STEP 2: Make sure ros2 CLI exists
# -----------------------------
step "2/9" "Checking ros2 command..."

if ! have_cmd ros2; then
    echo "ros2 command not found. Installing ${ROS2CLI_PKG}..."
    sudo apt update
    sudo apt install -y "${ROS2CLI_PKG}"
    # shellcheck disable=SC1090
    set +u  # ROS setup.bash reads unbound vars (AMENT_TRACE_SETUP_FILES) under set -u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
fi

if ! have_cmd ros2; then
    echo "ERROR: ros2 command still not found after installing ${ROS2CLI_PKG}."
    exit 1
fi

echo "ros2 found: $(command -v ros2)"

# -----------------------------
# STEP 3: Fix broken apt/dpkg state
# -----------------------------
step "3/9" "Fixing broken apt/dpkg state if needed..."

# Work around an appstreamcli SIGSEGV in APT's update hook: the DEP-11 cache
# refresh in /etc/apt/apt.conf.d/50appstream crashes on this Ubuntu/Jetson and
# aborts `apt update`. Disabling that one hook only skips GUI software-catalog
# metadata; package installs are unaffected. APT ignores *.disabled files, and
# it is fully reversible (mv back). No-op if already disabled or absent.
if [ -f /etc/apt/apt.conf.d/50appstream ]; then
    echo "Disabling crashing appstream APT update hook (50appstream -> .disabled)"
    sudo mv /etc/apt/apt.conf.d/50appstream /etc/apt/apt.conf.d/50appstream.disabled
fi

sudo dpkg --configure -a
sudo apt --fix-broken install -y

# -----------------------------
# STEP 4: Install ROS2 testing apt source
# -----------------------------
step "4/9" "Installing ROS2 testing apt source..."

sudo apt update
sudo apt install -y ros2-testing-apt-source
sudo apt update

# -----------------------------
# STEP 5: Remove conflicting plain v2 apt packages
# -----------------------------
step "5/9" "Removing conflicting plain DepthAI v2 apt packages..."

# Important:
# ros-humble-depthai installs /opt/ros/humble/include/depthai/*
# and conflicts with ros-humble-depthai-v3.
# We remove only plain non-v3 apt packages, not your source-built v2 workspace.
PLAIN_DEPTHAI_PKGS=$(dpkg -l | awk -v distro="$ROS_DISTRO" '
    $1 == "ii" &&
    $2 ~ "^ros-"distro"-depthai" &&
    $2 !~ "-v3$" &&
    $2 !~ "-v3-" {
        print $2
    }
')

if [ -n "${PLAIN_DEPTHAI_PKGS}" ]; then
    echo "Removing:"
    echo "${PLAIN_DEPTHAI_PKGS}"
    sudo apt remove -y ${PLAIN_DEPTHAI_PKGS}
    sudo apt --fix-broken install -y
else
    echo "No conflicting plain DepthAI v2 apt packages found."
fi

# -----------------------------
# STEP 6: Install DepthAI ROS v3
# -----------------------------
step "6/9" "Installing DepthAI ROS v3..."

sudo apt install -y "${V3_META_PKG}"

# -----------------------------
# STEP 7: Install udev rules for OAK / Myriad X devices
# -----------------------------
step "7/9" "Installing DepthAI udev rules..."

sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2485", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="f63b", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2150", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="f63b", MODE="0666", GROUP="plugdev"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

if getent group plugdev >/dev/null; then
    sudo usermod -aG plugdev "$USER" || true
else
    echo "NOTE: plugdev group not found. udev rule still uses MODE=0666."
fi

echo "NOTE: Unplug/replug OAK cameras after this script."
echo "NOTE: If permissions still fail, log out/in or reboot."

# -----------------------------
# STEP 8: Optional RViz install
# -----------------------------
step "8/9" "Checking optional RViz install..."

if [ "${INSTALL_RVIZ}" = "true" ]; then
    sudo apt install -y "ros-${ROS_DISTRO}-rviz2"
else
    echo "Skipping RViz install. To install it, run:"
    echo "INSTALL_RVIZ=true ./setup_depthai_v3_auto.sh"
fi

# -----------------------------
# STEP 9: Verification
# -----------------------------
step "9/9" "Verifying DepthAI v3 installation..."

# shellcheck disable=SC1090
set +u  # ROS setup.bash reads unbound vars (AMENT_TRACE_SETUP_FILES) under set -u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

echo ""
echo "DepthAI-related apt packages:"
dpkg -l | grep "ros-${ROS_DISTRO}-depthai" || true

echo ""
echo "DepthAI ROS packages:"
ros2 pkg list | grep depthai || true

echo ""
if ros2 pkg list | grep -q "^depthai_ros_driver_v3$"; then
    echo "OK: depthai_ros_driver_v3 found."
else
    echo "WARNING: depthai_ros_driver_v3 not found."
    echo "Check apt output above."
fi

echo ""
echo "========================================"
echo "DepthAI v3 installation COMPLETE"
echo "========================================"
echo ""
echo "To run v3:"
echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "ros2 launch depthai_ros_driver_v3 driver.launch.py"
echo ""
echo "To check topics after launching:"
echo "ros2 node list"
echo "ros2 topic list"
echo ""
echo "If you built v2 from source, use a separate terminal:"
echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "source ${DEPTHAI_V2_WS}/install/setup.bash"
echo ""
