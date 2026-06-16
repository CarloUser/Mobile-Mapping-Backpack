#!/bin/bash

set -u
set -o pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
ASSUME_YES=false
DRY_RUN=false

# run_all_jetson.sh installs a few common tools that may have existed before the
# backpack setup. Keep them by default; set true for a stricter wipe test.
WIPE_SHARED_DEPS="${WIPE_SHARED_DEPS:-false}"

for arg in "$@"; do
    case "$arg" in
        -y|--yes)
            ASSUME_YES=true
            ;;
        --dry-run)
            DRY_RUN=true
            ;;
        -h|--help)
            cat <<EOF
Usage: $(basename "$0") [--yes] [--dry-run]

Deletes the files, workspaces, udev rules, apt sources, and packages installed
by bash/run_all_jetson.sh so the Jetson setup can be tested from a clean state.

Environment:
  ROS_DISTRO=humble          ROS distribution to wipe.
  WIPE_SHARED_DEPS=false     Also purge generic packages installed by setup
                             scripts, such as git, cmake, curl, unzip, tar,
                             locales, and build-essential.
EOF
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            exit 2
            ;;
    esac
done

step() {
    echo ""
    echo "========================================="
    echo "$1"
    echo "========================================="
}

warn() {
    echo "WARNING: $*" >&2
}

run_cmd() {
    echo "+ $*"
    if [ "$DRY_RUN" = true ]; then
        return 0
    fi

    "$@" || warn "Command failed, continuing: $*"
}

confirm() {
    if [ "$DRY_RUN" = true ] || [ "$ASSUME_YES" = true ]; then
        return 0
    fi

    cat <<EOF
This will remove ROS 2 ${ROS_DISTRO}, sensor SDKs, workspaces, udev rules,
and packages installed by run_all_jetson.sh.

Type WIPE to continue:
EOF
    read -r reply
    if [ "$reply" != "WIPE" ]; then
        echo "Aborted."
        exit 1
    fi
}

need_sudo() {
    if [ "$DRY_RUN" = true ]; then
        return 0
    fi

    sudo -v
}

purge_installed() {
    local label="$1"
    shift

    step "Purging apt packages: $label"

    local packages
    mapfile -t packages < <(
        dpkg-query -W -f='${db:Status-Abbrev} ${binary:Package}\n' "$@" 2>/dev/null \
            | awk '$1 ~ /^(ii|rc)/ {print $2}' \
            | sort -u
    )

    if [ "${#packages[@]}" -eq 0 ]; then
        echo "No installed packages matched: $*"
        return 0
    fi

    printf 'Packages:\n'
    printf '  %s\n' "${packages[@]}"
    run_cmd sudo apt-get purge -y "${packages[@]}"
}

remove_paths() {
    local label="$1"
    shift

    step "Removing paths: $label"

    local path
    for path in "$@"; do
        if [ -e "$path" ] || [ -L "$path" ]; then
            run_cmd rm -rf -- "$path"
        else
            echo "Not present: $path"
        fi
    done
}

remove_sudo_paths() {
    local label="$1"
    shift

    step "Removing system paths: $label"

    local path
    for path in "$@"; do
        if [ -e "$path" ] || [ -L "$path" ]; then
            run_cmd sudo rm -rf -- "$path"
        else
            echo "Not present: $path"
        fi
    done
}

remove_opencv_compat_symlinks() {
    step "Removing Jetson OpenCV compatibility symlinks"

    shopt -s nullglob
    local link
    local found=false
    for link in /usr/lib/libopencv_*.so.4.8.0; do
        found=true
        if [ -L "$link" ]; then
            run_cmd sudo rm -f -- "$link"
        else
            echo "Keeping non-symlink file: $link"
        fi
    done
    shopt -u nullglob

    if [ "$found" = false ]; then
        echo "No matching symlinks found."
    fi
}

clean_bashrc() {
    step "Cleaning ROS setup lines from ~/.bashrc"

    local bashrc="$HOME/.bashrc"
    if [ ! -f "$bashrc" ]; then
        echo "Not present: $bashrc"
        return 0
    fi

    run_cmd sed -i \
        -e "\\|/opt/ros/${ROS_DISTRO}/setup.bash|d" \
        -e "\\|/ros_ws/install/setup.bash|d" \
        -e "\\|/ros2_ws/install/setup.bash|d" \
        -e "\\|depthai_install|d" \
        "$bashrc"
}

reload_udev() {
    step "Reloading udev rules"

    if command -v udevadm >/dev/null 2>&1; then
        run_cmd sudo udevadm control --reload-rules
        run_cmd sudo udevadm trigger
    else
        echo "udevadm not found, skipping reload."
    fi
}

apt_cleanup() {
    step "Cleaning apt state"

    run_cmd sudo dpkg --configure -a
    run_cmd sudo apt-get autoremove -y
    run_cmd sudo apt-get clean
    run_cmd sudo apt-get update
}

confirm
need_sudo

step "Mobile Mapping Backpack Jetson wipe"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "WIPE_SHARED_DEPS=$WIPE_SHARED_DEPS"
echo "DRY_RUN=$DRY_RUN"

# setup_ros2_humble.sh and setup_depthai_v3_auto.sh
purge_installed "ROS 2 ${ROS_DISTRO}" \
    "ros-${ROS_DISTRO}-*" \
    "ros-dev-tools" \
    "ros2-testing-apt-source" \
    "ros2-apt-source"

# setup_realsense_sdk_jetson.sh
purge_installed "RealSense SDK" \
    "librealsense2-*"

# jetson_libopencv.sh
purge_installed "Jetson/OpenCV packages" \
    "nvidia-opencv*" \
    "libopencv*" \
    "opencv-data" \
    "opencv-licenses" \
    "python3-opencv"

# setup_depthai_v2.sh, setup_hesai_sdk.sh, setup_hesai.sh, setup_xsens.sh,
# setup_gnss.sh, and rosdep-resolved runtime packages.
purge_installed "sensor-specific dependencies" \
    "libboost-all-dev" \
    "libpcap-dev" \
    "libpcl-dev" \
    "libssl-dev" \
    "libusb-1.0-0-dev" \
    "libyaml-cpp-dev" \
    "python3-rosdep" \
    "ros-${ROS_DISTRO}-camera-info-manager" \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-depthai*" \
    "ros-${ROS_DISTRO}-diagnostic-updater" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-mavros-msgs" \
    "ros-${ROS_DISTRO}-message-filters" \
    "ros-${ROS_DISTRO}-nmea-msgs" \
    "ros-${ROS_DISTRO}-rviz2" \
    "ros-${ROS_DISTRO}-tf2-geometry-msgs" \
    "ros-${ROS_DISTRO}-tf2-ros"

if [ "$WIPE_SHARED_DEPS" = true ]; then
    purge_installed "shared build/setup tools installed by run_all_jetson.sh" \
        "build-essential" \
        "cmake" \
        "curl" \
        "git" \
        "locales" \
        "software-properties-common" \
        "tar" \
        "unzip"
else
    echo ""
    echo "Skipping shared build/setup tools because WIPE_SHARED_DEPS=false."
fi

remove_paths "home workspaces and SDK directories" \
    "$HOME/ros2_ws" \
    "$HOME/ros_ws" \
    "$HOME/depthai_v2_ws" \
    "$HOME/hesai_sdk" \
    "$HOME/insta360_sdk" \
    "$HOME/.ros"

remove_sudo_paths "udev rules installed by setup/config scripts" \
    "/etc/udev/rules.d/50-ardusimple.rules" \
    "/etc/udev/rules.d/80-movidius.rules" \
    "/etc/udev/rules.d/99-insta.rules"

remove_sudo_paths "ROS apt source and rosdep files" \
    "/etc/apt/sources.list.d/ros2.list" \
    "/etc/apt/sources.list.d/ros2.sources" \
    "/etc/apt/sources.list.d/ros-latest.list" \
    "/etc/ros" \
    "/usr/share/keyrings/ros-archive-keyring.gpg" \
    "/etc/apt/trusted.gpg.d/ros-archive-keyring.gpg" \
    "/tmp/ros2-apt-source.deb"

remove_sudo_paths "OpenCV files touched by Jetson setup" \
    "/usr/lib/cmake/opencv4" \
    "/usr/lib/pkgconfig/opencv4.pc"

remove_opencv_compat_symlinks
clean_bashrc
reload_udev
apt_cleanup

step "Wipe complete"
echo "You can rerun bash/run_all_jetson.sh after opening a new shell."
echo "Note: plugdev group membership and locale changes are not reverted."
