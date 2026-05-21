#!/bin/bash

set -e

echo "======================================"
echo "GNSS (u-blox ZED-F9P) ROS2 Setup"
echo "======================================"

ROS_DISTRO=humble
WS=~/gnss_ws

# -----------------------------
# STEP 1: Source ROS
# -----------------------------
echo "[1/6] Sourcing ROS2..."
source /opt/ros/$ROS_DISTRO/setup.bash

# -----------------------------
# STEP 2: Create workspace
# -----------------------------
echo "[2/6] Creating workspace..."

mkdir -p $WS/src
cd $WS/src

# -----------------------------
# STEP 3: Clone ublox driver
# -----------------------------
echo "[3/6] Cloning ublox..."

if [ ! -d "ublox" ]; then
    git clone --branch ros2 https://github.com/KumarRobotics/ublox.git
else
    echo "Repo already exists"
fi

# -----------------------------
# STEP 4: Dependencies
# -----------------------------
echo "[4/6] Installing dependencies..."

sudo apt update
sudo apt install -y ros-dev-tools python3-rosdep

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update

cd $WS

rosdep install --from-paths src --ignore-src -r -y

# -----------------------------
# STEP 5: Build
# -----------------------------
echo "[5/6] Building..."

#!/bin/bash

set -e

echo "======================================"
echo "GNSS (u-blox ZED-F9P) ROS2 Setup"
echo "======================================"

ROS_DISTRO=humble
WS=~/gnss_ws

# -----------------------------
# STEP 1: Source ROS
# -----------------------------
echo "[1/6] Sourcing ROS2..."
source /opt/ros/$ROS_DISTRO/setup.bash

# -----------------------------
# STEP 2: Create workspace
# -----------------------------
echo "[2/6] Creating workspace..."

mkdir -p $WS/src
cd $WS/src

# -----------------------------
# STEP 3: Clone ublox driver
# -----------------------------
echo "[3/6] Cloning ublox..."

if [ ! -d "ublox" ]; then
    git clone --branch ros2 https://github.com/KumarRobotics/ublox.git
else
    echo "Repo already exists"
fi

# -----------------------------
# STEP 4: Dependencies
# -----------------------------
echo "[4/6] Installing dependencies..."

sudo apt update
sudo apt install -y ros-dev-tools python3-rosdep

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update

cd $WS

rosdep install --from-paths src --ignore-src -r -y

# -----------------------------
# STEP 5: Build
# -----------------------------
echo "[5/6] Building..."

colcon build --symlink-install --parallel-workers 2

source ~/gnss_ws/install/setup.bash

# -----------------------------
# STEP 6: Done
# -----------------------------
echo "======================================"
echo "GNSS setup COMPLETE"
echo "======================================"

echo ""
echo "Next:"
echo "Run config script to modify zed_f9p.yaml"colcon build --symlink-install --parallel-workers 2

# -----------------------------
# STEP 6: Done
# -----------------------------
echo "======================================"
echo "GNSS setup COMPLETE"
echo "======================================"

echo ""
echo "Next:"
echo "Run config script to modify zed_f9p.yaml"
