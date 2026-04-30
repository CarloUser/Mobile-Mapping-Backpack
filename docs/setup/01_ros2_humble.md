# ROS2 Humble Installation (Jetson Orin Nano S — Ubuntu 22.04)

## Prerequisites

ROS2 Humble requires **Ubuntu 22.04**. Verify before proceeding:

```bash
lsb_release -a
# Must show: Ubuntu 22.04.x LTS
```

If the Jetson is running JetPack 5.x (Ubuntu 20.04), you need to reflash with
JetPack 6.x first. Download SDK Manager from NVIDIA and flash via USB-C.

---

## 1. Set Locale

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## 2. Add ROS2 Apt Repository

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

## 3. Install ROS2 Humble

```bash
sudo apt install -y ros-humble-desktop
```

## 4. Install Build Tools

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip
sudo rosdep init
rosdep update
```

## 5. Install MCAP Storage Plugin

rosbag2 defaults to SQLite3, which is slow for high-rate sensors. MCAP is
required for reliable multi-sensor recording.

```bash
sudo apt install -y ros-humble-rosbag2-storage-mcap
```

## 6. Shell Setup

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 7. Verify

```bash
ros2 doctor
# Should report no critical issues
ros2 bag record --help | grep mcap
# Should show mcap as an available storage plugin
```

---

## Next Step

→ [02_sensor_drivers.md](02_sensor_drivers.md) — install drivers for each sensor.
