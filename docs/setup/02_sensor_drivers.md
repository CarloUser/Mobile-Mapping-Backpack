# Sensor Driver Installation

Install and verify each driver individually before combining in the full launch.
All builds go into `~/ros2_ws` unless noted otherwise.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

---

## HESAI JT128 LiDAR

**Interface**: Ethernet (GbE). Default LiDAR IP: `192.168.1.201`.
Set the Jetson's Ethernet interface to a static IP in the same subnet, e.g. `192.168.1.100`.

```bash
cd ~/ros2_ws/src
git clone https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git --recursive

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select hesai_ros_driver
source install/setup.bash
```

**Verify** (LiDAR powered and connected):
```bash
ros2 launch hesai_ros_driver start.py
# In another terminal:
ros2 topic hz /lidar_points
# Should show ~10 Hz (default rotation rate)
```

**Jetson Ethernet static IP setup**:
```bash
# Replace eth0 with your actual interface name (check: ip link)
sudo nmcli con add type ethernet ifname eth0 con-name hesai-lidar \
  ip4 192.168.1.100/24
sudo nmcli con up hesai-lidar
```

---

## OAK-4D Luxonis and OAK-D Lite Luxonis

Both use `depthai-ros`. The OAK-4D uses the v3 API; OAK-D Lite uses v2.
Install udev rules first so USB access works without sudo.

```bash
# udev rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

```bash
# ROS2 package (apt — prebuilt for Humble)
sudo apt install -y ros-humble-depthai-ros

# If apt package is outdated, build from source instead:
# cd ~/ros2_ws/src
# git clone https://github.com/luxonis/depthai-ros.git
# cd ~/ros2_ws && colcon build --symlink-install --packages-select depthai_ros_driver
```

**Verify** (camera plugged in via USB3 — use the USB3 port, not USB2):
```bash
# OAK-D Lite
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-LITE
ros2 topic hz /oak/rgb/image_raw

# OAK-4D (use camera_model:=OAK-4D or check depthai-ros docs for exact name)
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-4
```

**Note**: If running both OAK cameras simultaneously, each needs a separate
USB3 port (or a powered USB3 hub). They must be launched with different
namespaces — see `cameras.launch.py`.

---

## Intel RealSense

```bash
# librealsense2 via Intel apt repo
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# ROS2 wrapper
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description
```

**Verify**:
```bash
# First check the camera is detected
realsense-viewer  # optional GUI
rs-enumerate-devices

ros2 launch realsense2_camera rs_launch.py
ros2 topic hz /camera/color/image_raw
```

---

## Xsens MTi-610R IMU

**Interface**: USB (shows as `/dev/ttyUSB0` or `/dev/ttyACM0`).

```bash
# udev rule for serial access without sudo
sudo usermod -aG dialout $USER   # log out and back in after this

cd ~/ros2_ws/src
git clone https://github.com/nobleo/xsens_mti_ros2_driver.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select xsens_mti_ros2_driver
source install/setup.bash
```

**Verify**:
```bash
# Check which port the IMU appears on
ls /dev/ttyUSB* /dev/ttyACM*

ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
ros2 topic hz /imu/data    # should be 400 Hz for MTi-610R
```

**Baud rate**: The MTi-610R defaults to 115200 baud via USB but can go up to
921600. Configure with MT Manager (Windows/Linux) before deployment if needed.

---

## u-blox GNSS (with ANN-MB-00 Antenna)

The ANN-MB-00 is a multi-band antenna — it connects to a u-blox receiver
(e.g., ZED-F9P evaluation board or equivalent). Confirm what receiver board
you have; the antenna alone does not connect to USB.

```bash
sudo apt install -y ros-humble-ublox
# Or build from source for more recent fixes:
# cd ~/ros2_ws/src
# git clone https://github.com/KumarRobotics/ublox.git -b ros2
# colcon build --symlink-install --packages-select ublox_gps
```

**Verify**:
```bash
# Check serial port
ls /dev/ttyUSB* /dev/ttyACM*

ros2 run ublox_gps ublox_gps_node --ros-args \
  -p device:=/dev/ttyACM0 \
  -p frame_id:=gnss \
  -p config_on_startup:=false

ros2 topic echo /fix     # NavSatFix messages once outdoors with sky view
```

---

## Insta360 (360° Camera)

There is no ROS2 driver for the Insta360. Use this workflow instead:

1. **Record internally** to the Insta360's SD card during each run.
2. **Sync timestamps** post-hoc using the GPS track in the rosbag to align
   the Insta360 footage via GPS-embedded metadata or a visible event
   (e.g., a flash LED triggered at recording start).
3. Export frames with `ffmpeg` if needed for offline processing.

No driver installation required.

---

## Build Everything Together

After cloning all source-built packages:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Add workspace to `.bashrc` so it persists:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Next Step

→ [03_time_sync.md](03_time_sync.md) — configure time synchronization before
doing any multi-sensor recording.
