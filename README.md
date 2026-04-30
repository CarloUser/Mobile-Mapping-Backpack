# Mobile Mapping Backpack

Multi-sensor data acquisition platform integrating LiDAR, cameras, IMU, and GNSS.
Built on ROS2 Humble. Goal: synchronized, time-stamped rosbag recordings for
offline calibration validation and trajectory evaluation.

## Sensor Configuration

| Sensor | Type | Interface | ROS2 Driver |
|--------|------|-----------|-------------|
| HESAI JT128 | 128-beam LiDAR | GbE | `hesai_ros_driver` |
| OAK-4D Luxonis | RGB-D + IMU | USB3 | `depthai_ros_driver` (v3) |
| OAK-D Lite Luxonis | RGB-D | USB3 | `depthai_ros_driver` |
| Intel RealSense D4xx | RGB-D | USB3 | `realsense2_camera` |
| Xsens MTi-610R | IMU | USB/RS-422 | `xsens_mti_ros2_driver` |
| u-blox + ANN-MB-00 | GNSS/RTK | USB/UART | `ublox_gps` |
| Insta360 | 360° video | Internal SD | *(no ROS2 driver — sync post-hoc)* |

## Compute

| Device | Role |
|--------|------|
| NVIDIA Jetson Orin Nano S | Primary ROS2 host — runs all drivers and rosbag2 recording |
| Raspberry Pi | Auxiliary — serial bridge for IMU/GNSS if needed |

## Repository Structure

```
docs/
  setup/
    01_ros2_humble.md       ROS2 Humble installation on Jetson (Ubuntu 22.04)
    02_sensor_drivers.md    Per-sensor driver build and udev rules
    03_time_sync.md         Chrony + PTP time synchronization
  data_collection_protocol.md   Week 1 deliverable — frame IDs, topic names, recording procedure

ros2_ws/src/mmb_bringup/
  launch/
    all_sensors.launch.py   Master launch — brings up every sensor driver
    lidar.launch.py
    cameras.launch.py
    imu_gnss.launch.py
    recording.launch.py     Starts rosbag2 with MCAP storage
  config/
    hesai_jt128.yaml        Hesai driver parameters
    topics.yaml             Topic list for recording

evaluation/
  evo_workflow.md           ATE/RPE evaluation with the evo toolkit
```

## Quick Start

```bash
# 1. Check Ubuntu version — must be 22.04 for Humble
lsb_release -a

# 2. Install ROS2 Humble
# Follow docs/setup/01_ros2_humble.md

# 3. Install sensor drivers
# Follow docs/setup/02_sensor_drivers.md

# 4. Build the bringup package
cd ros2_ws && colcon build --symlink-install
source install/setup.bash

# 5. Launch all sensors
ros2 launch mmb_bringup all_sensors.launch.py

# 6. Start recording (separate terminal)
ros2 launch mmb_bringup recording.launch.py bag_name:=mmb_20260101_1200_hallway_run01
```

## Team

| Student | Role |
|---------|------|
| 1 | Camera Calibration Lead |
| 2 | LiDAR Calibration Lead |
| 3 | GNSS-IMU Calibration Lead |
| 4 | Extrinsic Calibration Integrator |
| 5 | CAD Design Lead |
| 6 | System Integration & Testing Lead |
