# Sensor Driver & Workspace Setup Scripts

Bash scripts to install the SDKs and build the ROS 2 (Humble) workspaces for each
sensor on the mobile-mapping backpack. Intended for the Jetson / Ubuntu 22.04
target. Each `setup_*` script prints clear step banners; `wipe_*` scripts remove
an install for a clean redo.

First-time machine: run `setup_ros2_humble.sh` first, then the per-sensor scripts.

## Base
- setup_ros2_humble.sh - Install ROS 2 Humble (locale, apt sources, desktop, dev tools).

## LiDAR
- setup_hesai_sdk.sh - Install the HESAI JT128 SDK and dependencies.
- setup_hesai_ws.sh - Build the HESAI JT128 ROS 2 driver workspace (~/hesai_ws).
- setup_livox.sh - Set up the Livox Avia ROS 2 driver workspace.
- setup_livox_config.sh - Write/update the Livox lidar config JSON.
- setup_blickfeld_ws.sh - Blickfeld ROS 2 workspace (out of current scope; kept for reference).

## Cameras
- setup_realsense_sdk.sh - Install the Intel RealSense SDK (librealsense2).
- setup_realsense_ws.sh - Build the RealSense ROS 2 driver from source.
- setup_depthai_v2.sh / setup_depthai_v2_clean.sh - DepthAI (OAK) ROS 2 v2 setup; _clean does a fresh build.
- setup_depthai_v3.sh - DepthAI (OAK) ROS 2 v3 setup.
- setup_insta360_sdk.sh - Install the Insta360 SDK.
- setup_insta360_ws.sh - Build the Insta360 ROS 2 driver workspace (~/insta360_ws).

## IMU
- setup_xsens_ws.sh - Install the Xsens MTi SDK (.deb) and build the ROS 2 workspace (~/xsens_ws).

## GNSS
- setup_gnss_ws.sh - Build the u-blox ZED-F9P ROS 2 workspace (~/gnss_ws).
- config_gnss.sh - Apply the ZED-F9P configuration (zed_f9p.yaml).

## Wipe / reset
- wipe_xsens.sh, wipe_insta360_ws.sh, wipe_isnta360_sdk.sh, wipe_realsense_sdk.sh,
  wipe_realsense_ws.sh, wipe_ros_depthai.sh - Remove the matching SDK/workspace for a clean reinstall.

Note: filenames are preserved as-is from the original commits (e.g. `wipe_isnta360_sdk.sh` has a typo in its name).
