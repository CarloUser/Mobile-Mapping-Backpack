# `bash/` — setup, launch and recording scripts

Shell scripts to install the sensor SDKs and ROS 2 (Humble) driver workspaces,
launch the sensors, and record bags on the Jetson (Ubuntu 22.04). **This file is
only an index of what each script does — for the install order, the launch and
recording procedures, and troubleshooting, see [`manual.md`](manual.md).**

On a fresh machine the whole install runs via `run_all_jetson.sh`; to do it by hand,
run `jetson_libopencv.sh` and `setup_ros2_humble.sh` first, then the per-sensor
`setup_*`/`config_*` scripts, and finally `build_sensors.sh` (the exact order is in
`manual.md`).

## Setup — SDKs and driver workspaces

| Script(s) | Purpose |
| --- | --- |
| `run_all_jetson.sh` | One command: runs every setup script below in the correct order. |
| `jetson_libopencv.sh` | Repair the Jetson's system OpenCV — run **before** ROS 2. |
| `setup_ros2_humble.sh` | Install ROS 2 Humble (locale, apt sources, desktop, dev tools). |
| `setup_hesai_sdk.sh`, `setup_hesai.sh`, `config_hesai.sh` | HESAI JT128 SDK, driver workspace, device configuration. |
| `setup_livox.sh`, `setup_livox_config.sh` | Livox Avia driver workspace and lidar-config JSON. |
| `setup_blickfeld_ws.sh` | Blickfeld workspace (out of scope; kept for reference). |
| `setup_depthai_v2.sh`, `setup_depthai_v3.sh`, `setup_depthai_v3_auto.sh` | DepthAI (OAK) — v2 for OAK-1 / OAK-D Lite, v3 for OAK-4D. |
| `setup_realsense_sdk.sh`, `setup_realsense_sdk_jetson.sh`, `setup_realsense_binary.sh` | Intel RealSense SDK + driver (generic / Jetson / binary). |
| `setup_insta360_sdk.sh`, `setup_insta360_sdk_jetson.sh`, `setup_insta360.sh`, `setup_insta360_ws_with_sdk.sh`, `config_insta360.sh`, `config_insta360_jetson.sh` | Insta360 SDK, driver workspace and config (generic / Jetson). |
| `setup_xsens.sh` | Xsens MTi SDK (.deb) + ROS 2 workspace. |
| `setup_gnss.sh`, `config_gnss.sh` | u-blox ZED-F9P driver workspace and device configuration. |
| `setup_time_sync.sh` | chrony + PTP (`ptp4l`/`phc2sys`) installed as `systemd` units. |
| `build_sensors.sh` | Build all driver workspaces (`colcon build`). |
| `JT128_default_angle.csv` | HESAI JT128 default per-channel angle table (driver data). |

## Launch

| Script(s) | Purpose |
| --- | --- |
| `launch_all_sensors.sh` | Bring up every sensor driver. |
| `launch_lidars.sh`, `launch_cameras.sh`, `launch_gnss_imu.sh` | Bring up one sensor group. |

These wrap the `mmb_bringup` launch files; see `manual.md` → *Launching the drivers*.

## Record

| Script(s) | Purpose |
| --- | --- |
| `record_all.sh` | Launch all drivers and record every topic in `config/topics.yaml` to MCAP, with pre-flight checks. |
| `record_dual.sh`, `record_split.sh` | Two-Jetson split recording (see `docs/dual_jetson_recording.md`). |
| `record_pair.sh`, `record_cam_lidar.sh` | Per camera–LiDAR pair recording for the camera↔LiDAR calibration. |
| `record_oak4d.sh`, `record_oak1.sh`, `record_oakd_lite.sh`, `record_realsense.sh`, `record_insta360.sh` | Record a single camera. |

See `manual.md` → *Recording topics* for usage and the bag-naming convention.

## Wipe / reset

`wipe_*` scripts remove an SDK or workspace for a clean reinstall — **use with
care**: `wipe_run_all_jetson.sh`, `wipe_depthai_v2.sh`, `wipe_realsense_sdk.sh`,
`wipe_realsense_ws.sh`, `wipe_insta360_ws.sh`, `wipe_isnta360_sdk.sh`,
`wipe_xsens.sh`.

> Filenames are preserved as-is from the original commits (e.g.
> `wipe_isnta360_sdk.sh` has a typo in its name).
