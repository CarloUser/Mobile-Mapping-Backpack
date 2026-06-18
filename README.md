# Mobile Mapping Backpack

A wearable multi-sensor data-acquisition platform — two LiDARs, five cameras, an
IMU and a GNSS receiver, rigidly mounted on one frame — built on **ROS 2 Humble**.
The goal is synchronized, time-stamped `rosbag2` recordings that downstream
calibration and mapping tools can consume, with **every sensor expressed in one
common body frame, `base_link`**, taking the **Hesai JT128 as the fixed reference**.

This README is the **index** for the repository: what the hardware is, how the code
is organised, how to bring the rig up, and where every document lives.

## Hardware

| Sensor | Model | Role / direction | Interface | ROS 2 driver |
| --- | --- | --- | --- | --- |
| LiDAR (reference) | HESAI JT128 | front | GbE | `hesai_ros_driver` |
| LiDAR | Livox Avia | rear | GbE | `livox_ros2_avia` |
| Camera | OAK-4D (RVC4) | front | PoE | `depthai` v3 |
| Camera | OAK-D Lite | rear | USB3 | `depthai` v2 |
| Camera | OAK-1 | rear | USB3 | `depthai` v2 |
| Camera | Intel RealSense D4xx | rear | USB3 | `realsense2_camera` |
| Camera | Insta360 (dual-fisheye) | omni | USB | `insta360_ros_driver` |
| IMU | Xsens MTi-610R | — | USB/serial | `xsens_mti_ros2_driver` |
| GNSS | u-blox + ANN-MB-00 | — | USB/serial | `ublox_gps` |

Compute: **NVIDIA Jetson Orin Nano S** (primary ROS 2 host, runs all drivers and
recording). A two-Jetson split is documented for full-rate recording
(`docs/dual_jetson_recording.md`).

## Repository map

```
README.md                  this index
CALIBRATION_CONTEXT.md     full technical reference (sensors, frames, every method, status)
bash/                      driver/SDK install, config and recording scripts
  manual.md                driver install / launch / record / troubleshoot manual
  README.md                index of the setup_*/config_*/wipe_* scripts
docs/
  setup/                   step-by-step install reference (ROS 2, drivers, time sync)
  data_collection_protocol.md   naming, frame IDs, topic structure
  recording_workflow.md    all-sensor recording procedure
  dual_jetson_recording.md two-Jetson split for throughput
ros2_ws/src/mmb_bringup/   ROS 2 bring-up package (launch + config + static TF)
Camera_Calibration/        camera intrinsics (core engine + per-camera entry points)
extrinsic_calibration/     extrinsic calibration (see its README)
  lidar_lidar/             Hesai <-> Livox motion-based hand-eye  [done, verdict GOOD]
  camera_lidar/            camera <-> LiDAR ChArUco plane calibration
  scripts/                 CAD->REP-103 converter, readiness + seed validators
Coordinate_systems/        SolidWorks export (source of truth) + export macro
evaluation/                trajectory (ATE/RPE) evaluation with evo
```

## Coordinate frames

The whole tree uses **ROS REP-103**: `base_link` and all body frames (LiDARs, IMU,
GNSS) are **x-forward, y-left, z-up**; cameras are **optical** frames
(x-right, y-down, z-forward, matching `camera_info`/OpenCV); world frames are ENU.
The SolidWorks export (x-forward, y-up, z-right) is the immutable source of truth in
`Coordinate_systems/coordinate_systems.yaml`; it is re-expressed into REP-103 by
`extrinsic_calibration/scripts/convert_cad_to_ros_frame.py` (`Rx(+90°)`) to produce
the seed `extrinsics_initial.yaml`. Calibrated results are written to
`extrinsics_calibrated.yaml` — never overwrite the CAD source or the seed.

## Quick start

Full procedure: `bash/manual.md` (install / launch / record / troubleshoot) and
`docs/setup/`. In short, on the Jetson (Ubuntu 22.04 + ROS 2 Humble):

```bash
# 1. Install drivers (one command, or run bash/setup_*.sh in order — see manual.md)
bash bash/run_all_jetson.sh

# 2. Build the bring-up package
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# 3. Time sync (once): chrony + PTP master; enable PTP slave in the Hesai web UI
sudo bash bash/setup_time_sync.sh <lidar_iface>

# 4. Launch all sensors, then record every topic in config/topics.yaml to MCAP
ros2 launch mmb_bringup all_sensors.launch.py
bash bash/record_all.sh <location>_<run_type>_<run_number>
```

## Documentation index

| Document | Purpose |
| --- | --- |
| `CALIBRATION_CONTEXT.md` | Single self-contained technical reference: sensor suite, frame conventions, and every intrinsic/extrinsic calibration method, with current status. Start here to understand the project. |
| `bash/manual.md` | Driver install (three routes), per-driver launch commands, recording (`record_all.sh` + manual rosbag2), and a troubleshooting table. |
| `bash/README.md` | Index of the `setup_*` / `config_*` / `wipe_*` scripts. |
| `docs/setup/01_ros2_humble.md`, `02_sensor_drivers.md`, `03_time_sync.md` | Step-by-step install reference for ROS 2, each driver, and the chrony/PTP time-sync stack. |
| `docs/data_collection_protocol.md` | Naming conventions, frame IDs, topic structure and TF tree. |
| `docs/recording_workflow.md` | End-to-end all-sensor recording procedure. |
| `docs/dual_jetson_recording.md` | Two-Jetson split (PTP master/slave, per-Jetson topic sets). |
| `evaluation/evo_workflow.md` | Trajectory accuracy (ATE/RPE) evaluation with the evo toolkit. |
| `extrinsic_calibration/README.md` | Extrinsic-calibration workflow overview and calibration order. |
| `extrinsic_calibration/CALIBRATION_STRATEGY.md` | The calibration graph (backbone + loop closures) and the rationale. |
| `extrinsic_calibration/METHOD_EVALUATION.md` | The chosen methods evaluated against standard practice. |
| `extrinsic_calibration/lidar_lidar/README.md` | Hesai <-> Livox motion-based hand-eye: recording, processing, solve and validation. |
| `extrinsic_calibration/lidar_lidar/FASTLIO_GUIDE.md` | Building and running FAST-LIO for the narrow-FOV Livox odometry. |
| `extrinsic_calibration/camera_lidar/README.md` | Camera <-> LiDAR ChArUco plane-correspondence calibration: method, geometry and operation. |
| `extrinsic_calibration/camera_lidar/KOIDE_CROSSCHECK.md` | Optional target-less cross-check (Koide `direct_visual_lidar_calibration`). |

## Calibration status

| Stage | Status |
| --- | --- |
| Camera intrinsics | OAK-4D, Insta360 done; remaining rear cameras to (re)capture |
| Hesai <-> Livox (LiDAR-LiDAR) | **Solved — verdict GOOD** after `small_gicp` map-based refinement |
| OAK-4D <-> Hesai (camera-LiDAR) | Pipeline works; needs a re-record with the board isolated, then solve |
| Rear cameras <-> Livox | Drivers ready; not yet recorded |
| Insta360 <-> Hesai | Blocked on H.264 decode / missing `camera_info`; use Koide target-less |
| IMU / GNSS | Noise + bias characterised (Allan variance); lever-arm extrinsic pending |
| Loop closures, global optimisation | Not started |

## Team

| Member | Lead role |
| --- | --- |
| Mirutica Rajesh Karthik | Camera Calibration |
| Jakob Wickenhäuser | LiDAR Calibration |
| Vraj Orbit | GNSS-IMU Calibration |
| Maziar Shateri | Extrinsic Calibration Integrator |
| Burak Topaloglu | CAD Design |
| Carlo Brix Bronold | System Integration & Testing |
