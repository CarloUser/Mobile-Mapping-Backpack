# Data Collection Protocol

**Version**: 1.0  
**Owner**: Student 6 — System Integration & Testing Lead  
**Status**: Draft (circulate to team by end of Week 1 for review)

This document defines the naming conventions, frame IDs, topic structure,
recording procedure, and calibration sequences that all team members must
follow to ensure rosbags are interoperable across calibration workflows.

---

## 1. Coordinate Frames and TF Tree

All frames follow the ROS [REP-103](https://www.ros.org/reps/rep-0103.html)
convention: x-forward, y-left, z-up for body frames; ENU for world frames.

```
base_link                    ← physical center of the backpack frame
├── lidar_hesai              ← HESAI JT128 optical center
├── lidar_livox_avia         ← Livox Avia optical center
├── camera_oak4d             ← OAK-4D Luxonis left camera optical center
├── camera_oakd_lite         ← OAK-D Lite left camera optical center
├── camera_oak1              ← OAK-1 camera optical center
├── camera_realsense         ← Intel RealSense color camera optical center
├── camera_insta360          ← Insta360 camera frame
├── imu_xsens                ← Xsens MTi-610R measurement frame
└── gnss_antenna             ← u-blox ANN-MB-00 antenna phase center
```

**Static transforms** are published at launch from `base_link` to each sensor
frame using estimated poses from Student 5's CAD model. These are placeholders
until Student 4 delivers calibrated extrinsics; update `all_sensors.launch.py`
with the refined values in Week 2.

**Frame IDs used in sensor driver configs**:

| Sensor | `frame_id` |
|--------|-----------|
| HESAI JT128 | `lidar_hesai` |
| Livox Avia | `lidar_livox_avia` |
| OAK-4D (RGB) | `camera_oak4d` |
| OAK-D Lite (RGB) | `camera_oakd_lite` |
| OAK-1 | `camera_oak1` |
| Intel RealSense (color) | `camera_realsense` |
| Insta360 | `camera_insta360` |
| Xsens MTi-610R | `imu_xsens` |
| u-blox GNSS | `gnss_antenna` |

---

## 2. Topic Naming Convention

All sensor topics live under a sensor-specific namespace:

| Topic | Message Type | Hz (approx) |
|-------|-------------|-------------|
| `/lidar_points` | `sensor_msgs/PointCloud2` | 10 |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | driver-dependent |
| `/livox/imu` | `sensor_msgs/Imu` | driver-dependent |
| `/oak4d/rgb/image_raw` | `sensor_msgs/Image` | 30 |
| `/oak4d/depth/image_raw` | `sensor_msgs/Image` | 30 |
| `/oak4d/rgb/camera_info` | `sensor_msgs/CameraInfo` | 30 |
| `/oak4d/imu/data` | `sensor_msgs/Imu` | 500 |
| `/oakd_lite/rgb/image_raw` | `sensor_msgs/Image` | 30 |
| `/oakd_lite/depth/image_raw` | `sensor_msgs/Image` | 30 |
| `/oakd_lite/rgb/camera_info` | `sensor_msgs/CameraInfo` | 30 |
| `/realsense/color/image_raw` | `sensor_msgs/Image` | 30 |
| `/realsense/depth/image_rect_raw` | `sensor_msgs/Image` | 30 |
| `/realsense/color/camera_info` | `sensor_msgs/CameraInfo` | 30 |
| `/imu/data` | `sensor_msgs/Imu` | 400 |
| `/imu/mag` | `sensor_msgs/MagneticField` | 100 |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | 5–10 |
| `/gnss/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | 5–10 |
| `/tf` | `tf2_msgs/TFMessage` | — |
| `/tf_static` | `tf2_msgs/TFMessage` | — |

---

## 3. Bag Naming Convention

```
mmb_YYYYMMDD_HHMM_<location>_<run_type>_<run_number>.mcap
```

Examples:
```
mmb_20260505_1430_lab_calibration_01.mcap
mmb_20260512_1015_hallway_static_01.mcap
mmb_20260518_0930_courtyard_walk_03.mcap
```

**Run types**:
- `calibration` — camera/LiDAR calibration target sequences (static)
- `static` — sensor stationary for IMU noise characterization
- `walk` — normal walking pace, straight or looped path
- `loop` — closed loop for drift evaluation

Store bags on an **NVMe SSD** mounted at `/data/bags/`. Multi-sensor recording
at full rate produces ~1.5–2 GB/min. A 500 GB SSD gives roughly 4 hours of
recording time.

---

## 4. Recording Procedure

### 4.1 Pre-run Checklist

- [ ] Jetson booted, time sync confirmed (`chronyc tracking` shows < 2 ms offset)
- [ ] All sensors powered and publishing (`ros2 topic list` shows all topics)
- [ ] Storage has sufficient space (`df -h /data`)
- [ ] Topic rates verified (`ros2 topic hz /lidar_points`, `ros2 topic hz /livox/lidar`, `ros2 topic hz /livox/imu`, etc.)
- [ ] Insta360 internal recording started and timestamp noted

### 4.2 Start Recording

```bash
# Terminal 1 — launch all sensor drivers
ros2 launch mmb_bringup all_sensors.launch.py

# Terminal 2 — start recording (replace bag name as appropriate)
ros2 launch mmb_bringup recording.launch.py \
  bag_name:=/data/bags/mmb_20260505_1430_lab_calibration_01
```

### 4.3 During Recording

- Keep the backpack as still as possible for the first **10 seconds** to
  establish a clean IMU static baseline.
- For outdoor runs, wait for GNSS fix confirmation before starting motion
  (`ros2 topic echo /gnss/fix --once` should show `status.status >= 0`).
- Note the Insta360 start timestamp in the run log (see Section 6).

### 4.4 Stop Recording

```bash
# Ctrl+C in Terminal 2 to stop rosbag2
# Verify the bag is not corrupted
ros2 bag info /data/bags/mmb_20260505_1430_lab_calibration_01
```

---

## 5. Calibration Data Sequences

The following sequences must be captured before Week 2 to give Students 1–4
the data they need.

### 5.1 Camera Intrinsics (Student 1)

- Use a **7×10 chessboard** (square size 30 mm).
- Record ~60–80 frames with the board in varied positions and orientations.
- Cover all corners of the image frame and include tilted views.
- Bag type: `calibration`, camera stationary or hand-held board.

```bash
# Record cameras only for efficiency during intrinsics
ros2 bag record -o /data/bags/mmb_20260505_1000_lab_calibration_intrinsics \
  --storage mcap \
  /oak4d/rgb/image_raw /oak4d/rgb/camera_info \
  /oakd_lite/rgb/image_raw /oakd_lite/rgb/camera_info \
  /realsense/color/image_raw /realsense/color/camera_info
```

### 5.2 LiDAR Calibration (Student 2)

- Use a **flat planar target** (e.g., a large foam board) at various distances (1–5 m).
- Hold the board stationary for 3–5 seconds per position.
- Capture at least 10 positions covering different depths and angles.

### 5.3 IMU Static Calibration (Student 3)

- Place the backpack completely still on a flat surface.
- Record **at least 30 minutes** of static IMU data for Allan Variance analysis.

```bash
ros2 bag record -o /data/bags/mmb_20260505_1100_lab_static_imu \
  --storage mcap /imu/data /imu/mag
```

### 5.4 Extrinsic Calibration (Student 4)

- Print and mount **AprilTag board** (tagStandard41h12, 10 cm tags).
- Record all cameras and both LiDARs simultaneously viewing the calibration scene from multiple angles.
- Walk slowly around the board, pausing at each angle.

---

## 6. Run Log

Maintain a physical or shared-document log for every recording session.
Minimum entries per run:

| Field | Example |
|-------|---------|
| Date/Time | 2026-05-05 14:30 |
| Bag name | mmb_20260505_1430_lab_calibration_01 |
| Location | Lab room 203 |
| Operators | Student 6, Student 1 |
| Insta360 start time | 14:31:05 UTC |
| Weather (outdoor) | Overcast, dry |
| GNSS fix type (outdoor) | RTK Fixed |
| Notes | Board illumination uneven — retake positions 4–6 |

---

## 7. Quality Checks After Each Session

```bash
# Check bag integrity and duration
ros2 bag info /data/bags/<bag_name>

# Spot-check topic rates in the bag
ros2 bag play /data/bags/<bag_name> --clock &
ros2 topic hz /lidar_points
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /imu/data

# Check for timestamp jumps or gaps (>100 ms gap on any topic is a problem)
# Use evo_traj or a simple Python script to plot timestamps
```

If any topic is missing or the rate is consistently less than 50% of expected,
re-record before moving on.
