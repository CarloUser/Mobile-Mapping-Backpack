# Recording checklist — Hesai &harr; Livox calibration bag (with IMUs)

Goal: one bag with both LiDARs and both IMUs, Livox in **CustomMsg** format (so
FAST-LIO can deskew), captured during a 3-axis-excitation walk.

Topics to capture:

| Topic | Type | Source |
| --- | --- | --- |
| `/lidar_points` | PointCloud2 | Hesai JT128 |
| `/lidar_imu` | Imu | Hesai built-in IMU |
| `/livox/lidar` | **CustomMsg** | Livox Avia (CustomMsg launch!) |
| `/livox/imu` | Imu | Livox built-in IMU |

## A. Launch the drivers (separate terminals; source ROS first)

```bash
source /opt/ros/humble/setup.bash         # in every terminal

# Hesai -> /lidar_points + /lidar_imu
ros2 launch hesai_ros_driver start.py

# Livox in CustomMsg mode -> /livox/lidar (CustomMsg) + /livox/imu
ros2 launch livox_ros2_avia livox_lidar_msg_launch.py
```
(`livox_lidar_msg_launch.py` is the CustomMsg launch. Do NOT use
`livox_lidar_launch.py` — that publishes PointCloud2 with no per-point time.)

## B. Pre-flight checks (before walking)

```bash
ros2 topic list | grep -E "lidar_points|lidar_imu|livox"   # all four present?
ros2 topic hz /lidar_points     # ~10 Hz
ros2 topic hz /livox/lidar      # ~10 Hz
ros2 topic hz /lidar_imu        # ~100-400 Hz
ros2 topic hz /livox/imu        # ~100-200 Hz

# *** CRITICAL: confirm Livox is CustomMsg, and capture the exact type ***
ros2 topic info -v /livox/lidar
```
The last command must show a **CustomMsg** type (e.g. `livox_interfaces/msg/CustomMsg`
or `livox_ros_driver2/msg/CustomMsg`), **not** `sensor_msgs/msg/PointCloud2`.
**Send me that exact type string** — it determines the one-line FAST-LIO build fix.

## C. Record (one bag, all four topics)

```bash
ros2 bag record -o lidar_calib_imu_$(date +%Y%m%d_%H%M) \
    /lidar_points /lidar_imu /livox/lidar /livox/imu
```
Start recording, do the walk (section D), then Ctrl-C to stop.

## D. The walk (this is what makes or breaks it)

Hand-eye is unobservable without rotation about all three axes, and FAST-LIO needs
smooth motion in a structured scene. So, for 2-5 minutes in a feature-rich indoor
space (walls, furniture, corners — nearby structure in *both* LiDARs' views):

- Walk figure-of-eights, not straight lines.
- Include a ramp or staircase up and down (adds pitch).
- Deliberately tilt/roll the backpack side-to-side and nod it up/down.
- Vary heading and speed; keep motions **smooth** (no violent jerks — they saturate
  the IMU and hurt FAST-LIO).
- Pause 1-2 s occasionally.
- Avoid people/objects moving through the scene, and avoid long featureless
  corridors or staring at a single blank wall.

## E. After the walk — verify the bag

```bash
source /opt/ros/humble/setup.bash
ros2 bag info lidar_calib_imu_YYYYMMDD_HHMM     # 4 topics, sane message counts

# Timing sanity (no ROS needed) — reuse the diagnostic on the new bag:
python3 inspect_bag_timing.py --bag lidar_calib_imu_YYYYMMDD_HHMM --config config.yaml
```
Expect ~10 Hz clean LiDAR rates and the same "different header epochs" pattern as
before (that's fine — `header_aligned` handles the Hesai; FAST-LIO timestamps the
Livox).

## F. Hand back to processing

1. Send me the CustomMsg type from step B — I unblock the FAST-LIO build.
2. Then follow `FASTLIO_GUIDE.md`: Hesai via `run_odometry.py --only hesai`, Livox
   via FAST-LIO + `fastlio_odom_to_tum.py`, then `solve_extrinsic.py` /
   `validate.py` / `diagnose_trajectories.py`.

Success check: `diagnose_trajectories.py` angular-speed correlation > ~0.95 (was
0.16), and `validate.py` verdict GOOD.
