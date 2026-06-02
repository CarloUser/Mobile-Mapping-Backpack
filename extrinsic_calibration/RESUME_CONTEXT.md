# Resume Context: Mobile Mapping Backpack Extrinsic Calibration

Last updated: 2026-06-02 (sensor driver setup scripts now merged from `mazi`)

This file is a handoff summary for resuming work on the mobile mapping backpack
extrinsic calibration setup.

## User Goal

Set up the repository so extrinsic calibration can be completed by running
well-defined scripts/launch steps after collecting calibration data. The user is
building a multi-sensor mobile mapping backpack and wants all sensors calibrated
relative to a stable `base_link`.

## Key Decisions Already Made

- `base_link` is the center of the rigid sensor-mount frame, exported from
  SolidWorks.
- `base_link` should use normal ROS body-frame semantics:
  - x forward
  - y left
  - z up
- The exact `base_link` point is not mathematically special; the important part
  is that it is rigid, documented, repeatable, and used consistently.
- Do not use a camera optical frame as `base_link`.
- Blickfeld LiDAR is not used.
- Livox Avia LiDAR is used and interfaces over GbE.
- Insta360 is used and has a ROS driver.
- Livox drivers are expected to be pushed/added soon.
- Current CAD transforms are only initial estimates. Final calibrated values
  should be saved separately, not by overwriting the CAD initial estimate file.

## Current Sensor Scope

Sensors in scope:

- HESAI JT128 LiDAR
  - ROS frame: `lidar_hesai`
  - Confirmed PointCloud2 topic: `/lidar_points`
- Livox Avia LiDAR
  - ROS frame added in initial extrinsics: `lidar_livox_avia`
  - Confirmed PointCloud2 topic: `/livox/lidar`
  - Confirmed IMU topic: `/livox/imu`
  - Driver package/launch: `livox_ros2_avia`, `livox_lidar_launch.py`
- OAK-4D Luxonis RGB-D + IMU
  - ROS frame: `camera_oak4d`
  - Existing topics: `/oak4d/rgb/image_raw`, `/oak4d/depth/image_raw`,
    `/oak4d/rgb/camera_info`, `/oak4d/imu/data`
- OAK-D Lite Luxonis RGB-D
  - ROS frame: `camera_oakd_lite`
  - Existing topics: `/oakd_lite/rgb/image_raw`,
    `/oakd_lite/depth/image_raw`, `/oakd_lite/rgb/camera_info`
  - SolidWorks source frame is currently named `Oak_4_light`; confirm this is
    truly the OAK-D Lite.
- OAK-1
  - ROS frame added in initial extrinsics: `camera_oak1`
  - It appears in intrinsic calibration scripts, but not yet in ROS bringup.
  - Confirm whether it is physically mounted and needs driver/recording support.
- Intel RealSense D4xx RGB-D
  - ROS frame: `camera_realsense`
  - Existing topics: `/realsense/color/image_raw`,
    `/realsense/depth/image_rect_raw`, `/realsense/color/camera_info`
- Xsens MTi-610R IMU
  - ROS frame: `imu_xsens`
  - Existing topics: `/imu/data`, `/imu/mag`
- u-blox GNSS + ANN-MB-00 antenna
  - ROS frame: `gnss_antenna`
  - Existing topics: `/gnss/fix`, `/gnss/fix_velocity`
  - Treat rotation as not physically meaningful for a single antenna; translation
    is the lever arm.
- Insta360
  - ROS frame added in initial extrinsics: `camera_insta360`
  - User said it has a ROS driver.
  - Confirm actual driver frame/topic names and either match them or add a bridge
    static transform.

Sensors explicitly out of scope:

- Blickfeld LiDAR

## Source CAD Export

The user exported SolidWorks coordinate systems to:

`Coordinate_systems/coordinate_systems.yaml`

The export says:

- poses are sensor frames expressed in `base_link`
- translation units are meters
- quaternion order is x, y, z, w, matching ROS/tf2
- RPY convention is ZYX intrinsic, but the generated ROS config uses quaternions

The current source frames are:

- `Oak_4`
- `GNSS_Antenna`
- `Livox_Avia`
- `Hesai`
- `Intel_Realsense`
- `Oak_4_light`
- `Oak_1`
- `Xsens_IMU`
- `Insta_360`

## Generated ROS Extrinsics Config

Created:

`ros2_ws/src/mmb_bringup/config/extrinsics_initial.yaml`

It maps SolidWorks names to ROS frame names:

| SolidWorks frame | ROS child frame |
| --- | --- |
| `Oak_4` | `camera_oak4d` |
| `GNSS_Antenna` | `gnss_antenna` |
| `Livox_Avia` | `lidar_livox_avia` |
| `Hesai` | `lidar_hesai` |
| `Intel_Realsense` | `camera_realsense` |
| `Oak_4_light` | `camera_oakd_lite` |
| `Oak_1` | `camera_oak1` |
| `Xsens_IMU` | `imu_xsens` |
| `Insta_360` | `camera_insta360` |

Current transforms in `extrinsics_initial.yaml`:

```text
base_link -> camera_oak4d
xyz = [0.00926604120012522, 0.04022890058414, -0.0487610765723726]
q_xyzw = [0, 0, 0, 1]

base_link -> gnss_antenna
xyz = [0.00740394746195187, 0.121672119427252, 0.0952111214036253]
q_xyzw = [0, 0, 0, 1]

base_link -> lidar_livox_avia
xyz = [0.00615953452791859, -0.0261595830828822, -0.0639745409273749]
q_xyzw = [-3.49148133884313e-15, -3.49148133884313e-15, 1.0, -1.21904419394898e-29]

base_link -> lidar_hesai
xyz = [0.00825335383692801, 0.0507170454616155, 0.0965158453101299]
q_xyzw = [-0.707106781186549, 0, 0, 0.707106781186546]

base_link -> camera_realsense
xyz = [-0.0500226269231557, 0.106536011222252, 0.0951690299822231]
q_xyzw = [-3.49148133884313e-15, -3.49148133884313e-15, 1.0, -1.21904419394898e-29]

base_link -> camera_oakd_lite
xyz = [-0.0476411677891182, 0.113285974499897, -0.07604038256127]
q_xyzw = [0, 1.0, 0, 3.49148133884313e-15]

base_link -> camera_oak1
xyz = [-0.052987960674065, 0.117585651088969, 0.00942191946353569]
q_xyzw = [0, 1.0, 0, 3.49148133884313e-15]

base_link -> imu_xsens
xyz = [0.00293007514840342, 0.0981721194272516, -0.0992596717526523]
q_xyzw = [0, 0, 0, 1]

base_link -> camera_insta360
xyz = [0.001393353836928, 0.273672119427252, -0.00852001287943149]
q_xyzw = [0, 0, 0, 1]
```

## Repository Changes Made So Far

Added:

- `ros2_ws/src/mmb_bringup/config/extrinsics_initial.yaml`
- `ros2_ws/src/mmb_bringup/launch/static_extrinsics.launch.py`
- `extrinsic_calibration/README.md`
- `extrinsic_calibration/scripts/validate_initial_extrinsics.py`
- `extrinsic_calibration/RESUME_CONTEXT.md`

Modified:

- `Coordinate_systems/coordinate_systems.yaml`
  - User added `Insta_360`.
- `ros2_ws/src/mmb_bringup/launch/all_sensors.launch.py`
  - Replaced hard-coded identity static TFs with an include of
    `static_extrinsics.launch.py`.
- `ros2_ws/src/mmb_bringup/package.xml`
  - Added runtime dependency `python3-yaml`.

These changes are now committed and pushed to
`origin/claude/plan-backpack-project-U5DX0`
(commit "Add initial extrinsic calibration setup ...").

## Driver Integration Status (updated 2026-06-02)

The sensor driver/setup scripts were merged from branch `mazi` into
`claude/plan-backpack-project-U5DX0`. They live in a new top-level `bash/`
folder (see `bash/README.md` and `bash/launch_commands.md`). The merge was
conflict-free; the driver scripts and the extrinsic-calibration files do not
overlap.

Driver ROS packages and launch entry points are now known (from
`bash/launch_commands.md`):

| Sensor | ROS package | Launch / run |
| --- | --- | --- |
| HESAI JT128 | `hesai_ros_driver` | `ros2 launch hesai_ros_driver start.py` |
| Livox Avia | `livox_ros2_avia` | `livox_lidar_launch.py` (PointCloud2) |
| OAK-4D | `depthai_ros_driver_v3` | `driver.launch.py` |
| OAK-D Lite | `depthai_ros_driver_v3` | `driver.launch.py` (shared with OAK-4D) |
| OAK-1 | (not yet defined) | section empty in launch_commands.md |
| Intel RealSense | `realsense_ros2_camera` | `ros2 run realsense_ros2_camera realsense_ros2_camera` |
| Xsens MTi-610R | `xsens_mti_ros2_driver` | `xsens_mti_node.launch.py` |
| u-blox GNSS | `ublox_gps` | `ublox_gps_node-launch.py` |
| Insta360 | (SDK + ws scripts present) | launch command not yet documented |

Still UNCONFIRMED after the merge (verify on the real ROS machine):

- Actual published topic names and message `frame_id`s for every driver
  (the extrinsics config assumes the ROS frame names in the table in
  "Generated ROS Extrinsics Config"; the drivers must publish matching
  `frame_id`s or a bridge static TF is needed).
- OAK-4D and OAK-D Lite both use `depthai_ros_driver_v3` with the same
  `driver.launch.py`; confirm how the two devices are distinguished (device
  id / namespace) so their frames/topics do not collide.
- Insta360 launch file, image/camera_info topics, and `frame_id`.
- OAK-1 driver/launch (section is empty) and whether it is mounted at all.

Important: at the time the calibration files were first created, these changes
were not committed. They are now committed and pushed.

## Static TF Launch Behavior

`static_extrinsics.launch.py` loads `extrinsics_initial.yaml` and creates one
`tf2_ros/static_transform_publisher` node for each entry under `transforms`.

`all_sensors.launch.py` now includes:

- `lidar.launch.py`
- `cameras.launch.py`
- `imu_gnss.launch.py`
- `static_extrinsics.launch.py`

The static extrinsics launch accepts:

```bash
ros2 launch mmb_bringup static_extrinsics.launch.py \
  extrinsics_file:=/path/to/extrinsics_initial.yaml
```

By default it uses the installed package config:

`share/mmb_bringup/config/extrinsics_initial.yaml`

## Validation Script

Created:

`extrinsic_calibration/scripts/validate_initial_extrinsics.py`

It checks:

- required frames are present
- child frames are not duplicated
- each transform uses the configured reference parent
- each translation has 3 values
- each quaternion has 4 values
- quaternion norm is approximately 1.0

Required frames currently:

- `lidar_hesai`
- `lidar_livox_avia`
- `camera_oak1`
- `camera_oak4d`
- `camera_oakd_lite`
- `camera_realsense`
- `camera_insta360`
- `imu_xsens`
- `gnss_antenna`

Last validation command run:

```powershell
python extrinsic_calibration\scripts\validate_initial_extrinsics.py
```

Last result:

```text
Transforms: 9
Validation passed.
```

Syntax checks were also run without writing bytecode:

```powershell
@'
from pathlib import Path
for path in [
    Path('ros2_ws/src/mmb_bringup/launch/all_sensors.launch.py'),
    Path('ros2_ws/src/mmb_bringup/launch/static_extrinsics.launch.py'),
    Path('extrinsic_calibration/scripts/validate_initial_extrinsics.py'),
]:
    compile(path.read_text(encoding='utf-8'), str(path), 'exec')
    print(f'compiled {path}')
'@ | python -
```

Last result:

```text
compiled ros2_ws\src\mmb_bringup\launch\all_sensors.launch.py
compiled ros2_ws\src\mmb_bringup\launch\static_extrinsics.launch.py
compiled extrinsic_calibration\scripts\validate_initial_extrinsics.py
```

## Known Open Questions

1. Confirm `Oak_4_light` is really the OAK-D Lite.
2. Confirm whether OAK-1 is physically mounted and should be launched/recorded.
3. Confirm actual Insta360 ROS driver (package now installed via
   `bash/setup_insta360_*`, but launch file not yet documented):
   - launch file
   - image topics
   - camera info topic
   - frame_id
4. Livox Avia ROS driver is now known to be `livox_ros2_avia`
   (`livox_lidar_launch.py` for PointCloud2). Confirmed topics:
   - `/livox/lidar`
   - `/livox/imu`
   Still confirm:
   - frame_id
5. Confirm whether camera CAD frames are body-style frames or optical frames.
   If they are body-style frames, add separate optical-frame transforms for
   camera projection workflows.
6. Confirm whether HESAI/Livox frame axes from CAD match the driver frame axes.

## Recommended Next Step

Validate the CAD TFs on the real ROS machine before collecting serious
calibration data.

Recommended command sequence:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch mmb_bringup all_sensors.launch.py
```

Then check TFs:

```bash
ros2 run tf2_ros tf2_echo base_link lidar_hesai
ros2 run tf2_ros tf2_echo base_link lidar_livox_avia
ros2 run tf2_ros tf2_echo base_link camera_insta360
ros2 run tf2_ros tf2_echo base_link camera_oak1
```

Then check topics and frame IDs:

```bash
ros2 topic list
ros2 topic echo /tf_static --once
ros2 topic echo /lidar_points --once
ros2 topic echo /livox/lidar --once
ros2 topic echo /livox/imu --once
```

Also check the Livox and Insta360 message header frame IDs on the real ROS
machine.

## Recommended Calibration Sequence

After TF/topic sanity checks:

1. Record a 30-60 second static all-sensor test bag.
   - Goal: catch missing topics, duplicate TFs, timestamp jumps, and frame_id
     mismatches.
2. Collect the first real extrinsic calibration bag.
   - Use an AprilTag or ChArUco target for cameras.
   - Use a large flat/edged backing plane visible to LiDARs.
   - Pause 3-5 seconds per board pose.
   - Cover varied distances, heights, tilts, and left/right FOV regions.
3. First solve:
   - `lidar_hesai <-> lidar_livox_avia`
4. Then solve:
   - each camera to LiDAR/base reference
5. Then refine/check:
   - `imu_xsens` alignment using motion data
   - `gnss_antenna` lever arm during an outdoor RTK run
6. Save refined results as a separate calibrated YAML, for example:
   - `ros2_ws/src/mmb_bringup/config/extrinsics_calibrated.yaml`

## Notes for Future Assistant

- Do not overwrite `Coordinate_systems/coordinate_systems.yaml` unless the user
  explicitly asks; it is the SolidWorks source export.
- Do not overwrite `extrinsics_initial.yaml` with calibrated values. Create a
  calibrated config instead.
- HESAI recorder/calibration support uses the confirmed `/lidar_points` topic.
- Livox recorder support uses the confirmed `/livox/lidar` and `/livox/imu`
  topics.
- Do not add Insta360 recorder topics until the actual driver topic names are
  known.
- The current repo has a dirty worktree from this calibration setup and the
  user's updated CAD YAML. Avoid reverting user edits.
- Use `apply_patch` for manual edits.
- Use PowerShell-compatible commands on this machine.
