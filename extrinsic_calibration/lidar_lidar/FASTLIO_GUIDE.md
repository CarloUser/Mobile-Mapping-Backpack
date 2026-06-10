# Livox odometry via FAST-LIO (for the Hesai &harr; Livox calibration)

Pure point-to-point ICP (KISS-ICP) cannot track the Livox Avia's narrow 70&deg;
non-repetitive FOV — its odometry diverged (1500 m path over a 130 m walk). The
fix is IMU-aided odometry for the Livox via **FAST-LIO2**, fusing the Livox
built-in IMU. The **Hesai stays on KISS-ICP** (it tracks fine). Both trajectories
then feed the existing `solve_extrinsic.py` unchanged.

```
re-record (CustomMsg + IMUs)
   |-- Hesai /lidar_points  --> run_odometry.py --only hesai --> out/hesai_tum.txt
   |-- Livox /livox/lidar(CustomMsg)+/livox/imu --> FAST-LIO --> /Odometry
   |                                  --> fastlio_odom_to_tum.py --> out/livox_tum.txt
   '-- solve_extrinsic.py --> extrinsics_calibrated.yaml --> validate.py
```

## 1. Re-record (one new ~5 min walk) — ✅ DONE

> Already recorded: `~/recordings/lidar/lidar_calib_imu_20260609_190844` (153.7 s,
> all four topics; `/livox/lidar` is `livox_interfaces/msg/CustomMsg`, 3073 msgs).
> The Hesai trajectory is already built (`out/hesai_tum.txt`). Skip to **step 2**.
> Re-recording instructions kept below for reference.

FAST-LIO needs **per-point timestamps**, which only the Livox **CustomMsg** format
carries (your previous PointCloud2 had fields `x y z intensity tag line` &mdash; no
time). So launch the Livox driver in CustomMsg mode and record both IMUs:

```bash
# terminal 1: Hesai driver (publishes /lidar_points + /lidar_imu)
ros2 launch hesai_ros_driver start.py

# terminal 2: Livox driver in CustomMsg mode (publishes /livox/lidar as CustomMsg + /livox/imu)
ros2 launch livox_ros2_avia livox_lidar_msg_launch.py

# terminal 3: record
ros2 bag record -o lidar_calib_imu /lidar_points /lidar_imu /livox/lidar /livox/imu
```

Walk the same 3-axis excitation as before (figure-eights, ramps/stairs, deliberate
tilt/roll) for 2-5 minutes in a feature-rich space. Confirm the Livox cloud is now
CustomMsg: `ros2 topic info -v /livox/lidar` should show a `CustomMsg` type, not
`PointCloud2`.

## 2. Build FAST-LIO2 on the Jetson (with the CustomMsg fix)

**Two things the upstream README gets wrong for us:** (a) the ROS 2 code lives on
the **`ros2` branch**, not `main` (`main` is the old ROS 1/catkin version and will
not `colcon build`); (b) FAST-LIO is hard-coded to `livox_ros_driver2/msg/CustomMsg`,
but our `livox_ros2_avia` driver publishes `livox_interfaces/msg/CustomMsg`. Those
two messages are **field-for-field identical** (`header, timebase, point_num,
lidar_id, rsvd, points[]{offset_time,x,y,z,reflectivity,tag,line}`), so the fix is a
pure package rename — done by `fastlio_patch_livox_interfaces.sh` in this folder.
No relay, no second message package.

```bash
# 0. Source the driver workspace FIRST so the build can find livox_interfaces.
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
#    Confirm the message + field names (sanity):
ros2 interface show livox_interfaces/msg/CustomMsg
ros2 interface show livox_interfaces/msg/CustomPoint   # offset_time,x,y,z,reflectivity,tag,line

# 1. Clone the ROS 2 branch (NOT main) with submodules (ikd-Tree).
mkdir -p ~/fastlio_ws/src && cd ~/fastlio_ws/src
git clone -b ros2 --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git

# 2. Patch livox_ros_driver2 -> livox_interfaces (run from the repo root).
cd ~/fastlio_ws/src/FAST_LIO_ROS2
bash ~/Documents/Mobile-Mapping-Backpack/extrinsic_calibration/lidar_lidar/fastlio_patch_livox_interfaces.sh
#    (adjust the path to wherever this repo is checked out on the Jetson)

# 3. Build. rosdep can't resolve the local interface package, so skip it.
cd ~/fastlio_ws
rosdep install --from-paths src --ignore-src -y --skip-keys livox_interfaces
colcon build --symlink-install
source install/setup.bash
```

If `ros2 interface show` reports a CustomPoint field name different from the list
above, the rename alone isn't enough (the C++ in `src/preprocess.cpp` reads those
fields by name). That's unlikely for an SDK1 clone — but send me the output and I'll
give you the exact field-access edits.

## 3. Run FAST-LIO on the bag, record its odometry

The launch takes `config_path` (a directory) + `config_file` (a filename in it).
Point them at this folder's `fast_lio_avia.yaml`, and turn RViz off on the headless
Jetson (`rviz:=false`). FAST-LIO reads from live topics, so we play the recorded bag
into it. **Open 3 tmux panes**, each with ROS sourced
(`source /opt/ros/humble/setup.bash && source ~/fastlio_ws/install/setup.bash`):

```bash
CFG=~/Documents/Mobile-Mapping-Backpack/extrinsic_calibration/lidar_lidar
BAG=~/recordings/lidar/lidar_calib_imu_20260609_190844     # your recorded bag

# pane 1 — FAST-LIO (headless)
ros2 launch fast_lio mapping.launch.py \
    config_path:=$CFG config_file:=fast_lio_avia.yaml rviz:=false

# pane 2 — record only the odometry it emits
ros2 bag record -o ~/recordings/lidar/livox_odom /Odometry

# pane 3 — replay the calibration bag (feeds /livox/lidar + /livox/imu to FAST-LIO)
ros2 bag play "$BAG"
```

Watch pane 1: after a second or two it should print iteration/“IMU Initial” logs and
keep up with playback. When `ros2 bag play` finishes, Ctrl-C pane 2 (stop the
recorder), then pane 1. Quick health check on the odometry bag:

```bash
ros2 bag info ~/recordings/lidar/livox_odom     # /Odometry, a few thousand msgs
```

If pane 1 logs `Failed to find match for field 'time'` or the message count on
`/Odometry` is ~0, the Livox cloud wasn't CustomMsg / per-point time is missing —
re-check step 1. If FAST-LIO diverges (positions explode), reduce `point_filter_num`
or check the bag's IMU is on `/livox/imu`.

## 4. Build the two trajectories

```bash
cd ~/Documents/Mobile-Mapping-Backpack/extrinsic_calibration/lidar_lidar

# Hesai via KISS-ICP (only this sensor now)
python3 run_odometry.py --bag /path/to/lidar_calib_imu --config config.yaml \
    --out ./out --only hesai

# Livox via FAST-LIO odometry -> LiDAR-frame TUM (extrinsic composed automatically)
python3 fastlio_odom_to_tum.py --bag /path/to/livox_odom --topic /Odometry \
    --out out/livox_tum.txt
```

The adapter composes the Livox LiDAR&rarr;IMU extrinsic (FAST-LIO reports the IMU
pose) so the trajectory is in the Livox **LiDAR** frame, matching the CAD
`lidar_livox_avia` frame. Keep `--ext-t/--ext-r` identical to `extrinsic_T/R` in
`fast_lio_avia.yaml` (defaults already match).

## 5. Solve, validate, sanity-check

```bash
python3 solve_extrinsic.py --hesai-traj out/hesai_tum.txt --livox-traj out/livox_tum.txt --config config.yaml
python3 validate.py        --hesai-traj out/hesai_tum.txt --livox-traj out/livox_tum.txt --config config.yaml
python3 diagnose_trajectories.py --hesai-traj out/hesai_tum.txt --livox-traj out/livox_tum.txt --config config.yaml
```

Before trusting the result, run `diagnose_trajectories.py` and confirm the
**angular-speed correlation is now > ~0.95** and the two per-sensor path lengths
roughly agree (they were 128 m vs 1500 m before). Then `validate.py` should print a
small residual and `verdict: GOOD`.

## Notes

- `timestamp_source` in `config.yaml` only affects the Hesai (KISS-ICP) trajectory;
  the Livox trajectory now carries FAST-LIO's own timestamps. The solver still
  time-aligns the two via interpolation.
- If you later bump the Livox IMU rate (it was ~129 Hz; `imu_rate` in
  `livox_lidar_config.json`) FAST-LIO will be a touch more accurate, but 129 Hz is
  fine.
- Do not overwrite `extrinsics_initial.yaml`; results go to
  `extrinsics_calibrated.yaml`.
