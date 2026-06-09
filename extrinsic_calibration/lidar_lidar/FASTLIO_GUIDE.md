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

## 1. Re-record (one new ~5 min walk)

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

## 2. Build FAST-LIO2 on the Jetson

```bash
mkdir -p ~/fastlio_ws/src && cd ~/fastlio_ws/src
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
cd ~/fastlio_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

> **The one integration risk to check first (CustomMsg package).** The Avia is an
> SDK1 sensor, so the `livox_ros2_avia` driver publishes its CustomMsg from the
> `livox_interfaces` package. FAST_LIO_ROS2 expects `livox_ros_driver2/CustomMsg`.
> These have the same fields but are different message *packages*, so FAST-LIO
> won't subscribe to the wrong one. Check the actual type with
> `ros2 topic info -v /livox/lidar`, then either:
> - build FAST-LIO against the package your driver uses (point its CustomMsg
>   include/dependency at `livox_interfaces`), **or**
> - run a small relay that republishes `livox_interfaces/CustomMsg` as
>   `livox_ros_driver2/CustomMsg` (identical fields) and point FAST-LIO at that.
>
> This is the only part I couldn't pre-solve from here; tell me which CustomMsg
> type `topic info` reports and I'll hand you the exact one-file fix.

## 3. Run FAST-LIO on the bag, record its odometry

Copy `fast_lio_avia.yaml` (in this folder) into the FAST-LIO `config/` dir (or pass
its path). Then, on bag playback:

```bash
# terminal 1
ros2 launch fast_lio mapping.launch.py config_file:=fast_lio_avia.yaml
# terminal 2
ros2 bag record -o livox_odom /Odometry
# terminal 3
ros2 bag play lidar_calib_imu
```
Let it run to the end, then stop the recorder. Sanity-check in RViz that the Livox
map looks rigid (walls stay put), not smeared.

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
