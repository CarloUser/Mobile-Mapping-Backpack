# Jetson Handoff — Livox FAST-LIO odometry & finishing the LiDAR↔LiDAR calibration

Context file for an LLM agent (Claude Code CLI) running **locally on the Nvidia
Jetson**. Goal: build FAST-LIO with the CustomMsg fix, produce the Livox
trajectory, and finish the Hesai↔Livox extrinsic calibration. Read this whole file
first, then execute the task list at the bottom.

---

## 1. The project

**Mobile Mapping Backpack** — a wearable multi-sensor rig for 3D mapping. Sensors:

| Sensor | Role | Key ROS 2 topics |
| --- | --- | --- |
| Hesai JT128 | spinning LiDAR, **front-facing**, the **fixed reference frame** | `/lidar_points` (PointCloud2), `/lidar_imu` (Imu) |
| Livox Avia | solid-state LiDAR, narrow ~70° FOV, **rear-facing** | `/livox/lidar` (**livox_interfaces/msg/CustomMsg**), `/livox/imu` (Imu) |
| OAK-4D / OAK-1 / OAK-D-Lite | RGB-D cameras (front/rear) | (camera stage, separate) |
| Intel RealSense, Insta360 | depth + 360 cam | (camera stage, separate) |
| Xsens IMU, u-blox GNSS | global pose aiding | — |

Compute: **Nvidia Jetson, aarch64, Ubuntu 22.04, ROS 2 Humble**. Driver workspace:
`~/ros2_ws` (built via `bash/build_sensors.sh`), provides `hesai_ros_driver`,
`livox_ros2_avia`, and the **`livox_interfaces`** message package.

Repo: `github.com/CarloUser/Mobile-Mapping-Backpack`. On the Jetson it is checked
out locally (adjust paths in the tasks to wherever it lives, e.g.
`~/Documents/Mobile-Mapping-Backpack` or `~/Mobile-Mapping-Backpack`).

**The big task: extrinsic calibration** — express every sensor in one body frame
(`base_link`) with the **Hesai as the fixed reference**. It proceeds in stages:
LiDAR↔LiDAR (this file) → camera↔LiDAR (separate, see
`../camera_lidar/CONTEXT.md`). CAD provides initial guesses; calibration refines
them.

---

## 2. The LiDAR↔LiDAR method (motion-based hand-eye)

The two LiDARs **face opposite directions and do not share any FOV**, so no common
target can be seen. Instead we calibrate from **motion**: walk the rig around, run
odometry independently per LiDAR, and solve the classic hand-eye equation

```
A_i · X = X · B_i        X = T_hesai_livox  (constant rigid transform between them)
```

where `A_i` = Hesai relative motion between two times, `B_i` = Livox relative
motion over the same interval. Stacking many motions fixes the full 6-DoF `X`.
`X` is then chained onto the CAD `base_link→lidar_hesai` to get the refined
`base_link→lidar_livox_avia`.

**Observability:** the solve needs rotation about **all three axes**. Yaw-only
walking is degenerate (roll/pitch unconstrained). The recording walk therefore
includes figure-eights + ramp/stairs (pitch) + deliberate tilt/roll.

**The odometry split (important):**
- **Hesai → KISS-ICP.** Wide FOV, tracks fine with point-to-point ICP. Already done.
- **Livox → FAST-LIO (IMU-aided).** Pure ICP (KISS-ICP) **failed** on the Avia's
  narrow FOV — its trajectory diverged (≈1500 m path over a ≈130 m walk, angular-
  speed correlation with Hesai only 0.16). FAST-LIO fuses the Livox IMU and tracks
  the narrow FOV. **Building + running FAST-LIO is the remaining work.**

---

## 3. Current status

- ✅ Hand-eye solver, bag-reading, solve, validation, diagnostics — all built and
  unit-tested (synthetic: exact recovery on clean data, correctly fails on yaw-only).
- ✅ Recording done: **`~/recordings/lidar/lidar_calib_imu_20260609_190844`**
  (153.7 s). Contains all four topics: `/lidar_points` (1532), `/lidar_imu` (64654),
  `/livox/lidar` (3073, **livox_interfaces/msg/CustomMsg**), `/livox/imu` (31417).
- ✅ Hesai trajectory built: **`out/hesai_tum.txt`** (1532 scans, `header_aligned`).
- ⏳ **TODO: Livox trajectory via FAST-LIO**, then solve + validate. ← this handoff.
- 🔜 Camera↔LiDAR stage is a separate effort; do not start it here. Rear-camera
  results depend on a **GOOD** Hesai↔Livox result, so finishing this unblocks it.

---

## 4. Files in `extrinsic_calibration/lidar_lidar/`

Pipeline (Python, deps: numpy, scipy, pyyaml, rosbags, kiss-icp — all aarch64-OK):
- `handeye.py` — core solver: SE(3) helpers, SLERP pose interpolation,
  `motion_pairs`, Kabsch rotation solve, translation LS, `rotation_observability`,
  `solve_handeye_robust` (trims worst 15% over 3 iters), `residuals`. Unit-tested.
- `io_utils.py` — TUM trajectory I/O + the repo's extrinsics-YAML format
  (`load_extrinsics`, `write_calibrated_extrinsics`). `load_tum` sorts + drops
  non-increasing timestamps.
- `run_odometry.py` — KISS-ICP per topic → TUM, reads the bag with `rosbags` +
  an explicit ROS 2 Humble typestore (no ROS sourcing needed). Has `--only
  {both,hesai,livox}`; **use `--only hesai`** (Livox comes from FAST-LIO).
- `fastlio_odom_to_tum.py` — **verified adapter**: reads `nav_msgs/Odometry` from
  the FAST-LIO odom bag and composes the Avia LiDAR→IMU extrinsic so the output
  trajectory is in the Livox **LiDAR** frame (FAST-LIO reports the IMU-body pose).
  Defaults `--ext-t [0.04165,0.02326,-0.0284] --ext-r identity` (match the yaml).
- `solve_extrinsic.py` — loads both TUM trajectories, time-aligns (interpolates
  Livox onto Hesai stamps), forms motion pairs, observability gate, robust solve,
  chains to `base_link`, compares to CAD, writes `extrinsics_calibrated.yaml`.
- `validate.py` — residual distribution + two-fold cross-validation + plot;
  prints **`verdict: GOOD`** or `CHECK MOTION / SYNC`.
- `diagnose_trajectories.py` — compares the two trajectories; the key health check
  is **angular-speed correlation > ~0.95** (was 0.16 with the broken Livox odom).
- `inspect_bag_timing.py` — per-topic header vs bag-receive timing diagnostic.
- `config.yaml` — topics, `ros_distro: humble`, `timestamp_source: header_aligned`,
  pairing (stride 5, min_angle_deg 3.0, min_observability 0.10), CAD paths.
- `fast_lio_avia.yaml` — FAST-LIO params for the Avia (proper ROS 2
  `/**: ros__parameters:` layout; lid_topic `/livox/lidar`, imu_topic `/livox/imu`,
  lidar_type 1, scan_line 6, blind 1.0, extrinsic_est_en false, extrinsic_T/R Avia).
- `fastlio_patch_livox_interfaces.sh` — one-shot patch (renames
  `livox_ros_driver2`→`livox_interfaces` across FAST-LIO source). Tested.
- `FASTLIO_GUIDE.md` — the full build+run guide this task list condenses.

**Output convention:** trajectories in `out/`. Calibrated extrinsics go to
`../../ros2_ws/src/mmb_bringup/config/extrinsics_calibrated.yaml`.

---

## 5. Critical gotchas (read before running)

1. **FAST-LIO CustomMsg mismatch.** FAST-LIO is hard-coded to
   `livox_ros_driver2/msg/CustomMsg`; this rig publishes
   `livox_interfaces/msg/CustomMsg`. Fields are **identical**
   (`header, timebase, point_num, lidar_id, rsvd, points[]{offset_time,x,y,z,
   reflectivity,tag,line}`), so the fix is a pure package rename done by
   `fastlio_patch_livox_interfaces.sh`. No relay needed.
2. **Wrong branch trap.** The ROS 2 code is on the **`ros2`** branch of
   `Ericsii/FAST_LIO_ROS2`. `main` is the old ROS 1/catkin version and will not
   `colcon build`. Always `git clone -b ros2 --recursive`.
3. **Source `~/ros2_ws` before building FAST-LIO** so `find_package(livox_interfaces)`
   resolves. And **`rosdep ... --skip-keys livox_interfaces`** (rosdep doesn't know
   this local interface package).
4. **Two sensor clocks.** Hesai and Livox use different header epochs. The Hesai
   KISS-ICP path handles this with `timestamp_source: header_aligned`. FAST-LIO
   stamps the Livox trajectory with its own (Livox) clock; `solve_extrinsic.py`
   time-aligns the two by interpolation, and hand-eye uses **relative** motions so a
   constant offset is tolerated. Do not "fix" this further.
5. **QoS.** LiDAR/IMU sensor data is published **best-effort**; `ros2 topic echo`
   defaults to reliable (incompatible). Use `--qos-reliability best_effort` when
   spot-checking a live topic.
6. **tmux for long ops** over SSH so a Wi-Fi drop doesn't kill the build/run.
   Detach with `Ctrl-b d`, reattach `tmux attach -t <name>`. **Ctrl-C kills the
   process** — only use it to deliberately stop a recorder/node.
7. **Never overwrite** `extrinsics_initial.yaml` or `coordinate_systems.yaml`
   (CAD source of truth). Refined values only go to `extrinsics_calibrated.yaml`.
8. **A prior recording hang:** `record_lidar_calib.sh`'s `ros2 bag record` once hung
   at startup; manual recording worked. Not relevant to this task (bag exists), but
   be aware if you ever re-record.
9. **Git:** commit/push is done by the human from their Windows PC. Don't expect to
   push from the Jetson unless credentials are set up; leave changes in the tree and
   tell the user.

---

## 6. Success criteria

- FAST-LIO runs through the whole bag without diverging; `/Odometry` has a few
  thousand messages; the live map (if viewed) looks rigid (walls don't smear).
- `diagnose_trajectories.py`: **angular-speed correlation > ~0.95** and the two
  per-sensor path lengths roughly agree (were 128 m vs 1500 m when broken).
- `solve_extrinsic.py`: rotation-observability above the gate (no planar-motion
  warning), small residuals, correction-vs-CAD that is plausible (not tens of
  degrees / tens of cm).
- `validate.py`: **`verdict: GOOD`** (median residual rotation < 0.5°, translation
  < 2 cm, cross-validation agreement < 0.5° / 2 cm).
- If any check fails, the likely cause is motion (too planar — re-record with more
  pitch/roll) or a topic/clock issue, not the solver.

---

## 7. TASK LIST — execute on the Jetson (Claude Code CLI)

Assumes the repo is at `$REPO` (find it: `find ~ -maxdepth 4 -name lidar_lidar -path '*extrinsic_calibration*' 2>/dev/null`).
Set once per shell: `LL=$REPO/extrinsic_calibration/lidar_lidar`.
The recorded bag is `BAG=~/recordings/lidar/lidar_calib_imu_20260609_190844`.

### Task 1 — Confirm the message type & fields
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 interface show livox_interfaces/msg/CustomMsg
ros2 interface show livox_interfaces/msg/CustomPoint
```
Expect CustomPoint fields exactly: `offset_time, x, y, z, reflectivity, tag, line`.
If any field NAME differs, the rename alone is insufficient (C++ reads fields by
name in `preprocess.cpp`) — STOP and report the difference.

### Task 2 — Clone FAST-LIO (ros2 branch) and patch it
```bash
mkdir -p ~/fastlio_ws/src && cd ~/fastlio_ws/src
git clone -b ros2 --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git
cd ~/fastlio_ws/src/FAST_LIO_ROS2
bash "$LL/fastlio_patch_livox_interfaces.sh"
```
The script prints how many references it renamed in each of package.xml,
CMakeLists.txt, preprocess.h/.cpp, laserMapping.cpp and verifies none remain.

### Task 3 — Build FAST-LIO (use tmux; ~10–20 min on the Jetson)
```bash
cd ~/fastlio_ws
rosdep install --from-paths src --ignore-src -y --skip-keys livox_interfaces
colcon build --symlink-install
source install/setup.bash
```
If the build fails on a missing `livox_interfaces` header/typesupport, the driver
workspace wasn't sourced before building — re-source `~/ros2_ws/install/setup.bash`
and rebuild. Report any compile error verbatim.

### Task 4 — Run FAST-LIO over the bag, record its odometry (3 tmux panes)
Each pane: `source /opt/ros/humble/setup.bash && source ~/fastlio_ws/install/setup.bash`
```bash
# pane 1 — FAST-LIO headless
ros2 launch fast_lio mapping.launch.py \
    config_path:="$LL" config_file:=fast_lio_avia.yaml rviz:=false

# pane 2 — record only the odometry
ros2 bag record -o ~/recordings/lidar/livox_odom /Odometry

# pane 3 — replay the calibration bag into FAST-LIO
ros2 bag play "$BAG"
```
When pane 3 finishes, Ctrl-C pane 2 then pane 1. Verify:
```bash
ros2 bag info ~/recordings/lidar/livox_odom    # /Odometry, a few thousand msgs
```
Red flags: pane 1 logs `Failed to find match for field 'time'` (cloud not CustomMsg
/ no per-point time) or `/Odometry` ≈ 0 msgs, or positions diverging. Report and stop.

### Task 5 — Build the Livox trajectory (TUM)
```bash
cd "$LL"
python3 fastlio_odom_to_tum.py --bag ~/recordings/lidar/livox_odom \
    --topic /Odometry --out out/livox_tum.txt
```
(`out/hesai_tum.txt` already exists from KISS-ICP. If it's missing, regenerate:
`python3 run_odometry.py --bag "$BAG" --config config.yaml --out ./out --only hesai`.)

### Task 6 — Diagnose, solve, validate
```bash
cd "$LL"
python3 diagnose_trajectories.py --hesai-traj out/hesai_tum.txt --livox-traj out/livox_tum.txt --config config.yaml
python3 solve_extrinsic.py       --hesai-traj out/hesai_tum.txt --livox-traj out/livox_tum.txt --config config.yaml
python3 validate.py              --hesai-traj out/hesai_tum.txt --livox-traj out/livox_tum.txt --config config.yaml
```
PASS = angular-speed correlation > ~0.95 (diagnose) AND `verdict: GOOD` (validate).
`solve_extrinsic.py` writes `../../ros2_ws/src/mmb_bringup/config/extrinsics_calibrated.yaml`.

### Task 7 — Report back
Summarize: build outcome, `/Odometry` msg count, the three diagnostic/solve/validate
outputs (correlation, residuals, correction-vs-CAD, verdict), and the solved
`base_link→lidar_livox_avia` xyz + quaternion. Do **not** git push (the human does
that from their PC); list the files changed/created so they can commit.

### If FAST-LIO underperforms (fallback)
If FAST-LIO won't converge or the verdict stays CHECK after a clearly good walk,
the fallback is a **pure-Python gyro-aided Livox odometry** (no ROS build): read
`/livox/lidar` CustomMsg per-point times + `/livox/imu`, deskew, take relative
rotation from the gyro (Avia IMU↔LiDAR rotation is identity), solve translation via
point-to-plane ICP, accumulate, write `out/livox_tum.txt`. The CustomMsg is readable
in pure Python by registering `livox_interfaces/CustomMsg` + `CustomPoint` in the
`rosbags` typestore (verified). Mention this option to the user before building it.
