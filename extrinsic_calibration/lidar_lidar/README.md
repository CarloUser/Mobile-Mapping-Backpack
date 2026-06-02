# Hesai &harr; Livox Extrinsic Calibration (motion-based, no FOV overlap)

This directory calibrates the rigid transform between the two LiDARs on the
backpack: the **Hesai JT128** (reference) and the **Livox Avia**. You record one
dataset by walking the rig around, process it **on your PC**, and copy a single
calibrated YAML to the Jetson. Nothing here runs on the Jetson except the final
launch that consumes the result.

## Why this method (read this first)

The two LiDARs face opposite directions and **do not share a field of view**, so
no target can be seen by both at once. A checkerboard/board method is therefore
impossible here. The correct technique for non-overlapping LiDARs is
**motion-based hand-eye calibration**:

1. Walk the whole rig through a static, geometry-rich scene.
2. Run LiDAR odometry **independently** on each sensor &rarr; two trajectories.
3. Solve `A_i X = X B_i` for `X = T_hesai_livox`, where `A_i`, `B_i` are the two
   sensors' relative motions between frames. This works precisely *because* the
   two sensors are rigidly attached and move together &mdash; their motions differ
   only by the constant `X` we want.

A LiDAR sees geometry, not the printed pattern, so the checkerboard plays no role
in this step. (Keep your checkerboard/ChArUco board for the later *camera*&harr;LiDAR
calibration, where it is essential.)

Hesai is kept fixed as the reference; we refine only `base_link -> lidar_livox_avia`.

## The single most important requirement: excite all three rotation axes

Hand-eye calibration is **unobservable** if you only rotate about one axis.
Walking on flat ground turning left/right (yaw only) will produce confident-looking
but wrong roll/pitch. Our self-test reproduces this: yaw-only motion gives ~40-70&deg;
error, while 3-axis motion recovers the transform exactly. So during recording you
**must** add pitch and roll:

- Walk a figure-of-eight, not just straight lines.
- Go up and down a ramp or staircase (adds pitch).
- Deliberately tilt/roll the backpack side to side and nod it up/down.
- Vary speed; pause occasionally; keep motions smooth (no violent jerks).

`solve_extrinsic.py` prints a `rotation-observability` number in [0,1]; aim for
well above 0.1. If it warns, your motion was too planar &mdash; recollect.

## Files

| File | Purpose |
| --- | --- |
| `handeye.py` | Core `A X = X B` solver (Kabsch rotation + LS translation, robust trimming). |
| `test_handeye.py` | Unit tests on synthetic data (exact recovery; degenerate case must fail). |
| `run_odometry.py` | Step 1: bag &rarr; two TUM trajectories via KISS-ICP. |
| `solve_extrinsic.py` | Step 2: trajectories &rarr; `extrinsics_calibrated.yaml`. |
| `validate.py` | Step 3: residuals, cross-validation, consistency plot. |
| `io_utils.py` | TUM + extrinsics-YAML I/O. |
| `config.yaml` | Topic names and solver parameters. |

## Prerequisites

- On the **rig/Jetson**: both LiDAR drivers running and publishing
  `sensor_msgs/PointCloud2` (Hesai `start.py`, Livox `livox_lidar_launch.py`).
  Confirm the Livox topic name and set it in `config.yaml` (default `/livox/lidar`).
- Time sync between the two LiDARs should be in place (see
  `docs/setup/03_time_sync.md`); the solver interpolates over small offsets but
  cannot fix large clock skew.
- On the **PC**: `pip install -r requirements.txt`.

## Procedure

### 0. Sanity-check the math (optional, ~1 s)
```bash
python test_handeye.py        # should end with "ALL ASSERTIONS PASSED"
```

### 1. Record a bag on the rig
Only the two point-cloud topics are needed. While recording, follow the motion
guidance above for 2-5 minutes in a feature-rich space (walls, furniture, corners
&mdash; avoid wide-open fields and avoid moving people/vehicles in view).
```bash
ros2 bag record -o lidar_calib_$(date +%Y%m%d_%H%M) \
    /lidar/points /livox/lidar
```

### 2. Copy the bag to your PC
```bash
scp -r <jetson-user>@<jetson-ip>:~/lidar_calib_* .
```

### 3. Per-LiDAR odometry (PC)
```bash
python run_odometry.py --bag ./lidar_calib_YYYYMMDD_HHMM --config config.yaml --out ./out
```
Produces `out/hesai_tum.txt` and `out/livox_tum.txt`. (Internally runs
`kiss_icp_pipeline` once per topic. If you prefer, run KISS-ICP yourself and drop
the two timestamped-TUM files at those paths.)

### 4. Solve the extrinsic (PC)
```bash
python solve_extrinsic.py --hesai-traj out/hesai_tum.txt \
    --livox-traj out/livox_tum.txt --config config.yaml
```
Reads the CAD seed from `ros2_ws/.../extrinsics_initial.yaml`, keeps Hesai fixed,
and writes `ros2_ws/.../extrinsics_calibrated.yaml` with provenance (inlier count,
residuals, and the correction relative to CAD). A correction of a few degrees / a
few cm from the CAD value is normal; a very large one usually means a wrong topic,
a frame-axis mismatch, or bad time sync &mdash; investigate before trusting it.

### 5. Validate (PC)
```bash
python validate.py --hesai-traj out/hesai_tum.txt \
    --livox-traj out/livox_tum.txt --config config.yaml
```
Look for: low residuals (median rotation < ~0.5&deg;, translation < ~2 cm) **and**
tight two-fold cross-validation agreement. The verdict line prints `GOOD` or
`CHECK MOTION / SYNC`. A plot is saved to `out/validation.png`.

### 6. Deploy to the Jetson
Copy only the result; the Jetson already has everything else set up.
```bash
scp ros2_ws/src/mmb_bringup/config/extrinsics_calibrated.yaml \
    <jetson-user>@<jetson-ip>:~/Mobile-Mapping-Backpack/ros2_ws/src/mmb_bringup/config/
```
Then point the static-TF launch at the calibrated file:
```bash
ros2 launch mmb_bringup static_extrinsics.launch.py \
    extrinsics_file:=<...>/config/extrinsics_calibrated.yaml
```
(Or make `extrinsics_calibrated.yaml` the default once you trust it. Do **not**
overwrite `extrinsics_initial.yaml` or `Coordinate_systems/coordinate_systems.yaml`.)

## Troubleshooting

| Symptom | Likely cause / fix |
| --- | --- |
| `rotation-observability` warning, or unstable result | Motion too planar. Add pitch/roll (ramps, stairs, tilting). |
| Few or zero motion pairs | Sequence too short or too slow; lower `min_angle_deg`, raise `stride`, or record longer. |
| Large correction vs CAD | Wrong Livox topic, frame-axis mismatch, or time skew. Verify topics and `03_time_sync.md`. |
| High residuals but good observability | Odometry drift (featureless scene) or clock skew between LiDARs. |
| `kiss_icp_pipeline not found` | `pip install -r requirements.txt`. |

## Scope and next steps

This calibrates **only** Hesai&harr;Livox. With Hesai as the common reference, the
next phases attach the cameras (target-based, ChArUco/AprilGrid &mdash; this is where
the checkerboard returns), then the IMU (motion-based) and the GNSS lever arm.
See `../RESUME_CONTEXT.md` for the full sensor plan.

## Method references

- F. Park, B. Martin, "Robot sensor calibration: solving AX=XB on the Euclidean
  group," IEEE T-RA, 1994.
- K. Daniilidis, "Hand-eye calibration using dual quaternions," IJRR, 1999.
- Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP," IEEE RA-L, 2023.
