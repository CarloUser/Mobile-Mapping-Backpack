# Camera &harr; LiDAR Extrinsic Calibration — Handoff / Context

This is a pick-up document for whoever builds the camera&harr;LiDAR stage. The
LiDAR&harr;LiDAR stage (in `../lidar_lidar/`) is being finished separately; read
`../CALIBRATION_STRATEGY.md` first for the whole-system plan, then this for the
camera-specific detail.

Goal: express every camera in the common body frame `base_link`, with the
**Hesai JT128 as the fixed reference**. CAD seeds live in
`../../ros2_ws/src/mmb_bringup/config/extrinsics_initial.yaml`; refined values are
written to `extrinsics_calibrated.yaml` (never overwrite the CAD file).

## 1. The geometry that drives everything

The two LiDARs face opposite directions; the cameras are split front/rear. What
actually shares a field of view (and can therefore see a common target):

| Camera | Direction | Calibrate against | ROS frame (CAD) |
| --- | --- | --- | --- |
| OAK-4D | front | **Hesai** (+ Insta360) | `camera_oak4d` |
| OAK-1 | rear | **Livox** (+ Insta360) | `camera_oak1` |
| OAK-D Lite | rear | **Livox** (+ Insta360) | `camera_oakd_lite` |
| Intel RealSense | rear | **Livox** (+ Insta360) | `camera_realsense` |
| Insta360 (360 cam) | omni | **everything** (hub) | `camera_insta360` |

Consequences:
- The **rear cameras can only reach the reference through the Livox**: calibrate
  each to the Livox, then chain through the (separately solved) Hesai&harr;Livox
  transform. Their accuracy therefore depends on the LiDAR&harr;LiDAR result.
- The **Insta360 sees everything**, so it is the natural hub for loop closures and
  a final global optimization.

## 2. Reference frame and chaining

`base_link -> lidar_hesai` is fixed from CAD. For each camera we solve a pairwise
transform to a LiDAR, then chain:

- Front: `base_link -> camera_oak4d = (base_link->lidar_hesai) @ T_hesai_oak4d`
- Rear:  `base_link -> camera_X = (base_link->lidar_hesai) @ T_hesai_livox @ T_livox_X`
  where `T_hesai_livox` comes from `../lidar_lidar/` (the calibrated value, NOT the
  CAD estimate).

So the rear-camera results are only as good as the Hesai&harr;Livox calibration —
**wait for a trusted `extrinsics_calibrated.yaml` (lidar_lidar verdict GOOD)
before trusting any rear-camera chain.** Intrinsics and the raw camera&harr;Livox
pairwise solves can proceed in parallel; only the final chaining waits.

## 3. Method — camera &harr; LiDAR (target-based)

Use a **ChArUco board** (checkerboard + ArUco markers; robust to partial views and
pose-unambiguous). Per board pose:

- **Camera side:** detect the board, `cv2.solvePnP` &rarr; board pose in the camera
  frame &rarr; board plane (unit normal `n_c`, offset `d_c` with `n_c . X + d_c = 0`
  for points X on the board in camera coords). Orient `n_c` toward the camera.
- **LiDAR side:** segment the board's planar patch in the cloud (RANSAC plane in a
  user-defined ROI, or the dominant plane after removing ground/walls) &rarr; plane
  normal `n_l` and centroid `c_l`. Orient `n_l` toward the sensor.

Across many board orientations (&ge; ~10, well-spread tilts/distances), the plane
correspondences fix the full 6-DoF:
- **Rotation** `R_cl`: align the LiDAR normals to the camera normals via Kabsch
  (`n_c ~ R_cl n_l`). Needs &ge; 3 non-parallel board normals.
- **Translation** `t_cl`: least squares on `n_c . t_cl = -(d_c + n_c . R_cl c_l)`
  over poses.
- Optional nonlinear refine: minimise point-to-plane (LiDAR board points, mapped to
  the camera board plane) over all poses.

This is the same plane-correspondence idea used cross-modally; the printed pattern
is irrelevant to the LiDAR (it only sees the board's geometry) — what matters is a
large, rigid, flat board held at many tilts/positions inside the shared FOV.

Solve `T_cam_lidar` (X_cam = T_cam_lidar X_lidar); the trajectory/chaining wants
`T_lidar_cam = inv(T_cam_lidar)`.

## 4. Method — camera &harr; camera (Insta360 loop closures, optional but recommended)

When two cameras see the **same board in the same frame**, each `solvePnP` gives the
board pose in its own frame, and `T_a_b = T_a_board @ inv(T_b_board)`, averaged
robustly over frames. The Insta360 overlaps every camera, so it ties the front and
rear groups together and over-constrains the graph. Feed all pairwise edges
(camera-LiDAR backbone + Insta360 loop closures) into a final global SE(3)
pose-graph optimisation, fixing `lidar_hesai`, to distribute error and get a
consistency metric.

## 5. Insta360 specifics

It is a 360/dual-fisheye camera; calibrate its **intrinsics with a fisheye /
omnidirectional model** (`cv2.fisheye`), not the pinhole model used for the
OAK/RealSense cameras. Confirm how the driver exposes the image (equirectangular vs
dual-fisheye) and pick the matching model. It now runs in ROS on the PC and Jetson.

## 6. Order of operations

1. **Camera intrinsics first** — non-negotiable; extrinsic error and intrinsic
   error are inseparable. ChArUco intrinsics for OAK-4D, OAK-1, OAK-D Lite,
   RealSense (pinhole); fisheye for Insta360. There are existing intrinsic scripts
   in `../../Camera_Calibration_Jakob/` — confirm/refresh them.
2. **Camera &harr; LiDAR backbone:** OAK-4D&rarr;Hesai; {OAK-1, OAK-D Lite,
   RealSense}&rarr;Livox; Insta360&rarr;Hesai.
3. **Insta360 loop closures** (camera&harr;camera).
4. **Global pose-graph optimisation**, fixing `lidar_hesai`.
5. Chain everything to `base_link`, write `extrinsics_calibrated.yaml`, validate.

## 7. Recording requirements

- Drivers/topics (confirm exact names with `ros2 topic list` when running):
  OAK-4D `/oak4d/rgb/image_raw` + `/oak4d/rgb/camera_info`; OAK-D Lite
  `/oakd_lite/rgb/image_raw` + info; RealSense `/realsense/color/image_raw` + info;
  Insta360 (confirm driver topic); plus the LiDAR clouds `/lidar_points`,
  `/livox/lidar`.
- Per camera-LiDAR pair: hold the board **static** at each of ~15-25 poses (3-5 s
  each) spanning the shared FOV — varied distance, height, tilt, and left/right.
  This is the OPPOSITE motion profile to the LiDAR&harr;LiDAR walk (which wanted
  continuous rotation), so record a **dedicated** camera-calibration bag, not the
  LiDAR walk bag.
- Print a known-geometry ChArUco board on a stiff flat panel; record its exact
  square/marker sizes and dictionary.

## 8. What already exists you can build on

- **`board.py` (prototyped + unit-tested, not yet committed here):** a
  `BoardDetector` using OpenCV 4.13 `cv2.aruco.CharucoDetector` that returns board
  pose, plane `(normal, offset)`, and 3D corners in the camera frame. Verified on
  synthetic rendered boards to ~0.1-0.3 deg / ~1 mm pose recovery. Ask Carlo (or
  check chat history) for the file — it's the natural starting point for the camera
  side. Install: `pip install opencv-contrib-python` (aarch64 wheels exist).
- **Reusable patterns in `../lidar_lidar/`:**
  - `io_utils.py` — TUM I/O and, importantly, `load_extrinsics()` /
    `write_calibrated_extrinsics()` for the repo's extrinsics-YAML format. Reuse
    `write_calibrated_extrinsics` so camera results land in the same file/format.
  - `run_odometry.py` / `fastlio_odom_to_tum.py` — examples of reading a ROS 2 bag
    with `rosbags` + the `ROS2_HUMBLE` typestore (no ROS sourcing needed) and
    deserialising `PointCloud2` / `Imu` / `Odometry`. Copy that bag-reading idiom
    for reading `Image` + `CameraInfo` + `PointCloud2`.
  - `handeye.py` — `se3`, `inv`, Kabsch rotation solve (`solve_rotation`) you can
    reuse for the plane-normal alignment.
  - `config.yaml`, `README.md` — structure to mirror.

## 9. Suggested folder layout to build

```
camera_lidar/
  CONTEXT.md            <- this file
  board.py              <- ChArUco detection (prototype available)
  plane_solve.py        <- normal-align + offset LS + nonlinear refine
  io_cam.py             <- read Image/CameraInfo/PointCloud2 from a bag
  calibrate_cam_lidar.py<- per-pose extraction -> solve -> write yaml
  camera_camera.py      <- Insta360 loop closures
  global_optimize.py    <- pose-graph over all edges (final step)
  config.yaml           <- board params, topics, ROI, thresholds
  requirements.txt      <- numpy scipy opencv-contrib-python rosbags pyyaml
  README.md
```

## 10. Validation

- Reproject LiDAR points into each camera image using the solved transform; edges
  (board boundary, scene structure) should line up.
- Per-pose plane residuals: LiDAR board points should lie on the camera-defined
  plane to a few mm after solving.
- Global step: loop-closure residual via the Insta360 (front&harr;rear) should be
  small (fraction of a degree / few mm).

## 11. Status snapshot (as of handoff)

- LiDAR&harr;LiDAR: motion-based hand-eye built and working in synthetic tests;
  the real Livox odometry from KISS-ICP failed (narrow FOV) and is being redone
  with FAST-LIO (`../lidar_lidar/FASTLIO_GUIDE.md`). The Hesai&harr;Livox
  `extrinsics_calibrated.yaml` is therefore NOT final yet — rear-camera chaining
  must wait for it.
- Reference frame, CAD seeds, and the extrinsics-YAML format are settled.
- Open question: confirm `Oak_4_light` in CAD really is the OAK-D Lite (see
  `../RESUME_CONTEXT.md`), and confirm the Insta360 driver image format + topics.

## 12. References

- Plane-based camera-LiDAR: Zhang & Pless, "Extrinsic calibration of a camera and
  laser range finder," IROS 2004.
- Target-less option to consider instead of a board: Koide et al.,
  `direct_visual_lidar_calibration` (2023) — could reuse a single scene recording.
- ChArUco: OpenCV `cv2.aruco` (use the 4.7+ `CharucoDetector` API).
