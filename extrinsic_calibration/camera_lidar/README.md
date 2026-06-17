# Camera ↔ LiDAR extrinsic calibration

Target-based (ChArUco) plane-correspondence calibration of each camera against
its FOV-sharing LiDAR, chained to `base_link` through the Hesai reference.
Read `CONTEXT.md` for the full method and geometry; this README is operational.

## Files

| file | role |
| --- | --- |
| `board.py` | ChArUco detection → board pose + plane in the camera frame |
| `io_cam.py` | bag reading (Image/CameraInfo/PointCloud2/Livox CustomMsg) + extrinsics-YAML **merge** writer |
| `plane_solve.py` | RANSAC board-patch plane, Kabsch + LS closed form, point-to-plane refine |
| `calibrate_cam_lidar.py` | per-pair orchestrator: detect → holds → patches → solve → chain → write |
| `config.yaml` | board geometry, topics per pair, thresholds |
| `test_camera_lidar.py` | synthetic unit tests (`python3 -m pytest -q`) |

## Workflow

1. **Confirm config.** Topic names in `config.yaml` are placeholders from
   CONTEXT.md §7 — check with `ros2 topic list` while drivers run. Measure the
   printed board and set `board:` exactly (squares, square/marker size,
   dictionary). Camera intrinsics must be good first (`CameraInfo` is used
   directly); refresh with the scripts in `../../Camera_Calibration/` if
   needed.
2. **Record one bag per pair** (camera image + camera_info + that pair's LiDAR
   topic). Hold the board static 3–5 s at 15–25 poses spanning the shared FOV —
   vary distance (1–4 m), height, left/right, and **tilt strongly** (≥ ~30–45°
   in different directions: the rotation solve needs spread normals; the
   pipeline refuses near-parallel sets). Keep the board clear of walls — the
   patch segmentation crops around the CAD-predicted position and rejects
   wall-sized planes, but free space is cheaper than rejected holds.
3. **Run** (no ROS sourcing needed):
   ```bash
   python3 calibrate_cam_lidar.py --bag <bag_dir> --pair oak4d
   python3 calibrate_cam_lidar.py --bag <bag_dir> --pair oak1   # etc.
   ```
   Use `--dry-run` to inspect without writing, `--save-debug <dir>` to dump
   per-hold patches as `.npz` for plotting.
4. **Read the output.** Per-hold lines say why holds were skipped. After the
   solve: `normal spread` (want ≳ 0.3), point-to-plane residuals (want median
   ≲ 10 mm), and the result vs CAD. Results merge into
   `../../ros2_ws/src/mmb_bringup/config/extrinsics_calibrated.yaml`
   **without clobbering** the lidar_lidar entries.

## Chaining and ordering

- Front cameras (`oak4d`, `insta360`) calibrate against the Hesai: chain is
  CAD `base_link→lidar_hesai` ∘ solved camera↔Hesai. Can run any time.
- Rear cameras (`oak1`, `oakd_lite`, `realsense`) calibrate against the Livox:
  the chain uses the **calibrated** `base_link→lidar_livox_avia` from the
  lidar_lidar stage (read from `extrinsics_calibrated.yaml` automatically).
  The current Hesai↔Livox result is preliminary (validate said CHECK, stable
  to ~1 cm/0.5°) — rear-camera chains inherit that uncertainty. Re-running
  this stage after a better LiDAR↔LiDAR result only requires re-running the
  chaining (the pairwise camera↔Livox solve is unaffected).

## Caveats / open items

- The extrinsics seed is now ROS REP-103 (`extrinsics_initial.yaml`, converted
  from the SolidWorks CAD source `coordinate_systems.yaml` via
  `../scripts/convert_cad_to_ros_frame.py`); camera frames are optical, matching
  cv2, so there is no fudge matrix. The seed is used only to *find* the board in
  the cloud (crop + normal gate). If a pair skips most holds with "normal X deg
  off prediction", check that camera's seed orientation/mounting, raise
  `lidar.max_normal_vs_cad_deg` and rely on the extent check, then inspect with
  `--save-debug`.
- Insta360: confirm driver topic + image format (dual-fisheye vs
  equirectangular). `camera_model: fisheye` handles a single fisheye image
  with k1..k4; equirectangular would need a different projection (not built).
- Not built yet (CONTEXT.md §4/§6 steps 3–5): `camera_camera.py` (Insta360
  loop closures) and `global_optimize.py` (pose-graph over all edges).
- `Oak_4_light` vs OAK-D Lite naming question from `../RESUME_CONTEXT.md` is
  still open — confirm before trusting the `oakd_lite` CAD seed.

## Validation

- Per-hold point-to-plane residuals are printed by the refine step (mm).
- Reprojection overlay: project the saved debug patches into the image with
  the solved transform; board edges should line up (script TODO, or do it in
  a notebook from the `.npz` dumps).
