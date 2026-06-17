# Mobile Mapping Backpack — Calibration Context

A single, detailed reference for the **sensor suite**, the **frame conventions**,
and every **intrinsic and extrinsic calibration method** used in this project.
Written to be self-contained: someone resuming the work should be able to read
only this file and understand what each tool does, why it does it that way, what
has been solved, and what is still open.

---

## 1. The platform

A wearable **mobile mapping backpack**: two LiDARs facing opposite directions,
five cameras split front/rear, an IMU, and a GNSS antenna, all rigidly mounted on
one structure. Data is recorded on a Jetson (single-Jetson `record_all.sh`, with a
dual-Jetson split planned for full-rate recording). The end goal is to express
**every sensor in one common body frame, `base_link`**, so that LiDAR maps,
camera images, IMU motion and GNSS positions all fuse consistently.

### Sensor inventory

| Sensor | Model | Role / direction | Interface | Key topics |
| --- | --- | --- | --- | --- |
| LiDAR (reference) | **Hesai JT128** | front | Ethernet | `/lidar_points` (PointCloud2 ~10 Hz), `/lidar_imu` (~420 Hz) |
| LiDAR | **Livox Avia** | rear | Ethernet | `/livox/lidar` (CustomMsg, per-point time), `/livox/imu` (~200 Hz) |
| Camera | **OAK-4D (OAK-4-PRO, RVC4)** | front | **PoE** | `/oak4d/rgb/image_raw` + `/camera_info`, `/oak4d/imu/data` |
| Camera | **OAK-1** | rear | USB | `/oak1/rgb/image_raw` + `/camera_info` |
| Camera | **OAK-D Lite** | rear | USB | `/oakd_lite/rgb/image_raw` + `/camera_info` |
| Camera | **Intel RealSense** (D4xx) | rear | USB | `/realsense/color/image_raw` + `/camera_info` |
| Camera | **Insta360** (360°/dual-fisheye) | omni | USB hub | `/dual_fisheye/image/compressed` (H.264), `/imu/data_raw` |
| IMU | **Xsens MTi-610R** | n/a | USB/serial | `/imu/data` (400 Hz), `/imu/mag` (100 Hz) |
| GNSS | **u-blox** | n/a | USB/serial | `/gnss/fix`, `/gnss/fix_velocity` |

The **field-of-view map** is the single constraint that drives the whole extrinsic
strategy:

| Sensor | Direction | Shares FOV with |
| --- | --- | --- |
| Hesai JT128 (reference) | front | OAK-4D, Insta360 |
| Livox Avia | rear | OAK-1, OAK-D Lite, RealSense, Insta360 |
| OAK-4D | front | Hesai, Insta360 |
| OAK-1 / OAK-D Lite / RealSense | rear | Livox, Insta360 |
| Insta360 | omni | **every** sensor (calibration hub) |
| Xsens IMU | n/a | motion-based (no FOV) |
| u-blox GNSS | n/a | lever arm (no FOV) |

Because the two LiDARs face opposite ways, **no single target is visible to both**
— that is why Hesai↔Livox must be solved by motion, not by a board.

---

## 2. Frame conventions (read before touching any transform)

This is the most error-prone part of the whole project. **The entire repo now
uses ROS REP-103 conventions** — this was converted from the SolidWorks CAD
convention on 2026-06-17 (see history note at the end of this section).

- **`base_link` and all body frames** (LiDARs, IMU, GNSS) follow **ROS REP-103:
  X-forward, Y-left, Z-up**. World frames are ENU.
- **Camera frames** (`camera_oak4d`, `camera_oak1`, …) are ROS **optical frames:
  X-right, Y-down, Z-forward** — the same convention `cv2` solvePnP/ChArUco and
  `camera_info`/images use. So a camera's `extrinsics_initial.yaml` entry is the
  pose of its optical frame in `base_link`. A forward-facing camera's optical
  frame therefore has `q_xyzw ≈ [0.5, -0.5, 0.5, -0.5]` (e.g. the OAK-4D), which
  is the standard ROS forward-camera optical quaternion — **not** identity.
- **Consequence for the calibrator:** because both the measurements (cv2) and the
  camera frame are optical, `calibrate_cam_lidar.py` needs **no** optical→body
  fudge matrix. It solves `T_optical_lidar` directly and chains
  `T_base_cam = T_base_lidar ∘ inv(T_optical_lidar)`. A correct OAK-4D solve comes
  out **near its seed** (small "vs CAD" delta) — that's the sanity check.
- **Quaternions** are stored `xyzw`, translations in meters.
- **Sanity anchors:** in REP-103 base the **Hesai is identity** (`[0,0,0,1]`) — a
  Z-up LiDAR in a Z-up base; if it isn't identity after an edit, the base
  convention is wrong.

**History / provenance.** The SolidWorks export uses **X-forward, Y-up, Z-right**
(not REP-103). `extrinsics_initial.yaml` was re-expressed into REP-103 by a single
global reorientation `R_CAD2ROS = Rx(+90°) = [[1,0,0],[0,0,-1],[0,1,0]]` (and
cameras additionally rotated into the optical convention) using
`extrinsic_calibration/scripts/convert_cad_to_ros_frame.py`. base_link's
orientation is arbitrary, so it was simply redefined as REP-103. An earlier
calibrator carried a hardcoded `R_OPT2BODY`/`R_OPT2CAD` fudge matrix — now
**obsolete and removed**, since the whole tree is REP-103 with optical cameras.

Reference files:
- `ros2_ws/src/mmb_bringup/config/extrinsics_initial.yaml` — seed, **REP-103** (regenerable from the CAD source via the converter).
- `ros2_ws/src/mmb_bringup/config/extrinsics_calibrated.yaml` — merged solved results, REP-103.
- `Coordinate_systems/coordinate_systems.yaml` — original SolidWorks export, **CAD convention, source of truth, never overwrite**.
- `extrinsic_calibration/scripts/convert_cad_to_ros_frame.py` — the one-shot CAD→REP-103 converter (documents the exact rotation).
- `ros2_ws/src/mmb_bringup/launch/static_extrinsics.launch.py` — publishes the static TF tree from a chosen extrinsics file.

---

## 3. Intrinsic calibration (cameras)

**Location:** `Camera_Calibration/`
**Core engine:** `intrinsic_calibration_core.py`
**Per-camera entry points:** `calibrate_oak4d_front.py`, `calibrate_oak1_back.py`,
`calibrate_oaklite_back.py`, `calibrate_intel_back.py`,
`calibrate_insta360_front.py`, `calibrate_insta360_back.py` (each just calls
`core.run_camera("<name>", args)`).
**Outputs:** `Camera_Calibration/calibration_results/<name>_intrinsics.yaml`
(OpenCV `FileStorage`), aggregated in `all_calibration_results.yaml`.

### Method

A **ChArUco board** (checkerboard fused with ArUco markers) is the calibration
target. ChArUco is preferred over a plain checkerboard because each interior
corner is uniquely identified by surrounding markers, so detection is robust to
partial views and the board pose is never ambiguous.

Per camera the workflow is:

1. **Live capture loop** (`run_stage` / `run_internal_camera`): open the device,
   show a live preview, detect ChArUco corners each frame, and **auto-save a frame
   every `--capture-interval` seconds only when ≥ `--min-features` (default 8)
   ChArUco corners are visible**. Targets `--target-valid` (default 30) good
   frames per camera. Press `q` to stop early; a per-camera `--stage-timeout`
   (default 600 s) also stops it.
2. **Solve** (`calibrate_camera`): once ≥ `--min-valid` (default 20) frames exist,
   - **Pinhole cameras** (OAK-4D, OAK-1, OAK-D Lite, RealSense): use
     `cv2.aruco.calibrateCameraCharuco` → camera matrix `K`, `plumb_bob`
     distortion (`dist_coeffs`), RMS reprojection error.
   - **Fisheye cameras** (Insta360, `fisheye=True`): build object/image point
     pairs from the matched ChArUco corners and call `cv2.fisheye.calibrate` with
     `CALIB_RECOMPUTE_EXTRINSIC + CALIB_FIX_SKEW` → `K` + 4 fisheye coefficients
     (k1..k4).
3. **Write** an OpenCV YAML per camera with `camera_matrix`, `dist_coeffs`,
   `reprojection_error`, `image_width/height`, `fisheye_model` flag, and number of
   valid frames.

### Device handling (the messy real-world part)

The core file abstracts four capture backends behind a common `read()`:
- `OpenCVCamera` — `cv2.VideoCapture` with backend/index auto-probing and optional
  crop (`top/bottom/left/right/full`); used for the Insta360 over the OS video
  interface (the front and back fisheye halves are cropped from one stream).
- `RealSenseCamera` — `pyrealsense2` pipeline with a fallback ladder of
  resolution/fps profiles.
- `DepthAICamera` — DepthAI **v2** USB OAKs (OAK-1, OAK-D Lite), pinned by MXID so
  multiple OAKs don't race for the same device.
- `OAK4DProCamera` — DepthAI **v3** for the RVC4 PoE OAK-4D, addressed by IP
  (default `192.168.25.97`) or auto-discovered over TCP/IP.

A **two-stage flow** (`main`): the **back stage** calibrates the rear group
(RealSense + OAK-1 + OAK-D Lite + Insta360-back) in parallel child processes, then
prompts you to move the board to the **front** and runs the front stage (OAK-4D +
Insta360-front). `--list-devices` enumerates everything detected.

### Board parameters (intrinsic defaults)

`--dictionary DICT_5X5_250`, `--board-cols 11`, `--board-rows 8`,
`--square-size 0.030`, `--marker-ratio 0.75`. **Note:** these CLI defaults differ
slightly from the *physical* board used for the extrinsic stage (square length
**0.034 m**, marker **0.025 m**); always pass the measured physical dimensions.

### Existing results (already captured)

- `oak4d_front`: `K ≈ [[2981.6,0,2059.5],[0,2985.1,1065.2],[0,0,1]]` at 3840×2160,
  `plumb_bob` 5-coeff distortion, **reproj err 2.43 px**, 30 frames.
- `insta360_back` (fisheye): reproj **0.397 px**, 30 frames.
- `insta360_front` (fisheye): reproj **0.468 px**, 30 frames.

> Intrinsics are calibrated **first** because intrinsic and extrinsic error are
> not separable — the extrinsic stage assumes `camera_info` (K, D) is already
> correct.

---

## 4. Extrinsic calibration — overall strategy

**Location:** `extrinsic_calibration/` (see `CALIBRATION_STRATEGY.md`,
`METHOD_EVALUATION.md`, `RESUME_CONTEXT.md`).

Goal: one consistent `base_link → sensor` transform per sensor, with the **Hesai
JT128 as the fixed reference**. CAD values seed every step; refined values are
merged into `extrinsics_calibrated.yaml` (existing entries preserved).

### The calibration graph

```
                 base_link
                    | (CAD, fixed)
                 lidar_hesai  ========= lidar_livox_avia
                  /     \    (DONE,         /   |   \
            OAK-4D    Insta360  hand-eye)  OAK-1 OAKlite RealSense
               \       / | \                 \   |   /
                \     /  |  \__________________\__|__/
                 (Insta360 ties to every camera -> redundant loops)
```

**Backbone (minimum spanning tree — one metric path per sensor to the reference):**
- `lidar_hesai → lidar_livox_avia` — motion-based hand-eye **[DONE]**
- `lidar_hesai → OAK-4D` — target-based camera↔LiDAR (front)
- `lidar_livox_avia → {OAK-1, OAK-D Lite, RealSense}` — target-based, rear; each
  chains to the reference through the solved Hesai↔Livox transform
- `lidar_hesai → Insta360` — target-based (or Koide targetless)

**Loop closures (over-constrain, used by a future global optimisation):**
- `Insta360 → {OAK-4D, OAK-1, OAK-D Lite, RealSense}` camera↔camera, when both
  cameras see the same board in the same frame.

### Ordering

1. Camera intrinsics (Section 3).
2. `lidar_hesai → lidar_livox_avia` (Section 5) — **done**.
3. Camera↔LiDAR backbone (Section 6): OAK-4D→Hesai, then rear cams→Livox, then
   Insta360→Hesai.
4. Camera↔camera loop closures through the Insta360.
5. Global pose-graph optimisation (fix Hesai) → one consistent set + loop residual
   as the quality metric.
6. IMU (motion-based hand-eye), then GNSS lever arm.
7. Validation: reproject LiDAR into each image (edge alignment), overlay maps,
   check loop residuals.

Rationale for backbone-first (not camera↔camera or one big bundle adjustment from
the start): most camera pairs don't overlap, and camera↔camera alone can't reach
the LiDAR reference. The LiDAR-anchored backbone guarantees every sensor a metric
path to `base_link`; loop closures + global step then *refine* it, keeping each
step independently checkable.

---

## 5. LiDAR↔LiDAR extrinsic (Hesai ↔ Livox) — motion-based hand-eye  [DONE]

**Location:** `extrinsic_calibration/lidar_lidar/`
**Status:** solved, validation verdict **CHECK** (see memory
`mmb-lidar-calibration-status`).

### Why hand-eye

The two LiDARs share no FOV, so no target works. But they are rigidly attached and
move together, so their relative motions differ only by the constant transform
`X = T_hesai_livox` we want: **`Aᵢ X = X Bᵢ`**, where `Aᵢ`, `Bᵢ` are the two
sensors' body-frame relative motions between two times. This holds regardless of
each odometry's arbitrary map origin.

### Pipeline

1. **Record** one bag walking the rig through a feature-rich static scene (2–5
   min), exciting **all three rotation axes** (figure-eights, ramps/stairs, tilt,
   nod). `record_lidar_calib.sh` records only `/lidar_points` + `/livox/lidar`.
2. **Per-LiDAR odometry** (`run_odometry.py`): drives **KISS-ICP** independently on
   each LiDAR → two **TUM** trajectories (`out/hesai_tum.txt`, `out/livox_tum.txt`).
   FAST-LIO is also available (`fast_lio_avia.yaml`, `FASTLIO_GUIDE.md`) for the
   Livox; see memory note on building it.
3. **Solve** (`solve_extrinsic.py` → `handeye.py`):
   - Time-align: interpolate Livox poses onto Hesai stamps (SLERP rotation + linear
     translation, `interp_pose`).
   - Form relative-motion pairs over a `stride`, keep pairs with ≥ `min_angle_deg`
     rotation (`motion_pairs`).
   - **Rotation:** `Aᵢ X = X Bᵢ` ⟹ axes satisfy `axis(Aᵢ) = R_X axis(Bᵢ)`; solved
     by Kabsch/Wahba over `Σ αᵢ βᵢᵀ` (`solve_rotation`).
   - **Translation:** least squares `(R_Aᵢ − I) t_X = R_X t_Bᵢ − t_Aᵢ`
     (`solve_translation`).
   - **Robustness:** `solve_handeye_robust` iteratively trims the worst-residual
     15% and re-solves.
   - **Chain:** keep Hesai fixed; `T_base_livox = T_base_hesai @ X`; report
     correction vs CAD.
4. **Validate** (`validate.py`): residuals (target median rot < ~0.5°, trans < ~2
   cm), two-fold cross-validation, plot `out/validation.png`; prints `GOOD` or
   `CHECK MOTION / SYNC`.

### The one failure mode that matters: observability

Hand-eye is **unobservable under single-axis motion** (flat-ground yaw-only walking
gives confident but wrong roll/pitch — self-test reproduces ~40–70° error).
`rotation_observability` returns the normalised spread of rotation axes (smallest/
largest singular value, [0,1]); aim **well above 0.1**. `solve_extrinsic.py` warns
loudly below `min_observability`.

### Time-sync subtlety (gotcha)

The solver puts both LiDARs on one timeline. `config.yaml: bag.timestamp_source`:
- `header` — only if both drivers share a synced clock.
- **`header_aligned` (recommended for this rig)** — keep each sensor's low-jitter
  header spacing but shift both onto the shared recorder epoch via a robust median
  offset (the two drivers use different clock epochs).
- `bag_receive` — fallback if `header.stamp` is missing/zero.

`inspect_bag_timing.py` characterises both timestamp sources before you choose.
(See memory: clock/CAD-frame gotchas.)

### References
Park & Martin 1994 (AX=XB on SE(3)); Daniilidis 1999 (dual-quaternion hand-eye);
Vizzo et al. 2023 (KISS-ICP).

---

## 6. Camera↔LiDAR extrinsic — target-based plane correspondences

**Location:** `extrinsic_calibration/camera_lidar/`
**Driver:** `calibrate_cam_lidar.py` (one bag = one camera/LiDAR pair)
**Modules:** `board.py` (detection), `plane_solve.py` (geometry),
`io_cam.py` (bag + extrinsics I/O), `config.yaml` (params + pair table),
`test_camera_lidar.py` (unit tests).
**Status:** OAK-4D↔Hesai recording works at 1080p@10fps; frame-convention fix
applied (commit 86129d4); a clean open-space re-record is the remaining blocker
(see Section 8).

### The idea (Zhang & Pless 2004, cross-modal)

Hold a flat ChArUco board still in many orientations inside the **shared FOV** of a
camera and a LiDAR.
- The **camera** detects the board and `solvePnP` gives the board **plane**
  `(n_c, d_c)` in the camera frame.
- The **LiDAR** sees the board as a planar patch; RANSAC fits its **plane**
  `(n_l, c_l)`.
- Across many board tilts, **aligning the plane normals fixes the rotation** and
  the **plane offsets fix the translation**; a point-to-plane least-squares then
  refines. The printed pattern is irrelevant to the LiDAR — only flatness,
  rigidity, isolation from background, and a spread of tilts/positions/distances
  matter.

### Step-by-step (`calibrate_cam_lidar.py`)

1. **Detect** the board in every Nth image (`hcfg.image_stride`) over the whole
   bag (`board.py: BoardDetector.detect`):
   - `cv2.aruco.CharucoDetector` with subpixel corner refinement.
   - `board.matchImagePoints` → object/image correspondences.
   - **Pinhole:** `solvePnP` with `SOLVEPNP_IPPE` (planar target) then an
     `ITERATIVE` refine. **Fisheye:** `cv2.fisheye.undistortPoints` to normalised
     coords, then PnP with identity intrinsics.
   - Reject detections with < `min_corners` or reproj error > `max_reproj_err_px`.
   - Returns `T_cam_board`, plane `(n, d)` with `n` oriented toward the camera
     (`n·X + d = 0`), the used corners, and reproj error.
2. **Cluster into static holds** (`cluster_holds`): the protocol holds the board
   still 3–5 s per pose. A hold is ≥ `min_hold_s` of consecutive detections whose
   pose stays within `max_rot_deg` / `max_trans_m` of the hold's running median,
   tolerating dropouts up to `max_gap_s`. One **representative plane** per hold
   (`hold_representative`: median normal, median offset, board centre/origin).
   These stay in the camera **optical** frame — which is exactly what the
   REP-103 `camera_*` frames are — so they chain through the extrinsics with **no
   conversion** (`d` is rotation-invariant either way).
3. **LiDAR board patch per hold** (the part that makes segmentation *automatic*):
   - Accumulate LiDAR points within `±half_window_s` of the hold centre
     (`collect_cloud`) — the Livox's non-repetitive pattern needs ~1 s to densify;
     the Hesai needs one scan.
   - **Crop a sphere** around the **seed-predicted board position** (seed =
     `extrinsics_initial.yaml`, REP-103): `T_cam_lidar_seed =
     inv(T_base_cam_seed) @ T_base_lidar`, transform the board centre into the
     LiDAR frame, keep points within `board_diag/2 + crop_margin_m`.
   - **RANSAC a plane** (`plane_solve.ransac_plane`): random 3-point hypotheses,
     `ransac_dist_m` inlier band, two refit rounds; needs ≥ `min_inliers`.
   - **Wall guard #1 (size):** reject patches whose in-plane extent
     (`patch_extent`, ~2σ) exceeds `max(board_size) + max_extent_margin_m` — kills
     walls/floor.
   - **Wall guard #2 (orientation):** reject patches whose normal disagrees with
     the seed-predicted board normal by > `max_normal_vs_cad_deg`.
4. **Solve `T_cam_lidar`** (`plane_solve.solve_cam_lidar`):
   - **Observability gate:** `normal_spread` (smallest singular value of stacked
     unit normals) must exceed `min_normal_spread`, else rotation about the normal
     and in-plane translation are unobservable → "re-record with stronger tilts".
   - **Rotation:** Kabsch on normal pairs `n_c ≈ R n_l`.
   - **Translation:** least squares `n_c · t = −(d_c + n_c · R c_l)` per pose.
5. **Refine** (`refine_point_to_plane`): nonlinear least-squares (`soft_l1` robust
   loss) minimising point-to-plane distance of all LiDAR patch points against the
   camera board planes; reports median / 95th residual in mm and how far it moved
   the closed-form solution.
6. **Chain + merge:** `T_base_cam = T_base_lidar @ inv(T_cam_lidar)`. The reference
   LiDAR pose comes from the **calibrated** file when available (rear cameras
   inherit the solved Hesai↔Livox), else CAD. Writes `T_base_cam` into
   `extrinsics_calibrated.yaml` with provenance (method, reference, #holds,
   normal spread, residual). `--dry-run` skips the write; `--save-debug` dumps
   per-hold patches/planes (`dbg/hold_XX.npz`).

### Usage

```bash
cd extrinsic_calibration/camera_lidar
python3 calibrate_cam_lidar.py --bag <bag_dir> --pair oak4d            # solve + write
python3 calibrate_cam_lidar.py --bag <bag_dir> --pair oak1 --dry-run   # solve only
python3 calibrate_cam_lidar.py --bag <bag_dir> --pair oak4d --save-debug dbg
```

### Pair table (`config.yaml`)

| pair | camera_frame | lidar_frame | image / lidar topic | model |
| --- | --- | --- | --- | --- |
| `oak4d` | camera_oak4d | **lidar_hesai** | `/oak4d/rgb/image_raw` / `/lidar_points` | pinhole |
| `oak1` | camera_oak1 | lidar_livox_avia | `/oak1/rgb/image_raw` / `/livox/lidar` | pinhole |
| `oakd_lite` | camera_oakd_lite | lidar_livox_avia | `/oakd_lite/rgb/image_raw` / `/livox/lidar` | pinhole |
| `realsense` | camera_realsense | lidar_livox_avia | `/realsense/color/image_raw` / `/livox/lidar` | pinhole |
| `insta360` | camera_insta360 | lidar_hesai | `/dual_fisheye/image/compressed` / `/lidar_points` | fisheye |

`lidar_frame` decides the chaining: `lidar_hesai` → chain through CAD Hesai pose;
`lidar_livox_avia` → chain through the **calibrated** Livox pose (so rear cameras
inherit the lidar↔lidar result automatically).

**Insta360 is a documented TODO, not runnable here:** the driver publishes
**H.264** (`cv2.imdecode` can't read it) and there is **no `camera_info`** topic.
Decode offline first, or prefer the **Koide targetless** method
(`KOIDE_CROSSCHECK.md`).

### Board parameters (physical board, in `config.yaml`)

`squares_x: 11`, `squares_y: 8`, `square_len_m: 0.034`, `marker_len_m: 0.025`,
`dictionary: DICT_5X5_250`, printed on **A3**. The grid is confirmed recognized.
It is fine for the **physical sheet to be slightly larger than the squares grid**:
the method is plane-to-plane (matches normal + perpendicular distance, not in-plane
size), the white border is coplanar so it doesn't change the plane and gives more
LiDAR points. Only non-coplanar surfaces (walls/table/hands) in the crop hurt.

### Recording protocol (per pair)

Connect the camera + its paired LiDAR, run the pair's recording script
(`bash/record_pair.sh` or `bash/record_oak4d.sh` etc.), and hold the board **still
3–5 s per pose** across **≥ ~15–25 well-spread poses**: many tilts (roll/pitch, not
just yaw), positions and distances, **in open space** (≥ ~0.7 m clear behind the
board so the dominant nearby plane is the board, not a wall). Stop with Ctrl-C (or
`stop`); **kill the Hesai with SIGKILL** between recordings (it ignores SIGINT).

---

## 7. IMU and GNSS (planned)

- **Xsens IMU** — motion-based hand-eye: align IMU-integrated rotation to
  LiDAR-odometry rotation, then estimate the lever arm. Needs the same 3-axis
  excitation as the LiDAR↔LiDAR step (reuse a walking sequence).
- **u-blox GNSS** — single antenna ⇒ rotation is not meaningful; estimate only the
  **lever-arm translation** during an outdoor RTK run, comparing GNSS positions to
  LiDAR-inertial odometry.

---

## 8. Status & open items

| Stage | Status |
| --- | --- |
| Camera intrinsics | OAK-4D, Insta360 (front+back) **done**; OAK-1 / OAK-D Lite / RealSense to (re)capture |
| Hesai ↔ Livox (lidar↔lidar) | **Solved**, validation verdict **CHECK** (motion/sync — see memory) |
| OAK-4D ↔ Hesai (cam↔lidar) | 1080p@10fps works; REP-103 frames in place; **needs re-record with board ISOLATED** (2026-06-16 bag had board against body) then solve |
| Rear cams ↔ Livox | Drivers working; not yet recorded/solved |
| Insta360 ↔ Hesai | Blocked on H.264 decode + missing `camera_info`; use Koide targetless |
| Camera↔camera loop closures | Not started |
| Global pose-graph optimisation | Not started |
| IMU / GNSS | Not started |

**Known gotchas (carry forward):**
- The whole repo is now **REP-103** (converted 2026-06-17 from SolidWorks
  X-fwd/Y-up/Z-right via `Rx(+90°)`; cameras are optical frames). The calibrator
  no longer carries any optical→body fudge matrix. The original CAD source of
  truth is `Coordinate_systems/coordinate_systems.yaml`; regenerate the seed with
  `extrinsic_calibration/scripts/convert_cad_to_ros_frame.py` if needed. NOTE: the
  non-OAK-4D camera optical seeds assume the SolidWorks part frames share the
  OAK-4D's convention — these only seed the LiDAR crop, and each camera's own
  solve recovers the true optical pose, so verify per camera when calibrated.
- The cam↔LiDAR "0 holds / patch too large" failure on the 2026-06-16 OAK-4D↔Hesai
  bag was NOT a wall and NOT the frame mismatch: the **board was hand-held against
  the operator's body**, so the Hesai locked onto the ~0.9 m torso plane and the
  board itself returned ~0 isolated points (proven: 0 LiDAR pts within 5 cm of the
  camera-predicted board plane; LiDAR normal didn't tilt with the board). Fix =
  **re-record with the board isolated** (tripod/arm's length, ≥~0.5 m clear behind
  it), fill the frame (only 11–13 of ~70 ChArUco corners were detected), and use
  6–8+ varied tilts. Optional: segment by predicted normal+offset, not largest plane.
- Hesai ignores SIGINT — stop it with **SIGKILL** between recordings.
- OAK-4D is RVC4/PoE — must use **depthai v3** driver (3.3.0+; 3.2.1 has an ABI
  skew SIGSEGV); pass params via a **file**, not `-p` CLI.
- Git: a collaborator force-pushes `main` — always **fetch + pull before
  committing/pushing**; never force-push.

**Key paths:**
- Intrinsics: `Camera_Calibration/` → `calibration_results/*_intrinsics.yaml`
- LiDAR↔LiDAR: `extrinsic_calibration/lidar_lidar/`
- Camera↔LiDAR: `extrinsic_calibration/camera_lidar/`
- Extrinsics seed / output: `ros2_ws/src/mmb_bringup/config/extrinsics_{initial,calibrated}.yaml`
- Recorder topic list (single source of truth): `ros2_ws/src/mmb_bringup/config/topics.yaml`
- Recording: `bash/record_all.sh` (all sensors, one Jetson), per-pair `bash/record_*.sh`
</content>
</invoke>
