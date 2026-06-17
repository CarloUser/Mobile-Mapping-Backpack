# Extrinsic calibration methods — evaluation against standard practice

Written 2026-06-10, after the LiDAR↔LiDAR stage produced its first real result
and the camera↔LiDAR pipeline was implemented (awaiting a recording). For each
edge type: what we built, what the field considers standard, an honest
assessment, and whether an existing open-source tool ("template") would serve
us better — or as a cross-check.

TL;DR recommendations, in priority order:

1. **Add a map-based refinement step to LiDAR↔LiDAR** using `small_gicp`
   (pip-installable, aarch64 wheels exist). Hand-eye got us a stable ~1 cm/0.5°
   initial; map-based registration is the standard way to push that to
   mm/sub-0.1° and should turn the CHECK verdict into GOOD without re-recording.
2. **Keep the ChArUco plane pipeline as the camera↔LiDAR backbone**, and use
   Koide's `direct_visual_lidar_calibration` (ROS2-Humble docker, targetless,
   no initial guess needed) as an independent cross-check — and as the primary
   method for the Insta360, where our pinhole/fisheye ChArUco path is weakest.
3. **Keep the planned solvePnP loop-closure approach for camera↔camera.**
   Kalibr is the gold standard but is ROS1-centric and solves a harder problem
   (joint intrinsics+extrinsics from synchronized video) than we need given the
   LiDAR-anchored backbone.
4. The CAD seeds in `coordinate_systems.yaml` are SolidWorks **part frames**
   (X-fwd/Y-up/Z-right); the lidar_lidar stage proved they can differ from sensor
   data frames by axis permutations (120° apparent "error"). Treat CAD as a search
   seed, never as a validation reference.
   - UPDATE (2026-06-17): `extrinsics_initial.yaml` is now re-expressed in ROS
     REP-103 (X-fwd/Y-left/Z-up; cameras optical) via
     `scripts/convert_cad_to_ros_frame.py`. `coordinate_systems.yaml` stays the
     untouched CAD source of truth. Base convention is reconciled; remaining
     per-frame differences vs sensor data are genuine calibration corrections.

---

## 1. LiDAR ↔ LiDAR (Hesai JT128 ↔ Livox Avia, no FOV overlap)

### What we built

Motion-based hand-eye (`AX = XB`): per-sensor odometry (KISS-ICP on the Hesai,
FAST-LIO on the Livox), SLERP time-alignment, Kabsch rotation + LS translation,
robust trimming, rotation-observability gate, two-fold cross-validation.
Result on the 2026-06-09 walk: solution stable to ~1 cm / 0.5° across data
windows and pairing strides; per-pair residuals ~2° / 8 cm (verdict CHECK,
driven by per-pose odometry jitter, not solver error).

### Standard methods

| Method | Requires | Typical accuracy | Fit for us |
| --- | --- | --- | --- |
| Target-based (shared board/sphere) | FOV overlap | mm / <0.1° | **Impossible** — opposite-facing sensors |
| Motion-based hand-eye (ours) | rich 3-axis rotation | cm / ~0.5° | Works; accuracy odometry-limited |
| Map-based registration (register sensor B's points into a map built from sensor A's trajectory, optimizing the extrinsic) | environment revisited by both FOVs during the walk | mm–cm / <0.1° | **Best next step** — both lidars sweep the same building during a walk |
| Joint multi-LiDAR optimization (e.g. HKU-MARS `mlcc`, adaptive-voxel joint optimization) | good initial guess | mm / <0.1° | Same idea, heavier tooling (ROS1, catkin) |

Hand-eye is the *correct standard choice* for non-overlapping sensors as an
initializer, and our implementation follows accepted practice (robust Kabsch,
observability gating — the same structure found in published toolboxes). Its
known ceiling is exactly what we measured: translation error ≈ rotation noise ×
lever arm, with odometry jitter as the noise floor.

### Recommendation — map-based refinement (concrete, low-effort)

We already have everything needed:
- a Hesai trajectory in the Hesai frame (KISS-ICP, `out/hesai_tum.txt`),
- raw Livox scans with per-point time (`/livox/lidar` CustomMsg, readable in
  pure Python via the registered typestore in `camera_lidar/io_cam.py`),
- a good initial `T_hesai_livox` (the hand-eye result),
- `small_gicp` provides pip wheels for aarch64 / Python 3.10 — runs on this
  Jetson without any ROS build.

Procedure (one new script, `lidar_lidar/refine_map_gicp.py`):
1. Build a point-to-plane target map: Hesai scans deskewed/posed by the
   KISS-ICP trajectory, voxel-downsampled.
2. For each Livox scan k at time t_k: predict its world pose as
   `T_world_hesai(t_k) @ X` and register the scan to the map with GICP,
   optimizing **only X** (accumulate per-scan point-to-plane factors, solve
   jointly; or iterate scan-matching and re-average).
3. Validate: residual distribution + the existing `validate.py` criteria.

This is the standard "calibration by mapping consistency" refinement; expected
to reach the GOOD thresholds (<0.5°, <2 cm) from our current initial. It also
gives a direct visual QA artifact: the fused two-LiDAR map.

`mlcc` (HKU-MARS) implements the same concept with adaptive voxelization but is
a ROS1/catkin codebase — porting costs more than writing the ~200-line
small_gicp version against our existing I/O.

---

## 2. Camera ↔ LiDAR

### What we built

`camera_lidar/`: ChArUco board detection (`CharucoDetector`, pinhole + fisheye),
static-hold clustering, CAD-seeded automatic board segmentation in the cloud,
plane-correspondence closed form (Kabsch on normals + LS translation — Zhang &
Pless 2004), robust point-to-plane refinement, observability gate, results
merged into the shared extrinsics YAML. Unit-tested on synthetic data;
not yet run on a real recording.

### Standard methods

| Method | Examples | Notes |
| --- | --- | --- |
| Target-based, plane correspondences | MATLAB lidarCameraCalibrator, OpenCalib (PJLab SensorsCalibration), Autoware/TIER IV CalibrationTools, ours | The established baseline; needs a board session per pair |
| Target-based, 3D-edge / hole targets | velo2cam_calibration, OpenCalib variants | Higher accuracy potential; needs special targets |
| Targetless, appearance-based | **Koide `direct_visual_lidar_calibration`** (NID + SuperGlue init), mutual-information methods (Pandey et al.) | One casual scene recording; no target; needs LiDAR intensity + scene texture |
| Deep-learned | LCCNet etc. | Not appropriate for one-off rig calibration |

Our pipeline matches the standard target-based formulation, with two
deliberate strengths: the static-hold protocol matches plane-fit assumptions,
and the CAD-seeded crop makes LiDAR board segmentation automatic (most
reference implementations require manual ROI clicking per pose). The
plane-normal observability gate mirrors the standard degeneracy warning.

Honest weaknesses to track when the first real bag arrives:
- Livox point clustering near plane edges (beam divergence) biases plane fits —
  mitigated by the RANSAC threshold and extent gate, but watch residuals.
- The Insta360 path (fisheye, possibly equirectangular driver output) is the
  least-tested branch; ChArUco corner accuracy degrades off-axis in fisheye
  images.
- Plane methods constrain translation along the normal only — needs the
  varied-tilt protocol to be followed strictly (the gate enforces this).

### Recommendation

Keep ours as the backbone (it is the standard method, automated for our rig).
Adopt **`direct_visual_lidar_calibration`** as a *cross-check template*:
- ROS2 Humble docker images exist; it consumes a rosbag of one scene with
  camera + LiDAR, integrates Livox scans natively (`-d` dynamic integration),
  needs **no initial guess** (SuperGlue feature matching), and outputs a
  `calib.json` directly comparable to our result.
- Use it (a) to independently validate each backbone edge, (b) as the primary
  method for Insta360↔Hesai if the ChArUco fisheye path underdelivers — it
  supports omnidirectional camera models.
- Cost: pulling a docker image on the Jetson (or running it on the Windows PC —
  it is an offline tool reading bags).

OpenCalib is a useful grab-bag (it has manual-adjustment GUIs that are handy
for eyeballing), but its tools are x86-built C++ with mixed build hygiene; not
worth adopting wholesale on aarch64.

---

## 3. Camera ↔ Camera (Insta360 hub loop closures)

### What's planned (not yet implemented)

When two cameras see the board simultaneously: `T_a_b = T_a_board @
inv(T_b_board)`, robustly averaged over frames; the omnidirectional Insta360
ties front and rear groups; a final pose-graph optimization (fixing
`lidar_hesai`) distributes error over the over-constrained graph.

### Standard methods

| Method | Notes |
| --- | --- |
| OpenCV `stereoCalibrate` | Pairs with strong overlap + shared detections; joint refinement of the pair |
| **Kalibr multi-camera** | Gold standard: joint bundle adjustment of intrinsics + extrinsics over synchronized video of an AprilGrid; supports fisheye/omni models and camera chains with pairwise-only overlap |
| Board-pose composition (ours) | Simple, model-agnostic, works from static holds; no joint refinement |

Assessment: Kalibr would give tighter camera↔camera edges, but (a) it is
ROS1-tooled (docker + bag conversion needed), (b) it solves a bigger problem
than we have — our cameras are each *independently anchored to a LiDAR*, so the
camera↔camera edges are redundant loop closures, not the primary chain, and
(c) Kalibr wants synchronized continuous video, a different recording protocol
from our static holds. The board-pose composition approach is adequate for
loop-closure duty; the global pose graph is where the accuracy is recovered.

Recommendation: implement as planned (`camera_camera.py` + `global_optimize.py`,
scipy least_squares on SE(3) — the graph has ~7 nodes and ~12 edges; GTSAM is
overkill). Revisit Kalibr only if loop-closure residuals after global
optimization stay above ~0.5°/1 cm. Camera **intrinsics** are a different
story: if the existing `Camera_Calibration/` ChArUco intrinsics show >0.5 px
RMS reprojection, redoing intrinsics (possibly with Kalibr's models for the
Insta360) is the cheapest accuracy win in the whole graph — intrinsic error is
indistinguishable from extrinsic error downstream.

---

## 4. Edges nobody has scheduled yet (gaps in the plan)

- **Xsens IMU ↔ base_link**: currently CAD-only. Standard: camera↔IMU via
  Kalibr (continuous excitation recording) or LiDAR↔IMU via FAST-LIO-style
  joint estimation (we already run FAST-LIO — its `extrinsic_est_en: true`
  mode can refine Livox↔its-own-IMU; Hesai↔Xsens would need a dedicated
  hand-eye against the Xsens orientation stream, which our existing
  `handeye.py` can do directly from a rotation-rich walk bag).
  For mapping purposes the CAD lever arm (mm-accurate from SolidWorks) plus a
  hand-eye rotation alignment is likely sufficient.
- **GNSS antenna lever arm**: not observable by any FOV method; CAD is the
  standard answer. Document it as CAD-final.
- **CAD frame-convention reconciliation**: one sitting with the SolidWorks
  export (`Coordinate_systems/ExportCoordinateSystems_SW.bas`) to redefine
  each sensor's exported frame to match its *driver* frame would make
  "correction vs CAD" a meaningful validation metric again. Right now it is
  polluted by axis-permutation artifacts.

---

## Sources

- Livox time sync (PTP/GPS/PPS, priority PTP>GPS>PPS): [Livox device time synchronization manual](https://github.com/Livox-SDK/Livox-SDK/wiki/livox-device-time-synchronization-manual), [Livox wiki — Time Synchronization Instructions](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)
- Targetless camera↔LiDAR: [koide3/direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration), [docs + docker images](https://koide3.github.io/direct_visual_lidar_calibration/)
- Registration library: [koide3/small_gicp](https://github.com/koide3/small_gicp), [PyPI small-gicp (aarch64 wheels)](https://pypi.org/project/small-gicp/)
- Plane-based camera↔LiDAR: Zhang & Pless, IROS 2004. Multi-LiDAR joint calib: HKU-MARS `mlcc`.
