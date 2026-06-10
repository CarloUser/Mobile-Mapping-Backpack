# Targetless cross-check: direct_visual_lidar_calibration (Koide)

Independent validation for every camera↔LiDAR edge produced by
`calibrate_cam_lidar.py`, and the recommended primary method for
**Insta360↔Hesai** (it supports omnidirectional camera models; our ChArUco
fisheye path is the least-tested branch). See `../METHOD_EVALUATION.md` §2.

Tool: <https://github.com/koide3/direct_visual_lidar_calibration> — targetless
(NID-based fine registration, SuperGlue initialization, **no initial guess
needed**), ROS1/ROS2, MIT. Docs: <https://koide3.github.io/direct_visual_lidar_calibration/>.

## Where to run

The tool is an **offline bag processor** — it does not need to run on the rig.
The Jetson has no docker (and the SuperGlue step wants a beefier GPU anyway),
so run it on the Windows PC (Docker Desktop / WSL2) against bags copied off
the Jetson:

```bash
docker pull koide3/direct_visual_lidar_calibration:humble
```

## Recording requirements (per camera↔LiDAR pair)

Different from both the board protocol and the mapping walk — record a
dedicated short bag:

- One **static, structure-rich scene** (textured walls, furniture; NOT a bare
  corridor). Indoors is fine.
- Rig **stationary on a tripod/stand**, ~10-30 s. For the Livox pairs the
  tool integrates the non-repetitive scans over time (`--dynamic_points` /
  `-d` for moving captures; static is simpler and better).
- Topics: the pair's image topic + camera_info + the LiDAR topic
  (`/lidar_points` for Hesai pairs, `/livox/lidar` for Livox pairs — it reads
  Livox CustomMsg natively).
- 2-3 viewpoints (move the rig between captures, one bag each) make the
  estimate noticeably stronger than a single pose.

## Commands (per pair, inside the container)

```bash
# 1. preprocess: builds a dense integrated cloud + intensity image per bag
ros2 run direct_visual_lidar_calibration preprocess <bag_dir>... <out_dir> -a -d -v

# 2. initial guess, automatic (SuperGlue feature matching):
ros2 run direct_visual_lidar_calibration find_matches_superglue.py <out_dir>
ros2 run direct_visual_lidar_calibration initial_guess_auto <out_dir>
#    (fallback if SuperGlue struggles: initial_guess_manual <out_dir>)

# 3. fine registration (NID), writes calib.json
ros2 run direct_visual_lidar_calibration calibrate <out_dir>

# 4. visual inspection
ros2 run direct_visual_lidar_calibration viewer <out_dir>
```

`calib.json` contains `T_lidar_camera` (xyz + quaternion).

## Comparing against our result

Our YAML stores `base_link -> camera_X`. Convert both to the same edge:

```python
import numpy as np, sys
sys.path.insert(0, "extrinsic_calibration/camera_lidar")
import io_cam
_, f = io_cam.load_extrinsics("ros2_ws/src/mmb_bringup/config/extrinsics_calibrated.yaml")
T_lidar_cam_ours = np.linalg.inv(f["lidar_hesai"]) @ f["camera_oak4d"]   # per pair
# compare rotation (deg) and translation (cm) against calib.json's T_lidar_camera
```

Agreement expectations: < ~0.3° / < ~1 cm = both methods healthy. Larger
rotation disagreement on a *rear* camera pair points at the Hesai↔Livox chain;
larger disagreement on Insta360 points at our fisheye intrinsics.

## Status

- [ ] docker image pulled on the PC
- [ ] cross-check bags recorded (one per pair, static scene)
- [ ] OAK-4D↔Hesai compared
- [ ] OAK-1 / OAK-D Lite / RealSense ↔ Livox compared
- [ ] Insta360↔Hesai (primary method here, not just cross-check)
