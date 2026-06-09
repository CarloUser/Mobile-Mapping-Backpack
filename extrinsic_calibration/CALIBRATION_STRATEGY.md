# Extrinsic Calibration Strategy

Goal: express every sensor in one common body frame, `base_link`, using the
**Hesai JT128 as the fixed reference**. CAD values (`extrinsics_initial.yaml`)
seed every step; refined values are written to `extrinsics_calibrated.yaml`.

## Sensor field-of-view map (the constraint that drives everything)

The two LiDARs point in opposite directions and the cameras are split front/rear.
What actually shares a field of view (and can therefore see a common target):

| Sensor | Direction | Shares FOV with |
| --- | --- | --- |
| Hesai JT128 (reference) | front | OAK-4D, Insta360 |
| Livox Avia | rear | OAK-1, OAK-D Lite, Intel RealSense, Insta360 |
| OAK-4D | front | Hesai, Insta360 |
| OAK-1 | rear | Livox, Insta360 |
| OAK-D Lite | rear | Livox, Insta360 |
| Intel RealSense | rear | Livox, Insta360 |
| Insta360 (360 camera) | omni | **every** sensor |
| Xsens IMU | n/a | motion-based (no FOV) |
| u-blox GNSS | n/a | lever arm (no FOV) |

Two consequences:

1. **The rear cameras can only reach the reference through Livox.** There is no
   Hesai overlap for OAK-1 / OAK-D Lite / RealSense, so each is calibrated to
   Livox, then chained through the (already solved) Hesai&harr;Livox transform.
   Their accuracy therefore depends on the Livox calibration too.
2. **The Insta360 sees everything**, so it is the natural *hub*: it can be tied
   to both LiDARs and to every pinhole camera, which closes loops in the
   calibration graph and lets a final global optimization distribute error
   instead of letting it accumulate along chains.

## The calibration graph

Nodes are sensor frames; edges are pairwise calibrations we can actually measure.
`base_link` connects to the reference via the fixed CAD value for `lidar_hesai`.

```
                 base_link
                    | (CAD, fixed)
                 lidar_hesai  ============ lidar_livox_avia
                  /     \      (DONE,            /   |   \
            OAK-4D    Insta360  motion       OAK-1 OAKlite RealSense
               \       / | \    hand-eye)      \    |    /
                \     /  |  \____________________\___|___/
                 (Insta360 also ties to every camera -> redundant loops)
```

- Solid backbone (minimum spanning tree, one path per sensor to the reference):
  - `lidar_hesai -> lidar_livox_avia`  (motion-based hand-eye) **[done]**
  - `lidar_hesai -> OAK-4D`            (target-based camera&harr;LiDAR, front)
  - `lidar_livox_avia -> OAK-1`        (target-based camera&harr;LiDAR, rear)
  - `lidar_livox_avia -> OAK-D Lite`   (target-based camera&harr;LiDAR, rear)
  - `lidar_livox_avia -> RealSense`    (target-based camera&harr;LiDAR, rear)
  - `lidar_hesai -> Insta360`          (target-based camera&harr;LiDAR)
- Redundant loop closures (over-constrain the graph, used by global step):
  - `Insta360 -> OAK-4D / OAK-1 / OAK-D Lite / RealSense` (camera&harr;camera, both
    cameras see the same board at the same instant).

## Methods per edge

**Camera&harr;LiDAR (backbone).** Target-based, with a ChArUco board (a checkerboard
augmented with ArUco markers; robust to partial views and pose-unambiguous). The
camera detects the board and `solvePnP` gives the board plane in the camera frame;
the LiDAR sees the board as a planar patch and we fit its plane. Across many board
orientations, aligning the plane normals fixes the rotation and the plane offsets
fix the translation (the same plane-correspondence idea used for the LiDARs,
applied cross-modally), refined by a point-to-plane least squares. The board's
printed pattern is irrelevant to the LiDAR; what matters is that the board is held
**flat, rigid, isolated from background, and presented at many tilts/positions and
distances** inside the shared region.

**Camera&harr;camera (loop closures).** When two cameras both see the same board in
the same frame, each `solvePnP` gives the board pose in its own frame and the
relative transform is `T_a_b = T_a_board * inv(T_b_board)`, averaged robustly over
many frames. This is how the Insta360 hub ties front and rear groups together.

**Insta360 specifics.** It is a 360/fisheye camera; its images need a fisheye or
equirectangular model, not the pinhole model used for the OAK/RealSense cameras.
We calibrate its intrinsics with the fisheye model first, then treat it like any
other camera in the graph.

**IMU (Xsens).** Motion-based: reuse a walking sequence; align IMU-integrated
rotation to LiDAR-odometry rotation (hand-eye on rotation), then estimate the
lever arm. Needs the same 3-axis excitation as the LiDAR&harr;LiDAR step.

**GNSS.** Single antenna -> rotation is not meaningful; only the lever-arm
translation is estimated, during an outdoor RTK run by comparing GNSS positions to
LiDAR-inertial odometry.

## Ordering

1. **Intrinsics for every camera first.** Extrinsic and intrinsic error are not
   separable, so lock these down: ChArUco intrinsics for OAK-4D, OAK-1, OAK-D Lite
   and RealSense (pinhole); fisheye model for the Insta360. Confirm/refresh the
   existing scripts in `Camera_Calibration_Jakob/`.
2. **`lidar_hesai -> lidar_livox_avia`** &mdash; done (`lidar_lidar/`).
3. **Camera&harr;LiDAR backbone** &mdash; `OAK-4D -> Hesai`, then
   `{OAK-1, OAK-D Lite, RealSense} -> Livox`, then `Insta360 -> Hesai`.
   (This is the tool we build next, in `camera_lidar/`.)
4. **Camera&harr;camera loop closures** through the Insta360.
5. **Global pose-graph optimization** over all edges, fixing `lidar_hesai`,
   producing one consistent `base_link -> sensor` set and a loop-closure residual
   as the quality metric.
6. **IMU**, then **GNSS lever arm**.
7. **Validation:** reproject LiDAR points into each camera image (edge alignment),
   overlay maps, and check the global loop residuals. Write
   `extrinsics_calibrated.yaml`; never overwrite the CAD initial or the SolidWorks
   export.

## Why not camera&harr;camera first, or one big bundle adjustment from the start?

Camera&harr;camera alone cannot reach the LiDAR reference, and most camera pairs do
not overlap anyway (they are split front/rear like the LiDARs). Building the
LiDAR-anchored backbone first guarantees every sensor has a metric path to
`base_link`; the Insta360 loop closures and the global optimization then *refine*
that backbone rather than being load-bearing on their own. This keeps each step
independently checkable and prevents one bad sensor from silently corrupting the
rest.

## Deployment

All steps run on the Jetson (where the bags are recorded) using prebuilt aarch64
wheels (`kiss-icp`, `opencv-contrib-python`, `numpy`, `scipy`, `rosbags`). Each
stage reads a ROS 2 bag and writes/updates `extrinsics_calibrated.yaml` in place,
so there is nothing to copy off-device except, optionally, the validation plots.
