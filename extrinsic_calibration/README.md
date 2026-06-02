# Extrinsic Calibration Workflow

This folder tracks the calibration workflow built around the CAD initial
extrinsics in `ros2_ws/src/mmb_bringup/config/extrinsics_initial.yaml`.

## Current Frame Anchor

`base_link` is the center of the rigid sensor-mount frame exported from
SolidWorks. All initial static transforms are expressed as `base_link -> sensor`.

## Initial Checklist

1. Confirm `Oak_4_light` in the SolidWorks export is the OAK-D Lite.
2. Confirm the Insta360 ROS driver publishes a frame compatible with `camera_insta360`.
3. Confirm OAK-1 is physically mounted and decide whether it needs ROS bringup
   and recording support.
4. Add Insta360 recorder topics after the actual ROS driver topics are known.
5. Run the readiness checks after every CAD export or topic update.

## Readiness Check

From the repository root:

```bash
python extrinsic_calibration/scripts/check_calibration_readiness.py
```

On the PC that will run LiDAR-LiDAR calibration, install dependencies first and
then run the strict dependency check:

```bash
pip install -r extrinsic_calibration/lidar_lidar/requirements.txt
python extrinsic_calibration/scripts/check_calibration_readiness.py --strict-deps
```

The non-strict check is expected to pass on development machines even when
KISS-ICP/scipy/matplotlib are not installed yet; it will warn about those.

## Calibration Order

1. Validate TF and topic availability.
2. Calibrate `lidar_hesai <-> lidar_livox_avia`.
3. Calibrate each camera to the LiDAR/base reference using a target visible to
   both cameras and LiDARs.
4. Refine/check `imu_xsens` alignment using motion data.
5. Check the `gnss_antenna` lever arm during an outdoor RTK run.
6. Save refined results as a calibrated extrinsics YAML instead of overwriting
   the CAD initial estimates.
