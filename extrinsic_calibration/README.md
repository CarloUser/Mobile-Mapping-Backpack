# Extrinsic Calibration Workflow

This folder tracks the calibration workflow built around the CAD initial
extrinsics in `ros2_ws/src/mmb_bringup/config/extrinsics_initial.yaml`.

## Current Frame Anchor

`base_link` is the center of the rigid sensor-mount frame exported from
SolidWorks. All initial static transforms are expressed as `base_link -> sensor`.

## Initial Checklist

1. Confirm `Oak_4_light` in the SolidWorks export is the OAK-D Lite.
2. Confirm the Insta360 ROS driver publishes a frame compatible with `camera_insta360`.
3. Add Livox Avia and Insta360 topics to the recorder after their launch files
   and topic names are committed.
4. Run `scripts/validate_initial_extrinsics.py` after every CAD export update.

## Calibration Order

1. Validate TF and topic availability.
2. Calibrate `lidar_hesai <-> lidar_livox_avia`.
3. Calibrate each camera to the LiDAR/base reference using a target visible to
   both cameras and LiDARs.
4. Refine/check `imu_xsens` alignment using motion data.
5. Check the `gnss_antenna` lever arm during an outdoor RTK run.
6. Save refined results as a calibrated extrinsics YAML instead of overwriting
   the CAD initial estimates.
