#!/usr/bin/env python3
"""Minimal OAK-4D (RVC4) RGB publisher built on depthai v3.

WHY THIS EXISTS: the apt depthai_ros_driver_v3 (3.2.1) SIGSEGVs at device
connect against depthai-core 3.7.1 for this RVC4 PoE camera, while the depthai
v3.7.1 Python lib connects + streams + reads calibration perfectly. This node
uses that working lib to publish exactly what the camera<->LiDAR calibration
needs:  <ns>/rgb/image_raw (bgr8)  and  <ns>/rgb/camera_info.

Run:  python3 oak4d_v3_publisher.py [--device-id 1707542538] [--ns oak4d]
      (source /opt/ros/humble/setup.bash first; needs the depthai pip lib + cv2)
Stamps are host-clock (recorder clock) so they line up with the LiDAR in the bag.
"""
import argparse

import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def pick_device(device_id):
    devs = dai.Device.getAllAvailableDevices()
    if device_id:
        for d in devs:
            if str(getattr(d, "deviceId", "")) == device_id or \
               getattr(d, "name", "") == device_id:
                return d
    return devs[0] if devs else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device-id", default=None,
                    help="depthai deviceId or IP (default: first available)")
    ap.add_argument("--ns", default="oak4d")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--frame-id", default="oak4d_rgb_camera_optical_frame")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Node("oak4d_v3_publisher")
    log = node.get_logger()
    pub_img = node.create_publisher(Image, f"/{args.ns}/rgb/image_raw", 10)
    pub_info = node.create_publisher(CameraInfo, f"/{args.ns}/rgb/camera_info", 10)

    info = pick_device(args.device_id)
    device = dai.Device(info) if info is not None else dai.Device()
    log.info(f"OAK-4D connected: platform {device.getPlatformAsString()}")

    # CameraInfo from on-device calibration (constant; stamped per frame).
    cam_info = CameraInfo()
    cam_info.width, cam_info.height = args.width, args.height
    try:
        calib = device.readCalibration()
        K = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A,
                                      args.width, args.height)
        D = list(calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A))
        cam_info.k = [float(x) for row in K for x in row]
        cam_info.d = [float(x) for x in D]
        cam_info.distortion_model = "rational_polynomial" if len(D) >= 8 \
            else "plumb_bob"
        cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        cam_info.p = [K[0][0], 0.0, K[0][2], 0.0,
                      0.0, K[1][1], K[1][2], 0.0,
                      0.0, 0.0, 1.0, 0.0]
        log.info(f"intrinsics: fx={K[0][0]:.1f} fy={K[1][1]:.1f} "
                 f"cx={K[0][2]:.1f} cy={K[1][2]:.1f}, {len(D)} distortion coeffs")
    except Exception as e:  # noqa: BLE001
        log.warn(f"could not read calibration: {e!r} (publishing empty info)")

    pipeline = dai.Pipeline(device)
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    try:
        out = cam.requestOutput((args.width, args.height),
                                dai.ImgFrame.Type.BGR888i, fps=args.fps)
    except TypeError:  # older signature without fps kwarg
        out = cam.requestOutput((args.width, args.height),
                                dai.ImgFrame.Type.BGR888i)
    q = out.createOutputQueue()
    pipeline.start()
    log.info(f"streaming RGB on /{args.ns}/rgb/image_raw")

    try:
        while rclpy.ok():
            f = q.tryGet()
            if f is None:
                rclpy.spin_once(node, timeout_sec=0.005)
                continue
            arr = f.getCvFrame()  # HxWx3 uint8 BGR
            stamp = node.get_clock().now().to_msg()
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = args.frame_id
            msg.height, msg.width = int(arr.shape[0]), int(arr.shape[1])
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = int(arr.shape[1]) * 3
            msg.data = arr.tobytes()
            pub_img.publish(msg)
            cam_info.header.stamp = stamp
            cam_info.header.frame_id = args.frame_id
            pub_info.publish(cam_info)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            pipeline.stop()
            device.close()
        except Exception:  # noqa: BLE001
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
