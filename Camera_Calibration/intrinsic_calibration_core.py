import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

try:
    cv2.setLogLevel(0)
except AttributeError:
    pass

try:
    import depthai as dai
except ImportError:
    dai = None

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


SCRIPT_DIR = Path(__file__).resolve().parent
IMAGE_ROOT = SCRIPT_DIR / "images"
CALIB_ROOT = SCRIPT_DIR / "calibration_results"
DEPTHAI_KEEPALIVE = []

BACKENDS = {
    "dshow": cv2.CAP_DSHOW,
    "any": cv2.CAP_ANY,
    "msmf": cv2.CAP_MSMF,
}


@dataclass
class CameraState:
    name: str
    reader: object
    image_dir: Path
    fisheye: bool = False
    saved: int = 0
    file_index: int = 0
    valid_corners: list = field(default_factory=list)
    valid_ids: list = field(default_factory=list)
    fisheye_obj_pts: list = field(default_factory=list)
    fisheye_img_pts: list = field(default_factory=list)
    image_size: tuple | None = None
    last_marker_count: int = 0
    last_ch_count: int = 0


class OpenCVCamera:
    def __init__(self, index, backend, width, height, crop, max_index=6, allow_index_0=False):
        self.crop = crop
        self.last_full_frame = None
        self.cap = None
        self.index = None
        self.backend = None

        candidate_indices = [index] + [
            candidate
            for candidate in range(0 if allow_index_0 else 1, max_index + 1)
            if candidate != index
        ]
        candidate_backends = [backend] + [name for name in BACKENDS if name != backend]

        for candidate_index in candidate_indices:
            if candidate_index == 0 and not allow_index_0:
                continue
            for candidate_backend in candidate_backends:
                cap = cv2.VideoCapture(candidate_index, BACKENDS[candidate_backend])
                if not cap.isOpened():
                    cap.release()
                    continue
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                ok = False
                frame = None
                for _ in range(15):
                    ok, frame = cap.read()
                if ok and frame is not None:
                    self.cap = cap
                    self.index = candidate_index
                    self.backend = candidate_backend
                    print(
                        f"Using OpenCV camera index {self.index} with backend {self.backend}: "
                        f"requested {width}x{height}, actual {frame.shape[1]}x{frame.shape[0]}, "
                        f"calibration uses {describe_crop(frame, self.crop)}",
                        flush=True,
                    )
                    return
                cap.release()

        raise RuntimeError(
            f"Could not open any OpenCV camera stream. Tried preferred index {index}."
        )

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        self.last_full_frame = frame.copy()
        return crop_frame(frame, self.crop)

    def preview_frame(self, frame):
        return frame

    def close(self):
        self.cap.release()


class RealSenseCamera:
    def __init__(self, width, height, fps):
        if rs is None:
            raise RuntimeError("pyrealsense2 is not installed.")
        self.pipeline = rs.pipeline()
        profile_attempts = [
            (width, height, int(fps)),
            (1920, 1080, 8),
            (width, height, 15),
            (width, height, 10),
            (1280, 720, 15),
            (640, 480, 30),
            (640, 480, 15),
        ]
        profile_attempts = list(dict.fromkeys(profile_attempts))
        last_error = None
        for profile_width, profile_height, profile_fps in profile_attempts:
            self.config = rs.config()
            self.config.enable_stream(
                rs.stream.color,
                profile_width,
                profile_height,
                rs.format.bgr8,
                profile_fps,
            )
            try:
                self.pipeline.start(self.config)
                print(
                    f"Using RealSense color stream {profile_width}x{profile_height}@{profile_fps}",
                    flush=True,
                )
                break
            except RuntimeError as exc:
                last_error = exc
        else:
            raise RuntimeError(f"Could not start RealSense color stream: {last_error}")

        self.missed_frames = 0

    def read(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=500)
        except RuntimeError:
            self.missed_frames += 1
            if self.missed_frames % 10 == 0:
                print("RealSense: still waiting for color frames...", flush=True)
            return None
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        self.missed_frames = 0
        return np.asanyarray(color_frame.get_data())

    def close(self):
        self.pipeline.stop()


class DepthAICamera:
    def __init__(self, device_info, width, height, fps, socket_name):
        if dai is None:
            raise RuntimeError("depthai is not installed.")
        self.device = dai.Device(device_info)
        self.pipeline = dai.Pipeline(self.device)
        DEPTHAI_KEEPALIVE.append(self.device)
        DEPTHAI_KEEPALIVE.append(self.pipeline)
        socket = getattr(dai.CameraBoardSocket, socket_name)
        cam = self.pipeline.create(dai.node.Camera).build(socket)
        output = cam.requestOutput(
            (width, height),
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.CROP,
            fps=fps,
        )
        self.queue = output.createOutputQueue(maxSize=1, blocking=False)
        DEPTHAI_KEEPALIVE.append(self.queue)
        self.pipeline.start()
        print(f"Using DepthAI USB stream {width}x{height}@{fps}", flush=True)

    def read(self):
        packet = self.queue.tryGet()
        if packet is None:
            return None
        return packet.getCvFrame()

    def close(self):
        # Avoid DepthAI 3.6.1 / Python 3.13 shutdown crash.
        pass


class OAK4DProCamera:
    def __init__(self, ip_address, width, height, fps, socket_name):
        if dai is None:
            raise RuntimeError("depthai is not installed.")
        if ip_address:
            device_info = dai.DeviceInfo(ip_address)
        else:
            devices = depthai_oak4d_devices()
            if not devices:
                raise RuntimeError("No OAK 4D Pro found. Pass --oak4d-ip CAMERA_IP.")
            device_info = devices[0]

        print(f"Opening OAK4D Pro device: {device_info}", flush=True)
        self.device = dai.Device(device_info)
        self.pipeline = dai.Pipeline(self.device)
        DEPTHAI_KEEPALIVE.append(self.device)
        DEPTHAI_KEEPALIVE.append(self.pipeline)
        socket = getattr(dai.CameraBoardSocket, socket_name)
        cam = self.pipeline.create(dai.node.Camera).build(socket)
        output = cam.requestOutput(
            (width, height),
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.CROP,
            fps=fps,
        )
        self.queue = output.createOutputQueue(maxSize=2, blocking=False)
        DEPTHAI_KEEPALIVE.append(self.queue)
        self.pipeline.start()
        print(f"Using OAK4D stream {width}x{height}@{fps}", flush=True)

    def read(self):
        packet = self.queue.tryGet()
        if packet is None:
            return None
        return packet.getCvFrame()

    def close(self):
        pass


def crop_frame(frame, crop):
    if crop == "full":
        return frame
    h, w = frame.shape[:2]
    if crop == "top":
        return frame[: h // 2, :]
    if crop == "bottom":
        return frame[h // 2 :, :]
    if crop == "left":
        return frame[:, : w // 2]
    if crop == "right":
        return frame[:, w // 2 :]
    raise ValueError(f"Unknown crop: {crop}")


def describe_crop(frame, crop):
    cropped = crop_frame(frame, crop)
    return f"{crop} crop {cropped.shape[1]}x{cropped.shape[0]} from {frame.shape[1]}x{frame.shape[0]}"


def get_insta360_devices():
    command = (
        "Get-CimInstance Win32_PnPEntity | "
        "Where-Object { $_.Name -match 'Insta360|Insta|360' } | "
        "Select-Object Name,Present,Status,PNPClass,ConfigManagerErrorCode,DeviceID | "
        "ConvertTo-Json -Compress"
    )
    result = subprocess.run(
        ["powershell", "-NoProfile", "-Command", command],
        capture_output=True,
        text=True,
        check=False,
    )
    text = result.stdout.strip()
    if not text:
        return []
    data = json.loads(text)
    return [data] if isinstance(data, dict) else data


def get_windows_realsense_devices():
    command = (
        "Get-CimInstance Win32_PnPEntity | "
        "Where-Object { $_.Name -match 'RealSense|D435|Depth Camera' -or "
        "$_.DeviceID -match 'VID_8086&PID_0B3A' } | "
        "Select-Object Name,Present,Status,PNPClass,ConfigManagerErrorCode,DeviceID | "
        "ConvertTo-Json -Compress"
    )
    result = subprocess.run(
        ["powershell", "-NoProfile", "-Command", command],
        capture_output=True,
        text=True,
        check=False,
    )
    text = result.stdout.strip()
    if not text:
        return []
    data = json.loads(text)
    return [data] if isinstance(data, dict) else data


def require_insta360(skip_check):
    if skip_check:
        return
    devices = get_insta360_devices()
    usable = any(d.get("Present") is True and d.get("PNPClass") == "Camera" for d in devices)
    if not usable:
        print("Warning: Windows is not naming a present Insta360 Camera device.")
        print("The script will still try the configured OpenCV Insta360 index.")


def make_board(args):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
    board = cv2.aruco.CharucoBoard(
        (args.board_cols, args.board_rows),
        args.square_size,
        args.square_size * args.marker_ratio,
        dictionary,
    )
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(True)
    params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        detect = lambda gray: detector.detectMarkers(gray)
    else:
        detect = lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    return board, detect


def detect_board(frame, board, detect):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect(gray)
    marker_count = 0 if ids is None else len(ids)
    if ids is None:
        return corners, ids, None, None, marker_count, 0
    ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    ch_count = 0 if ch_ids is None else len(ch_ids)
    return corners, ids, ch_corners, ch_ids, marker_count, ch_count


def draw_status(
    frame,
    corners,
    ids,
    ch_corners,
    ch_ids,
    marker_count,
    ch_count,
    name,
    saved,
    target,
    min_features,
    max_width,
):
    display = frame.copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(display, corners, ids)
    # Draw actual ChArUco chessboard corners too. Marker boxes alone can look
    # like the board is "detected", but calibration only saves if ChArUco
    # chessboard corners are interpolated successfully.
    if ch_corners is not None and ch_ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(display, ch_corners, ch_ids, (255, 0, 255))
    ok = ch_count >= min_features
    color = (0, 255, 0) if ok else (0, 0, 255)
    cv2.putText(
        display,
        f"{name}: markers {marker_count} charuco {ch_count}/{min_features} saved {saved}/{target}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
        2,
    )
    h, w = display.shape[:2]
    if w > max_width:
        display = cv2.resize(display, (max_width, int(h * max_width / w)))
    return display


def next_frame_index(image_dir):
    max_index = -1
    for path in image_dir.glob("frame_*.png"):
        try:
            max_index = max(max_index, int(path.stem.split("_")[-1]))
        except ValueError:
            continue
    return max_index + 1


def prepare_capture_dir(state):
    state.image_dir.mkdir(parents=True, exist_ok=True)
    state.file_index = next_frame_index(state.image_dir)
    if state.file_index:
        print(
            f"{state.name}: existing images found; new files start at frame_{state.file_index:04d}.png",
            flush=True,
        )


def save_sample(state, frame, board, ch_corners, ch_ids, is_valid):
    """Save every timed frame, but only keep valid ChArUco detections for calibration."""
    state.image_dir.mkdir(parents=True, exist_ok=True)
    output_path = state.image_dir / f"frame_{state.file_index:04d}.png"
    ok = cv2.imwrite(os.fspath(output_path), frame)
    if not ok:
        raise RuntimeError(f"{state.name}: cv2.imwrite failed for {output_path}")

    state.file_index += 1
    state.saved += 1  # total timed pictures saved, valid or invalid
    state.image_size = frame.shape[1], frame.shape[0]

    if is_valid:
        state.valid_corners.append(ch_corners.astype(np.float32))
        state.valid_ids.append(ch_ids)
        if state.fisheye:
            obj = board.getChessboardCorners()[ch_ids.flatten()]
            state.fisheye_obj_pts.append(obj.reshape(-1, 1, 3).astype(np.float32))
            state.fisheye_img_pts.append(ch_corners.astype(np.float32))

    return output_path



def calibrate_camera(state, board, min_valid):
    if len(state.valid_corners) < min_valid:
        print(f"{state.name}: skipped, only {len(state.valid_corners)} valid frames")
        return

    if state.fisheye:
        k = np.zeros((3, 3))
        d = np.zeros((4, 1))
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
        rms, k, d, _, _ = cv2.fisheye.calibrate(
            state.fisheye_obj_pts,
            state.fisheye_img_pts,
            state.image_size,
            k,
            d,
            flags=flags,
        )
        dist = d
    else:
        rms, k, dist, _, _ = cv2.aruco.calibrateCameraCharuco(
            state.valid_corners,
            state.valid_ids,
            board,
            state.image_size,
            None,
            None,
        )

    CALIB_ROOT.mkdir(parents=True, exist_ok=True)
    output = CALIB_ROOT / f"{state.name}_intrinsics.yaml"
    fs = cv2.FileStorage(os.fspath(output), cv2.FILE_STORAGE_WRITE)
    fs.write("camera_name", state.name)
    fs.write("camera_matrix", k)
    fs.write("dist_coeffs", dist)
    fs.write("reprojection_error", float(rms))
    fs.write("image_width", int(state.image_size[0]))
    fs.write("image_height", int(state.image_size[1]))
    fs.write("fisheye_model", bool(state.fisheye))
    fs.write("valid_frames", int(len(state.valid_corners)))
    fs.release()

    print(f"\n{state.name}: saved {output}")
    print(f"{state.name}: reprojection error {rms:.4f} px")
    print(f"{state.name}: K =\n{k}")
    print(f"{state.name}: distortion = {dist.T}")


def depthai_devices_by_mxid():
    if dai is None:
        return {}
    return {device.deviceId: device for device in dai.Device.getAllAvailableDevices()}


def depthai_usb_oak_devices():
    if dai is None:
        return []
    return [
        device
        for device in dai.Device.getAllAvailableDevices()
        if "X_LINK_USB" in str(device) and "X_LINK_MYRIAD_X" in str(device)
    ]


def depthai_oak4d_devices():
    if dai is None:
        return []
    return [
        device
        for device in dai.Device.getAllAvailableDevices()
        if "X_LINK_TCP_IP" in str(device) or "X_LINK_RVC4" in str(device)
    ]


def realsense_devices():
    if rs is None:
        return []
    return list(rs.context().devices)


def preflight_back_stage(args):
    if not args.skip_oaks and len(depthai_usb_oak_devices()) < 2:
        raise RuntimeError(
            "Back stage needs 2 USB OAK devices. Run --list-devices and check "
            "that OAK1 and OAK-Lite are both listed as X_LINK_USB / MYRIAD_X."
        )

    if not args.skip_realsense and len(realsense_devices()) < 1:
        raise RuntimeError(
            "Back stage needs the Intel RealSense, but pyrealsense2 sees no device. "
            "Reconnect the RealSense directly over USB3, then rerun --list-devices."
        )


def preflight_front_stage(args):
    if not args.skip_oak4d and not args.oak4d_ip and len(depthai_oak4d_devices()) < 1:
        raise RuntimeError(
            "Front stage needs the OAK4D Pro. It was not found over DepthAI TCP/IP. "
            "Pass --oak4d-ip CAMERA_IP if auto-discovery fails."
        )


def print_devices(args):
    print("DepthAI / OAK devices:")
    if dai is None:
        print("  depthai is not installed")
    else:
        devices = dai.Device.getAllAvailableDevices()
        if not devices:
            print("  none")
        for device in devices:
            print(f"  {device}")

    print("RealSense devices:")
    if rs is None:
        print("  pyrealsense2 is not installed")
    else:
        devices = realsense_devices()
        if len(devices) == 0:
            print("  none")
        for device in devices:
            print(
                "  "
                + device.get_info(rs.camera_info.name)
                + " "
                + device.get_info(rs.camera_info.serial_number)
            )
    windows_realsense = get_windows_realsense_devices()
    if windows_realsense:
        print("Windows RealSense PnP entries:")
        for device in windows_realsense:
            print(
                f"  {device.get('Name')}: present={device.get('Present')}, "
                f"status={device.get('Status')}, class={device.get('PNPClass')}, "
                f"error={device.get('ConfigManagerErrorCode')}"
            )
    else:
        print("Windows RealSense PnP entries:")
        print("  none")

    print("Insta360 OpenCV streams:")
    for backend_name, backend in BACKENDS.items():
        for index in range(args.max_opencv_index + 1):
            cap = cv2.VideoCapture(index, backend)
            if not cap.isOpened():
                cap.release()
                continue
            ok = False
            frame = None
            for _ in range(10):
                ok, frame = cap.read()
            cap.release()
            if ok:
                print(f"  {backend_name} index {index}: {frame.shape}")


def pick_depthai_device(devices_by_id, mxid, fallback_index, name):
    if mxid:
        if mxid not in devices_by_id:
            raise RuntimeError(f"{name} MXID {mxid} not found.")
        return devices_by_id[mxid]
    devices = list(devices_by_id.values())
    if len(devices) <= fallback_index:
        raise RuntimeError(f"No DepthAI device available for {name}.")
    return devices[fallback_index]


def build_back_stage(args):
    require_insta360(args.skip_insta_check)
    preflight_back_stage(args)
    cameras = []
    devices_by_id = {device.deviceId: device for device in depthai_usb_oak_devices()}

    if not args.skip_realsense:
        cameras.append(
            CameraState(
                "intel_back",
                RealSenseCamera(
                    args.realsense_width,
                    args.realsense_height,
                    args.realsense_fps,
                ),
                IMAGE_ROOT / "intel_back",
            )
        )

    if not args.skip_oaks:
        cameras.append(
            CameraState(
                "oak1_back",
                DepthAICamera(
                    pick_depthai_device(devices_by_id, args.oak1_mxid, 0, "oak1"),
                    args.oak_width,
                    args.oak_height,
                    args.oak_fps,
                    args.oak_socket,
                ),
                IMAGE_ROOT / "oak1_back",
            )
        )
        cameras.append(
            CameraState(
                "oaklite_back",
                DepthAICamera(
                    pick_depthai_device(devices_by_id, args.oaklite_mxid, 1, "oaklite"),
                    args.oak_width,
                    args.oak_height,
                    args.oak_fps,
                    args.oak_socket,
                ),
                IMAGE_ROOT / "oaklite_back",
            )
        )

    cameras.append(
        CameraState(
            "insta360_back",
            OpenCVCamera(
                args.insta_index,
                args.insta_backend,
                args.insta_width,
                args.insta_height,
                args.insta_back_crop,
                args.max_opencv_index,
                args.allow_index_0,
            ),
            IMAGE_ROOT / "insta360_back",
            fisheye=True,
        )
    )
    return cameras


def build_front_stage(args):
    require_insta360(args.skip_insta_check)
    preflight_front_stage(args)
    cameras = []
    if not args.skip_oak4d:
        cameras.append(
            CameraState(
                "oak4d_front",
                OAK4DProCamera(
                    args.oak4d_ip,
                    args.oak4d_width,
                    args.oak4d_height,
                    args.oak4d_fps,
                    args.oak_socket,
                ),
                IMAGE_ROOT / "oak4d_front",
            )
        )

    cameras.append(
        CameraState(
            "insta360_front",
            OpenCVCamera(
                args.insta_index,
                args.insta_backend,
                args.insta_width,
                args.insta_height,
                args.insta_front_crop,
                args.max_opencv_index,
                args.allow_index_0,
            ),
            IMAGE_ROOT / "insta360_front",
            fisheye=True,
        )
    )
    return cameras


def run_stage(stage_name, cameras, board, detect, args):
    stage_label = ", ".join(c.name for c in cameras)
    window_name = f"{stage_name} preview: {stage_label}"
    for state in cameras:
        prepare_capture_dir(state)

    print(f"\n=== {stage_name.upper()} STAGE ===")
    print(
        f"Move the board in front of: {', '.join(c.name for c in cameras)}. "
        f"Waiting {args.initial_delay}s, then checking 1 image every {args.capture_interval}s. "
        f"Only frames with >= {args.min_features} ChArUco corners are saved. "
        f"Calibration starts after {args.target_valid} valid frames are collected."
    )
    print("Press Q to finish this stage early.")

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, args.preview_width, 720)
    window_positions = {
        "intel_back": (0, 0),
        "oak1_back": (520, 0),
        "oaklite_back": (1040, 0),
        "insta360_back": (0, 560),
        "oak4d_front": (520, 560),
        "insta360_front": (1040, 560),
    }
    first_name = cameras[0].name if cameras else ""
    x, y = window_positions.get(first_name, (100, 100))
    cv2.moveWindow(window_name, x, y)
    placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(
        placeholder,
        f"{stage_label}: waiting for frames",
        (30, 180),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
    )
    cv2.imshow(window_name, placeholder)
    cv2.waitKey(1)

    start = time.monotonic()

    # Simple timed capture logic:
    # 1. Wait args.initial_delay seconds so you can position the board.
    # 2. Every args.capture_interval seconds, check the current frame.
    # 3. Save only frames with enough ChArUco corners for calibration.
    next_capture = {state.name: start + args.initial_delay for state in cameras}

    last_frame_report = start
    last_status_report = {state.name: start for state in cameras}
    frame_counts = {state.name: 0 for state in cameras}
    stop_reason = "user stopped"

    while True:
        now = time.monotonic()
        if all(len(c.valid_corners) >= args.target_valid for c in cameras):
            stop_reason = "target valid calibration frames reached"
            break
        if all(c.saved >= args.max_saved for c in cameras):
            stop_reason = "maximum saved picture count reached"
            break
        if now - start >= args.stage_timeout:
            stop_reason = "stage timeout"
            break

        previews = []

        for state in cameras:
            frame = state.reader.read()
            if frame is None:
                continue

            frame_counts[state.name] += 1
            preview_frame = (
                state.reader.preview_frame(frame)
                if hasattr(state.reader, "preview_frame")
                else frame
            )

            corners, ids, ch_corners, ch_ids, marker_count, ch_count = detect_board(
                frame, board, detect
            )
            state.last_marker_count = marker_count
            state.last_ch_count = ch_count

            (
                preview_corners,
                preview_ids,
                preview_ch_corners,
                preview_ch_ids,
                preview_marker_count,
                preview_ch_count,
            ) = detect_board(preview_frame, board, detect)

            previews.append(
                draw_status(
                    preview_frame,
                    preview_corners,
                    preview_ids,
                    preview_ch_corners,
                    preview_ch_ids,
                    preview_marker_count,
                    preview_ch_count,
                    state.name,
                    len(state.valid_corners),
                    args.target_valid,
                    args.min_features,
                    args.preview_width,
                )
            )

            if (
                len(state.valid_corners) < args.target_valid
                and state.saved < args.max_saved
                and now >= next_capture[state.name]
            ):
                is_valid = (
                    ch_count >= args.min_features
                    and ch_corners is not None
                    and ch_ids is not None
                )

                if is_valid:
                    output_path = save_sample(
                        state,
                        frame,
                        board,
                        ch_corners,
                        ch_ids,
                        is_valid,
                    )
                    valid_count = len(state.valid_corners)
                    print(
                        f"{state.name}: picture {state.saved}/{args.target_valid} saved -> {output_path} | "
                        f"VALID for calibration | charuco {ch_count}/{args.min_features}, markers {marker_count}",
                        flush=True,
                    )
                else:
                    valid_count = len(state.valid_corners)
                    print(
                        f"{state.name}: not saved | charuco {ch_count}/{args.min_features}, markers {marker_count} | "
                        f"valid calibration frames {valid_count}/{args.target_valid}",
                        flush=True,
                    )

                # Schedule the next picture from now. This avoids burst-saving
                # several frames if one loop iteration was delayed.
                next_capture[state.name] = now + args.capture_interval

            elif now < next_capture[state.name] and now - last_status_report[state.name] >= 1.0:
                wait_left = next_capture[state.name] - now
                print(
                    f"{state.name}: next timed picture in {wait_left:.1f}s | "
                    f"currently charuco {ch_count}/{args.min_features}, markers {marker_count}",
                    flush=True,
                )
                last_status_report[state.name] = now

        if previews:
            max_h = max(p.shape[0] for p in previews)
            padded = []
            for preview in previews:
                if preview.shape[0] < max_h:
                    preview = cv2.copyMakeBorder(
                        preview,
                        0,
                        max_h - preview.shape[0],
                        0,
                        0,
                        cv2.BORDER_CONSTANT,
                    )
                padded.append(preview)
            cv2.imshow(window_name, np.hstack(padded))
        elif now - last_frame_report >= 3.0:
            counts = ", ".join(f"{name}={count}" for name, count in frame_counts.items())
            print(f"{stage_name}: no preview frames yet ({counts})", flush=True)
            last_frame_report = now
            cv2.imshow(window_name, placeholder)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            stop_reason = "Q pressed"
            break

    cv2.destroyAllWindows()
    print(f"{stage_name}: stopped because {stop_reason}", flush=True)
    for state in cameras:
        print(
            f"{state.name}: saved pictures {state.saved}/{args.max_saved}; "
            f"valid calibration frames {len(state.valid_corners)}; "
            f"last seen charuco {state.last_ch_count}/{args.min_features}, markers {state.last_marker_count}",
            flush=True,
        )
    for state in cameras:
        try:
            state.reader.close()
        except Exception as exc:
            print(f"{state.name}: close warning: {exc}")


def calibrate_stage(cameras, board, args):
    for state in cameras:
        calibrate_camera(state, board, args.min_valid)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Two-stage intrinsic calibration for back and front camera groups."
    )
    parser.add_argument("--target-valid", type=int, default=30)
    parser.add_argument("--min-valid", type=int, default=20)
    parser.add_argument("--max-saved", type=int, default=40)
    parser.add_argument("--min-features", type=int, default=15)
    parser.add_argument("--capture-interval", type=float, default=1.5)
    parser.add_argument("--initial-delay", type=float, default=5.0)
    parser.add_argument("--stage-timeout", type=float, default=1200.0)
    parser.add_argument("--preview-width", type=int, default=1280)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=8.0)
    parser.add_argument("--realsense-width", type=int, default=1920)
    parser.add_argument("--realsense-height", type=int, default=1080)
    parser.add_argument("--realsense-fps", type=float, default=8.0)
    parser.add_argument("--oak-width", type=int, default=1920)
    parser.add_argument("--oak-height", type=int, default=1080)
    parser.add_argument("--oak-fps", type=float, default=15.0)
    parser.add_argument("--oak4d-width", type=int, default=3840)
    parser.add_argument("--oak4d-height", type=int, default=2160)
    parser.add_argument("--oak4d-fps", type=float, default=5.0)
    parser.add_argument("--dictionary", default="DICT_5X5_250")
    parser.add_argument("--board-cols", type=int, default=11)
    parser.add_argument("--board-rows", type=int, default=8)
    parser.add_argument("--square-size", type=float, default=0.034)
    parser.add_argument("--marker-ratio", type=float, default=25.0 / 34.0)
    parser.add_argument("--oak-socket", default="CAM_A")
    parser.add_argument("--oak1-mxid")
    parser.add_argument("--oaklite-mxid")
    parser.add_argument("--oak4d-ip", default="192.168.25.97")
    parser.add_argument("--insta-index", type=int, default=1)
    parser.add_argument("--insta-backend", choices=sorted(BACKENDS), default="dshow")
    parser.add_argument("--insta-width", type=int, default=1920)
    parser.add_argument("--insta-height", type=int, default=1080)
    parser.add_argument(
        "--insta-back-crop",
        choices=("top", "bottom", "left", "right", "full"),
        default="top",
    )
    parser.add_argument(
        "--insta-front-crop",
        choices=("top", "bottom", "left", "right", "full"),
        default="bottom",
    )
    parser.add_argument("--skip-oaks", action="store_true")
    parser.add_argument("--skip-realsense", action="store_true")
    parser.add_argument("--skip-oak4d", action="store_true")
    parser.add_argument("--skip-insta-check", action="store_true")
    parser.add_argument("--allow-index-0", action="store_true")
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("--max-opencv-index", type=int, default=6)
    parser.add_argument("--internal-stage", choices=("back", "front"), help=argparse.SUPPRESS)
    parser.add_argument(
        "--internal-camera",
        choices=(
            "intel_back",
            "oak1_back",
            "oaklite_back",
            "insta360_back",
            "oak4d_front",
            "insta360_front",
        ),
        help=argparse.SUPPRESS,
    )
    return parser.parse_args()


def build_single_camera(camera_name, args):
    if camera_name == "intel_back":
        if len(realsense_devices()) < 1:
            raise RuntimeError("Intel RealSense is not visible to pyrealsense2.")
        return CameraState(
            "intel_back",
            RealSenseCamera(
                args.realsense_width,
                args.realsense_height,
                args.realsense_fps,
            ),
            IMAGE_ROOT / "intel_back",
        )

    if camera_name in ("oak1_back", "oaklite_back"):
        devices_by_id = {device.deviceId: device for device in depthai_usb_oak_devices()}
        if camera_name == "oak1_back":
            device = pick_depthai_device(devices_by_id, args.oak1_mxid, 0, "oak1")
            label = "oak1_back"
        else:
            fallback_index = 1 if len(devices_by_id) > 1 else 0
            device = pick_depthai_device(
                devices_by_id, args.oaklite_mxid, fallback_index, "oaklite"
            )
            label = "oaklite_back"
        return CameraState(
            label,
            DepthAICamera(
                device,
                args.oak_width,
                args.oak_height,
                args.oak_fps,
                args.oak_socket,
            ),
            IMAGE_ROOT / label,
        )

    if camera_name == "insta360_back":
        require_insta360(args.skip_insta_check)
        return CameraState(
            "insta360_back",
            OpenCVCamera(
                args.insta_index,
                args.insta_backend,
                args.insta_width,
                args.insta_height,
                args.insta_back_crop,
                args.max_opencv_index,
                args.allow_index_0,
            ),
            IMAGE_ROOT / "insta360_back",
            fisheye=True,
        )

    if camera_name == "oak4d_front":
        preflight_front_stage(args)
        return CameraState(
            "oak4d_front",
            OAK4DProCamera(
                args.oak4d_ip,
                args.oak4d_width,
                args.oak4d_height,
                args.oak4d_fps,
                args.oak_socket,
            ),
            IMAGE_ROOT / "oak4d_front",
        )

    if camera_name == "insta360_front":
        require_insta360(args.skip_insta_check)
        return CameraState(
            "insta360_front",
            OpenCVCamera(
                args.insta_index,
                args.insta_backend,
                args.insta_width,
                args.insta_height,
                args.insta_front_crop,
                args.max_opencv_index,
                args.allow_index_0,
            ),
            IMAGE_ROOT / "insta360_front",
            fisheye=True,
        )

    raise RuntimeError(f"Unknown internal camera: {camera_name}")


def run_internal_camera(args):
    board, detect = make_board(args)
    state = build_single_camera(args.internal_camera, args)
    stage_name = "front" if args.internal_camera.endswith("_front") else "back"
    run_stage(stage_name, [state], board, detect, args)
    calibrate_stage([state], board, args)

    sys.stdout.flush()
    sys.stderr.flush()
    if isinstance(state.reader, (DepthAICamera, OAK4DProCamera)):
        os._exit(0)


def run_internal_stage(args):
    board, detect = make_board(args)
    if args.internal_stage == "back":
        cameras = build_back_stage(args)
        run_stage("back", cameras, board, detect, args)
        calibrate_stage(cameras, board, args)
    elif args.internal_stage == "front":
        cameras = build_front_stage(args)
        run_stage("front", cameras, board, detect, args)
        calibrate_stage(cameras, board, args)
    else:
        raise RuntimeError(f"Unknown internal stage: {args.internal_stage}")

    sys.stdout.flush()
    sys.stderr.flush()
    if any(isinstance(state.reader, (DepthAICamera, OAK4DProCamera)) for state in cameras):
        os._exit(0)


def run_child_stage(stage_name):
    script = Path(__file__).resolve()
    child_args = [
        arg
        for arg in sys.argv[1:]
        if arg not in ("--list-devices", "--internal-stage", "back", "front")
    ]
    command = [sys.executable, os.fspath(script), *child_args, "--internal-stage", stage_name]
    return subprocess.run(command, cwd=os.fspath(SCRIPT_DIR)).returncode


def clean_child_args():
    cleaned = []
    skip_next = False
    options_with_values = {"--internal-stage", "--internal-camera"}
    for arg in sys.argv[1:]:
        if skip_next:
            skip_next = False
            continue
        if arg in options_with_values:
            skip_next = True
            continue
        if arg == "--list-devices":
            continue
        cleaned.append(arg)
    return cleaned


def run_child_cameras(camera_names):
    script = Path(__file__).resolve()
    base_args = clean_child_args()
    usb_oaks = depthai_usb_oak_devices()
    auto_oak1_mxid = usb_oaks[0].deviceId if len(usb_oaks) > 0 else None
    auto_oaklite_mxid = usb_oaks[1].deviceId if len(usb_oaks) > 1 else None
    has_oak1_mxid = "--oak1-mxid" in base_args
    has_oaklite_mxid = "--oaklite-mxid" in base_args
    processes = []
    for camera_name in camera_names:
        camera_args = list(base_args)
        if camera_name == "oak1_back" and auto_oak1_mxid and not has_oak1_mxid:
            camera_args.extend(["--oak1-mxid", auto_oak1_mxid])
        if (
            camera_name == "oaklite_back"
            and auto_oaklite_mxid
            and not has_oaklite_mxid
        ):
            camera_args.extend(["--oaklite-mxid", auto_oaklite_mxid])
        command = [
            sys.executable,
            os.fspath(script),
            *camera_args,
            "--internal-camera",
            camera_name,
        ]
        processes.append((camera_name, subprocess.Popen(command, cwd=os.fspath(SCRIPT_DIR))))

    failed = []
    for camera_name, process in processes:
        code = process.wait()
        if code != 0:
            failed.append((camera_name, code))
    return failed


def main():
    args = parse_args()
    if args.list_devices:
        print_devices(args)
        return

    if args.internal_camera:
        run_internal_camera(args)
        return

    if args.internal_stage:
        run_internal_stage(args)
        return

    back_cameras = ["intel_back", "oak1_back", "oaklite_back", "insta360_back"]
    if args.skip_realsense:
        back_cameras.remove("intel_back")
    if args.skip_oaks:
        back_cameras.remove("oak1_back")
        back_cameras.remove("oaklite_back")

    print("Starting BACK stage camera workers:", ", ".join(back_cameras), flush=True)
    failed = run_child_cameras(back_cameras)
    if failed:
        detail = ", ".join(f"{name}={code}" for name, code in failed)
        raise RuntimeError(f"Back stage failed: {detail}")

    input(
        "\nBack stage done. Move the calibration board to the FRONT side, "
        "then press ENTER to start OAK4D + Insta360 front calibration..."
    )

    front_cameras = ["oak4d_front", "insta360_front"]
    if args.skip_oak4d:
        front_cameras.remove("oak4d_front")

    print("Starting FRONT stage camera workers:", ", ".join(front_cameras), flush=True)
    failed = run_child_cameras(front_cameras)
    if failed:
        detail = ", ".join(f"{name}={code}" for name, code in failed)
        raise RuntimeError(f"Front stage failed: {detail}")

    print("\nAll requested intrinsics are done. YAML files are in:")
    print(CALIB_ROOT)


if __name__ == "__main__":
    main()
