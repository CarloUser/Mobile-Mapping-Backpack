"""Bag + YAML I/O for the camera<->LiDAR stage.

Reads ROS 2 Humble bags with `rosbags` (no ROS sourcing needed), mirroring the
idiom in ../lidar_lidar/run_odometry.py. Adds:
  - sensor_msgs/Image and CompressedImage -> BGR ndarray
  - sensor_msgs/CameraInfo -> (K, D, size, model)
  - sensor_msgs/PointCloud2 -> (N,3) float32
  - livox_interfaces/CustomMsg -> (N,3) float32 (type registered locally; the
    standard typestore does not know this rig-specific message)
  - update_calibrated_extrinsics(): merge camera results into the repo's
    extrinsics_calibrated.yaml WITHOUT clobbering entries calibrated by other
    stages (the lidar_lidar writer rewrites every entry; we must not).

Timestamps: this stage windows everything on BAG-RECEIVE time. The rig's sensors
do not share a clock (the Livox stamps headers with its hardware uptime — see
../lidar_lidar/JETSON_FASTLIO_HANDOFF.md gotcha 4), but the recorder writes one
clock for all topics, and board holds are seconds long, so receive time is both
safe and simple here. Header stamps are carried along for reference only.
"""
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

LIVOX_CUSTOMPOINT_MSG = """
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""

LIVOX_CUSTOMMSG_MSG = """
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_interfaces/CustomPoint[] points
"""


def open_bag(path, ros_distro="humble"):
    """AnyReader for a ROS 2 bag with the Livox custom types registered.

    Use as a context manager:  with open_bag(p) as reader: ...
    """
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    ts = get_typestore(getattr(Stores, f"ROS2_{ros_distro.upper()}"))
    types = {}
    types.update(get_types_from_msg(
        LIVOX_CUSTOMPOINT_MSG, "livox_interfaces/msg/CustomPoint"))
    types.update(get_types_from_msg(
        LIVOX_CUSTOMMSG_MSG, "livox_interfaces/msg/CustomMsg"))
    ts.register(types)
    return AnyReader([Path(path)], default_typestore=ts)


def _conns(reader, topic):
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        avail = sorted({c.topic for c in reader.connections})
        raise SystemExit(f"Topic '{topic}' not in bag. Available: {avail}")
    return conns


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def read_camera_info(reader, topic):
    """First CameraInfo on `topic` -> dict(K, D, width, height, model)."""
    for conn, _, raw in reader.messages(connections=_conns(reader, topic)):
        m = reader.deserialize(raw, conn.msgtype)
        return {
            "K": np.array(m.k, float).reshape(3, 3),
            "D": np.array(m.d, float),
            "width": int(m.width), "height": int(m.height),
            "model": str(m.distortion_model),
        }
    raise SystemExit(f"No CameraInfo message on '{topic}'.")


def iter_images(reader, topic, stride=1):
    """Yield (t_recv, t_header, bgr_image) from Image or CompressedImage."""
    import cv2
    i = -1
    for conn, t_recv_ns, raw in reader.messages(connections=_conns(reader, topic)):
        i += 1
        if i % stride:
            continue
        m = reader.deserialize(raw, conn.msgtype)
        t_recv = t_recv_ns * 1e-9
        t_hdr = stamp_to_sec(m.header.stamp)
        if conn.msgtype.endswith("CompressedImage"):
            img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
        else:
            enc = m.encoding.lower()
            buf = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)
            if enc in ("bgr8", "rgb8"):
                img = buf[:, : m.width * 3].reshape(m.height, m.width, 3)
                if enc == "rgb8":
                    img = img[:, :, ::-1]
                img = img.copy()
            elif enc in ("mono8", "8uc1"):
                img = buf[:, : m.width].copy()
            else:
                raise SystemExit(f"Unsupported image encoding '{m.encoding}' on "
                                 f"'{topic}' — extend io_cam.iter_images().")
        yield t_recv, t_hdr, img


def pointcloud2_to_xyz(msg):
    """sensor_msgs/PointCloud2 -> (N,3) float32, NaN/zero points dropped."""
    fields = {f.name: f for f in msg.fields}
    for axis in ("x", "y", "z"):
        if axis not in fields or fields[axis].datatype != 7:  # 7 = FLOAT32
            raise SystemExit("PointCloud2 without float32 x/y/z is not supported.")
    data = np.frombuffer(msg.data, np.uint8)
    n = len(data) // msg.point_step
    data = data[: n * msg.point_step].reshape(n, msg.point_step)
    xyz = np.empty((n, 3), np.float32)
    for i, axis in enumerate(("x", "y", "z")):
        off = fields[axis].offset
        xyz[:, i] = data[:, off:off + 4].copy().view(np.float32).ravel()
    good = np.isfinite(xyz).all(axis=1) & (np.abs(xyz) > 1e-6).any(axis=1)
    return xyz[good]


def livox_to_xyz(msg):
    """livox_interfaces/CustomMsg -> (N,3) float32."""
    pts = msg.points
    xyz = np.column_stack([
        np.array([p.x for p in pts], np.float32),
        np.array([p.y for p in pts], np.float32),
        np.array([p.z for p in pts], np.float32),
    ])
    good = np.isfinite(xyz).all(axis=1) & (np.abs(xyz) > 1e-6).any(axis=1)
    return xyz[good]


def collect_cloud(reader, topic, t_center, half_window):
    """Accumulate LiDAR points received within [t_center +- half_window] (s).

    Works for both PointCloud2 (Hesai) and livox_interfaces/CustomMsg (Livox —
    its non-repetitive scan pattern densifies nicely when integrated over ~1 s
    of a static hold). Windowing is on bag-receive time; see module docstring.
    """
    conns = _conns(reader, topic)
    lo, hi = t_center - half_window, t_center + half_window
    chunks = []
    for conn, t_recv_ns, raw in reader.messages(
            connections=conns, start=int(lo * 1e9), stop=int(hi * 1e9)):
        m = reader.deserialize(raw, conn.msgtype)
        if conn.msgtype.endswith("CustomMsg"):
            chunks.append(livox_to_xyz(m))
        else:
            chunks.append(pointcloud2_to_xyz(m))
    if not chunks:
        return np.empty((0, 3), np.float32)
    return np.concatenate(chunks, axis=0)


# ---------------------------------------------------------------------------
# Extrinsics YAML (repo format, see ../lidar_lidar/io_utils.py)
# ---------------------------------------------------------------------------

def T_from_xyz_quat(xyz, q_xyzw):
    T = np.eye(4)
    T[:3, :3] = R.from_quat(np.asarray(q_xyzw, float)).as_matrix()
    T[:3, 3] = np.asarray(xyz, float)
    return T


def load_extrinsics(path):
    """extrinsics YAML -> (doc, {child: 4x4 T_base_child})."""
    doc = yaml.safe_load(open(path))
    frames = {tf["child"]: T_from_xyz_quat(tf["xyz"], tf["q_xyzw"])
              for tf in doc["transforms"]}
    return doc, frames


def update_calibrated_extrinsics(out_path, initial_path, updates, provenance):
    """Merge `updates` ({child: 4x4 T_base_child}) into the calibrated YAML.

    Unlike lidar_lidar's write_calibrated_extrinsics (which rewrites every entry
    from the CAD file), this PRESERVES entries already calibrated by other
    stages: if `out_path` exists it is the merge base, else `initial_path`.
    Per-stage provenance is kept under a `provenance` list.
    """
    base_path = Path(out_path) if Path(out_path).exists() else Path(initial_path)
    doc = yaml.safe_load(open(base_path))

    prov = doc.get("provenance")
    prov = prov if isinstance(prov, list) else ([prov] if prov else [])
    prov.append(provenance)

    out = {
        "reference_frame": doc.get("reference_frame", "base_link"),
        "units": {"translation": "meters", "rotation": "quaternion_xyzw"},
        "provenance": prov,
        "transforms": [],
    }
    for tf in doc["transforms"]:
        child = tf["child"]
        entry = {"source_frame": tf.get("source_frame"),
                 "parent": tf["parent"], "child": child}
        if child in updates:
            T = np.asarray(updates[child], float)
            entry["xyz"] = [float(x) for x in T[:3, 3]]
            entry["q_xyzw"] = [float(x)
                               for x in R.from_matrix(T[:3, :3]).as_quat()]
            entry["calibrated"] = True
        else:
            entry["xyz"] = [float(x) for x in tf["xyz"]]
            entry["q_xyzw"] = [float(x) for x in tf["q_xyzw"]]
            entry["calibrated"] = bool(tf.get("calibrated", False))
            if not entry["calibrated"]:
                entry["note"] = "CAD initial estimate (not refined yet)"
        out["transforms"].append(entry)

    with open(out_path, "w") as f:
        f.write("# Calibrated sensor extrinsics. Updated by "
                "extrinsic_calibration/camera_lidar.\n")
        f.write("# Entries with calibrated: true were refined; others are CAD "
                "initial values.\n")
        f.write("# Do NOT overwrite extrinsics_initial.yaml or "
                "coordinate_systems.yaml.\n\n")
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None)
