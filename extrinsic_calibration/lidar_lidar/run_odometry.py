#!/usr/bin/env python3
"""Step 1: per-LiDAR trajectory from a ROS 2 bag via KISS-ICP (self-contained).

We read the bag ourselves with an explicit ROS 2 Humble typestore and drive
KISS-ICP through its Python API, writing out/hesai_tum.txt and out/livox_tum.txt
directly. This deliberately does NOT use the `kiss_icp_pipeline` CLI, which
- needs the installed kiss-icp package to be patched for the rosbags typestore
  (newer rosbags requires an explicit default_typestore for ROS 2 Humble bags),
- has no output-path flag and writes to a timestamped results/ dir that is
  awkward to locate, and writes the same filename for both topics of one bag.
Doing the I/O here avoids all three problems.

Usage:
    python run_odometry.py --bag /path/to/bag --config config.yaml --out ./out
"""
import argparse
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


def resolve_config_path(config_path, value):
    p = Path(value)
    return p if p.is_absolute() else (Path(config_path).resolve().parent / p).resolve()


def get_typestore_for(distro):
    """Map a ROS 2 distro name (from config) to a rosbags typestore."""
    from rosbags.typesys import Stores, get_typestore
    name = f"ROS2_{str(distro).strip().upper()}"
    try:
        store = getattr(Stores, name)
    except AttributeError:
        supported = sorted(s.name[5:].lower() for s in Stores if s.name.startswith("ROS2_"))
        raise SystemExit(f"Unknown ros_distro '{distro}'. Supported: {supported}")
    return get_typestore(store)


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def run_one(reader, topic, cfg_odo):
    """Run KISS-ICP over one topic; return (timestamps, list of 4x4 poses)."""
    from kiss_icp.kiss_icp import KissICP
    from kiss_icp.config import load_config
    from kiss_icp.tools.point_cloud2 import read_point_cloud

    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        avail = sorted({c.topic for c in reader.connections})
        raise SystemExit(f"Topic '{topic}' not found in bag. Available topics: {avail}")

    config = load_config(None)
    config.data.max_range = float(cfg_odo["max_range"])
    config.data.min_range = float(cfg_odo["min_range"])
    config.data.deskew = bool(cfg_odo.get("deskew", True))
    if cfg_odo.get("voxel_size"):
        config.mapping.voxel_size = float(cfg_odo["voxel_size"])

    odom = KissICP(config)
    times, poses = [], []
    for conn, _, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, conn.msgtype)
        pts, pts_t = read_point_cloud(msg)
        pts_t = pts_t if (pts_t is not None and len(pts_t) > 0) else np.array([])
        odom.register_frame(pts, pts_t)
        times.append(stamp_to_sec(msg.header.stamp))
        poses.append(odom.last_pose.copy())
    if not poses:
        raise SystemExit(f"No messages on topic '{topic}'.")
    print(f"  [{topic}] {len(poses)} scans processed")
    return np.asarray(times), poses


def write_tum(path, times, poses):
    with open(path, "w") as f:
        for t, T in zip(times, poses):
            q = R.from_matrix(T[:3, :3]).as_quat()       # x y z w
            p = T[:3, 3]
            f.write(f"{t:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="ROS 2 bag (folder, or .mcap/.db3 file)")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--out", default="./out")
    ap.add_argument("--hesai-topic", help="Override Hesai topic from config")
    ap.add_argument("--livox-topic", help="Override Livox topic from config")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    hesai_topic = args.hesai_topic or cfg["bag"]["hesai_topic"]
    livox_topic = args.livox_topic or cfg["bag"]["livox_topic"]
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

    from rosbags.highlevel import AnyReader
    typestore = get_typestore_for(cfg["bag"].get("ros_distro", "humble"))

    bag = Path(args.bag)
    print(f"[run_odometry] reading {bag}")
    with AnyReader([bag], default_typestore=typestore) as reader:
        th, WA = run_one(reader, hesai_topic, cfg["odometry"])
        tl, WB = run_one(reader, livox_topic, cfg["odometry"])

    write_tum(out / "hesai_tum.txt", th, WA)
    write_tum(out / "livox_tum.txt", tl, WB)
    print(f"[run_odometry] wrote {out / 'hesai_tum.txt'} and {out / 'livox_tum.txt'}")
    print("\nNext: python solve_extrinsic.py --hesai-traj out/hesai_tum.txt "
          "--livox-traj out/livox_tum.txt --config config.yaml")


if __name__ == "__main__":
    main()
