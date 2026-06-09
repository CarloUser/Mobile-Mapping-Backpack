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


def run_one(reader, topic, cfg_odo, timestamp_source="header"):
    """Run KISS-ICP over one topic; return (timestamps, list of 4x4 poses).

    timestamp_source: 'header' uses msg.header.stamp (acquisition time, best when
    both drivers share a synced clock); 'bag_receive' uses the recorder write clock
    (one clock for both topics), a robust fallback for unsynced sensor clocks.
    """
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
    header_t, recv_t, poses, n_zero = [], [], [], 0
    for conn, recv_ns, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, conn.msgtype)
        pts, pts_t = read_point_cloud(msg)
        pts_t = pts_t if (pts_t is not None and len(pts_t) > 0) else np.array([])
        odom.register_frame(pts, pts_t)
        ht = stamp_to_sec(msg.header.stamp)
        if ht == 0.0:
            n_zero += 1
        header_t.append(ht)
        recv_t.append(recv_ns * 1e-9)
        poses.append(odom.last_pose.copy())
    if not poses:
        raise SystemExit(f"No messages on topic '{topic}'.")
    header_t = np.asarray(header_t); recv_t = np.asarray(recv_t)
    if timestamp_source == "bag_receive":
        times = recv_t
    elif timestamp_source == "header_aligned":
        # Clean per-sensor header spacing, shifted onto the shared recorder epoch
        # by the robust median offset. Removes the cross-sensor header-epoch
        # difference while keeping header's low jitter. Best when each driver's
        # header is clean but the two sensors do not share a clock.
        times = header_t + np.median(recv_t - header_t)
    elif timestamp_source == "header":
        times = header_t
        if n_zero:
            print(f"  [{topic}] WARNING: {n_zero} frames have header.stamp==0; "
                  f"use timestamp_source: bag_receive or header_aligned")
    else:
        raise SystemExit(f"Unknown timestamp_source '{timestamp_source}' "
                         f"(use header, header_aligned, or bag_receive)")
    print(f"  [{topic}] {len(poses)} scans processed (timestamps: {timestamp_source})")
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
    ap.add_argument("--only", choices=["both", "hesai", "livox"], default="both",
                    help="Run KISS-ICP for only one sensor. Use 'hesai' when the "
                         "Livox trajectory comes from FAST-LIO instead.")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    hesai_topic = args.hesai_topic or cfg["bag"]["hesai_topic"]
    livox_topic = args.livox_topic or cfg["bag"]["livox_topic"]
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

    from rosbags.highlevel import AnyReader
    typestore = get_typestore_for(cfg["bag"].get("ros_distro", "humble"))

    tsrc = cfg["bag"].get("timestamp_source", "header")
    bag = Path(args.bag)
    print(f"[run_odometry] reading {bag}")
    written = []
    with AnyReader([bag], default_typestore=typestore) as reader:
        if args.only in ("both", "hesai"):
            th, WA = run_one(reader, hesai_topic, cfg["odometry"], tsrc)
            write_tum(out / "hesai_tum.txt", th, WA); written.append("hesai_tum.txt")
        if args.only in ("both", "livox"):
            tl, WB = run_one(reader, livox_topic, cfg["odometry"], tsrc)
            write_tum(out / "livox_tum.txt", tl, WB); written.append("livox_tum.txt")
    print(f"[run_odometry] wrote {', '.join(written)} to {out}")
    if args.only == "hesai":
        print("\nLivox trajectory expected from FAST-LIO: "
              "python fastlio_odom_to_tum.py --bag <odom_bag> --out out/livox_tum.txt")
    print("\nNext: python solve_extrinsic.py --hesai-traj out/hesai_tum.txt "
          "--livox-traj out/livox_tum.txt --config config.yaml")


if __name__ == "__main__":
    main()
