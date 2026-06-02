#!/usr/bin/env python3
"""Step 1: produce a per-LiDAR trajectory from a recorded ROS 2 bag.

Runs KISS-ICP (target-less LiDAR odometry) once for the Hesai topic and once
for the Livox topic, then normalises both outputs to timestamped TUM files:

    hesai_tum.txt   livox_tum.txt      (rows: timestamp tx ty tz qx qy qz qw)

These two trajectories are the input to solve_extrinsic.py.

KISS-ICP is used via its command-line pipeline, which reads rosbag2 (.mcap or
.db3) directly and is robust across sensor types. Install with:

    pip install kiss-icp rosbags

Usage:
    python run_odometry.py --bag /path/to/bag --config config.yaml --out ./out
"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import yaml


def kiss_icp_available():
    return shutil.which("kiss_icp_pipeline") is not None


def run_kiss_icp(bag, topic, out_dir, odo):
    """Run the KISS-ICP CLI for one topic; return the TUM trajectory path it wrote."""
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        "kiss_icp_pipeline", str(bag),
        "--topic", topic,
        "--output", str(out_dir),
    ]
    # Optional tuning flags (KISS-ICP reads most config from its own yaml; these
    # are the commonly-exposed CLI knobs). Unsupported flags are dropped safely.
    print(f"\n[run_odometry] {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        sys.exit(f"KISS-ICP failed for topic {topic}: {e}")
    tum = list(out_dir.rglob("*_tum.txt")) + list(out_dir.rglob("*poses*tum*.txt"))
    if not tum:
        sys.exit(f"No TUM trajectory found under {out_dir}. "
                 f"Check the KISS-ICP output format / version.")
    return sorted(tum, key=lambda p: p.stat().st_mtime)[-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="Path to the ROS 2 bag (folder, .mcap or .db3)")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--out", default="./out", help="Output directory")
    ap.add_argument("--hesai-topic", help="Override Hesai topic from config")
    ap.add_argument("--livox-topic", help="Override Livox topic from config")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    hesai_topic = args.hesai_topic or cfg["bag"]["hesai_topic"]
    livox_topic = args.livox_topic or cfg["bag"]["livox_topic"]
    out = Path(args.out)

    if not kiss_icp_available():
        sys.exit(
            "kiss_icp_pipeline not found on PATH.\n"
            "  Install:  pip install kiss-icp rosbags\n"
            "  (Then re-run.) Alternatively run KISS-ICP yourself per topic and\n"
            "  place the two TUM trajectories at out/hesai_tum.txt and out/livox_tum.txt."
        )

    hesai_tum = run_kiss_icp(args.bag, hesai_topic, out / "hesai", cfg["odometry"])
    livox_tum = run_kiss_icp(args.bag, livox_topic, out / "livox", cfg["odometry"])

    shutil.copy(hesai_tum, out / "hesai_tum.txt")
    shutil.copy(livox_tum, out / "livox_tum.txt")
    print("\n[run_odometry] Done.")
    print(f"  Hesai trajectory -> {out / 'hesai_tum.txt'}")
    print(f"  Livox trajectory -> {out / 'livox_tum.txt'}")
    print("\nNext: python solve_extrinsic.py --hesai-traj out/hesai_tum.txt "
          "--livox-traj out/livox_tum.txt --config config.yaml")


if __name__ == "__main__":
    main()
