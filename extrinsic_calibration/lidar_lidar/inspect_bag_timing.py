#!/usr/bin/env python3
"""Diagnose timing of the two LiDAR topics in a ROS 2 bag BEFORE calibrating.

The bag already contains two real per-message timestamps for every scan:
  - header.stamp : set by the driver (ideally sensor acquisition time)
  - bag receive  : set by the recorder when the message is written (ONE clock for
                   both topics)
You do NOT need to reconstruct timestamps from a start time and a nominal rate;
that assumes a perfectly constant rate and no dropped frames and is less accurate
than the real per-message stamps below. This tool characterises both sources so
you can pick the trustworthy one (set it as bag.timestamp_source in config.yaml).

Run:  python inspect_bag_timing.py --bag <bag> --config config.yaml
"""
import argparse
from pathlib import Path
import numpy as np
import yaml


def gather(reader, topic):
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        return None
    recv, hdr = [], []
    for conn, ts, raw in reader.messages(connections=conns):
        recv.append(ts * 1e-9)
        msg = reader.deserialize(raw, conn.msgtype)
        s = msg.header.stamp
        hdr.append(float(s.sec) + float(s.nanosec) * 1e-9)
    return np.array(recv), np.array(hdr)


def describe(name, recv, hdr):
    print(f"\n[{name}]  messages={len(recv)}")
    for label, t in (("header.stamp", hdr), ("bag receive", recv)):
        if np.allclose(t, 0):
            print(f"  {label:12s}: ALL ZERO (driver not stamping) -> unusable")
            continue
        dt = np.diff(t)
        rate = 1.0 / np.mean(dt) if np.mean(dt) > 0 else float("nan")
        mono = "monotonic" if np.all(dt > 0) else f"NON-monotonic ({int(np.sum(dt<=0))} backsteps)"
        print(f"  {label:12s}: {rate:5.2f} Hz | interval mean {np.mean(dt)*1e3:6.1f} ms "
              f"std {np.std(dt)*1e3:5.1f} ms max {np.max(dt)*1e3:6.1f} ms | {mono}")
    if not np.allclose(hdr, 0):
        off = hdr - recv
        print(f"  header-minus-receive: mean {np.mean(off)*1e3:.1f} ms "
              f"std {np.std(off)*1e3:.1f} ms (large or non-constant => clocks not shared)")
    return recv, hdr


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.config))
    ht, lt = cfg["bag"]["hesai_topic"], cfg["bag"]["livox_topic"]

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(getattr(Stores, f"ROS2_{cfg['bag'].get('ros_distro', 'humble').upper()}"))
    with AnyReader([Path(args.bag)], default_typestore=ts) as reader:
        h = gather(reader, ht); l = gather(reader, lt)
        if h is None or l is None:
            avail = sorted({c.topic for c in reader.connections})
            raise SystemExit(f"Topic missing. Available: {avail}")
        hr, hh = describe(f"HESAI {ht}", *h)
        lr, lh = describe(f"LIVOX {lt}", *l)

    print("\n[cross-topic, shared receive clock]")
    print(f"  start skew (livox - hesai): {(lr[0]-hr[0])*1e3:.1f} ms")
    print(f"  end skew:                   {(lr[-1]-hr[-1])*1e3:.1f} ms")

    # Show what timestamp_source: header_aligned would actually produce. This is
    # NOT applied to the raw stats above; run_odometry.py performs this shift.
    if not (np.allclose(hh, 0) or np.allclose(lh, 0)):
        off_h = np.median(hr - hh); off_l = np.median(lr - lh)
        ah, al = hh + off_h, lh + off_l
        print("\n[after header_aligned: each header shifted onto the shared recorder epoch]")
        print(f"  raw header epoch gap (livox - hesai): {(lh[0]-hh[0]):.1f} s  "
              f"<- the two sensor clocks differ; raw 'header' would be unusable")
        print(f"  per-topic shift applied:  hesai {off_h:+.3f} s   livox {off_l:+.3f} s")
        print(f"  aligned start skew (livox - hesai): {(al[0]-ah[0])*1e3:.1f} ms")
        print(f"  aligned end skew:                   {(al[-1]-ah[-1])*1e3:.1f} ms")
        print("  Both topics now share one epoch with clean header spacing. The remaining\n"
              "  skew is just the recording start offset (Livox began first); the solver's\n"
              "  overlap masking handles it. This is the timeline header_aligned uses.")
    print(
        "\nGuidance:\n"
        "  - If 'bag receive' is clean & monotonic on both topics, it is a safe common\n"
        "    timeline: set bag.timestamp_source: bag_receive.\n"
        "  - If header.stamp is sane and header-minus-receive is small & CONSTANT on\n"
        "    both, the drivers share a clock: header is best (keep timestamp_source: header).\n"
        "  - A large/varying header-minus-receive, or non-monotonic header stamps, means\n"
        "    the sensor clocks are not synced -> use bag_receive (or set up PTP/PPS).")


if __name__ == "__main__":
    main()
