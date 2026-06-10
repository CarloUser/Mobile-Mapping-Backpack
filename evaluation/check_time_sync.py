#!/usr/bin/env python3
"""Per-topic time-sync audit of a recorded bag.

For every topic in the bag (or a chosen subset) this reports, over all
messages with a std_msgs/Header:

  rate        observed message rate
  recv-hdr    bag-receive time minus header stamp: median, IQR jitter
  drift       linear drift of (recv - hdr) over the recording (clock skew)

and a PASS/FAIL verdict per topic against --max-offset / --max-jitter.

How to read it:
  - recv-hdr median near 0 (few ms):       sensor stamps on the recorder clock,
                                           or hardware sync (PTP) is working.
  - recv-hdr large but STABLE:             sensor runs its own clock epoch
                                           (e.g. Livox uptime). Calibration/
                                           mapping must align it in post — the
                                           printed median IS that correction.
  - recv-hdr drifting (ppm-level slope):   free-running sensor oscillator; fix
                                           with PTP/GPS sync, do not just
                                           subtract a constant for long bags.
  - header==0 messages:                    driver not stamping; fix the driver.

Works without ROS sourcing (rosbags), including livox_interfaces/CustomMsg.

Usage:
    python3 check_time_sync.py --bag <bag_dir> [--topics /a /b] \
        [--max-offset 0.05] [--max-jitter 0.01]
"""
import argparse
from pathlib import Path

import numpy as np

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
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(getattr(Stores, f"ROS2_{ros_distro.upper()}"))
    types = {}
    types.update(get_types_from_msg(LIVOX_CUSTOMPOINT_MSG,
                                    "livox_interfaces/msg/CustomPoint"))
    types.update(get_types_from_msg(LIVOX_CUSTOMMSG_MSG,
                                    "livox_interfaces/msg/CustomMsg"))
    ts.register(types)
    return AnyReader([Path(path)], default_typestore=ts)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topics", nargs="*", default=None,
                    help="subset of topics (default: every topic with a header)")
    ap.add_argument("--max-offset", type=float, default=0.05,
                    help="PASS threshold on |median(recv-hdr)| in seconds "
                         "(default 0.05)")
    ap.add_argument("--max-jitter", type=float, default=0.01,
                    help="PASS threshold on recv-hdr IQR in seconds "
                         "(default 0.01)")
    args = ap.parse_args()

    rows, worst = [], 0
    with open_bag(args.bag) as reader:
        conns = [c for c in reader.connections
                 if args.topics is None or c.topic in args.topics]
        if not conns:
            raise SystemExit("No matching topics in bag.")
        by_topic = {}
        for c in conns:
            by_topic.setdefault(c.topic, []).append(c)

        for topic in sorted(by_topic):
            recv, hdr, zeros = [], [], 0
            for conn, t_ns, raw in reader.messages(
                    connections=by_topic[topic]):
                try:
                    m = reader.deserialize(raw, conn.msgtype)
                except Exception:
                    rows.append((topic, conn.msgtype.split("/")[-1], 0,
                                 None, None, None, "no deserializer"))
                    break
                st = getattr(m, "header", None)
                if st is None:           # e.g. /tf, TFMessage has no top header
                    rows.append((topic, conn.msgtype.split("/")[-1], 0,
                                 None, None, None, "no header"))
                    break
                t_h = float(st.stamp.sec) + float(st.stamp.nanosec) * 1e-9
                if t_h == 0.0:
                    zeros += 1
                    continue
                recv.append(t_ns * 1e-9)
                hdr.append(t_h)
            else:
                pass
            if not recv:
                if zeros:
                    rows.append((topic, conn.msgtype.split("/")[-1], zeros,
                                 None, None, None, "header.stamp == 0"))
                continue
            recv = np.asarray(recv); hdr = np.asarray(hdr)
            off = recv - hdr
            dur = recv[-1] - recv[0]
            rate = (len(recv) - 1) / dur if dur > 0 else float("nan")
            med = float(np.median(off))
            iqr = float(np.percentile(off, 75) - np.percentile(off, 25))
            drift = (float(np.polyfit(recv - recv[0], off, 1)[0]) * 1e6
                     if dur > 10 else None)   # ppm, needs some duration
            note = ""
            if zeros:
                note = f"{zeros} zero-stamp msgs"
            rows.append((topic, conn.msgtype.split("/")[-1], len(recv),
                         rate, (med, iqr), drift, note))

    print(f"{'topic':38s} {'type':18s} {'msgs':>7s} {'rate':>8s} "
          f"{'recv-hdr med':>13s} {'jitter IQR':>11s} {'drift':>10s}  verdict")
    print("-" * 120)
    n_fail = 0
    for topic, typ, n, rate, oj, drift, note in rows:
        if oj is None:
            print(f"{topic:38s} {typ:18s} {n:7d} {'-':>8s} {'-':>13s} "
                  f"{'-':>11s} {'-':>10s}  SKIP ({note})")
            continue
        med, iqr = oj
        ok = abs(med) <= args.max_offset and iqr <= args.max_jitter
        n_fail += (not ok)
        drift_s = f"{drift:+.0f} ppm" if drift is not None else "-"
        flag = "PASS" if ok else "FAIL"
        extra = f" ({note})" if note else ""
        print(f"{topic:38s} {typ:18s} {n:7d} {rate:7.1f}Hz "
              f"{med*1e3:+11.1f}ms {iqr*1e3:9.2f}ms {drift_s:>10s}  "
              f"{flag}{extra}")
    print("-" * 120)
    if n_fail:
        print(f"{n_fail} topic(s) FAIL: either sync those sensors (PTP/chrony,"
              f" see docs/setup/03_time_sync.md) or carry the printed median"
              f" offset as a post-hoc correction (constant only if drift ~0).")
    else:
        print("All checked topics within thresholds.")
    raise SystemExit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
