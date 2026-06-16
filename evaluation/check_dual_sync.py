#!/usr/bin/env python3
"""Cross-Jetson sync check for the split (two-bag) recording.

Each Jetson records its own bag on its own (PTP-disciplined) clock. This tool
checks that the two bags are FUSEABLE:

  1. Wall-clock overlap   — do the two recordings cover a common time window?
                            (no overlap => nothing to fuse.)
  2. Shared epoch         — are receive-time stamps on the same absolute scale
                            (both UNIX epoch), not one on a hardware uptime?
  3. PTP servo health     — optional: parse ptp4l/phc2sys -m logs captured
                            during the recording (--ptp-log-a/-b) and report the
                            max |offset|. THIS is the real measure of how well
                            the two host clocks agreed; the bags alone cannot
                            show sub-second cross-clock error without a shared
                            signal.

Per-topic rate/jitter/drift within each bag: use check_time_sync.py.

Usage:
    python3 check_dual_sync.py --bag-a <lidar_bag> --bag-b <camera_bag> \
        [--ptp-log-a ptp_lidar.log] [--ptp-log-b ptp_cam.log]
"""
import argparse
import re
from pathlib import Path

import numpy as np

from check_time_sync import open_bag   # reuse the typestore-aware reader


def bag_span(path):
    """(t0, t1, n_msgs) of bag-receive times, in UNIX seconds."""
    t0, t1, n = None, None, 0
    with open_bag(path) as reader:
        for _conn, t_ns, _raw in reader.messages():
            t = t_ns * 1e-9
            t0 = t if t0 is None else min(t0, t)
            t1 = t if t1 is None else max(t1, t)
            n += 1
    if n == 0:
        raise SystemExit(f"{path}: empty bag")
    return t0, t1, n


def max_ptp_offset(logpath):
    """Max |offset| (seconds) from ptp4l/phc2sys '-m' stdout captured to a file.

    Lines look like:  'ptp4l[123.4]: rms 12 max 34 ... offset  -8 s2 freq ...'
    or phc2sys: 'phc2sys[...]: CLOCK_REALTIME phc offset  -120 s2 ...' (ns).
    """
    vals = []
    pat = re.compile(r"offset\s+(-?\d+)")
    for line in Path(logpath).read_text(errors="ignore").splitlines():
        m = pat.search(line)
        if m:
            vals.append(abs(int(m.group(1))))   # nanoseconds
    if not vals:
        return None
    return max(vals) * 1e-9


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag-a", required=True, help="e.g. the LiDAR-Jetson bag")
    ap.add_argument("--bag-b", required=True, help="e.g. the camera-Jetson bag")
    ap.add_argument("--ptp-log-a", default=None)
    ap.add_argument("--ptp-log-b", default=None)
    ap.add_argument("--max-ptp-offset", type=float, default=0.001,
                    help="PASS threshold on max |PTP offset| in seconds "
                         "(default 1 ms)")
    args = ap.parse_args()

    a0, a1, an = bag_span(args.bag_a)
    b0, b1, bn = bag_span(args.bag_b)
    print(f"bag A {Path(args.bag_a).name}: {an:7d} msgs  "
          f"[{a0:.3f} .. {a1:.3f}]  dur {a1-a0:6.1f}s")
    print(f"bag B {Path(args.bag_b).name}: {bn:7d} msgs  "
          f"[{b0:.3f} .. {b1:.3f}]  dur {b1-b0:6.1f}s")

    # 2. shared epoch sanity (both should be modern UNIX time, ~1.7e9 in 2024+)
    fail = 0
    for name, t in (("A", a0), ("B", b0)):
        if t < 1e9:
            print(f"  FAIL: bag {name} receive-time {t:.0f} is not UNIX epoch — "
                  f"recorder clock not set (PTP/chrony not disciplining it).")
            fail += 1

    # 1. overlap
    lo, hi = max(a0, b0), min(a1, b1)
    overlap = hi - lo
    if overlap <= 0:
        print(f"  FAIL: NO time overlap (gap {-overlap:.1f}s) — the two bags do "
              f"not cover a common window; nothing to fuse.")
        fail += 1
    else:
        span = max(a1, b1) - min(a0, b0)
        print(f"  overlap: {overlap:.1f}s of {span:.1f}s combined "
              f"({100*overlap/span:.0f}%); start skew {abs(a0-b0):.1f}s")

    # 3. PTP servo health (the real cross-clock measure)
    for name, log in (("A", args.ptp_log_a), ("B", args.ptp_log_b)):
        if not log:
            continue
        off = max_ptp_offset(log)
        if off is None:
            print(f"  PTP {name}: no offset lines parsed from {log}")
            continue
        ok = off <= args.max_ptp_offset
        fail += (not ok)
        print(f"  PTP {name}: max |offset| {off*1e6:8.1f} us  "
              f"{'PASS' if ok else 'FAIL'} (<= {args.max_ptp_offset*1e3:.1f} ms)")
    if not (args.ptp_log_a or args.ptp_log_b):
        print("  NOTE: no --ptp-log given. Overlap+epoch look right, but the "
              "actual cross-Jetson clock error is only provable from the PTP "
              "servo logs. Capture them during recording:\n"
              "    journalctl -fu mmb-ptp4l > ptp_<role>.log   (on each Jetson)")

    print("-" * 70)
    if fail:
        print(f"{fail} check(s) FAILED — see docs/dual_jetson_recording.md.")
    else:
        print("Bags are fuseable (overlap + shared epoch"
              + (" + PTP within bound)." if (args.ptp_log_a or args.ptp_log_b)
                 else "; verify PTP logs for the final word)."))
    raise SystemExit(1 if fail else 0)


if __name__ == "__main__":
    main()
