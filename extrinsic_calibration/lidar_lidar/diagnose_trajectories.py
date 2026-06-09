#!/usr/bin/env python3
"""Diagnose WHY a LiDAR<->LiDAR hand-eye solve has large residuals.

Both LiDARs are rigidly attached, so for every motion their rotation MAGNITUDE
must match (rotation angle is invariant to the extrinsic). This script compares
the two odometry trajectories directly to tell whether the problem is:
  - a bad odometry on one sensor (the trajectories disagree on the motion),
  - a residual time offset (they agree but are shifted), or
  - degenerate motion / a solver issue (they agree and are aligned).

Usage:
  python diagnose_trajectories.py --hesai-traj out/hesai_tum.txt \
      --livox-traj out/livox_tum.txt --config config.yaml
"""
import argparse
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
import handeye as he
import io_utils as io


def rel(WA, WB, stride):
    angA, angB, trA, trB = [], [], [], []
    for i in range(0, len(WA) - stride, stride):
        Ai = he.inv(WA[i]) @ WA[i + stride]
        Bi = he.inv(WB[i]) @ WB[i + stride]
        angA.append(np.degrees(np.linalg.norm(R.from_matrix(Ai[:3, :3]).as_rotvec())))
        angB.append(np.degrees(np.linalg.norm(R.from_matrix(Bi[:3, :3]).as_rotvec())))
        trA.append(np.linalg.norm(Ai[:3, 3])); trB.append(np.linalg.norm(Bi[:3, 3]))
    return map(np.array, (angA, angB, trA, trB))


def speed_series(times, poses, n=2000):
    tc, w = [], []
    for i in range(len(poses) - 1):
        dT = he.inv(poses[i]) @ poses[i + 1]
        dt = times[i + 1] - times[i]
        if dt > 0:
            w.append(np.linalg.norm(R.from_matrix(dT[:3, :3]).as_rotvec()) / dt)
            tc.append(0.5 * (times[i] + times[i + 1]))
    grid = np.linspace(times[0], times[-1], n)
    return grid, np.interp(grid, tc, w)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hesai-traj", required=True)
    ap.add_argument("--livox-traj", required=True)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--plot", default="out/diagnose.png")
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.config)); stride = cfg["pairing"]["stride"]

    th, WA = io.load_tum(args.hesai_traj)
    tl, WB = io.load_tum(args.livox_traj)
    lo, hi = max(th[0], tl[0]), min(th[-1], tl[-1])
    m = (th >= lo) & (th <= hi)
    th = th[m]; WA = [WA[i] for i in np.where(m)[0]]
    WB = he.interp_pose(tl, WB, th)
    print(f"aligned {len(WA)} frames over {th[-1]-th[0]:.1f}s, stride={stride}")

    angA, angB, trA, trB = rel(WA, WB, stride)
    print("\nper-sensor totals (gross sanity; both see the same rigid motion):")
    print(f"  Hesai: total rotation {angA.sum():.0f} deg, total path {trA.sum():.1f} m")
    print(f"  Livox: total rotation {angB.sum():.0f} deg, total path {trB.sum():.1f} m")

    mism = np.abs(angA - angB)
    corr = np.corrcoef(angA, angB)[0, 1]
    print("\nrigid-body rotation check (rotation magnitudes MUST match if both "
          "odometries are good):")
    print(f"  |angle_hesai - angle_livox|: median {np.median(mism):.2f} deg, "
          f"90th {np.percentile(mism,90):.2f} deg")
    print(f"  angular-speed correlation: {corr:.3f}   (want > ~0.95)")

    g, wa = speed_series(th, WA); _, wb = speed_series(th, WB)
    wa -= wa.mean(); wb -= wb.mean()
    lags = np.arange(-40, 41)
    cc = [np.dot(wa, np.roll(wb, k)) for k in lags]
    lag = lags[int(np.argmax(cc))] * (g[1] - g[0])
    print(f"  estimated time offset (livox vs hesai): {lag*1e3:.0f} ms")

    print("\nLIKELY CAUSE:")
    if corr < 0.8 or np.median(mism) > 2.0:
        print("  The two odometries DISAGREE on the motion -> one LiDAR's odometry is")
        print("  unreliable (compare the trajectory plots to see which). A narrow-FOV")
        print("  solid-state LiDAR (Livox Avia) often can't be tracked by pure point-to-")
        print("  point ICP and needs IMU-aided odometry (e.g. FAST-LIO), denser frame")
        print("  integration, or KISS-ICP retuning (smaller voxel_size / max_range).")
    elif abs(lag) > 0.05:
        print("  Odometries agree but are time-shifted -> timestamp problem; revisit "
              "timestamp_source / inspect_bag_timing.")
    else:
        print("  Odometries agree and are aligned -> residual is likely degenerate "
              "motion or a solver/axis issue, not the data.")

    try:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))
        pa = np.array([T[:3, 3] for T in WA]); pb = np.array([T[:3, 3] for T in WB])
        ax[0].plot(pa[:, 0], pa[:, 1], lw=1); ax[0].set_title("Hesai trajectory (own frame, XY)"); ax[0].axis("equal")
        ax[1].plot(pb[:, 0], pb[:, 1], lw=1, color="C1"); ax[1].set_title("Livox trajectory (own frame, XY)"); ax[1].axis("equal")
        ax[2].plot(g, wa + wa.mean(), lw=0.8, label="Hesai"); ax[2].plot(g, wb + wb.mean(), lw=0.8, label="Livox")
        ax[2].set_title("Angular speed vs time"); ax[2].set_xlabel("s"); ax[2].legend()
        fig.tight_layout(); fig.savefig(args.plot, dpi=120)
        print(f"\nplot -> {args.plot}  (a healthy pair shows two similar clean paths)")
    except Exception as e:
        print(f"(plot skipped: {e})")


if __name__ == "__main__":
    main()
