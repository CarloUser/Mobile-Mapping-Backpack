#!/usr/bin/env python3
"""Step 3: validate a solved Hesai<->Livox extrinsic.

Reads the two trajectories and the calibrated YAML, then reports:
  * residual distribution of A_i X = X B_i (rotation deg, translation m),
  * a two-fold cross-validation: solve X on each half of the data independently
    and report how consistent the two estimates are (a real calibration should
    agree to a fraction of a degree / a few mm),
  * a PNG with residual histograms and a trajectory-consistency plot.

A good result: low, unbiased residuals AND tight cross-validation agreement.
High residuals or large split disagreement => insufficient/!degenerate motion or
a topic/time-sync problem; recollect before trusting the numbers.

Usage:
  python validate.py --hesai-traj out/hesai_tum.txt --livox-traj out/livox_tum.txt \
      --config config.yaml
"""
import argparse
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

import handeye as he
import io_utils as io


def x_diff(Xa, Xb):
    E = he.inv(Xa) @ Xb
    return (np.degrees(np.linalg.norm(R.from_matrix(E[:3, :3]).as_rotvec())),
            np.linalg.norm(E[:3, 3]))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hesai-traj", required=True)
    ap.add_argument("--livox-traj", required=True)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--calibrated", help="Calibrated YAML (default from config)")
    ap.add_argument("--plot", default="out/validation.png")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    stride = cfg["pairing"]["stride"]
    min_angle = cfg["pairing"]["min_angle_deg"]
    cal_path = args.calibrated or cfg["paths"]["output_extrinsics"]

    # X from the calibrated file: X = inv(base->hesai) @ (base->livox_calibrated).
    _, frames = io.load_extrinsics(cal_path)
    X = he.inv(frames["lidar_hesai"]) @ frames["lidar_livox_avia"]

    th, WA = io.load_tum(args.hesai_traj)
    tl, WB = io.load_tum(args.livox_traj)
    lo, hi = max(th[0], tl[0]), min(th[-1], tl[-1])
    m = (th >= lo) & (th <= hi)
    th, WA = th[m], [WA[i] for i in np.where(m)[0]]
    WB = he.interp_pose(tl, WB, th)
    A, B = he.motion_pairs(WA, WB, min_angle_deg=min_angle, stride=stride)

    rot, trn = he.residuals(A, B, X)
    obs = he.rotation_observability(A)
    print(f"pairs={len(A)}  observability={obs:.3f}")
    print(f"residual rotation   (deg): median {np.median(rot):.3f}  mean {rot.mean():.3f}  "
          f"95th {np.percentile(rot,95):.3f}  max {rot.max():.3f}")
    print(f"residual translation (m):  median {np.median(trn):.4f}  mean {trn.mean():.4f}  "
          f"95th {np.percentile(trn,95):.4f}  max {trn.max():.4f}")

    # Two-fold cross-validation (interleaved split to balance motion content).
    idx = np.arange(len(A))
    Aa = [A[i] for i in idx[0::2]]; Ba = [B[i] for i in idx[0::2]]
    Ab = [A[i] for i in idx[1::2]]; Bb = [B[i] for i in idx[1::2]]
    Xa = he.solve_handeye_robust(Aa, Ba)[0]
    Xb = he.solve_handeye_robust(Ab, Bb)[0]
    dr, dt = x_diff(Xa, Xb)
    print(f"cross-validation (two halves): rotation {dr:.3f} deg, translation {dt*100:.2f} cm")
    verdict = "GOOD" if (np.median(rot) < 0.5 and np.median(trn) < 0.02 and dr < 0.5 and dt < 0.02) \
        else "CHECK MOTION / SYNC"
    print(f"verdict: {verdict}")

    # Plots (headless).
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(15, 4))
        ax[0].hist(rot, bins=30, color="#4C78A8"); ax[0].set_title("Rotation residual (deg)")
        ax[1].hist(trn * 100, bins=30, color="#F58518"); ax[1].set_title("Translation residual (cm)")
        # Consistency: Hesai positions vs Livox positions mapped into Hesai frame by X.
        pa = np.array([T[:3, 3] for T in WA])
        pb = np.array([(T @ he.inv(X))[:3, 3] for T in WB])
        ax[2].plot(pa[:, 0], pa[:, 1], label="Hesai", lw=1)
        ax[2].plot(pb[:, 0], pb[:, 1], label="Livox->Hesai via X", lw=1, ls="--")
        ax[2].set_title("Trajectory consistency (XY)"); ax[2].axis("equal"); ax[2].legend()
        fig.tight_layout(); fig.savefig(args.plot, dpi=120)
        print(f"plot -> {args.plot}")
    except Exception as e:
        print(f"(plot skipped: {e})")


if __name__ == "__main__":
    main()
