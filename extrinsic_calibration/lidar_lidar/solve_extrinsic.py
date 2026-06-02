#!/usr/bin/env python3
"""Step 2: solve the Hesai <-> Livox extrinsic from two LiDAR trajectories.

Pipeline:
  1. Load both TUM trajectories (from run_odometry.py).
  2. Interpolate the Livox trajectory onto the Hesai timestamps (time alignment).
  3. Form relative-motion pairs (A_i = Hesai motion, B_i = Livox motion).
  4. Check rotation observability; warn loudly if the motion was too planar.
  5. Robustly solve A_i X = X B_i for X = T_hesai_livox.
  6. Chain with the CAD base_link->lidar_hesai (Hesai is the fixed reference) to
     get the refined base_link->lidar_livox_avia, and report the correction vs CAD.
  7. Write extrinsics_calibrated.yaml.

Usage:
  python solve_extrinsic.py --hesai-traj out/hesai_tum.txt \
      --livox-traj out/livox_tum.txt --config config.yaml
"""
import argparse
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

import handeye as he
import io_utils as io


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hesai-traj", required=True)
    ap.add_argument("--livox-traj", required=True)
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--out", help="Output calibrated YAML (default from config)")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    stride = cfg["pairing"]["stride"]
    min_angle = cfg["pairing"]["min_angle_deg"]
    min_obs = cfg["pairing"]["min_observability"]
    init_path = cfg["paths"]["initial_extrinsics"]
    out_path = args.out or cfg["paths"]["output_extrinsics"]

    # 1-2. Load + time-align (Livox interpolated onto Hesai stamps).
    th, WA = io.load_tum(args.hesai_traj)          # A = Hesai (reference)
    tl, WB_raw = io.load_tum(args.livox_traj)      # B = Livox
    overlap = (max(th[0], tl[0]), min(th[-1], tl[-1]))
    mask = (th >= overlap[0]) & (th <= overlap[1])
    th, WA = th[mask], [WA[i] for i in np.where(mask)[0]]
    WB = he.interp_pose(tl, WB_raw, th)
    print(f"[solve] aligned {len(WA)} frames over {th[-1]-th[0]:.1f}s")

    # 3. Motion pairs.
    A, B = he.motion_pairs(WA, WB, min_angle_deg=min_angle, stride=stride)
    if len(A) < 15:
        raise SystemExit(f"Only {len(A)} usable motion pairs. Record a longer "
                         f"sequence with more rotation, or lower min_angle_deg.")

    # 4. Observability gate.
    obs = he.rotation_observability(A)
    print(f"[solve] motion pairs={len(A)}  rotation-observability={obs:.3f} "
          f"(want > {min_obs})")
    if obs < min_obs:
        print("  *** WARNING: motion was nearly planar (e.g. yaw-only walking). ***\n"
              "  *** Relative roll/pitch is poorly constrained. Re-record with    ***\n"
              "  *** pitch/roll excitation (tilt the rig, ramps/stairs).         ***")

    # 5. Robust solve.
    X, A_in, B_in = he.solve_handeye_robust(A, B)
    rot_res, trn_res = he.residuals(A_in, B_in, X)
    print(f"[solve] X = T_hesai_livox solved on {len(A_in)} inliers")
    print(f"  residual rotation:    median {np.median(rot_res):.3f} deg, "
          f"95th {np.percentile(rot_res,95):.3f} deg")
    print(f"  residual translation: median {np.median(trn_res):.4f} m, "
          f"95th {np.percentile(trn_res,95):.4f} m")

    # 6. Chain to base_link and compare with CAD.
    doc, frames = io.load_extrinsics(init_path)
    T_base_hesai = frames["lidar_hesai"]            # reference, kept from CAD
    T_base_livox_cad = frames["lidar_livox_avia"]
    T_base_livox_cal = T_base_hesai @ X

    X_cad = he.inv(T_base_hesai) @ T_base_livox_cad  # CAD-implied hesai->livox
    dE = he.inv(X) @ X_cad
    d_rot = np.degrees(np.linalg.norm(R.from_matrix(dE[:3, :3]).as_rotvec()))
    d_trn = np.linalg.norm(dE[:3, 3])
    print(f"[solve] correction vs CAD: rotation {d_rot:.3f} deg, "
          f"translation {d_trn*100:.2f} cm")
    if d_rot > 15 or d_trn > 0.15:
        print("  *** Large deviation from CAD. Verify frame axes/topics before trusting. ***")

    # 7. Write calibrated YAML.
    provenance = {
        "method": "motion-based hand-eye (A X = X B) on per-LiDAR KISS-ICP trajectories",
        "reference_frame": cfg["reference_frame"],
        "refined_frame": cfg["child_frame"],
        "inlier_pairs": int(len(A_in)),
        "rotation_observability": round(obs, 4),
        "residual_rot_med_deg": round(float(np.median(rot_res)), 4),
        "residual_trn_med_m": round(float(np.median(trn_res)), 5),
        "correction_vs_cad_rot_deg": round(float(d_rot), 4),
        "correction_vs_cad_trn_m": round(float(d_trn), 5),
    }
    io.write_calibrated_extrinsics(out_path, doc,
                                   {"lidar_livox_avia": T_base_livox_cal}, provenance)
    print(f"\n[solve] wrote {out_path}")
    print("  base_link -> lidar_livox_avia:")
    print(f"    xyz    = {[round(float(v),6) for v in T_base_livox_cal[:3,3]]}")
    print(f"    q_xyzw = {[round(float(v),6) for v in R.from_matrix(T_base_livox_cal[:3,:3]).as_quat()]}")
    print("\nNext: python validate.py --hesai-traj %s --livox-traj %s --config %s"
          % (args.hesai_traj, args.livox_traj, args.config))


if __name__ == "__main__":
    main()
