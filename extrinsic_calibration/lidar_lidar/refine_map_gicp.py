#!/usr/bin/env python3
"""Map-based refinement of the Hesai<->Livox extrinsic (small_gicp).

Idea (the standard 'calibration by mapping consistency' upgrade over hand-eye):
build a world map from the Hesai scans posed by the KISS-ICP trajectory, then
register each Livox scan into that map. Each registration k yields an
independent extrinsic estimate

    X_k = inv(T_world_hesai(t_k)) @ T_world_scan_k        (X = T_hesai_livox)

because the Livox scan's world pose must equal T_world_hesai(t_k) @ X. A
robust average over hundreds of scans, iterated with shrinking correspondence
distance, refines X far below the hand-eye noise floor (hand-eye is limited by
per-pose odometry jitter; here every scan is anchored to the full map).

Inputs are the SAME artifacts the hand-eye stage already produced: the
calibration bag, out/hesai_tum.txt, and the hand-eye X as the initial guess.

Usage:
    python3 refine_map_gicp.py --bag ~/recordings/lidar/lidar_calib_imu_20260609_190844 \
        [--hesai-traj out/hesai_tum.txt] [--config config.yaml] [--dry-run]

Outputs: refined X + diagnostics, two-fold split agreement, GOOD/CHECK verdict
(same thresholds as validate.py), updated extrinsics_calibrated.yaml, and a
top-down fused-map PNG (out/fused_map.png) for visual QA.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "camera_lidar"))
import io_cam                                    # bag readers + merging writer
import handeye as he
import io_utils as io


def log(msg):
    print(f"[refine] {msg}", flush=True)


def median_offset(reader, topic):
    """median(bag_receive - header) over `topic` -> seconds."""
    offs = []
    conns = [c for c in reader.connections if c.topic == topic]
    for conn, t_ns, raw in reader.messages(connections=conns):
        m = reader.deserialize(raw, conn.msgtype)
        st = m.header.stamp
        offs.append(t_ns * 1e-9 - (float(st.sec) + float(st.nanosec) * 1e-9))
    return float(np.median(offs))


def robust_mean_se3(Ts, trim=0.2, rounds=2):
    """Chordal-mean rotation + median translation with iterative trimming.

    Returns (T_mean, keep_mask, rot_dev_deg, trn_dev_m) — deviations of the
    KEPT samples from the mean, i.e. the per-scan residual distribution.
    """
    Ts = list(Ts)
    keep = np.ones(len(Ts), bool)
    for _ in range(rounds + 1):
        Rs = np.stack([T[:3, :3] for T, k in zip(Ts, keep) if k])
        ts = np.stack([T[:3, 3] for T, k in zip(Ts, keep) if k])
        U, _, Vt = np.linalg.svd(Rs.sum(axis=0))
        Rm = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
        tm = np.median(ts, axis=0)
        rot_dev = np.array([np.degrees(np.linalg.norm(
            R.from_matrix(T[:3, :3] @ Rm.T).as_rotvec())) for T in Ts])
        trn_dev = np.array([np.linalg.norm(T[:3, 3] - tm) for T in Ts])
        score = (rot_dev / max(np.median(rot_dev[keep]), 1e-9)
                 + trn_dev / max(np.median(trn_dev[keep]), 1e-9))
        order = np.argsort(score)
        n_keep = max(10, int(len(Ts) * (1 - trim)))
        keep[:] = False
        keep[order[:n_keep]] = True
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = tm
    return T, keep, rot_dev[keep], trn_dev[keep]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--hesai-traj", default="out/hesai_tum.txt")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--hesai-stride", type=int, default=2)
    ap.add_argument("--livox-stride", type=int, default=5)
    ap.add_argument("--map-voxel", type=float, default=0.05)
    ap.add_argument("--scan-voxel", type=float, default=0.10)
    ap.add_argument("--max-gyro-dps", type=float, default=45.0,
                    help="skip Livox scans where the rig rotates faster than "
                         "this (deg/s): anchor-pose interpolation error and "
                         "scan smear dominate there")
    ap.add_argument("--min-range", type=float, default=1.0)
    ap.add_argument("--max-range", type=float, default=60.0)
    ap.add_argument("--threads", type=int, default=6)
    ap.add_argument("--dry-run", action="store_true",
                    help="solve + report but do not write the extrinsics YAML")
    args = ap.parse_args()
    import small_gicp

    cfg = yaml.safe_load(open(args.config))
    here = Path(args.config).resolve().parent

    def rcfg(p):
        q = Path(p)
        return q if q.is_absolute() else (here / q).resolve()

    init_path = rcfg(cfg["paths"]["initial_extrinsics"])
    out_path = rcfg(cfg["paths"]["output_extrinsics"])
    _, frames_init = io_cam.load_extrinsics(init_path)
    _, frames_cal = io_cam.load_extrinsics(out_path)
    T_bh = frames_init["lidar_hesai"]
    T_bl = frames_cal.get("lidar_livox_avia", frames_init["lidar_livox_avia"])
    X = np.linalg.inv(T_bh) @ T_bl
    X_handeye = X.copy()
    log(f"initial X (hand-eye): t={np.round(X[:3,3],4).tolist()}")

    th, WH = io.load_tum(args.hesai_traj)
    hesai_topic = cfg["bag"].get("hesai_topic", "/lidar_points")
    livox_topic = cfg["bag"].get("livox_topic", "/livox/lidar")

    with io_cam.open_bag(args.bag) as reader:
        # --- clock alignment onto the hesai_tum (header_aligned) timeline ----
        # Livox scan stamps shifted by median(recv-header) over /livox/imu land
        # on that timeline with ~0 residual lag (diagnose_trajectories measured
        # 0 ms with exactly this pairing). The Hesai side needs NO time logic:
        # KISS-ICP emitted exactly one pose per scan, so scan k <-> WH[k].
        off_l = median_offset(reader, "/livox/imu")
        log(f"livox clock shift: {off_l:.3f}s")

        # --- 1. build the world map from Hesai scans (indexed 1:1) -----------
        t0 = time.time()
        chunks, n_scan = [], 0
        conns = [c for c in reader.connections if c.topic == hesai_topic]
        for conn, _, raw in reader.messages(connections=conns):
            k = n_scan
            n_scan += 1
            if k >= len(WH) or k % args.hesai_stride:
                continue
            m = reader.deserialize(raw, conn.msgtype)
            xyz = io_cam.pointcloud2_to_xyz(m).astype(np.float64)
            r = np.linalg.norm(xyz, axis=1)
            xyz = xyz[(r > args.min_range) & (r < args.max_range)]
            if len(xyz) < 100:
                continue
            xyz = small_gicp.voxelgrid_sampling(xyz, args.map_voxel).points()[:, :3]
            Twh = WH[k]
            chunks.append(xyz @ Twh[:3, :3].T + Twh[:3, 3])
        if n_scan != len(WH):
            log(f"WARNING: {n_scan} Hesai scans vs {len(WH)} trajectory poses "
                f"— 1:1 indexing assumption violated, results suspect")
        world = np.concatenate(chunks)
        del chunks
        log(f"map: {len(world):,} pts from {n_scan} Hesai scans "
            f"(stride {args.hesai_stride}) in {time.time()-t0:.0f}s")
        t0 = time.time()
        # preprocess_points voxelizes at map_voxel (do NOT pass 0 — it is a
        # divide-by-zero that silently collapses the cloud), then estimates
        # covariances and builds the KdTree.
        map_pc, map_tree = small_gicp.preprocess_points(
            world, downsampling_resolution=args.map_voxel,
            num_threads=args.threads)
        del world
        log(f"map: {map_pc.size():,} pts after {args.map_voxel} m voxel; "
            f"covariances + KdTree in {time.time()-t0:.0f}s")

        # Rig angular speed over time (from the trajectory) — used to skip
        # scans where interpolation error / scan smear dominates.
        w_t, w_v = [], []
        for i in range(0, len(WH) - 3, 3):
            dt = th[i + 3] - th[i]
            if dt <= 0:
                continue
            dR = np.linalg.inv(WH[i]) @ WH[i + 3]
            w_v.append(np.degrees(np.linalg.norm(
                R.from_matrix(dR[:3, :3]).as_rotvec())) / dt)
            w_t.append(0.5 * (th[i] + th[i + 3]))
        w_t, w_v = np.array(w_t), np.array(w_v)

        # --- 2. load the Livox scans we will register ------------------------
        scans, n_fast = [], 0
        conns = [c for c in reader.connections if c.topic == livox_topic]
        k = 0
        for conn, _, raw in reader.messages(connections=conns):
            k += 1
            if (k - 1) % args.livox_stride:
                continue
            m = reader.deserialize(raw, conn.msgtype)
            t = (float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9
                 + off_l)
            if t < th[0] + 0.5 or t > th[-1] - 0.5:
                continue
            if np.interp(t, w_t, w_v) > args.max_gyro_dps:
                n_fast += 1
                continue
            xyz = io_cam.livox_to_xyz(m).astype(np.float64)
            r = np.linalg.norm(xyz, axis=1)
            xyz = xyz[(r > args.min_range) & (r < args.max_range)]
            if len(xyz) < 500:
                continue
            scans.append((t, xyz))
        log(f"{len(scans)} Livox scans to register (stride {args.livox_stride}, "
            f"{n_fast} skipped > {args.max_gyro_dps} deg/s)")

    # --- 3. iterate: register every scan, robustly average X -----------------
    schedule = [1.0, 0.5, 0.25]          # max_correspondence_distance per round
    for rnd, max_corr in enumerate(schedule):
        t0 = time.time()
        Xs, used_t = [], []
        n_fail = 0
        for t, xyz in scans:
            Twh = he.interp_pose(th, WH, np.array([t]))[0]
            init = Twh @ X
            src_pc, _ = small_gicp.preprocess_points(
                xyz, downsampling_resolution=args.scan_voxel,
                num_threads=args.threads)
            res = small_gicp.align(
                map_pc, src_pc, map_tree, init_T_target_source=init,
                registration_type="GICP",
                max_correspondence_distance=max_corr,
                num_threads=args.threads, max_iterations=30)
            if not res.converged or res.num_inliers < 300:
                n_fail += 1
                continue
            Xs.append(np.linalg.inv(Twh) @ res.T_target_source)
            used_t.append(t)
        X, keep, rot_dev, trn_dev = robust_mean_se3(Xs)
        log(f"round {rnd+1}/{len(schedule)} (corr {max_corr} m): "
            f"{len(Xs)} ok, {n_fail} failed, {int(keep.sum())} kept | "
            f"dev med {np.median(rot_dev):.3f} deg / "
            f"{np.median(trn_dev)*100:.2f} cm | {time.time()-t0:.0f}s")

    # --- 4. diagnostics -------------------------------------------------------
    used_t = np.array(used_t)
    kept_idx = np.where(keep)[0]
    halves = used_t[kept_idx] < np.median(used_t)
    Xa, *_ = robust_mean_se3([Xs[i] for i in kept_idx[halves]], trim=0.1)
    Xb, *_ = robust_mean_se3([Xs[i] for i in kept_idx[~halves]], trim=0.1)
    dT = np.linalg.inv(Xa) @ Xb
    cv_rot = np.degrees(np.linalg.norm(R.from_matrix(dT[:3, :3]).as_rotvec()))
    cv_trn = np.linalg.norm(dT[:3, 3])

    dX = np.linalg.inv(X_handeye) @ X
    d_rot = np.degrees(np.linalg.norm(R.from_matrix(dX[:3, :3]).as_rotvec()))
    d_trn = np.linalg.norm(dX[:3, 3])

    rot_med, trn_med = float(np.median(rot_dev)), float(np.median(trn_dev))
    n_kept = int(keep.sum())
    # The verdict is on the AVERAGED estimate, not on single-scan scatter:
    # each registration is an independent estimate of X, so scatter/sqrt(N)
    # bounds random error, and the two-fold split agreement exposes the
    # systematic error (trajectory drift, residual time offset). Gate on the
    # split (same 0.5 deg / 2 cm bar as validate.py's cross-validation) plus a
    # minimum healthy sample count.
    good = (cv_rot < 0.5 and cv_trn < 0.02 and n_kept >= 100)

    print()
    log(f"per-scan scatter (diagnostic): rotation med {rot_med:.3f} deg "
        f"(95th {np.percentile(rot_dev,95):.3f}), translation med "
        f"{trn_med*100:.2f} cm (95th {np.percentile(trn_dev,95)*100:.2f})")
    log(f"random error of the mean (~scatter/sqrt(N), N={n_kept}): "
        f"{rot_med/np.sqrt(n_kept):.4f} deg / "
        f"{trn_med/np.sqrt(n_kept)*100:.3f} cm")
    log(f"two-fold split agreement (systematic bound): "
        f"{cv_rot:.3f} deg / {cv_trn*100:.2f} cm")
    log(f"refined-vs-hand-eye correction: {d_rot:.3f} deg / {d_trn*100:.2f} cm")
    log(f"verdict: {'GOOD' if good else 'CHECK'} "
        f"(criteria: split agreement < 0.5 deg / 2 cm, >= 100 scans kept)")

    T_bl_new = T_bh @ X
    q = R.from_matrix(T_bl_new[:3, :3]).as_quat()
    log(f"base_link -> lidar_livox_avia: xyz={np.round(T_bl_new[:3,3],5).tolist()} "
        f"q_xyzw={np.round(q,5).tolist()}")

    # --- 5. QA artifact: top-down fused map ----------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 11))
        mp = map_pc.points()[:, :3]
        sel = np.random.default_rng(0).choice(
            len(mp), min(len(mp), 800_000), replace=False)
        ax.scatter(mp[sel, 0], mp[sel, 1], s=0.05, c="0.65", lw=0,
                   label="Hesai map")
        fused = []
        for j, (t, xyz) in enumerate(scans):
            if j % max(1, len(scans)//120):
                continue
            Twh = he.interp_pose(th, WH, np.array([t]))[0]
            w = xyz[::20] @ (Twh @ X)[:3, :3].T + (Twh @ X)[:3, 3]
            fused.append(w)
        fused = np.concatenate(fused)
        ax.scatter(fused[:, 0], fused[:, 1], s=0.05, c="crimson", lw=0,
                   label="Livox @ refined X")
        ax.set_aspect("equal"); ax.legend(markerscale=40)
        ax.set_title("Fused map, top-down — walls should coincide")
        out_png = Path("out/fused_map.png")
        fig.savefig(out_png, dpi=160, bbox_inches="tight")
        log(f"QA plot -> {out_png}")
    except Exception as e:   # plotting must never kill the result
        log(f"QA plot skipped: {e}")

    if args.dry_run:
        log("--dry-run: extrinsics YAML not written")
        return
    provenance = {
        "stage": "lidar_lidar_map_refine",
        "method": "small_gicp GICP scan-to-map registration, robust SE(3) mean",
        "n_scans_kept": int(keep.sum()),
        "dev_rot_med_deg": round(float(rot_med), 4),
        "dev_trn_med_m": round(float(trn_med), 5),
        "split_agreement_deg": round(float(cv_rot), 4),
        "split_agreement_m": round(float(cv_trn), 5),
        "verdict": "GOOD" if good else "CHECK",
    }
    io_cam.update_calibrated_extrinsics(
        out_path, init_path, {"lidar_livox_avia": T_bl_new}, provenance)
    log(f"wrote {out_path}")


if __name__ == "__main__":
    main()
