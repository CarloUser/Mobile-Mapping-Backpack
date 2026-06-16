#!/usr/bin/env python3
"""Camera<->LiDAR extrinsic calibration from one recorded bag (one pair).

Pipeline per CONTEXT.md section 3:
  1. Detect the ChArUco board in every Nth image -> board pose + plane (camera).
  2. Cluster detections into STATIC HOLDS (the recording protocol holds the
     board still 3-5 s per pose); one representative plane per hold.
  3. For each hold, accumulate LiDAR points around the hold centre, crop a
     sphere around the CAD-PREDICTED board position (seeded from
     extrinsics_initial.yaml — this is what makes segmentation automatic),
     RANSAC a plane, sanity-check its size and normal.
  4. Solve T_cam_lidar from the plane correspondences (closed form), then
     point-to-plane nonlinear refine.
  5. Chain to base_link through the pair's reference LiDAR (the calibrated
     Hesai<->Livox transform for rear cameras) and merge into
     extrinsics_calibrated.yaml.

Usage:
    python3 calibrate_cam_lidar.py --bag <bag_dir> --pair oak4d [--config config.yaml]
    python3 calibrate_cam_lidar.py --bag <bag_dir> --pair oak1 --dry-run
"""
import argparse
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot

import io_cam
import plane_solve as ps
from board import BoardDetector

# cv2 solvePnP/ChArUco returns the board pose in the OPTICAL frame
# (X-right, Y-down, Z-forward). This rig's extrinsics use base-aligned CAD/body
# frames (REP-103: X-forward, Y-left, Z-up). Convert the camera-frame board
# quantities optical->body so they chain correctly through the CAD extrinsics
# (identity seed then crops correctly) and the solved T_base_cam comes out in
# the body convention, matching extrinsics_initial.yaml. Plane offset d is
# rotation-invariant (frames share an origin), so it is left unchanged.
R_OPT2BODY = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


def resolve(config_path, value):
    p = Path(value)
    return p if p.is_absolute() else (Path(config_path).resolve().parent / p).resolve()


def cluster_holds(detections, min_hold_s, max_gap_s, max_rot_deg, max_trans_m):
    """Group time-ordered detections into static holds.

    detections: list of (t, det_dict). A hold ends when the board moves beyond
    the thresholds vs the hold's running median pose, or detections stop for
    longer than max_gap_s. Returns a list of holds, each a list of (t, det).
    """
    holds, cur = [], []

    def med_pose(seg):
        ts = np.array([Rot.from_matrix(d["T_cam_board"][:3, :3]).as_rotvec()
                       for _, d in seg])
        xyz = np.array([d["T_cam_board"][:3, 3] for _, d in seg])
        return np.median(ts, axis=0), np.median(xyz, axis=0)

    def fits(seg, det):
        rv_med, t_med = med_pose(seg)
        rv = Rot.from_matrix(det["T_cam_board"][:3, :3]).as_rotvec()
        dr = np.degrees(np.linalg.norm(
            (Rot.from_rotvec(rv) * Rot.from_rotvec(rv_med).inv()).as_rotvec()))
        dt = np.linalg.norm(det["T_cam_board"][:3, 3] - t_med)
        return dr <= max_rot_deg and dt <= max_trans_m

    for t, det in detections:
        if cur and (t - cur[-1][0] > max_gap_s or not fits(cur, det)):
            holds.append(cur)
            cur = []
        cur.append((t, det))
    if cur:
        holds.append(cur)
    return [h for h in holds if h[-1][0] - h[0][0] >= min_hold_s]


def hold_representative(hold):
    """Median plane + central time of one hold."""
    ts = np.array([t for t, _ in hold])
    normals = np.array([d["n"] for _, d in hold])
    # Median of normals then renormalise; median of offsets.
    n = np.median(normals, axis=0)
    n /= np.linalg.norm(n)
    d = float(np.median([d_["d"] for _, d_ in hold]))
    centers = np.array([d_["T_cam_board"][:3, 3] for _, d_ in hold])
    boards = np.array(
        [(d_["T_cam_board"][:3, :3] @ d_["corners_obj"].mean(axis=0))
         + d_["T_cam_board"][:3, 3] for _, d_ in hold])
    # optical -> base-aligned body frame (see R_OPT2BODY note); d is invariant.
    return {
        "t_center": float(ts[len(ts) // 2]),
        "n": R_OPT2BODY @ n, "d": d,
        "board_center_cam": R_OPT2BODY @ np.median(boards, axis=0),
        "board_origin_cam": R_OPT2BODY @ np.median(centers, axis=0),
        "n_frames": len(hold),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--pair", required=True, help="pair name from config.yaml")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--dry-run", action="store_true",
                    help="run everything but do not write the output YAML")
    ap.add_argument("--save-debug", default=None,
                    help="directory for per-hold debug dumps (patches, planes)")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    if args.pair not in cfg["pairs"]:
        raise SystemExit(f"Unknown pair '{args.pair}'. "
                         f"Available: {sorted(cfg['pairs'])}")
    pair = cfg["pairs"][args.pair]
    bcfg, hcfg, lcfg, scfg = cfg["board"], cfg["holds"], cfg["lidar"], cfg["solve"]

    init_path = resolve(args.config, cfg["paths"]["initial_extrinsics"])
    out_path = resolve(args.config, cfg["paths"]["output_extrinsics"])
    _, frames_init = io_cam.load_extrinsics(init_path)
    # Reference-LiDAR pose in base: use the CALIBRATED file when it has the
    # frame (rear cameras chain through the solved Hesai<->Livox), else CAD.
    frames_cal = {}
    if out_path.exists():
        _, frames_cal = io_cam.load_extrinsics(out_path)
    lidar_frame, cam_frame = pair["lidar_frame"], pair["camera_frame"]
    T_base_lidar = frames_cal.get(lidar_frame, frames_init.get(lidar_frame))
    if T_base_lidar is None:
        raise SystemExit(f"Frame '{lidar_frame}' not found in extrinsics YAMLs.")
    src = "calibrated" if lidar_frame in frames_cal else "CAD initial"
    print(f"[calib] pair={args.pair}: {cam_frame} <-> {lidar_frame} "
          f"(T_base_{lidar_frame} from {src})")

    # CAD seed for T_cam_lidar = inv(T_base_cam_CAD) @ T_base_lidar — used ONLY
    # to predict where the board is in the lidar cloud (crop + normal gate).
    T_base_cam_cad = frames_init.get(cam_frame)
    if T_base_cam_cad is None:
        raise SystemExit(f"Frame '{cam_frame}' not in {init_path}.")
    T_cam_lidar_seed = np.linalg.inv(T_base_cam_cad) @ T_base_lidar
    T_lidar_cam_seed = np.linalg.inv(T_cam_lidar_seed)

    detector = BoardDetector(
        bcfg["squares_x"], bcfg["squares_y"], bcfg["square_len_m"],
        bcfg["marker_len_m"], bcfg["dictionary"], bcfg["min_corners"])
    board_diag = float(np.hypot(*detector.size_m))
    fisheye = pair.get("camera_model", "pinhole") == "fisheye"

    # --- 1. detect the board over the whole bag --------------------------------
    with io_cam.open_bag(args.bag, cfg.get("ros_distro", "humble")) as reader:
        info = io_cam.read_camera_info(reader, pair["camera_info_topic"])
        print(f"[calib] camera: {info['width']}x{info['height']} "
              f"model={info['model']} (treating as "
              f"{'fisheye' if fisheye else 'pinhole'})")
        detections, n_img = [], 0
        for t_recv, _, img in io_cam.iter_images(
                reader, pair["image_topic"], stride=hcfg["image_stride"]):
            n_img += 1
            det = detector.detect(img, info["K"], info["D"], fisheye=fisheye)
            if det and det["reproj_err"] <= bcfg["max_reproj_err_px"]:
                detections.append((t_recv, det))
        print(f"[calib] board seen in {len(detections)}/{n_img} processed images")
        if not detections:
            raise SystemExit("Board never detected — check board params/topics.")

        # --- 2. static holds ----------------------------------------------------
        holds = cluster_holds(detections, hcfg["min_hold_s"], hcfg["max_gap_s"],
                              hcfg["max_rot_deg"], hcfg["max_trans_m"])
        reps = [hold_representative(h) for h in holds]
        print(f"[calib] {len(reps)} static holds "
              f"(want ~15-25 well-spread poses)")

        # --- 3. lidar board patches --------------------------------------------
        planes_cam, planes_lidar, patches, used = [], [], [], []
        crop_r = board_diag / 2 + lcfg["crop_margin_m"]
        for k, rep in enumerate(reps):
            cloud = io_cam.collect_cloud(reader, pair["lidar_topic"],
                                         rep["t_center"], lcfg["half_window_s"])
            if not len(cloud):
                print(f"  hold {k:2d}: no lidar points in window — skipped")
                continue
            center_lidar = (T_lidar_cam_seed[:3, :3] @ rep["board_center_cam"]
                            + T_lidar_cam_seed[:3, 3])
            crop = cloud[np.linalg.norm(cloud - center_lidar, axis=1) < crop_r]
            fit = ps.ransac_plane(crop, lcfg["ransac_dist_m"],
                                  lcfg["ransac_iters"], lcfg["min_inliers"],
                                  rng=k)
            if fit is None:
                print(f"  hold {k:2d}: no plane in crop "
                      f"({len(crop)} pts) — skipped")
                continue
            n_l, d_l, c_l, mask = fit
            patch = crop[mask]
            # Wall guard: patch must be board-sized...
            e1, e2 = ps.patch_extent(patch, n_l)
            lim = max(detector.size_m) + lcfg["max_extent_margin_m"]
            if e1 > lim:
                print(f"  hold {k:2d}: patch {e1:.2f}x{e2:.2f} m too large "
                      f"(board {detector.size_m[0]:.2f}x"
                      f"{detector.size_m[1]:.2f}) — skipped")
                continue
            # ...and its normal must roughly agree with the CAD prediction.
            n_pred = T_lidar_cam_seed[:3, :3] @ rep["n"]
            ang = np.degrees(np.arccos(np.clip(abs(n_l @ n_pred), -1, 1)))
            if ang > lcfg["max_normal_vs_cad_deg"]:
                print(f"  hold {k:2d}: patch normal {ang:.0f} deg off CAD "
                      f"prediction — skipped")
                continue
            planes_cam.append((rep["n"], rep["d"]))
            planes_lidar.append((n_l, c_l))
            patches.append(patch)
            used.append(k)
            print(f"  hold {k:2d}: t={rep['t_center']:.1f} "
                  f"{len(patch):5d} patch pts, extent {e1:.2f}x{e2:.2f} m")
            if args.save_debug:
                dbg = Path(args.save_debug); dbg.mkdir(parents=True, exist_ok=True)
                np.savez(dbg / f"hold_{k:02d}.npz", patch=patch, crop=crop,
                         n_l=n_l, c_l=c_l, n_c=rep["n"], d_c=rep["d"])

    # --- 4. solve ---------------------------------------------------------------
    if len(planes_cam) < scfg["min_poses"]:
        raise SystemExit(f"Only {len(planes_cam)} usable holds "
                         f"(need >= {scfg['min_poses']}). Re-record with more "
                         f"poses / check the skipped-hold reasons above.")
    T0, info_solve = ps.solve_cam_lidar(planes_cam, planes_lidar,
                                        scfg["min_normal_spread"])
    print(f"[solve] {len(planes_cam)} planes, normal spread "
          f"{info_solve['normal_spread']:.3f}, normal residual "
          f"{info_solve['normal_residual_deg']['median']:.2f} deg median")

    T = T0
    if scfg.get("refine", True):
        T, res = ps.refine_point_to_plane(T0, planes_cam, patches)
        r_mm = np.abs(res.fun) * 1000
        print(f"[refine] point-to-plane |residual|: median {np.median(r_mm):.1f} mm,"
              f" 95th {np.percentile(r_mm, 95):.1f} mm over {len(r_mm)} pts")
        d_ang = np.degrees(np.linalg.norm(Rot.from_matrix(
            T[:3, :3] @ T0[:3, :3].T).as_rotvec()))
        print(f"[refine] moved solution by {d_ang:.2f} deg / "
              f"{np.linalg.norm(T[:3, 3] - T0[:3, 3]) * 100:.1f} cm")

    # --- 5. chain + report + write ----------------------------------------------
    T_base_cam = T_base_lidar @ np.linalg.inv(T)
    q = Rot.from_matrix(T_base_cam[:3, :3]).as_quat()
    print(f"\n[result] T_cam_lidar ({cam_frame} <- {lidar_frame}):")
    print(f"  xyz    = {np.round(T[:3, 3], 4).tolist()}")
    print(f"  q_xyzw = {np.round(Rot.from_matrix(T[:3, :3]).as_quat(), 4).tolist()}")
    print(f"[result] base_link -> {cam_frame}:")
    print(f"  xyz    = {np.round(T_base_cam[:3, 3], 4).tolist()}")
    print(f"  q_xyzw = {np.round(q, 4).tolist()}")
    dcad = T_base_cam_cad[:3, 3] - T_base_cam[:3, 3]
    dang = np.degrees(np.linalg.norm(Rot.from_matrix(
        T_base_cam[:3, :3] @ T_base_cam_cad[:3, :3].T).as_rotvec()))
    print(f"[result] vs CAD: rotation {dang:.1f} deg, translation "
          f"{np.linalg.norm(dcad) * 100:.1f} cm  "
          f"(NB: CAD frames are SolidWorks part frames — large rotation "
          f"offsets may be convention, not error; see lidar_lidar findings)")

    if args.dry_run:
        print("[calib] --dry-run: not writing output YAML")
        return
    provenance = {
        "stage": "camera_lidar", "pair": args.pair,
        "method": "ChArUco plane correspondences (Zhang-Pless) + "
                  "point-to-plane refine",
        "lidar_reference": lidar_frame,
        "lidar_reference_source": src,
        "n_holds_used": len(planes_cam),
        "normal_spread": round(info_solve["normal_spread"], 4),
        "normal_residual_med_deg":
            round(info_solve["normal_residual_deg"]["median"], 3),
    }
    io_cam.update_calibrated_extrinsics(
        out_path, init_path, {cam_frame: T_base_cam}, provenance)
    print(f"[calib] wrote {out_path}")


if __name__ == "__main__":
    main()
