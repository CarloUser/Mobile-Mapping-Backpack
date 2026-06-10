"""Synthetic unit tests for the camera<->LiDAR stage. Run: python3 -m pytest -q

Covers:
  - BoardDetector pose/plane recovery on a rendered, homography-warped board
  - plane solve: exact recovery on clean data, graceful noise behaviour,
    refusal on degenerate (near-parallel) board normals
  - static-hold clustering
  - extrinsics YAML merge (does not clobber other stages' calibrated entries)
"""
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation as Rot

import io_cam
import plane_solve as ps
from board import BoardDetector
from calibrate_cam_lidar import cluster_holds


def rand_T(rng, max_deg=180.0, max_t=1.0):
    T = np.eye(4)
    ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
    T[:3, :3] = Rot.from_rotvec(
        np.radians(rng.uniform(5, max_deg)) * ax).as_matrix()
    T[:3, 3] = rng.uniform(-max_t, max_t, 3)
    return T


# ---------------------------------------------------------------------------
# board.py
# ---------------------------------------------------------------------------

BOARD = dict(squares_x=7, squares_y=5, square_len_m=0.10, marker_len_m=0.075)
K = np.array([[800.0, 0, 640], [0, 800.0, 360], [0, 0, 1]])


def render_board_view(detector, T_cam_board, px_per_m=2000):
    """Render the canonical board and warp it into a synthetic camera view."""
    import cv2
    w_m, h_m = detector.size_m
    canvas = detector.board.generateImage(
        (int(w_m * px_per_m), int(h_m * px_per_m)))
    # Board-plane metric coords of the rendered image corners (z=0 plane).
    src = np.array([[0, 0], [w_m, 0], [w_m, h_m], [0, h_m]], np.float64)
    Rm, t = T_cam_board[:3, :3], T_cam_board[:3, 3]
    pts3 = src @ np.array([[1, 0, 0], [0, 1, 0]], float) @ Rm.T + t
    img_pts, _ = cv2.projectPoints(
        pts3, np.zeros(3), np.zeros(3), K, np.zeros(5))
    src_px = (src * px_per_m).astype(np.float32)
    H = cv2.getPerspectiveTransform(src_px, img_pts.reshape(-1, 2).astype(np.float32))
    return cv2.warpPerspective(canvas, H, (1280, 720),
                               borderValue=120)


def test_board_detection_recovers_pose():
    det = BoardDetector(**BOARD)
    T_true = np.eye(4)
    T_true[:3, :3] = Rot.from_euler("xyz", [15, -20, 8], degrees=True).as_matrix()
    T_true[:3, 3] = [-0.25, -0.15, 1.6]
    img = render_board_view(det, T_true)
    out = det.detect(img, K, np.zeros(5))
    assert out is not None, "board not detected in synthetic image"
    dT = np.linalg.inv(T_true) @ out["T_cam_board"]
    ang = np.degrees(np.linalg.norm(Rot.from_matrix(dT[:3, :3]).as_rotvec()))
    assert ang < 0.5, f"rotation error {ang:.2f} deg"
    assert np.linalg.norm(dT[:3, 3]) < 0.01, "translation error > 1 cm"
    # Plane convention: n toward camera, n.X + d = 0 on the board.
    center = (T_true[:3, :3] @ [det.size_m[0] / 2, det.size_m[1] / 2, 0]
              + T_true[:3, 3])
    assert abs(out["n"] @ center + out["d"]) < 5e-3
    assert out["n"] @ center < 0


def test_board_rejects_blank_image():
    det = BoardDetector(**BOARD)
    assert det.detect(np.full((720, 1280), 90, np.uint8), K, np.zeros(5)) is None


# ---------------------------------------------------------------------------
# plane_solve.py
# ---------------------------------------------------------------------------

def synth_planes(rng, T_cam_lidar, n_poses=15, noise_m=0.0, tilt_spread=60):
    """Boards at varied poses; returns (planes_cam, planes_lidar, patches)."""
    T_lidar_cam = np.linalg.inv(T_cam_lidar)
    planes_cam, planes_lidar, patches = [], [], []
    for _ in range(n_poses):
        # Board pose in the CAMERA frame: 1-4 m out, tilted up to tilt_spread.
        ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
        Rb = Rot.from_rotvec(np.radians(rng.uniform(0, tilt_spread)) * ax
                             ).as_matrix()
        c = np.array([rng.uniform(-1, 1), rng.uniform(-0.6, 0.6),
                      rng.uniform(1.0, 4.0)])
        n = Rb[:, 2].copy()
        if n @ c > 0:
            n = -n
        d = -float(n @ c)
        planes_cam.append((n, d))
        # Board points sampled in the camera frame, mapped into the lidar frame.
        uv = rng.uniform(-0.35, 0.35, (400, 2))
        pts_cam = c + uv @ np.stack([Rb[:, 0], Rb[:, 1]])
        pts_lid = pts_cam @ T_lidar_cam[:3, :3].T + T_lidar_cam[:3, 3]
        if noise_m:
            pts_lid = pts_lid + rng.normal(0, noise_m, pts_lid.shape)
        n_l, _, c_l, _ = ps.fit_plane(pts_lid)
        planes_lidar.append((n_l, c_l))
        patches.append(pts_lid)
    return planes_cam, planes_lidar, patches


def test_solve_exact_recovery():
    rng = np.random.default_rng(1)
    T_true = rand_T(rng)
    pc, pl, _ = synth_planes(rng, T_true)
    T, info = ps.solve_cam_lidar(pc, pl)
    dT = np.linalg.inv(T_true) @ T
    ang = np.degrees(np.linalg.norm(Rot.from_matrix(dT[:3, :3]).as_rotvec()))
    assert ang < 0.01 and np.linalg.norm(dT[:3, 3]) < 1e-4
    assert info["normal_spread"] > 0.1


def test_solve_with_noise_and_refine():
    rng = np.random.default_rng(2)
    T_true = rand_T(rng)
    pc, pl, patches = synth_planes(rng, T_true, n_poses=20, noise_m=0.01)
    T0, _ = ps.solve_cam_lidar(pc, pl)
    T, _ = ps.refine_point_to_plane(T0, pc, patches)
    dT = np.linalg.inv(T_true) @ T
    ang = np.degrees(np.linalg.norm(Rot.from_matrix(dT[:3, :3]).as_rotvec()))
    assert ang < 0.5, f"rotation error {ang:.2f} deg with 1 cm point noise"
    assert np.linalg.norm(dT[:3, 3]) < 0.02


def test_solve_rejects_parallel_normals():
    rng = np.random.default_rng(3)
    T_true = rand_T(rng)
    pc, pl, _ = synth_planes(rng, T_true, tilt_spread=2)  # all ~facing camera
    with pytest.raises(ValueError, match="parallel"):
        ps.solve_cam_lidar(pc, pl)


def test_ransac_plane_finds_board_among_clutter():
    rng = np.random.default_rng(4)
    board = np.column_stack([rng.uniform(-0.4, 0.4, 300),
                             rng.uniform(-0.3, 0.3, 300),
                             rng.normal(0, 0.003, 300)]) + [0, 0, 2.0]
    clutter = rng.uniform(-1, 1, (150, 3)) + [0, 0, 2.0]
    fit = ps.ransac_plane(np.vstack([board, clutter]), dist_thresh=0.02,
                          min_inliers=100, rng=0)
    assert fit is not None
    n, d, c, mask = fit
    assert abs(n[2]) > 0.99 and mask.sum() >= 290


# ---------------------------------------------------------------------------
# hold clustering
# ---------------------------------------------------------------------------

def fake_det(rv_deg, xyz):
    T = np.eye(4)
    T[:3, :3] = Rot.from_euler("xyz", rv_deg, degrees=True).as_matrix()
    T[:3, 3] = xyz
    return {"T_cam_board": T}


def test_cluster_holds_splits_on_motion():
    dets = []
    t = 0.0
    for _ in range(20):                       # hold A: 2 s static
        dets.append((t, fake_det([10, 0, 0], [0, 0, 2]))); t += 0.1
    for i in range(10):                       # board moving
        dets.append((t, fake_det([14 + 4 * i, 0, 0], [0, 0, 2]))); t += 0.1
    for _ in range(20):                       # hold B: 2 s static elsewhere
        dets.append((t, fake_det([45, 5, 0], [0.5, 0, 2.5]))); t += 0.1
    holds = cluster_holds(dets, min_hold_s=1.5, max_gap_s=0.6,
                          max_rot_deg=1.5, max_trans_m=0.02)
    assert len(holds) == 2
    assert len(holds[0]) == 20 and len(holds[1]) == 20


# ---------------------------------------------------------------------------
# YAML merge
# ---------------------------------------------------------------------------

def test_update_preserves_other_calibrations(tmp_path):
    initial = {
        "reference_frame": "base_link",
        "transforms": [
            {"source_frame": "A", "parent": "base_link", "child": "lidar_hesai",
             "xyz": [0, 0, 0], "q_xyzw": [0, 0, 0, 1]},
            {"source_frame": "B", "parent": "base_link",
             "child": "lidar_livox_avia",
             "xyz": [0.1, 0, 0], "q_xyzw": [0, 0, 0, 1]},
            {"source_frame": "C", "parent": "base_link", "child": "camera_oak4d",
             "xyz": [0, 0.2, 0], "q_xyzw": [0, 0, 0, 1]},
        ],
    }
    init_p, out_p = tmp_path / "init.yaml", tmp_path / "cal.yaml"
    yaml.safe_dump(initial, open(init_p, "w"))

    # Stage 1 (lidar_lidar-like): calibrate the livox.
    T_livox = np.eye(4); T_livox[:3, 3] = [-0.12, 0.05, 0.27]
    io_cam.update_calibrated_extrinsics(out_p, init_p,
                                        {"lidar_livox_avia": T_livox},
                                        {"stage": "lidar_lidar"})
    # Stage 2 (this stage): calibrate a camera; livox must survive.
    T_cam = np.eye(4); T_cam[:3, 3] = [0.0, 0.21, 0.01]
    io_cam.update_calibrated_extrinsics(out_p, init_p,
                                        {"camera_oak4d": T_cam},
                                        {"stage": "camera_lidar"})

    doc = yaml.safe_load(open(out_p))
    by_child = {tf["child"]: tf for tf in doc["transforms"]}
    assert by_child["lidar_livox_avia"]["calibrated"] is True
    assert by_child["lidar_livox_avia"]["xyz"] == pytest.approx([-0.12, 0.05, 0.27])
    assert by_child["camera_oak4d"]["calibrated"] is True
    assert by_child["lidar_hesai"]["calibrated"] is False
    assert [p["stage"] for p in doc["provenance"]] == \
        ["lidar_lidar", "camera_lidar"]
