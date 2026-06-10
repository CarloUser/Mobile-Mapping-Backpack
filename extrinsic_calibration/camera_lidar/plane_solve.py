"""Plane-correspondence camera<->LiDAR extrinsic solve (Zhang & Pless 2004).

Inputs, one per static board hold:
  camera side:  plane (n_c, d_c), n_c . X + d_c = 0 in camera coords, n_c
                oriented toward the camera (board.py convention)
  lidar side:   board-patch points in lidar coords (RANSAC plane inliers),
                their fitted normal n_l (oriented toward the lidar) + centroid

Solves T_cam_lidar with X_cam = T_cam_lidar @ X_lidar:
  rotation     Kabsch on normal pairs        n_c ~ R n_l
  translation  least squares on              n_c . t = -(d_c + n_c . R c_l)
  refine       point-to-plane least squares over all patch points

Degenerate-motion gate: with all board normals near-parallel the rotation about
the normal and the in-plane translation are unobservable. `normal_spread` is the
smallest singular value of the stacked unit normals (0 = planar/parallel,
~0.5+ = well spread); solve_cam_lidar refuses below `min_spread`.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R


def fit_plane(points):
    """SVD plane fit -> (n, d, centroid, rms): n . X + d = 0, n toward origin."""
    pts = np.asarray(points, float)
    c = pts.mean(axis=0)
    _, S, Vt = np.linalg.svd(pts - c, full_matrices=False)
    n = Vt[2]
    if float(n @ c) > 0:                       # orient toward the sensor origin
        n = -n
    d = -float(n @ c)
    rms = float(S[2] / np.sqrt(len(pts))) if len(pts) > 3 else 0.0
    return n, d, c, rms


def ransac_plane(points, dist_thresh=0.02, iters=500, min_inliers=50, rng=None):
    """RANSAC plane on (N,3) points -> (n, d, centroid, inlier_mask) or None."""
    pts = np.asarray(points, float)
    if len(pts) < max(min_inliers, 3):
        return None
    rng = np.random.default_rng(rng)
    best_mask, best_count = None, -1
    for _ in range(iters):
        idx = rng.choice(len(pts), 3, replace=False)
        p0, p1, p2 = pts[idx]
        n = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        mask = np.abs((pts - p0) @ n) < dist_thresh
        cnt = int(mask.sum())
        if cnt > best_count:
            best_count, best_mask = cnt, mask
    if best_mask is None or best_count < min_inliers:
        return None
    # Two refit rounds on inliers tighten the plane.
    for _ in range(2):
        n, d, c, _ = fit_plane(pts[best_mask])
        best_mask = np.abs(pts @ n + d) < dist_thresh
        if best_mask.sum() < min_inliers:
            return None
    n, d, c, _ = fit_plane(pts[best_mask])
    return n, d, c, best_mask


def patch_extent(points, n):
    """In-plane extents (len_major, len_minor) of a planar patch — used to check
    the segmented patch is board-sized, not a wall."""
    pts = np.asarray(points, float)
    c = pts.mean(axis=0)
    q = pts - c - np.outer((pts - c) @ n, n)   # project into the plane
    _, S, _ = np.linalg.svd(q, full_matrices=False)
    half = 2.0 * S[:2] / np.sqrt(len(pts))     # ~2 sigma half-extents
    return float(2 * half[0]), float(2 * half[1])


def kabsch(vec_a, vec_b):
    """Rotation R with vec_a_i ~ R vec_b_i (rows are unit vectors)."""
    A = np.asarray(vec_a, float)
    B = np.asarray(vec_b, float)
    H = B.T @ A
    U, _, Vt = np.linalg.svd(H)
    S = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ S @ U.T


def normal_spread(normals):
    """Smallest singular value of stacked unit normals / sqrt(N), in [0, ~0.58].

    0 when all normals are coplanar-parallel; rises as tilts span 3D. Use as the
    observability gate for the rotation solve.
    """
    N = np.asarray(normals, float)
    return float(np.linalg.svd(N / np.sqrt(len(N)), compute_uv=False)[2])


def solve_cam_lidar(planes_cam, planes_lidar, min_spread=0.1):
    """Closed-form solve from plane correspondences.

    planes_cam:   list of (n_c (3,), d_c float)
    planes_lidar: list of (n_l (3,), c_l (3,))   c_l = patch centroid
    Returns (T_cam_lidar 4x4, info dict). Raises ValueError on degeneracy.
    """
    if len(planes_cam) != len(planes_lidar) or len(planes_cam) < 3:
        raise ValueError("Need >= 3 plane correspondences (>=6-10 recommended).")
    n_c = np.array([p[0] for p in planes_cam], float)
    d_c = np.array([p[1] for p in planes_cam], float)
    n_l = np.array([p[0] for p in planes_lidar], float)
    c_l = np.array([p[1] for p in planes_lidar], float)

    spread = normal_spread(n_c)
    if spread < min_spread:
        raise ValueError(
            f"Board normals nearly parallel (spread {spread:.3f} < {min_spread}):"
            f" rotation/translation unobservable. Re-record with stronger tilts.")

    Rcl = kabsch(n_c, n_l)
    # n_c . t = -(d_c + n_c . (R c_l)) per pose, solved in least squares.
    rhs = -(d_c + np.einsum("ij,ij->i", n_c, c_l @ Rcl.T))
    t, *_ = np.linalg.lstsq(n_c, rhs, rcond=None)

    T = np.eye(4)
    T[:3, :3] = Rcl
    T[:3, 3] = t
    ang = np.degrees(np.arccos(np.clip(
        np.einsum("ij,ij->i", n_c, n_l @ Rcl.T), -1, 1)))
    info = {"normal_spread": spread,
            "normal_residual_deg": {"median": float(np.median(ang)),
                                    "max": float(ang.max())}}
    return T, info


def refine_point_to_plane(T0, planes_cam, patches_lidar, max_pts_per_pose=300,
                          loss="soft_l1", f_scale=0.01):
    """Nonlinear refine: minimise point-to-plane distance of LiDAR patch points
    against the camera board planes over all holds.

    T0: 4x4 initial T_cam_lidar. patches_lidar: list of (Ni,3) inlier points in
    the lidar frame. Returns (T, result) with per-point residuals in `result`.
    """
    rng = np.random.default_rng(0)
    pts, nrm, off = [], [], []
    for (n_c, d_c), patch in zip(planes_cam, patches_lidar):
        p = np.asarray(patch, float)
        if len(p) > max_pts_per_pose:
            p = p[rng.choice(len(p), max_pts_per_pose, replace=False)]
        pts.append(p)
        nrm.append(np.repeat([n_c], len(p), axis=0))
        off.append(np.full(len(p), d_c))
    pts = np.concatenate(pts); nrm = np.concatenate(nrm)
    off = np.concatenate(off)

    R0 = T0[:3, :3]; t0 = T0[:3, 3]

    def residual(x):
        Rm = R.from_rotvec(x[:3]).as_matrix() @ R0
        t = t0 + x[3:]
        Xc = pts @ Rm.T + t
        return np.einsum("ij,ij->i", nrm, Xc) + off

    res = least_squares(residual, np.zeros(6), loss=loss, f_scale=f_scale)
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(res.x[:3]).as_matrix() @ R0
    T[:3, 3] = t0 + res.x[3:]
    return T, res
