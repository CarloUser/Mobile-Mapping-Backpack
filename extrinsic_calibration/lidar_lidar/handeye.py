"""Motion-based hand-eye extrinsic calibration: solve A_i X = X B_i for X = T_A_B.

Used to calibrate two LiDARs that do NOT share a field of view, from their
independently-estimated trajectories (e.g. KISS-ICP run per sensor).

Convention: X = T_A_B is the pose of frame B expressed in frame A. With A =
Hesai (reference) and B = Livox, X = T_hesai_livox. A_i and B_i are the relative
(body-frame) motions of each sensor between two timestamps; they satisfy
A_i X = X B_i regardless of each odometry's arbitrary world/map origin.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp


def se3(Rm, t):
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = t; return T


def inv(T):
    Ri = T[:3, :3].T
    return se3(Ri, -Ri @ T[:3, 3])


def interp_pose(times, Ts, query):
    """Interpolate trajectory (4x4 poses at `times`) at `query` times: SLERP + linear."""
    times = np.asarray(times, float)
    rots = R.from_matrix(np.array([T[:3, :3] for T in Ts]))
    slerp = Slerp(times, rots)
    pos = np.array([T[:3, 3] for T in Ts])
    q = np.clip(np.asarray(query, float), times[0], times[-1])
    Rq = slerp(q).as_matrix()
    Pq = np.vstack([np.interp(q, times, pos[:, k]) for k in range(3)]).T
    return [se3(Rq[i], Pq[i]) for i in range(len(q))]


def motion_pairs(WA, WB, min_angle_deg=5.0, stride=1):
    """Relative motions over `stride` frames; keep pairs with enough rotation.

    Larger stride => larger, better-conditioned motions (10 Hz consecutive
    frames barely rotate). Returns parallel lists A_i, B_i with A_i X = X B_i.
    """
    A, B = [], []
    for i in range(0, len(WA) - stride, stride):
        Ai = inv(WA[i]) @ WA[i + stride]
        Bi = inv(WB[i]) @ WB[i + stride]
        ang = np.linalg.norm(R.from_matrix(Ai[:3, :3]).as_rotvec())
        if np.degrees(ang) >= min_angle_deg:
            A.append(Ai); B.append(Bi)
    return A, B


def solve_rotation(A, B):
    """A_i X = X B_i implies axis(A_i) = R_X axis(B_i). Solve via Kabsch/Wahba."""
    M = np.zeros((3, 3))
    for Ai, Bi in zip(A, B):
        a = R.from_matrix(Ai[:3, :3]).as_rotvec()
        b = R.from_matrix(Bi[:3, :3]).as_rotvec()
        M += np.outer(a, b)                 # sum alpha_i beta_i^T
    U, _, Vt = np.linalg.svd(M)
    d = np.sign(np.linalg.det(U @ Vt))
    return U @ np.diag([1.0, 1.0, d]) @ Vt


def solve_translation(A, B, Rx):
    """Least squares: (R_Ai - I) t_X = Rx t_Bi - t_Ai."""
    C, d = [], []
    for Ai, Bi in zip(A, B):
        C.append(Ai[:3, :3] - np.eye(3))
        d.append(Rx @ Bi[:3, 3] - Ai[:3, 3])
    t, *_ = np.linalg.lstsq(np.vstack(C), np.concatenate(d), rcond=None)
    return t


def rotation_observability(A):
    """Normalised spread of rotation axes: smallest/largest singular value, in [0,1].

    ~0 means every motion rotated about (nearly) the same axis -> the relative
    roll/pitch (or yaw) is NOT observable. Flat-ground, yaw-only walking is the
    classic failure case. The ratio is independent of the number of pairs; aim
    for it to be well above ~0.1 (a few tenths is healthy 3-axis excitation).
    """
    axes = np.array([R.from_matrix(Ai[:3, :3]).as_rotvec() for Ai in A])
    axes = np.array([v / (np.linalg.norm(v) + 1e-12) for v in axes])
    s = np.linalg.svd(axes, compute_uv=False)
    return float(s[-1] / (s[0] + 1e-12))


def solve_handeye(A, B):
    Rx = solve_rotation(A, B)
    return se3(Rx, solve_translation(A, B, Rx))


def solve_handeye_robust(A, B, n_iter=3, keep=0.85):
    """Solve, then iteratively drop the worst-residual pairs and re-solve."""
    A, B = list(A), list(B)
    X = solve_handeye(A, B)
    for _ in range(n_iter):
        rot, trn = residuals(A, B, X)
        score = rot / max(np.median(rot), 1e-6) + trn / max(np.median(trn), 1e-6)
        order = np.argsort(score)
        n = max(10, int(len(A) * keep))
        idx = sorted(order[:n])
        A = [A[i] for i in idx]; B = [B[i] for i in idx]
        X = solve_handeye(A, B)
    return X, A, B


def residuals(A, B, X):
    """Per-pair rotation(deg) and translation(m) residual of A_i X = X B_i."""
    rot, trn = [], []
    for Ai, Bi in zip(A, B):
        E = inv(Ai @ X) @ (X @ Bi)
        rot.append(np.degrees(np.linalg.norm(R.from_matrix(E[:3, :3]).as_rotvec())))
        trn.append(np.linalg.norm(E[:3, 3]))
    return np.array(rot), np.array(trn)
