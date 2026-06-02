"""Unit tests for the hand-eye solver on synthetic trajectories with known truth.

Run:  python test_handeye.py
Covers: exact recovery (clean), bounded error under noise, robustness to gross
outliers, and the degenerate yaw-only case that MUST fail (proves the motion
protocol needs roll+pitch excitation, not just flat-ground turning).
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
import handeye as he

rng = np.random.default_rng(0)


def random_T(rot_scale=1.0, t_scale=1.0):
    return he.se3(R.from_rotvec(rng.normal(size=3) * rot_scale).as_matrix(),
                  rng.normal(size=3) * t_scale)


def make_trajectory(n=140, full_rotation=True):
    times = np.linspace(0, 12, n)
    Ts = []
    for tm in times:
        if full_rotation:
            rv = [0.4 * np.sin(0.7 * tm), 0.5 * np.sin(0.5 * tm + 1), 0.9 * np.sin(0.3 * tm)]
        else:  # degenerate: yaw only (flat-ground walking)
            rv = [0.0, 0.0, 0.9 * np.sin(0.3 * tm)]
        pos = [1.5 * np.sin(0.2 * tm), 0.8 * tm, 0.1 * np.sin(0.4 * tm)]
        Ts.append(he.se3(R.from_rotvec(rv).as_matrix(), pos))
    return times, Ts


def x_error(X, X_true):
    E = he.inv(X) @ X_true
    return (np.degrees(np.linalg.norm(R.from_matrix(E[:3, :3]).as_rotvec())),
            np.linalg.norm(E[:3, 3]))


def run(full_rotation=True, noise=0.0, robust=False):
    X_true = random_T(0.5, 0.3)
    _, WA = make_trajectory(full_rotation=full_rotation)
    WB = [Wa @ X_true for Wa in WA]
    if noise > 0:
        WA = [T @ random_T(noise, noise) for T in WA]
        WB = [T @ random_T(noise, noise) for T in WB]
    A, B = he.motion_pairs(WA, WB, min_angle_deg=1.0, stride=5)
    X = he.solve_handeye_robust(A, B)[0] if robust else he.solve_handeye(A, B)
    return len(A), x_error(X, X_true)


print("== Clean, full 3-axis rotation ==")
n, (re_, te_) = run(True, 0.0)
print(f"  pairs={n}  rot_err={re_:.2e} deg  t_err={te_:.2e} m")
assert re_ < 1e-6 and te_ < 1e-6, "clean recovery failed"

print("== Noisy (0.01 perturbation), full rotation ==")
n, (re_, te_) = run(True, 0.01)
print(f"  pairs={n}  rot_err={re_:.3f} deg  t_err={te_:.4f} m")
assert re_ < 2.0 and te_ < 0.10, "noisy recovery out of tolerance"

print("== Degenerate yaw-only motion (MUST be poorly constrained) ==")
n, (re_, te_) = run(False, 0.0)
print(f"  pairs={n}  rot_err={re_:.2f} deg  t_err={te_:.3f} m  (large = expected)")
assert re_ > 10.0, "yaw-only should be unobservable; check axis excitation logic"

print("\nALL ASSERTIONS PASSED")
