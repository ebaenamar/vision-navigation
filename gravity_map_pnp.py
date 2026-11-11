"""
Soft-gravity MAP PnP with a pose covariance (review revision, "A + B").

Problem.  For a nadir camera over near-planar terrain, PnP is close to the
affine regime, where tilt and horizontal position are nearly interchangeable.
The free 6-DOF solution then produces confident false fixes (Exp 4b/4c).
Fixing the vertical (4-DOF) removes them, but it trusts the IMU tilt exactly;
with a 0.5 deg MEMS tilt it loses accuracy against 6-DOF on good fixes.

Solver A (MAP).  Minimise
    sum_i || (x_i - pi(R, C; X_i)) / s_px ||^2  +  || t(R, R_imu) / sigma_tilt ||^2
over the full pose (R, C), where t(.) is the two horizontal components of the
world-frame rotation from the IMU attitude to R (heading is left free) and
sigma_tilt is the IMU's own tilt uncertainty.  Hypotheses come from the
gravity-constrained 4-DOF solver and from free 6-DOF EPnP; the one with the
lower MAP cost after refinement is kept.  Inliers are re-selected (8 px)
between refinements and s_px is re-estimated from the inlier residuals.

Covariance B.  Sigma = (J^T J)^-1 of the whitened residuals at the optimum;
the horizontal position block is reported.  Because correspondences are not
independent (and the map has its own errors), this raw covariance is expected
to be optimistic, so a single inflation factor is calibrated on held-out data
(see map_pnp_eval.py).

sigma_tilt = None gives the plain 6-DOF refinement with the same covariance,
for the paper's solver.
"""

import math

import numpy as np
from scipy.optimize import least_squares

from map_match_attitude import rot_exp, rot_log, solve_4dof, solve_6dof


def project(R, C, X, K):
    Xc = (R @ (X - C).T).T
    uv = (K @ Xc.T).T
    return uv[:, :2] / uv[:, 2:3], Xc[:, 2]


def _residuals(p, R0, X, x, K, s_px, R_imu, sig_t):
    R = rot_exp(p[:3]) @ R0
    C = p[3:]
    uv, z = project(R, C, X, K)
    r = ((uv - x) / s_px).ravel()
    if R_imu is not None:
        dw = rot_log(R_imu.T @ R)            # world-frame rotation IMU -> estimate
        r = np.concatenate([r, dw[:2] / sig_t])
    return r


def refine(R0, C0, X, x, K, R_imu=None, sig_t=None, reproj_px=8.0, min_inliers=12, rounds=3):
    """Iterated MAP refinement with inlier re-selection. Returns dict or None."""
    R, C = R0, np.asarray(C0, float)
    s_px = 2.0
    inl = None
    for _ in range(rounds):
        uv, z = project(R, C, X, K)
        e = np.linalg.norm(uv - x, axis=1)
        new = np.flatnonzero((e < reproj_px) & (z > 0))
        if len(new) < min_inliers:
            return None
        inl = new
        res = least_squares(_residuals, np.r_[np.zeros(3), C], method="lm",
                            args=(R, X[inl], x[inl], K, s_px, R_imu, sig_t))
        R = rot_exp(res.x[:3]) @ R
        C = res.x[3:]
        uv, _ = project(R, C, X[inl], K)
        s_px = max(0.3, float(np.sqrt(np.mean((uv - x[inl]) ** 2))))   # per-coordinate RMS
    # final whitened Jacobian at the optimum (with the final s_px)
    res = least_squares(_residuals, np.r_[np.zeros(3), C], method="lm", max_nfev=1,
                        args=(R, X[inl], x[inl], K, s_px, R_imu, sig_t))
    J = res.jac
    try:
        cov = np.linalg.inv(J.T @ J)
    except np.linalg.LinAlgError:
        return None
    # MSAC score over ALL correspondences (2 px scale, 8 px truncation) plus the
    # tilt prior, so that hypotheses are compared on the same footing.
    uv, z = project(R, C, X, K)
    e2 = np.sum((uv - x) ** 2, axis=1)
    e2[z <= 0] = reproj_px ** 2
    score = float(np.sum(np.minimum(e2, reproj_px ** 2)) / 4.0)
    if R_imu is not None:
        score += float(np.sum((rot_log(R_imu.T @ R)[:2] / sig_t) ** 2))
    return {"R": R, "C": C, "inliers": int(len(inl)), "cov_h": cov[3:5, 3:5],
            "s_px": s_px, "score": score}


def solve_map(X, x, K, R_imu, sigma_tilt_rad, rng=None):
    """Soft-gravity MAP solver (A). Hypotheses: 4-DOF (gravity) and 6-DOF."""
    cands = []
    h4 = solve_4dof(X, x, K, R_imu, rng=rng)
    if h4 is not None:
        cands.append(h4)
    h6 = solve_6dof(X, x, K)
    if h6 is not None:
        cands.append(h6)
    best = None
    for h in cands:
        sol = refine(h["R"], h["C"], X, x, K, R_imu, sigma_tilt_rad)
        if sol is None:
            continue
        if best is None or sol["score"] < best[0]:
            best = (sol["score"], sol)
    return None if best is None else best[1]


def solve_6dof_cov(X, x, K):
    """The paper's 6-DOF solver, refined the same way to obtain a covariance."""
    h = solve_6dof(X, x, K)
    return None if h is None else refine(h["R"], h["C"], X, x, K)


def solve_4dof_cov(X, x, K, R_imu, rng=None):
    """Hard gravity (4-DOF) solver with a covariance: the tilt prior is made
    very tight (0.001 deg) so the refinement keeps the vertical fixed."""
    h = solve_4dof(X, x, K, R_imu, rng=rng)
    return None if h is None else refine(h["R"], h["C"], X, x, K, R_imu, math.radians(0.001))
