"""
Attitude-aided map matching on VPAIR (review revision, "option 1").

Question: does an attitude estimate from another layer of the architecture
(star tracker + IMU) improve visual map matching?  Two mechanisms are tested:

  1. matching: with the heading known, the query is matched at ONE in-plane
     rotation instead of the paper's search over eight;
  2. solving: with the attitude known, the camera POSITION is solved alone
     (3 unknowns, 2-point RANSAC) instead of the full 6-DOF PnP.

Truth attitude comes from VPAIR's GNSS/INS poses (roll, pitch, yaw, rad,
relative to the nadir north-up reference views).  Its mapping onto camera
axes is identified from the 6-DOF PnP solutions (see `convention`).  Attitude
errors are simulated at several levels: oracle, star tracker (30 arcsec),
calibrated INS (0.1 deg) and MEMS-only (0.5 deg).

Usage:
    python map_match_attitude.py convention --n 60
    python map_match_attitude.py run [--offsets 0,1,2,3,4] [--max-queries N]
"""

import argparse
import json
import math
import os
import time

import cv2
import numpy as np

from map_match import VPAIR, LightGlueMatcher, LocalFrame, calibrate, ref_pixels_to_enu, world_from_cam

DEG = math.pi / 180


# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------

def rot_log(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = math.acos(c)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return 0.5 * w if th < 1e-9 else w * th / (2 * math.sin(th))


def rot_exp(w):
    R, _ = cv2.Rodrigues(np.asarray(w, dtype=np.float64).reshape(3, 1))
    return R


def euler_zyx(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# ---------------------------------------------------------------------------
# Solvers
# ---------------------------------------------------------------------------

def K_of(calib):
    return np.array([[calib["f"], 0, calib["cx"]], [0, calib["f"], calib["cy"]], [0, 0, 1.0]])


def solve_6dof(pts3d, pq, K, reproj_px=8.0, min_inliers=12):
    """The paper's solver: EPnP + RANSAC, then LM refinement."""
    ok, rvec, tvec, inl = cv2.solvePnPRansac(pts3d, pq, K, None, iterationsCount=3000,
                                             reprojectionError=reproj_px, confidence=0.999,
                                             flags=cv2.SOLVEPNP_EPNP)
    if not ok or inl is None or len(inl) < min_inliers:
        return None
    idx = inl.ravel()
    rvec, tvec = cv2.solvePnPRefineLM(pts3d[idx], pq[idx], K, None, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    return {"R": R, "C": (-R.T @ tvec).ravel(), "inliers": len(idx)}


def _center_ls(d, X):
    """Least-squares camera centre C from unit rays d_i (world) through points
    X_i, minimising the perpendicular distances: sum (I - d d^T)(X_i - C) = 0."""
    M = np.eye(3)[None] - d[:, :, None] * d[:, None, :]
    return np.linalg.solve(M.sum(axis=0), np.einsum("nij,nj->i", M, X))


def solve_3dof(pts3d, pq, K, R, reproj_px=8.0, min_inliers=12, iters=500, rng=None):
    """Position-only solver with the attitude R (world -> camera) given.
    2-point RANSAC on the ray constraint, inliers by reprojection error,
    then Gauss-Newton refinement of C on the inliers."""
    rng = rng or np.random.default_rng(0)
    n = len(pq)
    if n < min_inliers:
        return None
    Kinv = np.linalg.inv(K)
    rays = (R.T @ (Kinv @ np.c_[pq, np.ones(n)].T)).T
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)

    def reproj(C):
        Xc = (R @ (pts3d - C).T).T
        with np.errstate(divide="ignore", invalid="ignore"):
            uv = (K @ Xc.T).T
            uv = uv[:, :2] / uv[:, 2:3]
        e = np.linalg.norm(uv - pq, axis=1)
        e[Xc[:, 2] <= 0] = np.inf
        return e

    best = None
    for _ in range(iters):
        i, j = rng.choice(n, 2, replace=False)
        try:
            C = _center_ls(rays[[i, j]], pts3d[[i, j]])
        except np.linalg.LinAlgError:
            continue
        inl = np.flatnonzero(reproj(C) < reproj_px)
        if best is None or len(inl) > len(best):
            best = inl
    if best is None or len(best) < min_inliers:
        return None
    C = _center_ls(rays[best], pts3d[best])
    for _ in range(3):                                  # re-select inliers and re-fit
        inl = np.flatnonzero(reproj(C) < reproj_px)
        if len(inl) < min_inliers:
            return None
        C = _center_ls(rays[inl], pts3d[inl])
    return {"R": R, "C": C, "inliers": len(inl)}


def rot_z(psi):
    c, s_ = math.cos(psi), math.sin(psi)
    return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])


def solve_4dof(pts3d, pq, K, R_tilt, reproj_px=8.0, min_inliers=12, rng=None,
               coarse_deg=10.0, iters=60):
    """Gravity-only solver: the vertical (roll, pitch) is known from an IMU but
    the heading is not.  R(psi) = R_tilt @ Rz_world(psi); psi is searched on a
    coarse grid (3-DOF RANSAC at each value), then refined on a fine grid."""
    rng = rng or np.random.default_rng(0)
    best = None
    start = rng.uniform(0, coarse_deg)          # no grid point at the true heading
    for psi in np.radians(np.arange(start, 360, coarse_deg)):
        sol = solve_3dof(pts3d, pq, K, R_tilt @ rot_z(psi), reproj_px, min_inliers, iters, rng)
        if sol is not None and (best is None or sol["inliers"] > best[1]["inliers"]):
            best = (psi, sol)
    if best is None:
        return None
    psi0 = best[0]
    for step, half in ((1.0, coarse_deg), (0.1, 1.0)):
        cands = []
        for psi in psi0 + np.radians(np.arange(-half, half + 1e-9, step)):
            sol = solve_3dof(pts3d, pq, K, R_tilt @ rot_z(psi), reproj_px, min_inliers, iters, rng)
            if sol is not None:
                cands.append((sol["inliers"], psi, sol))
        if not cands:
            return None
        _, psi0, sol = max(cands, key=lambda c: c[0])
    return sol


def attitude_gate(sol, R_known, gate_deg):
    """Accept a 6-DOF solution only if its rotation agrees with the known attitude."""
    if sol is None:
        return None
    return sol if math.degrees(np.linalg.norm(rot_log(sol["R"] @ R_known.T))) <= gate_deg else None


# ---------------------------------------------------------------------------
# Shared per-query preparation
# ---------------------------------------------------------------------------

def matches_3d(data, calib, rid, pq, pr):
    pts3d, valid = ref_pixels_to_enu(pr, data.ref_depth(rid), calib)
    return pts3d[valid].astype(np.float64), pq[valid].astype(np.float64)


def truth_centre(data, qid, rid):
    return LocalFrame(data.refs[rid]["ecef"]).to_enu(data.queries[qid]["ecef"])


def ref_R_world_to_cam(calib):
    return world_from_cam(calib).T


def rpy(data, qid):
    q = data.queries[qid]
    return float(q["roll"]), float(q["pitch"]), float(q["yaw"])


# ---------------------------------------------------------------------------
# Convention identification
# ---------------------------------------------------------------------------

def signed_perms():
    import itertools
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            P = np.zeros((3, 3))
            for i, (j, s) in enumerate(zip(perm, signs)):
                P[i, j] = s
            if np.linalg.det(P) > 0:
                out.append(P)
    return out


def candidate_R_rel(P, signs, transpose, r, p, y):
    E = euler_zyx(signs[0] * r, signs[1] * p, signs[2] * y)
    E = E.T if transpose else E
    return P @ E @ P.T                       # cam_ref -> cam_query


def identify_convention(data, calib, matcher, ids):
    """Match each query against its own reference view, solve 6-DOF, and pick
    the mapping of VPAIR (roll, pitch, yaw) onto camera axes that best
    reproduces the PnP rotations (median angular disagreement)."""
    import itertools
    K, R_ref = K_of(calib), ref_R_world_to_cam(calib)
    rels, angs = [], []
    for qid in ids:
        pq, pr = matcher.match(data.query_image(qid), data.ref_image(qid))
        if len(pq) < 12:
            continue
        X, x = matches_3d(data, calib, qid, pq, pr)
        sol = solve_6dof(X, x, K)
        if sol is None or sol["inliers"] < 100:
            continue
        rels.append(sol["R"] @ R_ref.T)
        angs.append(rpy(data, qid))
    best = None
    for P in signed_perms():
        for signs in itertools.product((1, -1), repeat=3):
            for tr in (False, True):
                errs = [np.linalg.norm(rot_log(candidate_R_rel(P, signs, tr, *a).T @ Rr))
                        for a, Rr in zip(angs, rels)]
                med = float(np.median(errs))
                if best is None or med < best[0]:
                    best = (med, P, signs, tr, errs)
    med, P, signs, tr, errs = best
    return {"P": P.tolist(), "signs": list(signs), "transpose": tr, "n": len(rels),
            "median_deg": math.degrees(med), "p95_deg": math.degrees(float(np.percentile(errs, 95)))}


# ---------------------------------------------------------------------------
# Main experiment
# ---------------------------------------------------------------------------

LEVELS = [("oracle (INS truth, boresight-calibrated)", 0.0),
          ("star tracker + gyro (30 arcsec)", 30 / 3600),
          ("calibrated INS (0.1 deg)", 0.1),
          ("MEMS-only (0.5 deg)", 0.5)]


def ins_R_rel(conv, data, qid):
    P = np.array(conv["P"])
    return candidate_R_rel(P, conv["signs"], conv["transpose"], *rpy(data, qid))


def match_cached(cache, key, fn):
    if key not in cache:
        t0 = time.perf_counter()
        pq, pr = fn()
        cache[key] = (pq, pr, time.perf_counter() - t0)
    return cache[key]


def run(a, data, calib, matcher):
    conv = json.load(open(os.path.join(a.out, "exp4b_convention.json")))
    K, R_ref = K_of(calib), ref_R_world_to_cam(calib)
    offsets = [int(x) for x in a.offsets.split(",")]
    ids = data.ids[: a.max_queries] if a.max_queries else data.ids
    rng = np.random.default_rng(20261111)

    # In-plane angle (deg) that de-rotates a query to north-up, from attitude.
    def heading_angle(R_rel, sign):
        return sign * math.degrees(math.atan2(R_rel[1, 0], R_rel[0, 0]))

    # Sign of the de-rotation angle: pick the one that matches best on a few queries.
    votes = {1: 0, -1: 0}
    big_yaw = sorted(ids, key=lambda q: -abs(rpy(data, q)[2]))[:8]   # sign only matters at large yaw
    for qid in big_yaw:
        Rr = ins_R_rel(conv, data, qid)
        for sgn in (1, -1):
            matcher.angles = [heading_angle(Rr, sgn)]
            pq, _ = matcher.match(data.query_image(qid), data.ref_image(qid))
            votes[sgn] += len(pq)
    sign = max(votes, key=votes.get)
    print("heading sign", sign, votes, flush=True)

    # Pass 1: matches.  "search8" = paper (8 rotations); "heading" = one rotation
    # from the attitude perturbed at the WORST level (0.5 deg), so matching
    # gets no oracle advantage.
    cache_path = os.path.join(a.out, "exp4b_matches.npz")
    cache = {}
    if os.path.exists(cache_path):
        z = np.load(cache_path)
        for k in z.files:
            if k.endswith("|pq"):
                base = k[:-3]
                qid, off, mode = base.split("|")
                cache[(qid, int(off), mode)] = (z[base + "|pq"], z[base + "|pr"], float(z[base + "|t"]))
        print(f"  loaded {len(cache)} cached match sets", flush=True)
    eight = [i * 45.0 for i in range(8)]
    att_noise = {}
    for i, qid in enumerate(ids):
        Rr = ins_R_rel(conv, data, qid)
        for off in offsets:
            rid = data.neighbour(qid, off)
            if rid is None:
                continue
            iq, ir = data.query_image(qid), data.ref_image(rid)
            matcher.angles = eight
            match_cached(cache, (qid, off, "search8"), lambda: matcher.match(iq, ir))
            w = rng.normal(0, 0.5 * DEG, 3)
            matcher.angles = [heading_angle(rot_exp(w) @ Rr, sign)]
            match_cached(cache, (qid, off, "heading"), lambda: matcher.match(iq, ir))
        for name, sd in LEVELS:
            att_noise[(qid, name)] = rng.normal(0, sd * DEG, 3)
        if (i + 1) % 20 == 0:
            print(f"  matched {i + 1}/{len(ids)}", flush=True)
    out = {}
    for (qid, off, mode), (pq, pr, t) in cache.items():
        b = f"{qid}|{off}|{mode}"
        out[b + "|pq"], out[b + "|pr"], out[b + "|t"] = pq, pr, np.array(t)
    np.savez_compressed(cache_path, **out)

    # Boresight (camera-INS misalignment), two-fold: estimated on one half of
    # the queries from 6-DOF solutions against their own reference view, applied
    # to the other half.
    folds = [ids[0::2], ids[1::2]]
    B = {}
    for k in (0, 1):
        ws = []
        for qid in folds[k]:
            pq, pr, _ = cache[(qid, 0, "search8")]
            if len(pq) < 12:
                continue
            X, x = matches_3d(data, calib, qid, pq, pr)
            sol = solve_6dof(X, x, K)
            if sol is not None and sol["inliers"] >= 100:
                ws.append(rot_log(sol["R"] @ R_ref.T @ ins_R_rel(conv, data, qid).T))
        Bk = rot_exp(np.mean(ws, axis=0))
        for qid in folds[1 - k]:
            B[qid] = Bk
        print(f"  boresight from fold {k}: {math.degrees(np.linalg.norm(np.mean(ws, axis=0))):.3f} deg "
              f"(n={len(ws)})", flush=True)

    # Pass 2: solve.
    rows = []
    for qid in ids:
        Rr = B[qid] @ ins_R_rel(conv, data, qid)
        for off in offsets:
            rid = data.neighbour(qid, off)
            if rid is None:
                continue
            truth = truth_centre(data, qid, rid)
            for mode in ("search8", "heading"):
                pq, pr, t_match = cache[(qid, off, mode)]
                X, x = matches_3d(data, calib, rid, pq, pr) if len(pq) else (np.zeros((0, 3)), np.zeros((0, 2)))
                solvers = [("6-DOF PnP (paper)", "6dof", None)]
                solvers += [(f"3-DOF, {n}", "3dof", n) for n, _ in LEVELS]
                if mode == "search8":
                    solvers += [(f"6-DOF + attitude gate, {n}", "gate", n) for n, _ in LEVELS]
                    solvers += [(f"4-DOF gravity only, {n}", "4dof", n) for n, sd in LEVELS if sd >= 0.1]
                sol6 = solve_6dof(X, x, K) if len(x) >= 12 else None
                for sname, kind, lvl in solvers:
                    R = rot_exp(att_noise[(qid, lvl)]) @ Rr @ R_ref if lvl else None
                    if len(x) < 12:
                        sol = None
                    elif kind == "6dof":
                        sol = sol6
                    elif kind == "3dof":
                        sol = solve_3dof(X, x, K, R, rng=rng)
                    elif kind == "gate":
                        sd = dict(LEVELS)[lvl]
                        sol = attitude_gate(sol6, R, max(1.0, 6 * sd))
                    else:
                        sol = solve_4dof(X, x, K, R, rng=rng)
                    r = {"qid": qid, "offset": off, "match": mode, "solver": sname,
                         "t_match": t_match, "landcover": data.queries[qid].get("landcover", ""),
                         "prior_m": float(np.hypot(*truth[:2]))}
                    if sol is None:
                        r.update(success=False)
                    else:
                        e = sol["C"] - truth
                        r.update(success=True, err_m=float(np.hypot(e[0], e[1])), inliers=sol["inliers"])
                    rows.append(r)
    with open(os.path.join(a.out, "exp4b_rows.json"), "w") as f:
        json.dump(rows, f)
    report(rows, a.out)


def report(rows, out):
    import collections
    g = collections.defaultdict(list)
    for r in rows:
        g[(r["match"], r["solver"], r["offset"])].append(r)
    L = ["# Experiment 4b: attitude-aided map matching on VPAIR (199 real queries)", "",
         "Success = >= 12 inliers. Errors over successful fixes; 'within 25 m' and 'false fix (> 25 m)' over all queries.", "",
         "| Matching | Solver | Offset (frames) | Mean prior error (m) | Success (%) | Median (m) [95% CI] | 95th pct (m) [95% CI] | Within 25 m (%) | False fixes > 25 m (%) [95% CI] | Median match time (s) |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    brng = np.random.default_rng(0)

    def boot(rs, fn, B=1000):
        vals = []
        for _ in range(B):
            smp = [rs[i] for i in brng.integers(0, len(rs), len(rs))]
            v = fn(smp)
            if v is not None:
                vals.append(v)
        return np.percentile(vals, [2.5, 97.5]) if vals else (np.nan, np.nan)

    def f_med(rs):
        e = [r["err_m"] for r in rs if r["success"]]
        return float(np.median(e)) if e else None

    def f_p95(rs):
        e = [r["err_m"] for r in rs if r["success"]]
        return float(np.percentile(e, 95)) if e else None

    def f_false(rs):
        return 100 * sum(1 for r in rs if r["success"] and r["err_m"] > 25) / len(rs)
    for (m, s, off), rs in sorted(g.items(), key=lambda kv: (kv[0][2], kv[0][0], kv[0][1])):
        n = len(rs)
        ok = [r for r in rs if r["success"]]
        e = np.array([r["err_m"] for r in ok])
        cm, cp, cf = boot(rs, f_med), boot(rs, f_p95), boot(rs, f_false)
        L.append(f"| {m} | {s} | {off} | {np.mean([r['prior_m'] for r in rs]):.0f} | {100 * len(ok) / n:.1f} | "
                 f"{np.median(e):.1f} [{cm[0]:.1f}–{cm[1]:.1f}] | {np.percentile(e, 95):.1f} [{cp[0]:.1f}–{cp[1]:.1f}] | "
                 f"{100 * np.sum(e <= 25) / n:.1f} | {100 * np.sum(e > 25) / n:.1f} [{cf[0]:.1f}–{cf[1]:.1f}] | "
                 f"{np.median([r['t_match'] for r in rs]):.2f} |"
                 if len(e) else f"| {m} | {s} | {off} | – | 0.0 | – | – | 0.0 | 0.0 | – |")
    with open(os.path.join(out, "exp4b.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("\n".join(L))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=["convention", "run"])
    ap.add_argument("--data", default="vpair_sample")
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--offsets", default="0,1,2,3,4")
    ap.add_argument("--max-queries", type=int, default=None)
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    data = VPAIR(a.data)
    calib = calibrate(data)
    matcher = LightGlueMatcher()
    if a.stage == "convention":
        conv = identify_convention(data, calib, matcher, data.ids[: a.n])
        print(json.dumps(conv, indent=1))
        with open(os.path.join(a.out, "exp4b_convention.json"), "w") as f:
            json.dump(conv, f, indent=1)
    else:
        run(a, data, calib, matcher)


if __name__ == "__main__":
    main()
