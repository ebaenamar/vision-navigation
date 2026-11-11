"""
Evaluation of the soft-gravity MAP PnP (A) and of its calibrated covariance /
protection levels (B) on real data, against the paper's 6-DOF PnP and the
hard-gravity 4-DOF solver.  All solvers see identical cached correspondences.

  python map_pnp_eval.py solve vpair      (uses results/exp4b_matches.npz)
  python map_pnp_eval.py solve visloc     (uses results/cache_visloc/)
  python map_pnp_eval.py report           (accuracy, coverage, protection levels,
                                           within- and cross-dataset)

Attitude: VPAIR uses its INS attitude (boresight-calibrated, two-fold) plus
simulated IMU errors at 30 arcsec, 0.1, 0.5, 1 and 2 deg; the MAP solver is told
the matching sigma.  UAV-VisLoc uses the real recorded onboard attitude only
(about 1.5 deg), and the MAP solver is told 1.5 deg.
"""

import json
import math
import os
import sys

import numpy as np

from gravity_map_pnp import refine
from map_match_attitude import (DEG, K_of, candidate_R_rel, matches_3d, ref_R_world_to_cam, rot_exp,
                                rot_log, rpy, solve_4dof, solve_6dof, truth_centre)

OUT = "results/exp4d"
CHI2_95, CHI2_99 = 5.991, 9.210
LEVELS_VPAIR = [("30 arcsec", 30 / 3600), ("0.1 deg", 0.1), ("0.5 deg", 0.5), ("1 deg", 1.0), ("2 deg", 2.0)]


def solve_all(X, x, K, R_imu, sig_rad, rng, h6=None):
    out = {}
    h4 = solve_4dof(X, x, K, R_imu, rng=rng) if len(x) >= 12 else None
    out["4-DOF hard gravity"] = refine(h4["R"], h4["C"], X, x, K, R_imu, math.radians(0.001)) if h4 else None
    cands = [refine(h["R"], h["C"], X, x, K, R_imu, sig_rad) for h in (h4, h6) if h is not None]
    cands = [c for c in cands if c is not None]
    out["MAP soft gravity"] = min(cands, key=lambda c: c["score"]) if cands else None
    return out


def pack(sol, truth_h):
    if sol is None:
        return {"success": False}
    e = sol["C"][:2] - truth_h
    return {"success": True, "e": e.tolist(), "cov": sol["cov_h"].tolist(), "inliers": sol["inliers"]}


def solve_vpair():
    from map_match import VPAIR, calibrate
    data = VPAIR("vpair_sample")
    calib = calibrate(data)
    K, R_ref = K_of(calib), ref_R_world_to_cam(calib)
    conv = json.load(open("results/exp4b_convention.json"))
    P = np.array(conv["P"])
    ins = lambda q: candidate_R_rel(P, conv["signs"], conv["transpose"], *rpy(data, q))
    z = np.load("results/exp4b_matches.npz")
    get = lambda q, o: (z[f"{q}|{o}|search8|pq"], z[f"{q}|{o}|search8|pr"])
    ids = data.ids
    # two-fold boresight, as in Exp 4b
    B = {}
    folds = [ids[0::2], ids[1::2]]
    for k in (0, 1):
        ws = []
        for q in folds[k]:
            pq, pr = get(q, 0)
            if len(pq) < 12:
                continue
            X, x = matches_3d(data, calib, q, pq, pr)
            s = solve_6dof(X, x, K)
            if s is not None and s["inliers"] >= 100:
                ws.append(rot_log(s["R"] @ R_ref.T @ ins(q).T))
        for q in folds[1 - k]:
            B[q] = rot_exp(np.mean(ws, axis=0))
    rng = np.random.default_rng(4)
    rows = []
    for n, q in enumerate(ids):
        R_true = B[q] @ ins(q) @ R_ref
        for off in range(5):
            rid = data.neighbour(q, off)
            if rid is None:
                continue
            pq, pr = get(q, off)
            truth = truth_centre(data, q, rid)[:2]
            if len(pq) < 12:
                X, x = np.zeros((0, 3)), np.zeros((0, 2))
            else:
                X, x = matches_3d(data, calib, rid, pq, pr)
            h6 = solve_6dof(X, x, K) if len(x) >= 12 else None
            s6 = refine(h6["R"], h6["C"], X, x, K) if h6 else None
            base = {"dataset": "vpair", "qid": q, "offset": off}
            rows.append({**base, "solver": "6-DOF PnP (paper)", "level": "-", **pack(s6, truth)})
            for name, sd in LEVELS_VPAIR:
                R_imu = rot_exp(rng.normal(0, sd * DEG, 3)) @ R_true
                res = solve_all(X, x, K, R_imu, sd * DEG, rng, h6) if len(x) >= 12 else \
                    {"4-DOF hard gravity": None, "MAP soft gravity": None}
                for sname, sol in res.items():
                    rows.append({**base, "solver": sname, "level": name, **pack(sol, truth)})
        if (n + 1) % 20 == 0:
            print(f"  vpair {n + 1}/{len(ids)}", flush=True)
            json.dump(rows, open(os.path.join(OUT, "rows_vpair.json"), "w"))
    json.dump(rows, open(os.path.join(OUT, "rows_vpair.json"), "w"))


def solve_visloc():
    import visloc_attitude as V
    d = V.VisLoc()
    cal = json.load(open("results/cache_visloc/calib.json"))
    P, B = np.array(cal["P"]), np.array(cal["B"])
    z = np.load("results/cache_visloc/matches.npz")
    keys = sorted({k.rsplit("|", 1)[0] for k in z.files}, key=lambda s: tuple(map(int, s.split("|"))))
    rng = np.random.default_rng(5)
    sig = math.radians(1.5)
    rows = []
    for n, b in enumerate(keys):
        i, rad = map(int, b.split("|"))
        pq, X = z[b + "|pq"], z[b + "|X"]
        s, h, w, te, tn = z[b + "|meta"]
        K = V.K_for(cal["focal_px"], s, (int(h), int(w)))
        R_imu = B @ candidate_R_rel(P, cal["signs"], cal["transpose"], *d.angles(i, cal["heading"])) @ V.R_REF
        truth = np.array([te, tn])
        h6 = solve_6dof(X, pq, K) if len(pq) >= 12 else None
        s6 = refine(h6["R"], h6["C"], X, pq, K) if h6 else None
        base = {"dataset": "visloc", "qid": i, "offset": rad}
        rows.append({**base, "solver": "6-DOF PnP (paper)", "level": "-", **pack(s6, truth)})
        res = solve_all(X, pq, K, R_imu, sig, rng, h6) if len(pq) >= 12 else \
            {"4-DOF hard gravity": None, "MAP soft gravity": None}
        for sname, sol in res.items():
            rows.append({**base, "solver": sname, "level": "onboard (1.5 deg)", **pack(sol, truth)})
        if (n + 1) % 100 == 0:
            print(f"  visloc {n + 1}/{len(keys)}", flush=True)
    json.dump(rows, open(os.path.join(OUT, "rows_visloc.json"), "w"))


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def nees(r):
    e, S = np.array(r["e"]), np.array(r["cov"])
    return float(e @ np.linalg.solve(S, e))


def lam_max(r):
    return float(np.max(np.linalg.eigvalsh(np.array(r["cov"]))))


def cov_stats(rows, k):
    """Coverage and protection-level stats of successful fixes with inflation k."""
    ok = [r for r in rows if r["success"]]
    if not ok:
        return None
    N = np.array([nees(r) for r in ok]) / k
    err = np.array([np.hypot(*r["e"]) for r in ok])
    pl99 = np.sqrt(k * CHI2_99 * np.array([lam_max(r) for r in ok]))
    return {"cov95": 100 * np.mean(N <= CHI2_95), "cov99": 100 * np.mean(N <= CHI2_99),
            "pl99_median": float(np.median(pl99)), "exceed_pl99": 100 * np.mean(err > pl99),
            "avail_al25": 100 * np.mean(pl99 <= 25.0), "n": len(ok)}


def k_from(rows):
    ok = [r for r in rows if r["success"]]
    return float(np.quantile([nees(r) for r in ok], 0.95) / CHI2_95)


def report():
    L = ["# Experiment 4d: soft-gravity MAP PnP (A) and calibrated covariance / protection levels (B)", ""]
    data = {}
    for ds in ("vpair", "visloc"):
        p = os.path.join(OUT, f"rows_{ds}.json")
        if os.path.exists(p):
            data[ds] = json.load(open(p))
    # ---- accuracy
    L += ["## A. Accuracy (pooled over priors)", "",
          "| Dataset | Solver | IMU tilt | Success (%) | Median (m) | 95th pct (m) | False > 25 m (%) |",
          "| --- | --- | --- | --- | --- | --- | --- |"]
    groups = {}
    for ds, rows in data.items():
        idx = {(r["qid"], r["offset"], r["solver"], r["level"]): r for r in rows}
        for r in list(rows):
            if r["solver"] == "4-DOF hard gravity":
                mp = idx.get((r["qid"], r["offset"], "MAP soft gravity", r["level"]))
                h = dict(r, solver="Hybrid: 4-DOF position + MAP covariance")
                if r["success"] and mp is not None and mp["success"]:
                    h["cov"] = mp["cov"]
                else:
                    h = dict(h, success=False)
                rows.append(h)
        for r in rows:
            groups.setdefault((ds, r["solver"], r["level"]), []).append(r)
    for (ds, s, lv), rs in sorted(groups.items()):
        ok = [r for r in rs if r["success"]]
        e = np.array([np.hypot(*r["e"]) for r in ok])
        L.append(f"| {ds} | {s} | {lv} | {100 * len(ok) / len(rs):.1f} | {np.median(e):.1f} | "
                 f"{np.percentile(e, 95):.1f} | {100 * np.sum(e > 25) / len(rs):.1f} |")
    # ---- covariance: raw, two-fold within dataset, cross-dataset
    L += ["", "## B. Covariance consistency and 99% protection levels", "",
          "Raw = solver covariance as is. Two-fold = inflation k estimated on half of the locations, "
          "evaluated on the other half (averaged). Cross = k from the OTHER dataset (same solver; "
          "for VPAIR levels, k from the matching level is not available on UAV-VisLoc, so the 0.5 deg "
          "level is paired with UAV-VisLoc's onboard attitude).", "",
          "| Dataset | Solver | IMU tilt | Raw 95% coverage | k (two-fold) | Held-out 95% / 99% coverage | "
          "Median PL99 (m) | Integrity: errors > PL99 (%) | Availability: PL99 <= 25 m alert limit (%) | Cross-dataset 95% / 99% coverage |",
          "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    ks = {}
    for (ds, s, lv), rs in sorted(groups.items()):
        ok = [r for r in rs if r["success"]]
        if len(ok) < 20:
            continue
        raw = cov_stats(rs, 1.0)
        qids = sorted({r["qid"] for r in rs}, key=str)
        fa = set(qids[0::2])
        A = [r for r in rs if r["qid"] in fa]
        Bf = [r for r in rs if r["qid"] not in fa]
        kA, kB = k_from(A), k_from(Bf)
        sA, sB = cov_stats(Bf, kA), cov_stats(A, kB)
        avg = {m: (sA[m] + sB[m]) / 2 for m in sA if m != "n"}
        ks[(ds, s, lv)] = (kA + kB) / 2
        groups[(ds, s, lv)] = rs
        L.append([ds, s, lv, raw, (kA + kB) / 2, avg])
    for idx, row in enumerate(L):
        if isinstance(row, list):
            ds, s, lv, raw, k, avg = row
            other = "visloc" if ds == "vpair" else "vpair"
            lv_o = {"vpair": "onboard (1.5 deg)", "visloc": "0.5 deg"}[ds] if s != "6-DOF PnP (paper)" else "-"
            kc = ks.get((other, s, lv_o))
            cross = cov_stats(groups[(ds, s, lv)], kc) if kc else None
            cs = f"{cross['cov95']:.0f} / {cross['cov99']:.0f}" if cross else "–"
            L[idx] = (f"| {ds} | {s} | {lv} | {raw['cov95']:.0f}% | {k:.1f} | {avg['cov95']:.0f} / {avg['cov99']:.0f}% | "
                      f"{avg['pl99_median']:.1f} | {avg['exceed_pl99']:.1f} | {avg['avail_al25']:.0f} | {cs} |")
    open(os.path.join(OUT, "exp4d.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    stage = sys.argv[1]
    if stage == "solve":
        solve_vpair() if sys.argv[2] == "vpair" else solve_visloc()
    else:
        report()
