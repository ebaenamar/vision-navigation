"""
Cache UAV-VisLoc matches and calibration for the soft-gravity PnP study.

Repeats the calibration of visloc_attitude.py (focal length, attitude
convention, camera-INS boresight; all from every 5th image only) and stores the
8-rotation-search LightGlue matches for every evaluation image and 2-D prior,
so that solvers can be compared on identical correspondences.
Usage:  python visloc_cache.py [--step 3] [--radii 0,100,200,300]
"""

import argparse
import itertools
import json
import math
import os

import numpy as np

import visloc_attitude as V
from map_match import LightGlueMatcher
from map_match_attitude import candidate_R_rel, rot_exp, rot_log, signed_perms, solve_6dof

OUT = "results/cache_visloc"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=int, default=3)
    ap.add_argument("--radii", default="0,100,200,300")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    d = V.VisLoc()
    m = LightGlueMatcher(max_keypoints=4096)
    rng = np.random.default_rng(20261112)
    n = len(d.df)
    truth = {i: d.enu(d.df.lat[i], d.df.lon[i]) for i in range(n)}

    cal = []
    for i in range(0, n, 5):
        q, s = d.query(i)
        m.angles = [j * 45.0 for j in range(8)]
        pq, pr = m.match(q, d.crop(truth[i]))
        if len(pq) >= 30:
            cal.append((i, q.shape, s, pq, V.ref_points(pr, truth[i])))
    best_f = None
    for f in np.linspace(2000, 5000, 61):
        errs = []
        for i, shp, s, pq, X in cal:
            sol = solve_6dof(X, pq, V.K_for(f, s, shp))
            if sol is not None and sol["inliers"] >= 50:
                errs.append(sol["C"][2] - d.df.height[i])
        if len(errs) > 0.5 * len(cal):
            sc = abs(float(np.median(errs)))
            if best_f is None or sc < best_f[1]:
                best_f = (f, sc)
    f_full = float(best_f[0])
    rels = []
    for i, shp, s, pq, X in cal:
        sol = solve_6dof(X, pq, V.K_for(f_full, s, shp))
        if sol is not None and sol["inliers"] >= 100:
            rels.append((i, sol["R"] @ V.R_REF.T))
    best = None
    for which in ("Phi1", "Phi2"):
        for P in signed_perms():
            for signs in itertools.product((1, -1), repeat=3):
                for tr in (False, True):
                    e = [np.linalg.norm(rot_log(candidate_R_rel(P, signs, tr, *d.angles(i, which)).T @ Rr))
                         for i, Rr in rels]
                    if best is None or np.median(e) < best[0]:
                        best = (float(np.median(e)), which, P, signs, tr)
    _, which, P, signs, tr = best
    B = rot_exp(np.mean([rot_log(Rr @ candidate_R_rel(P, signs, tr, *d.angles(i, which)).T)
                         for i, Rr in rels], axis=0))
    calib = {"focal_px": f_full, "heading": which, "P": P.tolist(), "signs": list(signs),
             "transpose": tr, "B": B.tolist()}
    json.dump(calib, open(os.path.join(OUT, "calib.json"), "w"), indent=1)
    print("calibration", calib, flush=True)

    radii = [float(x) for x in a.radii.split(",")]
    eval_ids = [i for i in range(0, n, a.step) if i % 5 != 0]
    store = {}
    for k, i in enumerate(eval_ids):
        q, s = d.query(i)
        for rad in radii:
            ang = rng.uniform(0, 2 * math.pi)
            c = truth[i] + rad * np.array([math.cos(ang), math.sin(ang)])
            m.angles = [j * 45.0 for j in range(8)]
            pq, pr = m.match(q, d.crop(c))
            b = f"{i}|{int(rad)}"
            store[b + "|pq"], store[b + "|X"] = pq, V.ref_points(pr, c) if len(pr) else np.zeros((0, 3))
            store[b + "|meta"] = np.array([s, q.shape[0], q.shape[1], truth[i][0], truth[i][1]])
        if (k + 1) % 20 == 0:
            print(f"  {k + 1}/{len(eval_ids)}", flush=True)
            np.savez_compressed(os.path.join(OUT, "matches.npz"), **store)
    np.savez_compressed(os.path.join(OUT, "matches.npz"), **store)
    print("done", flush=True)


if __name__ == "__main__":
    main()
