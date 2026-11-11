"""
Experiment 4c: attitude-aided map matching on a SECOND real dataset,
UAV-VisLoc (Xu et al., 2024), example flight 03 (Taizhou, 768 images,
~466 m altitude, 0.3 m Google satellite map).  Get the example from
https://github.com/IntelliSensing/UAV-VisLoc and unzip it into visloc/raw.

Differences from Exp 4b (VPAIR):
  - the reference is a north-up crop of a satellite mosaic around a PRIOR that
    is displaced in a random 2-D direction (0-300 m), so priors are no longer
    along-track only;
  - satellite imagery instead of aerial orthophotos; a different camera, region
    and season;
  - no DSM and no published intrinsics: the ground is taken as flat (the
    region is a river plain), and the focal length is calibrated on a held-out
    subset from the GNSS altitude.

Solvers and attitude levels are those of map_match_attitude.py.
Usage:  python visloc_attitude.py [--step 3] [--radii 0,100,200,300]
"""

import argparse
import json
import math
import os
import time

import cv2
import numpy as np
import pandas as pd
import rasterio
from rasterio.windows import from_bounds

from map_match import LightGlueMatcher
from map_match_attitude import (DEG, LEVELS, attitude_gate, candidate_R_rel, report, rot_exp,
                                rot_log, signed_perms, solve_3dof, solve_4dof, solve_6dof)

ROOT = "visloc/raw/03"
R_REF = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])   # ENU -> nadir north-up camera
QUERY_W = 1000
HALF_M = 400.0
REF_PX = 1600


class VisLoc:
    def __init__(self, root=ROOT):
        self.root = root
        self.df = pd.read_csv(os.path.join(root, "03.csv"))
        self.sat = rasterio.open(os.path.join(root, "satellite03.tif"))
        self.lat0 = float(self.df.lat.mean())
        self.lon0 = float(self.df.lon.mean())

    def enu(self, lat, lon):
        """Local tangent-plane east/north (m) about the flight centre."""
        e = math.radians(lon - self.lon0) * 6378137.0 * math.cos(math.radians(self.lat0))
        n = math.radians(lat - self.lat0) * 6378137.0
        return np.array([e, n])

    def latlon(self, e, n):
        lat = self.lat0 + math.degrees(n / 6378137.0)
        lon = self.lon0 + math.degrees(e / (6378137.0 * math.cos(math.radians(self.lat0))))
        return lat, lon

    def query(self, i):
        img = cv2.imread(os.path.join(self.root, "drone", self.df.filename[i]), cv2.IMREAD_GRAYSCALE)
        s = QUERY_W / img.shape[1]
        return cv2.resize(img, (QUERY_W, int(round(img.shape[0] * s))), interpolation=cv2.INTER_AREA), s

    def crop(self, centre_en):
        """North-up crop of +-HALF_M around an east/north centre, REF_PX square."""
        lat_c, lon_c = self.latlon(*centre_en)
        dlat = math.degrees(HALF_M / 6378137.0)
        dlon = math.degrees(HALF_M / (6378137.0 * math.cos(math.radians(self.lat0))))
        win = from_bounds(lon_c - dlon, lat_c - dlat, lon_c + dlon, lat_c + dlat, self.sat.transform)
        rgb = self.sat.read(window=win, out_shape=(3, REF_PX, REF_PX), boundless=True, fill_value=0)
        return cv2.cvtColor(np.transpose(rgb, (1, 2, 0)), cv2.COLOR_RGB2GRAY)

    def angles(self, i, which):
        r = self.df.iloc[i]
        return math.radians(r.Omega), math.radians(r.Kappa), math.radians(r[which])


def ref_points(pr, centre_en):
    """Reference crop pixels -> ENU points on flat ground (z = 0), absolute frame."""
    s = 2 * HALF_M / REF_PX
    c = (REF_PX - 1) / 2
    e = centre_en[0] + (pr[:, 0] - c) * s
    n = centre_en[1] - (pr[:, 1] - c) * s
    return np.c_[e, n, np.zeros(len(pr))]


def K_for(f_full, scale, shape):
    f = f_full * scale
    h, w = shape
    return np.array([[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1.0]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=int, default=3)
    ap.add_argument("--radii", default="0,100,200,300")
    ap.add_argument("--out", default="results/exp4c")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    d = VisLoc()
    m = LightGlueMatcher(max_keypoints=4096)
    rng = np.random.default_rng(20261112)
    n_img = len(d.df)
    calib_ids = list(range(0, n_img, 5))                      # held out for calibration
    eval_ids = [i for i in range(0, n_img, a.step) if i % 5 != 0]
    truth = {i: d.enu(d.df.lat[i], d.df.lon[i]) for i in range(n_img)}
    h_agl = float(d.df.height.median())

    # ---- 1. calibration subset: matches at the true position (8-rotation search)
    cal = []
    for i in calib_ids:
        q, s = d.query(i)
        pq, pr = m.match(q, d.crop(truth[i]))
        if len(pq) >= 30:
            cal.append((i, q.shape, s, pq, ref_points(pr, truth[i])))
    print(f"calibration images with matches: {len(cal)}", flush=True)

    # Focal length (full-resolution px): the value for which 6-DOF PnP heights
    # agree best with the GNSS altitude.
    best_f = None
    for f in np.linspace(2000, 5000, 61):
        errs = []
        for i, shp, s, pq, X in cal:
            sol = solve_6dof(X, pq, K_for(f, s, shp))
            if sol is not None and sol["inliers"] >= 50:
                errs.append(sol["C"][2] - d.df.height[i])
        if len(errs) > 0.5 * len(cal):
            score = abs(float(np.median(errs)))
            if best_f is None or score < best_f[1]:
                best_f = (f, score, len(errs))
    f_full = best_f[0]
    print(f"focal {f_full:.0f} px (full res), height residual {best_f[1]:.2f} m over {best_f[2]}", flush=True)

    # Attitude convention (which heading column, axis mapping) and boresight,
    # both from the calibration subset only.
    rels = []
    for i, shp, s, pq, X in cal:
        sol = solve_6dof(X, pq, K_for(f_full, s, shp))
        if sol is not None and sol["inliers"] >= 100:
            rels.append((i, sol["R"] @ R_REF.T))
    import itertools
    best = None
    for which in ("Phi1", "Phi2"):
        for P in signed_perms():
            for signs in itertools.product((1, -1), repeat=3):
                for tr in (False, True):
                    errs = [np.linalg.norm(rot_log(candidate_R_rel(P, signs, tr, *d.angles(i, which)).T @ Rr))
                            for i, Rr in rels]
                    med = float(np.median(errs))
                    if best is None or med < best[0]:
                        best = (med, which, P, signs, tr)
    med, which, P, signs, tr = best
    ins = lambda i: candidate_R_rel(P, signs, tr, *d.angles(i, which))
    B = rot_exp(np.mean([rot_log(Rr @ ins(i).T) for i, Rr in rels], axis=0))
    conv = {"heading": which, "P": P.tolist(), "signs": list(signs), "transpose": tr,
            "median_disagreement_deg": math.degrees(med), "n": len(rels),
            "boresight_deg": math.degrees(np.linalg.norm(rot_log(B))), "focal_px": f_full}
    print(json.dumps(conv), flush=True)
    json.dump(conv, open(os.path.join(a.out, "calibration.json"), "w"), indent=1)

    # ---- 2. evaluation
    radii = [float(x) for x in a.radii.split(",")]
    rows = []
    for k, i in enumerate(eval_ids):
        q, s = d.query(i)
        K = K_for(f_full, s, q.shape)
        Rr = B @ ins(i)
        # The recorded attitude is used as is (no simulated noise): it is the
        # real onboard estimate, about 1.5 deg per axis after boresight calibration.
        for rad in radii:
            ang = rng.uniform(0, 2 * math.pi)
            centre = truth[i] + rad * np.array([math.cos(ang), math.sin(ang)])
            ref = d.crop(centre)
            matches = {}
            m.angles = [j * 45.0 for j in range(8)]
            t0 = time.perf_counter()
            matches["search8"] = (*m.match(q, ref), time.perf_counter() - t0)
            Rh = Rr                                            # recorded heading
            m.angles = [math.degrees(math.atan2(Rh[1, 0], Rh[0, 0]))]
            t0 = time.perf_counter()
            matches["heading"] = (*m.match(q, ref), time.perf_counter() - t0)
            for mode, (pq, pr, tm) in matches.items():
                X = ref_points(pr, centre) if len(pr) else np.zeros((0, 3))
                sol6 = solve_6dof(X, pq, K) if len(pq) >= 12 else None
                lvl = "onboard attitude as recorded"
                solvers = [("6-DOF PnP (paper)", "6dof", None), (f"3-DOF, {lvl}", "3dof", lvl)]
                if mode == "search8":
                    solvers += [(f"6-DOF + attitude gate (5 deg), {lvl}", "gate", lvl),
                                (f"4-DOF gravity only, onboard tilt", "4dof", lvl)]
                for sname, kind, lvl in solvers:
                    R = Rr @ R_REF if lvl else None
                    if len(pq) < 12:
                        sol = None
                    elif kind == "6dof":
                        sol = sol6
                    elif kind == "3dof":
                        sol = solve_3dof(X, pq, K, R, rng=rng)
                    elif kind == "gate":
                        sol = attitude_gate(sol6, R, 5.0)
                    else:
                        sol = solve_4dof(X, pq, K, R, rng=rng)
                    r = {"qid": int(i), "offset": int(rad), "match": mode, "solver": sname,
                         "t_match": tm, "prior_m": rad}
                    if sol is None:
                        r["success"] = False
                    else:
                        r.update(success=True, inliers=sol["inliers"],
                                 err_m=float(np.hypot(*(sol["C"][:2] - truth[i]))))
                    rows.append(r)
        if (k + 1) % 10 == 0:
            print(f"  {k + 1}/{len(eval_ids)}", flush=True)
            json.dump(rows, open(os.path.join(a.out, "rows.json"), "w"))
    json.dump(rows, open(os.path.join(a.out, "rows.json"), "w"))
    report(rows, a.out)
    os.replace(os.path.join(a.out, "exp4b.md"), os.path.join(a.out, "exp4c.md"))


if __name__ == "__main__":
    main()
