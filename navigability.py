"""
Experiment 6b (review revision): can navigability be predicted from the map alone?

Paper, Section 5.11: "In a deployed system [fix availability] would be estimated
from the orbital imagery itself, for example by testing how reliably each map
tile matches a perturbed copy of itself."  This script builds that predictor and
tests it against REAL outcomes of real drone images (Exp 4b VPAIR, Exp 4c
UAV-VisLoc), using nothing but the map to compute it.

Map-only predictors, computed for the map tile under each query's true
footprint:
  self_match   median LightGlue homography inliers between the tile and three
               synthetically degraded copies (rotation, scale, gamma, blur,
               noise): the paper's proposal
  alias_ratio  second-best / best homography inliers when a degraded copy of the
               tile is matched against the surrounding +-400 m search crop
               (UAV-VisLoc only; VPAIR has no mosaic): a false-fix predictor
  baselines    SuperPoint keypoint count, mean gradient magnitude, grey-level
               entropy (classic scene-suitability texture indices)

Outcomes come from the real runs (rows.json), so no flight data enters the
predictors.
Usage:  python navigability.py scores   then   python navigability.py analyse
"""

import argparse
import json
import math
import os

import cv2
import numpy as np

from map_match import LightGlueMatcher, VPAIR

OUT = "results/exp6b"


# ---------------------------------------------------------------------------
# Map-only predictors
# ---------------------------------------------------------------------------

def degrade(img, rng):
    h, w = img.shape
    ang = rng.uniform(-20, 20)
    sc = rng.uniform(0.85, 1.15)
    A = cv2.getRotationMatrix2D((w / 2, h / 2), ang, sc)
    out = cv2.warpAffine(img, A, (w, h), borderMode=cv2.BORDER_REFLECT)
    g = rng.uniform(0.6, 1.6)
    out = 255.0 * (out / 255.0) ** g
    out = cv2.GaussianBlur(out, (0, 0), rng.uniform(0.5, 2.0))
    out = out + rng.normal(0, 5, out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def homography_inliers(p0, p1, thr=8.0):
    if len(p0) < 8:
        return 0, np.zeros(len(p0), bool)
    _, mask = cv2.findHomography(p0, p1, cv2.RANSAC, thr)
    if mask is None:
        return 0, np.zeros(len(p0), bool)
    mask = mask.ravel().astype(bool)
    return int(mask.sum()), mask


def self_match(m, tile, rng, n=3):
    m.angles = [0.0]
    scores = []
    for _ in range(n):
        p0, p1 = m.match(degrade(tile, rng), tile)
        scores.append(homography_inliers(p0, p1)[0])
    return float(np.median(scores))


def alias_ratio(m, tile, search, rng):
    """Best vs second-best geometric mode when a degraded tile is matched
    against the whole search crop."""
    m.angles = [0.0]
    p0, p1 = m.match(degrade(tile, rng), search)
    n1, mask = homography_inliers(p0, p1)
    if n1 == 0:
        return np.nan, 0
    n2, _ = homography_inliers(p0[~mask], p1[~mask])
    return n2 / n1, n1


def texture(m, tile):
    import torch
    with torch.no_grad():
        f = m.extractor.extract(m.to_torch(tile).to(m.device))
    n_kp = int(f["keypoints"].shape[1])
    gx = cv2.Sobel(tile.astype(float), cv2.CV_64F, 1, 0)
    gy = cv2.Sobel(tile.astype(float), cv2.CV_64F, 0, 1)
    hist = np.bincount(tile.ravel(), minlength=256) / tile.size
    ent = float(-(hist[hist > 0] * np.log2(hist[hist > 0])).sum())
    return {"n_keypoints": n_kp, "grad_mean": float(np.hypot(gx, gy).mean()), "entropy": ent}


def scores():
    os.makedirs(OUT, exist_ok=True)
    rng = np.random.default_rng(7)
    m = LightGlueMatcher(max_keypoints=4096)
    res = {"visloc": {}, "vpair": {}}

    # UAV-VisLoc: tile = 500 x 360 m footprint at 0.5 m/px; search crop as in Exp 4c.
    import visloc_attitude as V
    d = V.VisLoc()
    rows = json.load(open("results/exp4c/rows.json"))
    for qid in sorted({r["qid"] for r in rows}):
        c = d.enu(d.df.lat[qid], d.df.lon[qid])
        V.HALF_M, V.REF_PX = 250.0, 1000
        tile = d.crop(c)[140:860, :]                    # 500 m x 360 m, like the query footprint
        V.HALF_M, V.REF_PX = 400.0, 1600
        search = d.crop(c)
        ar, n1 = alias_ratio(m, tile, search, rng)
        res["visloc"][int(qid)] = {"self_match": self_match(m, tile, rng), "alias_ratio": ar,
                                   "alias_best": n1, **texture(m, tile)}
    print("visloc scores", len(res["visloc"]), flush=True)

    # VPAIR: tile = the rendered reference view at the query position (map-only product).
    data = VPAIR("vpair_sample")
    for qid in data.ids:
        tile = data.ref_image(qid)
        res["vpair"][qid] = {"self_match": self_match(m, tile, rng), **texture(m, tile)}
    print("vpair scores", len(res["vpair"]), flush=True)
    json.dump(res, open(os.path.join(OUT, "scores.json"), "w"), indent=1)


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def outcomes(rows, solver6, solver_c):
    """Per query: pooled over priors, success rate and false fixes of the
    paper's 6-DOF solver, and inliers/error of the constrained solver."""
    by = {}
    for r in rows:
        if r["match"] != "search8":
            continue
        o = by.setdefault(r["qid"], {"n6": 0, "ok6": 0, "false6": 0, "inl": [], "err": [], "okc": 0, "nc": 0})
        if r["solver"] == solver6:
            o["n6"] += 1
            if r["success"]:
                o["ok6"] += 1
                o["false6"] += r["err_m"] > 25
        elif r["solver"] == solver_c:
            o["nc"] += 1
            if r["success"]:
                o["okc"] += 1
                o["inl"].append(r["inliers"])
                o["err"].append(r["err_m"])
    out = {}
    for q, o in by.items():
        good6 = o["ok6"] - o["false6"]
        out[q] = {"bad6": 1.0 - good6 / o["n6"],                        # failed or false, 6-DOF
                  "fail_c": 1.0 - o["okc"] / o["nc"],
                  "inliers_c": float(np.median(o["inl"])) if o["inl"] else 0.0,
                  "err_c": float(np.median(o["err"])) if o["err"] else np.nan}
    return out


def auc(score, label):
    """P(score of a good location > score of a bad one), ties counted half."""
    pos, neg = score[label == 0], score[label == 1]
    if len(pos) == 0 or len(neg) == 0:
        return np.nan
    return float(np.mean((pos[:, None] > neg[None, :]) + 0.5 * (pos[:, None] == neg[None, :])))


def analyse():
    from scipy.stats import spearmanr
    S = json.load(open(os.path.join(OUT, "scores.json")))
    sets = {
        "visloc": (json.load(open("results/exp4c/rows.json")), "6-DOF PnP (paper)", "4-DOF gravity only, onboard tilt"),
        "vpair": (json.load(open("results/exp4b_rows.json")), "6-DOF PnP (paper)", "4-DOF gravity only, calibrated INS (0.1 deg)"),
    }
    rng = np.random.default_rng(0)
    L = ["# Experiment 6b: navigability predicted from the map alone, tested on real outcomes", ""]
    for name, (rows, s6, sc) in sets.items():
        out = outcomes(rows, s6, sc)
        qs = [q for q in out if str(q) in S[name]]
        preds = [k for k in S[name][str(qs[0])] if k != "alias_best"]
        Y = {k: np.array([out[q][k] for q in qs]) for k in ("bad6", "fail_c", "inliers_c", "err_c")}
        bad_any = (Y["bad6"] > 0).astype(int)
        L += [f"## {name} ({len(qs)} locations; {int(bad_any.sum())} with at least one failed or false 6-DOF fix)", "",
              "| Map-only predictor | Spearman vs constrained-solver inliers [95% CI] | Spearman vs 6-DOF bad-fix rate | AUC: good vs bad location (6-DOF) [95% CI] |",
              "| --- | --- | --- | --- |"]
        for p in preds:
            x = np.array([S[name][str(q)][p] for q in qs], dtype=float)
            sign = -1.0 if p == "alias_ratio" else 1.0      # high aliasing = bad
            ok = np.isfinite(x)
            rho_i = spearmanr(x[ok], Y["inliers_c"][ok]).correlation
            rho_b = spearmanr(x[ok], Y["bad6"][ok]).correlation
            a = auc(sign * x[ok], bad_any[ok])
            bs_r, bs_a = [], []
            idx = np.flatnonzero(ok)
            for _ in range(1000):
                b = rng.choice(idx, len(idx))
                bs_r.append(spearmanr(x[b], Y["inliers_c"][b]).correlation)
                bs_a.append(auc(sign * x[b], bad_any[b]))
            ci_r = np.nanpercentile(bs_r, [2.5, 97.5])
            ci_a = np.nanpercentile(bs_a, [2.5, 97.5])
            L.append(f"| {p} | {rho_i:.2f} [{ci_r[0]:.2f}–{ci_r[1]:.2f}] | {rho_b:.2f} | {a:.2f} [{ci_a[0]:.2f}–{ci_a[1]:.2f}] |")
        L.append("")
    open(os.path.join(OUT, "exp6b.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=["scores", "analyse"])
    a = ap.parse_args()
    scores() if a.stage == "scores" else analyse()
