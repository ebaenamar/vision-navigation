"""
V.I.S.I.O.N. Experiment 4: visual map matching on VPAIR (Schleiss et al., 2022).

For each aerial query image, match it against a rendered reference view of the
map (orthophoto + 3D surface model), turn matched reference pixels into 3D
points with the reference depth map, and solve the query camera position with
PnP + RANSAC. Compare with the true position from the GNSS/INS poses.

Two matchers:
  - SIFT + ratio test (classical baseline; rotation invariant)
  - SuperPoint + LightGlue (learned; not rotation invariant, so the query is
    tried at 8 in-plane rotations and the best is kept)

Two prior-error conditions (reference offset):
  - offset 0: the query's own reference view (coarse search already correct,
    as in VPAIR's own localisation benchmark)
  - offset 1, 2: the reference view 1 or 2 frames later (about 45 or 90 m
    away), simulating a coarse position fix that is off by that much

VPAIR does not publish camera intrinsics, so the focal length, image-axis
orientation and depth convention are calibrated from the reference views
themselves (consecutive nadir, north-up renders at known positions).

Usage:
    pip install opencv-python numpy matplotlib torch
    pip install git+https://github.com/cvg/LightGlue.git
    python map_match.py --data vpair_sample            # full run
    python map_match.py --data vpair_sample --quick    # 20 queries, test only
    python map_match.py --data vpair_sample --no-lightglue

Outputs in results/: exp4.md, exp4_summary.json, fig_exp4_error_cdf.png
"""

import argparse
import json
import math
import os
import time

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Geodesy: ECEF -> geodetic -> local East-North-Up
# ---------------------------------------------------------------------------

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def ecef_to_geodetic(x, y, z):
    """Latitude and longitude in radians (iterative, accurate to mm)."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - WGS84_E2))
    for _ in range(6):
        n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - WGS84_E2 * n / (n + h)))
    return lat, lon


def enu_matrix(lat, lon):
    """Rows are the East, North and Up unit vectors in ECEF."""
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    return np.array([[-so, co, 0.0],
                     [-sl * co, -sl * so, cl],
                     [cl * co, cl * so, sl]])


class LocalFrame:
    """East-North-Up frame centred on one ECEF point."""

    def __init__(self, ecef_origin):
        self.origin = np.asarray(ecef_origin, dtype=float)
        lat, lon = ecef_to_geodetic(*self.origin)
        self.R = enu_matrix(lat, lon)

    def to_enu(self, ecef):
        return self.R @ (np.asarray(ecef, dtype=float) - self.origin)


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def read_poses(path):
    rows = {}
    with open(path) as f:
        header = f.readline().strip().split(",")
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 4:
                continue
            rec = dict(zip(header, parts))
            key = os.path.splitext(os.path.basename(rec["filepath"]))[0]
            rec["ecef"] = np.array([float(rec["x"]), float(rec["y"]), float(rec["z"])])
            rows[key] = rec
    return rows


class VPAIR:
    def __init__(self, root):
        self.root = root
        self.queries = read_poses(os.path.join(root, "poses_query.txt"))
        self.refs = read_poses(os.path.join(root, "poses_reference_view.txt"))
        self.ids = sorted(k for k in self.queries if k in self.refs)
        if not self.ids:
            raise RuntimeError("No query ids with matching reference views found.")

    def query_image(self, qid):
        return cv2.imread(os.path.join(self.root, "queries", f"{qid}.png"), cv2.IMREAD_GRAYSCALE)

    def ref_image(self, rid):
        return cv2.imread(os.path.join(self.root, "reference_views", f"{rid}.png"), cv2.IMREAD_GRAYSCALE)

    def ref_depth(self, rid):
        return np.load(os.path.join(self.root, "reference_views", f"depth_{rid}.npy")).astype(np.float64)

    def neighbour(self, qid, offset):
        """Reference id `offset` frames after qid, or None if not in the sample."""
        nid = f"{int(qid) + offset:0{len(qid)}d}"
        return nid if nid in self.refs else None


# ---------------------------------------------------------------------------
# Matchers
# ---------------------------------------------------------------------------

class SiftMatcher:
    name = "SIFT"

    def __init__(self, n_features=4000, ratio=0.8):
        self.sift = cv2.SIFT_create(nfeatures=n_features)
        self.bf = cv2.BFMatcher(cv2.NORM_L2)
        self.ratio = ratio

    def match(self, img_q, img_r):
        kq, dq = self.sift.detectAndCompute(img_q, None)
        kr, dr = self.sift.detectAndCompute(img_r, None)
        if dq is None or dr is None or len(kq) < 8 or len(kr) < 8:
            return np.zeros((0, 2)), np.zeros((0, 2))
        good = []
        for pair in self.bf.knnMatch(dq, dr, k=2):
            if len(pair) == 2 and pair[0].distance < self.ratio * pair[1].distance:
                good.append(pair[0])
        pq = np.array([kq[m.queryIdx].pt for m in good], dtype=np.float64).reshape(-1, 2)
        pr = np.array([kr[m.trainIdx].pt for m in good], dtype=np.float64).reshape(-1, 2)
        return pq, pr


class LightGlueMatcher:
    name = "SuperPoint+LightGlue"

    def __init__(self, max_keypoints=2048, n_rotations=8):
        import torch
        from lightglue import LightGlue, SuperPoint
        from lightglue.utils import numpy_image_to_torch, rbd
        self.torch = torch
        self.rbd = rbd
        self.to_torch = numpy_image_to_torch
        if torch.backends.mps.is_available():
            self.device = torch.device("mps")
        elif torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
        self.extractor = SuperPoint(max_num_keypoints=max_keypoints).eval().to(self.device)
        self.matcher = LightGlue(features="superpoint").eval().to(self.device)
        self.angles = [i * 360.0 / n_rotations for i in range(n_rotations)]
        self._ref_cache = {}

    def _features(self, img):
        with self.torch.no_grad():
            return self.extractor.extract(self.to_torch(img).to(self.device))

    def _match_feats(self, f0, f1):
        with self.torch.no_grad():
            out = self.matcher({"image0": f0, "image1": f1})
        f0, f1, out = [self.rbd(x) for x in (f0, f1, out)]
        m = out["matches"].cpu().numpy()
        k0 = f0["keypoints"].cpu().numpy()
        k1 = f1["keypoints"].cpu().numpy()
        return k0[m[:, 0]].astype(np.float64), k1[m[:, 1]].astype(np.float64)

    def match(self, img_q, img_r):
        """Try the query at several in-plane rotations; keep the one with most
        geometrically consistent matches, mapped back to original pixels."""
        fr = self._features(img_r)
        h, w = img_q.shape
        centre = (w / 2.0, h / 2.0)
        best = (np.zeros((0, 2)), np.zeros((0, 2)), -1)
        for angle in self.angles:
            A = cv2.getRotationMatrix2D(centre, angle, 1.0)
            rot = cv2.warpAffine(img_q, A, (w, h))
            pq_rot, pr = self._match_feats(self._features(rot), fr)
            if len(pq_rot) < 8:
                continue
            _, mask = cv2.estimateAffinePartial2D(pq_rot, pr, method=cv2.RANSAC,
                                                  ransacReprojThreshold=8.0)
            n_in = int(mask.sum()) if mask is not None else 0
            if n_in > best[2]:
                A_inv = cv2.invertAffineTransform(A)
                pq = (A_inv[:, :2] @ pq_rot.T).T + A_inv[:, 2]
                best = (pq, pr, n_in)
        return best[0], best[1]


# ---------------------------------------------------------------------------
# Camera calibration from consecutive reference views
# ---------------------------------------------------------------------------

def calibrate(data, n_pairs=40):
    """
    Reference views are nadir, north-up renders. Moving the camera by (dE, dN)
    metres shifts a ground point at depth Z by (-sx*f*dE/Z, sy*f*dN/Z) pixels,
    where sx = +1 if image x points East and sy = +1 if image y points South.
    Fitting this over many matches gives f, the axis signs, and whether the
    depth maps store z-depth or range along the ray.
    """
    sift = SiftMatcher(n_features=3000, ratio=0.75)
    h, w = data.ref_image(data.ids[0]).shape
    cx, cy = w / 2.0, h / 2.0
    samples = []   # (u, v, du, dv, depth, dE, dN)
    pairs_used = 0
    for rid in data.ids:
        nid = data.neighbour(rid, 1)
        if nid is None:
            continue
        frame = LocalFrame(data.refs[rid]["ecef"])
        dE, dN, _ = frame.to_enu(data.refs[nid]["ecef"])
        if math.hypot(dE, dN) < 5:
            continue
        p0, p1 = sift.match(data.ref_image(rid), data.ref_image(nid))
        if len(p0) < 30:
            continue
        _, mask = cv2.findHomography(p0, p1, cv2.RANSAC, 3.0)
        if mask is None:
            continue
        p0, p1 = p0[mask.ravel() == 1], p1[mask.ravel() == 1]
        depth = data.ref_depth(rid)
        for (u, v), (u1, v1) in zip(p0, p1):
            d = depth[int(round(min(v, h - 1))), int(round(min(u, w - 1)))]
            if np.isfinite(d) and d > 0:
                samples.append((u, v, u1 - u, v1 - v, d, dE, dN))
        pairs_used += 1
        if pairs_used >= n_pairs:
            break
    if pairs_used < 5:
        raise RuntimeError("Calibration failed: too few consecutive reference pairs matched.")

    s = np.array(samples)
    u, v, du, dv, d, dE, dN = s.T

    def fit(Z):
        # du*Z = -sx f dE  and  dv*Z = sy f dN. Per-axis estimates gx=-sx f, gy=sy f
        # give the signs; one isotropic f (square pixels) is then fitted to both
        # axes jointly, so an axis with little motion cannot distort it.
        gx = np.sum(du * Z * dE) / np.sum(dE * dE)
        gy = np.sum(dv * Z * dN) / np.sum(dN * dN)
        sxx, syy = -np.sign(gx), np.sign(gy)
        f_iso = (np.sum(-sxx * du * Z * dE) + np.sum(syy * dv * Z * dN)) / (np.sum(dE * dE) + np.sum(dN * dN))
        resid = np.concatenate([du * Z + sxx * f_iso * dE, dv * Z - syy * f_iso * dN])
        scale = f_iso * np.sqrt(np.mean(np.concatenate([dE ** 2, dN ** 2])))
        return -sxx * f_iso, syy * f_iso, float(np.sqrt(np.mean(resid ** 2)) / scale), gx, gy

    gx, gy, err_z, ax_x, ax_y = fit(d)                       # depth = z-depth
    f_r = abs(gx)
    for _ in range(10):                                      # depth = range along ray
        cos_ray = 1.0 / np.sqrt(1 + ((u - cx) / f_r) ** 2 + ((v - cy) / f_r) ** 2)
        gx_r, gy_r, err_r, ax_xr, ax_yr = fit(d * cos_ray)
        f_r = abs(gx_r)
    depth_is_range = err_r < err_z
    if depth_is_range:
        gx, gy, ax_x, ax_y = gx_r, gy_r, ax_xr, ax_yr
    sx, sy = -np.sign(gx), np.sign(gy)
    fx, fy = abs(ax_x), abs(ax_y)          # per-axis estimates, diagnostics only
    calib = {"fx": float(fx), "fy": float(fy), "f": float(abs(gx)),
             "cx": cx, "cy": cy, "width": w, "height": h,
             "sx": int(sx), "sy": int(sy), "depth_is_range": bool(depth_is_range),
             "relative_residual_zdepth": err_z, "relative_residual_range": err_r,
             "pairs_used": pairs_used, "matches_used": int(len(s)),
             "hfov_deg": float(math.degrees(2 * math.atan(w / 2 / abs(gx)))),
             "mean_east_step_m": float(np.mean(np.abs(dE))), "mean_north_step_m": float(np.mean(np.abs(dN)))}
    if sx * sy * 1 != 1:
        # x=E, y=S, z=Down is right-handed only if sx*sy == +1 (see world_from_cam)
        calib["warning"] = "Axis signs imply a mirrored image; check before trusting results."
    return calib


def world_from_cam(calib):
    """Rotation taking camera-frame vectors to local ENU for a nadir north-up view."""
    sx, sy = calib["sx"], calib["sy"]
    return np.array([[sx, 0.0, 0.0],
                     [0.0, -sy, 0.0],
                     [0.0, 0.0, -1.0]])


def ref_pixels_to_enu(pix, depth_map, calib):
    """3D ENU points (metres, relative to the reference camera) for reference pixels."""
    h, w = depth_map.shape
    ui = np.clip(np.round(pix[:, 0]).astype(int), 0, w - 1)
    vi = np.clip(np.round(pix[:, 1]).astype(int), 0, h - 1)
    d = depth_map[vi, ui]
    x = (pix[:, 0] - calib["cx"]) / calib["f"]
    y = (pix[:, 1] - calib["cy"]) / calib["f"]
    if calib["depth_is_range"]:
        z = d / np.sqrt(1 + x * x + y * y)
    else:
        z = d
    cam = np.stack([x * z, y * z, z], axis=1)
    valid = np.isfinite(d) & (d > 0)
    return (world_from_cam(calib) @ cam.T).T, valid


# ---------------------------------------------------------------------------
# Localisation of one query
# ---------------------------------------------------------------------------

def localise(matcher, data, calib, qid, rid, min_inliers=12, reproj_px=8.0, debug=False):
    img_q, img_r = data.query_image(qid), data.ref_image(rid)
    t0 = time.perf_counter()
    pq, pr = matcher.match(img_q, img_r)
    out = {"n_matches": int(len(pq)), "n_inliers": 0, "success": False}
    if len(pq) >= min_inliers:
        pts3d, valid = ref_pixels_to_enu(pr, data.ref_depth(rid), calib)
        pq, pts3d = pq[valid], pts3d[valid]
        if len(pq) >= min_inliers:
            K = np.array([[calib["f"], 0, calib["cx"]],
                          [0, calib["f"], calib["cy"]],
                          [0, 0, 1.0]])
            ok, rvec, tvec, inl = cv2.solvePnPRansac(
                pts3d.astype(np.float64), pq.astype(np.float64), K, None,
                iterationsCount=3000, reprojectionError=reproj_px, confidence=0.999,
                flags=cv2.SOLVEPNP_EPNP)
            if ok and inl is not None and len(inl) >= min_inliers:
                idx = inl.ravel()
                rvec, tvec = cv2.solvePnPRefineLM(pts3d[idx], pq[idx], K, None, rvec, tvec)
                R, _ = cv2.Rodrigues(rvec)
                centre = (-R.T @ tvec).ravel()       # query camera centre, ENU of reference
                truth = LocalFrame(data.refs[rid]["ecef"]).to_enu(data.queries[qid]["ecef"])
                err = centre - truth
                out.update(success=True, n_inliers=int(len(idx)),
                           horiz_err_m=float(math.hypot(err[0], err[1])),
                           err_3d_m=float(np.linalg.norm(err)),
                           prior_offset_m=float(math.hypot(truth[0], truth[1])))
    out["time_s"] = time.perf_counter() - t0
    if debug:
        n_h = 0
        if out["n_matches"] >= 8:
            _, hm = cv2.findHomography(pq if len(pq) else np.zeros((0, 2)), pr[:len(pq)] if len(pq) else np.zeros((0, 2)), cv2.RANSAC, 8.0) if len(pq) >= 8 else (None, None)
            n_h = int(hm.sum()) if hm is not None else 0
        print(f"    query {qid} vs ref {rid}: {out['n_matches']} matches, "
              f"{n_h} homography inliers, {out['n_inliers']} PnP inliers"
              + (f", error {out['horiz_err_m']:.1f} m" if out['success'] else ""))
    out["landcover"] = data.queries[qid].get("landcover", "")
    return out


# ---------------------------------------------------------------------------
# Experiment
# ---------------------------------------------------------------------------

def summarise(rows):
    n = len(rows)
    ok = [r for r in rows if r["success"]]
    e = np.array([r["horiz_err_m"] for r in ok]) if ok else np.array([])
    pct = lambda a: 100.0 * a / n if n else 0.0
    return {
        "queries": n,
        "success_pct": pct(len(ok)),
        "median_horiz_m": float(np.median(e)) if len(e) else None,
        "p95_horiz_m": float(np.percentile(e, 95)) if len(e) else None,
        "within_25m_pct": pct(int(np.sum(e <= 25))),
        "within_50m_pct": pct(int(np.sum(e <= 50))),
        "median_inliers": float(np.median([r["n_inliers"] for r in ok])) if ok else 0.0,
        "median_time_s": float(np.median([r["time_s"] for r in rows])) if rows else None,
        "mean_prior_offset_m": float(np.mean([r["prior_offset_m"] for r in ok])) if ok else None,
    }


def fmt(x, nd=1):
    return "–" if x is None else f"{x:.{nd}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="vpair_sample")
    ap.add_argument("--out", default="results")
    ap.add_argument("--offsets", default="0,1,2")
    ap.add_argument("--max-queries", type=int, default=None)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--no-lightglue", action="store_true")
    ap.add_argument("--debug", action="store_true", help="print match counts per query")
    a = ap.parse_args()
    if a.quick:
        a.max_queries = 20
    os.makedirs(a.out, exist_ok=True)
    offsets = [int(x) for x in a.offsets.split(",")]

    data = VPAIR(a.data)
    ids = data.ids[: a.max_queries] if a.max_queries else data.ids
    print(f"{len(data.ids)} query/reference pairs found; using {len(ids)}.")

    print("Calibrating camera from consecutive reference views...")
    calib = calibrate(data)
    print(f"  f = {calib['f']:.1f} px (per-axis estimates fx {calib['fx']:.1f}, fy {calib['fy']:.1f}; "
          f"mean steps east {calib['mean_east_step_m']:.1f} m, north {calib['mean_north_step_m']:.1f} m), "
          f"horizontal FOV {calib['hfov_deg']:.1f} deg, axis signs sx={calib['sx']}, sy={calib['sy']}, "
          f"depth stored as {'range' if calib['depth_is_range'] else 'z-depth'}")
    if "warning" in calib:
        print("  WARNING:", calib["warning"])

    matchers = [SiftMatcher()]
    if not a.no_lightglue:
        try:
            matchers.append(LightGlueMatcher())
        except Exception as e:
            print(f"LightGlue unavailable ({type(e).__name__}: {e}); running SIFT only.")

    all_rows, summary = {}, []
    for m in matchers:
        for off in offsets:
            rows = []
            print(f"\n{m.name}, reference offset {off}:")
            for i, qid in enumerate(ids):
                rid = data.neighbour(qid, off)
                if rid is None:
                    continue
                rows.append(localise(m, data, calib, qid, rid, debug=a.debug))
                if (i + 1) % 25 == 0:
                    print(f"  {i + 1}/{len(ids)}")
            s = summarise(rows)
            s.update(matcher=m.name, offset=off)
            summary.append(s)
            all_rows[f"{m.name}|{off}"] = rows
            print(f"  success {s['success_pct']:.1f}%, median {fmt(s['median_horiz_m'])} m, "
                  f"within 25 m {s['within_25m_pct']:.1f}%")

    # ---- per-landcover breakdown (offset 0)
    land = {}
    for m in matchers:
        rows = all_rows.get(f"{m.name}|0", [])
        for code in sorted({r["landcover"] for r in rows}):
            sub = [r for r in rows if r["landcover"] == code]
            land[f"{m.name}|{code}"] = summarise(sub)

    # ---- figure: CDF of horizontal error, offset 0
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for m in matchers:
        rows = all_rows.get(f"{m.name}|0", [])
        e = np.sort([r["horiz_err_m"] for r in rows if r["success"]])
        if len(e) and rows:
            ax.step(e, np.arange(1, len(e) + 1) / len(rows) * 100, where="post", label=m.name)
    ax.axvline(25, linestyle=":", color="k", label="25 m (C4 target)")
    ax.axvline(10, linestyle="--", color="0.5", label="10 m (stretch goal)")
    ax.set_xscale("log")
    ax.set_xlabel("Horizontal position error (m)")
    ax.set_ylabel("Queries localised within error (%)")
    ax.set_ylim(0, 100)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "fig_exp4_error_cdf.png"), dpi=200)
    plt.close(fig)

    # ---- report
    L = ["# Experiment 4: Map matching on VPAIR", ""]
    if a.quick or a.max_queries:
        L.append(f"> Test run on {len(ids)} queries only.\n")
    L += ["## Camera calibration (from consecutive reference views)", "",
          f"- Focal length: {calib['f']:.1f} px (fx {calib['fx']:.1f}, fy {calib['fy']:.1f}); "
          f"horizontal field of view {calib['hfov_deg']:.1f}°; principal point assumed at image centre.",
          f"- Axis signs: sx = {calib['sx']}, sy = {calib['sy']}; depth maps treated as "
          f"{'range along the ray' if calib['depth_is_range'] else 'z-depth'} "
          f"(relative residuals: z-depth {calib['relative_residual_zdepth']:.3f}, range {calib['relative_residual_range']:.3f}).",
          f"- From {calib['pairs_used']} reference pairs and {calib['matches_used']} matches.",
          "", "## Results", "",
          "| Matcher | Reference offset | Mean prior error (m) | Queries | Success (%) | Median horizontal error (m) | 95th percentile (m) | Within 25 m (%) | Within 50 m (%) | Median inliers | Median time per query (s) |",
          "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for s in summary:
        L.append(f"| {s['matcher']} | {s['offset']} | {fmt(s['mean_prior_offset_m'])} | {s['queries']} | "
                 f"{s['success_pct']:.1f} | {fmt(s['median_horiz_m'])} | {fmt(s['p95_horiz_m'])} | "
                 f"{s['within_25m_pct']:.1f} | {s['within_50m_pct']:.1f} | {s['median_inliers']:.0f} | "
                 f"{fmt(s['median_time_s'], 2)} |")
    L += ["", "Success = PnP with at least 12 inliers within 8 px. Errors are over successful queries; "
          "\"within\" percentages are over all queries, with failures counted as not within.",
          "Unlike VPAIR's Table IV, the within-25 m figure applies no rotation threshold.",
          "", "## By land cover (offset 0)", "",
          "| Matcher | Land-cover code | Queries | Success (%) | Median horizontal error (m) | Within 25 m (%) |",
          "| --- | --- | --- | --- | --- | --- |"]
    for k, s in land.items():
        mname, code = k.split("|")
        L.append(f"| {mname} | {code} | {s['queries']} | {s['success_pct']:.1f} | "
                 f"{fmt(s['median_horiz_m'])} | {s['within_25m_pct']:.1f} |")
    with open(os.path.join(a.out, "exp4.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    with open(os.path.join(a.out, "exp4_summary.json"), "w") as f:
        json.dump({"calibration": calib, "summary": summary, "landcover": land}, f, indent=2)
    print(f"\nDone. See {a.out}/exp4.md and {a.out}/fig_exp4_error_cdf.png")


if __name__ == "__main__":
    main()
