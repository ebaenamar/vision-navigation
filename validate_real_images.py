"""
Validation of the V.I.S.I.O.N. star pipeline on REAL night-sky images.

Data: the two real star-camera images distributed with ESA's open-source
tetra3 solver (examples/data, Apache-2.0), taken from the ground on
2019-07-29 at 40 deg and 60 deg altitude.  Get them with
    git clone --depth 1 https://github.com/esa/tetra3 tetra3_repo

Reference ("truth"): the independent tetra3 plate solution of the same
image (Hipparcos with proper motion to 2023, its own centroiding and
pattern matching).  V.I.S.I.O.N. is scored against it by
  - boresight separation between the two solutions,
  - agreement of identified Hipparcos numbers,
  - residuals of tetra3's matched stars under the V.I.S.I.O.N. attitude
    (a full three-axis check, including roll).

Usage:
    python validate_real_images.py --images tetra3_repo/examples/data
Output: results/real_images.md
"""

import argparse
import glob
import math
import os

import numpy as np

np.math = math          # tetra3 still uses np.math, removed in NumPy 2
import tetra3            # noqa: E402
from PIL import Image    # noqa: E402
from scipy import ndimage  # noqa: E402

from vision_core import (Camera, StarCatalog, StarIdentifier, detect_stars,  # noqa: E402
                         radec_to_vec, boresight_inertial, angle_between_rad, vec_to_radec)

ARCSEC = math.degrees(1.0) * 3600.0


def flatten(img, size=64):
    """Remove the sky gradient of a ground-based image (median filter)."""
    bg = ndimage.median_filter(img, size=size)
    return img - bg + np.median(bg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default="tetra3_repo/examples/data")
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    cat = StarCatalog.from_hipparcos(mag_limit=7.0, epoch=2019.58)   # images taken 2019-07-29
    hip_index = {int(h): i for i, h in enumerate(cat.ids)}
    t3 = tetra3.Tetra3()

    rows = []
    for path in sorted(glob.glob(os.path.join(a.images, "*.tif*"))):
        name = os.path.basename(path)
        with Image.open(path) as im:
            ref = t3.solve_from_image(im, distortion=[-0.2, 0.1], return_matches=True)
            img = np.array(im, dtype=float)
        if ref.get("RA") is None:
            rows.append({"image": name, "note": "tetra3 failed"})
            continue
        h, w = img.shape
        cam = Camera(width=w, height=h, fov_deg=float(ref["FOV"]))
        paper_ident = StarIdentifier(cat, cam, id_mag_limit=5.5, tol_deg=0.02, refine_tol_deg=0.03)
        tuned_ident = StarIdentifier(cat, cam, id_mag_limit=6.0, tol_deg=0.03, refine_tol_deg=0.05)

        row = {"image": name, "fov_deg": float(ref["FOV"]), "t3_matches": int(ref["Matches"]),
               "t3_rmse_arcsec": float(ref["RMSE"])}
        for variant, im_used, ident, ms, mt in (
                ("paper settings", img, paper_ident, 8, 40),
                ("paper settings + gradient removal", flatten(img), paper_ident, 8, 40),
                ("tuned (mag 6, 12 stars) + gradient removal", flatten(img), tuned_ident, 12, 200)):
            xy, _ = detect_stars(im_used)
            out = ident.identify(xy, max_stars=ms, max_triplets=mt)
            key = variant
            if out is None:
                row[key] = {"detected": len(xy), "solved": False}
                continue
            R = out["R"]
            # 1) boresight vs tetra3
            bs_ref = radec_to_vec(ref["RA"], ref["Dec"])
            sep = angle_between_rad(boresight_inertial(R), bs_ref) * ARCSEC
            # 2) ID agreement
            ours = set(int(x) for x in out["cat_ids"])
            theirs = set(int(x) for x in ref["matched_catID"])
            # 3) three-axis check: tetra3's matched stars projected with OUR attitude
            # tetra3 returns (y, x) with the top-left pixel centre at (0.5, 0.5);
            # V.I.S.I.O.N. puts pixel centres at integers, so shift by -0.5.
            yx = np.asarray(ref["matched_centroids"]) - 0.5
            v_img = cam.pixel_to_vec(yx[:, 1], yx[:, 0])
            res = []
            for k, hip in enumerate(ref["matched_catID"]):
                if int(hip) in hip_index:
                    r_in = cat.vecs[hip_index[int(hip)]]
                    res.append(angle_between_rad(v_img[k], R @ r_in) * ARCSEC)
            row[key] = {"detected": len(xy), "solved": True, "identified": len(ours),
                        "ids_in_tetra3": len(ours & theirs), "ids_common_denominator": len(ours),
                        "boresight_sep_arcsec": sep,
                        "t3_star_residual_rms_arcsec": float(np.sqrt(np.mean(np.square(res)))),
                        "own_residual_rms_arcsec": float(np.sqrt(np.mean(out["residuals_rad"] ** 2)) * ARCSEC),
                        "radec": vec_to_radec(boresight_inertial(R))}
        rows.append(row)
        print(row)

    L = ["# Validation on real night-sky images (ESA tetra3 sample data)", "",
         "Reference solution: tetra3 (independent solver, Hipparcos with proper motion to 2023).",
         "Pixel scale follows from the tetra3 field of view; no lens distortion is modelled.", "",
         "| Image | FOV (deg) | Variant | Detected | Solved | Identified | IDs also in tetra3 | Boresight vs tetra3 (arcsec) | tetra3 stars, RMS residual under V.I.S.I.O.N. attitude (arcsec) | Own post-fit RMS (arcsec) | tetra3 RMSE (arcsec) |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in rows:
        if "note" in r:
            L.append(f"| {r['image']} | – | {r['note']} | | | | | | | | |")
            continue
        for variant in ("paper settings", "paper settings + gradient removal", "tuned (mag 6, 12 stars) + gradient removal"):
            v = r[variant]
            if not v["solved"]:
                L.append(f"| {r['image']} | {r['fov_deg']:.2f} | {variant} | {v['detected']} | no | – | – | – | – | – | {r['t3_rmse_arcsec']:.1f} |")
            else:
                L.append(f"| {r['image']} | {r['fov_deg']:.2f} | {variant} | {v['detected']} | yes | {v['identified']} | "
                         f"{v['ids_in_tetra3']}/{v['identified']} | {v['boresight_sep_arcsec']:.1f} | "
                         f"{v['t3_star_residual_rms_arcsec']:.1f} | {v['own_residual_rms_arcsec']:.1f} | {r['t3_rmse_arcsec']:.1f} |")
    with open(os.path.join(a.out, "real_images.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
