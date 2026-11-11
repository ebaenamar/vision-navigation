"""
V.I.S.I.O.N. experiments: produces every number, table and figure for the paper.

Usage (on your own computer, internet needed once to download Hipparcos):
    pip install numpy scipy matplotlib skyfield pandas astroquery
    python run_experiments.py                    # real run, Hipparcos catalogue
    python run_experiments.py --quick            # fast check that everything works
    python run_experiments.py --catalog synthetic --quick   # offline code test ONLY

Outputs go to ./results/ :
    results.md         all tables, ready to paste into the paper
    summary.json       every number in machine-readable form
    fig2_star_field.png, fig3_attitude_error.png, fig4_gyro_bias.png,
    fig5_noise_sweep.png
"""

import argparse
import json
import os
import platform
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from vision_core import (Camera, StarCatalog, StarIdentifier, LegacyPipeline,
                         render_star_image, detect_stars, random_rotation,
                         attitude_error_rad, attitude_error_axes_rad, angle_between_rad,
                         boresight_inertial, run_mekf)

ARCSEC = np.degrees(1.0) * 3600.0        # rad -> arcsec
SUCCESS_DEG = 0.1                         # a solution "succeeds" if boresight error < 0.1 deg


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--catalog", choices=["hipparcos", "synthetic"], default="hipparcos")
    p.add_argument("--frames", type=int, default=200, help="images per condition")
    p.add_argument("--sweep-frames", type=int, default=60, help="images per noise level")
    p.add_argument("--mc-runs", type=int, default=20, help="filter Monte Carlo runs")
    p.add_argument("--duration", type=float, default=3600.0, help="filter run length (s)")
    p.add_argument("--seed", type=int, default=2026)
    p.add_argument("--out", default="results")
    p.add_argument("--quick", action="store_true", help="small run to test the code")
    a = p.parse_args()
    if a.quick:
        a.frames, a.sweep_frames, a.mc_runs, a.duration = 20, 10, 3, 900.0
    return a


# ---------------------------------------------------------------------------
# Experiment 1: star identification and single-frame attitude
# ---------------------------------------------------------------------------

def id_error_count(out, xy, truth, max_px=2.0):
    """Number of identified detections that do NOT lie within max_px of a
    rendered catalogue star with the same Hipparcos number (false objects,
    blends and swapped identities all count as wrong)."""
    tx, ty, tid = np.asarray(truth["x"]), np.asarray(truth["y"]), np.asarray(truth["ids"])
    wrong = 0
    for d, hip in zip(out["det_idx"], out["cat_ids"]):
        dist = np.hypot(tx - xy[d, 0], ty - xy[d, 1])
        k = int(np.argmin(dist)) if dist.size else -1
        if k < 0 or dist[k] > max_px or tid[k] != hip:
            wrong += 1
    return wrong


def run_condition(name, cat, cam, ident, legacy, rng, frames, read_noise, n_false,
                  id_kwargs=None):
    id_kwargs = id_kwargs or {}
    rows = []
    for _ in range(frames):
        R_true = random_rotation(rng)
        img, truth = render_star_image(cat, R_true, cam, rng,
                                       read_noise=read_noise, n_false=n_false)
        xy, _ = detect_stars(img)
        bs_true = boresight_inertial(R_true)

        t0 = time.perf_counter()
        out = ident.identify(xy, **id_kwargs)
        t_new = time.perf_counter() - t0
        if out is not None:
            att = attitude_error_rad(out["R"], R_true)
            axes = attitude_error_axes_rad(out["R"], R_true)
            bor = angle_between_rad(boresight_inertial(out["R"]), bs_true)
            n_used = len(out["det_idx"])
            res = out["residuals_rad"]
            n_wrong = id_error_count(out, xy, truth)
        else:
            att = bor = np.nan
            axes = np.full(3, np.nan)
            n_used = 0
            res = np.array([])
            n_wrong = 0

        t0 = time.perf_counter()
        bs_old = legacy.solve(xy)
        t_old = time.perf_counter() - t0
        bor_old = angle_between_rad(bs_old, bs_true) if bs_old is not None else np.nan

        rows.append({"n_detected": len(xy), "n_used": n_used, "att": att, "bor": bor,
                     "bor_old": bor_old, "t_new": t_new, "t_old": t_old,
                     "residuals": res, "axes": axes, "n_wrong": n_wrong,
                     "solved": out is not None})
    return summarise(name, rows, read_noise, n_false)


def summarise(name, rows, read_noise, n_false):
    def ok(x):
        return np.isfinite(x) & (np.degrees(x) < SUCCESS_DEG)
    att = np.array([r["att"] for r in rows])
    bor = np.array([r["bor"] for r in rows])
    bor_old = np.array([r["bor_old"] for r in rows])
    solved = np.array([r["solved"] for r in rows])
    axes = np.array([r["axes"] for r in rows])
    good = ok(bor)
    n_wrong = sum(r["n_wrong"] for r, g in zip(rows, good) if g)
    n_ids = sum(r["n_used"] for r, g in zip(rows, good) if g)
    res = np.concatenate([r["residuals"] for r, g in zip(rows, good) if g] or [np.array([])])
    # Per-axis 1-sigma (RMS) over successful frames.
    ax_ok = axes[good] * ARCSEC
    return {
        "gross_wrong_solution_pct": float(100 * (solved & ~good).mean()),
        "no_solution_pct": float(100 * (~solved).mean()),
        "star_misid_pct": float(100 * n_wrong / n_ids) if n_ids else None,
        "rms_cross_boresight_arcsec": float(np.sqrt(np.mean(ax_ok[:, :2] ** 2))) if good.any() else None,
        "rms_roll_arcsec": float(np.sqrt(np.mean(ax_ok[:, 2] ** 2))) if good.any() else None,
        "condition": name, "read_noise": read_noise, "false_objects": n_false,
        "frames": len(rows),
        "mean_detected": float(np.mean([r["n_detected"] for r in rows])),
        "mean_identified": float(np.mean([r["n_used"] for r in rows if r["n_used"] > 0] or [0])),
        "success_new_pct": float(100 * good.mean()),
        "success_old_pct": float(100 * ok(bor_old).mean()),
        "att_err_median_arcsec": float(np.median(att[good]) * ARCSEC) if good.any() else None,
        "att_err_p95_arcsec": float(np.percentile(att[good], 95) * ARCSEC) if good.any() else None,
        "bor_err_median_arcsec": float(np.median(bor[good]) * ARCSEC) if good.any() else None,
        "bor_err_old_median_deg": float(np.nanmedian(np.degrees(bor_old))) if np.isfinite(bor_old).any() else None,
        "star_residual_rms_arcsec": float(np.sqrt(np.mean(res**2)) * ARCSEC) if res.size else None,
        "time_new_ms_median": float(1000 * np.median([r["t_new"] for r in rows])),
        "time_old_ms_median": float(1000 * np.median([r["t_old"] for r in rows])),
    }


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def fig_star_field(cat, cam, ident, rng, path):
    R_true = random_rotation(rng)
    img, _ = render_star_image(cat, R_true, cam, rng)
    xy, _ = detect_stars(img)
    out = ident.identify(xy)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(np.log10(np.clip(img, 1, None)), cmap="gray", origin="upper")
    ax.scatter(xy[:, 0], xy[:, 1], s=120, facecolors="none", edgecolors="tab:orange",
               linewidths=1.0, label="Detected")
    if out is not None:
        for d, hip in list(zip(out["det_idx"], out["cat_ids"]))[:12]:
            ax.annotate(f"{cat.source.split()[0][:3]} {hip}", xy[d], xytext=(8, -8),
                        textcoords="offset points", color="tab:cyan", fontsize=8)
        ax.scatter(xy[out["det_idx"], 0], xy[out["det_idx"], 1], s=200,
                   facecolors="none", edgecolors="tab:cyan", linewidths=1.0,
                   label="Identified")
    ax.set_title("Synthetic star field: detected and identified stars")
    ax.set_xlabel("x (pixels)")
    ax.set_ylabel("y (pixels)")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def fig_attitude(log, path, outage=None):
    t = log["t"] / 60.0
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.semilogy(t, log["err_gyro"] * ARCSEC, label="Gyro only (from one star fix)")
    ax.semilogy(t, log["err_mekf"] * ARCSEC, label="MEKF (gyro + star updates)")
    ax.semilogy(t, log["sigma3"] * ARCSEC, "k--", lw=0.8, label="MEKF 3σ bound")
    if outage is not None:
        ax.axvspan(outage[0] / 60, outage[1] / 60, color="0.85", label="Star outage (no star updates)")
    ax.set_xlabel("Time (min)")
    ax.set_ylabel("Attitude error (arcsec)")
    ax.set_title("Attitude error over time")
    ax.legend()
    ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def fig_bias(log, path):
    t = log["t"] / 60.0
    to_dph = np.degrees(1.0) * 3600.0
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for i, axis in enumerate("xyz"):
        line, = ax.plot(t, log["bias_hat"][:, i] * to_dph, label=f"estimated {axis}")
        ax.plot(t, log["bias_true"][:, i] * to_dph, "--", color=line.get_color(),
                lw=0.8, label=f"true {axis}")
    ax.set_xlabel("Time (min)")
    ax.set_ylabel("Gyro bias (deg/h)")
    ax.set_title("Gyro bias estimation")
    ax.legend(ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def fig_sweep(sweep, path):
    noise = [s["read_noise"] for s in sweep]
    fig, ax1 = plt.subplots(figsize=(8, 4.5))
    ax1.plot(noise, [s["success_new_pct"] for s in sweep], "o-", label="Corrected pipeline")
    ax1.plot(noise, [s["success_old_pct"] for s in sweep], "s--", label="Recovered early draft")
    ax1.set_xlabel("Read noise (counts, 1σ)")
    ax1.set_ylabel(f"Success rate (%), boresight error < {SUCCESS_DEG}°")
    ax1.set_ylim(-5, 105)
    ax1.grid(True, alpha=0.3)
    ax2 = ax1.twinx()
    med = [s["att_err_median_arcsec"] if s["att_err_median_arcsec"] is not None else np.nan
           for s in sweep]
    ax2.plot(noise, med, "^:", color="tab:green", label="Median attitude error (corrected)")
    ax2.set_ylabel("Median attitude error (arcsec)")
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc="center left")
    ax1.set_title("Robustness to image noise")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def fmt(x, nd=1):
    return "–" if x is None else f"{x:.{nd}f}"


def main():
    a = parse_args()
    os.makedirs(a.out, exist_ok=True)
    rng = np.random.default_rng(a.seed)

    print("Loading catalogue...")
    full = (StarCatalog.from_hipparcos(mag_limit=7.0) if a.catalog == "hipparcos"
            else StarCatalog.synthetic(seed=a.seed))
    if a.catalog == "synthetic":
        print("!! SYNTHETIC CATALOGUE: for testing the code only. Do NOT report these numbers.")
    cam = Camera(width=1024, height=1024, fov_deg=20.0)
    params = {"catalog": full.source, "camera_px": "1024 x 1024", "fov_deg": cam.fov_deg,
              "pixel_scale_arcsec": round(cam.pixel_scale_arcsec, 2),
              "render_mag_limit": 6.5, "id_mag_limit": 5.5, "psf_sigma_px": 1.2,
              "background_counts": 10.0, "read_noise_nominal": 2.0,
              "detection_k_sigma": 5.0, "pair_tolerance_deg": 0.02,
              "refine_tolerance_deg": 0.03, "legacy_catalog_stars": 1000,
              "legacy_tolerance_deg": 0.5, "seed": a.seed,
              "computer": f"{platform.processor() or platform.machine()} / {platform.system()}"}

    print("Building star-pair tables...")
    ident = StarIdentifier(full, cam, id_mag_limit=5.5, tol_deg=0.02)
    legacy = LegacyPipeline(full, cam, n_stars=1000, tol_deg=0.5)
    params["id_catalog_stars"] = len(ident.cat)
    params["id_catalog_pairs"] = ident.n_pairs

    # ---- Experiment 1
    print("Experiment 1: star identification...")
    wide = {"max_stars": 12, "max_triplets": 220}     # all triplets of the 12 brightest
    conds = [("Nominal noise", 2.0, 0, None), ("High noise", 8.0, 0, None),
             ("Nominal + 5 false objects", 2.0, 5, None),
             ("Nominal + 5 false objects, 12-star search", 2.0, 5, wide)]
    table2 = [run_condition(n, full, cam, ident, legacy, rng, a.frames, rn, nf, kw)
              for n, rn, nf, kw in conds]
    for row in table2:
        print("  ", row["condition"], f"success {row['success_new_pct']:.1f}% (original {row['success_old_pct']:.1f}%)")

    print("Noise sweep...")
    sweep = [run_condition(f"noise {rn}", full, cam, ident, legacy, rng, a.sweep_frames, rn, 0)
             for rn in [2, 4, 8, 16, 24, 32]]

    # ---- Experiment 2
    rms = table2[0]["star_residual_rms_arcsec"]
    sigma_star = np.radians((rms if rms else 10.0) / np.sqrt(2) / 3600.0)  # per-axis
    bright = full.subset(mag_limit=5.5)
    outage_len = min(600.0, 0.3 * a.duration)
    outage = (a.duration * 0.4, a.duration * 0.4 + outage_len)
    print(f"Experiment 2: MEKF, {a.mc_runs} runs, star noise {np.degrees(sigma_star)*3600:.1f} arcsec/axis...")
    finals, finals_go, maxes_outage, maxes_outage_go, bias_err, consist = [], [], [], [], [], []
    example, example_out = None, None
    for m in range(a.mc_runs):
        L = run_mekf(bright, cam, rng, duration_s=a.duration, sigma_star_rad=sigma_star)
        Lo = run_mekf(bright, cam, rng, duration_s=a.duration, sigma_star_rad=sigma_star,
                      star_outage=outage)
        finals.append(L["err_mekf"][-1])
        finals_go.append(L["err_gyro"][-1])
        settled = L["t"] > 600
        consist.append(np.mean(L["err_mekf"][settled] < L["sigma3"][settled]))
        bias_err.append(np.linalg.norm(L["bias_hat"][-1] - L["bias_true"][-1]))
        win = (Lo["t"] >= outage[0]) & (Lo["t"] < outage[1])
        maxes_outage.append(Lo["err_mekf"][win].max())
        if m == 0:
            example, example_out = L, Lo

    to_dph = np.degrees(1.0) * 3600.0
    table3 = {
        "runs": a.mc_runs, "duration_s": a.duration,
        "sigma_star_arcsec_per_axis": float(np.degrees(sigma_star) * 3600),
        "gyro_arw_rad_per_sqrt_s": float(np.sqrt(10) * 1e-7),
        "gyro_rrw_rad_per_s15": float(np.sqrt(10) * 1e-10),
        "initial_bias_deg_h": 1.0, "body_rate_deg_s": 0.1, "initial_error_deg": 0.5,
        "final_err_gyro_median_arcsec": float(np.median(finals_go) * ARCSEC),
        "final_err_mekf_median_arcsec": float(np.median(finals) * ARCSEC),
        "final_bias_err_median_deg_h": float(np.median(bias_err) * to_dph),
        "pct_time_within_3sigma": float(100 * np.mean(consist)),
        "outage_s": outage_len,
        "outage_max_err_mekf_median_arcsec": float(np.median(maxes_outage) * ARCSEC),
    }

    # ---- Experiment 2b: realistic sensors (added in review revision)
    # Gyro: ADIS16448 MEMS IMU noise as calibrated for the real EuRoC MAV
    # dataset (Burri et al., IJRR 2016; Kalibr imu.yaml): gyroscope noise
    # density 1.6968e-4 rad/s/sqrt(Hz), bias random walk 1.9393e-5 rad/s^2/sqrt(Hz).
    # Initial bias 14.5 deg/h = ADIS16448 datasheet in-run bias stability.
    # Star tracker bias: constant per run, 3 arcsec cross-boresight and 15 arcsec
    # roll (1 sigma), the order of the bias + LSFE terms in small-satellite
    # tracker datasheets (e.g. Sodern Hydra, BCT NST).
    deg = np.pi / 180
    scen = [
        ("Paper gyro (textbook FOG-grade), white star noise", {}),
        ("Paper gyro, body rate 0 (no bias averaging)", {"rate_deg_s": 0.0}),
        ("MEMS gyro (EuRoC ADIS16448), white star noise",
         {"sigma_v": 1.6968e-4, "sigma_u": 1.9393e-5, "bias0_deg_h": 14.5}),
        ("MEMS gyro + star-tracker bias (3/3/15 arcsec)",
         {"sigma_v": 1.6968e-4, "sigma_u": 1.9393e-5, "bias0_deg_h": 14.5,
          "star_bias_sigma_rad": (3 / 3600 * deg, 3 / 3600 * deg, 15 / 3600 * deg)}),
    ]
    table3b = []
    for name, kw in scen:
        fin, fgo, mout, nees = [], [], [], []
        for m in range(a.mc_runs):
            L2 = run_mekf(bright, cam, rng, duration_s=a.duration, sigma_star_rad=sigma_star, **kw)
            Lo2 = run_mekf(bright, cam, rng, duration_s=a.duration, sigma_star_rad=sigma_star,
                           star_outage=outage, **kw)
            fin.append(L2["err_mekf"][-1]); fgo.append(L2["err_gyro"][-1])
            win = (Lo2["t"] >= outage[0]) & (Lo2["t"] < outage[1])
            mout.append(Lo2["err_mekf"][win].max())
            nees.append(L2["nees"][L2["t"] > 600])
        nees = np.array(nees)                     # (runs, samples)
        anees = float(np.mean(nees))              # average NEES, 3 dof expected
        from scipy.stats import chi2
        n = a.mc_runs
        lo, hi = chi2.ppf(0.025, 3 * n) / n, chi2.ppf(0.975, 3 * n) / n
        inside = float(100 * np.mean((nees.mean(axis=0) >= lo) & (nees.mean(axis=0) <= hi)))
        table3b.append({"scenario": name,
                        "final_err_median_arcsec": float(np.median(fin) * ARCSEC),
                        "gyro_only_final_median_arcsec": float(np.median(fgo) * ARCSEC),
                        "outage_max_median_arcsec": float(np.median(mout) * ARCSEC),
                        "anees": anees, "nees_bounds": (float(lo), float(hi)),
                        "pct_time_nees_in_bounds": inside})
        print("  ", name, table3b[-1])

    # ---- Figures
    print("Figures...")
    fig_star_field(full, cam, ident, rng, os.path.join(a.out, "fig2_star_field.png"))
    fig_attitude(example, os.path.join(a.out, "fig3_attitude_error.png"))
    fig_attitude(example_out, os.path.join(a.out, "fig3b_attitude_error_outage.png"), outage)
    fig_bias(example, os.path.join(a.out, "fig4_gyro_bias.png"))
    fig_sweep(sweep, os.path.join(a.out, "fig5_noise_sweep.png"))

    # ---- Write results
    summary = {"parameters": params, "table2": table2, "noise_sweep": sweep, "table3": table3, "table3b": table3b,
               "run_date": time.strftime("%Y-%m-%d %H:%M"), "quick": a.quick}
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2, default=float)

    L = []
    if a.catalog == "synthetic" or a.quick:
        L.append("> WARNING: test run (synthetic catalogue or --quick). Do not report these numbers.\n")
    L.append("## Table 1. Simulation parameters\n")
    L.append("| Parameter | Value |\n| --- | --- |")
    for k, v in params.items():
        L.append(f"| {k} | {v} |")
    L.append(f"| filter gyro ARW (rad/√s) | {table3['gyro_arw_rad_per_sqrt_s']:.2e} |")
    L.append(f"| filter gyro bias RW (rad/s^1.5) | {table3['gyro_rrw_rad_per_s15']:.2e} |")
    L.append(f"| filter star noise (arcsec/axis) | {table3['sigma_star_arcsec_per_axis']:.1f} |")
    L.append(f"| filter runs × duration | {a.mc_runs} × {a.duration:.0f} s |")

    L.append("\n## Table 2. Star identification and single-frame attitude\n")
    L.append("| Condition | Frames | Stars detected (mean) | Stars identified (mean) | Success, corrected (%) | Success, early draft (%) | Attitude error median (arcsec) | Attitude error 95th pct (arcsec) | Boresight error median (arcsec) | Early-draft boresight error median (deg) | Time/frame corrected (ms) |")
    L.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for r in table2:
        L.append(f"| {r['condition']} | {r['frames']} | {fmt(r['mean_detected'])} | {fmt(r['mean_identified'])} | "
                 f"{fmt(r['success_new_pct'])} | {fmt(r['success_old_pct'])} | {fmt(r['att_err_median_arcsec'])} | "
                 f"{fmt(r['att_err_p95_arcsec'])} | {fmt(r['bor_err_median_arcsec'])} | {fmt(r['bor_err_old_median_deg'])} | "
                 f"{fmt(r['time_new_ms_median'])} |")
    L.append(f"\nSuccess = boresight error below {SUCCESS_DEG}°. Per-star residual RMS (nominal): "
             f"{fmt(table2[0]['star_residual_rms_arcsec'])} arcsec.")
    L.append("\n## Table 2b. Identification scored on catalogue IDs, per-axis accuracy (1σ RMS)\n")
    L.append("| Condition | Success (%) | Gross wrong solution (%) | No solution (%) | Misidentified stars in successful frames (%) | Cross-boresight 1σ (arcsec) | Roll 1σ (arcsec) | Time/frame (ms) |")
    L.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for r in table2:
        L.append(f"| {r['condition']} | {fmt(r['success_new_pct'])} | {fmt(r['gross_wrong_solution_pct'])} | "
                 f"{fmt(r['no_solution_pct'])} | {fmt(r['star_misid_pct'], 2)} | "
                 f"{fmt(r['rms_cross_boresight_arcsec'])} | "
                 f"{fmt(r['rms_roll_arcsec'])} | {fmt(r['time_new_ms_median'])} |")

    L.append("\n## Table 3. Filter performance (medians over runs)\n")
    L.append("| Method | Final attitude error (arcsec) | Max error during star outage (arcsec) | Final bias error (deg/h) |")
    L.append("| --- | --- | --- | --- |")
    L.append(f"| Gyro only (from one star fix) | {fmt(table3['final_err_gyro_median_arcsec'])} | – | – |")
    L.append(f"| MEKF with star updates | {fmt(table3['final_err_mekf_median_arcsec'], 2)} | {fmt(table3['outage_max_err_mekf_median_arcsec'])} | {fmt(table3['final_bias_err_median_deg_h'], 4)} |")
    L.append(f"\nMEKF error inside its 3σ bound {fmt(table3['pct_time_within_3sigma'])}% of the time after the first 10 minutes.")

    L.append("\n## Table 3b. Filter with realistic sensors (medians over runs; NEES after 10 min)\n")
    L.append("| Scenario | Final attitude error (arcsec) | Gyro-only final (arcsec) | Max error in 10-min outage (arcsec) | Average NEES (3 = consistent) | 95% NEES bounds | Time with ensemble NEES inside bounds (%) |")
    L.append("| --- | --- | --- | --- | --- | --- | --- |")
    for r in table3b:
        L.append(f"| {r['scenario']} | {fmt(r['final_err_median_arcsec'], 2)} | {fmt(r['gyro_only_final_median_arcsec'])} | "
                 f"{fmt(r['outage_max_median_arcsec'])} | {fmt(r['anees'], 2)} | "
                 f"{r['nees_bounds'][0]:.2f}–{r['nees_bounds'][1]:.2f} | {fmt(r['pct_time_nees_in_bounds'])} |")

    L.append("\n## Noise sweep (Figure 5)\n")
    L.append("| Read noise | Success corrected (%) | Success early draft (%) | Median attitude error (arcsec) |")
    L.append("| --- | --- | --- | --- |")
    for s in sweep:
        L.append(f"| {s['read_noise']} | {fmt(s['success_new_pct'])} | {fmt(s['success_old_pct'])} | {fmt(s['att_err_median_arcsec'])} |")

    with open(os.path.join(a.out, "results.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    print(f"\nDone. Results written to {a.out}/results.md and figures saved in {a.out}/")


if __name__ == "__main__":
    main()
