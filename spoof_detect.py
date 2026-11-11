"""
Experiment 6c (review revision): GNSS spoofing detection with visual fixes.

Paper L78/L203: "a GPS receiver can be treated as one more provider that must
pass this test, so that GPS positions inconsistent with the visual fixes can be
rejected".  Here that claim is quantified with REAL visual-fix error
distributions:
  - UAV-VisLoc (Exp 4c), 2-D priors 100-300 m, real onboard attitude;
  - VPAIR (Exp 4b), priors 45-91 m, attitude at 0.1 deg;
for the paper's 6-DOF solver and the gravity-constrained 4-DOF solver.
Missed fixes and confident false fixes are included exactly as measured.

Monitor (model-free): every visual fix (one per 10 s) gives the residual
r = GNSS position - visual position.  The alarm statistic is the norm of the
mean residual over the last N fixes; its threshold is set from the empirical
no-spoof distribution so that the false-alarm rate is 1 per flight hour.
The spoofer drags the GNSS position away in a random direction at a constant
rate (0.1-5 m/s; i.e. a slow ramp that a plain gate would not see at once).
GNSS noise: 3 m (1 sigma) per axis, assumed.

Limitations: visual errors are drawn independently per fix with random
direction (only magnitudes are stored), so temporal correlation of errors is
not modelled; GNSS is assumed otherwise nominal.
Usage:  python spoof_detect.py
"""

import json
import os

import numpy as np

FIX_DT = 10.0
GNSS_SIGMA = 3.0
FA_PER_HOUR = 1.0
RATES = [0.1, 0.25, 0.5, 1.0, 2.0, 5.0]
WINDOWS = [1, 3, 6] if "--robust" not in __import__("sys").argv else [3, 6]
T_MAX = 900.0
N_RUNS = 1000
OUT = "results/exp6c"


def empirical(rows, solver, offsets, match="search8"):
    """List of per-attempt outcomes: error magnitude (m) or None for no fix."""
    out = []
    for r in rows:
        if r["match"] == match and r["solver"] == solver and r["offset"] in offsets:
            out.append(r["err_m"] if r["success"] else None)
    return out


def draw(samples, rng, n):
    """n visual-fix error vectors (NaN where no fix)."""
    pick = [samples[i] for i in rng.integers(0, len(samples), n)]
    mag = np.array([np.nan if p is None else p for p in pick])
    ang = rng.uniform(0, 2 * np.pi, n)
    return np.c_[mag * np.cos(ang), mag * np.sin(ang)]


def residual_stream(samples, rng, n):
    e_vis = draw(samples, rng, n)
    e_gnss = rng.normal(0, GNSS_SIGMA, (n, 2))
    return e_gnss - e_vis                          # NaN rows = no visual fix


ROBUST = False      # True: component-wise median instead of mean (outlier-robust)


def window_stat(res, N):
    """Norm of the mean (or median, if ROBUST) of the last N available residuals."""
    stats = np.full(len(res), np.nan)
    buf = []
    for k, r in enumerate(res):
        if np.isfinite(r[0]):
            buf.append(r)
            buf = buf[-N:]
            if len(buf) == N:
                agg = np.median(buf, axis=0) if ROBUST else np.mean(buf, axis=0)
                stats[k] = np.linalg.norm(agg)
    return stats


def threshold(samples, N, rng, n=400_000):
    """Alarm threshold for FA_PER_HOUR, from the empirical no-spoof statistic."""
    s = window_stat(residual_stream(samples, rng, n), N)
    s = s[np.isfinite(s)]
    p_fa = FA_PER_HOUR * FIX_DT / 3600.0           # per test
    return float(np.quantile(s, 1 - p_fa)), float(np.max(s))


def detect(samples, N, tau, rate, rng):
    n = int(T_MAX / FIX_DT)
    t = (np.arange(n) + 1) * FIX_DT
    times, offsets = [], []
    for _ in range(N_RUNS):
        # warm-up of N fixes before onset so the window is full
        pre = residual_stream(samples, rng, 3 * N)
        res = residual_stream(samples, rng, n)
        ang = rng.uniform(0, 2 * np.pi)
        res = res + (rate * t)[:, None] * np.array([np.cos(ang), np.sin(ang)])
        stat = window_stat(np.vstack([pre, res]), N)[len(pre):]
        hit = np.flatnonzero(stat > tau)
        if hit.size:
            times.append(t[hit[0]])
            offsets.append(rate * t[hit[0]])
        else:
            times.append(np.inf)
            offsets.append(np.inf)
    times, offsets = np.array(times), np.array(offsets)
    return {"median_t_s": float(np.median(times)), "p95_t_s": float(np.percentile(times, 95)),
            "median_offset_m": float(np.median(offsets)), "p95_offset_m": float(np.percentile(offsets, 95)),
            "detected_by_120s_pct": float(100 * np.mean(times <= 120))}


def main():
    global OUT
    import sys
    if "--robust" in sys.argv:
        globals()["ROBUST"] = True
        OUT = OUT + "_robust"
    os.makedirs(OUT, exist_ok=True)
    rng = np.random.default_rng(66)
    vis = json.load(open("results/exp4c/rows.json"))
    vp = json.load(open("results/exp4b_rows.json"))
    monitors = {
        "UAV-VisLoc, 6-DOF PnP (paper)": empirical(vis, "6-DOF PnP (paper)", {100, 200, 300}),
        "UAV-VisLoc, 4-DOF gravity only (onboard tilt)": empirical(vis, "4-DOF gravity only, onboard tilt", {100, 200, 300}),
        "VPAIR, 6-DOF PnP (paper)": empirical(vp, "6-DOF PnP (paper)", {1, 2}),
        "VPAIR, 4-DOF gravity only (0.1 deg tilt)": empirical(vp, "4-DOF gravity only, calibrated INS (0.1 deg)", {1, 2}),
    }
    L = ["# Experiment 6c: GNSS spoofing detection with real visual-fix errors"
         + (" (robust: window median)" if ROBUST else " (window mean)"), "",
         f"Visual fix every {FIX_DT:.0f} s; GNSS noise {GNSS_SIGMA} m/axis; threshold for {FA_PER_HOUR:g} false alarm per hour "
         "(empirical, no-spoof); spoofing ramp in a random direction from t = 0; "
         f"{N_RUNS} runs per cell; 'inf' = not detected within {T_MAX:.0f} s.", "",
         "## Thresholds and fix statistics", "",
         "| Monitor | Attempts | Fix availability (%) | Fixes > 25 m (%) | N | Threshold (m) |",
         "| --- | --- | --- | --- | --- | --- |"]
    res_tab = ["", "## Detection (median time to alarm / median spoofed offset at alarm / detected within 120 s)", "",
               "| Monitor | N | " + " | ".join(f"{v} m/s" for v in RATES) + " |",
               "| --- | --- | " + " | ".join("---" for _ in RATES) + " |"]
    summary = {}
    for name, smp in monitors.items():
        av = 100 * np.mean([s is not None for s in smp])
        big = 100 * np.mean([s is not None and s > 25 for s in smp])
        for N in WINDOWS:
            tau, mx = threshold(smp, N, rng)
            L.append(f"| {name} | {len(smp)} | {av:.1f} | {big:.1f} | {N} | {tau:.1f} |")
            cells = []
            for v in RATES:
                d = detect(smp, N, tau, v, rng)
                summary[f"{name}|N={N}|{v}"] = {"threshold_m": tau, **d}
                cells.append(f"{d['median_t_s']:.0f} s / {d['median_offset_m']:.0f} m / {d['detected_by_120s_pct']:.0f}%")
            res_tab.append(f"| {name} | {N} | " + " | ".join(cells) + " |")
            print(res_tab[-1], flush=True)
    open(os.path.join(OUT, "exp6c.md"), "w").write("\n".join(L + res_tab) + "\n")
    json.dump(summary, open(os.path.join(OUT, "summary.json"), "w"), indent=1)
    print("\n".join(L))


if __name__ == "__main__":
    main()
