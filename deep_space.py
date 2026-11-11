"""
V.I.S.I.O.N. Experiment 5: deep-space position fixes on an Earth-to-Mars transfer.

Once a day along a Hohmann transfer orbit, the spacecraft position is estimated
from a single epoch of measurements ("snapshot" fixes, no dynamics filter):

  planets : camera bearings to Venus, Earth, Mars and Jupiter (Section 5.9),
            solved by Gauss-Newton; bodies within SUN_EXCLUSION_DEG of the Sun
            are not observable.
  xnav    : pulse-arrival ranges to four millisecond pulsars, solving for
            position and clock offset (Section 5.9, optional provider).
  fused   : both measurement sets in one weighted least-squares solution.

Planet positions use JPL's approximate Keplerian elements (Standish), built in,
so nothing has to be downloaded. Truth and estimate use the same ephemeris, so
ephemeris error is not studied.

Usage:
    python deep_space.py            # full run
    python deep_space.py --quick
Outputs in results/: exp5.md, exp5_summary.json, fig_exp5_error_vs_time.png
"""

import argparse
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AU = 149_597_870.7          # km
MU_SUN = 1.32712440018e11   # km^3/s^2
DAY = 86400.0
ARCSEC = math.radians(1 / 3600)
C_KM_S = 299_792.458

# JPL approximate elements, J2000 ecliptic, valid 1800-2050 (Standish).
# a [AU], e, I [deg], L [deg], long. perihelion [deg], long. node [deg], L rate [deg/century]
ELEMENTS = {
    "Venus":   (0.72333566, 0.00677672, 3.39467605, 181.97909950, 131.60246718, 76.67984255, 58517.81538729),
    "Earth":   (1.00000261, 0.01671123, -0.00001531, 100.46457166, 102.93768193, 0.0, 35999.37244981),
    "Mars":    (1.52371034, 0.09339410, 1.84969142, -4.55343205, -23.94362959, 49.55953891, 19140.30268499),
    "Jupiter": (5.20288700, 0.04838624, 1.30439695, 34.39644051, 14.72847983, 100.47390909, 3034.74612775),
}

# Millisecond pulsars of the kind used by SEXTANT (approximate ICRS RA, Dec in degrees).
PULSARS = {
    "J0437-4715": (69.32, -47.25),
    "J0030+0451": (7.61, 4.86),
    "B1937+21": (294.91, 21.58),
    "B1821-24": (276.13, -24.87),
}
OBLIQUITY = math.radians(23.43928)

SUN_EXCLUSION_DEG = 20.0


# ---------------------------------------------------------------------------
# Ephemeris and trajectory
# ---------------------------------------------------------------------------

def solve_kepler(M, e):
    E = M if e < 0.8 else math.pi
    for _ in range(30):
        E -= (E - e * math.sin(E) - M) / (1 - e * math.cos(E))
    return E


def planet_position(name, t_days):
    """Heliocentric ecliptic position (km) at t days after J2000."""
    a, e, inc, L0, varpi, node, Ldot = ELEMENTS[name]
    T = t_days / 36525.0
    L = math.radians(L0 + Ldot * T)
    w = math.radians(varpi - node)
    Om, I = math.radians(node), math.radians(inc)
    M = (L - math.radians(varpi) + math.pi) % (2 * math.pi) - math.pi
    E = solve_kepler(M, e)
    xp = a * (math.cos(E) - e)
    yp = a * math.sqrt(1 - e * e) * math.sin(E)
    cw, sw, cO, sO, cI, sI = math.cos(w), math.sin(w), math.cos(Om), math.sin(Om), math.cos(I), math.sin(I)
    x = (cw * cO - sw * sO * cI) * xp + (-sw * cO - cw * sO * cI) * yp
    y = (cw * sO + sw * cO * cI) * xp + (-sw * sO + cw * cO * cI) * yp
    z = (sw * sI) * xp + (cw * sI) * yp
    return np.array([x, y, z]) * AU


def hohmann_position(t_since_dep_days, dep_dir, r1=None, r2=None):
    """Heliocentric position on an Earth(1 AU)-to-Mars(1.524 AU) Hohmann ellipse
    starting at perihelion in direction dep_dir (ecliptic plane).
    r1, r2 (km) override the radii (used by the phased option)."""
    r1 = 1.0 * AU if r1 is None else r1
    r2 = ELEMENTS["Mars"][0] * AU if r2 is None else r2
    a = 0.5 * (r1 + r2)
    e = (r2 - r1) / (r2 + r1)
    n = math.sqrt(MU_SUN / a ** 3)
    M = n * t_since_dep_days * DAY
    E = solve_kepler(M, e)
    nu = 2 * math.atan2(math.sqrt(1 + e) * math.sin(E / 2), math.sqrt(1 - e) * math.cos(E / 2))
    r = a * (1 - e * math.cos(E))
    u1 = np.asarray(dep_dir, dtype=float) / np.linalg.norm(dep_dir)
    u2 = np.cross([0.0, 0.0, 1.0], u1)               # prograde; equals the original for z = 0
    u2 /= np.linalg.norm(u2)
    return r * (math.cos(nu) * u1 + math.sin(nu) * u2), math.pi / n / DAY


def pulsar_dirs_ecliptic():
    out = []
    for ra, dec in PULSARS.values():
        ra, dec = math.radians(ra), math.radians(dec)
        v = np.array([math.cos(dec) * math.cos(ra), math.cos(dec) * math.sin(ra), math.sin(dec)])
        c, s = math.cos(OBLIQUITY), math.sin(OBLIQUITY)
        out.append(np.array([v[0], c * v[1] + s * v[2], -s * v[1] + c * v[2]]))
    return np.array(out)


# ---------------------------------------------------------------------------
# Measurements and estimation
# ---------------------------------------------------------------------------

def noisy_unit(u, sigma_rad, rng):
    """Perturb unit vector u by a small random angle (per-axis sigma)."""
    a = np.cross(u, [0, 0, 1.0])
    if np.linalg.norm(a) < 1e-6:
        a = np.cross(u, [1.0, 0, 0])
    a /= np.linalg.norm(a)
    b = np.cross(u, a)
    v = u + sigma_rad * (rng.normal() * a + rng.normal() * b)
    return v / np.linalg.norm(v)


def visible_bodies(r_sc, t_days):
    vis = []
    for name in ELEMENTS:
        rp = planet_position(name, t_days)
        to_body = rp - r_sc
        to_sun = -r_sc
        ang = math.degrees(math.acos(np.clip(np.dot(to_body, to_sun) /
                                              (np.linalg.norm(to_body) * np.linalg.norm(to_sun)), -1, 1)))
        if ang > SUN_EXCLUSION_DEG:
            vis.append((name, rp))
    return vis


def solve(bodies, u_meas, sig_b, pulsar_n, ranges, sig_x, x0, use_b, use_x, iters=10):
    """Weighted Gauss-Newton for state [r (km), clock offset c*dt (km)].
    Bearing residuals in radians (weight 1/sig_b); pulsar residuals in km (1/sig_x)."""
    x = np.append(x0, 0.0)
    for _ in range(iters):
        rows, res = [], []
        if use_b:
            for (name, rp), um in zip(bodies, u_meas):
                d = rp - x[:3]
                rho = np.linalg.norm(d)
                u = d / rho
                J = np.zeros((3, 4))
                J[:, :3] = -(np.eye(3) - np.outer(u, u)) / rho
                rows.append(J / sig_b)
                res.append((um - u) / sig_b)
        if use_x:
            for n, z in zip(pulsar_n, ranges):
                rows.append(np.append(n, 1.0)[None, :] / sig_x)
                res.append(np.array([z - (n @ x[:3] + x[3])]) / sig_x)
        A, y = np.vstack(rows), np.concatenate(res)
        if not use_x:                      # clock offset unobservable from bearings alone
            A = A[:, :3]
        dx, *_ = np.linalg.lstsq(A, y, rcond=None)
        if not use_x:
            dx = np.append(dx, 0.0)
        x += dx
        if np.linalg.norm(dx[:3]) < 1e-3:
            break
    return x[:3]


def phased_departure(t_lo=9000.0, t_hi=9800.0):
    """Departure day (after J2000) for which Mars is at the transfer's arrival
    point, so the Mars approach phase is actually simulated (review revision)."""
    # The arrival point is placed exactly at Mars; the departure point is the
    # opposite apsis at Earth's distance, and the departure day is chosen to put
    # it as close to Earth as possible (Mars's inclination prevents an exact
    # coplanar Hohmann match at both ends).
    best = None
    for t0 in np.arange(t_lo, t_hi, 0.5):
        e = planet_position("Earth", t0)
        r1, r2 = np.linalg.norm(e), ELEMENTS["Mars"][0] * AU
        for _ in range(6):
            _, tof = hohmann_position(0, [1.0, 0, 0], r1, r2)
            m = planet_position("Mars", t0 + tof)
            r2 = np.linalg.norm(m)
        u1 = -m / r2
        dep_miss = np.linalg.norm(r1 * u1 - e)
        if best is None or dep_miss < best[1]:
            best = (t0, dep_miss, r1, r2, u1)
    return best


def run(sig_b_arcsec, sig_x_km, n_days_step, rng, prior_km=50_000.0, t0_days=9000.0,
        min_body_km=0.0, radii=(None, None), dep_override=None):
    """One pass along the transfer. Returns per-epoch errors (km) for each method.
    min_body_km: skip epochs closer than this to any planet centre."""
    dep_dir = planet_position("Earth", t0_days)
    dep_dir = dep_dir / np.linalg.norm(dep_dir)
    if dep_override is not None:
        dep_dir = dep_override
    _, tof = hohmann_position(0, dep_dir, *radii)
    pulsar_n = pulsar_dirs_ecliptic()
    sig_b = sig_b_arcsec * ARCSEC
    rec = {"t": [], "planets": [], "xnav": [], "fused": [], "n_bodies": [], "nearest_au": []}
    for td in np.arange(1.0, tof, n_days_step):
        t = t0_days + td
        r, _ = hohmann_position(td, dep_dir, *radii)
        bodies = visible_bodies(r, t)
        if bodies and min(np.linalg.norm(rp - r) for _, rp in bodies) < min_body_km:
            continue
        u_meas = [noisy_unit((rp - r) / np.linalg.norm(rp - r), sig_b, rng) for _, rp in bodies]
        clock = rng.normal(0, 300.0)                         # c * clock offset, km
        ranges = [n @ r + clock + rng.normal(0, sig_x_km) for n in pulsar_n]
        x0 = r + rng.normal(0, prior_km / math.sqrt(3), 3)  # coarse prior position
        rec["t"].append(td)
        rec["n_bodies"].append(len(bodies))
        rec["nearest_au"].append(min(np.linalg.norm(rp - r) for _, rp in bodies) / AU if bodies else np.nan)
        if len(bodies) >= 2:
            rec["planets"].append(np.linalg.norm(solve(bodies, u_meas, sig_b, pulsar_n, ranges, sig_x_km, x0, True, False) - r))
        else:
            rec["planets"].append(np.nan)
        rec["xnav"].append(np.linalg.norm(solve(bodies, u_meas, sig_b, pulsar_n, ranges, sig_x_km, x0, False, True) - r))
        rec["fused"].append(np.linalg.norm(solve(bodies, u_meas, sig_b, pulsar_n, ranges, sig_x_km, x0, len(bodies) > 0, True) - r))
    return {k: np.array(v, dtype=float) for k, v in rec.items()}, tof


def stats(err):
    ok = np.isfinite(err)
    if not ok.any():
        return {"availability_pct": 0.0, "median_km": None, "p95_km": None}
    return {"availability_pct": 100 * float(ok.mean()),
            "median_km": float(np.median(err[ok])), "p95_km": float(np.percentile(err[ok], 95))}


def fmt(x):
    return "–" if x is None else f"{x:,.0f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--passes", type=int, default=20, help="Monte Carlo passes per setting")
    ap.add_argument("--out", default="results")
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--phased", action="store_true",
                    help="choose the departure so that Mars is at the arrival point (review revision)")
    a = ap.parse_args()
    t0, min_body, radii, dep = 9000.0, 0.0, (None, None), None
    if a.phased:
        t0, miss, r1, r2, dep = phased_departure()
        radii = (r1, r2)
        min_body = 100_000.0     # stop fixes inside 100,000 km of a planet centre
        print(f"Phased departure: day {t0:.1f} after J2000, departure point {miss:,.0f} km from Earth; "
              "arrival exactly at Mars")
    if a.quick:
        a.passes = 3
    os.makedirs(a.out, exist_ok=True)

    nominal = dict(sig_b=13.4, sig_x=5.0)   # Exp 1 per-axis star noise; XNAV per-pulsar range noise (assumed)
    settings = [("Nominal", 13.4, 5.0),
                ("Better camera (5 arcsec)", 5.0, 5.0),
                ("Worse camera (40 arcsec)", 40.0, 5.0),
                ("Better XNAV (2 km)", 13.4, 2.0),
                ("Worse XNAV (20 km)", 13.4, 20.0)]
    rows, example, tof = [], None, None
    for k, (name, sb, sx) in enumerate(settings):
        rng = np.random.default_rng([a.seed, k])
        acc = {m: [] for m in ("planets", "xnav", "fused")}
        nb = []
        for p in range(a.passes):
            rec, tof = run(sb, sx, 1.0, rng, t0_days=t0, min_body_km=min_body, radii=radii, dep_override=dep)
            for m in acc:
                acc[m].append(rec[m])
            nb.append(rec["n_bodies"])
            if example is None:
                example = rec
        row = {"setting": name, "sigma_bearing_arcsec": sb, "sigma_pulsar_km": sx}
        for m in acc:
            row[m] = stats(np.concatenate(acc[m]))
        fused, p_err, x_err = (np.concatenate(acc[m]) for m in ("fused", "planets", "xnav"))
        both = np.isfinite(p_err)
        row["fused_better_than_both_pct"] = 100 * float(np.mean((fused[both] < p_err[both]) & (fused[both] < x_err[both])))
        row["mean_visible_bodies"] = float(np.mean(np.concatenate(nb)))
        rows.append(row)
        print(f"{name}: planets median {fmt(row['planets']['median_km'])} km "
              f"({row['planets']['availability_pct']:.0f}% of days), xnav {fmt(row['xnav']['median_km'])} km, "
              f"fused {fmt(row['fused']['median_km'])} km")

    # ---- figure (nominal, first pass)
    fig, ax1 = plt.subplots(figsize=(8.5, 4.8))
    ax1.semilogy(example["t"], example["planets"], label="Planets only", lw=1)
    ax1.semilogy(example["t"], example["xnav"], label="XNAV only", lw=1)
    ax1.semilogy(example["t"], example["fused"], label="Fused", lw=1.2)
    ax1.axhline(5000, color="k", ls=":", label="C5a target (5,000 km)")
    ax1.set_xlabel("Days since departure")
    ax1.set_ylabel("Position error (km)")
    ax1.grid(True, which="both", alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(example["t"], example["nearest_au"], color="0.6", ls="--", lw=1, label="Distance to nearest visible body")
    ax2.set_ylabel("Distance to nearest visible body (AU)")
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "fig_exp5_error_vs_time.png"), dpi=200)
    plt.close(fig)

    # ---- report
    L = ["# Experiment 5: Deep-space position fixes on an Earth–Mars transfer", ""]
    if a.quick:
        L.append("> Test run. Do not report.\n")
    L += [f"Hohmann transfer of {tof:.0f} days; one snapshot fix per day; {a.passes} Monte Carlo passes per setting. "
          f"Bodies within {SUN_EXCLUSION_DEG:.0f}° of the Sun are unobservable. "
          "Planet-only fixes need at least two visible bodies.", "",
          "| Setting | Bearing noise (arcsec/axis) | Pulsar range noise (km) | Planets: available days (%) | Planets: median (km) | Planets: 95th pct (km) | XNAV: median (km) | XNAV: 95th pct (km) | Fused: median (km) | Fused: 95th pct (km) | Fused better than both (% of days) | Mean visible bodies |",
          "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in rows:
        L.append(f"| {r['setting']} | {r['sigma_bearing_arcsec']} | {r['sigma_pulsar_km']} | "
                 f"{r['planets']['availability_pct']:.0f} | {fmt(r['planets']['median_km'])} | {fmt(r['planets']['p95_km'])} | "
                 f"{fmt(r['xnav']['median_km'])} | {fmt(r['xnav']['p95_km'])} | {fmt(r['fused']['median_km'])} | "
                 f"{fmt(r['fused']['p95_km'])} | {r['fused_better_than_both_pct']:.0f} | {r['mean_visible_bodies']:.1f} |")
    L += ["", "C5a: planet-only median ≤ 5,000 km. C5b: fused error lower than both planet-only and XNAV-only.",
          "Assumptions: snapshot fixes (no dynamics filter); coarse prior within ~50,000 km; pulsar phase ambiguity assumed resolved; "
          "pulsar range noise is an assumed per-day value anchored to SEXTANT's below-10 km result; ephemeris error not modelled."]
    with open(os.path.join(a.out, "exp5.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    with open(os.path.join(a.out, "exp5_summary.json"), "w") as f:
        json.dump({"transfer_days": tof, "rows": rows}, f, indent=2)
    print(f"\nDone. See {a.out}/exp5.md")


if __name__ == "__main__":
    main()
