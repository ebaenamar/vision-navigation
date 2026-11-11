"""Experiment 3b (review revision): celestial Earth fix with a fuller error model.

Adds to Experiment 3 the error terms that the original simulation left out:
  - the true vertical is the plumb line: WGS-84 ellipsoid normal plus a random
    deflection of the vertical (10 arcsec per axis, about the global RMS);
    the method therefore returns astronomical (approximately geodetic)
    coordinates, and errors are measured on the WGS-84 ellipsoid;
  - camera-to-accelerometer misalignment that remains after calibration;
  - residual horizontal vehicle acceleration (the accelerometer measures
    specific force, not gravity, in flight);
  - Kuhlmann et al.'s separate pitch (1.1 deg) and roll (2.0 deg) spreads;
  - a realistic star limit for a phone-class camera (magnitude 4.0);
  - the attitude is checked against truth (success = error below 0.1 deg).

Run:  python earth_fix_realistic.py [--runs 200]
Writes results/exp3b.md
"""

import argparse
from pathlib import Path

import numpy as np
from skyfield.api import load
from skyfield.framelib import itrs

from vision_core import (StarCatalog, StarIdentifier, attitude_error_rad,
                         detect_stars, render_star_image, rotm)
from earth_fix import (CAMERA, CLOCK_OFFSET_S, ACCEL_NOISE_SIGMA, make_camera_attitude,
                       random_utc_2026)

G = 9.80665
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
ARCSEC = np.radians(1 / 3600)
DEG = np.radians(1.0)


def normal_ecef(lat, lon):
    """Unit normal of the WGS-84 ellipsoid at geodetic (lat, lon)."""
    c = np.cos(lat)
    return np.array([c * np.cos(lon), c * np.sin(lon), np.sin(lat)])


def enu_basis(lat, lon):
    e = np.array([-np.sin(lon), np.cos(lon), 0.0])
    n = np.array([-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)])
    return e, n


def ellipsoid_error_m(lat, lon, lat_est, lon_est):
    """Horizontal error on WGS-84 using the local radii of curvature."""
    s2 = np.sin(lat) ** 2
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * s2)
    M = WGS84_A * (1 - WGS84_E2) / (1 - WGS84_E2 * s2) ** 1.5
    dlon = (lon_est - lon + np.pi) % (2 * np.pi) - np.pi
    return float(np.hypot(M * (lat_est - lat), N * np.cos(lat) * dlon))


def one_fix(rng, ts, catalog, ident, sc):
    lat = np.arcsin(rng.uniform(-1, 1))      # geodetic latitude (uniform in sin: adequate here)
    lon = rng.uniform(-np.pi, np.pi)
    t = random_utc_2026(rng, ts)

    # True plumb line = ellipsoid normal tilted by the deflection of the vertical.
    e_hat, n_hat = enu_basis(lat, lon)
    up = normal_ecef(lat, lon)
    xi, eta = rng.normal(0, sc["dov_arcsec"] * ARCSEC, 2)
    plumb = up + xi * n_hat + eta * e_hat
    plumb /= np.linalg.norm(plumb)

    M = itrs.rotation_at(t)
    z_i = M.T @ plumb
    R_true = make_camera_attitude(z_i, rng)

    img, _ = render_star_image(catalog, R_true, CAMERA, rng, mag_limit=sc["mag_limit"])
    xy, _ = detect_stars(img)
    sol = ident.identify(xy)
    if sol is None or np.degrees(attitude_error_rad(sol["R"], R_true)) > 0.1:
        return None
    R_est = sol["R"]

    # Specific force in ECEF: gravity along the plumb line plus residual
    # horizontal vehicle acceleration (after averaging).
    a_h = rng.normal(0, sc["accel_ms2"], 2)
    f_ecef = G * plumb + a_h[0] * e_hat + a_h[1] * n_hat
    f_cam = R_true @ (M.T @ f_ecef)
    f_cam /= np.linalg.norm(f_cam)

    # Uncalibrated camera-accelerometer misalignment (fixed per device).
    mis = rng.normal(0, sc["misalign_deg"] * DEG, 3)
    f_cam = rotm(mis) @ f_cam

    # Sensor tilt bias: fixed magnitude, or per-axis Gaussian (pitch, roll).
    if sc["tilt"][0] == "fixed":
        d = rng.uniform(0, 2 * np.pi)
        b = sc["tilt"][1] * DEG * np.array([np.cos(d), np.sin(d), 0.0])
    else:
        b = np.array([rng.normal(0, sc["tilt"][1] * DEG), rng.normal(0, sc["tilt"][2] * DEG), 0.0])
    f_meas = rotm(b) @ f_cam + rng.normal(0, ACCEL_NOISE_SIGMA, 3)
    f_meas /= np.linalg.norm(f_meas)

    t_est = t + float(rng.choice((-1.0, 1.0))) * CLOCK_OFFSET_S / 86400.0
    z_e = itrs.rotation_at(t_est) @ (R_est.T @ f_meas)
    z_e /= np.linalg.norm(z_e)
    lat_est = np.arcsin(np.clip(z_e[2], -1, 1))        # astronomical ~ geodetic latitude
    lon_est = np.arctan2(z_e[1], z_e[0])
    return ellipsoid_error_m(lat, lon, lat_est, lon_est) / 1000.0


SCENARIOS = [
    ("Paper optimistic, reproduced (0.05° tilt; no other terms; mag 6.5)",
     dict(tilt=("fixed", 0.05), misalign_deg=0.0, accel_ms2=0.0, dov_arcsec=0.0, mag_limit=6.5)),
    ("Paper conservative with Kuhlmann roll (1.1° pitch, 2.0° roll)",
     dict(tilt=("gauss", 1.1, 2.0), misalign_deg=0.0, accel_ms2=0.0, dov_arcsec=0.0, mag_limit=6.5)),
    ("Calibrated sensor, static (0.05° tilt, 0.05° misalignment, DoV, mag 4)",
     dict(tilt=("fixed", 0.05), misalign_deg=0.05, accel_ms2=0.0, dov_arcsec=10.0, mag_limit=4.0)),
    ("Calibrated sensor, steady flight (+0.05 m/s² residual acceleration)",
     dict(tilt=("fixed", 0.05), misalign_deg=0.05, accel_ms2=0.05, dov_arcsec=10.0, mag_limit=4.0)),
    ("Calibrated sensor, manoeuvring flight (+0.3 m/s² residual acceleration)",
     dict(tilt=("fixed", 0.05), misalign_deg=0.05, accel_ms2=0.3, dov_arcsec=10.0, mag_limit=4.0)),
    ("Uncalibrated phone, static (Kuhlmann 1.1°/2.0°, 0.5° misalignment, mag 4)",
     dict(tilt=("gauss", 1.1, 2.0), misalign_deg=0.5, accel_ms2=0.0, dov_arcsec=10.0, mag_limit=4.0)),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=200)
    a = ap.parse_args()
    rng = np.random.default_rng(20261110)
    ts = load.timescale()
    base = Path(__file__).resolve().parent
    catalog = StarCatalog.from_hipparcos(mag_limit=7.0, cache=str(base / "hipparcos_mag7.csv"),
                                         epoch=2026.5)
    ident = StarIdentifier(catalog, CAMERA, id_mag_limit=5.5, tol_deg=0.02, refine_tol_deg=0.03)

    L = ["# Experiment 3b — celestial Earth fix with a fuller error model", "",
         f"{a.runs} Monte Carlo runs per scenario; 60° camera; positions on WGS-84. "
         "Success = star attitude within 0.1° of truth.", "",
         "| Scenario | Success (%) | Median error (km) | 95th percentile (km) |",
         "| --- | --- | --- | --- |"]
    for name, sc in SCENARIOS:
        errs = [one_fix(rng, ts, catalog, ident, sc) for _ in range(a.runs)]
        ok = np.array([e for e in errs if e is not None])
        succ = 100 * len(ok) / a.runs
        med = f"{np.median(ok):.2f}" if ok.size else "–"
        p95 = f"{np.percentile(ok, 95):.2f}" if ok.size else "–"
        L.append(f"| {name} | {succ:.1f} | {med} | {p95} |")
        print(L[-1], flush=True)
    (base / "results").mkdir(exist_ok=True)
    (base / "results" / "exp3b.md").write_text("\n".join(L) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
