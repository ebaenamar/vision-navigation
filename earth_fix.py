"""Experiment 3: celestial Earth-position fixes from stars, gravity and time.

Run from this directory with:
    python earth_fix.py

Requires vision_core.py in the same directory, plus NumPy, SciPy, pandas,
Skyfield, astroquery (only if the Hipparcos download fallback is needed), and
Matplotlib.

The script writes:
    results/exp3.md
    results/fig_exp3_sweep.png

The reported success rate is the fraction of attempted runs for which the
star-identification pipeline returned a valid attitude solution. Position
statistics are calculated over successful runs only.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from skyfield.api import load
from skyfield.framelib import itrs

from vision_core import (
    Camera,
    StarCatalog,
    StarIdentifier,
    attitude_error_rad,
    detect_stars,
    render_star_image,
    rotm,
)


# ---------------------------------------------------------------------------
# Experiment configuration
# ---------------------------------------------------------------------------

BASE_DIR = Path(__file__).resolve().parent
RESULTS_DIR = BASE_DIR / "results"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)

EARTH_RADIUS_KM = 6371.0
CLOCK_OFFSET_S = 0.821       # 24-hour holdover at 9.5 ppm
N_ACCEL_AVERAGES = 100
ACCEL_NOISE_M_S2 = 0.005
ACCEL_NOISE_SIGMA = (ACCEL_NOISE_M_S2 / 9.81) / np.sqrt(N_ACCEL_AVERAGES)

OPTIMISTIC_BIAS_DEG = 0.05
CONSERVATIVE_AXIS_SIGMA_DEG = 1.1
SWEEP_BIASES_DEG = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0]

N_OPTIMISTIC = 200
N_CONSERVATIVE = 200
N_SWEEP = 100

# The project configuration has used a 60-degree camera field of view.
CAMERA = Camera(width=1024, height=1024, fov_deg=60.0)


# ---------------------------------------------------------------------------
# Geometry and simulation helpers
# ---------------------------------------------------------------------------

def random_utc_2026(rng: np.random.Generator, ts):
    """Return a uniformly sampled UTC instant during calendar year 2026."""
    start = datetime(2026, 1, 1, tzinfo=timezone.utc)
    seconds_in_year = 365 * 24 * 60 * 60  # 2026 is not a leap year
    dt = start + timedelta(seconds=float(rng.uniform(0, seconds_in_year)))
    return ts.utc(dt.year, dt.month, dt.day, dt.hour, dt.minute,
                  dt.second + dt.microsecond / 1e6)


def earth_zenith_ecef(latitude_rad: float, longitude_rad: float) -> np.ndarray:
    """Geocentric local-zenith unit vector in Earth-fixed coordinates."""
    c = np.cos(latitude_rad)
    return np.array([
        c * np.cos(longitude_rad),
        c * np.sin(longitude_rad),
        np.sin(latitude_rad),
    ])


def make_camera_attitude(zenith_inertial: np.ndarray,
                         rng: np.random.Generator,
                         max_off_zenith_deg: float = 10.0) -> np.ndarray:
    """Build inertial-to-camera R, with its third row along the boresight.

    The boresight is sampled uniformly in solid angle inside a cone around
    the true zenith. A further uniform rotation about the boresight supplies
    a random camera roll. The returned matrix is a proper rotation (det=+1).
    """
    z0 = np.asarray(zenith_inertial, dtype=float)
    z0 /= np.linalg.norm(z0)

    # Construct two orthonormal tangent directions at the zenith.
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(reference @ z0)) > 0.95:
        reference = np.array([1.0, 0.0, 0.0])
    tangent_1 = np.cross(reference, z0)
    tangent_1 /= np.linalg.norm(tangent_1)
    tangent_2 = np.cross(z0, tangent_1)
    tangent_2 /= np.linalg.norm(tangent_2)

    max_angle = np.radians(max_off_zenith_deg)
    cos_theta = rng.uniform(np.cos(max_angle), 1.0)
    theta = np.arccos(cos_theta)  # uniform over the cone's solid angle
    azimuth = rng.uniform(0.0, 2.0 * np.pi)
    boresight = (np.cos(theta) * z0
                 + np.sin(theta) * (np.cos(azimuth) * tangent_1
                                    + np.sin(azimuth) * tangent_2))
    boresight /= np.linalg.norm(boresight)

    # Complete a right-handed camera basis: x_cam cross y_cam = z_cam.
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(reference @ boresight)) > 0.95:
        reference = np.array([1.0, 0.0, 0.0])
    x_axis = np.cross(reference, boresight)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(boresight, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    roll = rng.uniform(0.0, 2.0 * np.pi)
    x_rolled = np.cos(roll) * x_axis + np.sin(roll) * y_axis
    y_rolled = -np.sin(roll) * x_axis + np.cos(roll) * y_axis

    R_true = np.vstack((x_rolled, y_rolled, boresight))
    return R_true


def make_tilted_accelerometer(ideal_reading: np.ndarray,
                              rng: np.random.Generator,
                              bias_mode: str,
                              bias_deg: float | None = None) -> np.ndarray:
    """Apply a phone tilt bias and averaged accelerometer noise, then normalise.

    ``fixed``: bias_deg is the tilt magnitude; its direction is random for
    each simulated phone/run.
    ``gaussian_axes``: independent x/y tilt components are N(0, 1.1 deg).
    Bias is represented as a small rotation vector in camera x/y axes.
    """
    if bias_mode == "fixed":
        if bias_deg is None:
            raise ValueError("bias_deg is required for fixed-bias mode")
        direction = rng.uniform(0.0, 2.0 * np.pi)
        magnitude = np.radians(bias_deg)
        bias_rotation = magnitude * np.array([
            np.cos(direction), np.sin(direction), 0.0
        ])
    elif bias_mode == "gaussian_axes":
        sigma = np.radians(CONSERVATIVE_AXIS_SIGMA_DEG)
        bias_rotation = np.array([
            rng.normal(0.0, sigma),
            rng.normal(0.0, sigma),
            0.0,
        ])
    else:
        raise ValueError(f"Unknown bias mode: {bias_mode}")

    biased = rotm(bias_rotation) @ np.asarray(ideal_reading, dtype=float)
    noisy = biased + rng.normal(0.0, ACCEL_NOISE_SIGMA, size=3)
    norm = np.linalg.norm(noisy)
    if not np.isfinite(norm) or norm == 0.0:
        raise FloatingPointError("Invalid accelerometer vector")
    return noisy / norm


def great_circle_error_km(lat_true: float, lon_true: float,
                          lat_est: float, lon_est: float) -> float:
    """Great-circle surface distance using a numerically stable haversine."""
    dlat = lat_est - lat_true
    dlon = (lon_est - lon_true + np.pi) % (2.0 * np.pi) - np.pi
    a = (np.sin(dlat / 2.0) ** 2
         + np.cos(lat_true) * np.cos(lat_est) * np.sin(dlon / 2.0) ** 2)
    central_angle = 2.0 * np.arcsin(np.sqrt(np.clip(a, 0.0, 1.0)))
    return float(EARTH_RADIUS_KM * central_angle)


# ---------------------------------------------------------------------------
# One simulated position fix
# ---------------------------------------------------------------------------

def simulate_one_fix(rng: np.random.Generator,
                      ts,
                      catalog: StarCatalog,
                      identifier: StarIdentifier,
                      bias_mode: str,
                      bias_deg: float | None = None) -> dict:
    """Simulate one star-camera + accelerometer + clock Earth-position fix."""
    # 1. Random truth position and UTC epoch.
    latitude = np.arcsin(rng.uniform(-1.0, 1.0))  # uniform over Earth's surface
    longitude = rng.uniform(-np.pi, np.pi)
    t = random_utc_2026(rng, ts)

    # 2. True zenith: ECEF -> inertial using the specified ITRS transform.
    z_e_true = earth_zenith_ecef(latitude, longitude)
    M = itrs.rotation_at(t)
    z_i_true = M.T @ z_e_true
    z_i_true /= np.linalg.norm(z_i_true)

    # 3. Camera points within 10 degrees of zenith, with random roll.
    R_true = make_camera_attitude(z_i_true, rng)

    # 4. Simulated star image -> detected stars -> identified attitude.
    image, _truth = render_star_image(
        catalog, R_true, CAMERA, rng,
        mag_limit=6.5,
        psf_sigma=1.2,
        background=10.0,
        read_noise=2.0,
        flux_mag6=200.0,
        full_well=4095.0,
        n_false=0,
    )
    xy, _flux = detect_stars(image)
    solution = identifier.identify(xy)
    if solution is None:
        return {"success": False, "error_km": np.nan}
    R_est = solution["R"]

    # 5. Accelerometer: gravity direction in camera coordinates, with tilt
    # bias and the specified 100-sample averaged noise level.
    f_true = R_true @ z_i_true
    f_meas = make_tilted_accelerometer(f_true, rng, bias_mode, bias_deg)

    # 6. Clock: independently fast or slow by 0.821 s.
    clock_sign = float(rng.choice((-1.0, 1.0)))
    t_est = t + clock_sign * CLOCK_OFFSET_S / 86400.0

    # 7. Estimate inertial zenith, then rotate it into Earth-fixed coordinates.
    z_i_est = R_est.T @ f_meas
    z_i_est /= np.linalg.norm(z_i_est)
    z_e_est = itrs.rotation_at(t_est) @ z_i_est
    z_e_est /= np.linalg.norm(z_e_est)

    latitude_est = np.arcsin(np.clip(z_e_est[2], -1.0, 1.0))
    longitude_est = np.arctan2(z_e_est[1], z_e_est[0])

    # 8. Great-circle surface-position error.
    error_km = great_circle_error_km(
        latitude, longitude, latitude_est, longitude_est
    )
    return {
        "success": True,
        "error_km": error_km,
        "attitude_error_arcsec": np.degrees(
            attitude_error_rad(R_est, R_true)
        ) * 3600.0,
    }


# ---------------------------------------------------------------------------
# Monte Carlo conditions, reporting and plotting
# ---------------------------------------------------------------------------

def run_condition(label: str,
                  n_runs: int,
                  rng: np.random.Generator,
                  ts,
                  catalog: StarCatalog,
                  identifier: StarIdentifier,
                  bias_mode: str,
                  bias_deg: float | None = None) -> dict:
    errors = []
    attitude_errors = []
    failures = 0

    for run_index in range(n_runs):
        result = simulate_one_fix(
            rng, ts, catalog, identifier, bias_mode, bias_deg
        )
        if result["success"]:
            errors.append(result["error_km"])
            attitude_errors.append(result["attitude_error_arcsec"])
        else:
            failures += 1
        if (run_index + 1) % 25 == 0 or run_index + 1 == n_runs:
            print(f"  {label}: {run_index + 1}/{n_runs} runs", flush=True)

    success_rate = (n_runs - failures) / n_runs
    if errors:
        median_km = float(np.median(errors))
        p95_km = float(np.percentile(errors, 95))
        median_attitude_arcsec = float(np.median(attitude_errors))
    else:
        median_km = np.nan
        p95_km = np.nan
        median_attitude_arcsec = np.nan

    return {
        "label": label,
        "n_runs": n_runs,
        "n_success": len(errors),
        "success_rate": success_rate,
        "median_km": median_km,
        "p95_km": p95_km,
        "median_attitude_arcsec": median_attitude_arcsec,
        "errors": np.asarray(errors, dtype=float),
    }


def format_result_row(result: dict) -> str:
    if result["n_success"] == 0:
        median = p95 = attitude = "N/A"
    else:
        median = f"{result['median_km']:.3f}"
        p95 = f"{result['p95_km']:.3f}"
        attitude = f"{result['median_attitude_arcsec']:.2f}"
    return (f"| {result['label']} | {result['n_runs']} | "
            f"{result['success_rate'] * 100:.1f}% | {median} | {p95} | "
            f"{attitude} |")


def write_report(optimistic: dict, conservative: dict,
                 sweep_results: list[dict]) -> Path:
    lines = [
        "# Experiment 3 — Celestial Earth-position fix",
        "",
        "Position errors are great-circle surface distances on a spherical "
        "Earth of radius 6,371 km. Median and 95th-percentile errors are "
        "calculated over successful star-identification runs only; the "
        "success rate uses all attempted runs.",
        "",
        "## Main conditions",
        "",
        "| Condition | Runs | Success rate | Median error (km) | "
        "95th-percentile error (km) | Median star-attitude error (arcsec) |",
        "|---|---:|---:|---:|---:|---:|",
        format_result_row(optimistic),
        format_result_row(conservative),
        "",
        "## Tilt-bias sweep",
        "",
        "| Tilt-bias magnitude (deg) | Runs | Success rate | "
        "Median error (km) | 95th-percentile error (km) | "
        "Median star-attitude error (arcsec) |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for result in sweep_results:
        lines.append(format_result_row(result))

    lines += [
        "",
        "## Simulation assumptions",
        "",
        f"- Camera: {CAMERA.width} × {CAMERA.height} pixels, "
        f"{CAMERA.fov_deg:.1f}° horizontal field of view.",
        "- Camera boresight sampled uniformly in solid angle within 10° "
        "of the true zenith; camera roll sampled uniformly from 0° to 360°.",
        "- Accelerometer noise: 0.005 m/s² per axis, divided by 9.81 m/s² "
        f"and by √{N_ACCEL_AVERAGES} for {N_ACCEL_AVERAGES} averaged readings.",
        f"- Clock offset: randomly signed ±{CLOCK_OFFSET_S:.3f} s, "
        "representing a 24-hour holdover at 9.5 ppm.",
        "- The optimistic tilt bias has fixed magnitude 0.05° and random "
        "direction per simulated run. The conservative case draws independent "
        "x- and y-axis bias components from N(0, 1.1°). Sweep biases have "
        "fixed magnitude and random direction per run.",
        "- Annual aberration of starlight (up to approximately 20 arcsec, "
        "or approximately 0.6 km at Earth's surface) is ignored.",
        "",
        "## Interpretation note",
        "",
        "These are results from the stated synthetic simulation, not field "
        "measurements from a physical phone. The star-identification success "
        "rate is reported separately so that position-error statistics are "
        "not mistaken for end-to-end availability.",
        "",
    ]
    path = RESULTS_DIR / "exp3.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def plot_sweep(sweep_results: list[dict]) -> Path:
    biases = np.array([r["bias_deg"] for r in sweep_results], dtype=float)
    medians = np.array([r["median_km"] for r in sweep_results], dtype=float)
    p95s = np.array([r["p95_km"] for r in sweep_results], dtype=float)

    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    ax.plot(biases, medians, marker="o", label="Median error")
    ax.plot(biases, p95s, marker="s", linestyle="--",
            label="95th-percentile error")
    ax.axhline(8.0, linestyle=":", linewidth=1.5, label="8 km criterion")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Tilt-bias magnitude (degrees)")
    ax.set_ylabel("Great-circle position error (km)")
    ax.set_title("Experiment 3: Earth-fix error versus tilt bias")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()

    path = RESULTS_DIR / "fig_exp3_sweep.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def main() -> None:
    # Fixed seed makes this experiment reproducible. Change it to obtain a
    # different Monte Carlo realization while retaining the same setup.
    rng = np.random.default_rng(20261107)
    ts = load.timescale()

    print("Loading Hipparcos catalogue...", flush=True)
    catalog = StarCatalog.from_hipparcos(
        mag_limit=7.0,
        cache=str(BASE_DIR / "hipparcos_mag7.csv"),
    )
    print(f"Loaded {len(catalog)} catalogue stars.", flush=True)

    print("Building star-identification index...", flush=True)
    identifier = StarIdentifier(
        catalog,
        CAMERA,
        id_mag_limit=5.5,
        tol_deg=0.02,
        refine_tol_deg=0.03,
    )
    print(f"Star-identification index contains {identifier.n_pairs} pairs.",
          flush=True)

    print("\nRunning optimistic condition...", flush=True)
    optimistic = run_condition(
        "Optimistic (0.05° fixed tilt)", N_OPTIMISTIC, rng, ts,
        catalog, identifier, "fixed", OPTIMISTIC_BIAS_DEG,
    )

    print("\nRunning conservative condition...", flush=True)
    conservative = run_condition(
        "Conservative (1.1° Gaussian per axis)", N_CONSERVATIVE, rng, ts,
        catalog, identifier, "gaussian_axes",
    )

    sweep_results = []
    for bias in SWEEP_BIASES_DEG:
        print(f"\nRunning {bias:g}° sweep condition...", flush=True)
        result = run_condition(
            f"{bias:g}°", N_SWEEP, rng, ts,
            catalog, identifier, "fixed", bias,
        )
        result["bias_deg"] = bias
        sweep_results.append(result)

    report_path = write_report(optimistic, conservative, sweep_results)
    figure_path = plot_sweep(sweep_results)

    print("\nExperiment 3 complete.", flush=True)
    print(f"Report: {report_path}")
    print(f"Figure: {figure_path}")
    print("\n--- exp3.md ---\n", flush=True)
    print(report_path.read_text(encoding="utf-8"), flush=True)


if __name__ == "__main__":
    main()
