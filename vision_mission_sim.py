"""
V.I.S.I.O.N. interactive mission simulator (summer 2026 code, patched Nov 2026).

What changed from the original (search for "# CHANGED" to see each one):
  1. Star detection, identification and attitude now come from vision_core.py
     (sub-pixel centroids, triangle + 4th-star identification, corrected
     Wahba/SVD solution). The old kabsch_algorithm(), estimate_orientation(),
     estimate_radec_from_rotation(), match_stars(), generate_star_angle_catalog()
     and the pickle catalogue are no longer used.
  2. The camera field of view is asked for instead of assuming 60 degrees.
  3. A star image now only sets the camera POINTING. It no longer sets the
     spacecraft POSITION (a pointing direction says nothing about position).
  4. Fixed simulation epoch instead of ts.now(), so runs can be repeated.
  5. Removed the broken "horizons" import and asteroid targets (not in DE421).
  6. mu_sun is a constant, so the script no longer needs poliastro.
  7. The results/plot block ran twice; it now runs once.

Limitations (state these in the paper): Sun-only gravity, impulsive velocity
corrections with no fuel model. Treat the trajectory output as a demonstration,
not as a performance result.
"""

import math
import sys
from collections import defaultdict

import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

try:
    import cv2
except ImportError:
    cv2 = None
    print("Warning: OpenCV not available. Star images cannot be loaded.")

from skyfield.api import load, EarthSatellite

# CHANGED (1): new attitude pipeline
from vision_core import (Camera, StarCatalog, StarIdentifier, detect_stars,
                         boresight_inertial, vec_to_radec)

# CHANGED (6): constant instead of poliastro
MU_SUN = 1.32712440018e11        # km^3/s^2
R_EARTH_KM = 6378.137

# CHANGED (4): fixed epoch for reproducible runs
EPOCH = (2026, 1, 1)

ts = load.timescale()
planets = load("de421.bsp")
# CHANGED (5): only bodies that exist in DE421
planet_map = {
    "mercury": "mercury", "venus": "venus", "earth": "earth", "moon": "moon",
    "mars": "mars", "jupiter": "jupiter barycenter", "saturn": "saturn barycenter",
    "uranus": "uranus barycenter", "neptune": "neptune barycenter",
    "pluto": "pluto barycenter",
}
ISS_LINE1 = "1 25544U 98067A   21075.54791667  .00001264  00000-0  29621-4 0  9990"
ISS_LINE2 = "2 25544  51.6445  57.9090 0002328  93.6994 306.4291 15.48954312273137"
iss_sat = EarthSatellite(ISS_LINE1, ISS_LINE2, "ISS", ts)


def normalize(v):
    n = np.linalg.norm(v)
    return v / n if n != 0 else v


def rk4_step(position, velocity, dt_seconds, mu):
    """RK4 step for two-body motion about the Sun (Sun at the origin)."""
    def acc(r):
        rm = np.linalg.norm(r)
        return np.zeros(3) if rm < 1e-6 else -(mu / rm**3) * r

    k1_v = acc(position); k1_p = velocity
    k2_v = acc(position + k1_p * dt_seconds / 2); k2_p = velocity + k1_v * dt_seconds / 2
    k3_v = acc(position + k2_p * dt_seconds / 2); k3_p = velocity + k2_v * dt_seconds / 2
    k4_v = acc(position + k3_p * dt_seconds); k4_p = velocity + k3_v * dt_seconds
    new_p = position + dt_seconds / 6.0 * (k1_p + 2 * k2_p + 2 * k3_p + k4_p)
    new_v = velocity + dt_seconds / 6.0 * (k1_v + 2 * k2_v + 2 * k3_v + k4_v)
    return new_p, new_v


def adaptive_velocity_correction(velocity, position, target_pos, target_vel,
                                 approach_factor=0.5, max_correction_mag=0.1,
                                 dist_threshold_km=100000):
    """Heuristic guidance law (unchanged from the original)."""
    vec_to_target = target_pos - position
    distance = np.linalg.norm(vec_to_target)
    if distance < 100:
        desired_velocity = target_vel
    elif distance < dist_threshold_km:
        approach_speed = min(np.linalg.norm(velocity) * approach_factor, distance / 10000)
        desired_velocity = target_vel + normalize(vec_to_target) * approach_speed
    else:
        desired_velocity = normalize(vec_to_target) * np.linalg.norm(velocity)
    delta_v = desired_velocity - velocity
    if np.linalg.norm(delta_v) > max_correction_mag:
        delta_v = normalize(delta_v) * max_correction_mag
    return velocity + delta_v


def target_state(dest_name, target_body, t):
    if dest_name == "iss":
        e = planets["earth"].at(t)
        g = iss_sat.at(t)
        return e.position.km + g.position.km, e.velocity.km_per_s + g.velocity.km_per_s
    s = target_body.at(t)
    return s.position.km, s.velocity.km_per_s


def star_calibration():
    """CHANGED (1)-(3): estimate camera pointing from a real star image."""
    if cv2 is None:
        print("OpenCV is needed to read the image.")
        return None
    path = input("\nPath to star image: ").strip().strip("'\"")
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Could not load {path}.")
        return None
    try:
        fov = float(input("Horizontal field of view of the camera in degrees (e.g. 20): "))
    except ValueError:
        print("Invalid field of view.")
        return None
    h, w = img.shape
    cam = Camera(width=w, height=h, fov_deg=fov)
    print("Loading Hipparcos and building star-pair table...")
    catalog = StarCatalog.from_hipparcos(mag_limit=7.0)
    ident = StarIdentifier(catalog, cam)
    xy, _ = detect_stars(img.astype(float))
    print(f"Detected {len(xy)} stars.")
    out = ident.identify(xy)
    if out is None:
        print("Star identification failed.")
        return None
    ra, dec = vec_to_radec(boresight_inertial(out["R"]))
    rms = np.degrees(np.sqrt(np.mean(out["residuals_rad"] ** 2))) * 3600
    print(f"Identified {len(out['det_idx'])} stars. Camera pointing: "
          f"RA {ra:.4f} deg, Dec {dec:.4f} deg (residual RMS {rms:.1f} arcsec)")

    annotated = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for x, y in xy:
        cv2.circle(annotated, (int(round(x)), int(round(y))), 6, (0, 255, 0), 1)
    cv2.imwrite("output_detected_stars.jpg", annotated)
    print("Annotated image saved to output_detected_stars.jpg")
    return out["R"]


def plot_trajectory(trajectory, target_positions, planets_data, target_name):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection="3d")
    pos = np.array(trajectory) / 1.495978707e8          # km -> AU
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], color="tab:blue", lw=2, label="Spacecraft trajectory")
    ax.scatter(*pos[0], color="gold", s=60, label="Start")
    ax.scatter(*pos[-1], color="tab:green", s=60, label="End")
    tp = np.array(target_positions) / 1.495978707e8
    ax.plot(tp[:, 0], tp[:, 1], tp[:, 2], ":", color="tab:red", label=f"{target_name} path")
    for name in ("earth", "mars", "venus"):
        if name in planets_data and name != target_name.lower():
            p = np.array(planets_data[name]) / 1.495978707e8
            ax.plot(p[:, 0], p[:, 1], p[:, 2], "--", lw=0.8, alpha=0.6, label=f"{name.capitalize()} orbit")
    ax.scatter(0, 0, 0, color="orange", s=200, label="Sun")
    ax.set_xlabel("X (AU)"); ax.set_ylabel("Y (AU)"); ax.set_zlabel("Z (AU)")
    ax.set_title(f"Simulated trajectory to {target_name} (heliocentric, Sun-only gravity)")
    ax.legend(loc="upper left", fontsize=9)
    plt.tight_layout()
    plt.savefig("trajectory.png", dpi=200)
    print("Plot saved to trajectory.png")
    plt.show()


if __name__ == "__main__":
    print("==== V.I.S.I.O.N. mission simulator ====")
    start_time = ts.utc(*EPOCH)                         # CHANGED (4)

    if input("\nRun star calibration from an image first? (y/n): ").strip().lower() == "y":
        star_calibration()                              # CHANGED (3): pointing only

    earth = planets["earth"].at(start_time)
    position = np.array(earth.position.km, dtype=float)
    velocity = np.array(earth.velocity.km_per_s, dtype=float)
    print("Spacecraft starts at Earth's heliocentric position and velocity.")

    dest_name = input("\nTarget (planet, moon or iss): ").strip().lower()
    if dest_name == "iss":
        target_body = iss_sat
    elif dest_name in planet_map:
        target_body = planets[planet_map[dest_name]]
    else:
        print(f"Target '{dest_name}' not recognised.")
        sys.exit(1)

    try:
        duration_days = float(input("Simulation duration (days): "))
        dt_hours = float(input("Time step (hours): "))
    except ValueError:
        print("Invalid input. Using 365 days, 6 hours.")
        duration_days, dt_hours = 365.0, 6.0

    dt_s = dt_hours * 3600.0
    dt_days = dt_hours / 24.0
    steps = int(duration_days * 24 / dt_hours)

    trajectory, target_positions = [], []
    planet_positions = defaultdict(list)
    for i in tqdm(range(steps), desc=f"Simulating to {dest_name.capitalize()}"):
        t = start_time + i * dt_days
        tpos, tvel = target_state(dest_name, target_body, t)
        velocity = adaptive_velocity_correction(velocity, position, tpos, tvel,
                                                approach_factor=0.5, max_correction_mag=0.01)
        position, velocity = rk4_step(position, velocity, dt_s, MU_SUN)
        trajectory.append(position.copy())
        target_positions.append(tpos.copy())
        for name in ("earth", "mars", "venus"):
            planet_positions[name].append(planets[planet_map[name]].at(t).position.km)

    # CHANGED (7): results reported once
    tpos, tvel = target_state(dest_name, target_body, start_time + steps * dt_days)
    distance = np.linalg.norm(position - tpos)
    rel_vel = np.linalg.norm(velocity - tvel)
    print("\n==== Simulation results ====")
    print(f"Final distance to target: {distance:,.0f} km")
    print(f"Final relative velocity:  {rel_vel:.3f} km/s")
    plot_trajectory(trajectory, target_positions, planet_positions, dest_name.capitalize())
