## Table 1. Simulation parameters

| Parameter | Value |
| --- | --- |
| catalog | Hipparcos |
| camera_px | 1024 x 1024 |
| fov_deg | 20.0 |
| pixel_scale_arcsec | 71.04 |
| render_mag_limit | 6.5 |
| id_mag_limit | 5.5 |
| psf_sigma_px | 1.2 |
| background_counts | 10.0 |
| read_noise_nominal | 2.0 |
| detection_k_sigma | 5.0 |
| pair_tolerance_deg | 0.02 |
| refine_tolerance_deg | 0.03 |
| legacy_catalog_stars | 1000 |
| legacy_tolerance_deg | 0.5 |
| seed | 2026 |
| computer | arm / Darwin |
| id_catalog_stars | 2851 |
| id_catalog_pairs | 262336 |
| filter gyro ARW (rad/√s) | 3.16e-07 |
| filter gyro bias RW (rad/s^1.5) | 3.16e-10 |
| filter star noise (arcsec/axis) | 13.8 |
| filter runs × duration | 20 × 3600 s |

## Table 2. Star identification and single-frame attitude

| Condition | Frames | Stars detected (mean) | Stars identified (mean) | Success, corrected (%) | Success, early draft (%) | Attitude error median (arcsec) | Attitude error 95th pct (arcsec) | Boresight error median (arcsec) | Early-draft boresight error median (deg) | Time/frame corrected (ms) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Nominal noise | 200 | 46.0 | 46.0 | 100.0 | 0.0 | 10.6 | 32.3 | 2.6 | 88.5 | 11.1 |
| High noise | 200 | 15.6 | 15.6 | 99.5 | 0.0 | 17.2 | 55.3 | 3.8 | 93.8 | 10.0 |
| Nominal + 5 false objects | 200 | 50.0 | 47.0 | 74.5 | 0.0 | 10.7 | 29.7 | 2.5 | 92.8 | 614.9 |
| Nominal + 5 false objects, 12-star search | 200 | 50.8 | 45.6 | 99.5 | 0.0 | 10.3 | 31.5 | 2.2 | 85.5 | 797.4 |

Success = boresight error below 0.1°. Per-star residual RMS (nominal): 19.5 arcsec.

## Table 2b. Identification scored on catalogue IDs, per-axis accuracy (1σ RMS)

| Condition | Success (%) | Gross wrong solution (%) | No solution (%) | Misidentified stars in successful frames (%) | Cross-boresight 1σ (arcsec) | Roll 1σ (arcsec) | Time/frame (ms) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Nominal noise | 100.0 | 0.0 | 0.0 | 0.17 | 2.3 | 16.6 | 11.1 |
| High noise | 99.5 | 0.0 | 0.5 | 0.35 | 3.4 | 25.9 | 10.0 |
| Nominal + 5 false objects | 74.5 | 0.0 | 25.5 | 0.19 | 2.1 | 15.7 | 614.9 |
| Nominal + 5 false objects, 12-star search | 99.5 | 0.5 | 0.0 | 0.14 | 2.1 | 15.4 | 797.4 |

## Table 3. Filter performance (medians over runs)

| Method | Final attitude error (arcsec) | Max error during star outage (arcsec) | Final bias error (deg/h) |
| --- | --- | --- | --- |
| Gyro only (from one star fix) | 1728.8 | – | – |
| MEKF with star updates | 1.43 | 4.6 | 0.0041 |

MEKF error inside its 3σ bound 100.0% of the time after the first 10 minutes.

## Table 3b. Filter with realistic sensors (medians over runs; NEES after 10 min)

| Scenario | Final attitude error (arcsec) | Gyro-only final (arcsec) | Max error in 10-min outage (arcsec) | Average NEES (3 = consistent) | 95% NEES bounds | Time with ensemble NEES inside bounds (%) |
| --- | --- | --- | --- | --- | --- | --- |
| Paper gyro (textbook FOG-grade), white star noise | 2.15 | 2111.2 | 4.4 | 3.07 | 2.02–4.16 | 97.9 |
| Paper gyro, body rate 0 (no bias averaging) | 1.76 | 3600.3 | 5.3 | 3.05 | 2.02–4.16 | 98.1 |
| MEMS gyro (EuRoC ADIS16448), white star noise | 25.95 | 380081.0 | 51305.0 | 3.00 | 2.02–4.16 | 95.1 |
| MEMS gyro + star-tracker bias (3/3/15 arcsec) | 28.90 | 368378.1 | 48683.8 | 3.76 | 2.02–4.16 | 73.4 |

## Noise sweep (Figure 5)

| Read noise | Success corrected (%) | Success early draft (%) | Median attitude error (arcsec) |
| --- | --- | --- | --- |
| 2 | 100.0 | 0.0 | 9.7 |
| 4 | 100.0 | 0.0 | 12.5 |
| 8 | 96.7 | 0.0 | 14.6 |
| 16 | 91.7 | 0.0 | 21.1 |
| 24 | 88.3 | 0.0 | 33.4 |
| 32 | 96.7 | 0.0 | 32.9 |
