# Experiment 6c: GNSS spoofing detection with real visual-fix errors

Visual fix every 10 s; GNSS noise 3.0 m/axis; threshold for 1 false alarm per hour (empirical, no-spoof); spoofing ramp in a random direction from t = 0; 1000 runs per cell; 'inf' = not detected within 900 s.

## Thresholds and fix statistics

| Monitor | Attempts | Fix availability (%) | Fixes > 25 m (%) | N | Threshold (m) |
| --- | --- | --- | --- | --- | --- |
| UAV-VisLoc, 6-DOF PnP (paper) | 612 | 89.5 | 15.8 | 1 | 144.9 |
| UAV-VisLoc, 6-DOF PnP (paper) | 612 | 89.5 | 15.8 | 3 | 55.6 |
| UAV-VisLoc, 6-DOF PnP (paper) | 612 | 89.5 | 15.8 | 6 | 32.4 |
| UAV-VisLoc, 4-DOF gravity only (onboard tilt) | 612 | 89.4 | 0.0 | 1 | 24.8 |
| UAV-VisLoc, 4-DOF gravity only (onboard tilt) | 612 | 89.4 | 0.0 | 3 | 14.1 |
| UAV-VisLoc, 4-DOF gravity only (onboard tilt) | 612 | 89.4 | 0.0 | 6 | 9.9 |
| VPAIR, 6-DOF PnP (paper) | 395 | 96.5 | 2.3 | 1 | 191.8 |
| VPAIR, 6-DOF PnP (paper) | 395 | 96.5 | 2.3 | 3 | 68.4 |
| VPAIR, 6-DOF PnP (paper) | 395 | 96.5 | 2.3 | 6 | 36.2 |
| VPAIR, 4-DOF gravity only (0.1 deg tilt) | 395 | 96.2 | 0.5 | 1 | 125.0 |
| VPAIR, 4-DOF gravity only (0.1 deg tilt) | 395 | 96.2 | 0.5 | 3 | 90.7 |
| VPAIR, 4-DOF gravity only (0.1 deg tilt) | 395 | 96.2 | 0.5 | 6 | 46.4 |

## Detection (median time to alarm / median spoofed offset at alarm / detected within 120 s)

| Monitor | N | 0.1 m/s | 0.25 m/s | 0.5 m/s | 1.0 m/s | 2.0 m/s | 5.0 m/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| UAV-VisLoc, 6-DOF PnP (paper) | 1 | inf s / inf m / 3% | 520 s / 130 m / 3% | 280 s / 140 m / 2% | 150 s / 150 m / 12% | 80 s / 160 m / 100% | 30 s / 150 m / 100% |
| UAV-VisLoc, 6-DOF PnP (paper) | 3 | 490 s / 49 m / 2% | 220 s / 55 m / 3% | 120 s / 60 m / 55% | 70 s / 70 m / 100% | 40 s / 80 m / 100% | 30 s / 150 m / 100% |
| UAV-VisLoc, 6-DOF PnP (paper) | 6 | 320 s / 32 m / 2% | 160 s / 40 m / 12% | 100 s / 50 m / 96% | 70 s / 70 m / 100% | 50 s / 100 m / 100% | 30 s / 150 m / 100% |
| UAV-VisLoc, 4-DOF gravity only (onboard tilt) | 1 | 170 s / 17 m / 17% | 80 s / 20 m / 99% | 50 s / 25 m / 100% | 30 s / 30 m / 100% | 20 s / 40 m / 100% | 10 s / 50 m / 100% |
| UAV-VisLoc, 4-DOF gravity only (onboard tilt) | 3 | 130 s / 13 m / 43% | 70 s / 18 m / 100% | 40 s / 20 m / 100% | 30 s / 30 m / 100% | 20 s / 40 m / 100% | 10 s / 50 m / 100% |
| UAV-VisLoc, 4-DOF gravity only (onboard tilt) | 6 | 120 s / 12 m / 53% | 70 s / 18 m / 100% | 50 s / 25 m / 100% | 40 s / 40 m / 100% | 30 s / 60 m / 100% | 20 s / 100 m / 100% |
| VPAIR, 6-DOF PnP (paper) | 1 | inf s / inf m / 4% | 750 s / 188 m / 3% | 390 s / 195 m / 3% | 200 s / 200 m / 4% | 100 s / 200 m / 100% | 40 s / 200 m / 100% |
| VPAIR, 6-DOF PnP (paper) | 3 | 680 s / 68 m / 3% | 290 s / 72 m / 2% | 150 s / 75 m / 5% | 80 s / 80 m / 100% | 50 s / 100 m / 100% | 30 s / 150 m / 100% |
| VPAIR, 6-DOF PnP (paper) | 6 | 390 s / 39 m / 2% | 170 s / 42 m / 4% | 100 s / 50 m / 99% | 70 s / 70 m / 100% | 50 s / 100 m / 100% | 30 s / 150 m / 100% |
| VPAIR, 4-DOF gravity only (0.1 deg tilt) | 1 | inf s / inf m / 3% | 500 s / 125 m / 4% | 250 s / 125 m / 4% | 130 s / 130 m / 12% | 70 s / 140 m / 100% | 30 s / 150 m / 100% |
| VPAIR, 4-DOF gravity only (0.1 deg tilt) | 3 | inf s / inf m / 3% | 380 s / 95 m / 2% | 200 s / 100 m / 2% | 110 s / 110 m / 100% | 60 s / 120 m / 100% | 30 s / 150 m / 100% |
| VPAIR, 4-DOF gravity only (0.1 deg tilt) | 6 | 490 s / 49 m / 2% | 220 s / 55 m / 2% | 120 s / 60 m / 69% | 80 s / 80 m / 100% | 50 s / 100 m / 100% | 30 s / 150 m / 100% |
