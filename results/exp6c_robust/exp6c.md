# Experiment 6c: GNSS spoofing detection with real visual-fix errors (robust: window median)

Visual fix every 10 s; GNSS noise 3.0 m/axis; threshold for 1 false alarm per hour (empirical, no-spoof); spoofing ramp in a random direction from t = 0; 1000 runs per cell; 'inf' = not detected within 900 s.

## Thresholds and fix statistics

| Monitor | Attempts | Fix availability (%) | Fixes > 25 m (%) | N | Threshold (m) |
| --- | --- | --- | --- | --- | --- |
| UAV-VisLoc, 6-DOF PnP (paper) | 612 | 89.5 | 15.8 | 3 | 41.1 |
| UAV-VisLoc, 6-DOF PnP (paper) | 612 | 89.5 | 15.8 | 6 | 22.5 |
| UAV-VisLoc, 4-DOF gravity only (onboard tilt) | 612 | 89.4 | 0.0 | 3 | 16.5 |
| UAV-VisLoc, 4-DOF gravity only (onboard tilt) | 612 | 89.4 | 0.0 | 6 | 11.1 |
| VPAIR, 6-DOF PnP (paper) | 395 | 96.5 | 2.3 | 3 | 17.1 |
| VPAIR, 6-DOF PnP (paper) | 395 | 96.5 | 2.3 | 6 | 9.5 |
| VPAIR, 4-DOF gravity only (0.1 deg tilt) | 395 | 96.2 | 0.5 | 3 | 8.8 |
| VPAIR, 4-DOF gravity only (0.1 deg tilt) | 395 | 96.2 | 0.5 | 6 | 6.0 |

## Detection (median time to alarm / median spoofed offset at alarm / detected within 120 s)

| Monitor | N | 0.1 m/s | 0.25 m/s | 0.5 m/s | 1.0 m/s | 2.0 m/s | 5.0 m/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| UAV-VisLoc, 6-DOF PnP (paper) | 3 | 340 s / 34 m / 3% | 160 s / 40 m / 13% | 90 s / 45 m / 96% | 60 s / 60 m / 100% | 40 s / 80 m / 100% | 20 s / 100 m / 100% |
| UAV-VisLoc, 6-DOF PnP (paper) | 6 | 220 s / 22 m / 5% | 120 s / 30 m / 61% | 80 s / 40 m / 100% | 50 s / 50 m / 100% | 40 s / 80 m / 100% | 30 s / 150 m / 100% |
| UAV-VisLoc, 4-DOF gravity only (onboard tilt) | 3 | 150 s / 15 m / 28% | 80 s / 20 m / 100% | 50 s / 25 m / 100% | 30 s / 30 m / 100% | 20 s / 40 m / 100% | 20 s / 100 m / 100% |
| UAV-VisLoc, 4-DOF gravity only (onboard tilt) | 6 | 130 s / 13 m / 42% | 80 s / 20 m / 100% | 50 s / 25 m / 100% | 40 s / 40 m / 100% | 30 s / 60 m / 100% | 30 s / 150 m / 100% |
| VPAIR, 6-DOF PnP (paper) | 3 | 170 s / 17 m / 12% | 80 s / 20 m / 100% | 50 s / 25 m / 100% | 30 s / 30 m / 100% | 20 s / 40 m / 100% | 20 s / 100 m / 100% |
| VPAIR, 6-DOF PnP (paper) | 6 | 120 s / 12 m / 63% | 70 s / 18 m / 100% | 50 s / 25 m / 100% | 40 s / 40 m / 100% | 30 s / 60 m / 100% | 30 s / 150 m / 100% |
| VPAIR, 4-DOF gravity only (0.1 deg tilt) | 3 | 90 s / 9 m / 95% | 50 s / 12 m / 100% | 30 s / 15 m / 100% | 20 s / 20 m / 100% | 20 s / 40 m / 100% | 20 s / 100 m / 100% |
| VPAIR, 4-DOF gravity only (0.1 deg tilt) | 6 | 90 s / 9 m / 98% | 50 s / 12 m / 100% | 40 s / 20 m / 100% | 30 s / 30 m / 100% | 30 s / 60 m / 100% | 30 s / 150 m / 100% |
