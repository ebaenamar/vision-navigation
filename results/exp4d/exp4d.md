# Experiment 4d: soft-gravity MAP PnP (A) and calibrated covariance / protection levels (B)

## A. Accuracy (pooled over priors)

| Dataset | Solver | IMU tilt | Success (%) | Median (m) | 95th pct (m) | False > 25 m (%) |
| --- | --- | --- | --- | --- | --- | --- |
| visloc | 4-DOF hard gravity | onboard (1.5 deg) | 88.5 | 7.3 | 16.5 | 0.0 |
| visloc | 6-DOF PnP (paper) | - | 89.7 | 11.2 | 32.8 | 9.4 |
| visloc | Hybrid: 4-DOF position + MAP covariance | onboard (1.5 deg) | 88.5 | 7.3 | 16.5 | 0.0 |
| visloc | MAP soft gravity | onboard (1.5 deg) | 89.8 | 10.5 | 24.5 | 4.2 |
| vpair | 4-DOF hard gravity | 0.1 deg | 90.9 | 3.1 | 7.9 | 0.6 |
| vpair | 4-DOF hard gravity | 0.5 deg | 90.9 | 4.8 | 10.7 | 0.6 |
| vpair | 4-DOF hard gravity | 1 deg | 90.9 | 7.7 | 16.9 | 0.6 |
| vpair | 4-DOF hard gravity | 2 deg | 91.1 | 14.1 | 30.6 | 10.9 |
| vpair | 4-DOF hard gravity | 30 arcsec | 91.0 | 3.1 | 7.6 | 0.7 |
| vpair | 6-DOF PnP (paper) | - | 91.9 | 5.7 | 49.2 | 9.9 |
| vpair | Hybrid: 4-DOF position + MAP covariance | 0.1 deg | 90.9 | 3.1 | 7.9 | 0.6 |
| vpair | Hybrid: 4-DOF position + MAP covariance | 0.5 deg | 90.9 | 4.8 | 10.7 | 0.6 |
| vpair | Hybrid: 4-DOF position + MAP covariance | 1 deg | 90.9 | 7.7 | 16.9 | 0.6 |
| vpair | Hybrid: 4-DOF position + MAP covariance | 2 deg | 91.1 | 14.1 | 30.6 | 10.9 |
| vpair | Hybrid: 4-DOF position + MAP covariance | 30 arcsec | 91.0 | 3.1 | 7.6 | 0.7 |
| vpair | MAP soft gravity | 0.1 deg | 91.4 | 3.6 | 7.8 | 0.7 |
| vpair | MAP soft gravity | 0.5 deg | 91.4 | 5.0 | 12.4 | 0.8 |
| vpair | MAP soft gravity | 1 deg | 91.5 | 5.4 | 19.3 | 2.2 |
| vpair | MAP soft gravity | 2 deg | 91.5 | 5.6 | 27.0 | 5.3 |
| vpair | MAP soft gravity | 30 arcsec | 91.4 | 3.1 | 7.6 | 0.7 |

## B. Covariance consistency and 99% protection levels

Raw = solver covariance as is. Two-fold = inflation k estimated on half of the locations, evaluated on the other half (averaged). Cross = k from the OTHER dataset (same solver; for VPAIR levels, k from the matching level is not available on UAV-VisLoc, so the 0.5 deg level is paired with UAV-VisLoc's onboard attitude).

| Dataset | Solver | IMU tilt | Raw 95% coverage | k (two-fold) | Held-out 95% / 99% coverage | Median PL99 (m) | Integrity: errors > PL99 (%) | Availability: PL99 <= 25 m alert limit (%) | Cross-dataset 95% / 99% coverage |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| visloc | 4-DOF hard gravity | onboard (1.5 deg) | 0% | 7945.9 | 96 / 99% | 20.2 | 0.7 | 70 | 92 / 97 |
| visloc | 6-DOF PnP (paper) | - | 11% | 32.8 | 95 / 99% | 35.1 | 0.4 | 8 | 95 / 99 |
| visloc | Hybrid: 4-DOF position + MAP covariance | onboard (1.5 deg) | 23% | 18.5 | 95 / 100% | 25.9 | 0.0 | 45 | 100 / 100 |
| visloc | MAP soft gravity | onboard (1.5 deg) | 16% | 30.9 | 95 / 99% | 33.6 | 0.4 | 12 | 92 / 99 |
| vpair | 4-DOF hard gravity | 0.1 deg | 0% | 1443.7 | 93 / 99% | 6.2 | 1.0 | 92 | 100 / 100 |
| vpair | 4-DOF hard gravity | 0.5 deg | 0% | 6117.8 | 95 / 98% | 13.1 | 1.7 | 76 | 97 / 99 |
| vpair | 4-DOF hard gravity | 1 deg | 0% | 17007.2 | 94 / 99% | 22.4 | 1.0 | 55 | 80 / 89 |
| vpair | 4-DOF hard gravity | 2 deg | 0% | 39825.5 | 95 / 99% | 38.4 | 0.6 | 18 | 53 / 65 |
| vpair | 4-DOF hard gravity | 30 arcsec | 0% | 1134.1 | 90 / 99% | 5.5 | 1.5 | 93 | 100 / 100 |
| vpair | 6-DOF PnP (paper) | - | 8% | 32.1 | 95 / 98% | 18.7 | 1.5 | 60 | 95 / 98 |
| vpair | Hybrid: 4-DOF position + MAP covariance | 0.1 deg | 4% | 28.1 | 95 / 99% | 8.2 | 1.0 | 100 | 85 / 95 |
| vpair | Hybrid: 4-DOF position + MAP covariance | 0.5 deg | 15% | 45.8 | 95 / 98% | 20.6 | 1.0 | 58 | 83 / 89 |
| vpair | Hybrid: 4-DOF position + MAP covariance | 1 deg | 11% | 147.8 | 95 / 98% | 38.6 | 0.3 | 31 | 59 / 69 |
| vpair | Hybrid: 4-DOF position + MAP covariance | 2 deg | 8% | 516.0 | 95 / 98% | 73.3 | 0.7 | 2 | 36 / 44 |
| vpair | Hybrid: 4-DOF position + MAP covariance | 30 arcsec | 0% | 576.4 | 93 / 99% | 5.4 | 0.9 | 95 | 3 / 6 |
| vpair | MAP soft gravity | 0.1 deg | 4% | 30.8 | 95 / 99% | 8.7 | 0.9 | 100 | 95 / 99 |
| vpair | MAP soft gravity | 0.5 deg | 11% | 26.2 | 95 / 99% | 15.6 | 0.8 | 70 | 97 / 99 |
| vpair | MAP soft gravity | 1 deg | 11% | 27.9 | 95 / 99% | 16.9 | 0.8 | 65 | 96 / 99 |
| vpair | MAP soft gravity | 2 deg | 11% | 29.6 | 95 / 98% | 17.6 | 1.2 | 63 | 95 / 99 |
| vpair | MAP soft gravity | 30 arcsec | 0% | 573.2 | 92 / 99% | 5.4 | 0.9 | 95 | 6 / 10 |
