# Experiment 4b: attitude-aided map matching on VPAIR (199 real queries)

Success = >= 12 inliers. Errors over successful fixes; 'within 25 m' and 'false fix (> 25 m)' over all queries.

| Matching | Solver | Offset (frames) | Mean prior error (m) | Success (%) | Median (m) [95% CI] | 95th pct (m) [95% CI] | Within 25 m (%) | False fixes > 25 m (%) [95% CI] | Median match time (s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| heading | 3-DOF, MEMS-only (0.5 deg) | 0 | 0 | 98.0 | 3.9 [3.5–4.4] | 7.9 [7.1–8.6] | 98.0 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, calibrated INS (0.1 deg) | 0 | 0 | 98.0 | 1.8 [1.6–1.9] | 3.1 [2.9–3.4] | 98.0 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, oracle (INS truth, boresight-calibrated) | 0 | 0 | 98.0 | 1.7 [1.6–1.8] | 2.8 [2.5–3.1] | 98.0 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, star tracker + gyro (30 arcsec) | 0 | 0 | 98.0 | 1.7 [1.6–1.8] | 2.8 [2.6–3.2] | 98.0 | 0.0 [0.0–0.0] | 0.14 |
| heading | 6-DOF PnP (paper) | 0 | 0 | 98.0 | 2.9 [2.6–3.2] | 9.6 [6.6–12.4] | 98.0 | 0.0 [0.0–0.0] | 0.14 |
| search8 | 3-DOF, MEMS-only (0.5 deg) | 0 | 0 | 98.5 | 4.0 [3.5–4.3] | 7.9 [7.2–9.2] | 98.5 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 3-DOF, calibrated INS (0.1 deg) | 0 | 0 | 98.5 | 1.8 [1.6–1.9] | 3.2 [2.9–3.4] | 98.5 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 3-DOF, oracle (INS truth, boresight-calibrated) | 0 | 0 | 98.5 | 1.7 [1.6–1.8] | 2.8 [2.5–3.2] | 98.5 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 3-DOF, star tracker + gyro (30 arcsec) | 0 | 0 | 98.5 | 1.7 [1.6–1.8] | 2.8 [2.6–3.2] | 98.5 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 4-DOF gravity only, MEMS-only (0.5 deg) | 0 | 0 | 98.0 | 4.2 [3.6–4.4] | 8.0 [7.3–9.0] | 98.0 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 4-DOF gravity only, calibrated INS (0.1 deg) | 0 | 0 | 98.0 | 1.8 [1.6–1.9] | 3.1 [3.0–3.7] | 98.0 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 6-DOF + attitude gate, MEMS-only (0.5 deg) | 0 | 0 | 98.0 | 3.0 [2.8–3.3] | 9.1 [6.4–11.9] | 98.0 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 6-DOF + attitude gate, calibrated INS (0.1 deg) | 0 | 0 | 88.4 | 2.9 [2.5–3.1] | 5.5 [5.1–6.1] | 88.4 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 6-DOF + attitude gate, oracle (INS truth, boresight-calibrated) | 0 | 0 | 87.9 | 2.9 [2.5–3.2] | 5.4 [4.9–6.0] | 87.9 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 6-DOF + attitude gate, star tracker + gyro (30 arcsec) | 0 | 0 | 87.9 | 2.9 [2.5–3.2] | 5.4 [4.9–6.0] | 87.9 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 6-DOF PnP (paper) | 0 | 0 | 99.0 | 3.0 [2.8–3.3] | 10.3 [7.4–12.1] | 98.0 | 1.0 [0.0–2.5] | 0.71 |
| heading | 3-DOF, MEMS-only (0.5 deg) | 1 | 45 | 97.5 | 3.8 [3.4–4.3] | 8.6 [7.6–9.3] | 97.5 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, calibrated INS (0.1 deg) | 1 | 45 | 97.5 | 2.4 [2.2–2.4] | 3.8 [3.5–4.3] | 97.5 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, oracle (INS truth, boresight-calibrated) | 1 | 45 | 97.5 | 2.2 [2.1–2.3] | 3.4 [3.1–3.8] | 97.5 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, star tracker + gyro (30 arcsec) | 1 | 45 | 97.5 | 2.2 [2.1–2.3] | 3.5 [3.1–3.8] | 97.5 | 0.0 [0.0–0.0] | 0.14 |
| heading | 6-DOF PnP (paper) | 1 | 45 | 97.5 | 3.6 [3.3–4.1] | 8.7 [7.5–12.7] | 97.5 | 0.0 [0.0–0.0] | 0.14 |
| search8 | 3-DOF, MEMS-only (0.5 deg) | 1 | 45 | 97.5 | 3.8 [3.5–4.3] | 8.6 [7.7–9.2] | 97.5 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 3-DOF, calibrated INS (0.1 deg) | 1 | 45 | 97.5 | 2.3 [2.2–2.5] | 3.9 [3.5–4.3] | 97.5 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 3-DOF, oracle (INS truth, boresight-calibrated) | 1 | 45 | 97.5 | 2.2 [2.1–2.3] | 3.5 [3.1–4.0] | 97.5 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 3-DOF, star tracker + gyro (30 arcsec) | 1 | 45 | 97.5 | 2.2 [2.1–2.4] | 3.5 [3.2–4.0] | 97.5 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 4-DOF gravity only, MEMS-only (0.5 deg) | 1 | 45 | 97.5 | 3.9 [3.5–4.3] | 8.7 [7.6–9.3] | 97.5 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 4-DOF gravity only, calibrated INS (0.1 deg) | 1 | 45 | 97.5 | 2.3 [2.2–2.4] | 3.9 [3.5–4.3] | 97.5 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 6-DOF + attitude gate, MEMS-only (0.5 deg) | 1 | 45 | 93.9 | 3.9 [3.7–4.2] | 8.0 [7.3–9.0] | 93.9 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 6-DOF + attitude gate, calibrated INS (0.1 deg) | 1 | 45 | 75.8 | 3.5 [3.2–3.8] | 6.4 [5.9–6.8] | 75.8 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 6-DOF + attitude gate, oracle (INS truth, boresight-calibrated) | 1 | 45 | 76.3 | 3.5 [3.2–3.8] | 6.3 [5.8–6.6] | 76.3 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 6-DOF + attitude gate, star tracker + gyro (30 arcsec) | 1 | 45 | 75.8 | 3.5 [3.2–3.7] | 6.3 [5.8–6.6] | 75.8 | 0.0 [0.0–0.0] | 0.71 |
| search8 | 6-DOF PnP (paper) | 1 | 45 | 97.5 | 4.0 [3.7–4.4] | 10.1 [7.8–13.6] | 97.0 | 0.5 [0.0–1.5] | 0.71 |
| heading | 3-DOF, MEMS-only (0.5 deg) | 2 | 91 | 95.4 | 4.6 [4.0–4.9] | 9.5 [8.7–10.3] | 95.4 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, calibrated INS (0.1 deg) | 2 | 91 | 95.4 | 3.2 [3.1–3.4] | 4.8 [4.3–5.2] | 95.4 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, oracle (INS truth, boresight-calibrated) | 2 | 91 | 95.4 | 3.2 [3.1–3.3] | 4.3 [4.1–4.9] | 95.4 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, star tracker + gyro (30 arcsec) | 2 | 91 | 95.4 | 3.2 [3.1–3.3] | 4.3 [4.0–5.0] | 95.4 | 0.0 [0.0–0.0] | 0.14 |
| heading | 6-DOF PnP (paper) | 2 | 91 | 95.4 | 6.3 [5.8–7.0] | 19.9 [14.7–26.3] | 91.9 | 3.6 [1.0–6.1] | 0.14 |
| search8 | 3-DOF, MEMS-only (0.5 deg) | 2 | 91 | 94.4 | 4.6 [4.0–4.9] | 9.6 [8.7–10.6] | 93.9 | 0.5 [0.0–1.5] | 0.69 |
| search8 | 3-DOF, calibrated INS (0.1 deg) | 2 | 91 | 93.9 | 3.3 [3.1–3.5] | 4.9 [4.5–5.3] | 93.9 | 0.0 [0.0–0.0] | 0.69 |
| search8 | 3-DOF, oracle (INS truth, boresight-calibrated) | 2 | 91 | 94.4 | 3.2 [3.1–3.3] | 4.5 [4.2–5.5] | 93.9 | 0.5 [0.0–1.5] | 0.69 |
| search8 | 3-DOF, star tracker + gyro (30 arcsec) | 2 | 91 | 93.9 | 3.2 [3.1–3.3] | 4.4 [4.2–5.2] | 93.9 | 0.0 [0.0–0.0] | 0.69 |
| search8 | 4-DOF gravity only, MEMS-only (0.5 deg) | 2 | 91 | 94.9 | 4.4 [4.0–5.0] | 10.2 [9.2–11.1] | 93.9 | 1.0 [0.0–2.5] | 0.69 |
| search8 | 4-DOF gravity only, calibrated INS (0.1 deg) | 2 | 91 | 94.9 | 3.3 [3.2–3.5] | 5.3 [4.7–5.8] | 93.9 | 1.0 [0.0–2.5] | 0.69 |
| search8 | 6-DOF + attitude gate, MEMS-only (0.5 deg) | 2 | 91 | 84.8 | 6.0 [5.6–6.6] | 12.0 [10.6–14.4] | 84.8 | 0.0 [0.0–0.0] | 0.69 |
| search8 | 6-DOF + attitude gate, calibrated INS (0.1 deg) | 2 | 91 | 53.8 | 5.3 [4.8–5.6] | 7.9 [7.6–8.5] | 53.8 | 0.0 [0.0–0.0] | 0.69 |
| search8 | 6-DOF + attitude gate, oracle (INS truth, boresight-calibrated) | 2 | 91 | 53.3 | 5.2 [4.8–5.6] | 7.9 [7.4–8.5] | 53.3 | 0.0 [0.0–0.0] | 0.69 |
| search8 | 6-DOF + attitude gate, star tracker + gyro (30 arcsec) | 2 | 91 | 53.3 | 5.2 [4.8–5.6] | 7.9 [7.4–8.5] | 53.3 | 0.0 [0.0–0.0] | 0.69 |
| search8 | 6-DOF PnP (paper) | 2 | 91 | 95.4 | 6.6 [6.0–7.3] | 19.6 [15.8–40.0] | 91.4 | 4.1 [1.5–7.1] | 0.69 |
| heading | 3-DOF, MEMS-only (0.5 deg) | 3 | 136 | 90.8 | 5.4 [4.9–6.0] | 10.9 [9.7–11.5] | 90.8 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, calibrated INS (0.1 deg) | 3 | 136 | 90.8 | 4.5 [4.3–4.6] | 6.4 [5.9–6.9] | 90.8 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, oracle (INS truth, boresight-calibrated) | 3 | 136 | 90.8 | 4.5 [4.3–4.5] | 6.2 [5.6–6.6] | 90.8 | 0.0 [0.0–0.0] | 0.14 |
| heading | 3-DOF, star tracker + gyro (30 arcsec) | 3 | 136 | 90.8 | 4.5 [4.4–4.6] | 6.3 [5.5–6.7] | 90.8 | 0.0 [0.0–0.0] | 0.14 |
| heading | 6-DOF PnP (paper) | 3 | 136 | 91.8 | 11.3 [10.1–13.0] | 38.6 [29.9–50.3] | 79.6 | 12.2 [7.7–17.3] | 0.14 |
| search8 | 3-DOF, MEMS-only (0.5 deg) | 3 | 136 | 89.8 | 5.4 [5.0–6.0] | 11.3 [9.8–11.9] | 89.8 | 0.0 [0.0–0.0] | 0.68 |
| search8 | 3-DOF, calibrated INS (0.1 deg) | 3 | 136 | 89.8 | 4.5 [4.3–4.6] | 6.4 [6.1–7.3] | 89.8 | 0.0 [0.0–0.0] | 0.68 |
| search8 | 3-DOF, oracle (INS truth, boresight-calibrated) | 3 | 136 | 89.8 | 4.5 [4.4–4.6] | 6.3 [5.8–7.2] | 89.8 | 0.0 [0.0–0.0] | 0.68 |
| search8 | 3-DOF, star tracker + gyro (30 arcsec) | 3 | 136 | 89.8 | 4.5 [4.4–4.6] | 6.3 [5.7–7.2] | 89.8 | 0.0 [0.0–0.0] | 0.68 |
| search8 | 4-DOF gravity only, MEMS-only (0.5 deg) | 3 | 136 | 89.8 | 5.4 [5.0–6.1] | 11.7 [10.9–12.6] | 88.8 | 1.0 [0.0–2.6] | 0.68 |
| search8 | 4-DOF gravity only, calibrated INS (0.1 deg) | 3 | 136 | 89.8 | 4.7 [4.6–4.9] | 7.1 [6.4–7.7] | 89.3 | 0.5 [0.0–1.5] | 0.68 |
| search8 | 6-DOF + attitude gate, MEMS-only (0.5 deg) | 3 | 136 | 66.3 | 10.2 [9.5–11.1] | 17.7 [16.2–18.6] | 66.3 | 0.0 [0.0–0.0] | 0.68 |
| search8 | 6-DOF + attitude gate, calibrated INS (0.1 deg) | 3 | 136 | 19.4 | 7.6 [6.9–8.2] | 10.4 [9.5–10.8] | 19.4 | 0.0 [0.0–0.0] | 0.68 |
| search8 | 6-DOF + attitude gate, oracle (INS truth, boresight-calibrated) | 3 | 136 | 19.4 | 7.5 [6.8–8.3] | 10.2 [9.1–10.8] | 19.4 | 0.0 [0.0–0.0] | 0.68 |
| search8 | 6-DOF + attitude gate, star tracker + gyro (30 arcsec) | 3 | 136 | 19.4 | 7.5 [6.7–8.3] | 10.2 [9.1–10.8] | 19.4 | 0.0 [0.0–0.0] | 0.68 |
| search8 | 6-DOF PnP (paper) | 3 | 136 | 91.3 | 12.5 [11.2–14.0] | 50.6 [30.8–85.2] | 80.1 | 11.2 [6.6–15.8] | 0.68 |
| heading | 3-DOF, MEMS-only (0.5 deg) | 4 | 181 | 75.9 | 7.3 [6.4–8.0] | 13.1 [11.7–22.1] | 74.4 | 1.5 [0.0–3.6] | 0.13 |
| heading | 3-DOF, calibrated INS (0.1 deg) | 4 | 181 | 75.9 | 6.2 [6.0–6.4] | 10.0 [8.9–19.8] | 74.4 | 1.5 [0.0–3.6] | 0.13 |
| heading | 3-DOF, oracle (INS truth, boresight-calibrated) | 4 | 181 | 75.9 | 6.1 [5.9–6.3] | 10.1 [8.7–23.3] | 74.4 | 1.5 [0.0–3.1] | 0.13 |
| heading | 3-DOF, star tracker + gyro (30 arcsec) | 4 | 181 | 75.9 | 6.1 [6.0–6.3] | 10.2 [8.5–19.1] | 74.4 | 1.5 [0.0–3.6] | 0.13 |
| heading | 6-DOF PnP (paper) | 4 | 181 | 75.9 | 28.8 [23.6–32.4] | 140.7 [86.7–176.1] | 34.9 | 41.0 [34.9–48.2] | 0.13 |
| search8 | 3-DOF, MEMS-only (0.5 deg) | 4 | 181 | 73.8 | 7.2 [6.4–7.7] | 12.4 [11.6–13.7] | 73.3 | 0.5 [0.0–1.5] | 0.64 |
| search8 | 3-DOF, calibrated INS (0.1 deg) | 4 | 181 | 73.8 | 6.1 [5.9–6.3] | 9.4 [8.8–9.9] | 73.3 | 0.5 [0.0–1.6] | 0.64 |
| search8 | 3-DOF, oracle (INS truth, boresight-calibrated) | 4 | 181 | 73.8 | 6.1 [5.9–6.2] | 9.6 [8.1–9.8] | 73.3 | 0.5 [0.0–1.5] | 0.64 |
| search8 | 3-DOF, star tracker + gyro (30 arcsec) | 4 | 181 | 73.8 | 6.0 [5.9–6.3] | 9.5 [8.1–9.7] | 73.3 | 0.5 [0.0–1.5] | 0.64 |
| search8 | 4-DOF gravity only, MEMS-only (0.5 deg) | 4 | 181 | 73.3 | 7.2 [6.5–8.2] | 15.1 [13.1–20.7] | 71.8 | 1.5 [0.0–3.6] | 0.64 |
| search8 | 4-DOF gravity only, calibrated INS (0.1 deg) | 4 | 181 | 73.8 | 6.6 [6.3–6.9] | 10.8 [9.7–13.7] | 72.3 | 1.5 [0.0–3.6] | 0.64 |
| search8 | 6-DOF + attitude gate, MEMS-only (0.5 deg) | 4 | 181 | 30.8 | 13.5 [11.3–14.6] | 19.5 [17.8–20.7] | 30.8 | 0.0 [0.0–0.0] | 0.64 |
| search8 | 6-DOF + attitude gate, calibrated INS (0.1 deg) | 4 | 181 | 4.6 | 6.5 [4.5–10.0] | 11.0 [6.9–11.7] | 4.6 | 0.0 [0.0–0.0] | 0.64 |
| search8 | 6-DOF + attitude gate, oracle (INS truth, boresight-calibrated) | 4 | 181 | 4.6 | 6.6 [4.5–10.0] | 10.1 [6.7–10.1] | 4.6 | 0.0 [0.0–0.0] | 0.64 |
| search8 | 6-DOF + attitude gate, star tracker + gyro (30 arcsec) | 4 | 181 | 4.6 | 6.6 [4.5–10.0] | 10.1 [6.9–10.1] | 4.6 | 0.0 [0.0–0.0] | 0.64 |
| search8 | 6-DOF PnP (paper) | 4 | 181 | 75.9 | 21.0 [18.9–25.6] | 100.1 [82.0–182.4] | 42.6 | 33.3 [26.7–40.0] | 0.64 |
