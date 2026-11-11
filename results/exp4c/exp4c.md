# Experiment 4b: attitude-aided map matching on VPAIR (199 real queries)

Success = >= 12 inliers. Errors over successful fixes; 'within 25 m' and 'false fix (> 25 m)' over all queries.

| Matching | Solver | Offset (frames) | Mean prior error (m) | Success (%) | Median (m) [95% CI] | 95th pct (m) [95% CI] | Within 25 m (%) | False fixes > 25 m (%) [95% CI] | Median match time (s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| heading | 3-DOF, onboard attitude as recorded | 0 | 0 | 89.2 | 8.3 [7.7–9.3] | 16.5 [15.2–20.7] | 88.7 | 0.5 [0.0–1.5] | 0.20 |
| heading | 6-DOF PnP (paper) | 0 | 0 | 91.7 | 13.4 [12.3–14.9] | 33.6 [29.0–40.2] | 76.0 | 15.7 [11.3–21.1] | 0.20 |
| search8 | 3-DOF, onboard attitude as recorded | 0 | 0 | 89.7 | 8.8 [8.0–9.5] | 16.5 [15.5–21.3] | 89.2 | 0.5 [0.0–1.5] | 0.85 |
| search8 | 4-DOF gravity only, onboard tilt | 0 | 0 | 90.7 | 7.2 [6.7–8.2] | 16.6 [13.1–18.6] | 90.7 | 0.0 [0.0–0.0] | 0.85 |
| search8 | 6-DOF + attitude gate (5 deg), onboard attitude as recorded | 0 | 0 | 85.3 | 12.7 [10.9–13.9] | 30.3 [25.2–33.9] | 77.0 | 8.3 [4.9–12.3] | 0.85 |
| search8 | 6-DOF PnP (paper) | 0 | 0 | 90.7 | 13.1 [11.5–14.6] | 40.4 [31.5–47.1] | 77.5 | 13.2 [8.8–18.1] | 0.85 |
| heading | 3-DOF, onboard attitude as recorded | 100 | 100 | 90.7 | 8.7 [7.9–9.4] | 17.1 [15.7–20.3] | 90.2 | 0.5 [0.0–1.5] | 0.20 |
| heading | 6-DOF PnP (paper) | 100 | 100 | 91.2 | 13.2 [11.9–14.4] | 33.0 [27.6–35.9] | 78.4 | 12.7 [8.3–17.6] | 0.20 |
| search8 | 3-DOF, onboard attitude as recorded | 100 | 100 | 91.7 | 8.5 [7.7–9.2] | 17.0 [15.8–23.1] | 91.2 | 0.5 [0.0–1.5] | 0.84 |
| search8 | 4-DOF gravity only, onboard tilt | 100 | 100 | 91.7 | 7.2 [6.7–8.2] | 16.0 [13.4–18.6] | 91.7 | 0.0 [0.0–0.0] | 0.84 |
| search8 | 6-DOF + attitude gate (5 deg), onboard attitude as recorded | 100 | 100 | 86.3 | 13.0 [11.7–14.3] | 28.1 [25.0–32.0] | 78.4 | 7.8 [4.4–11.8] | 0.84 |
| search8 | 6-DOF PnP (paper) | 100 | 100 | 92.2 | 13.6 [12.2–15.0] | 32.1 [27.8–50.7] | 80.4 | 11.8 [7.8–16.2] | 0.84 |
| heading | 3-DOF, onboard attitude as recorded | 200 | 200 | 90.2 | 8.8 [8.0–9.4] | 17.1 [15.1–20.5] | 89.7 | 0.5 [0.0–1.5] | 0.20 |
| heading | 6-DOF PnP (paper) | 200 | 200 | 90.7 | 12.9 [11.4–14.1] | 34.8 [29.2–46.3] | 78.4 | 12.3 [7.8–17.2] | 0.20 |
| search8 | 3-DOF, onboard attitude as recorded | 200 | 200 | 90.7 | 8.6 [7.9–9.5] | 17.2 [15.8–21.0] | 90.7 | 0.0 [0.0–0.0] | 0.83 |
| search8 | 4-DOF gravity only, onboard tilt | 200 | 200 | 90.2 | 7.4 [6.8–8.2] | 15.5 [13.3–18.0] | 90.2 | 0.0 [0.0–0.0] | 0.83 |
| search8 | 6-DOF + attitude gate (5 deg), onboard attitude as recorded | 200 | 200 | 84.3 | 13.3 [11.8–14.9] | 29.0 [25.9–31.8] | 74.0 | 10.3 [6.4–14.7] | 0.83 |
| search8 | 6-DOF PnP (paper) | 200 | 200 | 91.2 | 14.2 [12.2–16.1] | 36.2 [31.3–55.3] | 75.5 | 15.7 [10.8–21.1] | 0.83 |
| heading | 3-DOF, onboard attitude as recorded | 300 | 300 | 84.3 | 8.5 [7.6–9.3] | 17.3 [15.5–22.1] | 82.8 | 1.5 [0.0–3.4] | 0.19 |
| heading | 6-DOF PnP (paper) | 300 | 300 | 85.8 | 15.2 [13.1–16.1] | 46.5 [31.5–69.3] | 73.5 | 12.3 [7.8–16.7] | 0.19 |
| search8 | 3-DOF, onboard attitude as recorded | 300 | 300 | 84.8 | 8.4 [7.5–9.2] | 17.6 [16.2–22.8] | 84.8 | 0.0 [0.0–0.0] | 0.82 |
| search8 | 4-DOF gravity only, onboard tilt | 300 | 300 | 86.3 | 7.3 [6.7–8.1] | 17.0 [13.5–19.0] | 86.3 | 0.0 [0.0–0.0] | 0.82 |
| search8 | 6-DOF + attitude gate (5 deg), onboard attitude as recorded | 300 | 300 | 75.0 | 13.3 [12.0–14.5] | 30.7 [28.2–33.3] | 64.2 | 10.8 [6.9–15.2] | 0.82 |
| search8 | 6-DOF PnP (paper) | 300 | 300 | 85.3 | 14.1 [13.1–15.1] | 53.8 [38.7–62.7] | 65.2 | 20.1 [14.7–26.0] | 0.82 |
