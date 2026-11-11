# Experiment 3b — celestial Earth fix with a fuller error model

200 Monte Carlo runs per scenario; 60° camera; positions on WGS-84. Success = star attitude within 0.1° of truth.

| Scenario | Success (%) | Median error (km) | 95th percentile (km) |
| --- | --- | --- | --- |
| Paper optimistic, reproduced (0.05° tilt; no other terms; mag 6.5) | 100.0 | 5.48 | 6.16 |
| Paper conservative with Kuhlmann roll (1.1° pitch, 2.0° roll) | 100.0 | 213.73 | 474.11 |
| Calibrated sensor, static (0.05° tilt, 0.05° misalignment, DoV, mag 4) | 100.0 | 8.83 | 16.54 |
| Calibrated sensor, steady flight (+0.05 m/s² residual acceleration) | 100.0 | 38.00 | 78.98 |
| Calibrated sensor, manoeuvring flight (+0.3 m/s² residual acceleration) | 100.0 | 218.29 | 461.37 |
| Uncalibrated phone, static (Kuhlmann 1.1°/2.0°, 0.5° misalignment, mag 4) | 100.0 | 217.93 | 498.56 |
