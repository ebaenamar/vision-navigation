# Experiment 6: Navigability-aware planning

200 random 20 × 20 km maps per condition; 50 Monte Carlo flights per route; speed 15.0 m/s; fix attempt every 10 s; accelerometer noise 0.05 m/s²; accelerometer bias 0.0 mg (1σ per axis, estimated by the filter); routes limited to 15% longer than the shortest.

## Final horizontal position error

| Condition | Integrity monitor | Shortest route, median (m) | Navigability-aware, median (m) | Improvement (%) | Shortest, 95th pct (m) | Navigability-aware, 95th pct (m) | Mean extra length (%) | Maps where route changed (%) | Maps where planner was better (%) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| VPAIR terrain classes only | off | 2.6 | 2.6 | -0.0 | 6.3 | 6.5 | 3.7 | 43.5 | 50.0 |
| VPAIR terrain classes only | on | 2.6 | 2.6 | 0.6 | 6.1 | 6.0 | 3.7 | 43.5 | 49.5 |
| VPAIR terrain classes only | recovery | 2.6 | 2.5 | 0.5 | 5.8 | 5.7 | 3.7 | 43.5 | 52.5 |
| With 20% featureless terrain (assumed) | off | 3.0 | 2.9 | 1.1 | 65.4 | 28.8 | 5.7 | 63.5 | 55.5 |
| With 20% featureless terrain (assumed) | on | 3.0 | 3.0 | 1.4 | 82.3 | 29.7 | 5.7 | 63.5 | 56.0 |
| With 20% featureless terrain (assumed) | recovery | 3.0 | 2.9 | 1.5 | 61.5 | 19.6 | 5.7 | 63.5 | 55.0 |
| Sensitivity: 40% featureless | off | 4.0 | 3.7 | 7.2 | 392.1 | 161.2 | 7.3 | 83.0 | 68.0 |
| Sensitivity: 40% featureless | on | 4.1 | 3.8 | 7.9 | 419.5 | 173.5 | 7.3 | 83.0 | 69.5 |
| Sensitivity: 40% featureless | recovery | 3.9 | 3.7 | 6.7 | 359.9 | 142.4 | 7.3 | 83.0 | 67.5 |
| Sensitivity: 60% featureless | off | 45.4 | 19.9 | 56.1 | 1018.8 | 301.2 | 8.0 | 91.5 | 73.5 |
| Sensitivity: 60% featureless | on | 48.7 | 20.1 | 58.7 | 1030.7 | 315.0 | 8.0 | 91.5 | 76.5 |
| Sensitivity: 60% featureless | recovery | 44.8 | 18.8 | 58.1 | 979.2 | 275.4 | 8.0 | 91.5 | 76.0 |

## Secondary analysis: maximum error along the route

Not part of criterion C7 (which uses final error); reported because staying within a safe corridor depends on the worst error during the flight. Measured after the first 30 s, so that the initial uncertainty before the first fixes is excluded.

| Condition | Integrity monitor | Shortest, median max (m) | Navigability-aware, median max (m) | Improvement (%) | Shortest, 95th pct max (m) | Navigability-aware, 95th pct max (m) |
| --- | --- | --- | --- | --- | --- | --- |
| VPAIR terrain classes only | off | 9.7 | 9.1 | 6.5 | 98.4 | 98.8 |
| VPAIR terrain classes only | on | 10.3 | 10.1 | 2.5 | 59.3 | 63.8 |
| VPAIR terrain classes only | recovery | 10.3 | 10.1 | 2.4 | 22.6 | 22.4 |
| With 20% featureless terrain (assumed) | off | 72.7 | 68.4 | 5.8 | 308.4 | 307.0 |
| With 20% featureless terrain (assumed) | on | 54.2 | 44.5 | 17.8 | 314.3 | 340.2 |
| With 20% featureless terrain (assumed) | recovery | 49.8 | 36.8 | 26.1 | 262.4 | 275.2 |
| Sensitivity: 40% featureless | off | 182.1 | 153.7 | 15.6 | 789.1 | 654.3 |
| Sensitivity: 40% featureless | on | 178.8 | 151.5 | 15.2 | 754.0 | 631.8 |
| Sensitivity: 40% featureless | recovery | 173.7 | 145.5 | 16.2 | 699.3 | 565.7 |
| Sensitivity: 60% featureless | off | 328.6 | 276.4 | 15.9 | 1282.7 | 951.0 |
| Sensitivity: 60% featureless | on | 325.7 | 273.2 | 16.1 | 1272.2 | 945.6 |
| Sensitivity: 60% featureless | recovery | 321.5 | 269.6 | 16.1 | 1213.3 | 898.1 |

## Fix outcomes (% of all attempted fixes, both routes pooled)

| Condition | Integrity monitor | Missed | Correct, accepted | Correct, rejected | False, accepted | False, rejected | Attempts |
| --- | --- | --- | --- | --- | --- | --- | --- |
| VPAIR terrain classes only | off | 0.80 | 98.25 | 0.00 | 0.949 | 0.000 | 2002350 |
| VPAIR terrain classes only | on | 0.80 | 96.21 | 2.04 | 0.002 | 0.947 | 2002350 |
| VPAIR terrain classes only | recovery | 0.80 | 97.03 | 1.22 | 0.002 | 0.947 | 2002350 |
| With 20% featureless terrain (assumed) | off | 16.69 | 82.61 | 0.00 | 0.698 | 0.000 | 2037800 |
| With 20% featureless terrain (assumed) | on | 16.69 | 80.38 | 2.23 | 0.022 | 0.675 | 2037800 |
| With 20% featureless terrain (assumed) | recovery | 16.69 | 81.51 | 1.10 | 0.022 | 0.676 | 2037800 |
| Sensitivity: 40% featureless | off | 38.43 | 60.99 | 0.00 | 0.581 | 0.000 | 2062150 |
| Sensitivity: 40% featureless | on | 38.43 | 59.13 | 1.87 | 0.029 | 0.552 | 2062150 |
| Sensitivity: 40% featureless | recovery | 38.43 | 60.12 | 0.87 | 0.029 | 0.552 | 2062150 |
| Sensitivity: 60% featureless | off | 57.13 | 42.51 | 0.00 | 0.362 | 0.000 | 2072100 |
| Sensitivity: 60% featureless | on | 57.13 | 41.29 | 1.21 | 0.016 | 0.347 | 2072100 |
| Sensitivity: 60% featureless | recovery | 57.13 | 41.92 | 0.59 | 0.016 | 0.347 | 2072100 |

## Fix model (from Experiment 4, VPAIR, SuperPoint+LightGlue, own reference view)

| Terrain | Fix success | False among successes | Median error (m) |
| --- | --- | --- | --- |
| code 1 | 1.000 | 0.000 | 2.98 |
| code 2 | 1.000 | 0.105 | 2.98 |
| code 3 | 1.000 | 0.000 | 3.65 |
| code 5 | 0.833 | 0.000 | 5.91 |
| featureless | 0.000 | 0.000 | – |

False fixes: error 30–150 m in a random direction. Featureless terrain is an assumption, not measured in Experiment 4. Integrity monitor: reject a fix if its normalised innovation exceeds 9.2103 (chi-squared, 2 dof, 99%). "recovery" adds covariance inflation (position standard deviation doubled) after two consecutive rejected fixes; it was added after the plain gate was observed to lock out genuine fixes in some runs.

C7 is met if the improvement is at least 20% with at most 15% extra length.
