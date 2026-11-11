# Experiment 6: Navigability-aware planning

200 random 20 × 20 km maps per condition; 50 Monte Carlo flights per route; speed 15.0 m/s; fix attempt every 10 s; accelerometer noise 0.05 m/s²; accelerometer bias 2.0 mg (1σ per axis, estimated by the filter); routes limited to 15% longer than the shortest.

## Final horizontal position error

| Condition | Integrity monitor | Shortest route, median (m) | Navigability-aware, median (m) | Improvement (%) | Shortest, 95th pct (m) | Navigability-aware, 95th pct (m) | Mean extra length (%) | Maps where route changed (%) | Maps where planner was better (%) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| VPAIR terrain classes only | off | 2.6 | 2.6 | -0.8 | 6.3 | 6.4 | 14.1 | 100.0 | 48.0 |
| VPAIR terrain classes only | on | 2.6 | 2.6 | -0.1 | 6.8 | 6.6 | 14.1 | 100.0 | 54.0 |
| VPAIR terrain classes only | recovery | 2.6 | 2.6 | -0.1 | 5.9 | 5.8 | 14.1 | 100.0 | 52.0 |
| With 20% featureless terrain (assumed) | off | 3.0 | 2.9 | 3.3 | 71.1 | 29.8 | 13.5 | 99.5 | 52.5 |
| With 20% featureless terrain (assumed) | on | 3.1 | 3.0 | 3.6 | 110.2 | 50.4 | 13.5 | 99.5 | 54.5 |
| With 20% featureless terrain (assumed) | recovery | 3.0 | 2.9 | 3.8 | 67.8 | 22.1 | 13.5 | 99.5 | 53.0 |
| Sensitivity: 40% featureless | off | 4.0 | 3.7 | 7.1 | 510.4 | 168.1 | 12.8 | 100.0 | 69.0 |
| Sensitivity: 40% featureless | on | 4.2 | 3.9 | 7.7 | 676.6 | 238.9 | 12.8 | 100.0 | 73.0 |
| Sensitivity: 40% featureless | recovery | 4.0 | 3.7 | 7.8 | 475.8 | 152.0 | 12.8 | 100.0 | 70.5 |
| Sensitivity: 60% featureless | off | 47.9 | 20.8 | 56.6 | 2980.0 | 357.2 | 11.3 | 99.0 | 80.5 |
| Sensitivity: 60% featureless | on | 54.0 | 22.9 | 57.6 | 4028.6 | 446.1 | 11.3 | 99.0 | 77.5 |
| Sensitivity: 60% featureless | recovery | 48.9 | 20.4 | 58.4 | 2979.9 | 342.2 | 11.3 | 99.0 | 78.5 |

## Secondary analysis: maximum error along the route

Not part of criterion C7 (which uses final error); reported because staying within a safe corridor depends on the worst error during the flight. Measured after the first 30 s, so that the initial uncertainty before the first fixes is excluded.

| Condition | Integrity monitor | Shortest, median max (m) | Navigability-aware, median max (m) | Improvement (%) | Shortest, 95th pct max (m) | Navigability-aware, 95th pct max (m) |
| --- | --- | --- | --- | --- | --- | --- |
| VPAIR terrain classes only | off | 10.0 | 8.4 | 16.2 | 100.7 | 100.1 |
| VPAIR terrain classes only | on | 10.7 | 10.7 | -0.0 | 98.3 | 103.0 |
| VPAIR terrain classes only | recovery | 10.7 | 10.7 | -0.0 | 24.8 | 24.3 |
| With 20% featureless terrain (assumed) | off | 86.9 | 83.3 | 4.1 | 705.3 | 781.0 |
| With 20% featureless terrain (assumed) | on | 72.3 | 64.8 | 10.5 | 902.6 | 1044.2 |
| With 20% featureless terrain (assumed) | recovery | 64.7 | 54.6 | 15.6 | 641.4 | 709.3 |
| Sensitivity: 40% featureless | off | 322.9 | 289.2 | 10.4 | 2538.2 | 2595.8 |
| Sensitivity: 40% featureless | on | 322.9 | 284.5 | 11.9 | 2812.7 | 3103.0 |
| Sensitivity: 40% featureless | recovery | 302.5 | 264.4 | 12.6 | 2328.1 | 2569.4 |
| Sensitivity: 60% featureless | off | 766.0 | 722.2 | 5.7 | 8413.9 | 6083.9 |
| Sensitivity: 60% featureless | on | 778.4 | 725.5 | 6.8 | 9102.4 | 6702.5 |
| Sensitivity: 60% featureless | recovery | 748.0 | 702.1 | 6.1 | 8262.4 | 5868.8 |

## Fix outcomes (% of all attempted fixes, both routes pooled)

| Condition | Integrity monitor | Missed | Correct, accepted | Correct, rejected | False, accepted | False, rejected | Attempts |
| --- | --- | --- | --- | --- | --- | --- | --- |
| VPAIR terrain classes only | off | 0.74 | 98.39 | 0.00 | 0.868 | 0.000 | 2106000 |
| VPAIR terrain classes only | on | 0.74 | 95.55 | 2.84 | 0.004 | 0.864 | 2106000 |
| VPAIR terrain classes only | recovery | 0.74 | 97.14 | 1.25 | 0.003 | 0.865 | 2106000 |
| With 20% featureless terrain (assumed) | off | 16.93 | 82.43 | 0.00 | 0.642 | 0.000 | 2116400 |
| With 20% featureless terrain (assumed) | on | 16.93 | 79.30 | 3.12 | 0.024 | 0.619 | 2116400 |
| With 20% featureless terrain (assumed) | recovery | 16.93 | 81.13 | 1.30 | 0.023 | 0.619 | 2116400 |
| Sensitivity: 40% featureless | off | 40.11 | 59.34 | 0.00 | 0.553 | 0.000 | 2119500 |
| Sensitivity: 40% featureless | on | 40.11 | 56.87 | 2.46 | 0.036 | 0.517 | 2119500 |
| Sensitivity: 40% featureless | recovery | 40.11 | 58.25 | 1.09 | 0.035 | 0.518 | 2119500 |
| Sensitivity: 60% featureless | off | 58.48 | 41.15 | 0.00 | 0.367 | 0.000 | 2105200 |
| Sensitivity: 60% featureless | on | 58.48 | 39.57 | 1.58 | 0.019 | 0.348 | 2105200 |
| Sensitivity: 60% featureless | recovery | 58.48 | 40.32 | 0.83 | 0.019 | 0.348 | 2105200 |

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
