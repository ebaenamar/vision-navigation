# Review-revision experiments for V.I.S.I.O.N.

Based on shivenngupta/vision-navigation @ 7d6fd1e. The original experiments are kept and still reproduce the paper's numbers (`planner.py --accel-bias-mg 0` and `deep_space.py` without `--phased` are bit-identical; the small changes in Exp 1 come from the principal-point fix in Section 1). Tables are in `results/`; large match caches and logs are not committed.

**Data needed (not included):**
- Real star images: `git clone --depth 1 https://github.com/esa/tetra3 tetra3_repo`.
- VPAIR sample (CC BY 4.0): https://zenodo.org/records/6473989, unzipped so that `vpair_sample/` holds `queries/`, `reference_views/` and the pose files.
- UAV-VisLoc example flight 03: https://github.com/IntelliSensing/UAV-VisLoc (see its licence), unzipped into `visloc/raw/` (so that `visloc/raw/03/03.csv` exists).

Hipparcos is downloaded automatically (CDS, NASA HEASARC fallback, with proper motions).

**Run order for the new experiments:** `validate_real_images.py`; `run_experiments.py`; `earth_fix_realistic.py`; `deep_space.py --phased`; `planner.py --accel-bias-mg 2`; `map_match_attitude.py convention` then `run`; `visloc_attitude.py`; `visloc_cache.py`; `map_pnp_eval.py solve vpair`, `solve visloc`, `report`; `navigability.py scores` then `analyse`; `spoof_detect.py` and `spoof_detect.py --robust`.

## 1. Real data: star pipeline on real night-sky images (new)

`validate_real_images.py` runs the paper's detector, identifier and Wahba solver on the two real star-camera images that ship with ESA tetra3 (2019-07-29, 11.4° FOV). Each result is scored against tetra3's independent plate solution.

| Image | Boresight vs tetra3 | tetra3's matched stars, RMS residual under V.I.S.I.O.N. attitude | tetra3's own RMSE |
| --- | --- | --- | --- |
| Alt40 | 6.6–7.0″ | 7.0–7.2″ | 6.5″ |
| Alt60 | 0.3–1.1″ | 8.2″ | 5.0″ |

Both images are solved with the paper's own settings. **This exposed a real bug.** `Camera` placed the optical centre at W/2 instead of (W−1)/2. In simulation the renderer has the same offset, so it cancels and stays invisible. On real images it adds a constant 0.5-pixel bias, which was 25–30″ here. It is fixed in `vision_core.py`.

Only two real images are public with an independent solution, so this is a sanity check, not a statistical validation. The catalogue now comes from NASA HEASARC (CDS was unreachable) and includes proper motion.

## 2. Exp 1: identification scored on catalogue IDs; per-axis 1σ

New Table 2b in `results/results.md`. Nominal case: cross-boresight 1σ = 2.3″ and roll 1σ = 16.6″. These are the industry convention and compare with BCT NST at 6″/40″. Gross wrong solutions are 0%, and 0.17% of stars are misidentified. With 5 false objects, the paper's 8-star search succeeds in 74.5% of frames, while a 12-star search succeeds in 99.5% (but takes 0.8 s per frame). The absolute numbers moved slightly from the paper (10.6″ vs 9.5″ median) because of the centre fix and the new catalogue.

## 3. Exp 2: MEKF with realistic sensors (Table 3b)

| Scenario | Final error | Gyro-only after 1 h | Max in 10-min outage | NEES consistent |
| --- | --- | --- | --- | --- |
| Paper gyro (textbook FOG-grade) | 2.2″ | 2,111″ | 4.4″ | yes |
| Paper gyro, body rate 0 | 1.8″ | 3,600″ (expected 1°/h; the paper's 1,595″ was bias averaging over exactly one revolution) | 5.3″ | yes |
| MEMS gyro (EuRoC ADIS16448 real calibration) | 26″ | ~105° | ~14° | yes |
| MEMS + star-tracker bias 3/3/15″ | 29″ | ~102° | ~13.5° | **no** (73% in bounds) |

With a real MEMS-grade gyro, the "1.14 arcsec" result becomes about 26–29″, and a 10-minute star outage is no longer harmless. The EuRoC Kalibr bias-random-walk value is conservative, so the outage figure is an upper end.

## 4. Exp 3b: celestial Earth fix with a fuller error model (`earth_fix_realistic.py`)

| Scenario | Median | 95th pct |
| --- | --- | --- |
| Paper optimistic, reproduced | 5.5 km | 6.2 km |
| Paper conservative with Kuhlmann 2.0° roll | 214 km | 474 km |
| Calibrated sensor, static (+0.05° misalignment, DoV, mag-4 camera) | 8.8 km | 16.5 km |
| Same, steady flight (0.05 m/s² residual accel) | 38 km | 79 km |
| Same, manoeuvring (0.3 m/s²) | 218 km | 461 km |
| Uncalibrated phone, static | 218 km | 499 km |

The C3 target (≤ 8 km) is not met in any realistic scenario, and even in steady flight the error is about 40 km. Note: the method actually returns astronomical (≈ geodetic) coordinates. The "geocentric" wording in the paper (L401) is a labelling error, not a 21 km error in the method.

## 5. Exp 5: phased Mars arrival (`deep_space.py --phased`)

The original run reproduces the paper exactly (9,591 km). With the arrival placed at Mars, the approach phase is simulated for the first time. Planet-only snapshot errors **grow** to 24,000–28,000 km in the last 75 days, even at 0.8 million km from Mars (`results/exp5_phased/approach_phases.md`). With one nearby body, the range along the line of sight is unobservable in a snapshot. This contradicts L524 and shows that an orbit-determination filter with dynamics (not implemented) is required.

## 6. Exp 6: estimated accelerometer bias in the planner (`planner.py --accel-bias-mg 2`)

`--accel-bias-mg 0` reproduces the paper bit for bit. With a 2 mg bias estimated by the filter, the final error barely changes, so the C7 conclusion stands. The worst error along the route roughly doubles, though (VPAIR + 40% featureless terrain: 174 → 303 m; 60%: 322 → 748 m), and the planner now picks routes 11–14% longer. The route-safety metric, not final error, is where realistic drift matters.

## Not done

Exp 4 with realistic 2-D priors needs the full VPAIR orthophoto mosaic (not in the public sample). Exp 5 still needs a dynamics filter. The integrated 15-state estimator remains unimplemented.

## 7. Exp 4b: does attitude from the star/IMU layer help map matching? (`map_match_attitude.py`, real VPAIR data)

All 199 real VPAIR queries; reference view 0–4 frames ahead (prior error 0–181 m, along-track). Attitude "truth" is VPAIR's GNSS/INS attitude, mapped onto camera axes from the data. The camera–INS boresight (0.65–0.67°, identical in both folds) was estimated on one half of the queries and applied to the other. The attitude is then perturbed at 30″ (star tracker + gyro), 0.1° (calibrated INS) and 0.5° (MEMS only). The paper's 6-DOF baseline reproduces its numbers (3.0 m median, 10.3 m 95th pct at offset 0).

| Prior error | Paper 6-DOF: median / 95th / false > 25 m | 3-DOF, 30″ attitude: median / 95th / false | 3-DOF, 0.5° (MEMS): median / 95th / false |
| --- | --- | --- | --- |
| 0 m | 3.0 / 10.3 m / 1.0% | 1.7 / 2.8 m / 0% | 3.9 / 7.9 m / 0% |
| 45 m | 4.0 / 10.1 m / 0.5% | 2.2 / 3.5 m / 0% | 3.8 / 8.6 m / 0% |
| 91 m | 6.6 / 19.6 m / 4.1% | 3.2 / 4.5 m / 0.5% | 4.6 / 9.6 m / 0.5% |
| 136 m | 12.5 / 50.6 m / 11.2% | 4.5 / 6.5 m / 0% | 5.5 / 11.1 m / 0% |
| 181 m | 21.0 / 100.1 m / 33.3% | 6.0 / 9.5 m / 0.5% | 7.2 / 12.4 m / 0.5% |

Heading-guided matching (one rotation instead of eight) takes 0.14–0.15 s per image instead of 0.66–0.75 s, with the same success rate.

**Indication: yes.** A star-tracker-grade attitude roughly halves the median error. It cuts the 95th percentile by 4–10× and almost eliminates confident false fixes as the prior error grows (33% → 0.5% at 181 m). The share of queries within 25 m at 181 m prior error rises from 43% to 73%. Even a 0.5° MEMS attitude keeps the tails bounded. This is the first piece of evidence that one layer of the architecture measurably improves another.

Limits: one flight, one region, one season. Prior offsets are along-track only. Attitude errors are simulated as white noise on a real INS attitude, not taken from a real star tracker. Success rates at large priors are limited by matching, not by the solver.

## 8. Is it the star-tracker heading, or just the vertical? (decisive baselines, same VPAIR run)

Same cached matches (8-rotation search) and bootstrap 95% CIs over queries. Prior error 181 m:

| Solver | Success | Median | 95th pct | False > 25 m |
| --- | --- | --- | --- | --- |
| 6-DOF PnP (paper) | 75.9% | 21.0 m [18.9–25.6] | 100 m [82–182] | 33.3% [26.7–40.0] |
| 6-DOF + attitude gate (30″) | 4.6% | 6.6 m | 10 m | 0% |
| 4-DOF, gravity only (IMU tilt 0.1°, heading unknown) | 73.8% | 6.6 m [6.3–6.9] | 10.8 m [9.7–13.7] | 1.5% [0–3.6] |
| 4-DOF, gravity only (MEMS tilt 0.5°) | 73.3% | 7.2 m | 15.1 m | 1.5% |
| 3-DOF, full attitude (30″, star tracker + gyro) | 73.8% | 6.0 m [5.9–6.3] | 9.5 m [8.1–9.7] | 0.5% [0–1.5] |

Findings:
1. The paper's large-prior failures are tilt–position ambiguity, not bad matches. Gating 6-DOF solutions by attitude removes them but also removes almost every fix, because the 6-DOF attitude itself is wrong by more than 1°.
2. **Almost all of the gain comes from knowing the vertical**, which an IMU provides. Adding the star-tracker heading improves accuracy only marginally (median 6.6 → 6.0 m, 95th 10.8 → 9.5 m, CIs overlap). Constraining absolute pose with a known vertical is established (e.g. Kukelova et al.; Ventura et al., CVPR 2024), and IMU-aided satellite geolocalization exists (Li, RA-L 2025).
3. What the heading still buys is a single-rotation match instead of an 8-rotation search plus a yaw search (0.14 s vs 0.7 s per image for matching), and robustness when magnetic heading is unavailable.
4. A fixed 0.65° camera–INS boresight exists in VPAIR. Leaving it uncalibrated would put about 4 m of error on every fix at 350 m altitude.

## 9. Second dataset: UAV-VisLoc (`visloc_attitude.py`)

UAV-VisLoc example flight 03 (Taizhou, China; 466 m altitude; Google satellite map at 0.3 m). Unlike VPAIR, priors are displaced in a **random 2-D direction** (0–300 m) and the reference is a crop of a satellite mosaic. The attitude is the **real recorded onboard attitude**, with no simulated noise. After a boresight calibrated on held-out images, it is only good to about 1.5° per axis, and its heading has a direction-dependent bias (crab/gimbal). The focal length (4050 px) and boresight come from every 5th image, and evaluation uses 204 other images. Flat ground is assumed (river plain, no DSM).

| Prior (2-D) | 6-DOF PnP (paper): median / 95th / false > 25 m | 4-DOF, onboard tilt only | 3-DOF, onboard tilt + heading |
| --- | --- | --- | --- |
| 0 m | 13.1 / 40.4 m / 13.2% | 7.2 / 16.6 m / 0% | 8.8 / 16.5 m / 0.5% |
| 100 m | 13.6 / 32.1 m / 11.8% | 7.2 / 16.0 m / 0% | 8.5 / 17.0 m / 0.5% |
| 200 m | 14.2 / 36.2 m / 15.7% | 7.4 / 15.5 m / 0% | 8.6 / 17.2 m / 0% |
| 300 m | 14.1 / 53.8 m / 20.1% | 7.3 / 17.0 m / 0% | 8.4 / 17.6 m / 0% |

**The effect generalises.** On a different region, camera, map source and prior geometry, constraining the vertical halves the median error, cuts the 95th percentile 2–3× and removes confident false fixes (12–20% → 0%), even with a real, mediocre (~1.5°) onboard attitude. The 6-DOF failure is worse here than on VPAIR and appears even with a perfect prior, consistent with tilt–position ambiguity being stronger over flat terrain. The recorded heading adds nothing (slightly worse than gravity only), and attitude gating still fails. Bootstrap 95% CIs in `results/exp4c/exp4c.md`.

## 10. Can navigability be predicted from the map alone? (`navigability.py`, Exp 6b)

This tests the paper's proposal (L207): score each map tile by how well it matches degraded copies of itself, and check whether that predicts the REAL outcome of real drone images at that location. It is compared with classic scene-suitability texture indices. Full table in `results/exp6b/exp6b.md`.

| Dataset | Predictor | Spearman vs real inliers [95% CI] | AUC good vs bad location [95% CI] |
| --- | --- | --- | --- |
| UAV-VisLoc (204) | self-match (paper's idea) | 0.43 [0.32–0.54] | 0.64 [0.56–0.71] |
| | mean gradient | 0.41 [0.28–0.52] | 0.64 [0.56–0.71] |
| | SuperPoint keypoint count | 0.37 [0.24–0.48] | 0.63 [0.55–0.71] |
| | aliasing ratio (false-fix risk) | −0.15 | 0.53 (chance) |
| VPAIR (199) | self-match | 0.61 [0.49–0.70] | 0.71 [0.64–0.78] |
| | SuperPoint keypoint count | 0.69 [0.60–0.77] | 0.68 [0.61–0.75] |
| | mean gradient | 0.58 [0.46–0.68] | 0.61 [0.53–0.69] |

**Result: mostly negative.** Map-only scores do predict real matchability, but only moderately (AUC 0.64–0.71). The paper's self-match proposal is **not better** than a keypoint count or a gradient mean, which are classic, cheap texture indices with overlapping CIs. The aliasing score does not predict false fixes. This is consistent with Section 8: false fixes come from solver geometry (tilt–position ambiguity), not from map aliasing. The ceiling is probably set by what the map cannot see, i.e. the change between map and flight (season, illumination, new construction).

## 11. GNSS spoofing detection with real visual fixes (`spoof_detect.py`, Exp 6c)

This quantifies the paper's claim (L78, L203) that visual fixes can expose GNSS spoofing. Visual-fix outcomes (misses, errors, confident false fixes) are resampled from the real runs of Sections 7–9. One fix every 10 s, GNSS noise 3 m. The spoofer drags GNSS away at 0.1–5 m/s. Alarm thresholds are set empirically for **1 false alarm per hour**. Full tables in `results/exp6c/` (window mean) and `results/exp6c_robust/` (window median).

Median time to alarm / spoofed offset at alarm, 6-fix window (60 s):

| Monitor | Threshold | 0.1 m/s | 0.25 m/s | 0.5 m/s | 1 m/s |
| --- | --- | --- | --- | --- | --- |
| UAV-VisLoc, 6-DOF (paper), mean | 32 m | 320 s / 32 m | 160 s / 40 m | 100 s / 50 m | 70 s / 70 m |
| UAV-VisLoc, 6-DOF (paper), median | 23 m | 220 s / 22 m | 120 s / 30 m | 80 s / 40 m | 50 s / 50 m |
| UAV-VisLoc, 4-DOF gravity, mean | 10 m | 120 s / 12 m | 70 s / 18 m | 50 s / 25 m | 40 s / 40 m |
| VPAIR, 4-DOF gravity, **mean** | 46 m | 490 s / 49 m | 220 s / 55 m | 120 s / 60 m | 80 s / 80 m |
| VPAIR, 4-DOF gravity, **median** | 6 m | 90 s / 9 m | 50 s / 12 m | 40 s / 20 m | 30 s / 30 m |

Findings:
1. With gravity-constrained fixes, a visual monitor detects **slow** spoofing ramps (0.1–0.25 m/s) within 1–2 minutes and 10–20 m of drag, at 1 false alarm per hour. With the paper's 6-DOF fixes the same monitor needs 2–5× larger thresholds, and slow ramps run to 30–50 m before detection.
2. **Integrity is set by the tail, not the median.** On VPAIR, two confident false fixes among 395 attempts inflate a mean-based threshold from 6 m to 46 m. A window median removes the effect. A plain chi-squared gate on single fixes, as the paper proposes, inherits that tail directly (N = 1 thresholds of 25–190 m).
3. Limits: visual errors are resampled independently (temporal correlation along the track is not modelled, so averaging is optimistic). Thresholds come from 395–612 real attempts, so the far tail is uncertain. The spoofer is a simple ramp.

## 12. Soft-gravity MAP PnP (A) and calibrated covariance / protection levels (B) (`gravity_map_pnp.py`, `map_pnp_eval.py`)

All solvers use identical cached correspondences: VPAIR 199 queries × 5 priors, with simulated IMU tilt errors of 30″–2° on the boresight-calibrated INS attitude, and UAV-VisLoc 204 images × 4 2-D priors with the real onboard attitude. Full tables in `results/exp4d/exp4d.md`.

**A. Accuracy (median / 95th pct / false > 25 m, pooled over priors):**

| | 6-DOF (paper) | 4-DOF hard gravity | MAP soft gravity |
| --- | --- | --- | --- |
| VPAIR, tilt 0.1° | 5.7 / 49.2 m / 9.9% | 3.1 / 7.9 / 0.6% | 3.6 / 7.8 / 0.7% |
| VPAIR, tilt 0.5° | | 4.8 / 10.7 / 0.6% | 5.0 / 12.4 / 0.8% |
| VPAIR, tilt 1° | | 7.7 / 16.9 / 0.6% | 5.4 / 19.3 / 2.2% |
| VPAIR, tilt 2° | | 14.1 / 30.6 / 10.9% | 5.6 / 27.0 / 5.3% |
| UAV-VisLoc, onboard | 11.2 / 32.8 / 9.4% | 7.3 / 16.5 / 0.0% | 10.5 / 24.5 / 4.2% |

The soft prior is **not** uniformly better. It ties with the hard constraint for good IMUs, wins on median error only when the tilt is poor (≥ 1°), and wins on the tails only at 2°. With UAV-VisLoc's real attitude (told σ = 1.5°, which is probably too loose) it lets part of the ambiguity back. It always beats the paper's 6-DOF.

**B. Covariance.** The raw Jacobian covariances are badly optimistic for every solver (0–16% of errors inside the 95% ellipse). One inflation factor k fixes coverage within a dataset. The finding is about **transferability**:
- For 6-DOF and MAP (tilt prior 0.1–2°), k is about the same everywhere (k ≈ 26–33, i.e. σ × 5–6) on both datasets. A k calibrated on one dataset gives 92–99% held-out coverage on the other, and errors exceed PL99 in 0.4–1.2% of fixes, close to the nominal 1%.
- For the hard 4-DOF constraint, the covariance ignores tilt error, so k swings from 1,100 to 40,000 with IMU grade. Cross-dataset coverage collapses to 53–89% at 1–2° tilt. The same happens to MAP with a 30″ prior (k ≈ 570). Pairing 4-DOF positions with MAP covariances does not fix it.
- Availability (PL99 ≤ 25 m): MAP 63–100% on VPAIR vs 60% for 6-DOF. On UAV-VisLoc only 12% (hard 4-DOF: 70%, but with a non-transferable k).

**Reading.** Fixing the vertical is the right estimator, but a fix covariance used for integrity must propagate tilt uncertainty. Even then a residual error source (map georeferencing, ground truth, correlated matches) that the Jacobian does not see dominates when the tilt prior is tight. A transferable protection level needs an explicit model of that term, which this study does not provide. A gives a modest, conditional gain; B gives a useful empirical finding, not a finished method.
