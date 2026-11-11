# Experiment 6b: navigability predicted from the map alone, tested on real outcomes

## visloc (204 locations; 99 with at least one failed or false 6-DOF fix)

| Map-only predictor | Spearman vs constrained-solver inliers [95% CI] | Spearman vs 6-DOF bad-fix rate | AUC: good vs bad location (6-DOF) [95% CI] |
| --- | --- | --- | --- |
| self_match | 0.43 [0.32–0.54] | -0.30 | 0.64 [0.56–0.71] |
| alias_ratio | -0.15 [-0.28–-0.00] | 0.04 | 0.53 [0.45–0.61] |
| n_keypoints | 0.37 [0.24–0.48] | -0.28 | 0.63 [0.55–0.71] |
| grad_mean | 0.41 [0.28–0.52] | -0.29 | 0.64 [0.56–0.71] |
| entropy | 0.35 [0.22–0.48] | -0.28 | 0.63 [0.54–0.70] |

## vpair (199 locations; 116 with at least one failed or false 6-DOF fix)

| Map-only predictor | Spearman vs constrained-solver inliers [95% CI] | Spearman vs 6-DOF bad-fix rate | AUC: good vs bad location (6-DOF) [95% CI] |
| --- | --- | --- | --- |
| self_match | 0.61 [0.49–0.70] | -0.49 | 0.71 [0.64–0.78] |
| n_keypoints | 0.69 [0.60–0.77] | -0.45 | 0.68 [0.61–0.75] |
| grad_mean | 0.58 [0.46–0.68] | -0.33 | 0.61 [0.53–0.69] |
| entropy | 0.35 [0.22–0.48] | -0.24 | 0.56 [0.48–0.64] |

