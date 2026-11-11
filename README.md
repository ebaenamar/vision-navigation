# V.I.S.I.O.N.

**Visual Inference System for Intelligent Orbital Navigation**

Code and results for the paper *"V.I.S.I.O.N.: Visual Inference System for Intelligent Orbital Navigation — A Cross-Domain Architecture for GPS-Denied Navigation, with Component-Level Evaluation from Orbit to the Ground"* by Shivenn Gupta.

The project evaluates whether low-cost visual and celestial navigation components can provide useful position and orientation where GPS is unavailable, in space and on Earth. Every table and figure in the paper is produced by the scripts below from a fixed random seed.

## What each script does

| Script | Experiment | Output |
| --- | --- | --- |
| `vision_core.py` | Shared library: star detection, star identification, Wahba/SVD attitude, MEKF | (imported by the others) |
| `run_experiments.py` | Experiments 1 and 2: star identification and gyro fusion | `results/results.md`, figures |
| `earth_fix.py` | Experiment 3: position on Earth from stars, gravity and time | `results/exp3.md`, `fig_exp3_sweep.png` |
| `map_match.py` | Experiment 4: map matching on the VPAIR dataset | `results/exp4.md`, `fig_exp4_error_cdf.png` |
| `deep_space.py` | Experiment 5: deep-space position (planets, XNAV, fused) | `results/exp5.md`, `fig_exp5_error_vs_time.png` |
| `planner.py` | Experiment 6: navigability-aware planning and integrity monitor | `results/exp6.md`, example maps |
| `vision_mission_sim.py` | Demonstration trajectory simulator (not part of the evaluation) | — |

## Review-revision experiments

`REVIEW_FIXES.md` describes additional experiments added after an external review: validation on real night-sky images, realistic sensor models, attitude-aided map matching on VPAIR and UAV-VisLoc, a soft-gravity PnP with calibrated covariance, map-only navigability prediction and GNSS spoofing detection. It lists the datasets they need and the run order.

## Setup

Python 3.12 is recommended.

```
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

The Hipparcos star catalogue is downloaded automatically on first run and cached as `hipparcos_mag7.csv`.

For Experiment 4, download the VPAIR sample dataset (Schleiss et al., 2022) from https://zenodo.org/record/6473989 and unzip it into a folder named `vpair_sample` next to the scripts. The dataset is not included here; please follow its own licence.

## Running

```
python run_experiments.py                  # Experiments 1 and 2
python earth_fix.py                        # Experiment 3
python map_match.py --data vpair_sample    # Experiment 4
python deep_space.py                       # Experiment 5
python planner.py                          # Experiment 6
```

Most scripts accept `--quick` for a short test run. Results are written to `results/`.

## Scope and responsible use

This is an experimental research prototype evaluated mainly in simulation. It is not a flight-ready or safety-certified navigation system. It contains no weapon-targeting or target-selection functionality. See the Ethics and dual use section of the paper.

## AI assistance

Parts of the code were drafted with the help of AI assistants and then run, checked and edited by the author, as described in the paper's statement on the use of AI tools.

## Licence

MIT licence; see `LICENSE`.
