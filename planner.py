"""
V.I.S.I.O.N. Experiment 6: navigability-aware route planning.

A drone flies from start to goal over terrain where visual map fixes are more
or less available. Between fixes its position error grows (inertial drift).
The planner chooses, among routes at most 15% longer than the shortest, the
one with the lowest predicted final position uncertainty. Monte Carlo flights
compare it with the shortest route.

Fix outcomes are modelled per terrain class from Experiment 4 (VPAIR,
SuperPoint+LightGlue, own reference view), and every attempted fix is one of:
  missed            - no match returned
  correct_accepted  - correct fix, used by the filter
  correct_rejected  - correct fix, wrongly rejected by the integrity monitor
  false_accepted    - wrong fix that looked successful and was used
  false_rejected    - wrong fix caught by the integrity monitor

Usage:
    python planner.py            # full run
    python planner.py --quick    # small test run
Outputs in results/: exp6.md, exp6_summary.json, fig_exp6_example.png
"""

import argparse
import json
import os

import numpy as np
from scipy.ndimage import gaussian_filter
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# ---------------------------------------------------------------------------
# Fix model per terrain class, from Experiment 4 (Table 9 and land-cover rows)
# ---------------------------------------------------------------------------
# Median 2D error m of a circular Gaussian relates to per-axis sigma by
# m = sigma * sqrt(2 ln 2) = 1.1774 sigma.
MED_TO_SIGMA = 1.0 / np.sqrt(2 * np.log(2))

TERRAIN = {
    # name: (fix success prob, false fraction among successes, median error m)
    "code 1":      (160 / 160, 0 / 160, 2.98),
    "code 2":      (19 / 19,   2 / 19,  2.98),
    "code 3":      (8 / 8,     0 / 8,   3.65),
    "code 5":      (10 / 12,   0 / 10,  5.91),
    "featureless": (0.0,       0.0,     np.nan),   # ASSUMED: e.g. water, fresh snow
}
CLASS_NAMES = list(TERRAIN)

# VPAIR sample proportions of codes 1, 2, 3, 5 (160, 19, 8, 12 of 199)
VPAIR_PROPS = np.array([160, 19, 8, 12], dtype=float) / 199

FALSE_FIX_RANGE_M = (30.0, 150.0)   # Exp 4: class-2 false fixes >25 m, one >100 m
CHI2_2DOF_99 = 9.2103               # integrity gate on normalised innovation

# ---------------------------------------------------------------------------
# Vehicle and filter
# ---------------------------------------------------------------------------
SPEED = 15.0          # m/s
FIX_DT = 10.0         # s between fix attempts (150 m)
SIGMA_ACC = 0.05      # m/s^2, MEMS-grade accelerometer noise (white, 1 s)
SIGMA_P0 = 10.0       # m, initial position uncertainty when GPS is lost
SIGMA_V0 = 0.2        # m/s, initial velocity uncertainty

CELL = 500.0          # m
GRID = 40             # 40 x 40 cells = 20 x 20 km
MAX_EXTRA = 0.15      # C7 length limit


SIGMA_BIAS = 0.0      # m/s^2, constant accelerometer bias per flight (1 sigma, per axis);
                      # 0 reproduces the original paper. Set with --accel-bias-mg.


def transition(dt):
    """Error state [px, py, vx, vy, bx, by]: position, velocity and a constant
    accelerometer bias that drives the velocity error (estimated by the filter)."""
    F1 = np.array([[1.0, dt, 0.5 * dt ** 2], [0.0, 1.0, dt], [0.0, 0.0, 1.0]])
    Q1 = SIGMA_ACC ** 2 * np.array([[dt ** 3 / 3, dt ** 2 / 2, 0.0],
                                    [dt ** 2 / 2, dt, 0.0],
                                    [0.0, 0.0, 1e-12]])
    F = np.kron(F1, np.eye(2))      # state [px, py, vx, vy, bx, by]
    Q = np.kron(Q1, np.eye(2))
    return F, Q


def transition_pv(dt):
    """Original 4-state [px, py, vx, vy] model (no accelerometer bias)."""
    F1 = np.array([[1.0, dt], [0.0, 1.0]])
    Q1 = SIGMA_ACC ** 2 * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
    return np.kron(F1, np.eye(2)), np.kron(Q1, np.eye(2))


def setup(sigma_bias):
    """sigma_bias = 0 keeps the original 4-state model exactly (bit-for-bit
    reproduction of the paper); > 0 adds the estimated bias states."""
    global SIGMA_BIAS, F, Q, H, P0, NX
    SIGMA_BIAS = sigma_bias
    F, Q = transition(FIX_DT) if sigma_bias > 0 else transition_pv(FIX_DT)
    NX = F.shape[0]
    H = np.hstack([np.eye(2), np.zeros((2, NX - 2))])
    P0 = np.diag(([SIGMA_P0 ** 2] * 2 + [SIGMA_V0 ** 2] * 2 + [SIGMA_BIAS ** 2] * 2)[:NX])


setup(0.0)


# ---------------------------------------------------------------------------
# Maps and routes
# ---------------------------------------------------------------------------

def make_map(rng, featureless_frac):
    """Random patchy terrain: a smoothed noise field split into class bands
    whose areas match the requested proportions."""
    props = VPAIR_PROPS * (1 - featureless_frac)
    props = np.append(props, featureless_frac)
    field = gaussian_filter(rng.normal(size=(GRID, GRID)), sigma=3.0)
    order = rng.permutation(len(props))          # random class order per map
    edges = np.cumsum(props[order])[:-1]
    q = np.quantile(field, np.clip(edges, 0, 1))
    band = np.searchsorted(q, field)
    classes = order[band]
    return classes


def polyline_samples(points, step):
    """Points every `step` metres along a polyline (including the start)."""
    out = [points[0]]
    carry = 0.0
    for a, b in zip(points[:-1], points[1:]):
        seg = b - a
        L = np.linalg.norm(seg)
        d = step - carry
        while d <= L:
            out.append(a + seg * (d / L))
            d += step
        carry = L - (d - step)
    return np.array(out)


def route_length(points):
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def candidate_routes(rng, start, goal, n=40):
    L = np.linalg.norm(goal - start)
    direction = (goal - start) / L
    normal = np.array([-direction[1], direction[0]])
    routes = [np.array([start, goal])]
    size = GRID * CELL
    tries = 0
    while len(routes) < n and tries < 2000:
        tries += 1
        k = rng.integers(1, 3)
        ts = np.sort(rng.uniform(0.15, 0.85, size=k))
        offs = rng.uniform(-0.3, 0.3, size=k) * L
        wps = [start + t * (goal - start) + o * normal for t, o in zip(ts, offs)]
        pts = np.array([start] + wps + [goal])
        if np.any(pts < 0) or np.any(pts > size):
            continue
        if route_length(pts) <= (1 + MAX_EXTRA) * L:
            routes.append(pts)
    return routes


def classes_along(classes, samples):
    ij = np.clip((samples // CELL).astype(int), 0, GRID - 1)
    return classes[ij[:, 1], ij[:, 0]]


# ---------------------------------------------------------------------------
# Planner prediction and Monte Carlo flight
# ---------------------------------------------------------------------------

def class_params(c):
    p, f, med = TERRAIN[CLASS_NAMES[c]]
    sigma = med * MED_TO_SIGMA if np.isfinite(med) else np.nan
    return p, f, sigma


def predicted_final_sigma(cls_seq):
    """Deterministic covariance prediction: a fix is assumed wherever the
    terrain's success probability is at least 0.5."""
    P = P0.copy()
    for c in cls_seq[1:]:
        P = F @ P @ F.T + Q
        p, _, sigma = class_params(c)
        if p >= 0.5:
            R = sigma ** 2 * np.eye(2)
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            P = (np.eye(NX) - K @ H) @ P
    return float(np.sqrt(np.trace(P[:2, :2])))


def fly(cls_seq, rng, n_runs, integrity):
    """integrity: "off", "on" (plain chi-squared gate) or "recovery" (gate,
    plus doubling the position standard deviation after two consecutive
    rejected apparent fixes, so the filter cannot lock out genuine fixes)."""
    """Vectorised Monte Carlo over n_runs flights along one route.
    Simulates the true navigation error e and the filter covariance P;
    after an update, e <- (I - K H) e + K n, where n is the fix error."""
    N = n_runs
    e = rng.multivariate_normal(np.zeros(NX), P0, size=N)         # (N,NX)
    P = np.repeat(P0[None], N, axis=0)                           # (N,4,4)
    LQ = np.linalg.cholesky(Q)
    counts = dict(missed=0, correct_accepted=0, correct_rejected=0,
                  false_accepted=0, false_rejected=0)
    I4 = np.eye(NX)
    worst = np.zeros(N)
    consec = np.zeros(N, dtype=int)
    for step, c in enumerate(cls_seq[1:], start=1):
        e = e @ F.T + rng.normal(size=(N, NX)) @ LQ.T
        P = F @ P @ F.T + Q
        p, f, sigma = class_params(c)
        got = rng.random(N) < p
        counts["missed"] += int((~got).sum())
        if not got.any():
            if step * FIX_DT > 30.0:
                worst = np.maximum(worst, np.linalg.norm(e[:, :2], axis=1))
            continue
        false = got & (rng.random(N) < f)
        n = rng.normal(size=(N, 2)) * (sigma if np.isfinite(sigma) else 0.0)
        mag = rng.uniform(*FALSE_FIX_RANGE_M, size=N)
        ang = rng.uniform(0, 2 * np.pi, size=N)
        n[false] = np.stack([mag * np.cos(ang), mag * np.sin(ang)], axis=1)[false]
        R = sigma ** 2 * np.eye(2)                    # filter's claimed fix accuracy
        S = H @ P @ H.T + R                           # (N,2,2)
        Sinv = np.linalg.inv(S)
        nu = n - e[:, :2]                             # innovation z - H x_hat
        nis = np.einsum("ni,nij,nj->n", nu, Sinv, nu)
        accept = got & ((nis <= CHI2_2DOF_99) if integrity != "off" else True)
        rejected = got & ~accept
        consec = np.where(rejected, consec + 1, np.where(accept, 0, consec))
        if integrity == "recovery":
            inflate = rejected & (consec >= 2)
            if inflate.any():
                P[inflate, :2, :] *= 2.0
                P[inflate, :, :2] *= 2.0       # position variances x4, cross terms x2
                consec[inflate] = 0
        counts["correct_accepted"] += int((accept & ~false).sum())
        counts["correct_rejected"] += int((got & ~false & ~accept).sum())
        counts["false_accepted"] += int((accept & false).sum())
        counts["false_rejected"] += int((got & false & ~accept).sum())
        if accept.any():
            K = P[accept] @ H.T @ Sinv[accept]        # (M,4,2)
            IKH = I4 - K @ H
            e[accept] = np.einsum("mij,mj->mi", IKH, e[accept]) + np.einsum("mij,mj->mi", K, n[accept])
            P[accept] = IKH @ P[accept]
        if step * FIX_DT > 30.0:          # ignore the initial uncertainty before the first fixes
            worst = np.maximum(worst, np.linalg.norm(e[:, :2], axis=1))
    final = np.linalg.norm(e[:, :2], axis=1)
    return final, worst, counts


# ---------------------------------------------------------------------------
# Experiment
# ---------------------------------------------------------------------------

def run_condition(name, featureless_frac, n_maps, n_runs, seed):
    rng = np.random.default_rng(seed)
    per_map, outcomes, example = [], {"off": None, "on": None, "recovery": None}, None
    for m in range(n_maps):
        classes = make_map(rng, featureless_frac)
        size = GRID * CELL
        while True:
            start, goal = rng.uniform(1000, size - 1000, size=(2, 2))
            if np.linalg.norm(goal - start) >= 12000:
                break
        routes = candidate_routes(rng, start, goal)
        seqs = [classes_along(classes, polyline_samples(r, SPEED * FIX_DT)) for r in routes]
        preds = [predicted_final_sigma(s) for s in seqs]
        best = int(np.argmin(preds))
        L0 = route_length(routes[0])
        rec = {"extra_length_pct": 100 * (route_length(routes[best]) / L0 - 1),
               "planner_changed_route": best != 0}
        for integrity in ("off", "on", "recovery"):
            # common random numbers: identical draws with the monitor on and off
            fs, ws, cs = fly(seqs[0], np.random.default_rng([seed, m, 0]), n_runs, integrity)
            fb, wb, cb = fly(seqs[best], np.random.default_rng([seed, m, 1]), n_runs, integrity)
            tag = integrity
            rec[f"short_{tag}"], rec[f"plan_{tag}"] = fs, fb
            rec[f"wshort_{tag}"], rec[f"wplan_{tag}"] = ws, wb
            for counts in (cs, cb):
                if outcomes[integrity] is None:
                    outcomes[integrity] = {k: 0 for k in counts}
                for k, v in counts.items():
                    outcomes[integrity][k] += v
        per_map.append(rec)
        if example is None and best != 0:
            example = (classes, routes[0], routes[best], preds[0], preds[best])
    return per_map, outcomes, example


def summarise(per_map, tag):
    s = np.concatenate([r[f"short_{tag}"] for r in per_map])
    p = np.concatenate([r[f"plan_{tag}"] for r in per_map])
    med_s, med_p = float(np.median(s)), float(np.median(p))
    extra = np.array([r["extra_length_pct"] for r in per_map])
    ws = np.concatenate([r[f"wshort_{tag}"] for r in per_map])
    wp = np.concatenate([r[f"wplan_{tag}"] for r in per_map])
    map_medians = np.array([[np.median(r[f"short_{tag}"]), np.median(r[f"plan_{tag}"])] for r in per_map])
    return {
        "median_short_m": med_s, "median_plan_m": med_p,
        "p95_short_m": float(np.percentile(s, 95)), "p95_plan_m": float(np.percentile(p, 95)),
        "improvement_pct": 100 * (1 - med_p / med_s) if med_s > 0 else 0.0,
        "mean_extra_length_pct": float(np.mean(extra)),
        "max_extra_length_pct": float(np.max(extra)),
        "pct_maps_route_changed": 100 * float(np.mean([r["planner_changed_route"] for r in per_map])),
        "pct_maps_planner_better": 100 * float(np.mean(map_medians[:, 1] < map_medians[:, 0])),
        "median_max_short_m": float(np.median(ws)), "median_max_plan_m": float(np.median(wp)),
        "p95_max_short_m": float(np.percentile(ws, 95)), "p95_max_plan_m": float(np.percentile(wp, 95)),
        "max_improvement_pct": 100 * (1 - float(np.median(wp)) / float(np.median(ws))),
    }


def plot_example(example, path, title):
    classes, short, plan, ps, pp = example
    cmap = ListedColormap(["#d9d9d9", "#9ecae1", "#a1d99b", "#fdae6b", "#ffffff"])
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ext = [0, GRID * CELL / 1000, 0, GRID * CELL / 1000]
    im = ax.imshow(classes, origin="lower", extent=ext, cmap=cmap, vmin=-0.5, vmax=4.5,
                   interpolation="nearest", alpha=0.9)
    ax.plot(short[:, 0] / 1000, short[:, 1] / 1000, "k--", lw=2,
            label=f"Shortest (predicted σ {ps:.0f} m)")
    ax.plot(plan[:, 0] / 1000, plan[:, 1] / 1000, "-", color="tab:red", lw=2,
            label=f"Navigability-aware (predicted σ {pp:.0f} m)")
    ax.plot(*short[0] / 1000, "ko")
    ax.plot(*short[-1] / 1000, "k*", ms=12)
    cb = fig.colorbar(im, ax=ax, ticks=range(5), fraction=0.046, pad=0.04)
    cb.ax.set_yticklabels(CLASS_NAMES)
    ax.set_xlabel("East (km)")
    ax.set_ylabel("North (km)")
    ax.set_title(title)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps", type=int, default=200)
    ap.add_argument("--runs", type=int, default=50)
    ap.add_argument("--out", default="results")
    ap.add_argument("--seed", type=int, default=6)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--accel-bias-mg", type=float, default=0.0,
                    help="constant accelerometer bias per flight, 1 sigma per axis (milli-g); "
                         "0 = original paper model")
    a = ap.parse_args()
    setup(a.accel_bias_mg * 9.80665e-3)
    if a.quick:
        a.maps, a.runs = 20, 20
    os.makedirs(a.out, exist_ok=True)

    conditions = [("VPAIR terrain classes only", 0.0),
                  ("With 20% featureless terrain (assumed)", 0.2),
                  ("Sensitivity: 40% featureless", 0.4),
                  ("Sensitivity: 60% featureless", 0.6)]
    results, all_outcomes = [], []
    for i, (name, ff) in enumerate(conditions):
        print(f"{name}: {a.maps} maps x {a.runs} flights per route...")
        per_map, outcomes, example = run_condition(name, ff, a.maps, a.runs, a.seed + i)
        for tag in ("off", "on", "recovery"):
            s = summarise(per_map, tag)
            s.update(condition=name, integrity=tag)
            results.append(s)
            print(f"  integrity {tag}: final {s['median_short_m']:.1f} -> {s['median_plan_m']:.1f} m "
                  f"({s['improvement_pct']:.1f}%), max-along-route {s['median_max_short_m']:.1f} -> "
                  f"{s['median_max_plan_m']:.1f} m ({s['max_improvement_pct']:.1f}%), extra length {s['mean_extra_length_pct']:.1f}%")
        for integ, cnt in outcomes.items():
            tot = sum(cnt.values())
            all_outcomes.append({"condition": name, "integrity": integ,
                                 **{k: 100 * v / tot for k, v in cnt.items()}, "attempts": tot})
        if example is not None:
            plot_example(example, os.path.join(a.out, f"fig_exp6_example_{i + 1}.png"), name)

    # ---- report
    L = ["# Experiment 6: Navigability-aware planning", ""]
    if a.quick:
        L.append("> Test run with reduced maps and flights. Do not report.\n")
    L += [f"{a.maps} random 20 × 20 km maps per condition; {a.runs} Monte Carlo flights per route; "
          f"speed {SPEED} m/s; fix attempt every {FIX_DT:.0f} s; accelerometer noise {SIGMA_ACC} m/s²; "
          f"accelerometer bias {a.accel_bias_mg} mg (1σ per axis, estimated by the filter); "
          f"routes limited to {int(MAX_EXTRA * 100)}% longer than the shortest.", "",
          "## Final horizontal position error", "",
          "| Condition | Integrity monitor | Shortest route, median (m) | Navigability-aware, median (m) | Improvement (%) | Shortest, 95th pct (m) | Navigability-aware, 95th pct (m) | Mean extra length (%) | Maps where route changed (%) | Maps where planner was better (%) |",
          "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for s in results:
        L.append(f"| {s['condition']} | {s['integrity']} | {s['median_short_m']:.1f} | {s['median_plan_m']:.1f} | "
                 f"{s['improvement_pct']:.1f} | {s['p95_short_m']:.1f} | {s['p95_plan_m']:.1f} | "
                 f"{s['mean_extra_length_pct']:.1f} | {s['pct_maps_route_changed']:.1f} | {s['pct_maps_planner_better']:.1f} |")
    L += ["", "## Secondary analysis: maximum error along the route", "",
          "Not part of criterion C7 (which uses final error); reported because staying within a safe corridor depends on the worst error during the flight. Measured after the first 30 s, so that the initial uncertainty before the first fixes is excluded.", "",
          "| Condition | Integrity monitor | Shortest, median max (m) | Navigability-aware, median max (m) | Improvement (%) | Shortest, 95th pct max (m) | Navigability-aware, 95th pct max (m) |",
          "| --- | --- | --- | --- | --- | --- | --- |"]
    for s in results:
        L.append(f"| {s['condition']} | {s['integrity']} | {s['median_max_short_m']:.1f} | {s['median_max_plan_m']:.1f} | "
                 f"{s['max_improvement_pct']:.1f} | {s['p95_max_short_m']:.1f} | {s['p95_max_plan_m']:.1f} |")
    L += ["", "## Fix outcomes (% of all attempted fixes, both routes pooled)", "",
          "| Condition | Integrity monitor | Missed | Correct, accepted | Correct, rejected | False, accepted | False, rejected | Attempts |",
          "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for o in all_outcomes:
        L.append(f"| {o['condition']} | {o['integrity']} | {o['missed']:.2f} | {o['correct_accepted']:.2f} | "
                 f"{o['correct_rejected']:.2f} | {o['false_accepted']:.3f} | {o['false_rejected']:.3f} | {o['attempts']} |")
    L += ["", "## Fix model (from Experiment 4, VPAIR, SuperPoint+LightGlue, own reference view)", "",
          "| Terrain | Fix success | False among successes | Median error (m) |", "| --- | --- | --- | --- |"]
    for name, (p, f, med) in TERRAIN.items():
        L.append(f"| {name} | {p:.3f} | {f:.3f} | {med if np.isfinite(med) else '–'} |")
    L += ["", f"False fixes: error {FALSE_FIX_RANGE_M[0]:.0f}–{FALSE_FIX_RANGE_M[1]:.0f} m in a random direction. "
          "Featureless terrain is an assumption, not measured in Experiment 4. "
          f"Integrity monitor: reject a fix if its normalised innovation exceeds {CHI2_2DOF_99} (chi-squared, 2 dof, 99%). "
          "\"recovery\" adds covariance inflation (position standard deviation doubled) after two consecutive rejected fixes; "
          "it was added after the plain gate was observed to lock out genuine fixes in some runs.",
          "", "C7 is met if the improvement is at least 20% with at most 15% extra length."]
    with open(os.path.join(a.out, "exp6.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    with open(os.path.join(a.out, "exp6_summary.json"), "w") as f:
        json.dump({"results": results, "outcomes": all_outcomes}, f, indent=2)
    print(f"\nDone. See {a.out}/exp6.md")


if __name__ == "__main__":
    main()
