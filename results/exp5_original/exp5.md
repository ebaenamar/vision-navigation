# Experiment 5: Deep-space position fixes on an Earth–Mars transfer

Hohmann transfer of 259 days; one snapshot fix per day; 20 Monte Carlo passes per setting. Bodies within 20° of the Sun are unobservable. Planet-only fixes need at least two visible bodies.

| Setting | Bearing noise (arcsec/axis) | Pulsar range noise (km) | Planets: available days (%) | Planets: median (km) | Planets: 95th pct (km) | XNAV: median (km) | XNAV: 95th pct (km) | Fused: median (km) | Fused: 95th pct (km) | Fused better than both (% of days) | Mean visible bodies |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Nominal | 13.4 | 5.0 | 100 | 9,591 | 25,749 | 15 | 39 | 15 | 39 | 50 | 3.7 |
| Better camera (5 arcsec) | 5.0 | 5.0 | 100 | 3,598 | 9,679 | 15 | 40 | 15 | 39 | 52 | 3.7 |
| Worse camera (40 arcsec) | 40.0 | 5.0 | 100 | 28,105 | 76,106 | 15 | 39 | 15 | 39 | 50 | 3.7 |
| Better XNAV (2 km) | 13.4 | 2.0 | 100 | 9,510 | 25,591 | 6 | 16 | 6 | 16 | 49 | 3.7 |
| Worse XNAV (20 km) | 13.4 | 20.0 | 100 | 9,514 | 26,829 | 60 | 155 | 59 | 155 | 52 | 3.7 |

C5a: planet-only median ≤ 5,000 km. C5b: fused error lower than both planet-only and XNAV-only.
Assumptions: snapshot fixes (no dynamics filter); coarse prior within ~50,000 km; pulsar phase ambiguity assumed resolved; pulsar range noise is an assumed per-day value anchored to SEXTANT's below-10 km result; ephemeris error not modelled.
