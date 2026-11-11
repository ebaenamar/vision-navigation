# Validation on real night-sky images (ESA tetra3 sample data)

Reference solution: tetra3 (independent solver, Hipparcos with proper motion to 2023).
Pixel scale follows from the tetra3 field of view; no lens distortion is modelled.

| Image | FOV (deg) | Variant | Detected | Solved | Identified | IDs also in tetra3 | Boresight vs tetra3 (arcsec) | tetra3 stars, RMS residual under V.I.S.I.O.N. attitude (arcsec) | Own post-fit RMS (arcsec) | tetra3 RMSE (arcsec) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2019-07-29T204726_Alt40_Azi-135_Try1.tiff | 11.43 | paper settings | 8 | yes | 8 | 6/8 | 6.6 | 7.0 | 6.4 | 6.5 |
| 2019-07-29T204726_Alt40_Azi-135_Try1.tiff | 11.43 | paper settings + gradient removal | 33 | yes | 18 | 11/18 | 7.0 | 7.2 | 6.2 | 6.5 |
| 2019-07-29T204726_Alt40_Azi-135_Try1.tiff | 11.43 | tuned (mag 6, 12 stars) + gradient removal | 33 | yes | 18 | 11/18 | 7.0 | 7.2 | 6.2 | 6.5 |
| 2019-07-29T204726_Alt60_Azi-135_Try1.tiff | 11.42 | paper settings | 15 | yes | 15 | 13/15 | 1.1 | 8.2 | 6.7 | 5.0 |
| 2019-07-29T204726_Alt60_Azi-135_Try1.tiff | 11.42 | paper settings + gradient removal | 40 | yes | 19 | 16/19 | 0.3 | 8.2 | 7.2 | 5.0 |
| 2019-07-29T204726_Alt60_Azi-135_Try1.tiff | 11.42 | tuned (mag 6, 12 stars) + gradient removal | 40 | yes | 19 | 16/19 | 0.3 | 8.2 | 7.2 | 5.0 |
