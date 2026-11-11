"""
V.I.S.I.O.N. core library
-------------------------
Star detection, star identification, attitude determination (Wahba's problem
via SVD), a reproduction of the recovered early draft of the summer-2026 code,
and a multiplicative extended Kalman filter (MEKF) for gyro fusion.

Conventions
- R is the attitude matrix that maps INERTIAL vectors to BODY (camera) vectors:
      b = R @ r
- Camera frame: x to the right, y down the image, z along the boresight.
- Angles inside the code are radians unless a name ends in _deg.
"""

import itertools
import time

import numpy as np
from scipy import ndimage


# ---------------------------------------------------------------------------
# Small math helpers
# ---------------------------------------------------------------------------

def radec_to_vec(ra_deg, dec_deg):
    """RA/Dec in degrees -> unit vector(s) in the inertial (ICRS) frame."""
    ra = np.radians(ra_deg)
    dec = np.radians(dec_deg)
    return np.stack([np.cos(dec) * np.cos(ra),
                     np.cos(dec) * np.sin(ra),
                     np.sin(dec)], axis=-1)


def vec_to_radec(v):
    """Unit vector -> (RA, Dec) in degrees."""
    v = np.asarray(v, dtype=float)
    v = v / np.linalg.norm(v)
    ra = np.degrees(np.arctan2(v[1], v[0])) % 360.0
    dec = np.degrees(np.arcsin(np.clip(v[2], -1.0, 1.0)))
    return ra, dec


def skew(v):
    """Cross-product matrix [v x]."""
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def rotm(phi):
    """Rotation matrix exp([phi x]) (Rodrigues' formula)."""
    theta = np.linalg.norm(phi)
    K = skew(phi)
    if theta < 1e-12:
        return np.eye(3) + K
    return (np.eye(3) + np.sin(theta) / theta * K
            + (1.0 - np.cos(theta)) / theta**2 * K @ K)


def random_rotation(rng):
    """Uniformly distributed random rotation matrix."""
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_angle(R):
    """Angle (rad) of a rotation matrix."""
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def attitude_error_rad(R_est, R_true):
    """Total 3-axis attitude error between two attitude matrices (rad)."""
    return rotation_angle(R_est @ R_true.T)


def attitude_error_axes_rad(R_est, R_true):
    """Body-frame rotation vector of the attitude error (rad): components
    x, y are cross-boresight, z is about the boresight (roll)."""
    E = R_est @ R_true.T
    theta = rotation_angle(E)
    w = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    if theta < 1e-12:
        return 0.5 * w
    return w * theta / (2.0 * np.sin(theta))


def boresight_inertial(R):
    """Inertial direction of the camera boresight (+z body)."""
    return R.T @ np.array([0.0, 0.0, 1.0])


def angle_between_rad(u, v):
    u = u / np.linalg.norm(u)
    v = v / np.linalg.norm(v)
    return float(np.arccos(np.clip(u @ v, -1.0, 1.0)))


# ---------------------------------------------------------------------------
# Camera model
# ---------------------------------------------------------------------------

class Camera:
    """Pinhole camera, same model as the original pixel_to_unit_vector()."""

    def __init__(self, width=1024, height=1024, fov_deg=20.0):
        self.width = width
        self.height = height
        self.fov_deg = fov_deg
        self.f = width / (2.0 * np.tan(np.radians(fov_deg) / 2.0))
        # Pixel centres sit at integer coordinates (the convention of
        # scipy.ndimage.center_of_mass and of the renderer), so the optical
        # centre of a W-pixel axis is (W - 1) / 2, not W / 2.  The original
        # W / 2 is invisible in simulation (renderer and solver share it) but
        # puts a constant 0.5-pixel bias on real images; see
        # validate_real_images.py.
        self.cx = (width - 1) / 2.0
        self.cy = (height - 1) / 2.0

    @property
    def diagonal_fov_rad(self):
        half_diag = 0.5 * np.hypot(self.width, self.height)
        return 2.0 * np.arctan(half_diag / self.f)

    @property
    def pixel_scale_arcsec(self):
        return np.degrees(1.0 / self.f) * 3600.0

    def pixel_to_vec(self, x, y):
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        v = np.stack([(x - self.cx) / self.f,
                      (y - self.cy) / self.f,
                      np.ones_like(x)], axis=-1)
        return v / np.linalg.norm(v, axis=-1, keepdims=True)

    def vec_to_pixel(self, v):
        v = np.atleast_2d(v)
        with np.errstate(divide="ignore", invalid="ignore"):
            x = self.f * v[:, 0] / v[:, 2] + self.cx
            y = self.f * v[:, 1] / v[:, 2] + self.cy
        return x, y

    def in_view(self, v, margin=0.0):
        v = np.atleast_2d(v)
        x, y = self.vec_to_pixel(v)
        return ((v[:, 2] > 0) & (x >= margin) & (x < self.width - margin)
                & (y >= margin) & (y < self.height - margin))


# ---------------------------------------------------------------------------
# Star catalog
# ---------------------------------------------------------------------------

class StarCatalog:
    def __init__(self, ids, vecs, mags, source=""):
        order = np.argsort(mags, kind="stable")       # brightest first
        self.ids = np.asarray(ids)[order]
        self.vecs = np.asarray(vecs, dtype=float)[order]
        self.mags = np.asarray(mags, dtype=float)[order]
        self.source = source

    def __len__(self):
        return len(self.ids)

    def subset(self, mag_limit=None, n_brightest=None):
        keep = np.ones(len(self), dtype=bool)
        if mag_limit is not None:
            keep &= self.mags <= mag_limit
        idx = np.flatnonzero(keep)
        if n_brightest is not None:
            idx = idx[:n_brightest]
        return StarCatalog(self.ids[idx], self.vecs[idx], self.mags[idx],
                           self.source)

    @classmethod
    def from_hipparcos(cls, mag_limit=7.0, cache="hipparcos_mag7.csv", epoch=None):
        """
        Load the Hipparcos catalogue (ESA 1997, CDS catalogue I/239).
        Order of attempts:
          1. a local cache file made by an earlier successful run;
          2. Skyfield, which uses ./hip_main.dat if present, else downloads it
             from CDS in France;
          3. the same catalogue from VizieR mirror servers (Harvard, Japan, CDS).
        The first success is saved to `cache` so later runs work offline.
        """
        import os
        import pandas as pd

        if os.path.exists(cache):
            df = pd.read_csv(cache)
            print(f"Hipparcos loaded from local cache {cache} ({len(df)} stars).")
        else:
            df = None
            try:
                from skyfield.api import load
                from skyfield.data import hipparcos
                with load.open(hipparcos.URL) as f:
                    h = hipparcos.load_dataframe(f)
                h = h[h["ra_degrees"].notnull() & h["dec_degrees"].notnull()
                      & h["magnitude"].notnull()]
                df = pd.DataFrame({"hip": h.index.to_numpy(),
                                   "ra": h["ra_degrees"].to_numpy(),
                                   "dec": h["dec_degrees"].to_numpy(),
                                   "mag": h["magnitude"].to_numpy(),
                                   "pm_ra": h["ra_mas_per_year"].to_numpy(),
                                   "pm_dec": h["dec_mas_per_year"].to_numpy()})
                df = df[df["mag"] <= 7.0]
                print("Hipparcos loaded via Skyfield / CDS.")
            except Exception as e:
                print(f"CDS download failed ({type(e).__name__}); trying NASA HEASARC...")
                try:
                    df = _hipparcos_from_heasarc(mag_limit=7.0)
                except Exception as e2:
                    print(f"HEASARC failed ({type(e2).__name__}); trying VizieR mirrors...")
                    df = _hipparcos_from_vizier(mag_limit=7.0)
            df.to_csv(cache, index=False)
            print(f"Saved {len(df)} stars to {cache}; later runs will use this file.")

        df = df[df["mag"] <= mag_limit]
        ra, dec = df["ra"].to_numpy(), df["dec"].to_numpy()
        if epoch is not None:
            if "pm_ra" not in df:
                raise ValueError("Cache has no proper motions; delete it and re-download.")
            # Linear proper-motion propagation from the Hipparcos epoch J1991.25.
            # pm_ra is mu_alpha* = mu_alpha cos(dec), in mas/yr.
            dt = epoch - 1991.25
            pmra = np.nan_to_num(df["pm_ra"].to_numpy())
            pmde = np.nan_to_num(df["pm_dec"].to_numpy())
            dec = dec + pmde * dt / 3.6e6
            ra = ra + pmra * dt / 3.6e6 / np.cos(np.radians(dec))
        vecs = radec_to_vec(ra, dec)
        return cls(df["hip"].to_numpy(), vecs, df["mag"].to_numpy(),
                   source="Hipparcos")

    @classmethod
    def synthetic(cls, n=9000, mag_limit=7.0, seed=0):
        """Random sky for TESTING the code only. Never report these numbers."""
        rng = np.random.default_rng(seed)
        v = rng.normal(size=(n, 3))
        v /= np.linalg.norm(v, axis=1, keepdims=True)
        u = rng.uniform(1e-4, 1.0, size=n)
        mags = mag_limit + np.log10(u) / 0.45   # N(<m) grows ~10^(0.45 m)
        mags = np.clip(mags, -1.5, mag_limit)
        return cls(np.arange(1, n + 1), v, mags, source="synthetic (test only)")



HEASARC_HIP_URL = ("https://heasarc.gsfc.nasa.gov/FTP/heasarc/dbase/tdat_files/"
                   "heasarc_hipparcos.tdat.gz")


def _hipparcos_from_heasarc(mag_limit=7.0, url=HEASARC_HIP_URL):
    """Hipparcos main catalogue (ESA 1997) from NASA HEASARC, with proper motions.
    Positions are ICRS at epoch J1991.25; pm_ra is mu_alpha* in mas/yr."""
    import gzip
    import io
    import urllib.request
    import pandas as pd
    print("  downloading Hipparcos from NASA HEASARC ...")
    raw = urllib.request.urlopen(url, timeout=300).read()
    rows, in_data = [], False
    for line in io.TextIOWrapper(gzip.GzipFile(fileobj=io.BytesIO(raw)), encoding="latin-1"):
        if line.startswith("<DATA>"):
            in_data = True
            continue
        if not in_data or line.startswith("<END>"):
            continue
        p = line.split("|")
        try:
            hip, ra, dec, mag = int(p[5]), float(p[1]), float(p[2]), float(p[7])
        except (ValueError, IndexError):
            continue
        if mag > mag_limit:
            continue
        pmra = float(p[14]) if p[14] else np.nan
        pmde = float(p[15]) if p[15] else np.nan
        rows.append((hip, ra, dec, mag, pmra, pmde))
    df = pd.DataFrame(rows, columns=["hip", "ra", "dec", "mag", "pm_ra", "pm_dec"])
    if len(df) < 1000:
        raise RuntimeError(f"only {len(df)} stars parsed")
    print(f"  Hipparcos loaded from HEASARC ({len(df)} stars).")
    return df


def _hipparcos_from_vizier(mag_limit=7.0):
    """Hipparcos main catalogue (I/239/hip_main) from VizieR mirror servers."""
    import pandas as pd
    from astroquery.vizier import Vizier
    mirrors = ["vizier.cfa.harvard.edu", "vizier.nao.ac.jp", "vizier.cds.unistra.fr"]
    for server in mirrors:
        try:
            print(f"  trying {server} ...")
            v = Vizier(columns=["HIP", "RAICRS", "DEICRS", "Vmag"],
                       column_filters={"Vmag": f"<{mag_limit}"},
                       row_limit=-1, vizier_server=server, timeout=120)
            tables = v.get_catalogs("I/239/hip_main")
            if len(tables) == 0:
                raise RuntimeError("empty result")
            t = tables[0].to_pandas()
            df = pd.DataFrame({"hip": t["HIP"].astype(int),
                               "ra": t["RAICRS"].astype(float),
                               "dec": t["DEICRS"].astype(float),
                               "mag": t["Vmag"].astype(float)}).dropna()
            if len(df) < 1000:
                raise RuntimeError(f"only {len(df)} stars returned")
            print(f"  Hipparcos loaded from {server} ({len(df)} stars).")
            return df
        except Exception as e:
            print(f"  {server} failed: {type(e).__name__}: {e}")
    raise RuntimeError(
        "Could not download Hipparcos from CDS or any VizieR mirror. "
        "Try another network (e.g. a phone hotspot).")

# ---------------------------------------------------------------------------
# Synthetic star-field rendering (ground truth known)
# ---------------------------------------------------------------------------

def render_star_image(catalog, R_true, cam, rng, mag_limit=6.5,
                      psf_sigma=1.2, background=10.0, read_noise=2.0,
                      flux_mag6=200.0, full_well=4095.0, n_false=0):
    """
    Render a 12-bit star image for attitude R_true.
    Returns (image, truth) where truth holds the ids and exact pixel
    positions of every rendered catalog star.
    """
    body = catalog.vecs @ R_true.T
    sel = (catalog.mags <= mag_limit) & cam.in_view(body, margin=-4 * psf_sigma)
    x, y = cam.vec_to_pixel(body[sel])
    mags = catalog.mags[sel]
    flux = flux_mag6 * 10.0 ** (-0.4 * (mags - 6.0))

    if n_false > 0:   # hot pixels / debris / other bright objects
        fx = rng.uniform(0, cam.width, n_false)
        fy = rng.uniform(0, cam.height, n_false)
        fflux = flux_mag6 * 10.0 ** (-0.4 * (rng.uniform(2.0, 5.0, n_false) - 6.0))
        x_all = np.concatenate([x, fx])
        y_all = np.concatenate([y, fy])
        flux_all = np.concatenate([flux, fflux])
    else:
        x_all, y_all, flux_all = x, y, flux

    img = np.full((cam.height, cam.width), background, dtype=float)
    r = int(np.ceil(4 * psf_sigma))
    norm = 1.0 / (2.0 * np.pi * psf_sigma**2)
    for xs, ys, fl in zip(x_all, y_all, flux_all):
        x0, x1 = max(int(np.floor(xs)) - r, 0), min(int(np.floor(xs)) + r + 2, cam.width)
        y0, y1 = max(int(np.floor(ys)) - r, 0), min(int(np.floor(ys)) + r + 2, cam.height)
        if x0 >= x1 or y0 >= y1:
            continue
        gx = np.arange(x0, x1)
        gy = np.arange(y0, y1)
        px = np.exp(-0.5 * ((gx - xs) / psf_sigma) ** 2)
        py = np.exp(-0.5 * ((gy - ys) / psf_sigma) ** 2)
        img[y0:y1, x0:x1] += fl * norm * np.outer(py, px)

    img = rng.poisson(np.clip(img, 0, None)).astype(float)       # shot noise
    img += rng.normal(0.0, read_noise, size=img.shape)            # read noise
    img = np.clip(img, 0.0, full_well)

    truth = {"ids": catalog.ids[sel], "x": x, "y": y, "mags": mags,
             "n_false": n_false}
    return img, truth


# ---------------------------------------------------------------------------
# Star detection (corrected): sub-pixel, intensity-weighted centroids
# ---------------------------------------------------------------------------

def detect_stars(img, k_sigma=5.0, min_area=3, max_area=400):
    """
    Returns (xy, flux): float pixel centroids sorted brightest first.
    The original code used binary contours and int() centroids, which
    throws away sub-pixel accuracy.
    """
    bg = np.median(img)
    noise = 1.4826 * np.median(np.abs(img - bg))
    noise = max(noise, 1e-6)
    mask = img > bg + k_sigma * noise
    lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
    if n == 0:
        return np.zeros((0, 2)), np.zeros(0)
    idx = np.arange(1, n + 1)
    w = np.clip(img - bg, 0.0, None)
    areas = np.asarray(ndimage.sum(mask, lab, idx))
    flux = np.asarray(ndimage.sum(w, lab, idx))
    com = np.asarray(ndimage.center_of_mass(w, lab, idx))   # (row, col)
    keep = (areas >= min_area) & (areas <= max_area)
    xy = com[keep][:, ::-1]
    flux = flux[keep]
    order = np.argsort(-flux)
    return xy[order], flux[order]


# ---------------------------------------------------------------------------
# Wahba's problem (corrected SVD solution, Markley 1988)
# ---------------------------------------------------------------------------

def wahba_svd(b, r, w=None):
    """
    Optimal R (inertial -> body) minimising sum w_i |b_i - R r_i|^2.
    b, r: (N, 3) unit vectors. No centring: these are directions, not points.
    """
    b = np.asarray(b, dtype=float)
    r = np.asarray(r, dtype=float)
    if w is None:
        w = np.ones(len(b))
    B = (b * w[:, None]).T @ r               # sum w_i b_i r_i^T
    U, _, Vt = np.linalg.svd(B)
    d = np.linalg.det(U) * np.linalg.det(Vt)
    return U @ np.diag([1.0, 1.0, d]) @ Vt


# ---------------------------------------------------------------------------
# Star identification (corrected): triangle + 4th-star confirmation,
# then recursive matching of every detected star.
# ---------------------------------------------------------------------------

class StarIdentifier:
    def __init__(self, catalog, cam, id_mag_limit=5.5, tol_deg=0.02,
                 refine_tol_deg=0.03):
        self.cam = cam
        self.tol = np.radians(tol_deg)
        self.refine_tol = np.radians(refine_tol_deg)
        self.full = catalog                                  # used for refinement
        self.cat = catalog.subset(mag_limit=id_mag_limit)    # used for lost-in-space ID
        self.vecs = self.cat.vecs
        self._build_pairs()

    def _build_pairs(self):
        max_sep = self.cam.diagonal_fov_rad
        cos_max = np.cos(max_sep)
        pa, pb, pang = [], [], []
        V = self.vecs
        for i in range(len(V) - 1):
            d = V[i + 1:] @ V[i]
            j = np.flatnonzero(d >= cos_max) + i + 1
            if j.size:
                pa.append(np.full(j.size, i))
                pb.append(j)
                pang.append(np.arccos(np.clip(d[j - i - 1], -1, 1)))
        self.pa = np.concatenate(pa)
        self.pb = np.concatenate(pb)
        self.pang = np.concatenate(pang)
        order = np.argsort(self.pang)
        self.pa, self.pb, self.pang = self.pa[order], self.pb[order], self.pang[order]

    @property
    def n_pairs(self):
        return len(self.pang)

    def _pair_candidates(self, theta):
        lo = np.searchsorted(self.pang, theta - self.tol)
        hi = np.searchsorted(self.pang, theta + self.tol)
        a, b = self.pa[lo:hi], self.pb[lo:hi]
        return np.concatenate([a, b]), np.concatenate([b, a])   # both orderings

    def _stars_at(self, anchors, thetas, exclude):
        """Catalog stars whose angles to each anchor match the given thetas."""
        ok = np.ones(len(self.vecs), dtype=bool)
        for a, th in zip(anchors, thetas):
            ang = np.arccos(np.clip(self.vecs @ self.vecs[a], -1, 1))
            ok &= np.abs(ang - th) < self.tol
        ok[list(exclude)] = False
        return np.flatnonzero(ok)

    def identify(self, xy, max_stars=8, max_triplets=40):
        """
        xy: detected centroids, brightest first.
        Returns dict(R, det_idx, cat_ids, residuals_rad) or None.
        """
        n = min(len(xy), max_stars)
        if n < 4:
            return None
        v = self.cam.pixel_to_vec(xy[:, 0], xy[:, 1])
        A = np.arccos(np.clip(v[:n] @ v[:n].T, -1, 1))

        for t, (i, j, k) in enumerate(itertools.combinations(range(n), 3)):
            if t >= max_triplets:
                break
            others = [m for m in range(n) if m not in (i, j, k)]
            ca, cb = self._pair_candidates(A[i, j])
            for a, b in zip(ca, cb):
                for c in self._stars_at([a, b], [A[i, k], A[j, k]], {a, b}):
                    for rr in others:
                        ds = self._stars_at([a, b, c], [A[i, rr], A[j, rr], A[k, rr]],
                                            {a, b, c})
                        if len(ds) != 1:
                            continue
                        det = [i, j, k, rr]
                        cat = [a, b, c, ds[0]]
                        R = wahba_svd(v[det], self.vecs[cat])
                        res = np.arccos(np.clip(
                            np.sum(v[det] * (self.vecs[cat] @ R.T), axis=1), -1, 1))
                        if res.max() > 2 * self.tol:      # rejects mirror images
                            continue
                        out = self.refine(R, v)
                        if out is not None and len(out["det_idx"]) >= 4:
                            return out
        return None

    def refine(self, R, v_all, iterations=3):
        """Match every detection to the full catalog using the current R."""
        V = self.full.vecs
        out = None
        for _ in range(iterations):
            dots = v_all @ (R @ V.T)            # (n_det, n_cat)
            best = np.argmax(dots, axis=1)
            ang = np.arccos(np.clip(dots[np.arange(len(best)), best], -1, 1))
            ok = ang < self.refine_tol
            # each catalog star may be used once: keep the closest detection
            det_idx = np.flatnonzero(ok)
            order = det_idx[np.argsort(ang[det_idx])]
            used, keep = set(), []
            for d in order:
                if best[d] not in used:
                    used.add(best[d])
                    keep.append(d)
            keep = np.array(sorted(keep), dtype=int)
            if len(keep) < 3:
                return None
            R = wahba_svd(v_all[keep], V[best[keep]])
            res = np.arccos(np.clip(
                np.sum(v_all[keep] * (V[best[keep]] @ R.T), axis=1), -1, 1))
            out = {"R": R, "det_idx": keep, "cat_ids": self.full.ids[best[keep]],
                   "residuals_rad": res}
        return out


# ---------------------------------------------------------------------------
# Reproduction of the RECOVERED EARLY DRAFT of the summer-2026 code (baseline only)
# ---------------------------------------------------------------------------

class LegacyPipeline:
    """
    Faithful re-implementation of the recovered early draft (the final
    summer-2026 code was lost):
      - integer centroids (int() of the centroid)
      - pair-angle catalogue of the 1000 brightest stars, 0.5 deg tolerance,
        first match taken, no consistency check
      - Kabsch with centring and R = V d Wt, boresight = R @ [0, 0, 1]
    The correct camera field of view is supplied (the original defaulted to
    60 deg regardless of the image).
    """

    def __init__(self, catalog, cam, n_stars=1000, tol_deg=0.5):
        self.cam = cam
        self.tol = tol_deg
        sub = catalog.subset(n_brightest=n_stars)
        self.vecs = sub.vecs
        n = len(self.vecs)
        i, j = np.triu_indices(n, k=1)
        ang = np.degrees(np.arccos(np.clip(np.sum(self.vecs[i] * self.vecs[j], axis=1), -1, 1)))
        keys = (ang / tol_deg).astype(int)
        order = np.argsort(keys, kind="stable")       # keeps original insertion order
        self.keys, self.ang = keys[order], ang[order]
        self.i, self.j = i[order], j[order]

    def _first_match(self, angle_deg):
        key = int(angle_deg / self.tol)
        for off in (-1, 0, 1):
            lo = np.searchsorted(self.keys, key + off, side="left")
            hi = np.searchsorted(self.keys, key + off, side="right")
            if hi > lo:
                hit = np.flatnonzero(np.abs(self.ang[lo:hi] - angle_deg) < self.tol)
                if hit.size:
                    m = lo + hit[0]
                    return self.i[m], self.j[m]
        return None

    @staticmethod
    def kabsch_original(P, Q):
        Pc = P - P.mean(axis=0)
        Qc = Q - Q.mean(axis=0)
        C = Pc.T @ Qc
        V, S, Wt = np.linalg.svd(C)
        d = np.eye(3)
        d[2, 2] = np.linalg.det(V) * np.linalg.det(Wt)
        return V @ d @ Wt

    def solve(self, xy, max_stars=30):
        xy = np.floor(xy[:max_stars])                         # int() centroids
        if len(xy) < 2:
            return None
        v = self.cam.pixel_to_vec(xy[:, 0], xy[:, 1])
        P, Q = [], []
        for a, b in itertools.combinations(range(len(v)), 2):
            ang = np.degrees(np.arccos(np.clip(v[a] @ v[b], -1, 1)))
            m = self._first_match(ang)
            if m is None:
                continue
            P += [v[a], v[b]]
            Q += [self.vecs[m[0]], self.vecs[m[1]]]
        if len(P) < 3:
            return None
        R = self.kabsch_original(np.array(P), np.array(Q))
        return R @ np.array([0.0, 0.0, 1.0])   # original "camera forward" in sky


# ---------------------------------------------------------------------------
# Multiplicative EKF (attitude + gyro bias), Lefferts, Markley & Shuster 1982
# ---------------------------------------------------------------------------

def run_mekf(catalog, cam, rng, duration_s=3600.0, dt=0.1, star_dt=1.0,
             n_stars=6, sigma_star_rad=np.radians(10 / 3600),
             sigma_v=np.sqrt(10) * 1e-7, sigma_u=np.sqrt(10) * 1e-10,
             bias0_deg_h=1.0, rate_deg_s=0.1, init_err_deg=0.5,
             log_every_s=1.0, star_outage=None, gyro_only_init_err_rad=None,
             star_bias_sigma_rad=(0.0, 0.0, 0.0)):
    """
    star_bias_sigma_rad: 1-sigma of a constant star-tracker attitude bias
    (body x, y cross-boresight; z roll), drawn once per run.  It stands for
    the bias + low-spatial-frequency error that real trackers specify and
    that, unlike white noise, does not average out.
    Simulates truth, a gyro and a star tracker, and runs:
      - gyro-only propagation (baseline)
      - MEKF fusing gyro + star vectors
    star_outage: optional (t_start, t_end) with no star updates.
    The gyro-only baseline starts from ONE star-tracker-quality fix
    (error gyro_only_init_err_rad, default sigma_star_rad) and is then left
    on uncorrected gyros, so it shows pure drift.
    Returns a dict of time histories.
    """
    deg = np.pi / 180.0
    R_true = random_rotation(rng)
    w_dir = rng.normal(size=3)
    omega = rate_deg_s * deg * w_dir / np.linalg.norm(w_dir)
    b_dir = rng.normal(size=3)
    bias = bias0_deg_h * deg / 3600.0 * b_dir / np.linalg.norm(b_dir)

    e_dir = rng.normal(size=3)
    R_hat = rotm(-init_err_deg * deg * e_dir / np.linalg.norm(e_dir)) @ R_true
    if gyro_only_init_err_rad is None:
        gyro_only_init_err_rad = sigma_star_rad
    g_dir = rng.normal(size=3)
    R_go = rotm(-gyro_only_init_err_rad * g_dir / np.linalg.norm(g_dir)) @ R_true
    beta_hat = np.zeros(3)
    R_bias = rotm(np.asarray(star_bias_sigma_rad) * rng.normal(size=3))
    P = np.diag([(init_err_deg * deg) ** 2] * 3
                + [(2 * bias0_deg_h * deg / 3600.0) ** 2] * 3)

    n_steps = int(round(duration_s / dt))
    star_every = int(round(star_dt / dt))
    log_every = int(round(log_every_s / dt))
    Qd = np.diag([sigma_v**2 * dt] * 3 + [sigma_u**2 * dt] * 3)
    I6 = np.eye(6)

    log = {"t": [], "err_mekf": [], "err_gyro": [], "sigma3": [],
           "bias_true": [], "bias_hat": [], "nees": []}

    for k in range(1, n_steps + 1):
        t = k * dt
        # --- truth and sensors
        omega_m = omega + bias + sigma_v / np.sqrt(dt) * rng.normal(size=3)
        bias = bias + sigma_u * np.sqrt(dt) * rng.normal(size=3)
        R_true = rotm(-omega * dt) @ R_true

        # --- propagation
        w_hat = omega_m - beta_hat
        R_hat = rotm(-w_hat * dt) @ R_hat
        R_go = rotm(-omega_m * dt) @ R_go
        F = np.zeros((6, 6))
        F[:3, :3] = -skew(w_hat)
        F[:3, 3:] = -np.eye(3)
        Phi = I6 + F * dt
        P = Phi @ P @ Phi.T + Qd

        # --- star update
        in_outage = star_outage is not None and star_outage[0] <= t < star_outage[1]
        if k % star_every == 0 and not in_outage:
            body = catalog.vecs @ R_true.T
            vis = np.flatnonzero(cam.in_view(body))[:n_stars]   # brightest first
            if len(vis) >= 2:
                H, z, Rm = [], [], []
                for s in vis:
                    b_meas = R_bias @ body[s] + sigma_star_rad * rng.normal(size=3)
                    b_meas /= np.linalg.norm(b_meas)
                    b_hat = R_hat @ catalog.vecs[s]
                    Hs = np.zeros((3, 6))
                    Hs[:, :3] = skew(b_hat)
                    H.append(Hs)
                    z.append(b_meas - b_hat)
                H = np.vstack(H)
                z = np.concatenate(z)
                Rm = sigma_star_rad**2 * np.eye(len(z))
                S = H @ P @ H.T + Rm
                K = P @ H.T @ np.linalg.inv(S)
                dx = K @ z
                R_hat = rotm(-dx[:3]) @ R_hat
                beta_hat = beta_hat + dx[3:]
                IKH = I6 - K @ H
                P = IKH @ P @ IKH.T + K @ Rm @ K.T          # Joseph form
            P = 0.5 * (P + P.T)

        if k % log_every == 0:
            log["t"].append(t)
            log["err_mekf"].append(attitude_error_rad(R_hat, R_true))
            log["err_gyro"].append(attitude_error_rad(R_go, R_true))
            log["sigma3"].append(3 * np.sqrt(np.trace(P[:3, :3])))
            d = attitude_error_axes_rad(R_true, R_hat)     # true = exp(-[d x]) R_hat convention
            log["nees"].append(float(d @ np.linalg.solve(P[:3, :3], d)))
            log["bias_true"].append(bias.copy())
            log["bias_hat"].append(beta_hat.copy())

    return {key: np.array(val) for key, val in log.items()}


def timed(fn, *args, **kwargs):
    t0 = time.perf_counter()
    out = fn(*args, **kwargs)
    return out, time.perf_counter() - t0
