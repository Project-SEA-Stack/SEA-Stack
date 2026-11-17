"""Shared helpers for the radiated-wave visualisation spike.

Conventions (match SEA-Stack's HydroData):
  * Time convention e^{+i w t} (WEC-Sim / BEMIO).  Outgoing waves ~ H_m^(2)(k r).
  * B_dim = rho * w * B_file   (radiation_damping/all, "normalized by rho*w")
  * K_dim = rho * K_file       (radiation_damping/impulse_response_fun/K)
  * X_dim = rho * g * X_file   (excitation per unit wave amplitude, N/m or N m/m)
  * Matrices are stacked as [6N x 6N]; DOF order per body: surge, sway, heave,
    roll, pitch, yaw, about the body reference point stored in properties/cg.
  * Wave heading beta: direction the incident wave travels towards, measured
    from global +x, counter-clockwise (BEMIO wave_dir, degrees in file).
"""
from __future__ import annotations

import pathlib
from dataclasses import dataclass, field

import h5py
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[3]
DATA = REPO / "data" / "demos" / "run_seastack"
OUT = pathlib.Path(__file__).resolve().parent / "results"
OUT.mkdir(exist_ok=True)

FILES = {
    "sphere": DATA / "iea_sphere/assets/hydroData/sphere.h5",
    "rm3": DATA / "rm3/assets/hydroData/rm3.h5",
    "f3of": DATA / "f3of/assets/hydroData/f3of.h5",
    "oswec": DATA / "oswec/assets/hydroData/oswec.h5",
    "crane": DATA / "crane/assets/hydroData/crane_vessel.h5",
    "wigley": DATA / "wigley/assets/hydroData/wigley_directional.h5",
    "5sa": DATA / "5sa/assets/hydroData/5sa.h5",
    "5sa_dir": DATA / "5sa/assets/hydroData/5sa_directional.h5",
    "trimaran": DATA / "trimaran/assets/hydroData/trimaran.h5",
}

DOF = ["surge", "sway", "heave", "roll", "pitch", "yaw"]


@dataclass
class Hydro:
    name: str
    code: str
    rho: float
    g: float
    depth: float                     # np.inf for deep water
    w: np.ndarray                    # (Nw,) rad/s
    dirs: np.ndarray                 # (Nd,) rad
    names: list
    cg: np.ndarray                   # (Nb, 3) m
    t: np.ndarray                    # (Nt,) s, RIRF time
    K: np.ndarray                    # (Nt, 6N, 6N) dimensional
    X: np.ndarray                    # (Nw, Nd, 6N) complex, dimensional
    B: np.ndarray | None = None      # (Nw, 6N, 6N) dimensional, stored
    A: np.ndarray | None = None      # (Nw, 6N, 6N) dimensional, stored
    B_file: np.ndarray | None = None  # raw normalised values (for convention tests)
    extra: dict = field(default_factory=dict)

    @property
    def nb(self):
        return self.cg.shape[0]


def _scalar(ds):
    v = ds[()]
    if isinstance(v, bytes):
        return np.inf if v.strip().lower() == b"infinite" else float(v)
    return float(np.ravel(v)[0])


def _stack(f, nb, path):
    blocks = []
    for b in range(nb):
        key = f"body{b + 1}/{path}"
        if key not in f:
            return None
        blocks.append(np.asarray(f[key][()], dtype=float))   # (6, 6N, Nx)
    m = np.concatenate(blocks, axis=0)                        # (6N, 6N, Nx)
    return np.moveaxis(m, -1, 0)                              # (Nx, 6N, 6N)


def load(key: str) -> Hydro:
    path = FILES[key]
    with h5py.File(path, "r") as f:
        sp = f["simulation_parameters"]
        rho, g = _scalar(sp["rho"]), _scalar(sp["g"])
        depth = _scalar(sp["water_depth"])
        w = np.ravel(sp["w"][()]).astype(float)
        dirs = np.radians(np.ravel(sp["wave_dir"][()]).astype(float))
        nb = len([k for k in f if k.startswith("body")])
        names, cg, vol = [], [], []
        for b in range(nb):
            p = f[f"body{b + 1}/properties"]
            names.append(p["name"][()].decode() if "name" in p else f"body{b + 1}")
            cg.append(np.ravel(p["cg"][()]))
            vol.append(float(np.ravel(p["disp_vol"][()])[0]))
        code = f["bem_data/code"][()].decode() if "bem_data/code" in f else "capytaine-script"
        t = np.ravel(f["body1/hydro_coeffs/radiation_damping/impulse_response_fun/t"][()])
        K = _stack(f, nb, "hydro_coeffs/radiation_damping/impulse_response_fun/K") * rho
        B_file = _stack(f, nb, "hydro_coeffs/radiation_damping/all")
        A_file = _stack(f, nb, "hydro_coeffs/added_mass/all")
        X = []
        for b in range(nb):
            e = f[f"body{b + 1}/hydro_coeffs/excitation"]
            x = np.asarray(e["mag"][()]) * np.exp(1j * np.asarray(e["phase"][()]))   # (6, Nd, Nw)
            X.append(x)
        X = np.moveaxis(np.concatenate(X, axis=0), [0, 1, 2], [2, 1, 0]) * rho * g   # (Nw, Nd, 6N)
    B = None if B_file is None else B_file * (rho * w)[:, None, None]
    A = None if A_file is None else A_file * rho
    return Hydro(key, code, rho, g, depth, w, dirs, names, np.array(cg), t, K, X, B, A, B_file,
                 extra={"disp_vol": np.array(vol)})


def power_scaling(h: Hydro):
    """Diagonal D so that D B D has the units of N s/m for every DOF: rotations
    are scaled by L_b = V_b^(1/3), i.e. compared at equal surface speed."""
    L = np.cbrt(np.maximum(h.extra["disp_vol"], 1e-6))
    return np.concatenate([[1, 1, 1, 1 / Lb, 1 / Lb, 1 / Lb] for Lb in L])


# ----------------------------------------------------------------------------- dispersion
def wavenumber(w, g, h):
    """Finite-depth linear dispersion w^2 = g k tanh(k h); h = inf for deep water."""
    w = np.asarray(w, dtype=float)
    k = w ** 2 / g
    if np.isinf(h):
        return k
    k = np.where(k * h < 1e-6, w / np.sqrt(g * h), k)  # shallow start
    for _ in range(60):
        th = np.tanh(k * h)
        fval = g * k * th - w ** 2
        dfdk = g * th + g * k * h * (1.0 - th ** 2)
        k = k - fval / np.maximum(dfdk, 1e-30)
    return k


def group_velocity(w, g, h):
    k = wavenumber(w, g, h)
    cp = np.asarray(w) / np.maximum(k, 1e-30)
    if np.isinf(h):
        return 0.5 * cp
    kh = k * h
    n = 0.5 * (1.0 + np.where(kh < 50, 2 * kh / np.sinh(np.minimum(2 * kh, 700)), 0.0))
    return n * cp


# ----------------------------------------------------------------------------- K <-> B
def b_from_k_direct(t, K, w):
    """B(w) = int_0^T K(t) cos(w t) dt  (inverse of K = 2/pi int B cos dw), trapezoid."""
    c = np.cos(np.outer(w, t))                      # (Nw, Nt)
    wt = np.full_like(t, t[1] - t[0]); wt[0] *= 0.5; wt[-1] *= 0.5
    return np.einsum("wt,tij->wij", c * wt, K)


def b_from_k_fit(t, K, w_grid):
    """Least-squares fit K(t) = 2/pi sum_n B_n dw_n cos(w_n t) on the BEM grid.

    This inverts the discrete cosine sum that run_bem.py (and BEMIO on its own
    interpolated grid) used to build K, so it recovers B at w_grid exactly when
    K was built on that grid.
    """
    dw = np.gradient(w_grid)
    M = (2.0 / np.pi) * np.cos(np.outer(t, w_grid)) * dw[None, :]   # (Nt, Nw)
    nt, n, _ = K.shape
    sol, *_ = np.linalg.lstsq(M, K.reshape(nt, -1), rcond=None)
    return sol.reshape(len(w_grid), n, n)


def body_slice(b):
    return slice(6 * b, 6 * b + 6)


def haskind_b(h: Hydro, X=None, dirs=None):
    """Damping from excitation over headings (Newman):
         B_ij = k / (8 pi rho g c_g) * Re int X_i(beta) X_j(beta)^* dbeta
    Requires headings covering the full circle (handled by caller)."""
    X = h.X if X is None else X
    dirs = h.dirs if dirs is None else dirs
    k = wavenumber(h.w, h.g, h.depth)
    cg = group_velocity(h.w, h.g, h.depth)
    wq = periodic_weights(dirs)
    G = np.einsum("d,wdi,wdj->wij", wq, X, np.conj(X)).real
    return (k / (8 * np.pi * h.rho * h.g * cg))[:, None, None] * G


def periodic_weights(dirs):
    """Quadrature weights for a periodic set of headings in [0, 2pi)."""
    d = np.mod(dirs, 2 * np.pi)
    order = np.argsort(d)
    ds = d[order]
    gaps = np.diff(np.concatenate([ds, [ds[0] + 2 * np.pi]]))
    wq_sorted = 0.5 * (gaps + np.roll(gaps, 1))
    wq = np.empty_like(wq_sorted)
    wq[order] = wq_sorted
    return wq


def full_circle(h: Hydro, symmetric_xz=False, symmetric_yz=False):
    """Return (dirs, X) on a full circle without duplicates.

    Extends partial heading sets using body symmetry (mirror y -> -y about
    x-z plane; mirror x -> -x about y-z plane).  Only valid when the
    geometry and the reference points are symmetric -- caller decides.
    """
    dirs = np.mod(h.dirs, 2 * np.pi)
    X = h.X
    keep = np.unique(np.round(dirs, 9), return_index=True)[1]
    dirs, X = dirs[keep], X[:, keep, :]
    # parity of each DOF under y-mirror and x-mirror
    py = np.tile([1, -1, 1, -1, 1, -1], h.nb)
    px = np.tile([-1, 1, 1, 1, -1, -1], h.nb)
    D, XX = [dirs], [X]
    if symmetric_xz:
        D.append(np.mod(-dirs, 2 * np.pi)); XX.append(X * py)
    if symmetric_yz:
        for dd, xx in list(zip(D, XX)):
            D.append(np.mod(np.pi - dd, 2 * np.pi)); XX.append(xx * px)
    dirs = np.concatenate(D); X = np.concatenate(XX, axis=1)
    keep = np.unique(np.round(dirs, 9), return_index=True)[1]
    return dirs[keep], X[:, keep, :]


def x_wecsim(h: Hydro):
    """Excitation in the e^{+i w t} convention SEA-Stack expects.

    All shipped files now use this convention. Files written by the in-repo
    run_bem.py scripts before the phase fix stored Capytaine's e^{-i w t}
    phases (low-frequency surge excitation at -90 deg instead of +90); convert
    those with scripts/fix_capytaine_excitation_phase.py.
    """
    return h.X


def axisym_circle(h: Hydro, n=72):
    """Full-circle excitation for bodies axisymmetric about the global z axis,
    from the beta = 0 heading.  Forces and moments rotate as vectors."""
    i0 = int(np.argmin(np.abs(np.angle(np.exp(1j * h.dirs)))))
    X0 = x_wecsim(h)[:, i0, :]                              # (Nw, 6N)
    beta = np.linspace(0, 2 * np.pi, n, endpoint=False)
    c, s = np.cos(beta), np.sin(beta)
    X = np.zeros((len(h.w), n, 6 * h.nb), complex)
    for b in range(h.nb):
        o = 6 * b
        fx, fz, my = X0[:, o + 0], X0[:, o + 2], X0[:, o + 4]
        X[:, :, o + 0] = fx[:, None] * c
        X[:, :, o + 1] = fx[:, None] * s
        X[:, :, o + 2] = fz[:, None]
        X[:, :, o + 3] = -my[:, None] * s
        X[:, :, o + 4] = my[:, None] * c
    return beta, X


# ----------------------------------------------------------------------------- far field
# Far-field convention: eta(r, theta, t) = Re{ A(theta) (k r)^(-1/2) e^{i(w t - k r)} } per unit
# body velocity (m per m/s or per rad/s).  Energy: (rho g c_g / 2k) int |A|^2 dtheta = 1/2 U^H B U.
HASKIND_PHASE = -np.exp(1j * np.pi / 4)   # derived two ways (point source; flux balance), see README

# Assumed patterns per DOF (heading frame): harmonic order m and angular function.
PATTERNS = {
    0: (1, np.cos, "cos t"),     # surge
    1: (1, np.sin, "sin t"),     # sway
    2: (0, lambda t: np.ones_like(t), "1"),   # heave
    3: (1, np.sin, "sin t"),     # roll
    4: (1, np.cos, "cos t"),     # pitch
    5: (2, lambda t: np.sin(2 * t), "sin 2t"),  # yaw
}


def pattern_norm(m):
    return 2 * np.pi if m == 0 else np.pi


def haskind_far_field(h: Hydro, dirs, X, body=None):
    """A_j(theta) from Haskind, theta = dirs + pi.  If body is given, the phase
    reference is moved from the global origin to that body's reference point.
    Returns theta (Nd,), A (Nw, Nd, 6N)."""
    k = wavenumber(h.w, h.g, h.depth)
    cg = group_velocity(h.w, h.g, h.depth)
    theta = np.mod(dirs + np.pi, 2 * np.pi)
    A = HASKIND_PHASE * (k / (np.sqrt(8 * np.pi) * h.rho * h.g * cg))[:, None, None] * X
    if body is not None:
        p = h.cg[body, :2]
        shift = np.exp(-1j * np.outer(k, p[0] * np.cos(theta) + p[1] * np.sin(theta)))
        A = A * shift[:, :, None]
    return theta, A


def energy_matched_amplitude(h: Hydro, Bjj, m):
    """|alpha| = sqrt(k B_jj / (rho g c_g int f^2)), per unit velocity."""
    k = wavenumber(h.w, h.g, h.depth)
    cg = group_velocity(h.w, h.g, h.depth)
    return np.sqrt(np.maximum(k * Bjj, 0.0) / (h.rho * h.g * cg * pattern_norm(m)))


def model_phase(m, sign):
    """Point multipole with real strength: eta = c H_m^(2)(kr) f(theta) -> far-field phase
    e^{i(pi/4 + m pi/2)} times the sign of c."""
    return sign * np.exp(1j * (np.pi / 4 + m * np.pi / 2))


def harmonics_from_pattern(theta, wq, A, M):
    """Complex Fourier coefficients a_m, m = -M..M, of A(theta) = sum a_m e^{i m theta}."""
    ms = np.arange(-M, M + 1)
    E = np.exp(-1j * np.outer(ms, theta))                       # (2M+1, Nd)
    return ms, (A @ (E * wq[None, :]).T) / (2 * np.pi)          # (..., 2M+1)


def outgoing_field(ms, a, k, x, y):
    """eta(x, y) = sum_m a_m sqrt(pi/2) e^{-i(pi/4 + |m| pi/2)} H_|m|^(2)(k r) e^{i m theta}
    (exact outgoing solution whose far field is sum a_m e^{i m theta} (kr)^-1/2 e^{-ikr}).
    Also returns d(eta)/dr."""
    from scipy.special import hankel2, h2vp
    r = np.hypot(x, y)
    th = np.arctan2(y, x)
    eta = np.zeros_like(r, dtype=complex)
    deta = np.zeros_like(r, dtype=complex)
    for m, am in zip(ms, a):
        if am == 0:
            continue
        c = am * np.sqrt(np.pi / 2) * np.exp(-1j * (np.pi / 4 + abs(m) * np.pi / 2))
        ang = np.exp(1j * m * th)
        eta += c * hankel2(abs(m), k * r) * ang
        deta += c * k * h2vp(abs(m), k * r) * ang
    return eta, deta


def is_full_circle(dirs, tol_deg=40.0):
    d = np.sort(np.mod(dirs, 2 * np.pi))
    if len(d) < 4:
        return False
    gaps = np.diff(np.concatenate([d, [d[0] + 2 * np.pi]]))
    return np.degrees(gaps.max()) <= tol_deg


def savefig(fig, name):
    p = OUT / name
    fig.savefig(p, dpi=110, bbox_inches="tight")
    print(f"  wrote {p.relative_to(REPO)}")
    return p
