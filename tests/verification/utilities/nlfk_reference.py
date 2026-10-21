#!/usr/bin/env python3
"""
Independent reference for nonlinear Froude-Krylov (NL FK) forces.

Theory, conventions and pass criteria: docs/theory/NONLINEAR_FROUDE_KRYLOV.md.

This module is deliberately independent of the C++ implementation. It uses no
mesh: body surfaces are analytic (sphere, vertical cylinder), the waterline on
each meridian is found by bisection, and the pressure is integrated with
Gauss-Legendre quadrature along the meridian and the trapezoidal rule in
azimuth (spectrally accurate for smooth, periodic integrands).

Conventions (same as SEA-Stack):
  - World frame: Z up, origin on the mean water level. SI units.
  - Incident wave: eta = r(t) * sum_i a_i cos(k_i . x - w_i t + phi_i),
    r(t) = 0.5 (1 - cos(pi t / T_r)) for t < T_r, else 1.
  - Gauge pressure, zero on the free surface; static part -rho g z.
  - Forces in world axes; moments about the body CG.
  - BEM excitation: F = r(t) * a * Re{X(w) exp(i (w t - phi))}, X dimensional.

Pressure models (keyword `model`):
  static          p = -rho g z, wetted below z = 0 (calm-water NL hydrostatics)
  airy_wheeler    Airy pressure with Wheeler stretching, wetted below eta(x, y)
  hydrostatic     long-wave pressure rho g (eta - z), wetted below eta(x, y)
  gr              Giorgi & Ringwood (2017): flat waterline at eta on the body
                  axis, unstretched Airy pressure, exact azimuth integration
  gr_longwave     as gr, and the horizontal phase k.x is dropped (their
                  long-wave approximation; closed form exists for a sphere)

Usage:
  python nlfk_reference.py selftest
  python nlfk_reference.py regular  --h5 sphere.h5 [--period 7.853982] [--amplitudes 0.1 1 2.5]
  python nlfk_reference.py runaway  --h5 sphere.h5 [--height 10] [--period 8]
  python nlfk_reference.py gr-compare
  (add --out DIR to regular / runaway to write CSV time series)

Dependencies: numpy, h5py.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

DEEP_KH = 89.4  # same deep-water switch as SEA-Stack is_in_deep_water()


# ---------------------------------------------------------------------------
# Incident wave
# ---------------------------------------------------------------------------

def wavenumber(omega: float, depth: float, g: float) -> float:
    """Linear dispersion w^2 = g k tanh(k h); deep water if depth <= 0 or inf."""
    k = omega * omega / g
    if not np.isfinite(depth) or depth <= 0.0:
        return k
    for _ in range(100):
        t = np.tanh(k * depth)
        f = omega * omega - g * k * t
        df = -g * t - g * k * depth * (1.0 - t * t)
        dk = f / df
        k -= dk
        if abs(dk) < 1e-14 * k:
            break
    return k


@dataclass
class WaveComponent:
    amplitude: float      # [m]
    omega: float          # [rad/s]
    k: float              # [rad/m]
    direction: float = 0  # [rad], 0 = +x
    phase: float = 0      # [rad]


class IncidentWave:
    def __init__(self, components, depth=np.inf, g=9.81, rho=1000.0, ramp_duration=0.0):
        self.components = list(components)
        self.depth = float(depth)
        self.g = float(g)
        self.rho = float(rho)
        self.ramp_duration = float(ramp_duration)
        self.a = np.array([c.amplitude for c in self.components])
        self.w = np.array([c.omega for c in self.components])
        self.k = np.array([c.k for c in self.components])
        self.kx = self.k * np.cos([c.direction for c in self.components])
        self.ky = self.k * np.sin([c.direction for c in self.components])
        self.phi = np.array([c.phase for c in self.components])

    @classmethod
    def regular(cls, amplitude, period, depth=np.inf, g=9.81, rho=1000.0, ramp_duration=0.0,
                direction=0.0, phase=0.0):
        w = 2.0 * np.pi / period
        comp = WaveComponent(amplitude, w, wavenumber(w, depth, g), direction, phase)
        return cls([comp], depth, g, rho, ramp_duration)

    @property
    def deep(self) -> bool:
        return (not np.isfinite(self.depth)) or self.depth <= 0.0

    def ramp(self, t: float) -> float:
        if self.ramp_duration <= 0.0 or t >= self.ramp_duration:
            return 1.0
        if t <= 0.0:
            return 0.0
        return 0.5 * (1.0 - np.cos(np.pi * t / self.ramp_duration))

    def _phase(self, x, y, t, drop_x=False):
        x = np.asarray(x, dtype=float)[..., None]
        y = np.asarray(y, dtype=float)[..., None]
        if drop_x:
            return np.broadcast_to(-self.w * t + self.phi, x.shape[:-1] + self.w.shape)
        return self.kx * x + self.ky * y - self.w * t + self.phi

    def eta(self, x, y, t, drop_x=False):
        """Ramped incident elevation [m]."""
        return self.ramp(t) * np.sum(self.a * np.cos(self._phase(x, y, t, drop_x)), axis=-1)

    def depth_factor(self, z):
        """Airy depth factor Z_i(z) = cosh k(z+h) / cosh kh, shape (..., n_comp)."""
        z = np.asarray(z, dtype=float)[..., None]
        if self.deep or np.all(self.k * self.depth > DEEP_KH):
            return np.exp(self.k * z)
        h = self.depth
        return (np.exp(self.k * z) + np.exp(-self.k * (z + 2.0 * h))) / (1.0 + np.exp(-2.0 * self.k * h))

    def stretch(self, z, eta):
        """Wheeler-stretched vertical coordinate; deep-water limit z - eta."""
        if self.deep:
            return z - eta
        h = self.depth
        return h * (z - eta) / (h + eta)

    def pressure(self, x, y, z, t, model: str):
        """Total gauge pressure [Pa] at points (x, y, z)."""
        rho_g = self.rho * self.g
        p = -rho_g * np.asarray(z, dtype=float)
        if model == "static":
            return p
        r = self.ramp(t)
        drop_x = model == "gr_longwave"
        cos_ph = np.cos(self._phase(x, y, t, drop_x))
        if model == "hydrostatic":
            z_factor = 1.0
        elif model == "airy_wheeler":
            z_factor = self.depth_factor(self.stretch(z, self.eta(x, y, t)))
        elif model in ("gr", "gr_longwave"):
            z_factor = self.depth_factor(z)
        else:
            raise ValueError(f"unknown pressure model '{model}'")
        return p + rho_g * r * np.sum(self.a * z_factor * cos_ph, axis=-1)

    def linear_pressure_coefficients(self, x, y, z, model: str):
        """Per-component (cos, sin) pressure shapes on the mean surface, per unit amplitude.

        p_lin_i(t) = rho g a_i Z_i(z) cos(k_i.x + theta_i),  theta_i = phi_i - w_i t
                   = rho g a_i Z_i(z) [cos(k.x) cos(theta) - sin(k.x) sin(theta)].
        Returns (P_c, P_s), each shape (..., n_comp), so that
        p_lin = r(t) sum_i a_i (P_c cos(theta_i) - P_s sin(theta_i)).
        """
        rho_g = self.rho * self.g
        if model == "hydrostatic":
            z_factor = np.ones(np.shape(z) + self.k.shape)
        else:
            z_factor = self.depth_factor(z)
        if model == "gr_longwave":
            kx = np.zeros(np.shape(x) + self.k.shape)
        else:
            kx = self.kx * np.asarray(x)[..., None] + self.ky * np.asarray(y)[..., None]
        return rho_g * z_factor * np.cos(kx), rho_g * z_factor * np.sin(kx)


# ---------------------------------------------------------------------------
# Analytic bodies: quadrature over the wetted surface
# ---------------------------------------------------------------------------

def _gauss(n):
    return np.polynomial.legendre.leggauss(n)


def _waterline_function(wave: IncidentWave, model: str, t: float, axis_xy):
    """Return s(x, y): free-surface height used to clip the hull."""
    if model == "static":
        return lambda x, y: np.zeros(np.broadcast(x, y).shape)
    if model in ("gr", "gr_longwave"):
        s0 = float(wave.eta(axis_xy[0], axis_xy[1], t))
        return lambda x, y: np.full(np.broadcast(x, y).shape, s0)
    return lambda x, y: wave.eta(x, y, t)


@dataclass
class Sphere:
    """Sphere of radius R whose centre sits at `center` relative to the CG [m]."""
    radius: float
    center: tuple = (0.0, 0.0, 0.0)
    n_theta: int = 48
    n_phi: int = 96

    def mean_volume(self, cg_z: float) -> float:
        """Analytic volume below z = 0 for the CG at height cg_z."""
        r = self.radius
        h = np.clip(r - (cg_z + self.center[2]), 0.0, 2.0 * r)
        return np.pi * h * h * (3.0 * r - h) / 3.0

    def wetted_points(self, cg, surface):
        """Quadrature points, outward normals and area weights of the hull below s(x, y)."""
        r = self.radius
        c = np.asarray(cg, dtype=float) + np.asarray(self.center, dtype=float)
        phi = 2.0 * np.pi * np.arange(self.n_phi) / self.n_phi
        cp, sp = np.cos(phi), np.sin(phi)

        def g(theta):  # height above the surface along each meridian
            x = c[0] + r * np.sin(theta) * cp
            y = c[1] + r * np.sin(theta) * sp
            return c[2] + r * np.cos(theta) - surface(x, y)

        samples = np.linspace(0.0, np.pi, 33)[:, None] * np.ones((1, self.n_phi))
        signs = np.sign(np.array([g(s) for s in samples]))
        if np.any(np.sum(np.abs(np.diff(signs, axis=0)) > 0, axis=0) > 1):
            raise ValueError("waterline crosses a meridian more than once (wave too steep for this body)")

        g_top, g_bot = g(np.zeros(self.n_phi)), g(np.full(self.n_phi, np.pi))
        lo, hi = np.zeros(self.n_phi), np.full(self.n_phi, np.pi)
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            above = g(mid) > 0.0
            lo = np.where(above, mid, lo)
            hi = np.where(above, hi, mid)
        theta_c = 0.5 * (lo + hi)
        theta_c = np.where(g_top <= 0.0, 0.0, theta_c)       # meridian fully wet
        theta_c = np.where(g_bot >= 0.0, np.pi, theta_c)     # meridian fully dry

        xg, wg = _gauss(self.n_theta)
        theta = theta_c[:, None] + (np.pi - theta_c)[:, None] * 0.5 * (xg[None, :] + 1.0)
        w_theta = (np.pi - theta_c)[:, None] * 0.5 * wg[None, :]
        st, ct = np.sin(theta), np.cos(theta)
        n = np.stack([st * cp[:, None], st * sp[:, None], ct], axis=-1)
        pts = c + r * n
        weights = r * r * st * w_theta * (2.0 * np.pi / self.n_phi)
        return pts.reshape(-1, 3), n.reshape(-1, 3), weights.ravel()


@dataclass
class VerticalCylinder:
    """Vertical cylinder, flat bottom and top, heights relative to the CG [m].

    The bottom must stay fully wet and the top fully dry (not checked beyond
    a sample grid); otherwise ValueError.
    """
    radius: float
    z_bottom: float
    z_top: float
    n_z: int = 32
    n_r: int = 32
    n_phi: int = 96

    def mean_volume(self, cg_z: float) -> float:
        zb, zt = cg_z + self.z_bottom, cg_z + self.z_top
        return np.pi * self.radius ** 2 * max(0.0, min(0.0, zt) - zb)

    def wetted_points(self, cg, surface):
        cg = np.asarray(cg, dtype=float)
        rad = self.radius
        zb, zt = cg[2] + self.z_bottom, cg[2] + self.z_top
        phi = 2.0 * np.pi * np.arange(self.n_phi) / self.n_phi
        cp, sp = np.cos(phi), np.sin(phi)
        dphi = 2.0 * np.pi / self.n_phi

        # Side wall: wet from the bottom up to min(top, s) on each generator line.
        s_wall = surface(cg[0] + rad * cp, cg[1] + rad * sp)
        z_wet = np.clip(s_wall, zb, zt)
        xg, wg = _gauss(self.n_z)
        z = zb + (z_wet - zb)[:, None] * 0.5 * (xg[None, :] + 1.0)
        wz = (z_wet - zb)[:, None] * 0.5 * wg[None, :]
        side_pts = np.stack([np.broadcast_to(cg[0] + rad * cp[:, None], z.shape),
                             np.broadcast_to(cg[1] + rad * sp[:, None], z.shape), z], axis=-1)
        side_n = np.stack([np.broadcast_to(cp[:, None], z.shape),
                           np.broadcast_to(sp[:, None], z.shape), np.zeros_like(z)], axis=-1)
        side_w = rad * wz * dphi

        # Bottom disc (outward normal -z) and the dry-top check.
        xr, wr = _gauss(self.n_r)
        rr = 0.5 * rad * (xr + 1.0)
        bx = cg[0] + rr[None, :] * cp[:, None]
        by = cg[1] + rr[None, :] * sp[:, None]
        s_disc = surface(bx, by)
        if np.any(s_disc <= zb):
            raise ValueError("cylinder bottom partly dry: not supported by this reference")
        if np.any(s_disc >= zt):
            raise ValueError("cylinder top partly wet: not supported by this reference")
        bot_pts = np.stack([bx, by, np.full_like(bx, zb)], axis=-1)
        bot_n = np.broadcast_to(np.array([0.0, 0.0, -1.0]), bot_pts.shape)
        bot_w = rr[None, :] * 0.5 * rad * wr[None, :] * dphi * np.ones_like(bx)

        pts = np.concatenate([side_pts.reshape(-1, 3), bot_pts.reshape(-1, 3)])
        nrm = np.concatenate([side_n.reshape(-1, 3), bot_n.reshape(-1, 3)])
        wts = np.concatenate([side_w.ravel(), bot_w.ravel()])
        return pts, nrm, wts


def _generalised_normals(pts, nrm, cg):
    """6-vectors [n, (r - r_cg) x n] at each quadrature point."""
    return np.concatenate([nrm, np.cross(pts - np.asarray(cg), nrm)], axis=1)


def pressure_force(body, cg, wave: IncidentWave, t: float, model: str) -> np.ndarray:
    """F_p = -int p n dS (force, then moment about cg) on the wetted surface [N, N m]."""
    surface = _waterline_function(wave, model, t, cg)
    pts, nrm, wts = body.wetted_points(cg, surface)
    p = wave.pressure(pts[:, 0], pts[:, 1], pts[:, 2], t, model)
    return -np.sum((p * wts)[:, None] * _generalised_normals(pts, nrm, cg), axis=0)


def linear_fk_coefficients(body, cg_eq, wave: IncidentWave, model: str):
    """Per-unit-amplitude linear FK 6-vectors (A_i, B_i) on the mean wetted surface.

    F_lin(t) = r(t) sum_i a_i (A_i cos(theta_i) + B_i sin(theta_i)), theta_i = phi_i - w_i t.
    """
    pts, nrm, wts = body.wetted_points(cg_eq, lambda x, y: np.zeros(np.broadcast(x, y).shape))
    pc, ps = wave.linear_pressure_coefficients(pts[:, 0], pts[:, 1], pts[:, 2], model)
    gn = _generalised_normals(pts, nrm, cg_eq) * wts[:, None]
    a_vec = -(pc.T @ gn)  # (n_comp, 6)
    b_vec = ps.T @ gn
    return a_vec, b_vec


def linear_fk_force(a_vec, b_vec, wave: IncidentWave, t: float) -> np.ndarray:
    theta = wave.phi - wave.w * t
    return wave.ramp(t) * np.sum(wave.a[:, None] * (a_vec * np.cos(theta)[:, None]
                                                    + b_vec * np.sin(theta)[:, None]), axis=0)


class NonlinearFKForce:
    """Static + wave pressure on the instantaneous surface minus the mesh linear FK."""

    def __init__(self, body, cg_eq, wave: IncidentWave, model: str):
        self.body, self.wave, self.model = body, wave, model
        self.a_vec, self.b_vec = linear_fk_coefficients(body, cg_eq, wave, model)

    def __call__(self, cg, t):
        return (pressure_force(self.body, cg, self.wave, t, self.model)
                - linear_fk_force(self.a_vec, self.b_vec, self.wave, t))


# ---------------------------------------------------------------------------
# Giorgi & Ringwood (2017) closed form: heaving sphere, long-wave approximation
# ---------------------------------------------------------------------------

def gr_sphere_longwave_heave(radius, zc, eta0, wave: IncidentWave, t):
    """Vertical static + dynamic FK on a sphere (centre height zc), waterline flat at eta0.

    Derived for f(s)^2 = R^2 - (s - zc)^2, so f f' = zc - s, F_z = 2 pi int P (zc - s) ds
    from the keel s1 = zc - R to s2 = min(eta0, zc + R); deep-water, unstretched,
    horizontal phase dropped (Giorgi & Ringwood 2017, Table 1, sphere).
    """
    rho_g = wave.rho * wave.g
    s1, s2 = zc - radius, min(eta0, zc + radius)
    if s2 <= s1:
        return 0.0
    f_st = 2.0 * np.pi * rho_g * ((s2 ** 3 - s1 ** 3) / 3.0 - zc * (s2 ** 2 - s1 ** 2) / 2.0)
    f_dy = 0.0
    r = wave.ramp(t)
    for a, w, k, phi in zip(wave.a, wave.w, wave.k, wave.phi):
        prim = lambda s: (zc - s + 1.0 / k) * np.exp(k * s) / k
        f_dy += 2.0 * np.pi * rho_g * r * a * np.cos(phi - w * t) * (prim(s2) - prim(s1))
    return f_st + f_dy


# ---------------------------------------------------------------------------
# BEM data and a one-degree-of-freedom heave model
# ---------------------------------------------------------------------------

def load_bem_heave(h5_path):
    """Dimensional heave coefficients of body1 (same scaling as SEA-Stack HydroData)."""
    import h5py
    with h5py.File(h5_path, "r") as f:
        sp = f["simulation_parameters"]
        rho = float(np.ravel(sp["rho"][()])[0])
        g = float(np.ravel(sp["g"][()])[0])
        depth = float(np.ravel(sp["water_depth"][()])[0])
        w = np.ravel(sp["w"][()])
        b = f["body1"]
        hc = b["hydro_coeffs"]
        a33 = rho * np.asarray(hc["added_mass/all"])[2, 2, :]
        b33 = rho * w * np.asarray(hc["radiation_damping/all"])[2, 2, :]
        ainf = rho * float(np.asarray(hc["added_mass/inf_freq"])[2, 2])
        k_t = np.ravel(hc["radiation_damping/impulse_response_fun/t"][()])
        k33 = rho * np.asarray(hc["radiation_damping/impulse_response_fun/K"])[2, 2, :]
        c33 = rho * g * float(np.asarray(hc["linear_restoring_stiffness"])[2, 2])
        mag = rho * g * np.asarray(hc["excitation/mag"])[2, 0, :]
        ph = np.asarray(hc["excitation/phase"])[2, 0, :]
        cg = np.ravel(b["properties/cg"][()])
        vol = float(np.ravel(b["properties/disp_vol"][()])[0])
    return dict(rho=rho, g=g, depth=depth, w=w, a33=a33, b33=b33, ainf33=ainf, k_t=k_t, k33=k33,
                c33=c33, x33=mag * np.exp(1j * ph), cg=cg, disp_vol=vol)


def heave_hydro_force(name, body, wave, bem, cg_eq):
    """Return f(z_cg, t) -> vertical hydrostatic/FK force [N] for a heave-only body."""
    if name == "linear":
        z_eq = cg_eq[2]
        weight = bem["rho"] * bem["g"] * bem["disp_vol"]
        return lambda z, t: weight - bem["c33"] * (z - z_eq)
    if name == "nl_hydrostatics":
        return lambda z, t: pressure_force(body, (cg_eq[0], cg_eq[1], z), wave, t, "static")[2]
    if name.startswith("nlfk_"):
        nl = NonlinearFKForce(body, cg_eq, wave, name[len("nlfk_"):])
        return lambda z, t: nl((cg_eq[0], cg_eq[1], z), t)[2]
    raise ValueError(f"unknown hydro model '{name}'")


def simulate_heave(body, wave: IncidentWave, bem, hydro_model: str, *, mass, t_end, dt=0.01,
                   radiation="rirf", pto_damping=0.0, z0_offset=0.0):
    """Heave-only Cummins model. Returns (t, z - z_eq).

    radiation = "constant": (M + A(w)) z'' + B(w) z' (single regular wave on the BEM grid)
    radiation = "rirf":     (M + A_inf) z'' + int K(tau) z'(t - tau) dtau
    """
    cg_eq = np.asarray(bem["cg"], dtype=float)
    f_hydro = heave_hydro_force(hydro_model, body, wave, bem, cg_eq)
    weight = mass * bem["g"]

    x_re = np.interp(wave.w, bem["w"], bem["x33"].real)
    x_im = np.interp(wave.w, bem["w"], bem["x33"].imag)
    x33 = x_re + 1j * x_im
    if np.any(np.asarray([c.direction for c in wave.components]) != 0.0):
        raise ValueError("only heading 0 is supported")

    def f_exc(t):
        return wave.ramp(t) * np.sum(wave.a * np.real(x33 * np.exp(1j * (wave.w * t - wave.phi))))

    if radiation == "constant":
        if len(wave.w) != 1:
            raise ValueError("constant-coefficient radiation needs a single regular wave")
        m_eff = mass + float(np.interp(wave.w[0], bem["w"], bem["a33"]))
        b_rad = float(np.interp(wave.w[0], bem["w"], bem["b33"]))
        kernel = None
    elif radiation == "rirf":
        m_eff = mass + bem["ainf33"]
        b_rad = 0.0
        tk = np.arange(0.0, bem["k_t"][-1] + 0.5 * dt, dt)
        kernel = np.interp(tk, bem["k_t"], bem["k33"]) * dt
        kernel[0] *= 0.5
    else:
        raise ValueError(f"unknown radiation '{radiation}'")

    n = int(round(t_end / dt))
    t = np.arange(n + 1) * dt
    z = np.empty(n + 1)
    v = np.empty(n + 1)
    z[0], v[0] = cg_eq[2] + z0_offset, 0.0

    def accel(zi, vi, ti, f_mem):
        f = f_hydro(zi, ti) - weight + f_exc(ti) - (b_rad + pto_damping) * vi - f_mem
        return f / m_eff

    for i in range(n):
        if kernel is not None:
            m = min(i + 1, len(kernel))
            f_mem = float(np.dot(kernel[:m], v[i::-1][:m]))
        else:
            f_mem = 0.0
        ti, zi, vi = t[i], z[i], v[i]
        k1z, k1v = vi, accel(zi, vi, ti, f_mem)
        k2z, k2v = vi + 0.5 * dt * k1v, accel(zi + 0.5 * dt * k1z, vi + 0.5 * dt * k1v, ti + 0.5 * dt, f_mem)
        k3z, k3v = vi + 0.5 * dt * k2v, accel(zi + 0.5 * dt * k2z, vi + 0.5 * dt * k2v, ti + 0.5 * dt, f_mem)
        k4z, k4v = vi + dt * k3v, accel(zi + dt * k3z, vi + dt * k3v, ti + dt, f_mem)
        z[i + 1] = zi + dt / 6.0 * (k1z + 2 * k2z + 2 * k3z + k4z)
        v[i + 1] = vi + dt / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
    return t, z - cg_eq[2]


def fit_harmonic(t, y, omega, n_periods=10):
    """Fit y ~ Re{Z exp(i w t)} + mean over the last n_periods; return (|Z|, arg Z, mean).

    Same convention as tests/comparison/sphere/compare_*_reg.py.
    """
    sel = t >= t[-1] - n_periods * 2.0 * np.pi / omega
    m = np.column_stack([np.cos(omega * t[sel]), np.sin(omega * t[sel]), np.ones(sel.sum())])
    c, *_ = np.linalg.lstsq(m, y[sel], rcond=None)
    zc = c[0] - 1j * c[1]
    return abs(zc), np.angle(zc), c[2]


# ---------------------------------------------------------------------------
# Self-test: analytic identities (doc section 9, Python side)
# ---------------------------------------------------------------------------

def _bessel_j1(x):
    th = np.linspace(0.0, np.pi, 2001)
    return np.trapezoid(np.cos(th - x * np.sin(th)), th) / np.pi


def self_test(verbose=True) -> bool:
    ok = True

    def check(cond, label):
        nonlocal ok
        ok &= bool(cond)
        if verbose or not cond:
            print(f"{'PASS' if cond else 'FAIL'}  {label}")

    rho, g = 1000.0, 9.81
    sphere = Sphere(radius=5.0, center=(0.0, 0.0, 2.0))  # IEA sphere: CG 2 m below centre

    # 1. Zero pressure on the free surface (both models, deep and finite depth).
    for depth in (np.inf, 30.0):
        wv = IncidentWave.regular(1.5, 6.0, depth=depth, g=g, rho=rho)
        xs = np.linspace(-20.0, 20.0, 41)
        for t in (0.0, 1.3, 4.7):
            eta = wv.eta(xs, 0.0 * xs, t)
            for model in ("airy_wheeler", "hydrostatic"):
                p = wv.pressure(xs, 0.0 * xs, eta, t, model)
                check(np.max(np.abs(p)) <= 1e-9 * rho * g * 1.5,
                      f"p = 0 at eta ({model}, depth={depth}, t={t})")

    # 2. Still water: static pressure force = rho g V (analytic cap) at several heights.
    wv0 = IncidentWave.regular(0.0, 8.0, g=g, rho=rho)
    for cg_z in (-6.0, -3.5, -2.0, 0.0, 1.5):
        f = pressure_force(sphere, (0.0, 0.0, cg_z), wv0, 0.0, "static")
        v = sphere.mean_volume(cg_z)
        check(abs(f[2] - rho * g * v) <= 1e-9 * rho * g * 523.6 + 1e-6,
              f"still water F_z = rho g V (cg_z={cg_z})")
        check(np.max(np.abs(f[[0, 1, 3, 4, 5]])) <= 1e-6 * rho * g * 523.6,
              f"still water: no lateral force or moment (cg_z={cg_z})")
        for model in ("airy_wheeler", "hydrostatic"):
            fm = pressure_force(sphere, (0.0, 0.0, cg_z), wv0, 0.0, model)
            check(np.max(np.abs(fm - f)) <= 1e-9 * rho * g * 523.6,
                  f"zero-amplitude {model} equals static (cg_z={cg_z})")

    # 3. Cylinder linear FK in heave vs rho g a e^{-kd} pi R^2 2 J1(kR)/(kR) (deep water).
    cyl = VerticalCylinder(radius=3.0, z_bottom=-4.0, z_top=3.0)
    for period in (5.0, 8.0, 12.0):
        wv = IncidentWave.regular(1.0, period, g=g, rho=rho)
        a_vec, b_vec = linear_fk_coefficients(cyl, (0.0, 0.0, 0.0), wv, "airy_wheeler")
        amp = abs(a_vec[0, 2] + 1j * b_vec[0, 2])
        kr = wv.k[0] * 3.0
        ref = rho * g * np.exp(-wv.k[0] * 4.0) * np.pi * 9.0 * 2.0 * _bessel_j1(kr) / kr
        check(abs(amp / ref - 1.0) <= 1e-6, f"cylinder linear FK heave vs analytic (T={period} s)")

    # 4. Fixed body: correction scales as a^2 (a doubled wave force would scale as a).
    # The waterline must be off the equator: there dA_w/dz = 0 and the long-wave
    # correction drops to O(a^3).
    cg_eq = (0.0, 0.0, -2.0)
    v0 = rho * g * sphere.mean_volume(-2.0)
    amps = np.array([1e-3, 1e-2, 1e-1])
    for cg_z, expected in ((-3.0, {"airy_wheeler": 2.0, "hydrostatic": 2.0}),
                           (-2.0, {"airy_wheeler": 2.0, "hydrostatic": 3.0})):
        cg_fix = (0.0, 0.0, cg_z)
        v_fix = rho * g * sphere.mean_volume(cg_z)
        for model, slope_ref in expected.items():
            res = []
            for a in amps:
                wv = IncidentWave.regular(a, 8.0, depth=200.0, g=g, rho=rho)
                nl = NonlinearFKForce(sphere, cg_fix, wv, model)
                ts = np.linspace(0.0, 8.0, 17)
                res.append(max(abs(nl(cg_fix, t)[2] - v_fix) for t in ts))
            slope = np.polyfit(np.log(amps), np.log(res), 1)[0]
            check(abs(slope - slope_ref) <= 0.1,
                  f"fixed sphere correction ~ a^{slope_ref:g} ({model}, cg_z={cg_z}, slope {slope:.3f})")

    # 5. Giorgi & Ringwood long-wave closed form vs quadrature (sphere, several heaves).
    wv = IncidentWave.regular(1.0, 7.0, g=g, rho=rho)
    for zc in (-1.0, 0.0, 1.5):
        for t in (0.0, 2.0, 5.0):
            eta0 = float(wv.eta(0.0, 0.0, t))
            cg = (0.0, 0.0, zc - 2.0)
            fq = pressure_force(sphere, cg, wv, t, "gr_longwave")[2]
            fa = gr_sphere_longwave_heave(5.0, zc, eta0, wv, t)
            check(abs(fq - fa) <= 1e-8 * abs(fa), f"G&R long-wave sphere closed form (zc={zc}, t={t})")

    # 6. Long waves: Airy+Wheeler and long-wave pressure agree as k -> 0, with an
    # error of first order in k (the depth decay e^{kz} over the draft).
    errs, ks = [], []
    for period in (60.0, 120.0):
        wv = IncidentWave.regular(1.0, period, g=g, rho=rho)
        a1, b1 = linear_fk_coefficients(sphere, cg_eq, wv, "airy_wheeler")
        a2, b2 = linear_fk_coefficients(sphere, cg_eq, wv, "hydrostatic")
        errs.append(abs(abs(a1[0, 2] + 1j * b1[0, 2]) / abs(a2[0, 2] + 1j * b2[0, 2]) - 1.0))
        ks.append(wv.k[0])
    order = np.log(errs[0] / errs[1]) / np.log(ks[0] / ks[1])
    check(errs[1] <= 0.01 and abs(order - 1.0) <= 0.1,
          f"Airy vs long-wave linear FK: {100 * errs[1]:.2f}% at T = 120 s, order in k {order:.3f}")

    # 7. Quadrature convergence: halving resolution changes the NL FK force by < 1e-6.
    wv = IncidentWave.regular(2.0, 8.0, depth=200.0, g=g, rho=rho)
    coarse = Sphere(5.0, (0.0, 0.0, 2.0), n_theta=24, n_phi=48)
    f_fine = pressure_force(sphere, (0.0, 0.0, -1.0), wv, 3.0, "airy_wheeler")
    f_coarse = pressure_force(coarse, (0.0, 0.0, -1.0), wv, 3.0, "airy_wheeler")
    check(np.max(np.abs(f_fine - f_coarse)) <= 1e-6 * v0, "quadrature converged (48x96 vs 24x48)")

    print(f"\nself-test: {'PASS' if ok else 'FAIL'}")
    return ok


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------

IEA_SPHERE = dict(radius=5.0, center=(0.0, 0.0, 2.0), mass=261.8e3)
HYDRO_MODELS = ("linear", "nl_hydrostatics", "nlfk_airy_wheeler", "nlfk_hydrostatic")


def _write_csv(path, t, cols: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["time_s"] + [f"heave_{k}_m" for k in cols])
        for i in range(len(t)):
            wr.writerow([f"{t[i]:.4f}"] + [f"{cols[k][i]:.9f}" for k in cols])


def _cmd_regular(args):
    bem = load_bem_heave(args.h5)
    body = Sphere(IEA_SPHERE["radius"], IEA_SPHERE["center"])
    w = 2.0 * np.pi / args.period
    print(f"IEA sphere, regular waves T = {args.period:g} s (w = {w:.6f} rad/s), "
          f"radiation = {args.radiation}, PTO {args.pto_damping:g} N s/m, ramp {args.ramp:g} s, {args.t_end:g} s")
    print("| a (m) | " + " | ".join(HYDRO_MODELS) + " |")
    print("|---|" + "---|" * len(HYDRO_MODELS))
    for a in args.amplitudes:
        wave = IncidentWave.regular(a, args.period, bem["depth"], bem["g"], bem["rho"], args.ramp)
        row, cols = [], {}
        for m in HYDRO_MODELS:
            t, z = simulate_heave(body, wave, bem, m, mass=IEA_SPHERE["mass"], t_end=args.t_end,
                                  dt=args.dt, radiation=args.radiation, pto_damping=args.pto_damping)
            amp, ph, mean = fit_harmonic(t, z, w)
            row.append(f"{amp:.4f} ({np.degrees(ph):+.1f} deg)")
            cols[m] = z
        print(f"| {a:g} | " + " | ".join(row) + " |")
        if args.out:
            _write_csv(Path(args.out) / f"regular_a{a:g}.csv", t, cols)


def _cmd_runaway(args):
    bem = load_bem_heave(args.h5)
    body = Sphere(IEA_SPHERE["radius"], IEA_SPHERE["center"])
    wave = IncidentWave.regular(0.5 * args.height, args.period, bem["depth"], bem["g"], bem["rho"], args.ramp)
    print(f"IEA sphere, H = {args.height:g} m, T = {args.period:g} s, RIRF radiation, {args.t_end:g} s")
    cols = {}
    for m in HYDRO_MODELS:
        try:
            t, z = simulate_heave(body, wave, bem, m, mass=IEA_SPHERE["mass"], t_end=args.t_end,
                                  dt=args.dt, radiation="rirf")
        except ValueError as e:
            print(f"  {m:20s} stopped: {e}")
            continue
        cols[m] = z
        print(f"  {m:20s} heave min {z.min():+8.2f} m, max {z.max():+8.2f} m")
    if args.out and cols:
        _write_csv(Path(args.out) / "runaway.csv", t, cols)


def _cmd_gr_compare(args):
    """RMS gap between the Giorgi & Ringwood assumptions and ours (doc section 9, C3)."""
    body = Sphere(IEA_SPHERE["radius"], IEA_SPHERE["center"])
    cg_z0 = -2.0
    print(f"IEA sphere, prescribed heave z = {cg_z0} + {args.heave:g} sin(0.37 w t), deep water")
    print("RMS heave-force difference vs airy_wheeler, % of its range")
    print("| T (s) | kR | a (m) | gr | gr_longwave | hydrostatic |")
    print("|---|---|---|---|---|---|")
    for period in args.periods:
        for a in args.amplitudes:
            wave = IncidentWave.regular(a, period)
            ts = np.linspace(0.0, 2.0 * period, 81)
            w = 2.0 * np.pi / period
            force = {m: np.array([pressure_force(body, (0.0, 0.0, cg_z0 + args.heave * np.sin(0.37 * w * t)),
                                                 wave, t, m)[2] for t in ts])
                     for m in ("airy_wheeler", "gr", "gr_longwave", "hydrostatic")}
            ref = force["airy_wheeler"]
            rng = ref.max() - ref.min()
            gaps = [100.0 * np.sqrt(np.mean((force[m] - ref) ** 2)) / rng
                    for m in ("gr", "gr_longwave", "hydrostatic")]
            print(f"| {period:g} | {wave.k[0] * body.radius:.3f} | {a:g} | "
                  + " | ".join(f"{g_:.2f}%" for g_ in gaps) + " |")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("selftest")
    r = sub.add_parser("regular")
    r.add_argument("--h5", required=True)
    r.add_argument("--period", type=float, default=2.0 * np.pi / 0.8)
    r.add_argument("--amplitudes", type=float, nargs="+", default=[0.1, 1.0, 2.5])
    r.add_argument("--radiation", choices=["constant", "rirf"], default="constant")
    r.add_argument("--pto-damping", type=float, default=0.0, help="linear heave PTO damping [N s/m]")
    r.add_argument("--ramp", type=float, default=20.0)
    r.add_argument("--t-end", type=float, default=140.0)
    r.add_argument("--dt", type=float, default=0.01)
    r.add_argument("--out")
    q = sub.add_parser("runaway")
    q.add_argument("--h5", required=True)
    q.add_argument("--height", type=float, default=10.0)
    q.add_argument("--period", type=float, default=8.0)
    q.add_argument("--ramp", type=float, default=20.0)
    q.add_argument("--t-end", type=float, default=60.0)
    q.add_argument("--dt", type=float, default=0.01)
    q.add_argument("--out")
    c = sub.add_parser("gr-compare")
    c.add_argument("--periods", type=float, nargs="+", default=[8.0, 12.0, 20.0, 30.0])
    c.add_argument("--amplitudes", type=float, nargs="+", default=[0.5, 1.5])
    c.add_argument("--heave", type=float, default=1.5)
    args = p.parse_args(argv)
    if args.cmd == "selftest":
        return 0 if self_test() else 1
    if args.cmd == "regular":
        _cmd_regular(args)
    elif args.cmd == "runaway":
        _cmd_runaway(args)
    else:
        _cmd_gr_compare(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
