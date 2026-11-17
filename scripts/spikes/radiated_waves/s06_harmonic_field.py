"""S6: harmonic radiated wave field and its energy flux.

Crane vessel (single body, 40 m depth, 24 headings) in combined heave + pitch
at one frequency.  Two fields, both built as sums of outgoing cylindrical
modes H_|m|^(2)(k r) e^{i m theta} so they are exact solutions of the linear
free-surface problem outside the body's circumscribing circle:

  Approximate: energy-matched assumed patterns (m = 0 for heave, cos theta for
               pitch, relative phase inside the surge/pitch group from B15)
               with constant real-multipole phase;
  Haskind:     BEM far-field pattern from the excitation, expanded to |m| <= M.

Energy flux through circles of radius r, exact for the cylindrical modes:
    F(r) = (rho g c_g / 2k) r int Re{-i eta (d eta/dr)^*} dtheta
compared with the radiated power 1/2 U^H B U from the stored damping.
"""
import json

import matplotlib.pyplot as plt
import numpy as np

import rwlib as R
from s03_patterns import circle

KEY = "crane"
OMEGA = 0.9            # rad/s
HEAVE_AMP = 0.5        # m
PITCH_AMP = np.radians(2.0)
PITCH_PHASE = np.pi / 2   # pitch leads heave


def main():
    h = R.load(KEY)
    iw = int(np.argmin(np.abs(h.w - OMEGA)))
    w = h.w[iw]
    k = R.wavenumber(np.array([w]), h.g, h.depth)[0]
    cg = R.group_velocity(np.array([w]), h.g, h.depth)[0]
    lam = 2 * np.pi / k
    U = np.zeros(6, complex)
    U[2] = 1j * w * HEAVE_AMP
    U[4] = 1j * w * PITCH_AMP * np.exp(1j * PITCH_PHASE)
    B = h.B[iw]
    P_true = 0.5 * np.real(np.conj(U) @ B @ U)

    # --- Approximate: harmonic coefficients a_m per unit velocity -----------------------
    scale0 = np.sqrt(k / (h.rho * h.g * cg * R.pattern_norm(0)))
    scale1 = np.sqrt(k / (h.rho * h.g * cg * R.pattern_norm(1)))
    a_heave = -scale0 * np.sqrt(B[2, 2]) * np.exp(1j * np.pi / 4)               # m = 0
    # surge/pitch group, relative phase from B15 (surge is the lead)
    c15 = np.clip(B[0, 4] / np.sqrt(B[0, 0] * B[4, 4]), -1, 1)
    a_pitch_cos = scale1 * np.sqrt(B[4, 4]) * np.exp(1j * np.arccos(c15)) * np.exp(1j * 3 * np.pi / 4)
    ms_a = np.array([-1, 0, 1])
    coef_a = np.array([0.5 * a_pitch_cos * U[4], a_heave * U[2], 0.5 * a_pitch_cos * U[4]])

    # --- Haskind: expand the BEM pattern ----------------------------------------------
    dirs, X = circle(h, "full")
    theta, A = R.haskind_far_field(h, dirs, X, body=0)
    order = np.argsort(theta); theta, A = theta[order], A[:, order, :]
    wq = R.periodic_weights(theta)
    M = len(theta) // 2 - 1
    Atot = A[iw] @ U                                       # pattern for this motion
    ms_h, coef_h = R.harmonics_from_pattern(theta, wq, Atot, M)

    # --- maps ----------------------------------------------------------------------------
    L = 4 * lam
    xs = np.linspace(-L, L, 401)
    Xg, Yg = np.meshgrid(xs, xs)
    r_in = 25.0                                            # ~ half hull length, inside: masked
    fields = {}
    for name, (ms, cf) in {"Approximate": (ms_a, coef_a), "Haskind (BEM pattern)": (ms_h, coef_h)}.items():
        eta, _ = R.outgoing_field(ms, cf, k, Xg, Yg)
        eta[np.hypot(Xg, Yg) < r_in] = np.nan
        fields[name] = eta

    # --- flux vs radius ------------------------------------------------------------------
    radii = np.linspace(5, 6 * lam, 120)
    th = np.linspace(0, 2 * np.pi, 720, endpoint=False)
    flux = {}
    for name, (ms, cf) in {"Approximate": (ms_a, coef_a), "Haskind (BEM pattern)": (ms_h, coef_h)}.items():
        ex, ff = [], []
        for r in radii:
            x, y = r * np.cos(th), r * np.sin(th)
            eta, deta = R.outgoing_field(ms, cf, k, x, y)
            ex.append(h.rho * h.g * cg / (2 * k) * r * np.mean(np.real(-1j * eta * np.conj(deta))) * 2 * np.pi)
            ff.append(0.5 * h.rho * h.g * cg * r * np.mean(np.abs(eta) ** 2) * 2 * np.pi)
        flux[name] = (np.array(ex), np.array(ff))

    fig = plt.figure(figsize=(18, 5.6))
    vmax = np.nanmax(np.abs(fields["Approximate"].real[np.hypot(Xg, Yg) > 1.5 * lam]))
    for i, (name, eta) in enumerate(fields.items()):
        ax = fig.add_subplot(1, 3, i + 1)
        im = ax.pcolormesh(xs, xs, eta.real, cmap="RdBu_r", vmin=-vmax, vmax=vmax, shading="auto")
        ax.set_aspect("equal")
        ax.set_title(f"{name}\nRe eta at t=0, w={w:.2f} rad/s, lambda={lam:.0f} m", fontsize=10)
        ax.set_xlabel("x [m]  (bow +x)"); ax.set_ylabel("y [m]")
        plt.colorbar(im, ax=ax, label="eta [m]")
    ax = fig.add_subplot(1, 3, 3)
    for (name, (ex, ff)), c in zip(flux.items(), ["C0", "C3"]):
        ax.plot(radii / lam, ex / P_true, c + "-", label=f"{name}: exact modal flux")
        ax.plot(radii / lam, ff / P_true, c + "--", lw=1, label=f"{name}: far-field 1/2 rho g c_g |eta|^2")
    ax.axhline(1, color="k", lw=0.8)
    ax.set_ylim(0, 2)
    ax.set_xlabel("r / lambda"); ax.set_ylabel("flux / (1/2 U^H B U)")
    ax.set_title(f"Energy flux through circle r, P = {P_true / 1e3:.1f} kW", fontsize=10)
    ax.legend(fontsize=7); ax.grid(alpha=0.3)
    fig.suptitle(f"S6  {KEY}: heave {HEAVE_AMP} m + pitch {np.degrees(PITCH_AMP):.0f} deg (pitch leads 90 deg); "
                 "masked inside r = 25 m (far-field forms not valid inside the body)", y=1.02)
    fig.tight_layout()
    R.savefig(fig, "s06_harmonic_field.png")
    summary = {
        "omega": float(w), "wavelength_m": float(lam), "P_true_W": float(P_true),
        "flux_ratio_exact_approx": [float(np.min(flux["Approximate"][0] / P_true)), float(np.max(flux["Approximate"][0] / P_true))],
        "flux_ratio_exact_haskind": [float(np.min(flux["Haskind (BEM pattern)"][0] / P_true)), float(np.max(flux["Haskind (BEM pattern)"][0] / P_true))],
        "farfield_proxy_at_r_eq_lambda_approx": float(np.interp(1.0, radii / lam, flux["Approximate"][1] / P_true)),
        "max_abs_eta_at_2lambda_m": {n: float(np.nanmax(np.abs(e[np.abs(np.hypot(Xg, Yg) - 2 * lam) < 2]))) for n, e in fields.items()},
        "haskind_harmonics_M": int(M),
    }
    print(json.dumps(summary, indent=1))
    (R.OUT / "s06_summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
