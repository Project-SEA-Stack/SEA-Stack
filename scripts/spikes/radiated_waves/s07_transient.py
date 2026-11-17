"""S7: free-decay transient, dispersive propagation and the energy balance.

Sphere (R = 5 m, 200 m depth), heave decay from 1 m.  The body motion is
integrated with the same Cummins equation SEA-Stack solves:

    (m + A_inf) x'' + int_0^t K(tau) x'(t - tau) dtau + C x = 0.

The radiated field is the linear filter of the actual velocity history,
eta(r, t) = h_r * x', with h_r from the energy-matched Approximate source
(constant multipole phase) and from Haskind (BEM phase), Hankel propagation.

Energy: the work done on the fluid by the memory term,
    W_rad(t) = int_0^t x'(t') int K x' dt',
is compared with the energy that has crossed circles of radius r,
    E_r(t) = int_0^t F(r, t') dt',
with the exact time-domain flux F = int int p u_r dz r dtheta built from the
same frequency components (finite-depth vertical structure).  The two agree
only after the waves have passed: there are propagation delays and energy
stored in the near field.
"""
import json

import h5py
import matplotlib.pyplot as plt
import numpy as np
from scipy.special import hankel2, h2vp

import rwlib as R
from s05_causality import taper

DT = 0.05
N = 2 ** 15
T_SIM = 200.0
X0 = 1.0
RADII_FLUX = [20.0, 60.0, 150.0, 300.0]


def simulate(h):
    with h5py.File(R.FILES["sphere"], "r") as f:
        A_inf = float(f["body1/hydro_coeffs/added_mass/inf_freq"][()][2, 2]) * h.rho
        C33 = float(f["body1/hydro_coeffs/linear_restoring_stiffness"][()][2, 2]) * h.rho * h.g
    mass = h.rho * h.extra["disp_vol"][0]
    tk = np.arange(0, h.t[-1] + 1e-9, DT)
    K = np.interp(tk, h.t, h.K[:, 2, 2])
    n = int(T_SIM / DT)
    x = np.zeros(n); v = np.zeros(n); f_mem = np.zeros(n)
    x[0] = X0
    for i in range(n - 1):
        m_ = min(i + 1, len(K))
        conv = np.sum(K[:m_] * v[i::-1][:m_]) * DT - 0.5 * K[0] * v[i] * DT
        f_mem[i] = conv
        a = -(conv + C33 * x[i]) / (mass + A_inf)
        v[i + 1] = v[i] + a * DT
        x[i + 1] = x[i] + v[i + 1] * DT
    f_mem[-1] = f_mem[-2]
    t = np.arange(n) * DT
    E_body = 0.5 * (mass + A_inf) * v ** 2 + 0.5 * C33 * x ** 2
    W_rad = np.cumsum(v * f_mem) * DT
    return t, x, v, E_body, W_rad, dict(mass=mass, A_inf=A_inf, C33=C33)


def transfer(h, variant, wb):
    from s05_causality import setup
    s = setup("sphere", 0, 2, 0.0, "axisym")
    assert np.allclose(s["w"], wb)
    if variant == "haskind":
        return s["Ah"]
    return s["mag"] * R.model_phase(0, -1.0)


def vertical_overlap(k, h_depth):
    """I_nm = int_{-h}^0 Z_n Z_m dz, Z = cosh k(z+h)/cosh kh (deep-water limit for kh > 15)."""
    kn, km = k[:, None], k[None, :]
    deep = 1.0 / (kn + km)
    if np.isinf(h_depth):
        return deep
    kh = k * h_depth
    small = (kh[:, None] < 15) & (kh[None, :] < 15)
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        S = lambda q: np.where(np.abs(q) < 1e-9, h_depth, np.sinh(q * h_depth) / np.where(q == 0, 1, q))
        exact = (S(kn + km) + S(kn - km)) / (2 * np.cosh(kn * h_depth) * np.cosh(km * h_depth))
    return np.where(small, exact, deep)


def main():
    h = R.load("sphere")
    t, x, v, E_body, W_rad, par = simulate(h)
    vpad = np.zeros(N); vpad[: len(v)] = v
    V = np.fft.rfft(vpad) * DT                       # continuous FT, e^{+iwt} synthesis
    wb = np.fft.rfftfreq(N, d=DT) * 2 * np.pi
    kb = R.wavenumber(np.maximum(wb, 1e-6), h.g, h.depth)
    cgb = R.group_velocity(np.maximum(wb, 1e-6), h.g, h.depth)
    band = (wb > 0.05) & (wb < 3.0)
    tw = np.arange(N) * DT
    out = {}
    radii = np.linspace(8, 400, 160)
    fig, axes = plt.subplots(2, 3, figsize=(19, 9))
    for col, variant in enumerate(["approximate", "haskind"]):
        Hs = transfer(h, variant, wb)
        st = np.zeros((len(radii), int(260 / DT)))
        for i, r in enumerate(radii):
            P = np.sqrt(np.pi / 2) * np.exp(-1j * np.pi / 4) * hankel2(0, np.maximum(kb * r, 1e-9)) * band
            eta = np.fft.irfft(Hs * P * V, n=N) / DT
            st[i] = eta[: st.shape[1]]
        ax = axes[0, col]
        vm = np.percentile(np.abs(st[radii > 30]), 99.5)
        ax.pcolormesh(np.arange(st.shape[1]) * DT, radii, st, cmap="RdBu_r", vmin=-vm, vmax=vm, shading="auto")
        for wr, ls in [(0.6, ":"), (1.2, "--"), (2.0, "-.")]:
            cgr = R.group_velocity(np.array([wr]), h.g, h.depth)[0]
            ax.plot(radii / cgr, radii, "k" + ls, lw=0.8, label=f"r = c_g t, w={wr}")
        ax.set_xlim(0, 260); ax.set_ylim(radii[0], radii[-1])
        ax.set_xlabel("t [s]"); ax.set_ylabel("r [m]")
        ax.set_title(f"eta(r, t), {variant} source; colour limit {vm * 100:.1f} cm", fontsize=10)
        if col == 0:
            ax.legend(fontsize=7, loc="lower right")
        out[variant + "_space_time"] = st

        # exact energy flux through circles
        flux_res = {}
        sel = np.flatnonzero(band)
        kk = kb[sel]
        I = vertical_overlap(kk, h.depth)
        nt = int(260 / DT)
        Et = np.exp(1j * np.outer(tw[:nt], wb[sel]))          # (nt, nf)
        for r in RADII_FLUX:
            Pm = np.sqrt(np.pi / 2) * np.exp(-1j * np.pi / 4)
            eta_c = Hs[sel] * Pm * hankel2(0, kk * r) * V[sel]
            deta_c = Hs[sel] * Pm * kk * h2vp(0, kk * r) * V[sel]
            dw = wb[1] - wb[0]
            # eta(t) = Re sum c_n e^{i w_n t} with c_n = eta_c * dw / pi
            c_eta = eta_c * dw / np.pi
            c_deta = deta_c * dw / np.pi
            p_c = h.rho * h.g * c_eta                         # pressure at z = 0 (times Z_n(z))
            u_c = 1j * h.g / wb[sel] * c_deta                 # radial velocity at z = 0 (times Z_n(z))
            p_t = np.real(Et * p_c[None, :])
            u_t = np.real(Et * u_c[None, :])
            F = 2 * np.pi * r * np.sum((p_t @ I) * u_t, axis=1)
            flux_res[r] = F
        out[variant + "_flux"] = flux_res

    tt = np.arange(int(260 / DT)) * DT
    ax = axes[0, 2]
    ax.plot(t, x, "k-", lw=1)
    ax.set_xlabel("t [s]"); ax.set_ylabel("heave [m]")
    ax.set_title(f"sphere heave decay from {X0} m (Cummins, K from file)", fontsize=10)
    ax.grid(alpha=0.3)
    ax = axes[1, 0]
    E0 = E_body[0]
    ax.plot(t, (E0 - E_body) / E0, "k-", lw=2, label="body energy lost")
    ax.plot(t, W_rad / E0, "y--", lw=1.5, label="work by radiation memory force")
    for r, c in zip(RADII_FLUX, ["C0", "C1", "C2", "C3"]):
        F = out["approximate_flux"][r]
        ax.plot(tt, np.cumsum(F) * DT / E0, c + "-", lw=1.2, label=f"energy through r = {r:.0f} m (approx)")
        Fh = out["haskind_flux"][r]
        ax.plot(tt, np.cumsum(Fh) * DT / E0, c + ":", lw=1.2)
    ax.set_xlim(0, 260); ax.set_ylim(-0.05, 1.15)
    ax.set_xlabel("t [s]"); ax.set_ylabel("energy / initial body energy")
    ax.set_title("cumulative energy: body vs wave flux (solid approx, dotted Haskind)", fontsize=10)
    ax.legend(fontsize=7); ax.grid(alpha=0.3)
    ax = axes[1, 1]
    win = int(10 / DT)
    P_rad = np.convolve(np.gradient(W_rad, DT), np.ones(win) / win, "same")
    ax.plot(t, P_rad / 1e3, "k-", lw=2, label="radiation power at body (10 s mean)")
    for r, c in zip(RADII_FLUX, ["C0", "C1", "C2", "C3"]):
        F = np.convolve(out["approximate_flux"][r], np.ones(win) / win, "same")
        ax.plot(tt, F / 1e3, c + "-", lw=1, label=f"flux at r = {r:.0f} m (10 s mean)")
    ax.set_xlim(0, 260); ax.set_xlabel("t [s]"); ax.set_ylabel("kW")
    ax.set_title("running-window power: body vs flux at radius r", fontsize=10)
    ax.legend(fontsize=7); ax.grid(alpha=0.3)
    ax = axes[1, 2]
    for r, c in zip([60.0, 150.0], ["C1", "C2"]):
        i = int(np.argmin(np.abs(radii - r)))
        ax.plot(tt, out["haskind_space_time"][i] * 100, c + "-", lw=1.4, label=f"Haskind r={r:.0f} m")
        ax.plot(tt, out["approximate_space_time"][i] * 100, c + "--", lw=1, label=f"approx r={r:.0f} m")
    ax.set_xlim(0, 200); ax.set_xlabel("t [s]"); ax.set_ylabel("eta [cm]")
    ax.set_title("wave elevation time series", fontsize=10)
    ax.legend(fontsize=7); ax.grid(alpha=0.3)
    fig.suptitle("S7  Sphere free heave decay: dispersive radiated waves and the energy balance", y=1.0)
    fig.tight_layout()
    R.savefig(fig, "s07_transient.png")

    summary = {"params": par, "E0_J": float(E0), "W_rad_end_over_E0": float(W_rad[-1] / E0),
               "body_energy_lost_end_over_E0": float((E0 - E_body[-1]) / E0)}
    for variant in ["approximate", "haskind"]:
        for r in RADII_FLUX:
            F = out[variant + "_flux"][r]
            E = np.cumsum(F) * DT
            summary[f"{variant}_E_through_{r:.0f}m_end_over_W_rad_end"] = float(E[-1] / W_rad[-1])
            summary[f"{variant}_E_through_{r:.0f}m_at_t=30s_over_W_rad_30s"] = float(E[int(30 / DT)] / W_rad[int(30 / DT)])
    st_a, st_h = out["approximate_space_time"], out["haskind_space_time"]
    m = radii > 30
    summary["space_time_correlation_approx_vs_haskind_r>30m"] = float(np.sum(st_a[m] * st_h[m]) / np.sqrt(np.sum(st_a[m] ** 2) * np.sum(st_h[m] ** 2)))
    summary["peak_eta_cm_at_60m"] = {v_: float(np.abs(out[v_ + "_space_time"][int(np.argmin(np.abs(radii - 60)))]).max() * 100) for v_ in ["approximate", "haskind"]}
    print(json.dumps(summary, indent=1))
    (R.OUT / "s07_summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
