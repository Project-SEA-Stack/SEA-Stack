"""S9: multi-body array -- superposed isolated patterns vs BEM (Haskind) array field.

5SA directional file (5 segments in a row along x, 12 headings): all segments
heave in phase with 0.5 m amplitude at one frequency.

  Approximate: each segment radiates its own energy-matched m=0 pattern from
               its own centre (self damping B_bb only), fields superposed;
  Haskind:     each segment's in-array far-field pattern from the array
               excitation (includes shadowing and interaction), expanded
               about its own centre (|m| <= 5) and superposed.

Flux through a large circle is compared with 1/2 U^T B U from the full
coupled damping matrix (K fit on the BEM grid).
"""
import json

import matplotlib.pyplot as plt
import numpy as np

import rwlib as R
from s03_patterns import circle

KEY = "5sa_dir"
OMEGAS = [0.56, 1.07]
AMP = 0.5


def main():
    h = R.load(KEY)
    B = R.b_from_k_fit(h.t, h.K, h.w)
    dirs, X = circle(h, "full")
    summary = {}
    fig, axes = plt.subplots(len(OMEGAS), 3, figsize=(19, 5.4 * len(OMEGAS)))
    for row, om in enumerate(OMEGAS):
        iw = int(np.argmin(np.abs(h.w - om)))
        w = h.w[iw]
        k = R.wavenumber(np.array([w]), h.g, h.depth)[0]
        cg = R.group_velocity(np.array([w]), h.g, h.depth)[0]
        lam = 2 * np.pi / k
        U = np.zeros(6 * h.nb, complex)
        U[2::6] = 1j * w * AMP
        P_true = 0.5 * np.real(np.conj(U) @ B[iw] @ U)
        xs = np.linspace(-150, 300, 451)
        ys = np.linspace(-225, 225, 451)
        Xg, Yg = np.meshgrid(xs, ys)
        th = np.linspace(0, 2 * np.pi, 720, endpoint=False)
        R_far = 2000.0
        xc = np.mean(h.cg[:, 0])
        fields, fluxes = {}, {}
        for name in ("Approximate (superposed isolated sources)", "Haskind (BEM array patterns)"):
            eta = np.zeros_like(Xg, dtype=complex)
            ef = np.zeros_like(th, dtype=complex); df = np.zeros_like(th, dtype=complex)
            for b in range(h.nb):
                p = h.cg[b, :2]
                if name.startswith("Approx"):
                    jj = 6 * b + 2
                    a0 = -np.sqrt(k * B[iw, jj, jj] / (h.rho * h.g * cg * 2 * np.pi)) * np.exp(1j * np.pi / 4)
                    ms, cf = np.array([0]), np.array([a0 * U[jj]])
                else:
                    theta, A = R.haskind_far_field(h, dirs, X, body=b)
                    o = np.argsort(theta)
                    Ab = A[iw][o][:, R.body_slice(b)] @ U[R.body_slice(b)]
                    ms, cf = R.harmonics_from_pattern(theta[o], R.periodic_weights(theta[o]), Ab, 5)
                e, _ = R.outgoing_field(ms, cf, k, Xg - p[0], Yg - p[1])
                eta += e
                e2, d2 = R.outgoing_field(ms, cf, k, xc + R_far * np.cos(th) - p[0], R_far * np.sin(th) - p[1])
                ef += e2; df += d2
            near = np.zeros_like(Xg, bool)
            for b in range(h.nb):
                near |= np.hypot(Xg - h.cg[b, 0], Yg - h.cg[b, 1]) < 16
            eta[near] = np.nan
            fields[name] = eta
            # far circle: |eta|^2 far-field flux (r >> array size)
            fluxes[name] = 0.5 * h.rho * h.g * cg * R_far * np.mean(np.abs(ef) ** 2) * 2 * np.pi
        vmax = np.nanpercentile(np.abs(fields["Haskind (BEM array patterns)"].real), 99)
        for c, (name, eta) in enumerate(fields.items()):
            ax = axes[row, c]
            im = ax.pcolormesh(xs, ys, eta.real, cmap="RdBu_r", vmin=-vmax, vmax=vmax, shading="auto")
            ax.plot(h.cg[:, 0], h.cg[:, 1], "k|", ms=14)
            ax.set_aspect("equal")
            ax.set_title(f"{name}\nw={w:.2f} rad/s, lambda={lam:.0f} m; flux/(1/2 U^T B U) = {fluxes[name] / P_true:.2f}", fontsize=9)
            plt.colorbar(im, ax=ax, label="eta [m]")
        ax = axes[row, 2]
        for name, eta, st in [("approx", fields["Approximate (superposed isolated sources)"], "C0-"),
                              ("haskind", fields["Haskind (BEM array patterns)"], "k-")]:
            j = int(np.argmin(np.abs(ys - 0)))
            ax.plot(xs, np.abs(eta[j]), st, label=f"|eta| along y=0, {name}")
            i = int(np.argmin(np.abs(xs - xc)))
            ax.plot(ys, np.abs(eta[:, i]), st.replace("-", "--"), label=f"|eta| along x={xc:.0f} (broadside), {name}")
        ax.set_xlabel("position [m]"); ax.set_ylabel("|eta| [m]"); ax.grid(alpha=0.3); ax.legend(fontsize=7)
        summary[f"w={w:.2f}"] = {"wavelength_m": float(lam), "P_true_kW": float(P_true / 1e3),
                                 **{n.split(" ")[0] + "_flux_ratio": float(f / P_true) for n, f in fluxes.items()}}
    fig.suptitle("S9  5SA: all segments heave in phase (0.5 m). Superposition of isolated sources vs BEM array field", y=1.0)
    fig.tight_layout()
    R.savefig(fig, "s09_array_field.png")
    print(json.dumps(summary, indent=1))
    (R.OUT / "s09_summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
