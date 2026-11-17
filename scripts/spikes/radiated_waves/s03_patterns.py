"""S3: assumed directional patterns vs Haskind far-field patterns.

For bodies with excitation over the full circle of headings, the Haskind
relation gives the BEM-exact far-field radiation pattern of each DOF
(including interaction with other bodies of the array):

    A_j(theta) = -e^{i pi/4} k / (sqrt(8 pi) rho g c_g) X_j(theta + pi)

referenced to the body point by e^{-i k p . e_theta}.  We compare it with the
energy-matched assumed pattern alpha_j f_j(theta), where |alpha_j| comes from
B_jj and f_j is the assumed harmonic (1, cos, sin, sin 2theta).

Metrics per DOF and frequency (only where the DOF radiates noticeably):
  coherence^2  = |<A, f>|^2 / (||A||^2 ||f||^2): fraction of the radiated
                 energy that lies in the assumed angular pattern;
  directional error = ||A - alpha f||^2 / ||A||^2 with the best phase
                 = 2 (1 - coherence) for an energy-matched pattern;
  phase error  = arg<A, f> - arg(model phase), model = real multipole strength.
"""
import json

import matplotlib.pyplot as plt
import numpy as np

import rwlib as R

# (case, body index, how to complete the circle)
CASES = [
    ("sphere", 0, "axisym"),
    ("rm3", 0, "axisym"),
    ("oswec", 0, "mirror"),
    ("crane", 0, "full"),
    ("wigley", 0, "full"),
    ("5sa_dir", 0, "full"),
    ("5sa_dir", 2, "full"),
]
KL_SHOW = [0.3, 1.0, 3.0]          # k * V^(1/3) for the polar plots
SIG = 0.02                          # DOF radiates if B_jj(w) > SIG * max_w B_jj


def circle(h, how):
    if how == "axisym":
        return R.axisym_circle(h, 72)
    if how == "mirror":
        h2 = R.Hydro(**{**h.__dict__, "X": R.x_wecsim(h)})
        return R.full_circle(h2, symmetric_xz=True, symmetric_yz=True)
    h2 = R.Hydro(**{**h.__dict__, "X": R.x_wecsim(h)})
    return R.full_circle(h2)


def damping(h):
    return h.B if h.B is not None else R.b_from_k_fit(h.t, h.K, h.w)


def analyse(key, body, how):
    h = R.load(key)
    dirs, X = circle(h, how)
    theta, A = R.haskind_far_field(h, dirs, X, body=body)
    order = np.argsort(theta)
    theta, A = theta[order], A[:, order, :]
    wq = R.periodic_weights(theta)
    B = damping(h)
    k = R.wavenumber(h.w, h.g, h.depth)
    L = float(np.cbrt(h.extra["disp_vol"][body]))
    out = {"theta": theta, "wq": wq, "w": h.w, "kL": k * L, "L": L, "dofs": {}, "h": h}
    for j in range(6):
        jj = 6 * body + j
        m, fun, label = R.PATTERNS[j]
        Bjj = B[:, jj, jj]
        active = Bjj > SIG * max(Bjj.max(), 1e-30)
        if not active.any():
            continue
        f = fun(theta)
        Aj = A[:, :, jj]
        nA2 = np.sum(wq * np.abs(Aj) ** 2, axis=1)
        nf2 = np.sum(wq * f ** 2)
        proj = np.sum(wq * Aj * f, axis=1)
        coh = np.abs(proj) / np.sqrt(np.maximum(nA2 * nf2, 1e-300))
        alpha = R.energy_matched_amplitude(h, Bjj, m)
        # energy consistency: Haskind energy vs B_jj
        e_ratio = nA2 / np.maximum((k * Bjj) / (h.rho * h.g * R.group_velocity(h.w, h.g, h.depth)), 1e-300)
        # sign of the real multipole from the lowest active frequencies
        i_low = np.flatnonzero(active)[: max(3, active.sum() // 10)]
        ref = R.model_phase(m, 1.0)
        sign = float(np.sign(np.real(np.mean(proj[i_low] / ref))))
        phase_err = np.angle(proj / (ref * sign))
        # angular-harmonic energy content
        mm = np.arange(0, 7)
        harm = np.array([np.abs(np.sum(wq * Aj * np.exp(-1j * q * theta), axis=1)) ** 2 +
                         (np.abs(np.sum(wq * Aj * np.exp(1j * q * theta), axis=1)) ** 2 if q else 0)
                         for q in mm]).T / (2 * np.pi * np.maximum(nA2, 1e-300))[:, None]
        out["dofs"][j] = dict(m=m, f=f, label=label, A=Aj, alpha=alpha, coh=coh, active=active,
                              e_ratio=e_ratio, sign=sign, phase_err=phase_err, harm=harm, Bjj=Bjj)
    return out


def main():
    results = [(key, body, analyse(key, body, how)) for key, body, how in CASES]
    summary = {}

    # ---- polar plots --------------------------------------------------------------
    dofs_show = [0, 2, 4, 1, 3, 5]
    fig = plt.figure(figsize=(18, 3.1 * len(results)))
    for r, (key, body, res) in enumerate(results):
        for c, j in enumerate(dofs_show):
            ax = fig.add_subplot(len(results), len(dofs_show), r * len(dofs_show) + c + 1, projection="polar")
            ax.set_xticklabels([]); ax.set_yticklabels([])
            if c == 0:
                ax.set_ylabel(f"{key} b{body + 1}\nL={res['L']:.1f} m", labelpad=22)
            if r == 0:
                ax.set_title(R.DOF[j], pad=12)
            if j not in res["dofs"]:
                ax.text(0, 0, "no radiation", ha="center", fontsize=8)
                continue
            d = res["dofs"][j]
            th = np.append(res["theta"], res["theta"][0] + 2 * np.pi)
            for q, kl in enumerate(KL_SHOW):
                i = int(np.argmin(np.abs(res["kL"] - kl)))
                if not d["active"][i]:
                    continue
                Ah = np.abs(d["A"][i]); Am = d["alpha"][i] * np.abs(d["f"])
                s = max(Ah.max(), Am.max(), 1e-30)
                ax.plot(th, np.append(Ah, Ah[0]) / s * (q + 1), f"C{q}-", lw=1.4)
                ax.plot(th, np.append(Am, Am[0]) / s * (q + 1), f"C{q}--", lw=1.0)
            ax.set_ylim(0, len(KL_SHOW) + 0.2)
    fig.suptitle("S3  |far-field pattern|: Haskind/BEM (solid) vs energy-matched assumed pattern (dashed);  "
                 "rings: kL = 0.3 (blue), 1 (orange), 3 (green), L = V^(1/3); theta = 0 is +x (bow), CCW", y=1.0)
    fig.tight_layout()
    R.savefig(fig, "s03_patterns_polar.png")

    # ---- coherence and phase vs kL -----------------------------------------------------
    fig, axes = plt.subplots(2, 6, figsize=(20, 7), sharex=True)
    for key, body, res in results:
        lab = f"{key} b{body + 1}"
        summ = {}
        for j, d in res["dofs"].items():
            a = d["active"]
            kl = np.where(a, res["kL"], np.nan)
            axes[0, j].plot(kl, d["coh"] ** 2, lw=1.2, label=lab)
            axes[1, j].plot(kl, np.degrees(d["phase_err"]), lw=1.0, label=lab)
            # kL where coherence^2 first drops below 0.8 (sustained)
            bad = a & (d["coh"] ** 2 < 0.8)
            first_bad = float(res["kL"][np.flatnonzero(bad)[0]]) if bad.any() else None
            wB = d["Bjj"] * a
            wmean = lambda v: float(np.sum(v * wB) / max(np.sum(wB), 1e-300))
            summ[R.DOF[j]] = {
                "multipole_sign_low_freq": d["sign"],
                "B_weighted_coherence2": wmean(d["coh"] ** 2),
                "min_coherence2": float(np.nanmin(np.where(a, d["coh"] ** 2, np.nan))),
                "first_kL_coherence2_below_0.8": first_bad,
                "B_weighted_abs_phase_err_deg": wmean(np.abs(np.degrees(d["phase_err"]))),
                "haskind_energy_over_Bjj_median": float(np.median(d["e_ratio"][a])),
                "B_weighted_harmonic_fraction_m0..6": [round(wmean(d["harm"][:, q]), 3) for q in range(7)],
            }
        summary[lab] = summ
        print(lab, json.dumps(summ)[:900])
    for j in range(6):
        axes[0, j].set_title(f"{R.DOF[j]}  (assumed {R.PATTERNS[j][2]})")
        axes[0, j].axhline(0.8, color="k", ls=":", lw=0.8)
        axes[0, j].set_ylim(0, 1.02)
        axes[1, j].set_ylim(-180, 180)
        axes[1, j].axhline(0, color="k", lw=0.5)
        axes[1, j].set_xlabel("k L   (L = V^(1/3))")
        axes[1, j].set_xscale("log")
        axes[0, j].grid(alpha=0.3); axes[1, j].grid(alpha=0.3)
    axes[0, 0].set_ylabel("coherence^2\n(energy fraction in assumed pattern)")
    axes[1, 0].set_ylabel("phase error vs real multipole [deg]")
    axes[0, 0].legend(fontsize=7)
    fig.suptitle("S3  Where do the assumed patterns hold?  (only frequencies where the DOF radiates, B_jj > 2% of peak)", y=1.0)
    fig.tight_layout()
    R.savefig(fig, "s03_pattern_fidelity.png")
    (R.OUT / "s03_summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
