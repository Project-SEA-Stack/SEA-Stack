"""S5: phase and causality of time-domain radiated-wave filters.

B(w) fixes |A(w)| but not its phase.  For one DOF and one direction theta0 the
wave elevation at radius r is eta = h_r * u with

    H_r(w) = A(w, theta0) * P_m(k r),
    P_m = (k r)^(-1/2) e^{-i k r}                        (far-field asymptote), or
    P_m = sqrt(pi/2) e^{-i(pi/4 + m pi/2)} H_m^(2)(k r)   (outgoing cylindrical mode).

Candidate source responses A(w), all with |A| = energy-matched |alpha(w)|:
  'const'   : real multipole strength, A = s |alpha| e^{i(pi/4 + m pi/2)} (frequency-independent phase);
  'minphase': A = s e^{i(pi/4 + m pi/2)} * minimum-phase(|alpha|)  (causal source by construction);
  'haskind' : BEM phase from the excitation (Haskind), taken as the reference.

Physical causality requires h_r(tau) = 0 for tau < 0.  We report the fraction
of kernel energy at tau < 0, and the normalised correlation with the Haskind
kernel.  A scan of an extra constant phase applied to the Haskind kernel tests
the derived Haskind phase constant: the true constant should be the most causal.
"""
import json

import matplotlib.pyplot as plt
import numpy as np
from scipy.special import hankel2

import rwlib as R

DT = 0.05
N = 2 ** 15                      # 1638 s window, dw = 0.0038 rad/s
CASES = [("sphere", 0, 2, 0.0, "axisym"), ("crane", 0, 2, 0.0, "full"), ("crane", 0, 4, 0.0, "full"),
         ("crane", 0, 1, np.pi / 2, "full"), ("5sa_dir", 2, 2, np.pi / 2, "full")]
RADII = [10.0, 30.0, 100.0, 300.0]


def taper(w, w_hi):
    """Cosine taper from 0.8 w_hi to w_hi."""
    x = np.clip((w - 0.8 * w_hi) / (0.2 * w_hi), 0, 1)
    return 0.5 * (1 + np.cos(np.pi * x))


def min_phase(mag):
    """Minimum-phase spectrum (numpy FFT convention = e^{+i w t} synthesis) from |H| on rfft bins."""
    full = np.concatenate([mag, mag[-2:0:-1]])
    floor = 1e-8 * full.max()
    c = np.fft.ifft(np.log(np.maximum(full, floor))).real
    n = len(full)
    win = np.zeros(n); win[0] = 1; win[1:n // 2] = 2; win[n // 2] = 1
    return np.exp(np.fft.fft(c * win))[: len(mag)]


def kernel(H):
    return np.fft.irfft(H, n=N) / DT        # continuous-time kernel samples, tau = n*DT (wraps)


def acausal_fraction(g):
    tau = np.fft.fftfreq(N, d=1 / (N * DT))  # signed times, wrap-around
    neg = (tau < 0) & (tau > -N * DT / 4)
    return float(np.sum(g[neg] ** 2) / max(np.sum(g ** 2), 1e-300))


def setup(key, body, dof, theta0, how):
    h = R.load(key)
    from s03_patterns import circle, damping
    dirs, X = circle(h, how)
    theta, A = R.haskind_far_field(h, dirs, X, body=body)
    jj = 6 * body + dof
    i0 = int(np.argmin(np.abs(np.angle(np.exp(1j * (theta - theta0))))))
    m, fun, _ = R.PATTERNS[dof]
    B = damping(h)
    alpha = R.energy_matched_amplitude(h, B[:, jj, jj], m) * abs(fun(np.array([theta0]))[0])
    Ah = A[:, i0, jj]
    # Phase available from a single-heading file: Haskind towards theta = beta0 + pi with
    # beta0 = 0 (surge/heave/pitch) or 90 deg (sway/roll), used in every direction.
    beta0 = np.pi / 2 if dof in (1, 3) else 0.0
    i_ref = int(np.argmin(np.abs(np.angle(np.exp(1j * (theta - (beta0 + np.pi)))))))
    f_ref = fun(np.array([beta0 + np.pi]))[0]
    f0 = fun(np.array([theta0]))[0]
    A_ref = A[:, i_ref, jj] / np.where(abs(f_ref) > 1e-9, f_ref, 1.0) * f0
    sign = -1.0 if dof == 2 else 1.0
    if dof in (3, 4):
        # sign relative to the low-frequency Haskind projection (as S3)
        ref = R.model_phase(m, 1.0) * np.sign(fun(np.array([theta0]))[0])
        sign = float(np.sign(np.real(np.mean(Ah[:5] / ref))))
    sign *= np.sign(fun(np.array([theta0]))[0])
    w_bins = np.fft.rfftfreq(N, d=DT) * 2 * np.pi
    wmax = h.w[-1] if key != "5sa_dir" else 3.0
    tp = taper(w_bins, min(wmax, 3.0))
    inside = w_bins <= h.w[-1]
    mag = np.interp(w_bins, h.w, alpha, left=0, right=0) * tp
    Ah_i = (np.interp(w_bins, h.w, Ah.real, left=0, right=0) + 1j * np.interp(w_bins, h.w, Ah.imag, left=0, right=0)) * tp
    kb = R.wavenumber(np.maximum(w_bins, 1e-6), h.g, h.depth)
    ph_ref = np.unwrap(np.angle(A_ref))
    xphase = mag * np.exp(1j * np.interp(w_bins, h.w, ph_ref))
    return dict(h=h, m=m, sign=sign, w=w_bins, k=kb, mag=mag, Ah=Ah_i * inside, xphase=xphase * inside,
                label=f"{key} b{body + 1} {R.DOF[dof]} theta0={np.degrees(theta0):.0f}")


def prop(k, r, m, kind):
    x = np.maximum(k * r, 1e-9)
    if kind == "asym":
        P = x ** -0.5 * np.exp(-1j * x)
    else:
        P = np.sqrt(np.pi / 2) * np.exp(-1j * (np.pi / 4 + m * np.pi / 2)) * hankel2(m, x)
    P[k * r < 1e-6] = 0
    return P


def main():
    summary = {}
    fig, axes = plt.subplots(len(CASES), 3, figsize=(18, 3.2 * len(CASES)))
    for row, case in enumerate(CASES):
        s = setup(*case)
        m, w, k = s["m"], s["w"], s["k"]
        ph = R.model_phase(m, s["sign"])
        A_const = s["mag"] * ph
        A_min = min_phase(s["mag"]) * ph
        variants = {"const": A_const, "minphase": A_min, "xphase": s["xphase"], "haskind": s["Ah"]}
        res = {}
        tau = np.arange(N) * DT
        tau = np.where(tau > N * DT / 2, tau - N * DT, tau)
        order = np.argsort(tau)
        for r in RADII:
            res[r] = {}
            gref = kernel(variants["haskind"] * prop(k, r, m, "hankel"))
            for name, A in variants.items():
                for kind in ("asym", "hankel"):
                    g = kernel(A * prop(k, r, m, kind))
                    corr = float(np.sum(g * gref) / np.sqrt(max(np.sum(g ** 2) * np.sum(gref ** 2), 1e-300)))
                    res[r][f"{name}/{kind}"] = {"acausal_energy": acausal_fraction(g), "corr_with_haskind": corr}
            if r in (30.0, 300.0):
                ax = axes[row, 0 if r == 30.0 else 1]
                for name, st in [("haskind", "k-"), ("const", "C0--"), ("minphase", "C2:"), ("xphase", "C1-.")]:
                    g = kernel(variants[name] * prop(k, r, m, "hankel"))
                    ax.plot(tau[order], g[order], st, lw=1.2 if name == "haskind" else 1.0, label=name)
                cg_max = np.max(R.group_velocity(w[(s["mag"] > 0.01 * s["mag"].max())], s["h"].g, s["h"].depth))
                ax.axvline(0, color="C3", lw=0.8)
                lo = -10
                hi = r / max(R.group_velocity(np.array([2.5]), s["h"].g, s["h"].depth)[0], 0.1) + 20
                ax.set_xlim(lo, hi)
                ax.set_title(f"{s['label']}: kernel at r = {r:.0f} m", fontsize=9)
                ax.set_xlabel("tau [s]")
                ax.grid(alpha=0.3)
                if row == 0:
                    ax.legend(fontsize=7)
        # Haskind phase-constant scan
        psis = np.linspace(-np.pi, np.pi, 73)
        scan = [acausal_fraction(kernel(s["Ah"] * np.exp(1j * p) * prop(k, 30.0, m, "hankel"))) for p in psis]
        ax = axes[row, 2]
        ax.semilogy(np.degrees(psis), scan, "k-")
        best = float(np.degrees(psis[int(np.argmin(scan))]))
        ax.axvline(0, color="C3", lw=0.8)
        ax.set_title(f"extra phase on Haskind kernel (r=30 m): most causal at {best:+.0f} deg", fontsize=9)
        ax.set_xlabel("added constant phase [deg]"); ax.set_ylabel("acausal energy fraction")
        ax.grid(alpha=0.3)
        res["haskind_phase_scan_best_deg"] = best
        summary[s["label"]] = {str(kk): vv for kk, vv in res.items()}
        print(s["label"], json.dumps({str(r): {kk: [round(v["acausal_energy"], 4), round(v["corr_with_haskind"], 3)]
                                               for kk, v in res[r].items()} for r in RADII}), "best psi", best)
    fig.suptitle("S5  Radiated-wave kernels h_r(tau) (Hankel propagation). Red line: tau = 0 (no response allowed before it)", y=1.0)
    fig.tight_layout()
    R.savefig(fig, "s05_causality.png")
    (R.OUT / "s05_summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
