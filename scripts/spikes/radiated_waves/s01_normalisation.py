"""S1: units, normalisation and phase conventions of the stored radiation data.

Checks, per demo file:
  * stored B(w) (rho*w*B_file) against B reconstructed from K(t) (rho*K_file)
    by the cosine transform B = int_0^T K cos(w t) dt, and (for files built on
    a coarse BEM grid) by a least-squares fit of the discrete cosine sum;
  * Haskind damping from the excitation over the full circle of headings,
    B = k/(8 pi rho g c_g) int X X^* dbeta -- an independent check of the
    rho*g excitation normalisation and of the Haskind constant;
  * the excitation phase convention (low-frequency surge phase at beta = 0).
"""
import json

import matplotlib.pyplot as plt
import numpy as np

import rwlib as R

CASES = ["sphere", "rm3", "f3of", "oswec", "crane", "wigley", "5sa_dir", "trimaran"]
PAIRS = [(0, 0, "B11 surge"), (2, 2, "B33 heave"), (4, 4, "B55 pitch"), (0, 4, "B15 surge-pitch")]


def haskind_for(h):
    """Return (B_haskind, how) or (None, reason)."""
    if R.is_full_circle(h.dirs):
        d, X = R.full_circle(h)
        return R.haskind_b(h, X, d), f"{len(d)} headings"
    if h.name == "oswec":
        d, X = R.full_circle(h, symmetric_xz=True, symmetric_yz=True)
        return R.haskind_b(h, X, d), f"{len(d)} headings (x-z and y-z mirror assumed)"
    if h.name in ("sphere", "rm3"):
        d, X = R.axisym_circle(h)
        return R.haskind_b(h, X, d), "axisymmetric extension of beta=0"
    return None, "single heading, not axisymmetric"


def band_ratio(est, ref, w, lo=0.2, hi=None):
    hi = hi or w[-1]
    m = (w >= lo) & (w <= hi) & (np.abs(ref) > 0.05 * np.abs(ref).max())
    if not m.any():
        return float("nan"), float("nan")
    r = est[m] / ref[m]
    return float(np.median(r)), float(np.percentile(np.abs(r - 1), 90))


def main():
    summary = {}
    fig, axes = plt.subplots(len(CASES), len(PAIRS), figsize=(17, 3.0 * len(CASES)))
    for row, key in enumerate(CASES):
        h = R.load(key)
        Bd = R.b_from_k_direct(h.t, h.K, h.w)
        Bfit = R.b_from_k_fit(h.t, h.K, h.w) if h.code == "capytaine-script" else None
        Bh, how = haskind_for(h)
        ref = h.B if h.B is not None else (Bfit if Bfit is not None else Bd)
        ref_name = "stored B" if h.B is not None else "K fit on BEM grid"
        info = {"code": h.code, "Nw": len(h.w), "w_range": [float(h.w[0]), float(h.w[-1])],
                "K_T_end_s": float(h.t[-1]), "K_dt_s": float(h.t[1] - h.t[0]),
                "depth_m": h.depth if np.isfinite(h.depth) else "inf", "reference": ref_name,
                "haskind": how}
        i0 = int(np.argmin(np.abs(np.angle(np.exp(1j * h.dirs)))))
        k3 = int(np.argmin(np.abs(h.w - 0.3)))
        info["surge_phase_deg_at_w0.3_beta0_raw"] = float(np.degrees(np.angle(h.X[k3, i0, 0])))
        for col, (i, j, label) in enumerate(PAIRS):
            ax = axes[row, col]
            if h.B is not None:
                ax.plot(h.w, h.B[:, i, j], "k-", lw=2, label="stored  rho*w*B_file")
            ax.plot(h.w, Bd[:, i, j], "C0--", lw=1.3, label="cosine transform of rho*K")
            if Bfit is not None:
                ax.plot(h.w, Bfit[:, i, j], "C2:", lw=1.8, label="LS fit of K on BEM grid")
            if Bh is not None:
                ax.plot(h.w, Bh[:, i, j], "C3.", ms=3, label="Haskind from excitation")
            if row == 0:
                ax.set_title(label)
            if col == 0:
                ax.set_ylabel(f"{key}\n[{h.code}]\nN s/m, N m s/rad")
            ax.set_xlim(0, min(h.w[-1], 4.0))
            ax.grid(alpha=0.3)
            if row == 0 and col == 0:
                ax.legend(fontsize=7)
            stats = {"direct_vs_ref": band_ratio(Bd[:, i, j], ref[:, i, j], h.w)}
            if Bh is not None:
                stats["haskind_vs_ref"] = band_ratio(Bh[:, i, j], ref[:, i, j], h.w)
            if Bfit is not None:
                stats["direct_vs_fit"] = band_ratio(Bd[:, i, j], Bfit[:, i, j], h.w)
            info[label] = stats
        summary[key] = info
        print(key, json.dumps(info, indent=None)[:600])
    for ax in axes[-1]:
        ax.set_xlabel("w [rad/s]")
    fig.suptitle("S1  Radiation damping body 1: stored vs reconstructed from K(t) vs Haskind (excitation)", y=1.0)
    fig.tight_layout()
    R.savefig(fig, "s01_B_reconstruction.png")
    (R.OUT / "s01_summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
