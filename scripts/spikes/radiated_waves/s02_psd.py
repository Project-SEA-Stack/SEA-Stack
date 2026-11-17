"""S2: symmetry and positive semidefiniteness of the full 6N x 6N damping matrix.

Passive radiation requires B(w) = B(w)^T >= 0 for the coupled system.  Single
off-diagonal entries may be negative; only negative eigenvalues of the full
matrix indicate non-physical data (numerical noise, irregular frequencies,
truncation of K).  Nothing is clamped here.

Scaling: rotations are compared at equal surface speed (D = diag(1,1,1,1/L,1/L,1/L)
per body, L = V^(1/3)); the sign of each eigenvalue is unchanged by this
congruence.  Eigenvalues are reported relative to the largest eigenvalue over
all frequencies, which is what matters for radiated power.
"""
import json

import matplotlib.pyplot as plt
import numpy as np

import rwlib as R

CASES = ["sphere", "rm3", "f3of", "oswec", "crane", "wigley", "5sa", "5sa_dir", "trimaran"]
SIGNIFICANT = 1e-2   # |lam_min| / max_w lam_max


def damping(h):
    if h.B is not None:
        return h.B, "stored"
    return R.b_from_k_fit(h.t, h.K, h.w), "K fit on BEM grid"


def main():
    summary = {}
    fig, axes = plt.subplots(3, 3, figsize=(16, 11))
    for ax, key in zip(axes.flat, CASES):
        h = R.load(key)
        B, src = damping(h)
        d = R.power_scaling(h)
        Bp = B * d[None, :, None] * d[None, None, :]
        asym = np.linalg.norm(Bp - np.swapaxes(Bp, 1, 2), axis=(1, 2))
        Bs = 0.5 * (Bp + np.swapaxes(Bp, 1, 2))
        lam, vec = np.linalg.eigh(Bs)
        scale = lam[:, -1].max()
        rel_min = lam[:, 0] / scale
        rel_asym = asym / scale
        worst = int(np.argmin(rel_min))
        v = vec[worst, :, 0]
        dom = [f"b{i // 6 + 1}:{R.DOF[i % 6]}({v[i]:+.2f})" for i in np.argsort(-np.abs(v))[:4]]
        sig = rel_min < -SIGNIFICANT
        # also the self blocks alone (what the per-body approximation would use)
        self_min = np.min([np.linalg.eigvalsh(Bs[:, R.body_slice(b), R.body_slice(b)])[:, 0]
                           for b in range(h.nb)], axis=0) / scale
        summary[key] = {
            "source": src,
            "max_asymmetry_over_max_eig": float(rel_asym.max()),
            "min_eig_over_max_eig": float(rel_min.min()),
            "at_w": float(h.w[worst]),
            "worst_eigvec": dom,
            "freqs_significant_negative": [round(float(x), 3) for x in h.w[sig]],
            "self_blocks_min_eig_over_max_eig": float(self_min.min()),
        }
        ax.plot(h.w, lam[:, -1] / scale, "k-", lw=1, label="lam_max / max_w lam_max")
        ax.plot(h.w, rel_min, "C3-", lw=1.2, label="lam_min (full coupled)")
        ax.plot(h.w, self_min, "C0--", lw=1, label="lam_min (per-body self blocks)")
        ax.plot(h.w, rel_asym, "C2:", lw=1, label="||B - B^T||")
        ax.axhline(-SIGNIFICANT, color="C3", ls=":", lw=0.8)
        ax.axhline(0, color="k", lw=0.5)
        ax.set_yscale("symlog", linthresh=1e-4)
        ax.set_ylim(-1.5, 1.5)
        ax.set_xlim(0, min(h.w[-1], 5))
        ax.set_title(f"{key} ({src})  min {rel_min.min():+.1e} at w={h.w[worst]:.2f}", fontsize=9)
        ax.grid(alpha=0.3)
        ax.set_xlabel("w [rad/s]")
        print(key, json.dumps(summary[key]))
    axes.flat[0].legend(fontsize=7)
    fig.suptitle("S2  Eigenvalues of the full coupled damping matrix (rotations at equal surface speed), "
                 "relative to the largest over all w", y=1.0)
    fig.tight_layout()
    R.savefig(fig, "s02_psd.png")
    (R.OUT / "s02_summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
