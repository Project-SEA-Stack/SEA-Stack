"""S4: damping implied by the assumed patterns vs the BEM damping matrix.

Single body.  The assumed patterns are orthogonal harmonic groups:
  m=0 {heave}, m=1 cos {surge, pitch}, m=1 sin {sway, roll}, m=2 {yaw}.
Two ways to set the amplitudes inside a group:
  'diag': b_j = s_j sqrt(B_jj), s_j = sign of the coupling to the group lead
          (exact power for every single-DOF motion; implied B_ij = s sqrt(B_ii B_jj));
  'eig' : leading eigenvector of the group block (captures most power; exact
          only when the block has rank one).
Cross-group damping (e.g. sway-yaw on a ship, surge-heave on a hull that is not
fore-aft symmetric) cannot be represented and is dropped.

Several bodies.  Each body radiates its own pattern from its own reference
point; superposition then implies cross-body damping
  B_impl[bi, cj] = (rho g c_g / k) Re int A_bi(theta) A_cj(theta)^* dtheta,
which we compare with the BEM cross-body blocks.

Power ratio range: for any motion u, P_drawn/P_true = u^T B_impl u / u^T B u.
Its range over all motions (generalised eigenvalues on the subspace where
B is non-negligible) bounds how wrong the drawn energy can be.
"""
import json

import matplotlib.pyplot as plt
import numpy as np

import rwlib as R

GROUPS = [[2], [0, 4], [1, 3], [5]]


def damping(h):
    return h.B if h.B is not None else R.b_from_k_fit(h.t, h.K, h.w)


def group_alpha(blk):
    """Complex per-DOF coefficients alpha (relative units) with Re(alpha alpha^H) = blk
    for a 1x1 or 2x2 PSD block: |alpha_j| = sqrt(B_jj), relative phase
    phi = arccos(B_12 / sqrt(B_11 B_22)).  Exact for every motion in the group."""
    d = np.sqrt(np.maximum(np.diag(blk), 0))
    a = d.astype(complex)
    if len(d) == 2 and d[0] > 0 and d[1] > 0:
        c = np.clip(blk[0, 1] / (d[0] * d[1]), -1.0, 1.0)
        a[1] = d[1] * np.exp(1j * np.arccos(c))
    return a


def group_approx(Bb, mode):
    """Approximate 6x6 block (power-scaled) for one body at one frequency."""
    out = np.zeros_like(Bb)
    for G in GROUPS:
        blk = Bb[np.ix_(G, G)]
        if mode == "phase":
            a = group_alpha(0.5 * (blk + blk.T))
            out[np.ix_(G, G)] = np.real(np.outer(a, np.conj(a)))
            continue
        if mode == "diag":
            d = np.sqrt(np.maximum(np.diag(blk), 0))
            s = np.ones(len(G))
            for q in range(1, len(G)):
                s[q] = np.sign(blk[0, q]) if blk[0, q] != 0 else 1.0
            b = s * d
        else:
            lam, vec = np.linalg.eigh(0.5 * (blk + blk.T))
            b = np.sqrt(max(lam[-1], 0)) * vec[:, -1]
        out[np.ix_(G, G)] = np.outer(b, b)
    return out


def ratio_range(Ba, B, tol=1e-3):
    """Range of u^T Ba u / u^T B u over u in the dominant subspace of B."""
    Bs = 0.5 * (B + B.T)
    lam, V = np.linalg.eigh(Bs)
    keep = lam > tol * max(lam.max(), 1e-300)
    if not keep.any():
        return np.nan, np.nan
    T = V[:, keep] / np.sqrt(lam[keep])
    M = T.T @ (0.5 * (Ba + Ba.T)) @ T
    mu = np.linalg.eigvalsh(M)
    return float(mu.min()), float(mu.max())


def implied_array(h, B, d, mode, ntheta=360):
    """Superposition of per-body approximate patterns, power-scaled (N s/m)."""
    th = np.linspace(0, 2 * np.pi, ntheta, endpoint=False)
    k = R.wavenumber(h.w, h.g, h.depth)
    cgv = R.group_velocity(h.w, h.g, h.depth)
    n = 6 * h.nb
    Bimp = np.zeros((len(h.w), n, n))
    for iw in range(len(h.w)):
        # per-body far-field amplitudes per unit (scaled) velocity
        Amp = np.zeros((n, ntheta), complex)
        for b in range(h.nb):
            sl = R.body_slice(b)
            Bb = (B[iw][sl, sl] * d[sl][:, None] * d[sl][None, :])
            Ba = group_approx(Bb, mode)
            p = h.cg[b, :2]
            shift = np.exp(1j * k[iw] * (p[0] * np.cos(th) + p[1] * np.sin(th)))
            for G in GROUPS:
                m, fun, _ = R.PATTERNS[G[0]]
                bvec = group_alpha(0.5 * (Bb[np.ix_(G, G)] + Bb[np.ix_(G, G)].T))
                scale = np.sqrt(k[iw] / (h.rho * h.g * cgv[iw] * R.pattern_norm(m)))
                lead_sign = -1.0 if G[0] == 2 else 1.0     # S3: heave -1, surge/sway +1
                for q, j in enumerate(G):
                    Amp[6 * b + j] = scale * bvec[q] * R.model_phase(m, lead_sign) * fun(th) * shift
        wq = 2 * np.pi / ntheta
        Bimp[iw] = (h.rho * h.g * cgv[iw] / k[iw]) * np.real(Amp @ np.conj(Amp).T) * wq
    return Bimp


def main():
    summary = {}
    single = ["sphere", "oswec", "crane", "wigley", "5sa_dir", "trimaran", "f3of"]
    fig, axes = plt.subplots(2, 4, figsize=(20, 8.5))
    for ax, key in zip(axes.flat, single):
        h = R.load(key)
        B = damping(h)
        d = R.power_scaling(h)
        res = {}
        for b in range(min(h.nb, 3)):
            sl = R.body_slice(b)
            rr = {"diag": [], "eig": [], "phase": []}
            cross, fro = [], {"diag": [], "eig": [], "phase": []}
            for iw in range(len(h.w)):
                Bb = B[iw][sl, sl] * d[sl][:, None] * d[sl][None, :]
                Bb = 0.5 * (Bb + Bb.T)
                mask = np.zeros((6, 6), bool)
                for G in GROUPS:
                    mask[np.ix_(G, G)] = True
                cross.append(np.linalg.norm(Bb[~mask]) / max(np.linalg.norm(Bb), 1e-300))
                for mode in rr:
                    Ba = group_approx(Bb, mode)
                    rr[mode].append(ratio_range(Ba, Bb))
                    fro[mode].append(np.linalg.norm(Ba - Bb) / max(np.linalg.norm(Bb), 1e-300))
            weight = np.trace(B[:, sl, sl] * (d[sl][:, None] * d[sl][None, :])[None], axis1=1, axis2=2)
            active = weight > 0.02 * weight.max()
            cross = np.array(cross)
            entry = {"cross_group_fraction_B_weighted": float(np.sum(cross * weight * active) / np.sum(weight * active)),
                     "cross_group_fraction_max": float(cross[active].max())}
            for mode in rr:
                a = np.array(rr[mode])
                entry[f"{mode}_power_ratio_min"] = float(np.nanmin(a[active, 0]))
                entry[f"{mode}_power_ratio_max"] = float(np.nanmax(a[active, 1]))
                entry[f"{mode}_frobenius_err_B_weighted"] = float(np.sum(np.array(fro[mode]) * weight * active) / np.sum(weight * active))
            res[f"body{b + 1}"] = entry
            if b == 0:
                a = np.array(rr["diag"]); e = np.array(rr["eig"]); p_ = np.array(rr["phase"])
                ww = np.where(active, h.w, np.nan)
                ax.fill_between(ww, a[:, 0], a[:, 1], color="C0", alpha=0.25, label="same-phase sqrt(B_jj): drawn/true power range")
                ax.plot(ww, e[:, 1], "C1--", lw=1, label="leading eigenvector: max ratio (min = 0)")
                ax.fill_between(ww, p_[:, 0], p_[:, 1], color="C2", alpha=0.5, label="relative-phase alpha: range")
                ax.plot(ww, cross, "C3:", lw=1.5, label="cross-group share ||B_x||/||B||")
        summary[key] = res
        ax.axhline(1, color="k", lw=0.6)
        ax.set_ylim(-0.05, 2.5)
        ax.set_xlim(0, min(h.w[-1], 4))
        ax.set_title(f"{key} body 1 (self block)", fontsize=10)
        ax.set_xlabel("w [rad/s]")
        ax.grid(alpha=0.3)
        print(key, json.dumps(res)[:700])
    axes.flat[0].legend(fontsize=7)

    # ---- multi-body superposition ---------------------------------------------------
    ax = axes.flat[-1]
    multi = {}
    for key, col in [("5sa_dir", "C0"), ("5sa", "C4"), ("f3of", "C1"), ("rm3", "C2"), ("trimaran", "C3")]:
        h = R.load(key)
        B = damping(h)
        d = R.power_scaling(h)
        Bp = B * d[None, :, None] * d[None, None, :]
        Bimp = implied_array(h, B, d, "phase")
        lo, hi, ip = [], [], []
        u = np.zeros(6 * h.nb); u[2::6] = 1.0
        ref_ip = np.array([u @ Bp[iw] @ u for iw in range(len(h.w))])
        for iw in range(len(h.w)):
            # only motions radiating >= 5 % of the strongest motion at this frequency
            a, b_ = ratio_range(Bimp[iw], Bp[iw], tol=0.05)
            lo.append(a); hi.append(b_)
            # in-phase heave of all bodies: a common case for arrays
            ok = ref_ip[iw] > 0.02 * ref_ip.max()
            ip.append(float(u @ Bimp[iw] @ u / ref_ip[iw]) if ok else np.nan)
        lo, hi, ip = map(np.array, (lo, hi, ip))
        weight = np.trace(Bp, axis1=1, axis2=2)
        active = weight > 0.02 * weight.max()
        ww = np.where(active, h.w, np.nan)
        ax.plot(ww, ip, col + "-", lw=1.4, label=f"{key}: all bodies heave in phase")
        ax.plot(ww, hi, col + ":", lw=1.0)
        cross_err = []
        for iw in range(len(h.w)):
            off = np.ones((6 * h.nb, 6 * h.nb), bool)
            for b in range(h.nb):
                off[R.body_slice(b), R.body_slice(b)] = False
            cross_err.append(np.linalg.norm((Bimp[iw] - Bp[iw])[off]) / max(np.linalg.norm(Bp[iw][off]), 1e-300))
        cross_err = np.array(cross_err)
        multi[key] = {
            "bodies": h.nb,
            "spacing_m": [float(np.linalg.norm(h.cg[i, :2] - h.cg[j, :2])) for i in range(h.nb) for j in range(i + 1, h.nb)],
            "inphase_heave_ratio_range": [float(np.nanmin(ip[active])), float(np.nanmax(ip[active]))],
            "any_motion_ratio_range": [float(np.nanmin(lo[active])), float(np.nanmax(hi[active]))],
            "cross_body_block_rel_err_B_weighted": float(np.sum(cross_err * weight * active) / np.sum(weight * active)),
        }
        print("multi", key, json.dumps(multi[key]))
    ax.axhline(1, color="k", lw=0.6)
    ax.set_ylim(0, 3)
    ax.set_xlim(0, 3)
    ax.set_title("arrays: drawn/true power, superposition of isolated patterns\n(solid: all heave in phase; dotted: worst motion)", fontsize=9)
    ax.set_xlabel("w [rad/s]")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    fig.suptitle("S4  Damping implied by the assumed patterns vs BEM damping (power at equal surface speed)", y=1.0)
    fig.tight_layout()
    R.savefig(fig, "s04_coupling.png")

    # ---- cross-body heave damping: BEM vs isolated point sources -------------------------
    fig2, axs = plt.subplots(1, 3, figsize=(17, 4.2))
    for ax, (key, pairs) in zip(axs, [("5sa_dir", [(0, 1), (1, 2), (0, 4)]), ("f3of", [(0, 1), (1, 2)]),
                                       ("trimaran", [(0, 1), (1, 2)])]):
        h = R.load(key)
        B = damping(h)
        k = R.wavenumber(h.w, h.g, h.depth)
        from scipy.special import j0
        for q, (b1, b2) in enumerate(pairs):
            i, j = 6 * b1 + 2, 6 * b2 + 2
            norm = np.sqrt(np.maximum(B[:, i, i] * B[:, j, j], 1e-300))
            act = norm > 0.02 * norm.max()
            dist = np.linalg.norm(h.cg[b1, :2] - h.cg[b2, :2])
            ax.plot(np.where(act, h.w, np.nan), B[:, i, j] / norm, f"C{q}-", lw=1.5,
                    label=f"BEM b{b1 + 1}-b{b2 + 1} (d={dist:.0f} m)")
            ax.plot(np.where(act, h.w, np.nan), j0(k * dist), f"C{q}--", lw=1, label="isolated sources J0(kd)")
        ax.set_ylim(-1.1, 1.1); ax.axhline(0, color="k", lw=0.5)
        ax.set_xlim(0, 3); ax.grid(alpha=0.3); ax.set_xlabel("w [rad/s]")
        ax.set_title(f"{key}: heave-heave B_bc / sqrt(B_bb B_cc)")
        ax.legend(fontsize=7)
    fig2.suptitle("S4  Body-to-body heave damping: BEM vs superposition of isolated point sources", y=1.02)
    fig2.tight_layout()
    R.savefig(fig2, "s04_array_heave_coupling.png")
    summary["multi_body"] = multi
    (R.OUT / "s04_summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
