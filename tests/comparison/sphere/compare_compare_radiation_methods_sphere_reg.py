#!/usr/bin/env python3
"""
Comparison script: radiation methods (RIRF, state space, constant
coefficients) on the IEA sphere in regular waves, heave only, against the
single-DOF frequency-domain solution

    Z = a X3 / (C33 - w^2 (M + A33(w)) + i w (B33(w) + Bpto))

Steady-state amplitude and phase are fitted by least squares on
[cos wt, sin wt, 1] over the last N_FIT_PERIODS wave periods.
Mean PTO power is Bpto * mean(v^2) over the same window (simulation) and
0.5 Bpto w^2 |Z|^2 (reference).

Pass criterion (constant coefficients only): |amplitude error| <= 1 %,
|phase error| <= 1 deg at every period. RIRF and state space are shown for
information.
"""

import json
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import HAS_MATPLOTLIB, write_comparison_status

import numpy as np

if HAS_MATPLOTLIB:
    import matplotlib.pyplot as plt

TEST_NAME = "compare_radiation_methods_sphere_reg"
N_FIT_PERIODS = 10
AMP_TOL_PCT = 1.0
PHASE_TOL_DEG = 1.0

METHODS = [
    {"tag": "constant_coefficients", "label": "Constant coefficients", "marker": "o", "color": "#d62728",
     "line": dict(ls="-", lw=3.0, alpha=0.5)},
    {"tag": "rirf", "label": "RIRF convolution", "marker": "s", "color": "#1f77b4", "line": dict(ls="--", lw=1.5)},
    {"tag": "state_space", "label": "State space", "marker": "^", "color": "#2ca02c", "line": dict(ls="-.", lw=1.5)},
]


def read_header_value(path, key):
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.lstrip("#").split()
            if line.startswith("#") and len(parts) == 2 and parts[0] == key:
                return float(parts[1])
    raise KeyError(f"{key} not found in {path}")


def fit_window(t, w):
    period = 2.0 * np.pi / w
    return t >= t[-1] - N_FIT_PERIODS * period


def fit_harmonic(t, y, w):
    """Return complex amplitude Z with y ~ Re{Z exp(i w t)} + c."""
    a = np.column_stack([np.cos(w * t), np.sin(w * t), np.ones_like(t)])
    c, *_ = np.linalg.lstsq(a, y, rcond=None)
    return c[0] - 1j * c[1]


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)
    base_dir = Path(sys.argv[1])
    ref_file = base_dir / "reference.txt"
    if not ref_file.exists():
        print(f"Missing {ref_file}")
        sys.exit(1)

    mass = read_header_value(ref_file, "mass_kg")
    b_pto = read_header_value(ref_file, "pto_damping_Ns_m")
    amp = read_header_value(ref_file, "wave_amplitude_m")
    ref = np.atleast_2d(np.loadtxt(ref_file, comments="#"))

    rows = []
    series = {}
    curve = []
    for k, w, a33, b33, c33, x_re, x_im in ref:
        k = int(k)
        z_ref = amp * (x_re + 1j * x_im) / (c33 - w**2 * (mass + a33) + 1j * w * (b33 + b_pto))
        p_ref = 0.5 * b_pto * w**2 * abs(z_ref) ** 2
        curve.append((2 * np.pi / w, z_ref))
        if not any((base_dir / f"{m['tag']}_k{k}.txt").exists() for m in METHODS):
            continue
        row = {"k": k, "omega": w, "period": 2 * np.pi / w, "z_ref": z_ref, "p_ref": p_ref}
        for m in METHODS:
            path = base_dir / f"{m['tag']}_k{k}.txt"
            if not path.exists():
                print(f"Missing {path}")
                continue
            data = np.loadtxt(path, comments="#")
            t, z, v = data[:, 0], data[:, 1], data[:, 2]
            sel = fit_window(t, w)
            z_sim = fit_harmonic(t[sel], z[sel], w)
            p_sim = b_pto * np.mean(v[sel] ** 2)
            row[m["tag"]] = {
                "amp_err_pct": 100.0 * (abs(z_sim) / abs(z_ref) - 1.0),
                "phase_err_deg": float(np.degrees(np.angle(z_sim / z_ref))),
                "power_err_pct": 100.0 * (p_sim / p_ref - 1.0),
                "z": z_sim,
            }
            series[(k, m["tag"])] = (t, z)
        rows.append(row)

    # ---- Metrics, status, summary table ---------------------------------
    metrics = {}
    passed = True
    lines = [
        "| Period (s) | w (rad/s) | Method | Amplitude error (%) | Phase error (deg) | Power error (%) |",
        "|---|---|---|---|---|---|",
    ]
    for row in rows:
        for m in METHODS:
            r = row.get(m["tag"])
            if r is None:
                if m["tag"] == "constant_coefficients":
                    passed = False
                continue
            key = f"{m['tag']}_k{row['k']}"
            metrics[f"{key}_amp_err_pct"] = r["amp_err_pct"]
            metrics[f"{key}_phase_err_deg"] = r["phase_err_deg"]
            metrics[f"{key}_power_err_pct"] = r["power_err_pct"]
            lines.append(
                f"| {row['period']:.3f} | {row['omega']:.3f} | {m['label']} | "
                f"{r['amp_err_pct']:+.3f} | {r['phase_err_deg']:+.3f} | {r['power_err_pct']:+.3f} |"
            )
            if m["tag"] == "constant_coefficients" and (
                abs(r["amp_err_pct"]) > AMP_TOL_PCT or abs(r["phase_err_deg"]) > PHASE_TOL_DEG
            ):
                passed = False
    summary = "\n".join(lines)
    print(summary)
    (base_dir / "summary.md").write_text(summary + "\n", encoding="utf-8")

    note = (
        f"Constant-coefficient heave response must be within {AMP_TOL_PCT:g} % amplitude and "
        f"{PHASE_TOL_DEG:g} deg phase of the frequency-domain solution at every period."
    )
    payload = write_comparison_status(str(base_dir), TEST_NAME, metrics, passed=passed)
    if not passed:
        payload["note"] = note
        (base_dir / "status.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if HAS_MATPLOTLIB and rows:
        make_plots(base_dir, rows, series, curve, amp)

    print(f"\n{TEST_NAME}: {'PASS' if passed else 'FAIL'}")
    sys.exit(0 if passed else 1)


def make_plots(base_dir, rows, series, curve, amp):
    plot_dir = base_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    periods = np.array([r["period"] for r in rows])
    z_ref = np.array([r["z_ref"] for r in rows])
    t_lo, t_hi = 0.9 * periods.min(), 1.1 * periods.max()
    curve = sorted(c for c in curve if t_lo <= c[0] <= t_hi)
    curve_t = np.array([c[0] for c in curve])
    curve_z = np.array([c[1] for c in curve])

    # RAO amplitude and phase vs period
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ref_label = "Frequency domain (reference, all BEM grid points)"
    ax1.plot(curve_t, np.abs(curve_z) / amp, "k-", lw=1.5, label=ref_label)
    ax2.plot(curve_t, np.degrees(np.angle(curve_z)), "k-", lw=1.5, label=ref_label)
    for m in METHODS:
        pts = [(r["period"], r[m["tag"]]["z"]) for r in rows if m["tag"] in r]
        if not pts:
            continue
        p, z = np.array([q[0] for q in pts]), np.array([q[1] for q in pts])
        style = dict(ls="none", marker=m["marker"], color=m["color"], mfc="none", ms=9, mew=1.5, label=m["label"])
        ax1.plot(p, np.abs(z) / amp, **style)
        ax2.plot(p, np.degrees(np.angle(z)), **style)
    ax1.set_ylabel("Heave RAO |Z|/a (m/m)")
    ax1.set_title("Sphere heave in regular waves (PTO damper): radiation methods vs frequency domain")
    ax2.set_ylabel("Heave phase re. wave (deg)")
    ax2.set_xlabel("Wave period (s)")
    for ax in (ax1, ax2):
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(plot_dir / "heave_rao_vs_period.png", dpi=150)
    plt.close(fig)

    # Errors vs period
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    keys = [("amp_err_pct", "Amplitude error (%)", AMP_TOL_PCT),
            ("phase_err_deg", "Phase error (deg)", PHASE_TOL_DEG),
            ("power_err_pct", "Mean PTO power error (%)", None)]
    for ax, (key, label, tol) in zip(axes, keys):
        for m in METHODS:
            pts = [(r["period"], r[m["tag"]][key]) for r in rows if m["tag"] in r]
            if pts:
                ax.plot(*zip(*pts), marker=m["marker"], color=m["color"], mfc="none", ms=8, label=m["label"])
        if tol is not None:
            ax.axhspan(-tol, tol, color="#d62728", alpha=0.08, label=f"CC pass band (±{tol:g})")
        ax.axhline(0.0, color="k", lw=0.8)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[0].set_title("Sphere heave: error relative to the frequency-domain solution")
    axes[0].legend(fontsize=9)
    axes[-1].set_xlabel("Wave period (s)")
    fig.tight_layout()
    fig.savefig(plot_dir / "heave_errors_vs_frequency_domain.png", dpi=150)
    plt.close(fig)

    # Time series at the period with the largest reference response
    i_res = int(np.argmax(np.abs(z_ref)))
    row = rows[i_res]
    w, k = row["omega"], row["k"]
    fig, ax = plt.subplots(figsize=(10, 5))
    t_ref = None
    for m in METHODS:
        if (k, m["tag"]) not in series:
            continue
        t, z = series[(k, m["tag"])]
        sel = t >= t[-1] - 4 * row["period"]
        ax.plot(t[sel], z[sel], color=m["color"], label=m["label"], **m["line"])
        t_ref = t[sel]
    if t_ref is not None:
        z_ss = np.real(row["z_ref"] * np.exp(1j * w * t_ref))
        ax.plot(t_ref, z_ss, "k:", lw=1.5, label="Frequency domain (steady state)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Heave from equilibrium (m)")
    ax.set_title(f"Sphere heave, last 4 periods at T = {row['period']:.2f} s (largest response)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(plot_dir / "heave_time_series_largest_response.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
