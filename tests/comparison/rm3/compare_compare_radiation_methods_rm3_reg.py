#!/usr/bin/env python3
"""
Comparison script: radiation methods (RIRF, state space, constant
coefficients) on RM3 in regular waves, two heave DOFs (float on plate, plate
on ground), against the coupled 2x2 frequency-domain solution

    Z = [C - w^2 (M + A(w)) + i w (B(w) + B_pto)]^-1 a X,
    B_pto = c_pto [[1, -1], [-1, 1]].

The same solve with the body-to-body radiation terms (A28, A82, B28, B82)
set to zero is plotted to show the size of the hydrodynamic coupling.

Steady-state amplitude and phase are fitted by least squares on
[cos wt, sin wt, 1] over the last N_FIT_PERIODS wave periods.
Mean PTO power is c_pto * mean((v1 - v2)^2) over the same window
(simulation) and 0.5 c_pto w^2 |Z1 - Z2|^2 (reference).

Pass criterion (constant coefficients only): float and plate heave within
1 % amplitude and 1 deg phase, mean PTO power within 2 %, at every period.
RIRF and state space are shown for information.
"""

import json
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import HAS_MATPLOTLIB, write_comparison_status

import numpy as np

if HAS_MATPLOTLIB:
    import matplotlib.pyplot as plt

TEST_NAME = "compare_radiation_methods_rm3_reg"
N_FIT_PERIODS = 10
AMP_TOL_PCT = 1.0
PHASE_TOL_DEG = 1.0
POWER_TOL_PCT = 2.0

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


def fit_harmonic(t, y, w):
    """Return complex amplitude Z with y ~ Re{Z exp(i w t)} + c."""
    a = np.column_stack([np.cos(w * t), np.sin(w * t), np.ones_like(t)])
    c, *_ = np.linalg.lstsq(a, y, rcond=None)
    return c[0] - 1j * c[1]


def solve_fd(w, amp, m1, m2, c_pto, a, b, c, x, coupled=True):
    """Two-DOF heave response [Z_float, Z_plate] (m)."""
    if not coupled:
        a = np.diag(np.diag(a))
        b = np.diag(np.diag(b))
    mass = np.diag([m1, m2])
    b_pto = c_pto * np.array([[1.0, -1.0], [-1.0, 1.0]])
    impedance = c - w**2 * (mass + a) + 1j * w * (b + b_pto)
    return np.linalg.solve(impedance, amp * x)


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)
    base_dir = Path(sys.argv[1])
    ref_file = base_dir / "reference.txt"
    if not ref_file.exists():
        print(f"Missing {ref_file}")
        sys.exit(1)

    m1 = read_header_value(ref_file, "float_mass_kg")
    m2 = read_header_value(ref_file, "plate_mass_kg")
    c_pto = read_header_value(ref_file, "pto_damping_Ns_m")
    amp = read_header_value(ref_file, "wave_amplitude_m")
    ref = np.atleast_2d(np.loadtxt(ref_file, comments="#"))

    rows = []
    series = {}
    curve = []
    for r in ref:
        k, w = int(r[0]), r[1]
        a = np.array([[r[2], r[3]], [r[4], r[5]]])
        b = np.array([[r[6], r[7]], [r[8], r[9]]])
        c = np.diag([r[10], r[11]])
        x = np.array([r[12] + 1j * r[13], r[14] + 1j * r[15]])
        z_ref = solve_fd(w, amp, m1, m2, c_pto, a, b, c, x)
        z_unc = solve_fd(w, amp, m1, m2, c_pto, a, b, c, x, coupled=False)
        row = {
            "k": k, "omega": w, "period": 2 * np.pi / w,
            "z_ref": z_ref, "z_unc": z_unc,
            "p_ref": 0.5 * c_pto * w**2 * abs(z_ref[0] - z_ref[1]) ** 2,
            "p_unc": 0.5 * c_pto * w**2 * abs(z_unc[0] - z_unc[1]) ** 2,
        }
        curve.append(row)
        if not any((base_dir / f"{m['tag']}_k{k}.txt").exists() for m in METHODS):
            continue
        for m in METHODS:
            path = base_dir / f"{m['tag']}_k{k}.txt"
            if not path.exists():
                print(f"Missing {path}")
                continue
            data = np.loadtxt(path, comments="#")
            t = data[:, 0]
            sel = t >= t[-1] - N_FIT_PERIODS * row["period"]
            z_sim = np.array([fit_harmonic(t[sel], data[sel, 1], w),
                              fit_harmonic(t[sel], data[sel, 2], w)])
            p_sim = c_pto * np.mean((data[sel, 3] - data[sel, 4]) ** 2)
            row[m["tag"]] = {
                "z": z_sim,
                "p": p_sim,
                "float_amp_err_pct": 100.0 * (abs(z_sim[0]) / abs(z_ref[0]) - 1.0),
                "plate_amp_err_pct": 100.0 * (abs(z_sim[1]) / abs(z_ref[1]) - 1.0),
                "float_phase_err_deg": float(np.degrees(np.angle(z_sim[0] / z_ref[0]))),
                "plate_phase_err_deg": float(np.degrees(np.angle(z_sim[1] / z_ref[1]))),
                "power_err_pct": 100.0 * (p_sim / row["p_ref"] - 1.0),
            }
            series[(k, m["tag"])] = (t, data[:, 1] - data[:, 2])
        rows.append(row)

    # ---- Metrics, status, summary table ---------------------------------
    metrics = {}
    passed = True
    err_keys = ["float_amp_err_pct", "float_phase_err_deg", "plate_amp_err_pct",
                "plate_phase_err_deg", "power_err_pct"]
    lines = [
        "| Period (s) | Method | Float amp (%) | Float phase (deg) | Plate amp (%) | "
        "Plate phase (deg) | PTO power (%) |",
        "|---|---|---|---|---|---|---|",
    ]
    for row in rows:
        for m in METHODS:
            r = row.get(m["tag"])
            if r is None:
                if m["tag"] == "constant_coefficients":
                    passed = False
                continue
            for key in err_keys:
                metrics[f"{m['tag']}_k{row['k']}_{key}"] = r[key]
            lines.append(f"| {row['period']:.3f} | {m['label']} | "
                         + " | ".join(f"{r[key]:+.3f}" for key in err_keys) + " |")
            if m["tag"] == "constant_coefficients" and (
                max(abs(r["float_amp_err_pct"]), abs(r["plate_amp_err_pct"])) > AMP_TOL_PCT
                or max(abs(r["float_phase_err_deg"]), abs(r["plate_phase_err_deg"])) > PHASE_TOL_DEG
                or abs(r["power_err_pct"]) > POWER_TOL_PCT
            ):
                passed = False
    lines.append("")
    lines.append("Coupling effect (uncoupled vs coupled FD): "
                 + ", ".join(f"T={row['period']:.2f}s plate {100 * (abs(row['z_unc'][1]) / abs(row['z_ref'][1]) - 1):+.1f}%"
                             for row in rows))
    summary = "\n".join(lines)
    print(summary)
    (base_dir / "summary.md").write_text(summary + "\n", encoding="utf-8")

    payload = write_comparison_status(str(base_dir), TEST_NAME, metrics, passed=passed)
    if not passed:
        payload["note"] = (
            f"Constant-coefficient float/plate heave must be within {AMP_TOL_PCT:g} % amplitude and "
            f"{PHASE_TOL_DEG:g} deg phase, and mean PTO power within {POWER_TOL_PCT:g} %, of the "
            "coupled frequency-domain solution at every period."
        )
        (base_dir / "status.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if HAS_MATPLOTLIB and rows:
        make_plots(base_dir, rows, series, curve, amp)

    print(f"\n{TEST_NAME}: {'PASS' if passed else 'FAIL'}")
    sys.exit(0 if passed else 1)


def method_points(rows, tag, fn):
    pts = [(r["period"], fn(r[tag])) for r in rows if tag in r]
    return (np.array([p[0] for p in pts]), np.array([p[1] for p in pts])) if pts else (None, None)


def make_plots(base_dir, rows, series, curve, amp):
    plot_dir = base_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    periods = np.array([r["period"] for r in rows])
    t_lo, t_hi = 0.9 * periods.min(), 1.1 * periods.max()
    curve = sorted((c for c in curve if t_lo <= c["period"] <= t_hi), key=lambda c: c["period"])
    curve_t = [c["period"] for c in curve]

    # Response vs period
    panels = [
        ("Float heave RAO |Z1|/a (m/m)", lambda r: abs(r["z_ref"][0]) / amp, lambda r: abs(r["z_unc"][0]) / amp,
         lambda d: abs(d["z"][0]) / amp),
        ("Plate heave RAO |Z2|/a (m/m)", lambda r: abs(r["z_ref"][1]) / amp, lambda r: abs(r["z_unc"][1]) / amp,
         lambda d: abs(d["z"][1]) / amp),
        ("Mean PTO power (kW)", lambda r: r["p_ref"] / 1e3, lambda r: r["p_unc"] / 1e3, lambda d: d["p"] / 1e3),
    ]
    fig, axes = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
    for ax, (label, f_ref, f_unc, f_sim) in zip(axes, panels):
        ax.plot(curve_t, [f_ref(r) for r in curve], "k-", lw=1.5,
                label="Coupled frequency domain (reference, all BEM grid points)")
        ax.plot(curve_t, [f_unc(r) for r in curve], color="0.5", ls="--", lw=1.2,
                label="Frequency domain without body-to-body radiation terms")
        for m in METHODS:
            p, y = method_points(rows, m["tag"], f_sim)
            if p is not None:
                ax.plot(p, y, ls="none", marker=m["marker"], color=m["color"], mfc="none", ms=9, mew=1.5,
                        label=m["label"])
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[0].set_title("RM3 two-body heave in regular waves: radiation methods vs coupled frequency domain")
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel("Wave period (s)")
    fig.tight_layout()
    fig.savefig(plot_dir / "rm3_response_vs_period.png", dpi=150)
    plt.close(fig)

    # Errors vs period
    panels = [
        ("float_amp_err_pct", "Float heave amplitude error (%)", AMP_TOL_PCT),
        ("plate_amp_err_pct", "Plate heave amplitude error (%)", AMP_TOL_PCT),
        ("float_phase_err_deg", "Float heave phase error (deg)", PHASE_TOL_DEG),
        ("plate_phase_err_deg", "Plate heave phase error (deg)", PHASE_TOL_DEG),
        ("power_err_pct", "Mean PTO power error (%)", POWER_TOL_PCT),
    ]
    fig, axes = plt.subplots(len(panels), 1, figsize=(10, 14), sharex=True)
    for ax, (key, label, tol) in zip(axes, panels):
        for m in METHODS:
            p, y = method_points(rows, m["tag"], lambda d: d[key])
            if p is not None:
                ax.plot(p, y, marker=m["marker"], color=m["color"], mfc="none", ms=8, label=m["label"])
        ax.axhspan(-tol, tol, color="#d62728", alpha=0.08, label=f"CC pass band (±{tol:g})")
        ax.axhline(0.0, color="k", lw=0.8)
        ax.set_ylabel(label, fontsize=9)
        ax.grid(True, alpha=0.3)
    axes[0].set_title("RM3 two-body heave: error relative to the coupled frequency-domain solution")
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel("Wave period (s)")
    fig.tight_layout()
    fig.savefig(plot_dir / "rm3_errors_vs_coupled_frequency_domain.png", dpi=150)
    plt.close(fig)

    # Relative heave (PTO stroke) time series at the largest-power period
    row = rows[int(np.argmax([r["p_ref"] for r in rows]))]
    w, k = row["omega"], row["k"]
    fig, ax = plt.subplots(figsize=(10, 5))
    t_ref = None
    for m in METHODS:
        if (k, m["tag"]) not in series:
            continue
        t, s = series[(k, m["tag"])]
        sel = t >= t[-1] - 4 * row["period"]
        ax.plot(t[sel], s[sel], color=m["color"], label=m["label"], **m["line"])
        t_ref = t[sel]
    if t_ref is not None:
        s_ss = np.real((row["z_ref"][0] - row["z_ref"][1]) * np.exp(1j * w * t_ref))
        ax.plot(t_ref, s_ss, "k:", lw=1.5, label="Coupled frequency domain (steady state)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Relative heave float - plate (m)")
    ax.set_title(f"RM3 PTO stroke, last 4 periods at T = {row['period']:.2f} s (largest power)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(plot_dir / "rm3_pto_stroke_time_series_largest_power.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
