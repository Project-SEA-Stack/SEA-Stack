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
sys.path.append(str(Path(__file__).parent.parent.parent / "verification" / "utilities"))
from compare_template_internal import HAS_MATPLOTLIB, plot_comparison, write_comparison_status
from verification_plot import create_verification_rao_plot

import numpy as np

TEST_NAME = "compare_radiation_methods_sphere_reg"
N_FIT_PERIODS = 10
AMP_TOL_PCT = 1.0
PHASE_TOL_DEG = 1.0

METHODS = [
    {"tag": "constant_coefficients", "label": "Constant coefficients"},
    {"tag": "rirf", "label": "RIRF convolution"},
    {"tag": "state_space", "label": "State space"},
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
    """Standard RAO page (constant coefficients vs frequency domain, other
    methods overlaid) and standard time-series page at the largest response."""
    plot_dir = base_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    for old in plot_dir.glob("*.png"):
        old.unlink()
    rows = sorted(rows, key=lambda r: r["omega"])
    checked = "constant_coefficients"
    rows_cc = [r for r in rows if checked in r]
    if not rows_cc:
        return
    w = np.array([r["omega"] for r in rows_cc])
    z_ref = np.array([r["z_ref"] for r in rows_cc])
    z_cc = np.array([r[checked]["z"] for r in rows_cc])

    periods = 2 * np.pi / w
    t_lo, t_hi = 0.9 * periods.min(), 1.1 * periods.max()
    curve = sorted(c for c in curve if t_lo <= c[0] <= t_hi)
    extra = {"Frequency domain (all BEM grid points)": (
        2 * np.pi / np.array([c[0] for c in curve]),
        np.abs([c[1] for c in curve]) / amp, None)}
    for m in METHODS:
        pts = [r for r in rows if m["tag"] in r]
        if m["tag"] != checked and pts:
            z = np.array([r[m["tag"]]["z"] for r in pts])
            extra[m["label"]] = (np.array([r["omega"] for r in pts]), np.abs(z) / amp, np.angle(z))

    create_verification_rao_plot(
        w, np.abs(z_ref) / amp, np.angle(z_ref), w, np.abs(z_cc) / amp, np.angle(z_cc),
        title="Sphere Heave RAO with PTO Damper — Radiation Methods vs Frequency Domain",
        amp_label="Heave RAO (m/m)",
        output_path=str(plot_dir / "heave_rao_vs_period.png"),
        ref_label="Frequency domain (at simulated periods)",
        sim_label="Constant coefficients",
        amp_errors=np.abs(np.abs(z_cc) / np.abs(z_ref) - 1.0),
        phase_errors=np.abs(np.angle(z_cc / z_ref)),
        ref_file_path=str(base_dir / "reference.txt"),
        test_file_path=str(base_dir),
        extra_sources=extra,
        error_thresholds=(AMP_TOL_PCT / 100.0, 2.0 * AMP_TOL_PCT / 100.0),
        legend_kwargs=dict(loc="lower right", ncol=2),
        connect_points=False,
        show_extra_errors=True,
    )

    row = rows_cc[int(np.argmax(np.abs(z_ref)))]
    if (row["k"], checked) in series:
        t, z = series[(row["k"], checked)]
        sel = t >= t[-1] - 4 * row["period"]
        z_ss = np.real(row["z_ref"] * np.exp(1j * row["omega"] * t[sel]))
        plot_comparison(
            t[sel], z_ss, z[sel],
            label_a="Frequency domain (steady state)",
            label_b="Constant coefficients",
            title=f"Sphere Heave, Last 4 Periods at T = {row['period']:.2f} s (Largest Response)",
            y_label="Heave from equilibrium (m)",
            output_path=str(plot_dir / "heave_time_series_largest_response.png"),
            test_name_full=TEST_NAME,
        )


if __name__ == "__main__":
    main()
