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
sys.path.append(str(Path(__file__).parent.parent.parent / "verification" / "utilities"))
from compare_template_internal import HAS_MATPLOTLIB, plot_comparison, write_comparison_status
from verification_plot import create_verification_rao_plot

import numpy as np

TEST_NAME = "compare_radiation_methods_rm3_reg"
N_FIT_PERIODS = 10
AMP_TOL_PCT = 1.0
PHASE_TOL_DEG = 1.0
POWER_TOL_PCT = 2.0

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


def make_plots(base_dir, rows, series, curve, amp):
    """Standard RAO pages (constant coefficients vs coupled frequency domain,
    other methods overlaid) and a standard time-series page of the PTO stroke."""
    plot_dir = base_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    for old in plot_dir.glob("*.png"):
        old.unlink()
    checked = "constant_coefficients"
    rows_cc = sorted((r for r in rows if checked in r), key=lambda r: r["omega"])
    if not rows_cc:
        return
    w = np.array([r["omega"] for r in rows_cc])
    periods = 2 * np.pi / w
    t_lo, t_hi = 0.9 * periods.min(), 1.1 * periods.max()
    curve = sorted((c for c in curve if t_lo <= c["period"] <= t_hi), key=lambda c: c["omega"])
    w_curve = np.array([c["omega"] for c in curve])

    # (file stem, title, y label, reference value, uncoupled value, simulated value,
    #  pass tolerance %, legend location clear of the data, quantity name if not an RAO)
    pages = [
        ("rm3_float_heave_rao", "RM3 Float Heave RAO", "Float heave RAO (m/m)",
         lambda r: r["z_ref"][0] / amp, lambda r: r["z_unc"][0] / amp, lambda d: d["z"][0] / amp,
         AMP_TOL_PCT, "lower right", None),
        ("rm3_plate_heave_rao", "RM3 Plate Heave RAO", "Plate heave RAO (m/m)",
         lambda r: r["z_ref"][1] / amp, lambda r: r["z_unc"][1] / amp, lambda d: d["z"][1] / amp,
         AMP_TOL_PCT, "upper left", None),
        ("rm3_mean_pto_power", "RM3 Mean PTO Power", "Mean PTO power (kW)",
         lambda r: r["p_ref"] / 1e3, lambda r: r["p_unc"] / 1e3, lambda d: d["p"] / 1e3,
         POWER_TOL_PCT, "upper right", "Mean PTO Power"),
    ]
    for stem, title, y_label, f_ref, f_unc, f_sim, tol, legend_loc, quantity in pages:
        v_ref = np.array([f_ref(r) for r in rows_cc])
        v_cc = np.array([f_sim(r[checked]) for r in rows_cc])
        is_complex = np.iscomplexobj(v_ref)
        extra = {
            "Coupled FD, all BEM points":
                (w_curve, np.abs([f_ref(c) for c in curve]), None),
            "FD without body coupling":
                (w_curve, np.abs([f_unc(c) for c in curve]), None),
        }
        for m in METHODS:
            pts = sorted((r for r in rows if m["tag"] in r), key=lambda r: r["omega"])
            if m["tag"] != checked and pts:
                extra[m["label"]] = (np.array([r["omega"] for r in pts]),
                                     np.abs([f_sim(r[m["tag"]]) for r in pts]), None)
        create_verification_rao_plot(
            w, np.abs(v_ref), np.angle(v_ref) if is_complex else None,
            w, np.abs(v_cc), np.angle(v_cc) if is_complex else None,
            title=f"{title} — Radiation Methods vs Coupled Frequency Domain",
            amp_label=y_label,
            output_path=str(plot_dir / f"{stem}.png"),
            ref_label="Coupled FD, simulated periods",
            sim_label="Constant coefficients",
            amp_errors=np.abs(np.abs(v_cc) / np.abs(v_ref) - 1.0),
            phase_errors=np.abs(np.angle(v_cc / v_ref)) if is_complex else None,
            ref_file_path=str(base_dir / "reference.txt"),
            test_file_path=str(base_dir),
            extra_sources=extra,
            error_thresholds=(tol / 100.0, 2.0 * tol / 100.0),
            # The power curves cross every corner of the panel, so that legend
            # sits in the gap under the panel. The RAO panels have an empty corner.
            legend_kwargs=(dict(loc="upper center", bbox_to_anchor=(0.5, -0.08),
                                ncol=3, fontsize=8) if quantity else
                           dict(loc=legend_loc, ncol=2, fontsize=8)),
            quantity=quantity,
            connect_points=False,
            show_extra_errors=True,
        )

    row = rows_cc[int(np.argmax([r["p_ref"] for r in rows_cc]))]
    if (row["k"], checked) in series:
        t, s = series[(row["k"], checked)]
        sel = t >= t[-1] - 4 * row["period"]
        s_ss = np.real((row["z_ref"][0] - row["z_ref"][1]) * np.exp(1j * row["omega"] * t[sel]))
        plot_comparison(
            t[sel], s_ss, s[sel],
            label_a="Coupled frequency domain (steady state)",
            label_b="Constant coefficients",
            title=f"RM3 PTO Stroke, Last 4 Periods at T = {row['period']:.2f} s (Largest Power)",
            y_label="Relative heave float - plate (m)",
            output_path=str(plot_dir / "rm3_pto_stroke_time_series_largest_power.png"),
            test_name_full=TEST_NAME,
        )


if __name__ == "__main__":
    main()
