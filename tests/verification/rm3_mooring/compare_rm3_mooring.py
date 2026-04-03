#!/usr/bin/env python3
"""
RM3 Mooring Verification Test -- compare SEA-Stack/MoorDyn results against
WEC-Sim/MoorDyn co-simulation reference data.

Usage (called automatically by CTest):
    python compare_rm3_mooring.py <reference_dir> <results_file>

    reference_dir:  data/verification/rm3_mooring/normalized/
    results_file:   results/tests/rm3_mooring/results_rm3_mooring.txt
"""

import sys
import os
from pathlib import Path

sys.path.append(os.path.join(os.path.dirname(__file__), '../../regression/utilities'))
from compare_template import create_comparison_plot, write_status_file

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../utilities'))
from multi_source_plot import plot_multi_source_overlay

sys.path.append(os.path.join(os.path.dirname(__file__), '../../../data/verification'))
from normalize_utils import load_and_normalize


# Two-tier cross-code tolerances: PASS (tight) / WARN (loose) / FAIL (beyond loose).
BODY_MOTION_TOL_PASS  = (1e-3, 0.15)     # (L2, L-inf) for body heave [m]
BODY_MOTION_TOL_WARN  = (5e-3, 0.5)
FAIRLEAD_TOL_PASS     = (5e+1, 5e+3)     # (L2, L-inf) for fairlead tension [N]
FAIRLEAD_TOL_WARN     = (2e+2, 2e+4)
ETA_TOL_PASS          = (1e-3, 0.05)     # (L2, L-inf) for wave elevation [m]
ETA_TOL_WARN          = (5e-3, 0.20)

RAMP_SKIP_TIME = 40.0


def load_reference(ref_dir):
    """Load all WEC-Sim/MoorDyn reference data via the normalization layer."""
    ref_dir = Path(ref_dir)

    body_file    = ref_dir / "wecsim_moordyn_body_motions.txt"
    tension_file = ref_dir / "wecsim_moordyn_fairlead_tensions.txt"

    t_body, body_cols, meta_b = load_and_normalize(body_file)
    assert meta_b['units'].startswith('s,'), f"Unexpected units in {body_file}"
    body_data = np.column_stack([t_body, body_cols])

    t_ten, ten_cols, meta_t = load_and_normalize(tension_file)
    assert meta_t['units'].startswith('s,'), f"Unexpected units in {tension_file}"
    tension_data = np.column_stack([t_ten, ten_cols])

    return body_data, tension_data


def load_reference_eta(ref_dir):
    """Load WEC-Sim reference wave elevation via the normalization layer."""
    ref_dir = Path(ref_dir)
    eta_file = ref_dir / "wecsim_moordyn_wave_elevation.txt"
    if not eta_file.exists():
        return None
    t, cols, meta = load_and_normalize(eta_file)
    return np.column_stack([t, cols])


def load_moordyn_lines_out(search_dirs):
    """Try to load MoorDyn's fairlead tension output from multiple candidate locations.

    MoorDyn writes output next to its input file with the same basename
    (e.g., lines_rm3.out for lines_rm3.txt).
    """
    patterns = ["lines_rm3.out", "lines.out"]
    for d in search_dirs:
        for name in patterns:
            for candidate in [d / name,
                              d / "Mooring" / name,
                              d / "mooring" / name]:
                if candidate.exists():
                    print(f"  Found MoorDyn output: {candidate}")
                    return np.loadtxt(str(candidate), skiprows=1)
    return None


def deduplicate_time(time, signal):
    """Remove duplicate time entries, keeping the first value for each time."""
    _, unique_idx = np.unique(time, return_index=True)
    return time[unique_idx], signal[unique_idx]


def resample_to_common_grid(ref_time, ref_signal, test_time, test_signal,
                            skip_time=0.0):
    """Interpolate both signals onto a common uniform time grid.

    If skip_time > 0, the comparison starts after that many seconds
    (to exclude ramp / initial transient).
    """
    t_start = max(ref_time[0], test_time[0], skip_time)
    t_end   = min(ref_time[-1], test_time[-1])
    dt      = max(ref_time[1] - ref_time[0], test_time[1] - test_time[0])
    n_pts   = int(np.round((t_end - t_start) / dt)) + 1
    t_common = np.linspace(t_start, t_end, n_pts)

    ref_interp  = np.interp(t_common, ref_time, ref_signal)
    test_interp = np.interp(t_common, test_time, test_signal)
    print(f"Resampled to common grid [{t_start:.2f}, {t_end:.2f}]s  "
          f"(dt={dt:.4f}, {n_pts} pts)")
    return t_common, ref_interp, test_interp


def compare_signal(ref_time, ref_signal, test_time, test_signal, label, units,
                   tol_pass, tol_warn, plots_dir, ref_file_label, test_file_label,
                   skip_time=0.0, report_ready=True):
    """Compare a single time-series signal and generate a plot.

    Uses create_comparison_plot from the regression template so that the
    plotted window, error metrics, and status all refer to the same data.

    When report_ready is False the plot is saved with an '_overlay' suffix
    so that the verification report generator excludes it automatically.

    Returns (l2, linf, status_str).
    """
    ref_time, ref_signal   = deduplicate_time(ref_time, ref_signal)
    test_time, test_signal = deduplicate_time(test_time, test_signal)

    t_common, ref_interp, test_interp = resample_to_common_grid(
        ref_time, ref_signal, test_time, test_signal, skip_time=skip_time)

    ref_data  = np.column_stack([t_common, ref_interp])
    test_data = np.column_stack([t_common, test_interp])

    suffix = 'comparison' if report_ready else 'overlay'
    n1, n2 = create_comparison_plot(
        ref_data, test_data,
        test_name=f"RM3 Mooring Verification - {label}",
        output_dir=str(plots_dir),
        ref_file_path=ref_file_label,
        test_file_path=test_file_label,
        y_label=f"{label} ({units})",
        output_suffix=suffix,
        ref_label="WEC-Sim",
    )

    if n1 <= tol_pass[0] and n2 <= tol_pass[1]:
        tag = "PASS"
    elif n1 <= tol_warn[0] and n2 <= tol_warn[1]:
        tag = "WARN"
    else:
        tag = "FAIL"
    print(f"  {label}: L2={n1:.2e} (pass {tol_pass[0]:.0e} / warn {tol_warn[0]:.0e}), "
          f"Linf={n2:.2e} (pass {tol_pass[1]:.0e} / warn {tol_warn[1]:.0e}) -> {tag}")

    return n1, n2, tag


def main():
    if len(sys.argv) != 3:
        print("Usage: compare_rm3_mooring.py <reference_dir> <results_file>")
        sys.exit(1)

    ref_dir      = Path(sys.argv[1])
    results_file = Path(sys.argv[2])

    print(f"Reference dir: {ref_dir}")
    print(f"Results file:  {results_file}")

    # Load reference
    ref_body, ref_tension = load_reference(ref_dir)
    ref_time = ref_body[:, 0]

    # Load SEA-Stack results
    ss_data = np.loadtxt(str(results_file), skiprows=1)
    ss_time = ss_data[:, 0]

    plots_dir = results_file.parent / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    for stale in plots_dir.glob("*.png"):
        stale.unlink()

    worst_status = "PASS"
    all_metrics = {}

    def _update_worst(current, new_status):
        if new_status == "FAIL":
            return "FAIL"
        if new_status == "WARN" and current != "FAIL":
            return "WARN"
        return current

    # ── Body motions ─────────────────────────────────────────────────────
    print("\n--- Body motion comparison (SEA-Stack vs WEC-Sim) ---")

    for col_idx, label, rpt_ready in [(1, "Float Heave Z", True),
                                       (2, "Plate Heave Z", False)]:
        n1, n2, sig_status = compare_signal(
            ref_time, ref_body[:, col_idx],
            ss_time, ss_data[:, col_idx],
            label, "m", BODY_MOTION_TOL_PASS, BODY_MOTION_TOL_WARN, plots_dir,
            f"wecsim_moordyn_body_motions.txt col {col_idx}",
            str(results_file.name),
            skip_time=RAMP_SKIP_TIME,
            report_ready=rpt_ready,
        )
        sname = label.lower().replace(" ", "_")
        all_metrics[sname] = {"l2_norm": n1, "linf_norm": n2}
        worst_status = _update_worst(worst_status, sig_status)

    # ── Fairlead tensions (MoorDyn lines.out) ────────────────────────────
    # MoorDyn writes output next to its input file. Search several likely locations.
    build_dir = Path(os.environ.get("SEASTACK_BUILD_DIR", ""))
    moordyn_data_dir = build_dir / "data" / "demos" / "rm3" / "mooring" if build_dir.is_dir() else Path()

    search_dirs = [
        moordyn_data_dir,
        results_file.parent,
        results_file.parent.parent.parent,
        Path.cwd(),
    ]
    ss_tension = load_moordyn_lines_out(search_dirs)

    if ss_tension is not None:
        print("\n--- Fairlead tension comparison (SEA-Stack/MoorDyn vs WEC-Sim/MoorDyn) ---")
        ss_t_time = ss_tension[:, 0]

        for col_idx, label, rpt_ready in [(1, "FairTen4", True),
                                           (2, "FairTen5", False),
                                           (3, "FairTen6", False)]:
            n1, n2, sig_status = compare_signal(
                ref_tension[:, 0], ref_tension[:, col_idx],
                ss_t_time, ss_tension[:, col_idx],
                label, "N", FAIRLEAD_TOL_PASS, FAIRLEAD_TOL_WARN, plots_dir,
                f"wecsim_moordyn_fairlead_tensions.txt col {col_idx}",
                "lines.out",
                skip_time=RAMP_SKIP_TIME,
                report_ready=rpt_ready,
            )
            sname = label.lower()
            all_metrics[sname] = {"l2_norm": n1, "linf_norm": n2}
            worst_status = _update_worst(worst_status, sig_status)
    else:
        print("\nWARNING: MoorDyn lines.out not found -- skipping fairlead tension comparison")

    # ── Wave elevation diagnostic ────────────────────────────────────────
    eta_status = None
    eta_l2 = eta_linf = float('nan')
    ref_eta = load_reference_eta(ref_dir)
    ss_eta_file = results_file.parent / (results_file.stem + "_eta.txt")

    if ref_eta is not None and ss_eta_file.exists():
        print("\n--- Wave elevation diagnostic (SEA-Stack reconstructed vs WEC-Sim) ---")
        ss_eta = np.loadtxt(str(ss_eta_file), skiprows=1)
        eta_l2, eta_linf, eta_status = compare_signal(
            ref_eta[:, 0], ref_eta[:, 1],
            ss_eta[:, 0], ss_eta[:, 1],
            "Wave Elevation", "m",
            ETA_TOL_PASS, ETA_TOL_WARN, plots_dir,
            "wecsim_moordyn_wave_elevation.txt",
            ss_eta_file.name,
            skip_time=RAMP_SKIP_TIME,
            report_ready=False,
        )
        all_metrics["wave_elevation"] = {"l2_norm": eta_l2, "linf_norm": eta_linf}

        # Amplitude envelope diagnostic: compare RMS in two time windows
        for t_lo, t_hi, label in [(40, 50, "post-ramp"), (140, 170, "late")]:
            ref_mask = (ref_eta[:, 0] >= t_lo) & (ref_eta[:, 0] <= t_hi)
            ss_mask = (ss_eta[:, 0] >= t_lo) & (ss_eta[:, 0] <= t_hi)
            if ref_mask.any() and ss_mask.any():
                ref_rms = np.std(ref_eta[ref_mask, 1])
                ss_rms = np.std(ss_eta[ss_mask, 1])
                ratio = ss_rms / ref_rms if ref_rms > 1e-10 else float('nan')
                print(f"  Eta RMS [{t_lo}-{t_hi}s] ({label}): "
                      f"ref={ref_rms:.4f} m, SS={ss_rms:.4f} m, ratio={ratio:.2f}")
                all_metrics[f"eta_rms_ratio_{label}"] = float(ratio)

        try:
            plot_multi_source_overlay(
                {
                    'SEA-Stack': (ss_eta[:, 0], ss_eta[:, 1]),
                    'WEC-Sim (reference)': (ref_eta[:, 0], ref_eta[:, 1]),
                },
                title="RM3 Mooring -- Wave Elevation",
                y_label="Elevation (m)",
                output_path=str(plots_dir / "rm3_mooring_wave_elevation_overlay.png"),
                baseline_key="WEC-Sim (reference)",
                time_range=(RAMP_SKIP_TIME, None),
            )
        except Exception as e:
            print(f"  Warning: eta overlay plot failed: {e}")
    else:
        print("\nINFO: Wave elevation diagnostic skipped (files not available)")

    # ── Diagnostic summary ────────────────────────────────────────────────
    print("\n" + "=" * 50)
    print("  RM3 MOORING DIAGNOSTIC SUMMARY")
    print("=" * 50)

    diag_rows = []
    for key, label in [("wave_elevation", "Wave elevation"),
                       ("float_heave_z", "Float heave"),
                       ("plate_heave_z", "Plate heave"),
                       ("fairten4", "FairTen4"),
                       ("fairten5", "FairTen5"),
                       ("fairten6", "FairTen6")]:
        m = all_metrics.get(key)
        if m:
            l2 = m["l2_norm"]
            li = m["linf_norm"]
            diag_rows.append((label, l2, li))

    for label, l2, li in diag_rows:
        print(f"  {label:20s}  L2={l2:.2e}  Linf={li:.2e}")

    print()
    print("  Interpretation:")
    if eta_status == "FAIL":
        print("    -> Wave elevation FAIL: possible depth, phase, or interpolation mismatch")
        print("    -> All downstream errors (motions, tensions) may be consequences of wave mismatch")
    elif eta_status in ("PASS", "WARN"):
        if worst_status == "FAIL":
            print("    -> Waves match but motions/tensions FAIL: hydrodynamic or mooring differences")
        else:
            print("    -> All signals within tolerance")
    else:
        print("    -> Wave elevation not compared (file missing)")

    print("=" * 50)

    # ── Status ───────────────────────────────────────────────────────────
    note = None
    if eta_status == "FAIL":
        note = "Wave elevation mismatch; downstream errors may be consequences"
    write_status_file(str(results_file.parent), "rm3_mooring", worst_status, all_metrics, note=note)

    print(f"\n=== RM3 MOORING VERIFICATION: {worst_status} ===")

    if worst_status == "FAIL":
        sys.exit(1)


if __name__ == "__main__":
    main()
