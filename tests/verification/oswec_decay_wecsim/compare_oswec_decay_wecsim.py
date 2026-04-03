#!/usr/bin/env python3
"""
OSWEC Decay WEC-Sim Verification -- compare SEA-Stack pitch decay against
WEC-Sim reference data.

Usage (called automatically by CTest):
    python compare_oswec_decay_wecsim.py <normalized_dir> <results_file>
"""

import sys
import os
from pathlib import Path

sys.path.append(os.path.join(os.path.dirname(__file__), '../../regression/utilities'))
from compare_template import create_comparison_plot, write_status_file

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../../../data/verification'))
from normalize_utils import load_and_normalize

# Two-tier cross-code tolerances: PASS (tight) / WARN (loose) / FAIL (beyond loose).
PITCH_TOL_PASS = (1e-2, 0.2)   # (L2, Linf) in radians
PITCH_TOL_WARN = (5e-2, 0.5)


def load_seastack(results_file):
    data = np.loadtxt(results_file, skiprows=1)
    return data[:, 0], data[:, 1]


def load_reference(ref_file):
    t, cols, meta = load_and_normalize(ref_file)
    assert meta['units'].startswith('s,'), f"Unexpected time unit in {ref_file}"
    return t, cols[:, 0]


def resample_to_common_grid(t_ref, y_ref, t_test, y_test):
    t_min = max(t_ref[0], t_test[0])
    t_max = min(t_ref[-1], t_test[-1])
    if t_max <= t_min:
        return None, None, None
    n_pts = min(len(t_ref), len(t_test))
    t_common = np.linspace(t_min, t_max, n_pts)
    y_ref_i = np.interp(t_common, t_ref, y_ref)
    y_test_i = np.interp(t_common, t_test, y_test)
    return t_common, y_ref_i, y_test_i


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <normalized_dir> <results_file>")
        sys.exit(1)

    normalized_dir = Path(sys.argv[1])
    results_file = Path(sys.argv[2])
    output_dir = results_file.parent / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    t_test, y_test = load_seastack(results_file)
    print(f"SEA-Stack data: {len(t_test)} points, t=[{t_test[0]:.2f}, {t_test[-1]:.2f}]")

    ref_file = normalized_dir / "wecsim" / "pitch_decay.txt"
    if not ref_file.exists():
        print(f"ERROR: Reference file not found at {ref_file}")
        write_status_file(str(results_file.parent), "oswec_decay_wecsim", "FAIL",
                          {"error": "reference_not_found"})
        sys.exit(1)

    t_ref, y_ref = load_reference(ref_file)
    print(f"WEC-Sim reference: {len(t_ref)} points, t=[{t_ref[0]:.2f}, {t_ref[-1]:.2f}]")

    t_common, y_ref_i, y_test_i = resample_to_common_grid(t_ref, y_ref, t_test, y_test)
    if t_common is None:
        print("ERROR: no time overlap between reference and test")
        write_status_file(str(results_file.parent), "oswec_decay_wecsim", "FAIL",
                          {"error": "no_time_overlap"})
        sys.exit(1)

    err = y_ref_i - y_test_i
    l2 = np.linalg.norm(err) / len(err)
    linf = np.linalg.norm(err, np.inf)

    if l2 <= PITCH_TOL_PASS[0] and linf <= PITCH_TOL_PASS[1]:
        status = "PASS"
    elif l2 <= PITCH_TOL_WARN[0] and linf <= PITCH_TOL_WARN[1]:
        status = "WARN"
    else:
        status = "FAIL"

    metrics = {
        "l2_norm": float(l2),
        "linf_norm": float(linf),
        "tol_pass_l2": PITCH_TOL_PASS[0],
        "tol_warn_l2": PITCH_TOL_WARN[0],
    }

    print(f"Pitch decay: L2={l2:.2e} (pass {PITCH_TOL_PASS[0]:.0e} / warn {PITCH_TOL_WARN[0]:.0e}), "
          f"Linf={linf:.2e} (pass {PITCH_TOL_PASS[1]:.0e} / warn {PITCH_TOL_WARN[1]:.0e}) -> {status}")

    ref_data  = np.column_stack([t_common, y_ref_i])
    test_data = np.column_stack([t_common, y_test_i])
    try:
        create_comparison_plot(
            ref_data, test_data,
            test_name="OSWEC Decay Verification - Pitch",
            output_dir=str(output_dir),
            ref_file_path=str(ref_file),
            test_file_path=str(results_file),
            y_label="Pitch (rad)",
            ref_label="WEC-Sim",
        )
    except Exception as e:
        print(f"Warning: comparison plot generation failed: {e}")

    write_status_file(str(results_file.parent), "oswec_decay_wecsim", status, metrics)

    print(f"\nOverall: {status}")
    sys.exit(1 if status == "FAIL" else 0)


if __name__ == '__main__':
    main()
