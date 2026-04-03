#!/usr/bin/env python3
"""
F3OF Decay Multi-Code Verification -- compare SEA-Stack Decay C1/Decay C3 decay
results against multi-code reference data (INW, WSM, WDN, PDS).

Usage (called automatically by CTest):
    python compare_f3of_decay_multicode.py <normalized_dir> <results_file>

    results_file is the base path; per-decay files are at
    <results_dir>/results_f3of_decay_multicode_decay_c1.txt, _decay_c3.txt
"""

import sys
import os
from pathlib import Path

sys.path.append(os.path.join(os.path.dirname(__file__), '../../regression/utilities'))
from compare_template import create_comparison_plot, write_status_file

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../../../data/verification'))
from normalize_utils import load_and_normalize

# Two-tier cross-code tolerances: PASS / WARN / FAIL.
# F3OF uses relaxed thresholds: different reference codes produce different results,
# so we accept good visual agreement rather than tight metric match.
F3OF_TOL_PASS = (0.35, 5.0)   # L2, Linf — relaxed for multicode variation
F3OF_TOL_WARN = (0.6, 8.0)

# Decay tests and their signals for comparison (each decay can have multiple signals)
DT_CONFIGS = {
    'decay_c1': [
        {'primary_col': 1, 'primary_name': 'surge', 'primary_unit': 'm'},
    ],
    'decay_c3': [
        {'primary_col': 3, 'primary_name': 'flap_fore', 'primary_unit': 'rad'},
        {'primary_col': 4, 'primary_name': 'flap_aft', 'primary_unit': 'rad'},
    ],
}

# Reference source directories and display names for plots
CODE_DISPLAY_NAMES = {'inw': 'InWave', 'wsm': 'WEC-Sim', 'wdn': 'WaveDyn'}
REFERENCE_SOURCES = [
    ('inw', F3OF_TOL_PASS, F3OF_TOL_WARN),
    ('wsm', F3OF_TOL_PASS, F3OF_TOL_WARN),
    ('wdn', F3OF_TOL_PASS, F3OF_TOL_WARN),
]


def load_seastack_dt(filepath):
    """Load SEA-Stack DT output (Time, Surge, Pitch, FlapFore, FlapAft)."""
    data = np.loadtxt(filepath, skiprows=1)
    return data


def get_reference_dt_name(dt_name):
    """Map new decay_c names to legacy dt names for reference data files."""
    mapping = {'decay_c1': 'dt1', 'decay_c3': 'dt3'}
    return mapping.get(dt_name, dt_name)


def load_reference_signal(ref_dir, source, dt_name, signal_name):
    """Load a reference signal file via the normalization layer."""
    # Convert new decay_c names to legacy dt names for reference data files
    ref_dt_name = get_reference_dt_name(dt_name)
    filename = f"{ref_dt_name}_{signal_name}.txt"
    ref_file = ref_dir / source / filename
    if not ref_file.exists():
        return None, None
    try:
        t, cols, meta = load_and_normalize(ref_file)
        assert meta['units'].startswith('s,'), f"Unexpected time unit in {ref_file}"
        return t, cols[:, 0]
    except Exception as e:
        print(f"  Warning: normalization failed for {ref_file}: {e}")
        data = np.loadtxt(str(ref_file), comments='#')
        return data[:, 0], data[:, 1]


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
    results_dir = results_file.parent
    base_name = results_file.stem
    output_dir = results_dir / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    worst_status = "PASS"
    all_metrics = {}

    for dt_name, signal_configs in DT_CONFIGS.items():
        dt_file = results_dir / f"{base_name}_{dt_name}.txt"
        if not dt_file.exists():
            print(f"  {dt_name}: results file not found, skipping")
            continue

        data = load_seastack_dt(dt_file)
        t_test = data[:, 0]

        for dt_cfg in signal_configs:
            y_test = data[:, dt_cfg['primary_col']]
            signal_name = dt_cfg['primary_name']
            print(f"\n{dt_name.upper()} ({signal_name}): {len(t_test)} points, "
                  f"t=[{t_test[0]:.2f}, {t_test[-1]:.2f}]")

            overlay_sources = {'SEA-Stack': (t_test, y_test)}

            for source, tol_pass, tol_warn in REFERENCE_SOURCES:
                t_ref, y_ref = load_reference_signal(
                    normalized_dir, source, dt_name, signal_name)

                if t_ref is None:
                    print(f"  vs {source}: no reference file, skipping")
                    continue

                display_name = CODE_DISPLAY_NAMES.get(source, source)
                overlay_sources[display_name] = (t_ref, y_ref)

                t_common, y_ref_i, y_test_i = resample_to_common_grid(
                    t_ref, y_ref, t_test, y_test)
                if t_common is None:
                    print(f"  vs {source}: no time overlap, skipping")
                    continue

                err = y_ref_i - y_test_i
                l2 = np.linalg.norm(err) / len(err)
                linf = np.linalg.norm(err, np.inf)

                if l2 <= tol_pass[0] and linf <= tol_pass[1]:
                    status_str = "PASS"
                elif l2 <= tol_warn[0] and linf <= tol_warn[1]:
                    status_str = "WARN"
                else:
                    status_str = "FAIL"
                print(f"  vs {source}: L2={l2:.2e} (pass {tol_pass[0]:.0e} / warn {tol_warn[0]:.0e}), "
                      f"Linf={linf:.2e} (pass {tol_pass[1]:.0e} / warn {tol_warn[1]:.0e}) -> {status_str}")

                if status_str == "FAIL":
                    worst_status = "FAIL"
                elif status_str == "WARN" and worst_status != "FAIL":
                    worst_status = "WARN"

                key = f"{dt_name}_{signal_name}_{source}"
                all_metrics[f"{key}_l2"] = float(l2)
                all_metrics[f"{key}_linf"] = float(linf)

            # Use first available reference as primary; others as extra overlays
            ref_sources = [s for s in REFERENCE_SOURCES
                          if CODE_DISPLAY_NAMES.get(s[0], s[0]) in overlay_sources]
            if ref_sources:
                primary_display = CODE_DISPLAY_NAMES.get(ref_sources[0][0], ref_sources[0][0])
                t_ref, y_ref = overlay_sources[primary_display]
                t_c, y_ref_i, y_test_i = resample_to_common_grid(t_ref, y_ref, t_test, y_test)
                if t_c is not None:
                    ref_data  = np.column_stack([t_c, y_ref_i])
                    test_data = np.column_stack([t_c, y_test_i])
                    extra = {}
                    for name, (t_src, y_src) in overlay_sources.items():
                        if name in ('SEA-Stack', primary_display):
                            continue
                        tc, yi, _ = resample_to_common_grid(t_src, y_src, t_test, y_test)
                        if tc is not None:
                            extra[name] = np.column_stack([tc, yi])
                    try:
                        create_comparison_plot(
                            ref_data, test_data,
                            test_name=f"F3OF {dt_name.upper()} Verification - {signal_name}",
                            output_dir=str(output_dir),
                            ref_file_path=str(normalized_dir),
                            test_file_path=str(results_dir),
                            y_label=f"{signal_name} ({dt_cfg['primary_unit']})",
                            ref_label=primary_display,
                            extra_sources=extra if extra else None,
                        )
                    except Exception as e:
                        print(f"  Warning: comparison plot failed: {e}")

    write_status_file(str(results_dir), "f3of_decay_multicode", worst_status, all_metrics)

    print(f"\nOverall: {worst_status}")
    sys.exit(1 if worst_status == "FAIL" else 0)


if __name__ == '__main__':
    main()
