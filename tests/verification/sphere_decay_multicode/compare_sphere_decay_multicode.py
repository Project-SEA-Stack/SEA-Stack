#!/usr/bin/env python3
"""
Sphere Decay Multi-Code Verification -- compare SEA-Stack heave decay against
multiple reference codes (ProteusDS, InWave, Marin, NREL CFD, WavEC).

Usage (called automatically by CTest):
    python compare_sphere_decay_multicode.py <normalized_dir> <results_file>
"""

import sys
import os
from pathlib import Path

# Import compare_template before numpy so missing deps exit 77 (CTest skip), not 1.
sys.path.append(os.path.join(os.path.dirname(__file__), '../../regression/utilities'))
from compare_template import create_comparison_plot, write_status_file

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../../../data/verification'))
from normalize_utils import load_and_normalize

# Two-tier cross-code tolerances: PASS (tight) / WARN (loose) / FAIL (beyond loose).
MULTICODE_TOL_PASS   = (5e-2, 1.0)
MULTICODE_TOL_WARN   = (1e-1, 3.0)     # generous for nonlinear codes like Marin

# Source key (directory name) -> display label for plots
CODE_DISPLAY_NAMES = {
    'proteusds': 'ProteusDS',
    'inwave': 'InWave',
    'marin': 'Marin',
    'wavec': 'Wavec',
    'nrel_cfd': 'NREL_CFD',
}

REFERENCE_SOURCES = [
    ('proteusds', 'heave_decay.txt', MULTICODE_TOL_PASS, MULTICODE_TOL_WARN),
    ('inwave',    'heave_decay.txt', MULTICODE_TOL_PASS, MULTICODE_TOL_WARN),
    ('marin',     'heave_decay.txt', MULTICODE_TOL_PASS, MULTICODE_TOL_WARN),
    ('wavec',     'heave_decay.txt', MULTICODE_TOL_PASS, MULTICODE_TOL_WARN),
    ('nrel_cfd',  'heave_decay.txt', MULTICODE_TOL_PASS, MULTICODE_TOL_WARN),
]


def load_seastack(results_file):
    """Load SEA-Stack decay output and convert to displacement from equilibrium.

    SEA-Stack outputs absolute CG z-position; reference data is displacement
    from equilibrium.  Estimate equilibrium from the tail of the signal (same
    method as normalize.py uses for multicode raw exports under
    data/verification/sphere_decay_multicode/raw/hydrochrono/ — not the core
    regression `ss_ref_*.txt` pipeline).
    """
    data = np.loadtxt(results_file, skiprows=1)
    t, z = data[:, 0], data[:, 1]
    n_tail = max(1, len(z) // 10)
    eq_z = np.mean(z[-n_tail:])
    print(f"  SEA-Stack equilibrium estimate: {eq_z:.4f} m")
    return t, z - eq_z


def load_reference(ref_file):
    """Load normalized reference data via the normalization layer."""
    t, cols, meta = load_and_normalize(ref_file)
    assert meta['units'].startswith('s,'), f"Unexpected time unit in {ref_file}"
    return t, cols[:, 0]


def resample_to_common_grid(t_ref, y_ref, t_test, y_test):
    """Interpolate both signals onto a common time grid."""
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

    worst_status = "PASS"
    all_metrics = {}
    overlay_sources = {'SEA-Stack': (t_test, y_test)}

    for source_name, filename, tol_pass, tol_warn in REFERENCE_SOURCES:
        ref_file = normalized_dir / source_name / filename
        if not ref_file.exists():
            print(f"  {source_name}: reference file not found, skipping")
            continue

        t_ref, y_ref = load_reference(ref_file)
        print(f"  {source_name}: {len(t_ref)} points, t=[{t_ref[0]:.2f}, {t_ref[-1]:.2f}]")

        overlay_sources[source_name] = (t_ref, y_ref)

        t_common, y_ref_i, y_test_i = resample_to_common_grid(t_ref, y_ref, t_test, y_test)
        if t_common is None:
            print(f"  {source_name}: no time overlap, skipping")
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
        print(f"  {source_name}: L2={l2:.2e} (pass {tol_pass[0]:.0e} / warn {tol_warn[0]:.0e}), "
              f"Linf={linf:.2e} (pass {tol_pass[1]:.0e} / warn {tol_warn[1]:.0e}) -> {status_str}")

        if status_str == "FAIL":
            worst_status = "FAIL"
        elif status_str == "WARN" and worst_status != "FAIL":
            worst_status = "WARN"

        all_metrics[f"{source_name}_l2"] = float(l2)
        all_metrics[f"{source_name}_linf"] = float(linf)

    # Use first available reference as primary; others as extra overlays
    ref_sources = [(s[0], s[1]) for s in REFERENCE_SOURCES if s[0] in overlay_sources]
    if ref_sources:
        primary_source = ref_sources[0][0]
        t_ref, y_ref = overlay_sources[primary_source]
        t_c, y_ref_i, y_test_i = resample_to_common_grid(t_ref, y_ref, t_test, y_test)
        if t_c is not None:
            ref_data  = np.column_stack([t_c, y_ref_i])
            test_data = np.column_stack([t_c, y_test_i])
            extra = {}
            for name, (t_src, y_src) in overlay_sources.items():
                if name in ('SEA-Stack', primary_source):
                    continue
                tc, yi, _ = resample_to_common_grid(t_src, y_src, t_test, y_test)
                if tc is not None:
                    display = CODE_DISPLAY_NAMES.get(name, name.replace('_', ' ').title())
                    extra[display] = np.column_stack([tc, yi])
            try:
                create_comparison_plot(
                    ref_data, test_data,
                    test_name="Sphere Decay Verification - Multicode",
                    output_dir=str(output_dir),
                    ref_file_path=str(normalized_dir),
                    test_file_path=str(results_file.parent),
                    y_label="Heave displacement from equilibrium (m)",
                    ref_label=CODE_DISPLAY_NAMES.get(primary_source, primary_source.replace('_', ' ').title()),
                    extra_sources=extra if extra else None,
                )
            except Exception as e:
                print(f"  Warning: comparison plot failed: {e}")

    write_status_file(str(results_file.parent), "sphere_decay_multicode", worst_status, all_metrics)

    print(f"\nOverall: {worst_status}")
    sys.exit(1 if worst_status == "FAIL" else 0)


if __name__ == '__main__':
    main()
