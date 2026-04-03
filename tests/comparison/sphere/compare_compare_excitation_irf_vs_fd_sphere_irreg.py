#!/usr/bin/env python3
"""
Comparison script: kPolar vs kCartesian excitation interpolation
on sphere irregular waves.

This is an informational comparison (no hard pass/fail threshold).
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import run_internal_comparison

def main():
    if len(sys.argv) != 2:
        print("Usage: compare_compare_excitation_irf_vs_fd_sphere_irreg.py <results_base_path>")
        sys.exit(1)

    base_dir = Path(sys.argv[1])
    file_a = base_dir / "polar.txt"
    file_b = base_dir / "cartesian.txt"

    if not file_a.exists() or not file_b.exists():
        print(f"Missing result files: {file_a} / {file_b}")
        sys.exit(1)

    l2, linf, _ = run_internal_comparison(
        str(file_a), str(file_b),
        test_name="compare_excitation_irf_vs_fd_sphere_irreg",
        label_a="kPolar (legacy)",
        label_b="kCartesian (improved)",
        y_label="Heave (m)",
        skip_time=60.0,
    )

    print(f"\nComparison complete: L2={l2:.4e}, Linf={linf:.4e}")

if __name__ == '__main__':
    main()
