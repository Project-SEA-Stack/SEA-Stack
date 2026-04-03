#!/usr/bin/env python3
"""
Comparison script: linear vs nonlinear hydrostatics on sphere free decay.

Compares heave displacement from equilibrium between linear and nonlinear
hydrostatic methods for 1 m and 5 m drop amplitudes. Generates two plots only.
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import (
    compute_norms,
    plot_comparison,
    write_comparison_status,
)

import numpy as np

TEST_NAME = "compare_linear_vs_nonlinear_hs_sphere_decay"

AMPLITUDES = [
    {"tag": "amp1m", "title": "1 m drop"},
    {"tag": "amp5m", "title": "5 m drop"},
]


def load_signal(path):
    """Load a 2-column text file (time, signal), skipping comment lines."""
    data = np.loadtxt(path, comments="#")
    return data[:, 0], data[:, 1]


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)

    base_dir = Path(sys.argv[1])
    output_dir = base_dir

    all_metrics = {}
    for amp in AMPLITUDES:
        tag = amp["tag"]
        file_a = base_dir / f"{tag}_linear_heave.txt"
        file_b = base_dir / f"{tag}_nonlinear_heave.txt"

        if not file_a.exists() or not file_b.exists():
            print(f"Missing result files for {tag}: {file_a} / {file_b}")
            continue

        time_a, signal_a = load_signal(str(file_a))
        time_b, signal_b = load_signal(str(file_b))

        min_len = min(len(time_a), len(time_b))
        time_a, signal_a = time_a[:min_len], signal_a[:min_len]
        time_b, signal_b = time_b[:min_len], signal_b[:min_len]

        l2, linf = compute_norms(signal_a, signal_b, skip_time=0.0)
        all_metrics[f"{tag}_heave_l2"] = float(l2)
        all_metrics[f"{tag}_heave_linf"] = float(linf)

        plot_path = output_dir / "plots" / f"{tag}_heave_comparison.png"
        plot_comparison(
            time_a,
            signal_a,
            signal_b,
            label_a="Linear",
            label_b="Nonlinear",
            title=f"{TEST_NAME}: Heave ({amp['title']})",
            y_label="Heave displacement from equilibrium (m)",
            output_path=plot_path,
        )

        print(f"  {tag}: L2={l2:.4e}, Linf={linf:.4e}")

    write_comparison_status(str(output_dir), TEST_NAME, all_metrics, passed=None)
    print(f"\nComparison complete. Status written to {output_dir}")


if __name__ == "__main__":
    main()
