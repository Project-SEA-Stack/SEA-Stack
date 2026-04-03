#!/usr/bin/env python3
"""
Comparison script: DFT-based eta-import vs direct eta convolution
on the sphere irregular wave case.

Compares:
  1. Eta reconstruction (primary): DFT eta vs convolution eta (original)
  2. Body response (secondary): DFT heave vs convolution heave

This is a non-gating (INFO/WARN) comparison.  Thresholds are documented
as engineering guidelines but do not fail CI.
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import (
    compute_norms,
    write_comparison_status,
    plot_comparison,
)

import numpy as np

SKIP_TIME = 60.0
TEST_NAME = "compare_eta_dft_vs_convolution_sphere_irreg"

# Engineering guideline thresholds (non-gating).
# With the Fourier-frequency DFT fix, eta reconstruction is nearly exact
# (limited only by band-pass truncation), so these can be tight.
ETA_L2_GUIDELINE   = 0.01   # m
ETA_LINF_GUIDELINE = 0.10   # m
ETA_RMS_RATIO_MIN  = 0.95   # late/early ratio
HEAVE_L2_GUIDELINE   = 0.05  # m
HEAVE_LINF_GUIDELINE = 0.25  # m


def load_signal(path):
    """Load a two-column text file (time, value) with # comments."""
    data = np.loadtxt(str(path), comments='#')
    return data[:, 0], data[:, 1]


def rms_in_window(time, signal, t_lo, t_hi):
    mask = (time >= t_lo) & (time <= t_hi)
    if not mask.any():
        return float('nan')
    return float(np.std(signal[mask]))


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)

    base_dir = Path(sys.argv[1])
    conv_eta_file  = base_dir / "convolution_eta.txt"
    dft_eta_file   = base_dir / "dft_eta.txt"
    conv_heave_file = base_dir / "convolution.txt"
    dft_heave_file  = base_dir / "dft.txt"

    for f in [conv_eta_file, dft_eta_file, conv_heave_file, dft_heave_file]:
        if not f.exists():
            print(f"Missing result file: {f}")
            sys.exit(1)

    output_dir = base_dir
    all_metrics = {}

    # ── 1. Eta reconstruction comparison (primary) ───────────────────
    print("\n--- Eta reconstruction: convolution vs DFT ---")
    t_conv_eta, conv_eta = load_signal(str(conv_eta_file))
    t_dft_eta, dft_eta   = load_signal(str(dft_eta_file))

    min_len = min(len(t_conv_eta), len(t_dft_eta))
    t_conv_eta, conv_eta = t_conv_eta[:min_len], conv_eta[:min_len]
    t_dft_eta, dft_eta   = t_dft_eta[:min_len], dft_eta[:min_len]

    eta_l2, eta_linf = compute_norms(
        conv_eta, dft_eta, skip_time=SKIP_TIME, time=t_conv_eta)

    all_metrics["eta_l2"]   = float(eta_l2)
    all_metrics["eta_linf"] = float(eta_linf)

    eta_warn = (eta_l2 > ETA_L2_GUIDELINE or eta_linf > ETA_LINF_GUIDELINE)
    eta_tag = "WARN" if eta_warn else "OK"
    print(f"  Eta: L2={eta_l2:.2e} (guideline {ETA_L2_GUIDELINE}), "
          f"Linf={eta_linf:.2e} (guideline {ETA_LINF_GUIDELINE}) [{eta_tag}]")

    # RMS ratio: detect amplitude decay over time
    t_max = t_conv_eta[-1]
    early_lo, early_hi = SKIP_TIME, min(SKIP_TIME + 60.0, t_max)
    late_lo, late_hi   = max(t_max - 60.0, SKIP_TIME + 60.0), t_max

    rms_early = rms_in_window(t_dft_eta, dft_eta, early_lo, early_hi)
    rms_late  = rms_in_window(t_dft_eta, dft_eta, late_lo, late_hi)
    rms_ratio = rms_late / rms_early if rms_early > 1e-10 else float('nan')
    all_metrics["eta_rms_ratio_late_over_early"] = float(rms_ratio)

    decay_warn = rms_ratio < ETA_RMS_RATIO_MIN
    print(f"  Eta RMS ratio (late/early): {rms_ratio:.3f} "
          f"(guideline > {ETA_RMS_RATIO_MIN}) "
          f"[{'WARN -- amplitude decay' if decay_warn else 'OK'}]")

    plot_comparison(
        t_conv_eta, conv_eta, dft_eta,
        label_a="Convolution (original)", label_b="DFT (reconstructed)",
        title=f"{TEST_NAME}: Eta",
        y_label="Elevation (m)",
        output_path=str(output_dir / "plots" / "eta.png"),
        skip_time=SKIP_TIME,
    )

    # ── 2. Body response comparison (secondary) ─────────────────────
    print("\n--- Heave response: convolution vs DFT ---")
    t_conv_h, conv_h = load_signal(str(conv_heave_file))
    t_dft_h, dft_h   = load_signal(str(dft_heave_file))

    min_len = min(len(t_conv_h), len(t_dft_h))
    t_conv_h, conv_h = t_conv_h[:min_len], conv_h[:min_len]
    t_dft_h, dft_h   = t_dft_h[:min_len], dft_h[:min_len]

    heave_l2, heave_linf = compute_norms(
        conv_h, dft_h, skip_time=SKIP_TIME, time=t_conv_h)

    all_metrics["heave_l2"]   = float(heave_l2)
    all_metrics["heave_linf"] = float(heave_linf)

    heave_warn = (heave_l2 > HEAVE_L2_GUIDELINE
                  or heave_linf > HEAVE_LINF_GUIDELINE)
    heave_tag = "WARN" if heave_warn else "OK"
    print(f"  Heave: L2={heave_l2:.2e} (guideline {HEAVE_L2_GUIDELINE}), "
          f"Linf={heave_linf:.2e} (guideline {HEAVE_LINF_GUIDELINE}) "
          f"[{heave_tag}]")

    plot_comparison(
        t_conv_h, conv_h, dft_h,
        label_a="Convolution", label_b="DFT",
        title=f"{TEST_NAME}: Heave",
        y_label="Heave (m)",
        output_path=str(output_dir / "plots" / "heave.png"),
        skip_time=SKIP_TIME,
    )

    # ── Status (non-gating) ──────────────────────────────────────────
    any_warn = eta_warn or decay_warn or heave_warn
    status = "WARN" if any_warn else None
    write_comparison_status(str(output_dir), TEST_NAME, all_metrics,
                            passed=status)

    print(f"\n=== {TEST_NAME}: {'WARN' if any_warn else 'INFO'} ===")


if __name__ == '__main__':
    main()
