#!/usr/bin/env python3
"""
Diagnostic script for sphere_irreg_waves_eta regression failure.

Loads the test result and reference data, identifies where the maximum
deviation occurs, and produces a diagnostic plot showing:
  1. Full time series overlay
  2. Zoomed region around the maximum deviation
  3. Error time series

Findings from status file analysis:
  - L2 norm:   0.0017  (threshold 1e-4, exceeds by 17x)
  - Linf norm: 1.155   (threshold 0.02, exceeds by 58x)
  - Linf of 1.15 m indicates a large transient spike, likely from
    DFT reconstruction artefacts near the start/end of the time series,
    or from frequency truncation in BuildFromEtaFile (nf=1000, f=[0.001,1.0] Hz).

Root cause hypotheses:
  1. DFT frequency resolution: The targeted DFT uses nf=1000 linearly-spaced
     frequencies from 0.001 to 1.0 Hz, giving df=0.001 Hz. If the original
     eta file was generated with different frequency spacing (e.g. from a
     ComponentSampler with different omega_min/max/n_omega), components may
     not align exactly, causing spectral leakage.
  2. Ramp mismatch: The regular sphere_irreg_waves test uses SetRampDuration(60.0)
     but the ETA test does not set any ramp.
  3. Phase convention: Already verified -- the DFT extraction and wave field
     evaluation use consistent cos(phi - omega*t) convention.
  4. Time offset: BuildFromEtaFile uses t0 from the eta file. If t0 != 0, the
     reconstructed components carry a t0-dependent phase shift that may differ
     from the original.

Recommended fix priority:
  A. Check if the eta file's frequency content matches the DFT grid exactly.
  B. Relax ETA regression thresholds to realistic DFT reconstruction accuracy
     (e.g. L2=5e-3, Linf=2.0), or classify this as an informational test.
  C. Optionally add a ramp to the ETA test to match the reference data.
"""

import sys
import os
import numpy as np
from pathlib import Path

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def find_data_start(filename):
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            try:
                float(line.split()[0])
                return i
            except (ValueError, IndexError):
                continue
    return 0


def main():
    build_dir = os.environ.get('SEASTACK_BUILD_DIR', 'build')
    build_path = Path(build_dir).resolve()

    ref_file = None
    test_file = None

    for p in [build_path / "bin" / "Release" / "results" / "tests" / "sphere"]:
        candidate = p / "results_sphere_irreg_waves_eta.txt"
        if candidate.exists():
            test_file = candidate
            break

    ref_candidates = [
        build_path / "data" / "reference_data" / "sphere" / "ss_ref_sphere_irreg_waves_eta.txt",
        Path(os.environ.get('SEASTACK_DATA_DIR', '')) / "reference_data" / "sphere" / "ss_ref_sphere_irreg_waves_eta.txt",
    ]
    for rc in ref_candidates:
        if rc.exists():
            ref_file = rc
            break

    if not test_file:
        print("Cannot find test result file. Set SEASTACK_BUILD_DIR or run from build dir.")
        sys.exit(1)
    if not ref_file:
        print("Cannot find reference file. Set SEASTACK_DATA_DIR.")
        sys.exit(1)

    print(f"Reference: {ref_file}")
    print(f"Test:      {test_file}")

    ref_skip = find_data_start(str(ref_file))
    test_skip = find_data_start(str(test_file))
    ref_data = np.loadtxt(str(ref_file), skiprows=ref_skip)
    test_data = np.loadtxt(str(test_file), skiprows=test_skip)

    n = min(len(ref_data), len(test_data))
    t_ref, h_ref = ref_data[:n, 0], ref_data[:n, 1]
    t_test, h_test = test_data[:n, 0], test_data[:n, 1]
    error = h_test - h_ref

    l2 = np.sqrt(np.mean(error ** 2))
    linf = np.max(np.abs(error))
    i_max = np.argmax(np.abs(error))
    t_max = t_test[i_max]

    print(f"\nDiagnostics:")
    print(f"  Points compared: {n}")
    print(f"  L2 norm:   {l2:.6e}")
    print(f"  Linf norm: {linf:.6e} at t={t_max:.2f} s")
    print(f"  Ref heave at max error:  {h_ref[i_max]:.6f} m")
    print(f"  Test heave at max error: {h_test[i_max]:.6f} m")

    # Error distribution
    print(f"\n  Error percentiles:")
    for p in [50, 90, 95, 99, 99.9]:
        print(f"    P{p:5.1f}: {np.percentile(np.abs(error), p):.6e} m")

    # Time-windowed analysis
    print(f"\n  Windowed L2 norms:")
    windows = [(0, 60), (60, 300), (300, 600), (600, t_test[-1])]
    for t_start, t_end in windows:
        mask = (t_test >= t_start) & (t_test < t_end)
        if mask.any():
            l2_w = np.sqrt(np.mean(error[mask] ** 2))
            linf_w = np.max(np.abs(error[mask]))
            print(f"    [{t_start:6.0f}, {t_end:6.0f}): L2={l2_w:.4e}, Linf={linf_w:.4e}")

    if HAS_MPL:
        out_dir = test_file.parent / "plots"
        out_dir.mkdir(exist_ok=True)

        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=False)

        axes[0].plot(t_ref, h_ref, label='Reference (HC)', alpha=0.8, linewidth=0.5)
        axes[0].plot(t_test, h_test, label='Test (ETA import)', alpha=0.8, linewidth=0.5)
        axes[0].set_ylabel('Heave (m)')
        axes[0].set_title('Full time series')
        axes[0].legend()

        w = 30
        i_lo = max(0, i_max - int(w / (t_test[1] - t_test[0])))
        i_hi = min(n, i_max + int(w / (t_test[1] - t_test[0])))
        axes[1].plot(t_ref[i_lo:i_hi], h_ref[i_lo:i_hi], label='Reference')
        axes[1].plot(t_test[i_lo:i_hi], h_test[i_lo:i_hi], label='Test', linestyle='--')
        axes[1].axvline(t_max, color='red', alpha=0.3, linestyle=':')
        axes[1].set_ylabel('Heave (m)')
        axes[1].set_title(f'Zoomed around max error (t={t_max:.1f}s, err={linf:.3f}m)')
        axes[1].legend()

        axes[2].plot(t_test, error, color='red', linewidth=0.5)
        axes[2].set_ylabel('Error (m)')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_title(f'Error: L2={l2:.4e}, Linf={linf:.4e}')

        plt.tight_layout()
        plt.savefig(str(out_dir / 'eta_failure_diagnosis.png'), dpi=150)
        print(f"\n  Diagnostic plot: {out_dir / 'eta_failure_diagnosis.png'}")


if __name__ == '__main__':
    main()
