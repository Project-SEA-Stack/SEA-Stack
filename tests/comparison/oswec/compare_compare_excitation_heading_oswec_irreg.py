#!/usr/bin/env python3
"""
Comparison script: IRF-convolution vs frequency-domain excitation for an
oblique (30 deg) long-crested sea on the OSWEC.

Pass/fail, using normalised RMS differences of flap pitch after the ramp:
  - IRF 30 deg vs FD 30 deg must agree (0.4 % on Windows, Nov 2026).
  - IRF 30 deg vs IRF 0 deg must differ (the heading must not be ignored;
    before the fix the two were bit-identical).
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import np, plot_comparison, write_comparison_status

SKIP_TIME = 60.0          # s, after the 30 s ramp has settled
MAX_IRF_VS_FD = 0.02      # normalised RMS, IRF 30 deg vs FD 30 deg
MIN_30_VS_0 = 0.05        # normalised RMS, IRF 30 deg vs IRF 0 deg

TEST_NAME = "compare_excitation_heading_oswec_irreg"


def load(path):
    data = np.loadtxt(path, comments="#")
    return data[:, 0], data[:, 1]


def nrms(a, b, mask):
    diff = a[mask] - b[mask]
    return float(np.sqrt(np.mean(diff ** 2)) / np.sqrt(np.mean(b[mask] ** 2)))


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)

    base_dir = Path(sys.argv[1])
    files = {name: base_dir / f"{name}.txt" for name in ("irf_0", "irf_30", "fd_30")}
    missing = [str(p) for p in files.values() if not p.exists()]
    if missing:
        print(f"Missing result files: {missing}")
        sys.exit(1)

    t, irf_0 = load(files["irf_0"])
    _, irf_30 = load(files["irf_30"])
    _, fd_30 = load(files["fd_30"])
    n = min(len(irf_0), len(irf_30), len(fd_30))
    t, irf_0, irf_30, fd_30 = t[:n], irf_0[:n], irf_30[:n], fd_30[:n]
    mask = t >= SKIP_TIME

    irf_vs_fd = nrms(irf_30, fd_30, mask)
    d30_vs_d0 = nrms(irf_30, irf_0, mask)
    passed = irf_vs_fd <= MAX_IRF_VS_FD and d30_vs_d0 >= MIN_30_VS_0

    metrics = {
        "nrms_irf30_vs_fd30": irf_vs_fd,
        "nrms_irf30_vs_irf0": d30_vs_d0,
        "max_nrms_irf30_vs_fd30": MAX_IRF_VS_FD,
        "min_nrms_irf30_vs_irf0": MIN_30_VS_0,
    }
    write_comparison_status(base_dir, TEST_NAME, metrics, passed)
    plot_comparison(t, fd_30, irf_30, label_a="FD 30 deg", label_b="IRF 30 deg",
                    title=f"{TEST_NAME}: FD vs IRF at 30 deg", y_label="Flap pitch (deg)",
                    output_path=str(base_dir / "plots" / "comparison.png"), skip_time=SKIP_TIME)

    print(f"  IRF 30 vs FD 30 : normalised RMS = {irf_vs_fd:.4f} (max {MAX_IRF_VS_FD})")
    print(f"  IRF 30 vs IRF 0 : normalised RMS = {d30_vs_d0:.4f} (min {MIN_30_VS_0})")
    print(f"  {TEST_NAME}: {'PASS' if passed else 'FAIL'}")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
