#!/usr/bin/env python3
"""
Comparison script: linear vs nonlinear hydrostatics on the IEA sphere in
regular waves (heave only, T = 8 s, wave amplitudes 0.1, 1.0, 2.5 m).

Steady-state heave amplitude and phase are fitted by least squares on
[cos wt, sin wt, 1] over the last N_FIT_PERIODS wave periods.

Pass criterion (smallest amplitude only): the nonlinear-hydrostatics
response is within AMP_TOL_PCT amplitude and PHASE_TOL_DEG phase of the
linear response. Larger amplitudes are reported for information: the
nonlinear run uses calm-water buoyancy with linear excitation, so the two
are expected to drift apart as the motion grows.
"""

import json
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import (
    HAS_MATPLOTLIB,
    plot_comparison,
    write_comparison_status,
)

import numpy as np

TEST_NAME = "compare_linear_vs_nonlinear_hs_sphere_reg"
N_FIT_PERIODS = 10
AMP_TOL_PCT = 1.0
PHASE_TOL_DEG = 1.0

CASES = [
    {"tag": "a0p1", "checked": True},
    {"tag": "a1p0", "checked": False},
    {"tag": "a2p5", "checked": False},
]


def read_header_value(path, key):
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.lstrip("#").split()
            if line.startswith("#") and len(parts) == 2 and parts[0] == key:
                return float(parts[1])
    raise KeyError(f"{key} not found in {path}")


def fit_harmonic(t, y, w):
    """Return (Z, mean) with y ~ Re{Z exp(i w t)} + mean."""
    a = np.column_stack([np.cos(w * t), np.sin(w * t), np.ones_like(t)])
    c, *_ = np.linalg.lstsq(a, y, rcond=None)
    return c[0] - 1j * c[1], c[2]


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)
    base_dir = Path(sys.argv[1])

    metrics = {}
    passed = True
    lines = [
        "| Wave amplitude (m) | Linear heave amp (m) | NL heave amp (m) | NL / linear - 1 (%) | "
        "Phase NL - linear (deg) | NL mean offset (m) | Checked |",
        "|---|---|---|---|---|---|---|",
    ]
    for case in CASES:
        tag = case["tag"]
        file_lin = base_dir / f"{tag}_linear_heave.txt"
        file_nl = base_dir / f"{tag}_nonlinear_heave.txt"
        if not file_lin.exists() or not file_nl.exists():
            print(f"Missing result files for {tag}: {file_lin} / {file_nl}")
            if case["checked"]:
                passed = False
            continue

        amp = read_header_value(file_lin, "wave_amplitude_m")
        w = read_header_value(file_lin, "omega_rad_s")
        lin = np.loadtxt(file_lin, comments="#")
        nl = np.loadtxt(file_nl, comments="#")
        n = min(len(lin), len(nl))
        t, z_lin, z_nl = lin[:n, 0], lin[:n, 1], nl[:n, 1]

        sel = t >= t[-1] - N_FIT_PERIODS * 2.0 * np.pi / w
        zl, _ = fit_harmonic(t[sel], z_lin[sel], w)
        zn, mean_nl = fit_harmonic(t[sel], z_nl[sel], w)
        amp_err = 100.0 * (abs(zn) / abs(zl) - 1.0)
        phase_err = float(np.degrees(np.angle(zn / zl)))

        metrics[f"{tag}_linear_amp_m"] = float(abs(zl))
        metrics[f"{tag}_nonlinear_amp_m"] = float(abs(zn))
        metrics[f"{tag}_amp_diff_pct"] = float(amp_err)
        metrics[f"{tag}_phase_diff_deg"] = phase_err
        metrics[f"{tag}_nonlinear_mean_m"] = float(mean_nl)
        lines.append(
            f"| {amp:.2f} | {abs(zl):.4f} | {abs(zn):.4f} | {amp_err:+.3f} | {phase_err:+.3f} | "
            f"{mean_nl:+.4f} | {'yes' if case['checked'] else 'info'} |"
        )
        if case["checked"] and (abs(amp_err) > AMP_TOL_PCT or abs(phase_err) > PHASE_TOL_DEG):
            passed = False

        if HAS_MATPLOTLIB:
            plot_comparison(
                t,
                z_lin,
                z_nl,
                label_a="Linear hydrostatics",
                label_b="Nonlinear hydrostatics (calm waterplane)",
                title=f"{TEST_NAME}: heave, wave amplitude {amp:g} m",
                y_label="Heave from equilibrium (m)",
                output_path=base_dir / "plots" / f"{tag}_heave_comparison.png",
            )

    summary = "\n".join(lines)
    print(summary)
    (base_dir / "summary.md").write_text(summary + "\n", encoding="utf-8")

    payload = write_comparison_status(str(base_dir), TEST_NAME, metrics, passed=passed)
    if not passed:
        payload["note"] = (
            f"At the smallest wave amplitude the nonlinear-hydrostatics heave must be within "
            f"{AMP_TOL_PCT:g} % amplitude and {PHASE_TOL_DEG:g} deg phase of the linear result."
        )
        (base_dir / "status.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print(f"\n{TEST_NAME}: {'PASS' if passed else 'FAIL'}")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
