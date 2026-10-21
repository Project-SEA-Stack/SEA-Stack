#!/usr/bin/env python3
"""
C2 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): IEA sphere heave in
regular waves vs the independent Python reference (nlfk_reference.py).

Steady heave amplitude and phase are fitted by least squares on
[cos wt, sin wt, 1] over the last N_FIT_PERIODS periods, as in the reference.

Pass criteria, for each NL FK model (Airy + Wheeler, long-wave) and for linear:
  - amplitude within AMP_TOL_PCT and phase within PHASE_TOL_DEG of the reference;
  - where the reference NL FK differs from linear by more than SIG_DIFF_PCT, the
    SEA-Stack NL FK - linear difference has the same sign and is within
    DIFF_REL_TOL of the reference difference.
Calm-water nonlinear hydrostatics is reported for information.

Reference values: stage 2a table in the theory doc. Reproduce with
  python tests/verification/utilities/nlfk_reference.py regular --h5 <sphere.h5>
      [--period 5.026548 --amplitudes 0.5 1.0] [--pto-damping 6.1e5 --amplitudes 1.0 2.5]
"""

import json
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import HAS_MATPLOTLIB, plot_comparison, write_comparison_status

import numpy as np

TEST_NAME = "compare_nlfk_vs_reference_sphere_reg"
N_FIT_PERIODS = 10
AMP_TOL_PCT = 1.0
PHASE_TOL_DEG = 2.0
SIG_DIFF_PCT = 2.0
DIFF_REL_TOL = 0.30

MODELS = ["linear", "nl_hydrostatics", "nlfk_airy_wheeler", "nlfk_hydrostatic"]
CHECKED = ["linear", "nlfk_airy_wheeler", "nlfk_hydrostatic"]

# Reference steady heave: (amplitude m, phase deg) per model, from nlfk_reference.py.
REFERENCE = {
    "free_w0p8_a0p1": [(0.1034, -0.5), (0.1034, -0.5), (0.1034, -0.5), (0.1034, -0.5)],
    "free_w0p8_a1p0": [(1.0343, -0.5), (1.0523, -0.6), (1.0340, -0.5), (1.0341, -0.5)],
    "free_w0p8_a2p5": [(2.5856, -0.5), (3.0033, -1.6), (2.5845, -0.5), (2.5861, -0.5)],
    "pto_w0p8_a1p0": [(0.6940, -42.2), (0.6963, -42.4), (0.6911, -42.4), (0.6913, -42.4)],
    "pto_w0p8_a2p5": [(1.7351, -42.2), (1.7735, -43.6), (1.6874, -43.3), (1.6910, -43.6)],
    "free_w1p25_a0p5": [(0.7344, -10.2), (0.7468, -10.8), (0.7364, -10.4), (0.7354, -10.3)],
    "free_w1p25_a1p0": [(1.4688, -10.2), (1.5877, -13.5), (1.4869, -11.0), (1.4786, -10.9)],
}


def read_header_value(path, key):
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.lstrip("#").split()
            if line.startswith("#") and len(parts) == 2 and parts[0] == key:
                return float(parts[1])
    raise KeyError(f"{key} not found in {path}")


def fit_harmonic(t, y, w):
    """Return (amplitude, phase deg) with y ~ Re{Z exp(i w t)} + mean over the last periods."""
    sel = t >= t[-1] - N_FIT_PERIODS * 2.0 * np.pi / w
    a = np.column_stack([np.cos(w * t[sel]), np.sin(w * t[sel]), np.ones(sel.sum())])
    c, *_ = np.linalg.lstsq(a, y[sel], rcond=None)
    z = c[0] - 1j * c[1]
    return abs(z), float(np.degrees(np.angle(z)))


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)
    base_dir = Path(sys.argv[1])

    metrics = {}
    failures = []
    lines = [
        "| Case | Model | Reference amp (m) | SEA-Stack amp (m) | Amp diff (%) | Phase diff (deg) | Checked |",
        "|---|---|---|---|---|---|---|",
    ]
    diff_lines = [
        "| Case | Model | Reference NL FK / linear - 1 (%) | SEA-Stack NL FK / linear - 1 (%) | Checked |",
        "|---|---|---|---|---|",
    ]
    for tag, ref_rows in REFERENCE.items():
        results = {}
        for model in MODELS:
            path = base_dir / f"{tag}_{model}_heave.txt"
            if not path.exists():
                failures.append(f"missing {path.name}")
                continue
            w = read_header_value(path, "omega_rad_s")
            data = np.loadtxt(path, comments="#")
            results[model] = (data[:, 0], data[:, 1], *fit_harmonic(data[:, 0], data[:, 1], w))

        for i, model in enumerate(MODELS):
            if model not in results:
                continue
            ref_amp, ref_ph = ref_rows[i]
            amp, ph = results[model][2], results[model][3]
            amp_err = 100.0 * (amp / ref_amp - 1.0)
            ph_err = ph - ref_ph
            checked = model in CHECKED
            metrics[f"{tag}_{model}_amp_m"] = amp
            metrics[f"{tag}_{model}_amp_diff_pct"] = amp_err
            metrics[f"{tag}_{model}_phase_diff_deg"] = ph_err
            lines.append(f"| {tag} | {model} | {ref_amp:.4f} | {amp:.4f} | {amp_err:+.2f} | {ph_err:+.2f} | "
                         f"{'yes' if checked else 'info'} |")
            if checked and (abs(amp_err) > AMP_TOL_PCT or abs(ph_err) > PHASE_TOL_DEG):
                failures.append(f"{tag} {model}: amplitude {amp_err:+.2f}%, phase {ph_err:+.2f} deg")

        if "linear" not in results:
            continue
        for i, model in ((2, "nlfk_airy_wheeler"), (3, "nlfk_hydrostatic")):
            if model not in results:
                continue
            ref_diff = 100.0 * (ref_rows[i][0] / ref_rows[0][0] - 1.0)
            sea_diff = 100.0 * (results[model][2] / results["linear"][2] - 1.0)
            significant = abs(ref_diff) > SIG_DIFF_PCT
            diff_lines.append(f"| {tag} | {model} | {ref_diff:+.2f} | {sea_diff:+.2f} | "
                              f"{'yes' if significant else 'info'} |")
            metrics[f"{tag}_{model}_nl_minus_linear_pct"] = sea_diff
            if significant and (np.sign(sea_diff) != np.sign(ref_diff)
                                or abs(sea_diff - ref_diff) > DIFF_REL_TOL * abs(ref_diff)):
                failures.append(f"{tag} {model}: NL FK - linear {sea_diff:+.2f}% vs reference {ref_diff:+.2f}%")

        if HAS_MATPLOTLIB and "nlfk_airy_wheeler" in results:
            t = results["linear"][0]
            n = min(len(t), len(results["nlfk_airy_wheeler"][1]))
            plot_comparison(t[:n], results["linear"][1][:n], results["nlfk_airy_wheeler"][1][:n],
                            label_a="Linear", label_b="NL FK (Airy + Wheeler)",
                            title=f"{TEST_NAME}: {tag}", y_label="Heave from equilibrium (m)",
                            output_path=base_dir / "plots" / f"{tag}_heave.png")

    summary = "\n".join(lines) + "\n\n" + "\n".join(diff_lines)
    print(summary)
    (base_dir / "summary.md").write_text(summary + "\n", encoding="utf-8")

    passed = not failures
    payload = write_comparison_status(str(base_dir), TEST_NAME, metrics, passed=passed)
    if failures:
        payload["note"] = "; ".join(failures)
        (base_dir / "status.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print("\nFailures:\n  " + "\n  ".join(failures))
    print(f"\n{TEST_NAME}: {'PASS' if passed else 'FAIL'}")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
