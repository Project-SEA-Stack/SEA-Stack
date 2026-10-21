#!/usr/bin/env python3
"""
C5 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): RM3 at steepness
0.0005, float with NL FK (plate linear) vs all linear.

Steady heave of float and plate is fitted on [cos wt, sin wt, 1] over the last
N_FIT_PERIODS periods. Pass: for both bodies at every frequency, amplitude
within AMP_TOL_PCT and phase within PHASE_TOL_DEG of the all-linear run.
"""

import json
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import write_comparison_status

import numpy as np

TEST_NAME = "compare_nlfk_float_rm3_reg"
N_FIT_PERIODS = 10
AMP_TOL_PCT = 1.0
PHASE_TOL_DEG = 1.0
BODIES = ["float", "plate"]


def read_header_value(path, key):
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.lstrip("#").split()
            if line.startswith("#") and len(parts) == 2 and parts[0] == key:
                return float(parts[1])
    raise KeyError(f"{key} not found in {path}")


def fit(path):
    w = read_header_value(path, "omega_rad_s")
    d = np.loadtxt(path, comments="#")
    t = d[:, 0]
    sel = t >= t[-1] - N_FIT_PERIODS * 2.0 * np.pi / w
    m = np.column_stack([np.cos(w * t[sel]), np.sin(w * t[sel]), np.ones(sel.sum())])
    out = []
    for col in (1, 2):
        c, *_ = np.linalg.lstsq(m, d[sel, col], rcond=None)
        out.append(c[0] - 1j * c[1])
    return w, out


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)
    base_dir = Path(sys.argv[1])
    metrics, failures = {}, []
    lines = ["| w (rad/s) | Body | Linear amp (m) | Float NL FK amp (m) | Amp diff (%) | Phase diff (deg) |",
             "|---|---|---|---|---|---|"]
    linear_files = sorted(base_dir.glob("w*_linear.txt"))
    if len(linear_files) != 3:
        failures.append(f"expected 3 frequencies, found {len(linear_files)}")
    for lin_path in linear_files:
        nl_path = base_dir / lin_path.name.replace("_linear", "_float_nlfk")
        if not nl_path.exists():
            failures.append(f"missing {nl_path.name}")
            continue
        w, z_lin = fit(lin_path)
        _, z_nl = fit(nl_path)
        for body, zl, zn in zip(BODIES, z_lin, z_nl):
            amp_err = 100.0 * (abs(zn) / abs(zl) - 1.0)
            ph_err = float(np.degrees(np.angle(zn / zl)))
            key = f"{lin_path.stem.split('_')[0]}_{body}"
            metrics[f"{key}_amp_diff_pct"] = amp_err
            metrics[f"{key}_phase_diff_deg"] = ph_err
            lines.append(f"| {w:.2f} | {body} | {abs(zl):.5f} | {abs(zn):.5f} | {amp_err:+.3f} | {ph_err:+.3f} |")
            if abs(amp_err) > AMP_TOL_PCT or abs(ph_err) > PHASE_TOL_DEG:
                failures.append(f"w = {w:.2f} {body}: amplitude {amp_err:+.3f}%, phase {ph_err:+.3f} deg")

    summary = "\n".join(lines)
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
