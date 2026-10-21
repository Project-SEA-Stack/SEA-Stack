#!/usr/bin/env python3
"""
C1 and C6 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): IEA sphere
heave RAO, free body, constant-coefficient radiation.

C1 (checked): at steepness H / lambda = 0.0005, both NL FK models are within
AMP_TOL_PCT amplitude and PHASE_TOL_DEG phase of linear at every frequency.

C6 (information): a = 1 m, four models. Reports the RAO of each model and the
long-wave minus Airy difference, which should grow with k times the draft.
"""

import json
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import write_comparison_status

import numpy as np

TEST_NAME = "compare_nlfk_rao_sphere_reg"
N_FIT_PERIODS = 10
AMP_TOL_PCT = 1.0
PHASE_TOL_DEG = 1.0
DRAFT_M = 5.0  # sphere centre on the mean surface at equilibrium

MODELS = ["linear", "nl_hydrostatics", "nlfk_airy_wheeler", "nlfk_hydrostatic"]


def read_header_value(path, key):
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.lstrip("#").split()
            if line.startswith("#") and len(parts) == 2 and parts[0] == key:
                return float(parts[1])
    raise KeyError(f"{key} not found in {path}")


def fit(path):
    """Return (omega, wave amplitude, RAO amplitude, phase deg) of a heave file."""
    w = read_header_value(path, "omega_rad_s")
    a = read_header_value(path, "wave_amplitude_m")
    d = np.loadtxt(path, comments="#")
    t, z = d[:, 0], d[:, 1]
    sel = t >= t[-1] - N_FIT_PERIODS * 2.0 * np.pi / w
    m = np.column_stack([np.cos(w * t[sel]), np.sin(w * t[sel]), np.ones(sel.sum())])
    c, *_ = np.linalg.lstsq(m, z[sel], rcond=None)
    zc = c[0] - 1j * c[1]
    return w, a, abs(zc) / a, float(np.degrees(np.angle(zc)))


def collect(base_dir, prefix):
    """{omega tag: {model: fit tuple}}"""
    out = {}
    for path in sorted(base_dir.glob(f"{prefix}_w*_heave.txt")):
        stem = path.name[len(prefix) + 1:-len("_heave.txt")]
        tag, model = stem.split("_", 1)
        out.setdefault(tag, {})[model] = fit(path)
    return out


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)
    base_dir = Path(sys.argv[1])
    metrics, failures = {}, []

    c1 = collect(base_dir, "c1")
    if len(c1) != 4:
        failures.append(f"C1: expected 4 frequencies, found {len(c1)}")
    lines = ["C1: steepness 0.0005, NL FK vs linear", "",
             "| w (rad/s) | a (m) | Linear RAO | Model | NL FK RAO | Amp diff (%) | Phase diff (deg) |",
             "|---|---|---|---|---|---|---|"]
    for tag, res in sorted(c1.items(), key=lambda kv: kv[1]["linear"][0]):
        w, a, rao_lin, ph_lin = res["linear"]
        for model in ("nlfk_airy_wheeler", "nlfk_hydrostatic"):
            if model not in res:
                failures.append(f"C1 {tag}: missing {model}")
                continue
            rao, ph = res[model][2], res[model][3]
            amp_err = 100.0 * (rao / rao_lin - 1.0)
            ph_err = ph - ph_lin
            metrics[f"c1_{tag}_{model}_amp_diff_pct"] = amp_err
            metrics[f"c1_{tag}_{model}_phase_diff_deg"] = ph_err
            lines.append(f"| {w:.2f} | {a:.4f} | {rao_lin:.4f} | {model} | {rao:.4f} | {amp_err:+.3f} | "
                         f"{ph_err:+.3f} |")
            if abs(amp_err) > AMP_TOL_PCT or abs(ph_err) > PHASE_TOL_DEG:
                failures.append(f"C1 w = {w:.2f}: {model} amplitude {amp_err:+.3f}%, phase {ph_err:+.3f} deg")

    c6 = collect(base_dir, "c6")
    lines += ["", "C6: a = 1 m, four models (information)", "",
              "| w (rad/s) | k draft | " + " | ".join(f"{m} RAO" for m in MODELS) +
              " | long-wave - Airy (%) |",
              "|---|---|" + "---|" * len(MODELS) + "---|"]
    for tag, res in sorted(c6.items(), key=lambda kv: kv[1]["linear"][0]):
        w = res["linear"][0]
        k = w * w / 9.81
        raos = [res[m][2] if m in res else float("nan") for m in MODELS]
        lw_minus_airy = 100.0 * (raos[3] / raos[2] - 1.0)
        metrics[f"c6_{tag}_long_wave_minus_airy_pct"] = lw_minus_airy
        lines.append(f"| {w:.2f} | {k * DRAFT_M:.3f} | " + " | ".join(f"{r:.4f}" for r in raos) +
                     f" | {lw_minus_airy:+.2f} |")

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
