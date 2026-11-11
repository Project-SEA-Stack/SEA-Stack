#!/usr/bin/env python3
"""
Comparison script: linear hydrostatics, calm-water nonlinear hydrostatics and
nonlinear Froude-Krylov (NL FK) on the IEA sphere in regular waves (heave
only, no PTO, T = 8 s, wave amplitudes 0.1, 0.5, 1.0, 2.5 m, i.e.
H = 0.2, 1, 2, 5 m).

Steady-state heave amplitude and phase are fitted by least squares on
[cos wt, sin wt, 1] over the last N_FIT_PERIODS wave periods.

Pass criteria (each model against linear):
  - calm-water nonlinear at a = 0.1 m: within CALM_AMP_TOL_PCT amplitude and
    PHASE_TOL_DEG phase;
  - NL FK at a = 0.5 m (H = 1 m, H / (g T^2) = 0.0016): within
    NLFK_AMP_TOL_PCT amplitude and PHASE_TOL_DEG phase.
Other cases are reported for information. The calm-water model uses
calm-water buoyancy with linear excitation, so it drifts from linear as the
motion grows; NL FK integrates the incident-wave pressure on the wetted hull.
"""

import json
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import (
    HAS_MATPLOTLIB,
    write_comparison_status,
)

import numpy as np

if HAS_MATPLOTLIB:
    import matplotlib.pyplot as plt

TEST_NAME = "compare_linear_vs_nonlinear_hs_sphere_reg"
N_FIT_PERIODS = 10
CALM_AMP_TOL_PCT = 1.0
NLFK_AMP_TOL_PCT = 2.0
PHASE_TOL_DEG = 1.0

TAGS = ["a0p1", "a0p5", "a1p0", "a2p5"]
# (file tag, label, {amplitude tag: amplitude tolerance [%]} for checked cases)
MODELS = [
    ("nonlinear", "Calm-water nonlinear hydrostatics", {"a0p1": CALM_AMP_TOL_PCT}),
    ("nlfk", "Nonlinear Froude-Krylov", {"a0p5": NLFK_AMP_TOL_PCT}),
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
        "| Wave height H (m) | Linear heave amp (m) | Model | Model heave amp (m) | "
        "Model / linear - 1 (%) | Phase model - linear (deg) | Model mean offset (m) | Checked |",
        "|---|---|---|---|---|---|---|---|",
    ]
    sweep = {"H": [], "linear": [], "nonlinear": [], "nlfk": []}
    for tag in TAGS:
        file_lin = base_dir / f"{tag}_linear_heave.txt"
        if not file_lin.exists():
            print(f"Missing result file {file_lin}")
            passed = False
            continue
        amp = read_header_value(file_lin, "wave_amplitude_m")
        w = read_header_value(file_lin, "omega_rad_s")
        lin = np.loadtxt(file_lin, comments="#")
        t = lin[:, 0]
        sel = t >= t[-1] - N_FIT_PERIODS * 2.0 * np.pi / w
        zl, _ = fit_harmonic(t[sel], lin[sel, 1], w)
        metrics[f"{tag}_linear_amp_m"] = float(abs(zl))
        sweep["H"].append(2.0 * amp)
        sweep["linear"].append(abs(zl) / amp)
        series = {"linear": (t, lin[:, 1])}

        for model, label, checked in MODELS:
            path = base_dir / f"{tag}_{model}_heave.txt"
            if not path.exists():
                print(f"Missing result file {path}")
                passed = False
                sweep[model].append(np.nan)
                continue
            data = np.loadtxt(path, comments="#")
            n = min(len(data), len(lin))
            tm, zm_t = data[:n, 0], data[:n, 1]
            selm = tm >= tm[-1] - N_FIT_PERIODS * 2.0 * np.pi / w
            zm, mean_m = fit_harmonic(tm[selm], zm_t[selm], w)
            amp_err = 100.0 * (abs(zm) / abs(zl) - 1.0)
            phase_err = float(np.degrees(np.angle(zm / zl)))
            metrics[f"{tag}_{model}_amp_m"] = float(abs(zm))
            metrics[f"{tag}_{model}_amp_diff_pct"] = float(amp_err)
            metrics[f"{tag}_{model}_phase_diff_deg"] = phase_err
            metrics[f"{tag}_{model}_mean_m"] = float(mean_m)
            sweep[model].append(abs(zm) / amp)
            series[model] = (tm, zm_t)
            is_checked = tag in checked
            lines.append(
                f"| {2.0 * amp:.1f} | {abs(zl):.4f} | {label} | {abs(zm):.4f} | {amp_err:+.3f} | "
                f"{phase_err:+.3f} | {mean_m:+.4f} | {'yes' if is_checked else 'info'} |"
            )
            if is_checked and (abs(amp_err) > checked[tag] or abs(phase_err) > PHASE_TOL_DEG):
                passed = False

        if HAS_MATPLOTLIB:
            fig, ax = plt.subplots(figsize=(10, 4))
            for model, label in [("linear", "Linear hydrostatics")] + [(m, l) for m, l, _ in MODELS]:
                if model in series:
                    ax.plot(series[model][0], series[model][1], label=label, lw=1.0)
            ax.set_xlim(t[-1] - 4 * 2.0 * np.pi / w, t[-1])
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Heave from equilibrium (m)")
            ax.set_title(f"IEA sphere, regular waves H = {2.0 * amp:g} m, T = {2.0 * np.pi / w:g} s")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right")
            (base_dir / "plots").mkdir(parents=True, exist_ok=True)
            fig.tight_layout()
            fig.savefig(base_dir / "plots" / f"{tag}_heave_comparison.png", dpi=120)
            plt.close(fig)

    if HAS_MATPLOTLIB and sweep["H"]:
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.plot(sweep["H"], sweep["linear"], "o-", label="Linear hydrostatics")
        for model, label, _ in MODELS:
            ax.plot(sweep["H"], sweep[model], "s--" if model == "nonlinear" else "^-", label=label)
        ax.set_xlabel("Wave height H (m)")
        ax.set_ylabel("Heave amplitude / wave amplitude (-)")
        ax.set_title("IEA sphere heave, T = 8 s, no PTO")
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(base_dir / "plots" / "amplitude_sweep.png", dpi=120)
        plt.close(fig)

    summary = "\n".join(lines)
    print(summary)
    (base_dir / "summary.md").write_text(summary + "\n", encoding="utf-8")

    payload = write_comparison_status(str(base_dir), TEST_NAME, metrics, passed=passed)
    if not passed:
        payload["note"] = (
            f"Against linear: calm-water nonlinear at a = 0.1 m within {CALM_AMP_TOL_PCT:g} % amplitude, "
            f"NL FK at H = 1 m within {NLFK_AMP_TOL_PCT:g} % amplitude, both within "
            f"{PHASE_TOL_DEG:g} deg phase."
        )
        (base_dir / "status.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print(f"\n{TEST_NAME}: {'PASS' if passed else 'FAIL'}")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
