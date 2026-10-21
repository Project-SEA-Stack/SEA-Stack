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
sys.path.append(str(Path(__file__).parent.parent.parent / "verification" / "utilities"))
from compare_template_internal import write_comparison_status
from verification_plot import create_verification_rao_plot

import numpy as np

TEST_NAME = "compare_nlfk_rao_sphere_reg"
N_FIT_PERIODS = 10
AMP_TOL_PCT = 1.0
PHASE_TOL_DEG = 1.0
DRAFT_M = 5.0  # sphere centre on the mean surface at equilibrium

MODELS = ["linear", "nl_hydrostatics", "nlfk_airy_wheeler", "nlfk_hydrostatic"]
MODEL_LABELS = {
    "linear": "SEA-Stack linear",
    "nl_hydrostatics": "SEA-Stack NL hydrostatics (calm water)",
    "nlfk_airy_wheeler": "SEA-Stack NL FK (Airy + Wheeler)",
    "nlfk_hydrostatic": "SEA-Stack NL FK (long-wave)",
}


def write_rao_plot(base_dir, results, title, file_name, error_thresholds, legend_kwargs=None):
    """Standard RAO page: NL FK (Airy + Wheeler) against linear, others overlaid.

    results: {omega tag: {model: fit tuple}}.
    """
    rows = sorted(results.values(), key=lambda res: res["linear"][0])
    rows = [res for res in rows if "nlfk_airy_wheeler" in res]
    if not rows:
        return

    def series(model):
        return (np.array([res[model][0] for res in rows]),
                np.array([res[model][2] for res in rows]),
                np.radians([res[model][3] for res in rows]))

    w_lin, rao_lin, ph_lin = series("linear")
    w_nl, rao_nl, ph_nl = series("nlfk_airy_wheeler")
    phase_err = np.abs(np.angle(np.exp(1j * (ph_nl - ph_lin))))
    extra = {MODEL_LABELS[m]: series(m) for m in MODELS[1:]
             if m != "nlfk_airy_wheeler" and all(m in res for res in rows)}
    create_verification_rao_plot(
        w_lin, rao_lin, ph_lin, w_nl, rao_nl, ph_nl,
        title=title,
        amp_label="Heave RAO (m/m)",
        output_path=str(base_dir / "plots" / file_name),
        ref_label=MODEL_LABELS["linear"],
        sim_label=MODEL_LABELS["nlfk_airy_wheeler"],
        amp_errors=np.abs(rao_nl / rao_lin - 1.0),
        phase_errors=phase_err,
        ref_file_path=str(base_dir),
        test_file_path=str(base_dir),
        extra_sources=extra,
        error_thresholds=error_thresholds,
        legend_kwargs=legend_kwargs,
        show_extra_errors=True,
    )


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

    write_rao_plot(base_dir, c1, "C1: Sphere Heave RAO, Steepness 0.0005 — NL FK vs Linear",
                   "c1_heave_rao_small_steepness.png",
                   error_thresholds=(AMP_TOL_PCT / 100.0, 2.0 * AMP_TOL_PCT / 100.0))
    write_rao_plot(base_dir, c6, "C6: Sphere Heave RAO, a = 1 m — Four Hydrostatic and FK Models",
                   "c6_heave_rao_four_models.png", error_thresholds=(0.10, 0.25),
                   legend_kwargs=dict(loc="upper right", ncol=2))

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
