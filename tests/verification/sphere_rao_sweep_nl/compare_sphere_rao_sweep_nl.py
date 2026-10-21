#!/usr/bin/env python3
"""
IEA OES Task 10 sphere at steepness S = H/(g T^2) = 0.01: SEA-Stack nonlinear
Froude-Krylov (NL FK) heave RAO vs the MARIN (aNySIM) and NREL (WEC-Sim)
weakly nonlinear results (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 10).

Usage (called by CTest):
    python compare_sphere_rao_sweep_nl.py <normalized_dir> <results_index_file>

Pass criterion (fixed in the theory doc before any run): for the free body
and the optimum-PTO body, at every period the NL FK (Airy + Wheeler) RAO lies
within [min(MARIN, NREL) - 5% of their mean, max(MARIN, NREL) + 5% of their
mean]. Linear and long-wave NL FK RAOs are reported for information.

RAO: first-harmonic heave amplitude (least-squares fit of cos, sin and a
constant over the last N_FIT_PERIODS periods) divided by the wave amplitude.
"""

import os
import sys
from pathlib import Path

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'regression', 'utilities'))
from compare_template import write_status_file

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'verification'))
from normalize_utils import load_standard

import numpy as np

TEST_NAME = "sphere_rao_sweep_nl"
N_FIT_PERIODS = 10
BAND_WIDENING = 0.05
CONFIGS = ["free", "pto"]
CHECKED_MODEL = "nlfk_airy_wheeler"
MODELS = ["linear", "nlfk_airy_wheeler", "nlfk_hydrostatic"]
NONLINEAR_REFS = ["marin", "nrel"]
LINEAR_REF = "inwave"


def read_header(path):
    values = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.startswith("#"):
                break
            parts = line.lstrip("#").split()
            if len(parts) == 2:
                try:
                    values[parts[0]] = float(parts[1])
                except ValueError:
                    values[parts[0]] = parts[1]
    return values


def fit_rao(path):
    """Return (omega, RAO amplitude) from a heave time-series file."""
    header = read_header(path)
    w, a = header["omega_rad_s"], header["wave_amplitude_m"]
    d = np.loadtxt(path, comments="#")
    t, z = d[:, 0], d[:, 1]
    sel = t >= t[-1] - N_FIT_PERIODS * 2.0 * np.pi / w
    m = np.column_stack([np.cos(w * t[sel]), np.sin(w * t[sel]), np.ones(sel.sum())])
    c, *_ = np.linalg.lstsq(m, z[sel], rcond=None)
    return w, float(np.hypot(c[0], c[1]) / a)


def load_ref(normalized_dir, source, config):
    _, data = load_standard(normalized_dir / source / f"heave_rao_{config}.txt")
    return data[:, 0], data[:, 1]


def write_plot(output_dir, config, ref, sim):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(figsize=(7, 4))
    for name, (w, rao) in ref.items():
        ax.plot(2 * np.pi / w, rao, "o--" if name in NONLINEAR_REFS else "x:", label=name)
    for name, (w, rao) in sim.items():
        ax.plot(2 * np.pi / w, rao, "s-", label=f"SEA-Stack {name}")
    ax.set_xlabel("Wave period (s)")
    ax.set_ylabel("Heave RAO (m/m)")
    ax.set_title(f"IEA sphere, S = 0.01, {'free' if config == 'free' else 'optimum PTO'}")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    plots = output_dir / "plots"
    plots.mkdir(parents=True, exist_ok=True)
    fig.savefig(plots / f"sphere_rao_sweep_nl_{config}.png", dpi=120)
    plt.close(fig)


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <normalized_dir> <results_index_file>")
        sys.exit(1)
    normalized_dir = Path(sys.argv[1])
    results_dir = Path(sys.argv[2]).parent
    metrics, failures = {}, []
    lines = []

    for config in CONFIGS:
        ref = {s: load_ref(normalized_dir, s, config) for s in NONLINEAR_REFS + [LINEAR_REF]}
        sim = {}
        for model in MODELS:
            pts = [fit_rao(p) for p in sorted(results_dir.glob(f"{config}_T*_{model}.txt"))]
            if len(pts) != len(ref["marin"][0]):
                failures.append(f"{config} {model}: found {len(pts)} result files")
                continue
            pts.sort(key=lambda p: -p[0])
            sim[model] = (np.array([p[0] for p in pts]), np.array([p[1] for p in pts]))
        if CHECKED_MODEL not in sim:
            continue

        lines += [f"### {'Free body' if config == 'free' else 'Optimum PTO damping'}", "",
                  "| T (s) | a (m) | MARIN | NREL | Band | InWave (lin) | SEA-Stack linear | "
                  "NL FK Airy | NL FK long-wave | In band |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
        w_ref = ref["marin"][0]
        for i, w in enumerate(w_ref):
            j = int(np.argmin(np.abs(sim[CHECKED_MODEL][0] - w)))
            if abs(sim[CHECKED_MODEL][0][j] - w) > 1e-6:
                failures.append(f"{config}: no SEA-Stack result at w = {w:.4f} rad/s")
                continue
            m_val, n_val = ref["marin"][1][i], ref["nrel"][1][i]
            widen = BAND_WIDENING * 0.5 * (m_val + n_val)
            lo, hi = min(m_val, n_val) - widen, max(m_val, n_val) + widen
            vals = {model: sim[model][1][j] for model in sim}
            ok = lo <= vals[CHECKED_MODEL] <= hi
            period = 2 * np.pi / w
            tag = f"{config}_T{period:04.1f}"
            metrics[f"{tag}_nlfk_rao"] = vals[CHECKED_MODEL]
            metrics[f"{tag}_band_lo"] = lo
            metrics[f"{tag}_band_hi"] = hi
            lines.append(
                f"| {period:.1f} | {0.5 * 0.01 * 9.81 * period ** 2:.2f} | {m_val:.4f} | {n_val:.4f} | "
                f"[{lo:.4f}, {hi:.4f}] | {ref[LINEAR_REF][1][i]:.4f} | {vals.get('linear', np.nan):.4f} | "
                f"{vals[CHECKED_MODEL]:.4f} | {vals.get('nlfk_hydrostatic', np.nan):.4f} | "
                f"{'yes' if ok else 'NO'} |")
            if not ok:
                failures.append(f"{config} T = {period:.1f} s: NL FK RAO {vals[CHECKED_MODEL]:.4f} "
                                f"outside [{lo:.4f}, {hi:.4f}]")
        lines.append("")
        write_plot(results_dir, config, ref, sim)

    summary = "\n".join(lines)
    print(summary)
    (results_dir / "summary.md").write_text(summary + "\n", encoding="utf-8")
    status = "FAIL" if failures else "PASS"
    write_status_file(str(results_dir), TEST_NAME, status, metrics,
                      note="; ".join(failures) if failures else None)
    if failures:
        print("Failures:\n  " + "\n  ".join(failures))
    print(f"\n{TEST_NAME}: {status}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
