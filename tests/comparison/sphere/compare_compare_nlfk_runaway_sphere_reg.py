#!/usr/bin/env python3
"""
C4 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): IEA sphere, H = 10 m,
T = 8 s, free heave.

Pass criteria:
  - both NL FK models keep the heave within +/-(draft + amplitude) = +/-10 m
    and never trip the divergence guard;
  - the calm-water nonlinear hydrostatics run logs the Stage 1 warning about
    nonlinear hydrostatics in waves, and the NL FK runs do not.
Reported for information: heave range of all four models (the calm-water run
is expected to leave the water, as in the original report).

When a run trips the divergence guard (ChronoForceAttacher), all hydro forces
are zeroed from then on and the body falls freely. The heave range is
therefore taken up to the divergence time only.
"""

import json
import re
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import write_comparison_status

import numpy as np

TEST_NAME = "compare_nlfk_runaway_sphere_reg"
HEAVE_LIMIT_M = 10.0  # draft 5 m + wave amplitude 5 m
MODELS = ["linear", "nl_hydrostatics", "nlfk_airy_wheeler", "nlfk_hydrostatic"]
NLFK = ("nlfk_airy_wheeler", "nlfk_hydrostatic")
CALM_WATER_WARNING = "nonlinear hydrostatics combined with waves"
DIVERGENCE = re.compile(r"(?:Simulation divergence|Invalid force) detected at t=([-+0-9.eE]+)")


def log_sections(path):
    """{model: log text} split at the NLFK_RUN markers."""
    sections, current = {}, None
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "NLFK_RUN " in line:
            current = line.split("NLFK_RUN ", 1)[1].strip()
            sections[current] = ""
        elif current:
            sections[current] += line + "\n"
    return sections


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)
    base_dir = Path(sys.argv[1])
    metrics, failures = {}, []

    log = base_dir / "run.log"
    sections = log_sections(log) if log.exists() else {}
    if not log.exists():
        failures.append("run.log missing")

    lines = ["| Model | Heave min (m) | Heave max (m) | Divergence guard tripped at (s) | Checked |",
             "|---|---|---|---|---|"]
    for model in MODELS:
        path = base_dir / f"runaway_{model}_heave.txt"
        if not path.exists():
            failures.append(f"missing {path.name}")
            continue
        data = np.loadtxt(path, comments="#")
        match = DIVERGENCE.search(sections.get(model, ""))
        t_div = float(match.group(1)) if match else None
        z = data[data[:, 0] <= t_div, 1] if t_div is not None else data[:, 1]
        metrics[f"{model}_heave_min_m"] = float(z.min())
        metrics[f"{model}_heave_max_m"] = float(z.max())
        metrics[f"{model}_divergence_time_s"] = t_div
        checked = model in NLFK
        lines.append(f"| {model} | {z.min():+.2f} | {z.max():+.2f} | "
                     f"{'-' if t_div is None else f'{t_div:.2f}'} | {'yes' if checked else 'info'} |")
        if checked and t_div is not None:
            failures.append(f"{model}: divergence guard tripped at t = {t_div:.2f} s")
        if checked and np.max(np.abs(z)) > HEAVE_LIMIT_M:
            failures.append(f"{model}: heave reaches {np.max(np.abs(z)):.2f} m (limit {HEAVE_LIMIT_M} m)")

    if sections:
        warned = {m: CALM_WATER_WARNING in sections.get(m, "") for m in MODELS}
        lines += ["", "| Model | Calm-water warning logged |", "|---|---|"]
        lines += [f"| {m} | {'yes' if warned[m] else 'no'} |" for m in MODELS]
        if not warned["nl_hydrostatics"]:
            failures.append("calm-water nonlinear hydrostatics run did not log the waves warning")
        for m in NLFK:
            if warned[m]:
                failures.append(f"{m} run logged the calm-water warning")

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
