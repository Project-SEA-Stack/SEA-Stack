#!/usr/bin/env python3
"""
C7 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): still-water decay of
the IEA sphere from 1 m and 5 m with the three nonlinear variants.

Pass criteria:
  - NL FK Airy + Wheeler and NL FK long-wave heave are bit-identical (in still
    water both take the same code path);
  - NL FK vs calm-water nonlinear hydrostatics: max |heave difference|
    <= MAX_DIFF_M (surface pressure integral vs divergence-theorem volume
    differ only by round-off).
"""

import json
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import write_comparison_status

import numpy as np

TEST_NAME = "compare_nlfk_variants_sphere_decay"
MAX_DIFF_M = 1e-6
CASES = ["decay_1m", "decay_5m"]


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)
    base_dir = Path(sys.argv[1])
    metrics, failures = {}, []
    lines = ["| Case | Airy vs long-wave | max |NL FK - NL hydrostatics| (m) | Heave range (m) |",
             "|---|---|---|---|"]
    for case in CASES:
        paths = {m: base_dir / f"{case}_{m}_heave.txt"
                 for m in ("nl_hydrostatics", "nlfk_airy_wheeler", "nlfk_hydrostatic")}
        missing = [p.name for p in paths.values() if not p.exists()]
        if missing:
            failures.append(f"{case}: missing {', '.join(missing)}")
            continue
        identical = (paths["nlfk_airy_wheeler"].read_text(encoding="utf-8").splitlines()[5:]
                     == paths["nlfk_hydrostatic"].read_text(encoding="utf-8").splitlines()[5:])
        z = {m: np.loadtxt(p, comments="#")[:, 1] for m, p in paths.items()}
        diff = float(np.max(np.abs(z["nlfk_airy_wheeler"] - z["nl_hydrostatics"])))
        metrics[f"{case}_nlfk_vs_nl_hydrostatics_max_diff_m"] = diff
        metrics[f"{case}_airy_vs_long_wave_identical"] = identical
        lines.append(f"| {case} | {'identical' if identical else 'DIFFERENT'} | {diff:.2e} | "
                     f"[{z['nl_hydrostatics'].min():+.3f}, {z['nl_hydrostatics'].max():+.3f}] |")
        if not identical:
            failures.append(f"{case}: NL FK Airy and long-wave runs are not bit-identical")
        if diff > MAX_DIFF_M:
            failures.append(f"{case}: NL FK differs from NL hydrostatics by {diff:.2e} m > {MAX_DIFF_M:g} m")

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
