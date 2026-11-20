#!/usr/bin/env python3
"""
Trimaran fea_arms end to end: two Chrono::FEA steel box cross-arms
(structure_file) clamped between three floating hulls with BEM
hydrodynamics in irregular waves, through run_seastack.

The trimaran folder is copied to a temporary location and the case is run
for 70 s (the sea ramps up over 60 s). Pass criteria:

  - the run succeeds (exit code 0) and the HDF5 output exists;
  - every diagnostics CSV value is finite;
  - load path: over the first 10 s the mean vertical mate reaction on the
    hulls (root + tip) of each arm equals minus the arm weight within 2 %
    (reactions are the force on the support body, world axes, z up);
  - after the ramp the peak vertical bending moment |Mz| of each arm is in a
    plausible band (1e5 to 5e6 N*m), so the arms really carry wave loads.

Standard library only. Usage:
    python test_trimaran_fea_arms.py --exe <path/to/run_seastack>
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_TRIMARAN = _REPO / "data" / "demos" / "run_seastack" / "trimaran"

_END_TIME_S = 70.0
_GRAVITY = 9.81            # m/s^2
_STEEL_DENSITY = 7850.0    # kg/m^3
_BOX_AREA = 0.8 * 1.0 - (0.8 - 2 * 0.025) * (1.0 - 2 * 0.025)  # m^2, 800x1000x25 mm
_ARM_LENGTH = math.hypot(7.0, 2.0)                                # m, (y, z) span
_ARM_WEIGHT_N = _BOX_AREA * _ARM_LENGTH * _STEEL_DENSITY * _GRAVITY


def _run_env() -> dict:
    env = dict(os.environ)
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    return env


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--exe", required=True, help="path to run_seastack")
    parser.add_argument("--keep", action="store_true", help="keep the temporary folder")
    args = parser.parse_args()

    failures = []

    def check(cond: bool, msg: str) -> None:
        print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
        if not cond:
            failures.append(msg)

    root = Path(tempfile.mkdtemp(prefix="seastack_trimaran_fea_"))
    try:
        trimaran = root / "trimaran"
        shutil.copytree(_TRIMARAN, trimaran, ignore=shutil.ignore_patterns(
            "outputs", "__pycache__", "*.log", ".venv"))
        case = trimaran / "fea_arms"
        sim = case / "trimaran_fea_arms.simulation.yaml"
        text = sim.read_text(encoding="utf-8")
        check("end_time: 100.0" in text, "simulation YAML has the expected end_time line")
        sim.write_text(text.replace("end_time: 100.0", f"end_time: {_END_TIME_S}"),
                       encoding="utf-8")

        t0 = time.perf_counter()
        proc = subprocess.run([args.exe, "--nogui", str(case)], capture_output=True, text=True,
                              encoding="utf-8", errors="replace", env=_run_env(), timeout=900)
        log = proc.stdout + proc.stderr
        print(f"fea_arms: exit {proc.returncode}, {time.perf_counter() - t0:.1f} s wall")
        check(proc.returncode == 0, "fea_arms: run_seastack succeeds")
        check(any((case / "outputs").glob("*.h5")), "fea_arms: HDF5 written")

        csv_path = case / "outputs" / "structure_diagnostics.csv"
        check(csv_path.is_file(), "fea_arms: structure diagnostics CSV written")
        if csv_path.is_file():
            with csv_path.open(newline="", encoding="utf-8") as f:
                rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
            check(len(rows) > 0 and rows[-1]["time_s"] >= _END_TIME_S - 0.1,
                  f"fea_arms: diagnostics reach t = {_END_TIME_S} s")
            check(all(math.isfinite(v) for r in rows for v in r.values()),
                  "fea_arms: all diagnostics values finite")

            early = [r for r in rows if r["time_s"] <= 10.0]
            late = [r for r in rows if r["time_s"] >= 50.0]
            for side in ("stbd", "port"):
                if early:
                    fz = sum(r[f"{side}_root_Fz_N"] + r[f"{side}_tip_Fz_N"] for r in early)
                    fz /= len(early)
                    err = 100.0 * (-fz - _ARM_WEIGHT_N) / _ARM_WEIGHT_N
                    check(abs(err) < 2.0,
                          f"{side}: mean hull reaction {-fz / 1e3:.2f} kN vs arm weight "
                          f"{_ARM_WEIGHT_N / 1e3:.2f} kN ({err:+.2f} %)")
                if late:
                    peak = max(r[f"arm_{side}_max_abs_Mz_Nm"] for r in late)
                    check(1.0e5 < peak < 5.0e6,
                          f"{side}: peak |Mz| after the ramp {peak / 1e6:.3f} MN*m in (0.1, 5)")
        if failures:
            print("\n".join(log.strip().splitlines()[-30:]))
    finally:
        if args.keep:
            print(f"kept {root}")
        else:
            shutil.rmtree(root, ignore_errors=True)

    if failures:
        print(f"\nFAILED: {len(failures)} check(s)")
        return 1
    print("\nTrimaran fea_arms passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
