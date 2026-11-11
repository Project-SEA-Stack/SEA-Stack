#!/usr/bin/env python3
"""
Crane demo end to end (external controller preview): short lift_baseline and
lift_3dmc runs through run_seastack.

Each case is copied to a temporary folder with end_time shortened to 15 s
(hold 0-8 s, then the first 7 s of the slew). Pass criteria, from the crane
diagnostics CSV written by crane_controller.py:

  - the run succeeds and every CSV value is finite;
  - the rope tension is never negative (tension-only rope);
  - after the first second, the rope never goes slack in flight;
  - the 3D-MC tip error (10-15 s) is below 0.2 m and below a quarter of the
    open-loop tip error (README, full 54 s run: 0.07-0.08 m vs 1.1 m).

Standard library only. Usage:
    python test_crane_lift.py --exe <path/to/run_seastack>
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
_CRANE = _REPO / "data" / "demos" / "run_seastack" / "crane"
_HELPER_DIR = _REPO / "libs" / "external" / "python"
END_TIME_S = 15.0


def _run_env() -> dict:
    env = dict(os.environ)
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    env["PYTHONPATH"] = os.pathsep.join([str(_HELPER_DIR), env.get("PYTHONPATH", "")])
    return env


def prepare(root: Path) -> Path:
    crane = root / "crane"
    shutil.copytree(_CRANE, crane, ignore=shutil.ignore_patterns(
        "outputs", "__pycache__", "*.log", "meshes", "data"))
    sim = crane / "common" / "crane.simulation.yaml"
    text = sim.read_text(encoding="utf-8")
    if "end_time: 54.0" not in text:
        raise RuntimeError("crane.simulation.yaml: expected 'end_time: 54.0'")
    sim.write_text(text.replace("end_time: 54.0", f"end_time: {END_TIME_S}"), encoding="utf-8")
    for case in ("lift_baseline", "lift_3dmc"):
        ctl = crane / case / "crane.controller.yaml"
        ctl.write_text(ctl.read_text(encoding="utf-8").replace(
            "command: [python,", f'command: ["{Path(sys.executable).as_posix()}",'),
            encoding="utf-8")
    return crane


def read_csv(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    cols = {k: [float(r[k]) for r in rows] for k in rows[0] if k != "mode"}
    return cols


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

    root = Path(tempfile.mkdtemp(prefix="seastack_crane_"))
    tip_err = {}
    try:
        crane = prepare(root)
        for case in ("lift_baseline", "lift_3dmc"):
            t0 = time.perf_counter()
            proc = subprocess.run([args.exe, "--nogui", str(crane / case)], capture_output=True,
                                  text=True, encoding="utf-8", errors="replace", env=_run_env(),
                                  timeout=900)
            wall = time.perf_counter() - t0
            print(f"{case}: exit {proc.returncode}, {wall:.1f} s wall for {END_TIME_S:.0f} s")
            check(proc.returncode == 0, f"{case}: run_seastack succeeds")
            if proc.returncode != 0:
                print("\n".join((proc.stdout + proc.stderr).strip().splitlines()[-30:]))
                continue
            d = read_csv(crane / case / "outputs" / "crane_diagnostics.csv")
            t = d["time_s"]
            check(abs(t[-1] - END_TIME_S) < 0.01, f"{case}: diagnostics reach t = {END_TIME_S:.0f} s")
            check(all(math.isfinite(v) for col in d.values() for v in col),
                  f"{case}: every diagnostics value is finite")
            check(min(d["tension_N"]) >= 0.0, f"{case}: rope tension never negative")
            slack_late = sum(1 for ti, s in zip(t, d["slack"]) if ti >= 1.0 and s >= 1.0)
            check(slack_late == 0, f"{case}: no slack rope after t = 1 s ({slack_late} samples)")
            window = [e for ti, e in zip(t, d["tip_err_3d_m"]) if 10.0 <= ti <= END_TIME_S]
            tip_err[case] = sum(window) / len(window)
            print(f"  mean tip error 10-{END_TIME_S:.0f} s: {tip_err[case]:.3f} m, "
                  f"peak tension / static: {max(d['tension_N']) / (3000.0 * 9.81):.2f}")
        if len(tip_err) == 2:
            check(tip_err["lift_3dmc"] < 0.2, "3D-MC tip error below 0.2 m")
            check(tip_err["lift_3dmc"] < 0.25 * tip_err["lift_baseline"],
                  "3D-MC tip error below a quarter of the open-loop tip error")
    finally:
        if args.keep:
            print(f"kept {root}")
        else:
            shutil.rmtree(root, ignore_errors=True)

    if failures:
        print(f"\nFAILED: {len(failures)} check(s)")
        return 1
    print("\nCrane lift checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
