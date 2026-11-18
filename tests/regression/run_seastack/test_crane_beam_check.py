#!/usr/bin/env python3
"""
Crane beam_check end to end: the flexible Euler-beam jib (structure_file)
coupled to the floating crane vessel, hydrodynamics and the tension-only
hoist rope, through run_seastack.

The case is copied to a temporary folder and run as authored (10 s, calm
water, parked crane). Pass criteria:

  - the run succeeds (exit code 0), which includes the setup check
    structure_static_load_path at 8 s (jib mate reactions along gravity
    within 5 % of the jib weight);
  - the log reports that check as PASSED, so it was really evaluated;
  - the HDF5 output exists.

Standard library only. Usage:
    python test_crane_beam_check.py --exe <path/to/run_seastack>
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_CRANE = _REPO / "data" / "demos" / "run_seastack" / "crane"


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

    root = Path(tempfile.mkdtemp(prefix="seastack_crane_beam_"))
    try:
        crane = root / "crane"
        shutil.copytree(_CRANE, crane, ignore=shutil.ignore_patterns(
            "outputs", "__pycache__", "*.log", "meshes", "data"))
        case = crane / "beam_check"
        t0 = time.perf_counter()
        proc = subprocess.run([args.exe, "--nogui", str(case)], capture_output=True, text=True,
                              encoding="utf-8", errors="replace", env=_run_env(), timeout=900)
        log = proc.stdout + proc.stderr
        print(f"beam_check: exit {proc.returncode}, {time.perf_counter() - t0:.1f} s wall")
        check(proc.returncode == 0, "beam_check: run_seastack succeeds")
        check("structure_static_load_path check PASSED" in log,
              "beam_check: load-path check evaluated and passed")
        check(any((case / "outputs").glob("*.h5")), "beam_check: HDF5 written")
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
    print("\nCrane beam_check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
