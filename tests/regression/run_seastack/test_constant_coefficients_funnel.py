#!/usr/bin/env python3
"""
Integration test: constant-coefficient radiation through run_seastack.

Exercises the real product funnel (run_seastack → RunSingleCase → HydroSystem::BuildModel)
for the RM3 regular_waves_cc demo:

  1. On-grid period (demo default) completes and writes results.
  2. Off-grid T = 8 s exits with a clean configuration error before Chrono
     stepping; no results HDF5 and no process abort.

Usage:
  test_constant_coefficients_funnel.py --exe <run_seastack>
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def _run_env() -> dict:
    """Environment for the run_seastack subprocess (Windows DLL dirs optional)."""
    env = os.environ.copy()
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    return env


_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parents[2]
_DEMO = _REPO / "data" / "demos" / "run_seastack" / "rm3" / "regular_waves_cc"
_ASSETS = _REPO / "data" / "demos" / "run_seastack" / "rm3" / "assets"

_ON_GRID_PERIOD = "7.853984"
_OFF_GRID_PERIOD = "8.0"
_SHORT_END_TIME = "1.0"


class Checker:
    def __init__(self) -> None:
        self.failures: list[str] = []

    def check(self, cond: bool, msg: str) -> None:
        print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
        if not cond:
            self.failures.append(msg)


def _set_yaml_scalar(text: str, key: str, value: str) -> str:
    """Replace the first `key: <value>` occurrence (YAML mapping line)."""
    pattern = re.compile(rf"^(\s*{re.escape(key)}:\s*)([^\n#]+)", re.MULTILINE)
    new_text, n = pattern.subn(rf"\g<1>{value}", text, count=1)
    if n != 1:
        raise RuntimeError(f"could not set {key} in YAML")
    return new_text


def _prepare_case(parent: Path, name: str, period: str) -> Path:
    case = parent / name
    case.mkdir(parents=True)
    for src in _DEMO.glob("*.yaml"):
        dst = case / src.name
        text = src.read_text(encoding="utf-8")
        if src.name.endswith(".simulation.yaml"):
            text = _set_yaml_scalar(text, "end_time", _SHORT_END_TIME)
        if src.name.endswith(".hydro.yaml"):
            text = _set_yaml_scalar(text, "period", period)
        dst.write_text(text, encoding="utf-8")
    return case


def _run(exe: str, case: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [exe, "--nogui", "--nobanner", str(case)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
        env=_run_env(),
    )


def _combined(proc: subprocess.CompletedProcess) -> str:
    return (proc.stdout or "") + "\n" + (proc.stderr or "")


def test_on_grid(exe: str, parent: Path, chk: Checker) -> None:
    print("\n[1] On-grid constant_coefficients (T = 7.853984 s)")
    case = _prepare_case(parent, "on_grid", _ON_GRID_PERIOD)
    proc = _run(exe, case)
    out = _combined(proc)
    chk.check(proc.returncode == 0, f"exit code 0 (got {proc.returncode})")
    results = list((case / "outputs").glob("results.*.h5")) if (case / "outputs").is_dir() else []
    chk.check(len(results) == 1, "wrote exactly one results.*.h5")
    chk.check("libc++abi" not in out and "terminating due to" not in out,
              "no process abort in output")
    if proc.returncode != 0:
        print(out[-2000:])


def test_off_grid(exe: str, parent: Path, chk: Checker) -> None:
    print("\n[2] Off-grid constant_coefficients (T = 8.0 s)")
    case = _prepare_case(parent, "off_grid", _OFF_GRID_PERIOD)
    proc = _run(exe, case)
    out = _combined(proc)
    chk.check(proc.returncode == 1, f"exit code 1 (got {proc.returncode})")
    chk.check(
        "Invalid hydrodynamic configuration" in out,
        "reports Invalid hydrodynamic configuration",
    )
    chk.check(
        "has no matching hydrodynamic coefficient" in out,
        "reports frequency-matching error",
    )
    chk.check("libc++abi" not in out and "terminating due to" not in out,
              "no libc++abi / terminate abort")
    # Progress bar text is printed once the step loop starts on rc6/mac abort path.
    chk.check(
        not re.search(r"t=\d+\.\d+\s*/\s*\d+", out),
        "no Chrono step-loop progress output",
    )
    outputs = case / "outputs"
    results = list(outputs.glob("results.*.h5")) if outputs.is_dir() else []
    chk.check(not results, "no results.*.h5 produced")
    if proc.returncode != 1 or "Invalid hydrodynamic configuration" not in out:
        print(out[-2000:])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", required=True, help="Path to run_seastack")
    args = parser.parse_args()

    exe = Path(args.exe)
    if not exe.is_file():
        print(f"FAIL: run_seastack not found: {exe}", file=sys.stderr)
        return 1
    if not _DEMO.is_dir() or not _ASSETS.is_dir():
        print(f"FAIL: demo or assets missing:\n  {_DEMO}\n  {_ASSETS}", file=sys.stderr)
        return 1

    print("=== test_constant_coefficients_funnel ===")
    print(f"exe:  {exe}")
    print(f"demo: {_DEMO}")

    chk = Checker()
    with tempfile.TemporaryDirectory(prefix="seastack_cc_funnel_") as tmp:
        parent = Path(tmp)
        # Demo YAML paths are ../assets/... relative to the case directory.
        assets_link = parent / "assets"
        try:
            assets_link.symlink_to(_ASSETS, target_is_directory=True)
        except OSError:
            # Windows without symlink privilege: copy is fine for this small asset set.
            shutil.copytree(_ASSETS, assets_link)

        test_on_grid(str(exe), parent, chk)
        test_off_grid(str(exe), parent, chk)

    print()
    if chk.failures:
        print(f"TESTS FAILED ({len(chk.failures)} failure(s))")
        for f in chk.failures:
            print(f"  - {f}")
        return 1
    print("ALL TESTS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
