#!/usr/bin/env python3
"""
One-command visual verification / comparison workflow for the three RM3
external-PTO demos under a shared irregular sea state.

1. Runs the three demos headlessly via run_seastack (same YAML configs as the
   automated regression tests: JONSWAP Hs=2 m, Tp=8 s, seed=42, 600 s).
2. Runs a native Chrono LinearPTO twin of the linear case (reference force/heave).
3. Generates comparison / verification plots (PNG + multipage PDF) and a summary.
4. Prints the PDF path (optionally opens it).

Does not re-implement simulation logic in Python — it only invokes run_seastack
and plot_verification.py. Expect several minutes: four 600 s Chrono runs.

Usage:
  python examples/external_pto/run_visual_verification.py \\
      --run-seastack build/bin/Release/run_seastack.exe \\
      --output-dir external_pto_verification
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Optional


_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parents[1]
_RM3 = _REPO / "data" / "demos" / "run_seastack" / "rm3"
_PLOT = _HERE / "plot_verification.py"

CASES = (
    ("linear", _RM3 / "external_pto"),
    ("adaptive", _RM3 / "external_pto_adaptive"),
    ("hydraulic", _RM3 / "external_pto_hydraulic"),
)


def _run_env() -> dict:
    env = os.environ.copy()
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    return env


def default_exe() -> str:
    for key in ("SS_RUN_EXE", "RUN_SEASTACK_EXE", "SEASTACK_EXE"):
        env = os.environ.get(key)
        if env and Path(env).exists():
            return env
    names = (("run_seastack.exe", "run_seastack") if os.name == "nt"
             else ("run_seastack", "run_seastack.exe"))
    for name in names:
        for p in (
            _REPO / "build" / "bin" / "Release" / name,
            _REPO / "build" / "bin" / "Debug" / name,
            _REPO / "bin" / name,
        ):
            if p.exists():
                return str(p)
    return names[0]


def run_case(exe: str, case_dir: Path) -> Path:
    cmd = [exe, "--nogui", "--quiet", str(case_dir)]
    print(f"Running: {' '.join(cmd)}")
    proc = subprocess.run(
        cmd, capture_output=True, text=True, encoding="utf-8",
        errors="ignore", env=_run_env(),
    )
    if proc.returncode != 0:
        sys.stderr.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        raise RuntimeError(f"run_seastack failed for {case_dir} (exit {proc.returncode})")
    return find_results_h5(case_dir)


def find_results_h5(case_dir: Path) -> Path:
    outs = list((case_dir / "outputs").glob("results.*.h5"))
    if not outs:
        raise FileNotFoundError(f"no results H5 under {case_dir / 'outputs'}")
    return max(outs, key=lambda p: p.stat().st_mtime)


def make_native_twin(damping: float) -> Path:
    src = _RM3 / "external_pto"
    twin = _RM3 / "_visual_native_linpto"
    if twin.exists():
        shutil.rmtree(twin)
    twin.mkdir()
    for suffix in ("simulation", "hydro"):
        shutil.copy2(
            src / f"rm3_external_pto.{suffix}.yaml",
            twin / f"rm3_native_linpto.{suffix}.yaml",
        )
    model = (src / "rm3_external_pto.model.yaml").read_text(encoding="utf-8")
    model = re.sub(
        r"damping_coefficient:\s*\S+.*",
        f"damping_coefficient: {damping:.1f}",
        model,
        count=1,
    )
    (twin / "rm3_native_linpto.model.yaml").write_text(model, encoding="utf-8")
    (twin / "rm3_native_linpto.setup.yaml").write_text(
        "model_file: rm3_native_linpto.model.yaml\n"
        "simulation_file: rm3_native_linpto.simulation.yaml\n"
        "hydro_file: rm3_native_linpto.hydro.yaml\n"
        "output_directory: outputs\n",
        encoding="utf-8",
    )
    return twin


def try_open(path: Path) -> None:
    try:
        if sys.platform.startswith("win"):
            os.startfile(path)  # type: ignore[attr-defined]
        elif sys.platform == "darwin":
            subprocess.run(["open", str(path)], check=False)
        else:
            subprocess.run(["xdg-open", str(path)], check=False)
    except Exception as exc:  # noqa: BLE001
        print(f"(could not open PDF automatically: {exc})")


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run-seastack", default=default_exe(),
        help="Path to run_seastack executable",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("external_pto_verification"),
    )
    parser.add_argument(
        "--damping", type=float, default=1.2e6,
        help="Native LinearPTO twin damping (must match linear demo config)",
    )
    parser.add_argument(
        "--skip-run", action="store_true",
        help="Skip simulations; only plot existing outputs",
    )
    parser.add_argument(
        "--open", action="store_true",
        help="Attempt to open the generated PDF",
    )
    parser.add_argument(
        "--keep-twin", action="store_true",
        help="Keep the ephemeral native LinearPTO twin directory",
    )
    args = parser.parse_args(argv)

    exe = args.run_seastack
    if not args.skip_run and not Path(exe).exists():
        sys.stderr.write(f"run_seastack not found: {exe}\n")
        return 2

    twin_dir: Optional[Path] = None
    try:
        results = {}
        if not args.skip_run:
            for name, case_dir in CASES:
                results[name] = run_case(exe, case_dir)
            twin_dir = make_native_twin(args.damping)
            results["native"] = run_case(exe, twin_dir)
        else:
            for name, case_dir in CASES:
                results[name] = find_results_h5(case_dir)
            twin_h5_dir = _RM3 / "_visual_native_linpto" / "outputs"
            native_candidates = (
                list(twin_h5_dir.glob("results.*.h5")) if twin_h5_dir.exists() else []
            )
            if native_candidates:
                results["native"] = max(
                    native_candidates, key=lambda p: p.stat().st_mtime
                )
            elif Path(exe).exists():
                # Replot without re-running the three demos, but rebuild the
                # native twin so linear-vs-native is not left INCOMPLETE.
                print("Native twin missing; running LinearPTO reference only…")
                twin_dir = make_native_twin(args.damping)
                results["native"] = run_case(exe, twin_dir)
            else:
                print("WARNING: native LinearPTO twin H5 not found; "
                      "linear verification will be INCOMPLETE")
                results["native"] = None

        cmd = [
            sys.executable, str(_PLOT),
            "--linear", str(results["linear"]),
            "--adaptive", str(results["adaptive"]),
            "--hydraulic", str(results["hydraulic"]),
            "--output-dir", str(args.output_dir),
        ]
        if results.get("native"):
            cmd.extend(["--native", str(results["native"])])
        print(f"Plotting: {' '.join(cmd)}")
        proc = subprocess.run(cmd)
        if proc.returncode != 0:
            return proc.returncode

        pdf = Path(args.output_dir).resolve() / "external_pto_verification.pdf"
        print(f"\nVerification PDF: {pdf}")
        if args.open and pdf.is_file():
            try_open(pdf)
        return 0
    finally:
        if twin_dir is not None and twin_dir.exists() and not args.keep_twin:
            # Keep H5 only if plot already consumed it; remove ephemeral twin.
            shutil.rmtree(twin_dir, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
