#!/usr/bin/env python3
"""
One-command visual verification / comparison workflow for the three
external-PTO demos (RM3 TSDA or OSWEC RSDA) under a shared irregular sea state.

1. Runs the three demos headlessly via run_seastack (same YAML configs as the
   automated regression tests: JONSWAP Hs=2 m, Tp=8 s, seed=42, 600 s).
2. Runs a native Chrono damper twin of the linear case (reference motion /
   actuator).
3. Generates comparison / verification plots (PNG + multipage PDF) and a summary.
4. Prints the PDF path (optionally opens it).

Does not re-implement simulation logic in Python — it only invokes run_seastack
and plot_verification.py. Expect several minutes: four 600 s Chrono runs.

Usage:
  python examples/external_pto/run_visual_verification.py \\
      --run-seastack build/bin/Release/run_seastack.exe \\
      --output-dir external_pto_verification

  python examples/external_pto/run_visual_verification.py --platform oswec \\
      --output-dir oswec_external_pto_verification
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

from plot_verification import PLATFORMS, PlatformSpec


_HERE = Path(__file__).resolve().parent
# Parent of examples/external_pto — repo root or release ZIP root.
_TREE_ROOT = _HERE.parents[1]
_PLOT = _HERE / "plot_verification.py"


def _run_env() -> dict:
    env = os.environ.copy()
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    # Prefer package/staged bin so Chrono/HDF5 DLLs next to run_seastack resolve.
    bin_dir = _TREE_ROOT / "bin"
    if bin_dir.is_dir():
        env["PATH"] = os.pathsep.join([str(bin_dir), env.get("PATH", "")])
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
            _TREE_ROOT / "bin" / name,  # release ZIP / staged install
            _TREE_ROOT / "build" / "bin" / "Release" / name,
            _TREE_ROOT / "build" / "bin" / "Debug" / name,
        ):
            if p.exists():
                return str(p)
    return names[0]


def case_dirs(plat: PlatformSpec) -> tuple:
    root = plat.demo_root
    return (
        ("linear", root / "external_pto"),
        ("adaptive", root / "external_pto_adaptive"),
        ("hydraulic", root / "external_pto_hydraulic"),
    )


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


def make_native_twin(plat: PlatformSpec, damping: float) -> Path:
    """Ephemeral native-damper twin of the linear external case (no IPC)."""
    src = plat.demo_root / "external_pto"
    twin = plat.demo_root / plat.twin_dirname
    if twin.exists():
        shutil.rmtree(twin)
    twin.mkdir()
    stem_src = plat.model_stem
    stem_dst = (
        "oswec_native_rsda" if plat.key == "oswec" else "rm3_native_linpto"
    )
    for suffix in ("simulation", "hydro"):
        shutil.copy2(
            src / f"{stem_src}.{suffix}.yaml",
            twin / f"{stem_dst}.{suffix}.yaml",
        )
    model = (src / f"{stem_src}.model.yaml").read_text(encoding="utf-8")
    model = re.sub(
        r"damping_coefficient:\s*\S+.*",
        f"damping_coefficient: {damping:.1f}",
        model,
        count=1,
    )
    (twin / f"{stem_dst}.model.yaml").write_text(model, encoding="utf-8")
    (twin / f"{stem_dst}.setup.yaml").write_text(
        f"model_file: {stem_dst}.model.yaml\n"
        f"simulation_file: {stem_dst}.simulation.yaml\n"
        f"hydro_file: {stem_dst}.hydro.yaml\n"
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
        "--platform", choices=sorted(PLATFORMS), default="rm3",
        help="Demo platform: rm3 (TSDA) or oswec (RSDA)",
    )
    parser.add_argument(
        "--run-seastack", default=default_exe(),
        help="Path to run_seastack executable",
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--damping", type=float, default=None,
        help="Native damper twin coefficient (must match linear demo config)",
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
        help="Keep the ephemeral native damper twin directory",
    )
    args = parser.parse_args(argv)

    plat = PLATFORMS[args.platform]
    damping = (
        float(args.damping) if args.damping is not None
        else plat.default_damping
    )
    if args.output_dir is None:
        args.output_dir = Path(
            "oswec_external_pto_verification" if plat.key == "oswec"
            else "external_pto_verification"
        )

    exe = args.run_seastack
    if not args.skip_run and not Path(exe).exists():
        sys.stderr.write(f"run_seastack not found: {exe}\n")
        return 2

    cases = case_dirs(plat)
    twin_dir: Optional[Path] = None
    try:
        results = {}
        if not args.skip_run:
            for name, case_dir in cases:
                results[name] = run_case(exe, case_dir)
            twin_dir = make_native_twin(plat, damping)
            results["native"] = run_case(exe, twin_dir)
        else:
            for name, case_dir in cases:
                results[name] = find_results_h5(case_dir)
            twin_h5_dir = plat.demo_root / plat.twin_dirname / "outputs"
            native_candidates = (
                list(twin_h5_dir.glob("results.*.h5"))
                if twin_h5_dir.exists() else []
            )
            if native_candidates:
                results["native"] = max(
                    native_candidates, key=lambda p: p.stat().st_mtime
                )
            elif Path(exe).exists():
                print(f"Native twin missing; running {plat.native_label} "
                      "reference only…")
                twin_dir = make_native_twin(plat, damping)
                results["native"] = run_case(exe, twin_dir)
            else:
                print(f"WARNING: {plat.native_label} twin H5 not found; "
                      "linear verification will be INCOMPLETE")
                results["native"] = None

        cmd = [
            sys.executable, str(_PLOT),
            "--platform", plat.key,
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

        pdf = Path(args.output_dir).resolve() / plat.pdf_name
        print(f"\nVerification PDF: {pdf}")
        if args.open and pdf.is_file():
            try_open(pdf)
        return 0
    finally:
        if twin_dir is not None and twin_dir.exists() and not args.keep_twin:
            shutil.rmtree(twin_dir, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
