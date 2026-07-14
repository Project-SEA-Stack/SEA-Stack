#!/usr/bin/env python3
"""
Focused Chrono regression for the RM3 external-PTO demos.

Runs the full RM3 decay case through run_seastack for each external module and
checks physically meaningful quantities without committing large time-history
baselines:

  * Linear  : EXACT-equivalence regression. A native Chrono LinearPTO twin of
              the same case (TSDA damping = c, no external block) is generated
              on the fly and run; the external linear damper must reproduce the
              native heave and PTO force to solver tolerance.
  * Adaptive: aggregate checks (net energy absorbed > 0, plausible peak force,
              motion decays).
  * Hydraulic: aggregate checks + cross-case ordering (accumulator preload makes
              the hydraulic peak force clearly exceed the linear damper's).

Skips (exit 77) if h5py is unavailable. Requires run_seastack (Chrono) and a
Python 3 interpreter on PATH (the demos spawn `python <module>.py`).

Usage:
  verify_rm3_regression.py --exe <run_seastack> [--tol 1e-3]
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


def _run_env() -> dict:
    """Environment for the run_seastack subprocess.

    ctest passes the Chrono/HDF5/VSG/interpreter DLL directories via
    SEASTACK_RUN_DLL_DIRS ('|'-separated) so we can inject them onto the child's
    PATH *without* polluting this script's own interpreter (which imports h5py
    and would otherwise pick up conflicting native DLLs).
    """
    env = os.environ.copy()
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    return env

try:
    import h5py  # type: ignore
    import numpy as np
except Exception:  # noqa: BLE001
    sys.stderr.write("verify_rm3_regression: h5py/numpy unavailable; skipping\n")
    sys.exit(77)

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parents[1]
_RM3 = _REPO / "data" / "demos" / "run_seastack" / "rm3"

HEAVE = "/results/model/bodies/body1/position"
FORCE = "/results/model/tsdas/PTO/force_mag"
POWER = "/results/model/tsdas/PTO/absorbed_power"
ENERGY = "/results/model/tsdas/PTO/absorbed_energy"
TIME = "/results/time/time"


class Checker:
    def __init__(self) -> None:
        self.failures = []

    def check(self, cond: bool, msg: str) -> None:
        print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
        if not cond:
            self.failures.append(msg)


def run_case(exe: str, case_dir: Path) -> Path:
    setups = list(case_dir.glob("*.setup.yaml"))
    if not setups:
        raise FileNotFoundError(f"no setup yaml in {case_dir}")
    cmd = [exe, "--nogui", "--quiet", str(case_dir)]
    proc = subprocess.run(cmd, capture_output=True, text=True,
                          encoding="utf-8", errors="ignore", env=_run_env())
    if proc.returncode != 0:
        raise RuntimeError(f"run_seastack failed for {case_dir.name}:\n"
                           f"{proc.stdout}\n{proc.stderr}")
    out = list((case_dir / "outputs").glob("results.*.h5"))
    if not out:
        raise FileNotFoundError(f"no results h5 for {case_dir.name}")
    return out[0]


def read(path: Path, dset: str, col: int | None = None) -> "np.ndarray":
    with h5py.File(path, "r") as f:
        arr = np.array(f[dset])
    if col is not None and arr.ndim == 2:
        return arr[:, col]
    return arr


def rms_rel(ref: "np.ndarray", sim: "np.ndarray") -> float:
    ref_rms = float(np.sqrt(np.mean(ref ** 2)))
    if ref_rms == 0.0:
        return float(np.sqrt(np.mean(sim ** 2)))
    return float(np.sqrt(np.mean((sim - ref) ** 2)) / ref_rms)


def make_native_twin(damping: float) -> Path:
    """Create a native-LinearPTO twin of the linear external case."""
    src = _RM3 / "external_pto"
    twin = _RM3 / "_regression_native_linpto"
    if twin.exists():
        shutil.rmtree(twin)
    twin.mkdir()

    # Copy sim + hydro unchanged; edit model (set TSDA damping, no external).
    for suffix, out in (("simulation", "simulation"), ("hydro", "hydro")):
        shutil.copy2(src / f"rm3_external_pto.{suffix}.yaml",
                     twin / f"rm3_native_linpto.{out}.yaml")
    model = (src / "rm3_external_pto.model.yaml").read_text()
    model = re.sub(r"damping_coefficient:\s*\S+.*",
                   f"damping_coefficient: {damping:.1f}", model, count=1)
    (twin / "rm3_native_linpto.model.yaml").write_text(model)
    (twin / "rm3_native_linpto.setup.yaml").write_text(
        "model_file: rm3_native_linpto.model.yaml\n"
        "simulation_file: rm3_native_linpto.simulation.yaml\n"
        "hydro_file: rm3_native_linpto.hydro.yaml\n"
        "output_directory: outputs\n")
    return twin


def cleanup(*dirs: Path) -> None:
    for d in dirs:
        out = d / "outputs"
        if out.exists():
            shutil.rmtree(out, ignore_errors=True)
    twin = _RM3 / "_regression_native_linpto"
    if twin.exists():
        shutil.rmtree(twin, ignore_errors=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", required=True)
    parser.add_argument("--tol", type=float, default=1e-3,
                        help="RMS-rel tolerance for the integrated heave response")
    parser.add_argument("--force-tol", type=float, default=2e-2,
                        help="RMS-rel tolerance for instantaneous PTO force. Looser "
                             "than heave because the external module is frozen per "
                             "accepted step (explicit-within-step) while the native "
                             "Chrono TSDA damper is re-evaluated every HHT iteration.")
    args = parser.parse_args()

    ck = Checker()
    linear_dir = _RM3 / "external_pto"
    adaptive_dir = _RM3 / "external_pto_adaptive"
    hydraulic_dir = _RM3 / "external_pto_hydraulic"
    twin_dir = None
    try:
        # --- Linear: exact equivalence vs native Chrono LinearPTO ---
        print("Linear: external damper vs native Chrono LinearPTO (c=1.2e6)")
        ext_h5 = run_case(args.exe, linear_dir)
        twin_dir = make_native_twin(1.2e6)
        nat_h5 = run_case(args.exe, twin_dir)

        t_ext = read(ext_h5, TIME)
        t_nat = read(nat_h5, TIME)
        heave_ext = read(ext_h5, HEAVE, 2)
        heave_nat = np.interp(t_ext, t_nat, read(nat_h5, HEAVE, 2))
        force_ext = read(ext_h5, FORCE)
        force_nat = np.interp(t_ext, t_nat, read(nat_h5, FORCE))

        he = rms_rel(heave_nat, heave_ext)
        fe = rms_rel(force_nat, force_ext)
        ck.check(he <= args.tol, f"heave matches native LinearPTO (RMSrel {he:.3g})")
        ck.check(fe <= args.force_tol,
                 f"PTO force matches native LinearPTO within explicit-step "
                 f"tolerance (RMSrel {fe:.3g})")

        lin_peak = float(np.max(np.abs(force_ext)))
        lin_energy = float(read(ext_h5, ENERGY)[-1])
        ck.check(lin_energy > 0.0, f"linear net absorbed energy > 0 ({lin_energy:.3g} J)")

        # --- Adaptive: aggregate checks ---
        print("Adaptive: aggregate checks")
        ad_h5 = run_case(args.exe, adaptive_dir)
        ad_energy = float(read(ad_h5, ENERGY)[-1])
        ad_power = read(ad_h5, POWER)
        ad_peak = float(np.max(np.abs(read(ad_h5, FORCE))))
        ad_heave = read(ad_h5, HEAVE, 2)
        ck.check(ad_energy > 0.0, f"adaptive net absorbed energy > 0 ({ad_energy:.3g} J)")
        ck.check(float(np.mean(ad_power)) > 0.0, "adaptive mean absorbed power > 0")
        ck.check(abs(ad_heave[-1]) < abs(np.min(ad_heave)) + 1e-6
                 and abs(ad_heave[-1]) > 0.3, "adaptive heave decays to a settled offset")
        ck.check(ad_peak > 0.4 * lin_peak, f"adaptive peak force plausible ({ad_peak:.3g} N)")

        # --- Hydraulic: aggregate checks + cross-case ordering ---
        print("Hydraulic: aggregate checks + ordering vs linear")
        hy_h5 = run_case(args.exe, hydraulic_dir)
        hy_energy = float(read(hy_h5, ENERGY)[-1])
        hy_power = read(hy_h5, POWER)
        hy_peak = float(np.max(np.abs(read(hy_h5, FORCE))))
        ck.check(hy_energy > 0.0, f"hydraulic net absorbed energy > 0 ({hy_energy:.3g} J)")
        ck.check(float(np.mean(hy_power)) > 0.0, "hydraulic mean absorbed power > 0")
        ck.check(hy_peak > 1.5 * lin_peak,
                 f"hydraulic peak force exceeds linear (accumulator preload): "
                 f"{hy_peak:.3g} vs {lin_peak:.3g} N")
    except Exception as e:  # noqa: BLE001
        sys.stderr.write(f"ERROR: {e}\n")
        return 1
    finally:
        cleanup(linear_dir, adaptive_dir, hydraulic_dir,
                *( [twin_dir] if twin_dir else [] ))

    print()
    if ck.failures:
        print(f"FAILED: {len(ck.failures)} check(s)")
        return 1
    print("RM3 external-PTO regression passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
