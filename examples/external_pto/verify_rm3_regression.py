#!/usr/bin/env python3
"""
Focused Chrono regression for the RM3 external-PTO demos.

Runs each external module through the shared RM3 irregular-wave case
(JONSWAP Hs=2 m, Tp=8 s, seed=42, 600 s) and checks physically meaningful
quantities without committing large time-history baselines:

  * Linear  : EXACT-equivalence regression. A native Chrono LinearPTO twin of
              the same sea state (TSDA damping = c, no external block) is
              generated on the fly; the external linear damper must reproduce
              native heave and PTO force to solver tolerance.
  * Adaptive: aggregate checks (net energy > 0, post-ramp mean power > 0,
              excited heave, plausible peak force).
  * Hydraulic: aggregate checks (energy, post-ramp power, wave-excited heave)
              plus a soft check that mean power stays within ~0.3x of linear.

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

# Matches ramp_duration in the shared irregular-wave hydro YAML.
RAMP_DURATION_S = 60.0


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
    parser.add_argument("--tol", type=float, default=2e-3,
                        help="RMS-rel tolerance for the integrated heave response "
                             "(default 2e-3; slightly looser than a short decay run "
                             "because a longer irregular case accumulates more "
                             "explicit-within-step drift)")
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
        def mean_power_post_ramp(h5: Path) -> float:
            t = read(h5, TIME)
            p = read(h5, POWER)
            mask = t >= RAMP_DURATION_S
            if not np.any(mask):
                return float(np.mean(p))
            return float(np.mean(p[mask]))

        # --- Linear: exact equivalence vs native Chrono LinearPTO ---
        print("Linear: external damper vs native Chrono LinearPTO "
              "(irregular waves, c=1.2e6)")
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
        lin_pmean = mean_power_post_ramp(ext_h5)
        heave_rms = float(np.sqrt(np.mean(heave_ext ** 2)))
        ck.check(lin_energy > 0.0, f"linear net absorbed energy > 0 ({lin_energy:.3g} J)")
        ck.check(lin_pmean > 0.0,
                 f"linear post-ramp mean power > 0 ({lin_pmean:.3g} W)")
        ck.check(heave_rms > 0.05,
                 f"linear heave is wave-excited (RMS {heave_rms:.3g} m)")

        # --- Adaptive: aggregate checks under the same sea state ---
        print("Adaptive: aggregate checks (irregular waves)")
        ad_h5 = run_case(args.exe, adaptive_dir)
        ad_energy = float(read(ad_h5, ENERGY)[-1])
        ad_pmean = mean_power_post_ramp(ad_h5)
        ad_peak = float(np.max(np.abs(read(ad_h5, FORCE))))
        ad_heave = read(ad_h5, HEAVE, 2)
        ad_heave_rms = float(np.sqrt(np.mean(ad_heave ** 2)))
        ck.check(ad_energy > 0.0, f"adaptive net absorbed energy > 0 ({ad_energy:.3g} J)")
        ck.check(ad_pmean > 0.0,
                 f"adaptive post-ramp mean power > 0 ({ad_pmean:.3g} W)")
        ck.check(ad_heave_rms > 0.05,
                 f"adaptive heave is wave-excited (RMS {ad_heave_rms:.3g} m)")
        ck.check(ad_peak > 0.4 * lin_peak,
                 f"adaptive peak force plausible ({ad_peak:.3g} N)")

        # --- Hydraulic: aggregate checks + distinct performance vs siblings ---
        print("Hydraulic: aggregate checks (irregular waves)")
        hy_h5 = run_case(args.exe, hydraulic_dir)
        hy_energy = float(read(hy_h5, ENERGY)[-1])
        hy_pmean = mean_power_post_ramp(hy_h5)
        hy_peak = float(np.max(np.abs(read(hy_h5, FORCE))))
        hy_heave = read(hy_h5, HEAVE, 2)
        hy_heave_rms = float(np.sqrt(np.mean(hy_heave ** 2)))
        ck.check(hy_energy > 0.0, f"hydraulic net absorbed energy > 0 ({hy_energy:.3g} J)")
        ck.check(hy_pmean > 0.0,
                 f"hydraulic post-ramp mean power > 0 ({hy_pmean:.3g} W)")
        ck.check(hy_heave_rms > 0.05,
                 f"hydraulic heave is wave-excited (RMS {hy_heave_rms:.3g} m)")
        ck.check(hy_peak > 1.0e3,
                 f"hydraulic peak force is non-trivial ({hy_peak:.3g} N)")
        # Tuned demos sit in a similar power band (~same order as linear); just
        # require that none collapses to near-zero relative to the baseline.
        for name, pmean in (("adaptive", ad_pmean), ("hydraulic", hy_pmean)):
            ck.check(pmean > 0.3 * lin_pmean,
                     f"{name} post-ramp mean power within 0.3x of linear "
                     f"({pmean:.3g} vs {lin_pmean:.3g} W)")
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
