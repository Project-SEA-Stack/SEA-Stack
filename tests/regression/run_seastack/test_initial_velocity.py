#!/usr/bin/env python3
"""
Initial body velocity from the model YAML (initial_linear_velocity) through
run_seastack, on the IEA sphere heave decay case.

Two runs of iea_sphere/decay, copied to a temporary folder:
  A) offset decay: the shipped case (body 1 m above equilibrium, at rest);
  B) velocity decay: body at equilibrium with initial_linear_velocity
     [0, 0, V0_M_S] (world frame, centre of mass; Chrono 10.0 ChParserMbsYAML).

Pass criteria:
  - B leaves equilibrium with vertical velocity V0_M_S: at the first output
    step t1, (z - z_eq) / t1 and vz both equal V0_M_S within V0_REL_TOL;
  - B moves up first (sign convention: z positive up);
  - B decays with the same damped period as A, within PERIOD_TOL_PCT, from
    the mean spacing of upward zero crossings of heave about equilibrium.

Needs numpy and h5py (exit code 77, skipped, if missing). Usage:
    python test_initial_velocity.py --exe <path/to/run_seastack>
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

try:
    import h5py
    import numpy as np
except ImportError as exc:
    print(f"SKIP: {exc}")
    sys.exit(77)

_REPO = Path(__file__).resolve().parents[3]
_SPHERE = _REPO / "data" / "demos" / "run_seastack" / "iea_sphere"
V0_M_S = 0.5           # initial heave velocity [m/s], positive up
Z_EQ_M = -2.0          # equilibrium CG height of the sphere [m]
PERIOD_TOL_PCT = 2.0
V0_REL_TOL = 1e-3      # over the first step the hydrostatic deceleration is O(1e-4) V0


def _run_env() -> dict:
    env = dict(os.environ)
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    return env


def prepare(root: Path, name: str, velocity: bool) -> Path:
    """Copy iea_sphere/decay (and its assets) under root/name; optionally switch to a velocity start."""
    sphere = root / name
    shutil.copytree(_SPHERE / "assets", sphere / "assets",
                    ignore=shutil.ignore_patterns("meshes", "*.obj", "eta"))
    case = sphere / "decay"
    shutil.copytree(_SPHERE / "decay", case, ignore=shutil.ignore_patterns("outputs", "expected"))
    if velocity:
        model = case / "iea_sphere_decay.model.yaml"
        text = model.read_text(encoding="utf-8")
        old = "      location: [0, 0, -1.0]\n"
        if old not in text:
            raise RuntimeError(f"{model}: expected body1 '{old.strip()}'")
        new = (f"      location: [0, 0, {Z_EQ_M}]\n"
               f"      initial_linear_velocity: [0, 0, {V0_M_S}]\n")
        model.write_text(text.replace(old, new, 1), encoding="utf-8")
    return case


def run(exe: str, case: Path):
    proc = subprocess.run([exe, "--nogui", "--quiet", str(case)], capture_output=True, text=True,
                          encoding="utf-8", errors="replace", env=_run_env(), timeout=600)
    if proc.returncode != 0:
        print("\n".join((proc.stdout + proc.stderr).strip().splitlines()[-30:]))
        return None
    h5 = next((case / "outputs").glob("results.*.h5"))
    with h5py.File(h5, "r") as f:
        t = f["results/time/time"][:]
        z = f["results/model/bodies/body1/position"][:, 2]
        vz = f["results/model/bodies/body1/velocity"][:, 2]
    return t, z, vz


def damped_period(t, z) -> float:
    """Mean spacing of upward zero crossings of z - Z_EQ_M (linear interpolation)."""
    y = z - Z_EQ_M
    idx = np.where((y[:-1] < 0.0) & (y[1:] >= 0.0))[0]
    tc = t[idx] - y[idx] * (t[idx + 1] - t[idx]) / (y[idx + 1] - y[idx])
    if len(tc) < 3:
        return float("nan")
    return float(np.mean(np.diff(tc[:4])))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--exe", required=True, help="path to run_seastack")
    args = parser.parse_args()

    failures = []

    def check(cond: bool, msg: str) -> None:
        print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
        if not cond:
            failures.append(msg)

    root = Path(tempfile.mkdtemp(prefix="seastack_initvel_"))
    try:
        offset = run(args.exe, prepare(root, "offset", velocity=False))
        velocity = run(args.exe, prepare(root, "velocity", velocity=True))
        check(offset is not None, "offset decay runs")
        check(velocity is not None, "velocity decay runs")
        if offset is None or velocity is None:
            return 1
        # The output starts at the first step t1 = time_step, not at t = 0.
        t, z, vz = velocity
        t1 = t[0]
        mean_v = (z[0] - Z_EQ_M) / t1
        check(abs(mean_v / V0_M_S - 1.0) < V0_REL_TOL,
              f"first step from equilibrium: (z(t1) - z_eq) / t1 = {mean_v:.6f} m/s = V0 = {V0_M_S}")
        check(abs(vz[0] / V0_M_S - 1.0) < V0_REL_TOL,
              f"heave velocity at t1 = {t1:g} s: {vz[0]:.6f} m/s = V0 within {100 * V0_REL_TOL:g} %")
        first = int(np.argmax(np.abs(z - Z_EQ_M) > 1e-3))
        check(z[first] > Z_EQ_M, "moves up first (z positive up)")
        t_off = damped_period(*offset[:2])
        t_vel = damped_period(t, z)
        diff_pct = 100.0 * (t_vel / t_off - 1.0)
        print(f"  damped period: offset {t_off:.3f} s, velocity {t_vel:.3f} s ({diff_pct:+.2f} %)")
        check(abs(diff_pct) < PERIOD_TOL_PCT, f"same damped period within {PERIOD_TOL_PCT:g} %")
    finally:
        shutil.rmtree(root, ignore_errors=True)

    if failures:
        print(f"\nFAILED: {len(failures)} check(s)")
        return 1
    print("\nInitial velocity checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
