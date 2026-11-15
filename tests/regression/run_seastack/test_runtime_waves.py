#!/usr/bin/env python3
"""
Live wave-height changes through run_seastack --schedule (PR-5 gate).

The wave amplitude envelope e(t) multiplies every wave component in the
excitation (frequency domain, IRF convolution, nonlinear Froude-Krylov) and in
the drawn sea. A change ramps in over two wave periods.

Cases, copied to a temporary folder:
  - iea_sphere/regular_waves_lin:  regular H = 6 m, frequency-domain excitation;
  - rm3/irregular_waves:           irregular Hs = 2 m, IRF excitation (auto);
  - iea_sphere/regular_waves_nlfk: regular H = 6 m, nonlinear Froude-Krylov.

For each case:
  A) baseline run, no schedule;
  S) schedule setting the height to its own value at step 0;
  B) schedule halving the height at step CHANGE_STEP;
  C) B again (determinism).

Pass criteria:
  - S == A bit-for-bit over every /results dataset (e = 1 exactly);
  - C == B bit-for-bit over /results and /events;
  - B == A before the change and differs within a few steps after it;
  - /events in B records step, time, parameter, value and previous value;
  - linear cases: once the start-up ramp, the transition and the radiation
    and IRF memory have passed (t > LATE_S), every body velocity RMS in B is
    half that of A (relative error < 1e-3). Velocities rather than positions,
    because RM3 has no surge restoring force: the surge offset left by the
    transient does not decay.

Needs numpy and h5py (exit code 77, skipped, if missing). Usage:
    python test_runtime_waves.py --exe <path/to/run_seastack>
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

try:
    import h5py
    import numpy as np
except ImportError as exc:
    print(f"SKIP: {exc}")
    sys.exit(77)

_REPO = Path(__file__).resolve().parents[3]
_DEMOS = _REPO / "data" / "demos" / "run_seastack"
CHANGE_STEP = 1000

CASES = [
    # (name, case folder, height in the hydro YAML [m], end time [s], late window start [s] or None)
    ("sphere_fd", "iea_sphere/regular_waves_lin", 6.0, 120.0, 60.0),
    ("rm3_irf", "rm3/irregular_waves", 2.0, 180.0, 120.0),
    ("sphere_nlfk", "iea_sphere/regular_waves_nlfk", 6.0, 40.0, None),
]


def _run_env() -> dict:
    env = dict(os.environ)
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    return env


def prepare(root: Path, name: str, case: str, end_time: float) -> Path:
    """Copy the case folder and its platform assets; set end_time."""
    platform, sub = case.split("/")
    dst = root / name / platform
    shutil.copytree(_DEMOS / platform / "assets", dst / "assets",
                    ignore=shutil.ignore_patterns("eta"))
    case_dir = dst / sub
    shutil.copytree(_DEMOS / platform / sub, case_dir,
                    ignore=shutil.ignore_patterns("outputs", "expected"))
    sim = next(case_dir.glob("*.simulation.yaml"))
    text = sim.read_text(encoding="utf-8")
    text, n = re.subn(r"end_time:\s*[0-9.eE+-]+", f"end_time: {end_time}", text, count=1)
    if n != 1:
        raise RuntimeError(f"{sim}: no end_time")
    sim.write_text(text, encoding="utf-8")
    return case_dir


def write_schedule(path: Path, events: list[tuple[int, str, float]], dt: float) -> None:
    lines = ["schedule_version: 1", f"time_step: {dt!r}", "events:"]
    for step, parameter, value in events:
        lines.append(f"  - {{step: {step}, parameter: \"{parameter}\", value: {value!r}}}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(exe: str, case: Path, schedule: Path | None, extra_args: list[str] | None = None) -> Path | None:
    out = case / "outputs"
    shutil.rmtree(out, ignore_errors=True)
    cmd = [exe, "--nogui", "--quiet", str(case)]
    if schedule is not None:
        cmd += ["--schedule", str(schedule)]
    cmd += extra_args or []
    proc = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                          errors="replace", env=_run_env(), timeout=1800)
    if proc.returncode != 0:
        print("\n".join((proc.stdout + proc.stderr).strip().splitlines()[-30:]))
        return None
    h5 = next(out.glob("results.*.h5"))
    kept = case.parent / f"{case.name}_{len(list(case.parent.glob('*.h5')))}.h5"
    shutil.copy(h5, kept)
    return kept


def datasets(path: Path, group: str) -> dict:
    out = {}
    with h5py.File(path, "r") as f:
        if group not in f:
            return out
        def visit(name, obj):
            if isinstance(obj, h5py.Dataset):
                out[name] = obj[()]
        f[group].visititems(visit)
    return out


def identical(a: dict, b: dict) -> tuple[bool, str]:
    if a.keys() != b.keys():
        return False, f"different datasets: {sorted(set(a) ^ set(b))[:5]}"
    for k in a:
        x, y = np.asarray(a[k]), np.asarray(b[k])
        if x.dtype == object:
            same = x.shape == y.shape and x.tolist() == y.tolist()
        else:
            same = x.shape == y.shape and x.tobytes() == y.tobytes()
        if not same:
            return False, f"{k} differs"
    return True, ""


def first_differing_row(a: dict, b: dict) -> int:
    """Smallest output row where any time series differs (bitwise); -1 if none."""
    n = len(a["time/time"])
    first = -1
    for k, x in a.items():
        x, y = np.asarray(x), np.asarray(b[k])
        if x.ndim == 0 or x.shape[0] != n:
            continue
        rows = np.nonzero([x[i].tobytes() != y[i].tobytes() for i in range(n)])[0]
        if len(rows) and (first < 0 or rows[0] < first):
            first = int(rows[0])
    return first


def velocity_rms_ratios(a: dict, b: dict, t_from: float) -> list[tuple[str, float]]:
    """RMS(b) / RMS(a) of every body velocity component after t_from."""
    late = np.asarray(a["time/time"]) > t_from
    ratios = []
    for k in sorted(a):
        if not (k.startswith("model/bodies/") and k.endswith("/velocity")) or "/ground/" in k:
            continue
        va, vb = np.asarray(a[k])[late], np.asarray(b[k])[late]
        rms_a = np.sqrt(np.mean(va ** 2, axis=0))
        rms_b = np.sqrt(np.mean(vb ** 2, axis=0))
        for col, axis in enumerate("xyz"):
            # Skip directions with no wave forcing (constrained, or sway in head
            # seas): their RMS is solver noise, ~1e-6 m/s in RM3.
            if rms_a[col] < 1e-4 * rms_a.max():
                continue
            ratios.append((f"{k.split('/')[2]}.v{axis}", rms_b[col] / rms_a[col]))
    return ratios


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

    root = Path(tempfile.mkdtemp(prefix="seastack_waves_"))
    parameter = "waves.height"
    try:
        for name, case, height, end_time, late_s in CASES:
            print(f"{name}: {case}, {parameter} = {height:g} m")
            case_dir = prepare(root, name, case, end_time)
            sim_text = next(case_dir.glob("*.simulation.yaml")).read_text(encoding="utf-8")
            dt = float(re.search(r"time_step:\s*([0-9.eE+-]+)", sim_text).group(1))

            same = case_dir / "same.schedule.yaml"
            write_schedule(same, [(0, parameter, height)], dt)
            halved = case_dir / "halved.schedule.yaml"
            write_schedule(halved, [(CHANGE_STEP, parameter, 0.5 * height)], dt)

            h5_a = run(args.exe, case_dir, None)
            h5_s = run(args.exe, case_dir, same)
            h5_b = run(args.exe, case_dir, halved)
            h5_c = run(args.exe, case_dir, halved)
            check(None not in (h5_a, h5_s, h5_b, h5_c), "all four runs succeed")
            if None in (h5_a, h5_s, h5_b, h5_c):
                continue

            a, s, b, c = (datasets(h, "results") for h in (h5_a, h5_s, h5_b, h5_c))
            ok, why = identical(a, s)
            check(ok, f"same-height schedule is bit-identical to no schedule {why}")
            ok, why = identical(b, c)
            ok_ev, why_ev = identical(datasets(h5_b, "events"), datasets(h5_c, "events"))
            check(ok and ok_ev, f"schedule rerun is bit-identical {why} {why_ev}")

            row = first_differing_row(a, b)
            check(CHANGE_STEP <= row <= CHANGE_STEP + 5,
                  f"results unchanged before step {CHANGE_STEP}, differ just after (first differing row {row})")

            ev = datasets(h5_b, "events")
            ok = (len(ev.get("step", [])) == 1 and ev["step"][0] == CHANGE_STEP
                  and abs(ev["time"][0] - CHANGE_STEP * dt) < 1e-9
                  and ev["parameter"][0].decode() == parameter
                  and ev["value"][0] == 0.5 * height and ev["previous_value"][0] == height
                  and ev["origin"][0].decode() == "schedule")
            check(ok, f"/events records the change: {dict((k, v[:1]) for k, v in ev.items())}")

            if late_s is not None:
                ratios = velocity_rms_ratios(a, b, late_s)
                worst = max((abs(r - 0.5) / 0.5 for _, r in ratios), default=float("inf"))
                shown = ", ".join(f"{k} {r:.6f}" for k, r in ratios)
                check(bool(ratios) and worst < 1e-3,
                      f"steady response halves with the height (t > {late_s:g} s): {shown}")
    finally:
        shutil.rmtree(root, ignore_errors=True)

    if failures:
        print(f"\nFAILED: {len(failures)} check(s)")
        return 1
    print("\nRuntime wave checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
