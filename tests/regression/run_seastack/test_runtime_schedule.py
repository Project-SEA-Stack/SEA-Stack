#!/usr/bin/env python3
"""
Runtime parameter changes through run_seastack --schedule (PR-4 gate).

Cases, copied to a temporary folder and shortened to END_TIME_S:
  - rm3/regular_waves: native TSDA PTO damper (c = 1.2e6 N*s/m);
  - oswec/native_pto:  native RSDA PTO damper (c = 12e6 N*m*s/rad).

For each case:
  A) baseline run, no schedule;
  B) schedule doubling the PTO damping at step CHANGE_STEP;
  C) B again (determinism);
  D) schedule setting the damping to its own value at step 0, which swaps in
     SEA-Stack's editable functor without changing the coefficients.

Pass criteria:
  - D == A bit-for-bit over every /results dataset (the swap is exact);
  - C == B bit-for-bit over every /results dataset and /events;
  - B == A bit-for-bit before the change and differs after it;
  - /events in B records step, time, parameter, value and previous value;
  - after the change, PTO absorbed power uses the new damping (c v^2).

Needs numpy and h5py (exit code 77, skipped, if missing). Usage:
    python test_runtime_schedule.py --exe <path/to/run_seastack>
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
END_TIME_S = 12.0
CHANGE_STEP = 300

CASES = [
    # (name, case folder, parameter id, results group, damping, speed dataset)
    ("rm3", "rm3/regular_waves", "tsda.PTO.damping_coefficient",
     "results/model/tsdas/PTO", 1.2e6, "speed"),
    ("oswec", "oswec/native_pto", "rsda.PTO.damping_coefficient",
     "results/model/rsdas/PTO", 12e6, "ang_speed"),
]


def _run_env() -> dict:
    env = dict(os.environ)
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    return env


def prepare(root: Path, name: str, case: str) -> Path:
    """Copy the case folder and its platform assets; shorten end_time."""
    platform, sub = case.split("/")
    dst = root / name / platform
    shutil.copytree(_DEMOS / platform / "assets", dst / "assets",
                    ignore=shutil.ignore_patterns("meshes", "eta"))
    case_dir = dst / sub
    shutil.copytree(_DEMOS / platform / sub, case_dir,
                    ignore=shutil.ignore_patterns("outputs", "expected"))
    sim = next(case_dir.glob("*.simulation.yaml"))
    text = sim.read_text(encoding="utf-8")
    text, n = re.subn(r"end_time:\s*[0-9.eE+-]+", f"end_time: {END_TIME_S}", text, count=1)
    if n != 1:
        raise RuntimeError(f"{sim}: no end_time")
    sim.write_text(text, encoding="utf-8")
    return case_dir


def write_schedule(path: Path, events: list[tuple[int, str, float]], dt: float) -> None:
    lines = ["schedule_version: 1", f"time_step: {dt!r}", "events:"]
    for step, parameter, value in events:
        lines.append(f"  - {{step: {step}, parameter: \"{parameter}\", value: {value!r}}}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(exe: str, case: Path, schedule: Path | None) -> Path | None:
    out = case / "outputs"
    shutil.rmtree(out, ignore_errors=True)
    cmd = [exe, "--nogui", "--quiet", str(case)]
    if schedule is not None:
        cmd += ["--schedule", str(schedule)]
    proc = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                          errors="replace", env=_run_env(), timeout=900)
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
        if x.dtype == object:  # strings: compare values, not object pointers
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

    root = Path(tempfile.mkdtemp(prefix="seastack_schedule_"))
    try:
        for name, case, parameter, group, damping, speed_name in CASES:
            print(f"{name}: {case}, {parameter} = {damping:g}")
            case_dir = prepare(root, name, case)
            sim_text = next(case_dir.glob("*.simulation.yaml")).read_text(encoding="utf-8")
            dt = float(re.search(r"time_step:\s*([0-9.eE+-]+)", sim_text).group(1))

            doubled = case_dir / "doubled.schedule.yaml"
            write_schedule(doubled, [(CHANGE_STEP, parameter, 2.0 * damping)], dt)
            same = case_dir / "same.schedule.yaml"
            write_schedule(same, [(0, parameter, damping)], dt)

            h5_a = run(args.exe, case_dir, None)
            h5_b = run(args.exe, case_dir, doubled)
            h5_c = run(args.exe, case_dir, doubled)
            h5_d = run(args.exe, case_dir, same)
            check(None not in (h5_a, h5_b, h5_c, h5_d), "all four runs succeed")
            if None in (h5_a, h5_b, h5_c, h5_d):
                continue

            a, b, c, d = (datasets(h, "results") for h in (h5_a, h5_b, h5_c, h5_d))
            ok, why = identical(a, d)
            check(ok, f"same-value schedule is bit-identical to no schedule {why}")
            ok, why = identical(b, c)
            ok_ev, why_ev = identical(datasets(h5_b, "events"), datasets(h5_c, "events"))
            check(ok and ok_ev, f"schedule rerun is bit-identical {why} {why_ev}")

            # Output row i holds the state after step i + 1 (results start after
            # the first step), so a change before step CHANGE_STEP first shows
            # at row CHANGE_STEP (decimation 1).
            row = first_differing_row(a, b)
            check(row == CHANGE_STEP, f"results change from output row {CHANGE_STEP} on (first differing row {row})")

            ev = datasets(h5_b, "events")
            ok = (len(ev.get("step", [])) == 1 and ev["step"][0] == CHANGE_STEP
                  and abs(ev["time"][0] - CHANGE_STEP * dt) < 1e-9
                  and ev["parameter"][0].decode() == parameter
                  and ev["value"][0] == 2.0 * damping and ev["previous_value"][0] == damping
                  and ev["origin"][0].decode() == "schedule")
            check(ok, f"/events records the change: {dict((k, v[:1]) for k, v in ev.items())}")

            sub = group.removeprefix("results/")
            speed = np.asarray(b[f"{sub}/{speed_name}"])
            power = np.asarray(b[f"{sub}/absorbed_power"])
            before = np.allclose(power[:CHANGE_STEP], damping * speed[:CHANGE_STEP] ** 2, rtol=1e-12, atol=0)
            after = np.allclose(power[CHANGE_STEP:], 2.0 * damping * speed[CHANGE_STEP:] ** 2, rtol=1e-12, atol=0)
            check(before and after, "absorbed power = c v^2 with c before / after the change")
    finally:
        shutil.rmtree(root, ignore_errors=True)

    if failures:
        print(f"\nFAILED: {len(failures)} check(s)")
        return 1
    print("\nRuntime schedule checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
