#!/usr/bin/env python3
"""
IPC-level equivalence check for a SEA-Stack external PTO module.

The golden tests (verify_examples.py) exercise the module physics *in-process*
and bypass the wire protocol. This script closes that gap: it runs the same
prescribed (time, displacement, velocity) trace through

  1. the full IPC transport  (external_pto_example --replay, which drives
     ExternalPtoModel + IpcExternalForceModel over TCP), and
  2. a direct in-process evaluation of the same module,

and asserts the two force histories agree. This verifies framing, JSON
round-tripping, config passing and the C++ time-caching, independent of Chrono.

Usage:
  compare_ipc_replay.py --exe <external_pto_example> --module <script.py> \
      --class <ClassName> [--config JSON] [--dt 0.01] \
      [--amplitude 0.8] [--period 6] [--duration 12]
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import List

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from replay_harness import make_sinusoid, replay, _load_module_class  # noqa: E402


def _read_force_csv(path: Path) -> List[float]:
    forces: List[float] = []
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            forces.append(float(row["force"]))
    return forces


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", required=True, type=Path)
    parser.add_argument("--module", required=True, type=Path)
    parser.add_argument("--class", dest="class_name", required=True)
    parser.add_argument("--config", default="{}")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--amplitude", type=float, default=0.8)
    parser.add_argument("--period", type=float, default=6.0)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--rtol", type=float, default=1e-9)
    parser.add_argument("--atol", type=float, default=1e-6)
    args = parser.parse_args()

    config = json.loads(args.config) if args.config else {}
    trace = make_sinusoid(args.amplitude, args.period, args.duration, args.dt)

    # 1) In-process reference (imports the module directly).
    cls = _load_module_class(args.module, args.class_name)
    ref = replay(cls(), config, trace).force

    with tempfile.TemporaryDirectory() as tmp:
        tmp_dir = Path(tmp)
        trace_csv = tmp_dir / "trace.csv"
        out_csv = tmp_dir / "ipc_force.csv"
        with open(trace_csv, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["time", "displacement", "velocity"])
            for (t, _dt, x, v) in trace:
                writer.writerow([repr(t), repr(x), repr(v)])

        # 2) IPC path through the C++ example.
        cmd = [
            str(args.exe), "--replay",
            "--python", str(args.module),
            "--config", args.config,
            "--dt", repr(args.dt),
            "--trace", str(trace_csv),
            "--out", str(out_csv),
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            sys.stderr.write(f"IPC replay failed ({proc.returncode}):\n"
                             f"{proc.stdout}\n{proc.stderr}\n")
            return 1
        ipc = _read_force_csv(out_csv)

    if len(ipc) != len(ref):
        sys.stderr.write(f"length mismatch: ipc={len(ipc)} ref={len(ref)}\n")
        return 1

    max_err = 0.0
    for a, b in zip(ipc, ref):
        err = abs(a - b)
        tol = args.atol + args.rtol * abs(b)
        max_err = max(max_err, err - tol)
        if err > tol:
            sys.stderr.write(
                f"force mismatch: ipc={a:.9g} ref={b:.9g} err={err:.3g} tol={tol:.3g}\n")
            return 1

    print(f"IPC == in-process for {args.module.name} "
          f"({len(ref)} samples, max margin {max_err:.3g})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
