#!/usr/bin/env python3
"""
Common prescribed-input replay harness for SEA-Stack external PTO modules.

Replays a (time, displacement, velocity) trace through any ExternalForceModule
subclass *in-process* (no IPC, no Chrono, no full RM3 run) and reports, per
sample and in aggregate:

  - force            F [N] returned by the module
  - mechanical power P_abs = -F v [W]   (power absorbed from the device)
  - cumulative absorbed energy E = integral P_abs dt [J]  (trapezoidal)
  - peak |force| [N]
  - saturation / limit events, when the module exposes them

This is the shared oracle used by verify_examples.py and is handy for quickly
sanity-checking a new module. It intentionally does not spawn the IPC server:
it exercises the model's physics and state machine directly.

Usage (CLI):
  python replay_harness.py --module hydraulic_accumulator_pto.py \
      --class HydraulicAccumulatorPTO --amplitude 0.8 --period 6 --duration 12
  python replay_harness.py --module adaptive_damping_pto.py \
      --class AdaptiveDampingPTO --input trace.csv --config '{"damping": 8e5}'
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


# A sample is (time [s], dt [s], displacement [m], velocity [m/s]).
Sample = Tuple[float, float, float, float]


def make_sinusoid(
    amplitude: float = 0.8,
    period: float = 6.0,
    duration: float = 12.0,
    dt: float = 0.01,
) -> List[Sample]:
    """Sinusoidal heave: x = A sin(w t), v = A w cos(w t)."""
    w = 2.0 * math.pi / period
    n = int(round(duration / dt))
    samples: List[Sample] = []
    for i in range(1, n + 1):
        t = i * dt
        x = amplitude * math.sin(w * t)
        v = amplitude * w * math.cos(w * t)
        samples.append((t, dt, x, v))
    return samples


def read_csv_trace(path: Path) -> List[Sample]:
    """Read a CSV with header columns time, displacement, velocity.

    dt is derived from consecutive time stamps (first dt reuses the second).
    """
    rows: List[Tuple[float, float, float]] = []
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            rows.append(
                (
                    float(row["time"]),
                    float(row["displacement"]),
                    float(row["velocity"]),
                )
            )
    samples: List[Sample] = []
    for i, (t, x, v) in enumerate(rows):
        if i == 0:
            dt = rows[1][0] - rows[0][0] if len(rows) > 1 else 0.0
        else:
            dt = t - rows[i - 1][0]
        samples.append((t, dt, x, v))
    return samples


class ReplayResult:
    """Per-sample arrays plus aggregate summary from a replay."""

    def __init__(self) -> None:
        self.t: List[float] = []
        self.disp: List[float] = []
        self.vel: List[float] = []
        self.force: List[float] = []
        self.power: List[float] = []       # P_abs = -F v
        self.energy: List[float] = []      # cumulative absorbed energy
        self.meta: Dict[str, Any] = {}
        self.diagnostics: Dict[str, Any] = {}

    @property
    def n(self) -> int:
        return len(self.t)

    def summary(self) -> Dict[str, Any]:
        if not self.t:
            return {}
        peak_force = max(abs(f) for f in self.force)
        mean_power = sum(self.power) / len(self.power)
        return {
            "samples": self.n,
            "mean_power_W": mean_power,
            "absorbed_energy_J": self.energy[-1],
            "peak_force_N": peak_force,
            "final_force_N": self.force[-1],
            **{k: v for k, v in self.diagnostics.items()},
        }


def replay(
    module: Any,
    config: Dict[str, Any],
    trace: Sequence[Sample],
    reset_first: bool = True,
) -> ReplayResult:
    """Initialize `module` with `config` and replay `trace` through it."""
    meta = module.initialize(dict(config)) or {}
    if reset_first:
        module.reset()
    result = ReplayResult()
    result.meta = dict(meta)

    e_cum = 0.0
    prev_p: Optional[float] = None
    prev_t: Optional[float] = None
    for (t, dt, x, v) in trace:
        out = module.evaluate(t, dt, [x, v])
        f = float(out[0])
        p = -f * v  # power absorbed from the device
        if prev_p is not None and prev_t is not None:
            e_cum += 0.5 * (p + prev_p) * (t - prev_t)
        prev_p, prev_t = p, t

        result.t.append(t)
        result.disp.append(x)
        result.vel.append(v)
        result.force.append(f)
        result.power.append(p)
        result.energy.append(e_cum)

    if hasattr(module, "diagnostics"):
        try:
            result.diagnostics = dict(module.diagnostics())
        except Exception:  # noqa: BLE001 — diagnostics are best-effort
            pass
    for attr in ("saturation_events", "relief_events", "limit_hits"):
        if hasattr(module, attr):
            result.diagnostics.setdefault(attr, getattr(module, attr))
    return result


def _load_module_class(module_path: Path, class_name: str) -> Any:
    spec = importlib.util.spec_from_file_location(module_path.stem, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load module from {module_path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return getattr(mod, class_name)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module", required=True, type=Path,
                        help="Path to the external PTO .py file")
    parser.add_argument("--class", dest="class_name", required=True,
                        help="ExternalForceModule subclass name")
    parser.add_argument("--config", default="{}",
                        help="JSON config passed to initialize()")
    parser.add_argument("--input", type=Path, default=None,
                        help="CSV trace (time,displacement,velocity); "
                             "defaults to a built-in sinusoid")
    parser.add_argument("--amplitude", type=float, default=0.8)
    parser.add_argument("--period", type=float, default=6.0)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--out", type=Path, default=None,
                        help="Optional CSV output of per-sample results")
    args = parser.parse_args(argv)

    cls = _load_module_class(args.module, args.class_name)
    config = json.loads(args.config) if args.config else {}
    trace = (read_csv_trace(args.input) if args.input
             else make_sinusoid(args.amplitude, args.period, args.duration, args.dt))

    result = replay(cls(), config, trace)

    if args.out:
        with open(args.out, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["time", "displacement", "velocity",
                             "force", "power", "energy"])
            for i in range(result.n):
                writer.writerow([result.t[i], result.disp[i], result.vel[i],
                                 result.force[i], result.power[i], result.energy[i]])

    print(json.dumps(result.summary(), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
