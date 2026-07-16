#!/usr/bin/env python3
"""
Adaptive-damping external PTO — step up from the linear damper.

Starts from the same law as linear_damper_pto.py:

    F = -c * v

and adds three small ideas:

  1. c is no longer fixed: a PI loop raises/lowers it so |v| tracks a setpoint
  2. the force is clipped to +/- force_max (saturation)
  3. reset() clears the controller state (integral + c)

That is enough to show a *stateful* external module. There is no spring,
coulomb friction, or sliding-window history — those belong in a more advanced
controller if you need them.

Law (extension positive, force opposes motion):

    err   = v_setpoint - |v|
    c     = clip(c0 - kp*err - ki*integral(err), c_min, c_max)
    F     = clip(-c * v, -force_max, +force_max)

Config keys (YAML `external_pto.config`):
  damping, force_max, kp, ki, vel_setpoint, c_min, c_max
  diagnostics_csv  optional per-step CSV (for verification plots)

seastack_external.py is only the IPC helper; keep your model in this file.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path
from typing import Any, Dict, Optional, TextIO

_here = Path(__file__).resolve().parent
if str(_here) not in sys.path:
    sys.path.insert(0, str(_here))
for parent in _here.parents:
    for candidate in (
        parent / "python",
        parent / "libs" / "external" / "python",
    ):
        if candidate.is_dir() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))

from seastack_external import PtoModule, PtoState, run


class AdaptiveDampingPTO(PtoModule):
    """Linear damper with PI-adapted c(t), force saturation, and reset()."""

    def setup(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        self.c0 = float(cfg.get("damping", 1.2e6))
        self.f_max = float(cfg.get("force_max", 5.0e6))
        self.kp = float(cfg.get("kp", 2.0e5))
        self.ki = float(cfg.get("ki", 5.0e4))
        self.vel_setpoint = float(cfg.get("vel_setpoint", 0.5))
        self.c_min = float(cfg.get("c_min", 1.0e5))
        self.c_max = float(cfg.get("c_max", 5.0e6))

        self._diag_path = str(cfg.get("diagnostics_csv", "") or "").strip()
        self._diag_fh: Optional[TextIO] = None
        self._diag_writer: Optional[csv.writer] = None
        if self._diag_path:
            path = Path(self._diag_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._diag_fh = open(path, "w", newline="", encoding="utf-8")
            self._diag_writer = csv.writer(self._diag_fh)
            self._diag_writer.writerow([
                "time", "displacement", "velocity", "force",
                "damping_c", "integral", "peak_abs_vel",
                "force_max", "saturated", "saturation_events",
            ])

        self.reset()
        return {
            "name": "AdaptiveDampingPTO",
            "version": "1.0",
            "n_states": 2,  # integral + adapted damping
        }

    def reset(self) -> None:
        self.c = self.c0
        self._integral = 0.0
        self.saturation_events = 0

    def force(self, state: PtoState) -> float:
        v = state.velocity
        dt = state.dt

        # --- adapt c (the only new state vs the linear damper) -------------
        if dt > 0.0:
            err = self.vel_setpoint - abs(v)  # +err => motion too small => lower c
            self._integral += err * dt
            u = self.c0 - self.kp * err - self.ki * self._integral
            # Anti-windup: undo the integral step when c hits its limits.
            if u > self.c_max:
                self._integral -= err * dt
                u = self.c_max
            elif u < self.c_min:
                self._integral -= err * dt
                u = self.c_min
            self.c = u

        # --- same F = -c v as the linear damper, then saturate -------------
        f_cmd = -self.c * v
        force = max(-self.f_max, min(self.f_max, f_cmd))
        saturated = 1 if force != f_cmd else 0
        if saturated:
            self.saturation_events += 1

        if self._diag_writer is not None:
            self._diag_writer.writerow([
                f"{state.time:.10g}", f"{state.displacement:.10g}",
                f"{v:.10g}", f"{force:.10g}",
                f"{self.c:.10g}", f"{self._integral:.10g}", f"{abs(v):.10g}",
                f"{self.f_max:.10g}", saturated, self.saturation_events,
            ])

        return force

    def shutdown(self) -> None:
        if self._diag_fh is not None:
            self._diag_fh.close()
            self._diag_fh = None
            self._diag_writer = None


if __name__ == "__main__":
    run(AdaptiveDampingPTO())
