#!/usr/bin/env python3
"""
Adaptive-damping external PTO for SEA-Stack (stateful example).

Force law (extension positive, force opposes motion):

    F_cmd = -k * x - c(t) * v - f_c * tanh(v / v_eps)
    F     = clip(F_cmd, -F_max, +F_max)

The viscous coefficient c(t) is adapted online by a PI law that tracks a target
peak-|v| amplitude measured over a short sliding window. This is a deliberately
simple *variable-damping* controller: it does not return net energy to the
device, so it is not "reactive control" in the WEC sense (which intentionally
supplies energy over part of the cycle). Keeping it PI-only (no derivative
term) avoids sensitivity to discretely sampled velocity while still exercising:

  - stateful evaluation across accepted time steps,
  - a history buffer (sliding-window peak measurement),
  - integral state with anti-windup,
  - force saturation,
  - reset() semantics,
  - configuration passing.

Config keys (JSON via setup YAML `external_pto.config`):
  stiffness      spring k [N/m]              (default 0)
  damping        initial viscous c [N.s/m]   (default 1.2e6)
  coulomb        coulomb magnitude [N]       (default 0)
  force_max      |F| saturation [N]          (default 5e6)
  kp, ki         PI gains on the |v| error
  vel_setpoint   target |v| amplitude [m/s]  (default 0.5)
  window_s       peak-|v| window length [s]  (default 4.0)
  c_min, c_max   damping clamp [N.s/m]
  v_eps          coulomb smoothing scale [m/s] (default 0.05)
  diagnostics_csv  optional path for per-step controller CSV (overwrite)

Units / signs match IPTOModel: extension positive, force opposes motion.

Coulomb is a *smoothed* friction term, f_c * tanh(v / v_eps), not a hard
sign(v). A discontinuous coulomb law chatters or steps when Chrono velocity
noise crosses zero; tanh keeps the force continuous while still opposing motion.

The physics live in this file. seastack_external.py is only the IPC helper.
"""

from __future__ import annotations

import csv
import math
import sys
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, TextIO, Tuple

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

from seastack_external import ExternalForceModule


class AdaptiveDampingPTO(ExternalForceModule):
    """Spring + PI-adapted viscous damping + smoothed coulomb + force saturation."""

    def initialize(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        self.k = float(cfg.get("stiffness", 0.0))
        self.c = float(cfg.get("damping", 1.2e6))
        self.f_c = float(cfg.get("coulomb", 0.0))
        self.f_max = float(cfg.get("force_max", 5.0e6))
        self.kp = float(cfg.get("kp", 2.0e5))
        self.ki = float(cfg.get("ki", 5.0e4))
        self.vel_setpoint = float(cfg.get("vel_setpoint", 0.5))
        self.window_s = float(cfg.get("window_s", 4.0))
        self.c_min = float(cfg.get("c_min", 1.0e5))
        self.c_max = float(cfg.get("c_max", 5.0e6))
        # Smoothing scale for coulomb: F_c * tanh(v / v_eps).
        self.v_eps = max(float(cfg.get("v_eps", 0.05)), 1.0e-6)

        self._c0 = self.c
        self._diag_path = str(cfg.get("diagnostics_csv", "") or "").strip()
        self._diag_fh: Optional[TextIO] = None
        self._diag_writer: Optional[csv.writer] = None
        self._open_diagnostics()
        self._reset_state()
        # Count of saturation events (force clipped) — diagnostic only.
        self.saturation_events = 0
        return {
            "name": "AdaptiveDampingPTO",
            "version": "1.0",
            "n_states": 2,  # integral error + adapted damping
        }

    def _open_diagnostics(self) -> None:
        """Open (overwrite) an example-only controller CSV if configured."""
        if not self._diag_path:
            return
        path = Path(self._diag_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._diag_fh = open(path, "w", newline="", encoding="utf-8")
        self._diag_writer = csv.writer(self._diag_fh)
        self._diag_writer.writerow([
            "time", "displacement", "velocity", "force",
            "damping_c", "integral", "peak_abs_vel",
            "force_max", "saturated", "saturation_events",
        ])
        self._diag_fh.flush()

    def _close_diagnostics(self) -> None:
        if self._diag_fh is not None:
            self._diag_fh.close()
            self._diag_fh = None
            self._diag_writer = None

    def _reset_state(self) -> None:
        self._integral = 0.0
        self.c = self._c0
        # (time, |vel|) samples for the sliding-window peak estimate
        self._history: Deque[Tuple[float, float]] = deque()
        self._last_t = None  # type: float | None
        self._step_count = 0
        self.saturation_events = 0
        self._last_peak = 0.0

    def reset(self) -> None:
        self._reset_state()

    def _peak_abs_vel(self, t: float, abs_vel: float) -> float:
        self._history.append((t, abs_vel))
        t_cut = t - self.window_s
        while self._history and self._history[0][0] < t_cut:
            self._history.popleft()
        if not self._history:
            return abs_vel
        return max(v for _, v in self._history)

    def _adapt_damping(self, t: float, dt: float, vel: float) -> None:
        if dt <= 0.0:
            return
        peak = self._peak_abs_vel(t, abs(vel))
        self._last_peak = peak
        err = self.vel_setpoint - peak  # positive => want more motion => lower c
        self._integral += err * dt
        # Anti-windup: hold the integrator when the command saturates.
        u = self._c0 - self.kp * err - self.ki * self._integral
        if u > self.c_max:
            self._integral -= err * dt
            u = self.c_max
        elif u < self.c_min:
            self._integral -= err * dt
            u = self.c_min
        self.c = u

    def evaluate(self, t: float, dt: float, inputs: List[float]) -> List[float]:
        disp, vel = inputs[0], inputs[1]

        # Prefer host-provided dt; fall back to wall clock between evaluates.
        step_dt = dt
        if step_dt <= 0.0 and self._last_t is not None:
            step_dt = max(0.0, t - self._last_t)

        self._adapt_damping(t, step_dt, vel)
        self._last_t = t
        self._step_count += 1

        f_cmd = -self.k * disp - self.c * vel - self.f_c * math.tanh(vel / self.v_eps)
        force = max(-self.f_max, min(self.f_max, f_cmd))
        saturated = 1 if force != f_cmd else 0
        if saturated:
            self.saturation_events += 1

        if self._diag_writer is not None:
            self._diag_writer.writerow([
                f"{t:.10g}", f"{disp:.10g}", f"{vel:.10g}", f"{force:.10g}",
                f"{self.c:.10g}", f"{self._integral:.10g}", f"{self._last_peak:.10g}",
                f"{self.f_max:.10g}", saturated, self.saturation_events,
            ])

        return [force]

    def shutdown(self) -> None:
        self._close_diagnostics()
        # Short summary for demos / debugging.
        print(
            f"[AdaptiveDampingPTO] steps={self._step_count} "
            f"final_c={self.c:.3e} integral={self._integral:.3e} "
            f"saturation_events={self.saturation_events}",
            file=sys.stderr,
        )


if __name__ == "__main__":
    ExternalForceModule.run(AdaptiveDampingPTO())
