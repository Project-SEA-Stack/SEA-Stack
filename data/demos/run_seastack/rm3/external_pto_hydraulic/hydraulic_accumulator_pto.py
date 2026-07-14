#!/usr/bin/env python3
"""
Reduced two-pressure-state hydraulic external PTO for SEA-Stack.

A deliberately minimal hydraulic power take-off, inspired by the public
WEC-Sim PTO-Sim RM3 hydraulic circuit (double-acting cylinder -> rectifying
check valves -> high/low-pressure gas accumulators -> hydraulic motor/load,
with a relief valve). It is a *reduced* circuit: two pressure states, ideal
rectification and a resistive motor. It is not a port of the full PTO-Sim
model.

Circuit and sign conventions (extension positive, force opposes motion):

  piston flow (always rectified into the HP side):
      s      = tanh(v / v_eps)          smoothed sign of velocity
      Q_cyl  = A_p * v * s   (>= 0)     oil pumped LP -> HP by the cylinder
  cylinder force:
      dp     = p_hi - p_lo
      F      = -A_p * dp * s            opposes motion
  motor (resistive load, HP -> LP):
      Q_mot  = G_mot * max(dp, 0)
  relief valve (bypass, HP -> LP, limits dp):
      Q_rel  = G_rel * max(dp - p_relief, 0)
  accumulator oil bookkeeping (V_oil = oil stored above precharge):
      dV_oil_hi = ( Q_cyl - Q_mot - Q_rel) * dt
      dV_oil_lo = (-Q_cyl + Q_mot + Q_rel) * dt
  gas pressure (polytropic, precharge at V_oil = 0):
      p = p_precharge * (V_gas0 / (V_gas0 - V_oil)) ** gamma

Absorbed mechanical power P_abs = -F v = dp * Q_cyl. With explicit sub-stepping
the following energy identity holds exactly, sub-step by sub-step:

      E_abs = dE_gas + E_motor + E_relief

where dE_gas = sum(p_hi dV_oil_hi + p_lo dV_oil_lo), E_motor = sum(dp Q_mot dt),
E_relief = sum(dp Q_rel dt). The golden test verifies this identity, so the
module doubles as its own energy-balance oracle.

Config keys (JSON via setup YAML `external_pto.config`), all SI:
  piston_area          A_p [m^2]                      (default 0.05)
  gamma                gas polytropic exponent        (default 1.4)
  V_gas_hi, V_gas_lo   precharge gas volumes [m^3]    (default 2.0, 2.0)
  p_precharge_hi       HP precharge [Pa]              (default 1.0e7)
  p_precharge_lo       LP precharge [Pa]              (default 2.0e6)
  p_relief             relief threshold on dp [Pa]    (default 3.0e7)
  relief_conductance   G_rel [m^3/s/Pa]               (default 1.0e-7)
  motor_conductance    G_mot [m^3/s/Pa]               (default 5.0e-9)
  v_eps                velocity smoothing [m/s]       (default 0.02)
  n_substeps           Euler sub-steps per host step  (default 10)
  diagnostics_csv      optional path for per-step hydraulic CSV (overwrite)

State (n_states = 2): the two accumulator oil volumes V_oil_hi, V_oil_lo.
Implements reset()/commit()/rollback() over the full internal state.

The physics live in this file. seastack_external.py is only the IPC helper.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, TextIO

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


class HydraulicAccumulatorPTO(ExternalForceModule):
    """Reduced hydraulic PTO with two accumulator pressure states."""

    def initialize(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        self.A_p = float(cfg.get("piston_area", 0.05))
        self.gamma = float(cfg.get("gamma", 1.4))
        self.Vg0_hi = float(cfg.get("V_gas_hi", 2.0))
        self.Vg0_lo = float(cfg.get("V_gas_lo", 2.0))
        self.p_pre_hi = float(cfg.get("p_precharge_hi", 1.0e7))
        self.p_pre_lo = float(cfg.get("p_precharge_lo", 2.0e6))
        self.p_relief = float(cfg.get("p_relief", 3.0e7))
        self.G_rel = float(cfg.get("relief_conductance", 1.0e-7))
        self.G_mot = float(cfg.get("motor_conductance", 5.0e-9))
        self.v_eps = float(cfg.get("v_eps", 0.02))
        self.n_sub = max(1, int(cfg.get("n_substeps", 10)))

        self._diag_path = str(cfg.get("diagnostics_csv", "") or "").strip()
        self._diag_fh: Optional[TextIO] = None
        self._diag_writer: Optional[csv.writer] = None
        self._last_q_mot = 0.0
        self._last_q_rel = 0.0

        self._open_diagnostics()
        self._reset_state()
        self._committed = self._snapshot()
        return {
            "name": "HydraulicAccumulatorPTO",
            "version": "1.0",
            "n_states": 2,  # V_oil_hi, V_oil_lo
        }

    def _open_diagnostics(self) -> None:
        """Open (overwrite) an example-only hydraulic state CSV if configured."""
        if not self._diag_path:
            return
        path = Path(self._diag_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._diag_fh = open(path, "w", newline="", encoding="utf-8")
        self._diag_writer = csv.writer(self._diag_fh)
        self._diag_writer.writerow([
            "time", "displacement", "velocity", "force",
            "p_hi", "p_lo", "dp",
            "Q_mot", "Q_rel", "P_motor",
            "E_abs", "E_gas", "E_motor", "E_relief", "residual",
            "relief_events", "limit_hits",
        ])
        self._diag_fh.flush()

    def _close_diagnostics(self) -> None:
        if self._diag_fh is not None:
            self._diag_fh.close()
            self._diag_fh = None
            self._diag_writer = None

    # --- state management ------------------------------------------------
    def _reset_state(self) -> None:
        self.V_oil_hi = 0.0
        self.V_oil_lo = 0.0
        self._last_t = None  # type: float | None
        self._last_force = 0.0
        self._last_q_mot = 0.0
        self._last_q_rel = 0.0
        # Cumulative energy diagnostics [J].
        self.E_abs = 0.0
        self.E_gas = 0.0
        self.E_motor = 0.0
        self.E_relief = 0.0
        self.relief_events = 0
        self.limit_hits = 0

    def _snapshot(self) -> Dict[str, Any]:
        return {
            "V_oil_hi": self.V_oil_hi,
            "V_oil_lo": self.V_oil_lo,
            "last_t": self._last_t,
            "last_force": self._last_force,
            "last_q_mot": self._last_q_mot,
            "last_q_rel": self._last_q_rel,
            "E_abs": self.E_abs,
            "E_gas": self.E_gas,
            "E_motor": self.E_motor,
            "E_relief": self.E_relief,
            "relief_events": self.relief_events,
            "limit_hits": self.limit_hits,
        }

    def _restore(self, s: Dict[str, Any]) -> None:
        self.V_oil_hi = s["V_oil_hi"]
        self.V_oil_lo = s["V_oil_lo"]
        self._last_t = s["last_t"]
        self._last_force = s["last_force"]
        self._last_q_mot = s.get("last_q_mot", 0.0)
        self._last_q_rel = s.get("last_q_rel", 0.0)
        self.E_abs = s["E_abs"]
        self.E_gas = s["E_gas"]
        self.E_motor = s["E_motor"]
        self.E_relief = s["E_relief"]
        self.relief_events = s["relief_events"]
        self.limit_hits = s["limit_hits"]

    def reset(self) -> None:
        self._reset_state()
        self._committed = self._snapshot()

    def commit(self) -> None:
        self._committed = self._snapshot()

    def rollback(self) -> None:
        self._restore(self._committed)

    # --- physics ---------------------------------------------------------
    def _pressure(self, p_pre: float, Vg0: float, V_oil: float,
                  count_limit: bool = False) -> float:
        V_gas = Vg0 - V_oil
        # Guard against overfilling the accumulator (near-singular gas volume).
        v_min = 1.0e-3 * Vg0
        if V_gas < v_min:
            V_gas = v_min
            if count_limit:
                self.limit_hits += 1
        return p_pre * (Vg0 / V_gas) ** self.gamma

    def _integrate(self, vel: float, dt: float) -> None:
        if dt <= 0.0:
            return
        h = dt / self.n_sub
        s = math.tanh(vel / self.v_eps)
        q_cyl = self.A_p * vel * s  # >= 0 by construction
        for _ in range(self.n_sub):
            p_hi = self._pressure(self.p_pre_hi, self.Vg0_hi, self.V_oil_hi, True)
            p_lo = self._pressure(self.p_pre_lo, self.Vg0_lo, self.V_oil_lo, True)
            dp = p_hi - p_lo
            q_mot = self.G_mot * dp if dp > 0.0 else 0.0
            over = dp - self.p_relief
            q_rel = self.G_rel * over if over > 0.0 else 0.0
            if q_rel > 0.0:
                self.relief_events += 1

            dV_hi = (q_cyl - q_mot - q_rel) * h
            dV_lo = (-q_cyl + q_mot + q_rel) * h

            # Energy bookkeeping with pre-update pressures (keeps the identity exact).
            self.E_abs += dp * q_cyl * h
            self.E_gas += p_hi * dV_hi + p_lo * dV_lo
            self.E_motor += dp * q_mot * h
            self.E_relief += dp * q_rel * h

            self.V_oil_hi += dV_hi
            self.V_oil_lo += dV_lo
            self._last_q_mot = q_mot
            self._last_q_rel = q_rel

    def evaluate(self, t: float, dt: float, inputs: List[float]) -> List[float]:
        disp, vel = inputs[0], inputs[1]

        # One advance per accepted time step; re-evaluations at the same (or
        # earlier) time return the cached force (SEA-Stack already enforces this
        # on the C++ side, but the guard keeps the module correct standalone).
        if self._last_t is not None and t <= self._last_t:
            return [self._last_force]

        step_dt = dt
        if step_dt <= 0.0 and self._last_t is not None:
            step_dt = max(0.0, t - self._last_t)

        self._integrate(vel, step_dt)
        self._last_t = t

        p_hi = self._pressure(self.p_pre_hi, self.Vg0_hi, self.V_oil_hi)
        p_lo = self._pressure(self.p_pre_lo, self.Vg0_lo, self.V_oil_lo)
        s = math.tanh(vel / self.v_eps)
        force = -self.A_p * (p_hi - p_lo) * s
        self._last_force = force

        if self._diag_writer is not None:
            dp = p_hi - p_lo
            residual = self.E_abs - (self.E_gas + self.E_motor + self.E_relief)
            self._diag_writer.writerow([
                f"{t:.10g}", f"{disp:.10g}", f"{vel:.10g}", f"{force:.10g}",
                f"{p_hi:.10g}", f"{p_lo:.10g}", f"{dp:.10g}",
                f"{self._last_q_mot:.10g}", f"{self._last_q_rel:.10g}",
                f"{(dp * self._last_q_mot):.10g}",
                f"{self.E_abs:.10g}", f"{self.E_gas:.10g}",
                f"{self.E_motor:.10g}", f"{self.E_relief:.10g}", f"{residual:.10g}",
                self.relief_events, self.limit_hits,
            ])

        return [force]

    def diagnostics(self) -> Dict[str, float]:
        p_hi = self._pressure(self.p_pre_hi, self.Vg0_hi, self.V_oil_hi)
        p_lo = self._pressure(self.p_pre_lo, self.Vg0_lo, self.V_oil_lo)
        return {
            "p_hi": p_hi,
            "p_lo": p_lo,
            "dp": p_hi - p_lo,
            "V_oil_hi": self.V_oil_hi,
            "V_oil_lo": self.V_oil_lo,
            "E_abs": self.E_abs,
            "E_gas": self.E_gas,
            "E_motor": self.E_motor,
            "E_relief": self.E_relief,
            "residual": self.E_abs - (self.E_gas + self.E_motor + self.E_relief),
            "relief_events": float(self.relief_events),
            "limit_hits": float(self.limit_hits),
        }

    def shutdown(self) -> None:
        self._close_diagnostics()
        d = self.diagnostics()
        print(
            f"[HydraulicAccumulatorPTO] dp={d['dp']:.3e} Pa "
            f"E_abs={d['E_abs']:.3e} J E_motor={d['E_motor']:.3e} J "
            f"E_relief={d['E_relief']:.3e} J residual={d['residual']:.3e} J "
            f"relief_events={int(d['relief_events'])}",
            file=sys.stderr,
        )


if __name__ == "__main__":
    ExternalForceModule.run(HydraulicAccumulatorPTO())
