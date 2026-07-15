#!/usr/bin/env python3
"""
Hydraulic accumulator external PTO — step up from the adaptive damper.

Adaptive added controller state (integral, c(t)). This demo adds *physics*
state inside the module: two gas-accumulator oil volumes that evolve each
step, plus commit/rollback so that state can be snapshotted.

Reduced circuit (inspired by WEC-Sim PTO-Sim RM3, not a full port):

    cylinder pumps oil LP -> HP (rectified)
    F = -A_p * (p_hi - p_lo) * sign(v)     # opposes motion
    motor drains HP -> LP; relief valve limits dp
    p = p_pre * (V_gas0 / (V_gas0 - V_oil))**gamma

Energy identity (checked by the golden test):

    E_abs = dE_gas + E_motor + E_relief

Config (YAML `external_pto.config`): piston_area, gamma, V_gas_hi/lo,
p_precharge_hi/lo, p_relief, relief_conductance, motor_conductance,
v_eps, n_substeps, optional diagnostics_csv.

seastack_external.py is only the IPC helper; keep your model in this file.
"""

from __future__ import annotations

import csv
import math
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

# Attributes restored by commit/rollback (and cleared by reset).
_STATE_ATTRS = (
    "V_oil_hi", "V_oil_lo",
    "_last_t", "_last_force", "_last_q_mot", "_last_q_rel",
    "E_abs", "E_gas", "E_motor", "E_relief",
    "relief_events", "limit_hits",
)


class HydraulicAccumulatorPTO(PtoModule):
    """Two-accumulator hydraulic PTO with commit/rollback over internal state."""

    def setup(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        self.A_p = float(cfg.get("piston_area", 0.05))
        self.gamma = float(cfg.get("gamma", 1.4))
        self.Vg0_hi = float(cfg.get("V_gas_hi", 2.0))
        self.Vg0_lo = float(cfg.get("V_gas_lo", 2.0))
        self.p_pre_hi = float(cfg.get("p_precharge_hi", 1.0e7))
        self.p_pre_lo = float(cfg.get("p_precharge_lo", 2.0e6))
        self.p_relief = float(cfg.get("p_relief", 3.0e7))
        self.G_rel = float(cfg.get("relief_conductance", 1.0e-7))
        self.G_mot = float(cfg.get("motor_conductance", 5.0e-9))
        self.v_eps = max(float(cfg.get("v_eps", 0.02)), 1.0e-6)
        self.n_sub = max(1, int(cfg.get("n_substeps", 10)))

        self._diag_fh: Optional[TextIO] = None
        self._diag_writer: Optional[csv.writer] = None
        diag_path = str(cfg.get("diagnostics_csv", "") or "").strip()
        if diag_path:
            path = Path(diag_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._diag_fh = open(path, "w", newline="", encoding="utf-8")
            self._diag_writer = csv.writer(self._diag_fh)
            self._diag_writer.writerow([
                "time", "E_abs", "E_gas", "E_motor", "E_relief",
                "residual", "relief_events",
            ])

        self.reset()
        return {
            "name": "HydraulicAccumulatorPTO",
            "version": "1.0",
            "n_states": 2,  # V_oil_hi, V_oil_lo
        }

    # --- lifecycle (new vs adaptive: commit / rollback) --------------------
    def reset(self) -> None:
        self.V_oil_hi = 0.0
        self.V_oil_lo = 0.0
        self._last_t = None  # type: float | None
        self._last_force = 0.0
        self._last_q_mot = 0.0
        self._last_q_rel = 0.0
        self.E_abs = 0.0
        self.E_gas = 0.0
        self.E_motor = 0.0
        self.E_relief = 0.0
        self.relief_events = 0
        self.limit_hits = 0
        self._committed = self._snapshot()

    def commit(self) -> None:
        self._committed = self._snapshot()

    def rollback(self) -> None:
        for key, value in self._committed.items():
            setattr(self, key, value)

    def _snapshot(self) -> Dict[str, Any]:
        return {key: getattr(self, key) for key in _STATE_ATTRS}

    # --- physics -----------------------------------------------------------
    def _pressure(self, p_pre: float, Vg0: float, V_oil: float,
                  count_limit: bool = False) -> float:
        V_gas = Vg0 - V_oil
        v_min = 1.0e-3 * Vg0
        if V_gas < v_min:
            V_gas = v_min
            if count_limit:
                self.limit_hits += 1
        return p_pre * (Vg0 / V_gas) ** self.gamma

    def _advance(self, vel: float, dt: float) -> None:
        """Integrate accumulator volumes over one host step (Euler sub-steps)."""
        if dt <= 0.0:
            return
        h = dt / self.n_sub
        s = math.tanh(vel / self.v_eps)
        q_cyl = self.A_p * vel * s  # >= 0 (rectified)
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

            # Energy with pre-update pressures keeps E_abs = dE_gas+E_motor+E_relief exact.
            self.E_abs += dp * q_cyl * h
            self.E_gas += p_hi * dV_hi + p_lo * dV_lo
            self.E_motor += dp * q_mot * h
            self.E_relief += dp * q_rel * h

            self.V_oil_hi += dV_hi
            self.V_oil_lo += dV_lo
            self._last_q_mot = q_mot
            self._last_q_rel = q_rel

    def force(self, state: PtoState) -> float:
        vel = state.velocity
        t = state.time
        dt = state.dt

        # One advance per accepted time; re-evals at the same t reuse the cache.
        if self._last_t is not None and t <= self._last_t:
            return self._last_force

        step_dt = dt
        if step_dt <= 0.0 and self._last_t is not None:
            step_dt = max(0.0, t - self._last_t)

        self._advance(vel, step_dt)
        self._last_t = t

        p_hi = self._pressure(self.p_pre_hi, self.Vg0_hi, self.V_oil_hi)
        p_lo = self._pressure(self.p_pre_lo, self.Vg0_lo, self.V_oil_lo)
        s = math.tanh(vel / self.v_eps)
        force = -self.A_p * (p_hi - p_lo) * s
        self._last_force = force

        if self._diag_writer is not None:
            residual = self.E_abs - (self.E_gas + self.E_motor + self.E_relief)
            self._diag_writer.writerow([
                f"{t:.10g}", f"{self.E_abs:.10g}", f"{self.E_gas:.10g}",
                f"{self.E_motor:.10g}", f"{self.E_relief:.10g}",
                f"{residual:.10g}", self.relief_events,
            ])

        return force

    def diagnostics(self) -> Dict[str, float]:
        """Snapshot used by the golden tests (energy balance, pressures, events)."""
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
        if self._diag_fh is not None:
            self._diag_fh.close()
            self._diag_fh = None
            self._diag_writer = None


if __name__ == "__main__":
    run(HydraulicAccumulatorPTO())
