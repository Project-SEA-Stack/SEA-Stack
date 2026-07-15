#!/usr/bin/env python3
"""
Prescribed-input golden tests for the three SEA-Stack external PTO examples.

Runs entirely in-process (no IPC, no Chrono, no RM3 simulation). Each case is
checked against an *independent* oracle, matching the verification ladder:

  linear_damper_pto.py        -> exact analytic law F = -c v
  adaptive_damping_pto.py     -> independent PI recurrence re-implemented here
  hydraulic_accumulator_pto.py-> exact internal energy balance + reset/rollback

Exit code 0 on success, 1 on any mismatch. Intended to run as a fast CI test.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from replay_harness import make_sinusoid, replay, _load_module_class  # noqa: E402

_REPO = _HERE.parents[1]
_DEMOS = _REPO / "data" / "demos" / "run_seastack" / "rm3"
LINEAR = _DEMOS / "external_pto" / "linear_damper_pto.py"
ADAPTIVE = _DEMOS / "external_pto_adaptive" / "adaptive_damping_pto.py"
HYDRAULIC = _DEMOS / "external_pto_hydraulic" / "hydraulic_accumulator_pto.py"


class Checker:
    def __init__(self) -> None:
        self.failures: List[str] = []

    def check(self, cond: bool, msg: str) -> None:
        status = "PASS" if cond else "FAIL"
        print(f"  [{status}] {msg}")
        if not cond:
            self.failures.append(msg)

    def close(self, cond: bool, actual: float, expected: float,
              tol: float, msg: str) -> None:
        ok = abs(actual - expected) <= tol
        self.check(ok, f"{msg} (actual={actual:.6g} expected={expected:.6g} "
                       f"|diff|={abs(actual - expected):.3g} tol={tol:.3g})")




# --- Case 1: linear damper -------------------------------------------------
def verify_linear(ck: Checker) -> None:
    print("Case 1: linear_damper_pto.py (transport / analytic oracle)")
    cls = _load_module_class(LINEAR, "LinearDamperPTO")
    c = 1.2e6
    trace = make_sinusoid(amplitude=0.8, period=6.0, duration=12.0, dt=0.01)
    res = replay(cls(), {"damping": c}, trace)

    ck.check(res.meta.get("n_states", -1) == 0, "n_states == 0 (stateless)")

    max_err = max(abs(f - (-c * v)) for f, v in zip(res.force, res.vel))
    ck.check(max_err <= 1e-6, f"F == -c v exactly (max abs err {max_err:.3g} N)")

    # Absorbed power should equal c v^2 at every sample.
    max_perr = max(abs(p - c * v * v) for p, v in zip(res.power, res.vel))
    ck.check(max_perr <= 1e-6, f"P_abs == c v^2 (max abs err {max_perr:.3g} W)")

    mean_v2 = sum(v * v for v in res.vel) / len(res.vel)
    ck.close(True, res.summary()["mean_power_W"], c * mean_v2,
             1e-3, "mean power == c <v^2>")


# --- Case 2: adaptive damping ----------------------------------------------
class _AdaptiveReference:
    """Independent re-implementation of the simplified PI variable-damping law."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.c = cfg["damping"]
        self.c0 = cfg["damping"]
        self.f_max = cfg["force_max"]
        self.kp = cfg["kp"]
        self.ki = cfg["ki"]
        self.vsp = cfg["vel_setpoint"]
        self.c_min = cfg["c_min"]
        self.c_max = cfg["c_max"]
        self.integral = 0.0
        self.sat = 0

    def step(self, t: float, dt: float, x: float, v: float) -> float:
        del t, x  # unused; law depends on velocity + controller state only
        if dt > 0.0:
            err = self.vsp - abs(v)
            self.integral += err * dt
            u = self.c0 - self.kp * err - self.ki * self.integral
            if u > self.c_max:
                self.integral -= err * dt
                u = self.c_max
            elif u < self.c_min:
                self.integral -= err * dt
                u = self.c_min
            self.c = u
        f_cmd = -self.c * v
        f = max(-self.f_max, min(self.f_max, f_cmd))
        if f != f_cmd:
            self.sat += 1
        return f


def verify_adaptive(ck: Checker) -> None:
    print("Case 2: adaptive_damping_pto.py (stateful PI + saturation)")
    cls = _load_module_class(ADAPTIVE, "AdaptiveDampingPTO")
    cfg = {
        "damping": 8.0e5,
        "force_max": 1.0e6,   # low enough to force saturation on velocity peaks
        "kp": 2.0e5,
        "ki": 5.0e4,
        "vel_setpoint": 0.4,
        "c_min": 1.0e5,
        "c_max": 3.0e6,
    }
    trace = make_sinusoid(amplitude=1.0, period=5.0, duration=15.0, dt=0.01)

    res = replay(cls(), cfg, trace)
    ref = _AdaptiveReference(cfg)
    ref_force = [ref.step(t, dt, x, v) for (t, dt, x, v) in trace]

    max_err = max(abs(a - b) for a, b in zip(res.force, ref_force))
    ck.check(max_err <= 1e-6,
             f"force matches independent recurrence (max abs err {max_err:.3g} N)")

    ck.check(res.meta.get("n_states", -1) == 2, "n_states == 2")
    ck.check(res.diagnostics.get("saturation_events", 0) == ref.sat,
             f"saturation events match ({ref.sat})")
    ck.check(ref.sat > 0, "saturation actually exercised")

    # Determinism + reset: replay again from a fresh reset, expect identical.
    mod = cls()
    res_a = replay(mod, cfg, trace)
    res_b = replay(mod, cfg, trace)   # replay() resets first
    max_reset_err = max(abs(a - b) for a, b in zip(res_a.force, res_b.force))
    ck.check(max_reset_err == 0.0,
             f"reset() restores identical trajectory (max diff {max_reset_err:.3g})")


# --- Case 3: hydraulic accumulator -----------------------------------------
def verify_hydraulic(ck: Checker) -> None:
    print("Case 3: hydraulic_accumulator_pto.py (dynamic subsystem)")
    cls = _load_module_class(HYDRAULIC, "HydraulicAccumulatorPTO")
    cfg = {
        "piston_area": 0.05,
        "gamma": 1.4,
        "V_gas_hi": 2.0,
        "V_gas_lo": 2.0,
        "p_precharge_hi": 1.0e7,
        "p_precharge_lo": 2.0e6,
        "p_relief": 8.0e6,          # low enough to trip the relief valve
        "relief_conductance": 1.0e-7,
        "motor_conductance": 5.0e-9,
        "v_eps": 0.02,
        "n_substeps": 10,
    }
    trace = make_sinusoid(amplitude=1.0, period=6.0, duration=18.0, dt=0.01)
    res = replay(cls(), cfg, trace)
    d = res.diagnostics

    # Exact internal energy balance: E_abs = dE_gas + E_motor + E_relief.
    e_abs = d["E_abs"]
    residual = d["residual"]
    rel = abs(residual) / max(abs(e_abs), 1.0)
    ck.check(rel <= 1e-6,
             f"energy balance E_abs=dE_gas+E_motor+E_relief (rel residual {rel:.3g})")

    ck.check(e_abs > 0.0, f"net absorbed energy > 0 ({e_abs:.3g} J)")
    ck.check(d["E_motor"] >= 0.0 and d["E_relief"] >= 0.0,
             "motor and relief energies non-negative")
    ck.check(d["limit_hits"] == 0.0, "accumulator never overfilled (limit_hits == 0)")
    ck.check(d["relief_events"] > 0.0, "relief valve actually exercised")
    ck.check(res.meta.get("n_states", -1) == 2, "n_states == 2 (two pressure states)")

    # reset() clears all internal state.
    mod = cls()
    replay(mod, cfg, trace)
    mod.reset()
    dr = mod.diagnostics()
    ck.check(dr["V_oil_hi"] == 0.0 and dr["V_oil_lo"] == 0.0 and dr["E_abs"] == 0.0,
             "reset() clears pressures and energy")

    # commit()/rollback() semantics: state after rollback == committed state.
    mod2 = cls()
    mod2.initialize(cfg)
    for (t, dt, x, v) in trace[:400]:
        mod2.evaluate(t, dt, [x, v])
    mod2.commit()
    committed = dict(mod2.diagnostics())
    forces_after: List[float] = []
    for (t, dt, x, v) in trace[400:800]:
        forces_after.append(mod2.evaluate(t, dt, [x, v])[0])
    mod2.rollback()
    rolled = dict(mod2.diagnostics())
    same = all(abs(committed[k] - rolled[k]) <= 1e-9 * max(1.0, abs(committed[k]))
               for k in ("V_oil_hi", "V_oil_lo", "E_abs", "E_motor", "E_relief"))
    ck.check(same, "rollback() restores committed state")

    # Replaying the same steps after rollback reproduces the forces exactly.
    forces_again = [mod2.evaluate(t, dt, [x, v])[0] for (t, dt, x, v) in trace[400:800]]
    max_rb_err = max(abs(a - b) for a, b in zip(forces_after, forces_again))
    ck.check(max_rb_err == 0.0,
             f"deterministic replay after rollback (max diff {max_rb_err:.3g})")


def _hyd_base_cfg() -> Dict[str, Any]:
    return {
        "piston_area": 0.05,
        "gamma": 1.4,
        "V_gas_hi": 2.0,
        "V_gas_lo": 2.0,
        "p_precharge_hi": 1.0e7,
        "p_precharge_lo": 2.0e6,
        "p_relief": 3.0e7,
        "relief_conductance": 1.0e-7,
        "motor_conductance": 5.0e-9,
        "v_eps": 0.02,
        "n_substeps": 10,
    }


def _drive_const(mod: Any, v: float, n: int, dt: float = 0.01,
                 t0: float = 0.0) -> Tuple[List[float], float]:
    """Drive the module at constant velocity; return (forces, last_t)."""
    forces: List[float] = []
    t = t0
    for _ in range(n):
        t += dt
        forces.append(mod.evaluate(t, dt, [0.0, v])[0])
    return forces, t


def verify_hydraulic_components(ck: Checker) -> None:
    print("Case 3b: hydraulic_accumulator_pto.py (component-level physics)")
    cls = _load_module_class(HYDRAULIC, "HydraulicAccumulatorPTO")

    # No-motion equilibrium: zero velocity => zero force and zero absorption;
    # the motor relaxes the precharge imbalance (motor dissipation > 0).
    mod = cls()
    mod.initialize(_hyd_base_cfg())
    dp0 = mod.diagnostics()["dp"]
    forces, _ = _drive_const(mod, 0.0, 300)
    d = mod.diagnostics()
    ck.check(all(f == 0.0 for f in forces), "no motion => force == 0 exactly")
    ck.check(d["E_abs"] == 0.0, "no motion => zero absorbed energy")
    ck.check(0.0 <= d["dp"] < dp0, f"motor relaxes dp toward 0 ({dp0:.3g}->{d['dp']:.3g})")
    ck.check(d["E_motor"] > 0.0, "motor dissipates the precharge imbalance")

    # Constant positive flow: cylinder pumps into HP (compression), force
    # opposes motion, energy is absorbed, low side stays positive.
    mod = cls()
    mod.initialize(_hyd_base_cfg())
    forces, t_end = _drive_const(mod, 1.0, 400)
    d = mod.diagnostics()
    ck.check(all(f < 0.0 for f in forces), "v>0 => force opposes motion (F<0)")
    ck.check(d["V_oil_hi"] > 0.0 and d["p_hi"] > _hyd_base_cfg()["p_precharge_hi"],
             "constant flow compresses HP accumulator (p_hi > precharge)")
    ck.check(d["E_abs"] > 0.0, "constant flow absorbs energy")
    ck.check(d["p_lo"] > 0.0 and d["limit_hits"] == 0.0,
             "pressures stay positive, no overfill")
    p_hi_compressed = d["p_hi"]

    # Expansion: hold still afterwards; the HP accumulator expands back down.
    # (Continue the clock from t_end so the time-cache guard does not skip steps.)
    _drive_const(mod, 0.0, 400, t0=t_end)
    ck.check(mod.diagnostics()["p_hi"] < p_hi_compressed,
             "HP accumulator expands when flow stops (p_hi decreases)")

    # Reversed motion: rectification keeps absorbing; force flips sign with v.
    mod = cls()
    mod.initialize(_hyd_base_cfg())
    _drive_const(mod, 1.0, 200)
    e_mid = mod.diagnostics()["E_abs"]
    forces_rev, _ = _drive_const(mod, -1.0, 200, t0=2.0)
    d = mod.diagnostics()
    ck.check(all(f > 0.0 for f in forces_rev), "v<0 => force opposes motion (F>0)")
    ck.check(d["E_abs"] > e_mid, "reversed motion still absorbs (rectification)")

    # Relief valve: low threshold + strong drive trips the relief and bounds dp.
    cfg_relief = _hyd_base_cfg()
    cfg_relief["p_relief"] = 6.0e6
    mod = cls()
    mod.initialize(cfg_relief)
    dp_max = 0.0
    t = 0.0
    for _ in range(600):
        t += 0.01
        mod.evaluate(t, 0.01, [0.0, 1.5])
        dp_max = max(dp_max, mod.diagnostics()["dp"])
    d = mod.diagnostics()
    ck.check(d["relief_events"] > 0.0, "relief valve opens under strong drive")
    ck.check(dp_max < 3.0 * cfg_relief["p_relief"],
             f"relief bounds dp (max {dp_max:.3g} Pa vs threshold "
             f"{cfg_relief['p_relief']:.3g} Pa)")
    ck.check(d["limit_hits"] == 0.0, "no accumulator overfill under relief")


def main() -> int:
    for p in (LINEAR, ADAPTIVE, HYDRAULIC):
        if not p.is_file():
            print(f"ERROR: missing example module {p}", file=sys.stderr)
            return 2

    ck = Checker()
    verify_linear(ck)
    verify_adaptive(ck)
    verify_hydraulic(ck)
    verify_hydraulic_components(ck)

    print()
    if ck.failures:
        print(f"FAILED: {len(ck.failures)} check(s)")
        for f in ck.failures:
            print(f"  - {f}")
        return 1
    print("All external PTO golden checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
