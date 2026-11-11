#!/usr/bin/env python3
"""
Winch controller for the IEA sphere (SEA-Stack external controller, preview).

What it does
    Holds the rope until haul_start_s, then hauls in haul_length_m at
    haul_speed_m_per_s by lowering the rope's rest length.

Reads
    tsda.winch_rope.rest_length  [m]   (initial rest length, read once)
    body.body1.pos_z             [m]   (printed at the end of the haul)

Writes
    tsda.winch_rope.rest_length  [m]

Expected result (static, linear hydrostatics)
    A rope of stiffness k in series with the sphere's hydrostatic stiffness
    rho*g*pi*R^2 (R = 5 m) lifts the sphere by
        dz = k * dL / (k + rho*g*pi*R^2)
    for a haul dL. With k = 770 kN/m and dL = 1 m, dz ~= 0.50 m.

Run it offline (no SEA-Stack) with ControllerHarness, see test_winch_controller()
at the bottom of this file:  python winch_controller.py --selftest
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict

_here = Path(__file__).resolve().parent
for parent in (_here, *_here.parents):
    for candidate in (parent / "python", parent / "libs" / "external" / "python"):
        if candidate.is_dir() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))

from seastack_external import ControllerHarness, ControllerModule, ControllerState, run


class WinchController(ControllerModule):
    def setup(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        self.t_start = float(cfg.get("haul_start_s", 20.0))
        self.haul = float(cfg.get("haul_length_m", 1.0))
        self.speed = float(cfg.get("haul_speed_m_per_s", 0.2))
        self.rest0 = None  # rope rest length at the first tick [m]
        self.reported = False
        return {"name": "WinchController", "version": "1.0"}

    def step(self, state: ControllerState) -> Dict[str, float]:
        rope = state.tsda("winch_rope")
        if self.rest0 is None:
            self.rest0 = rope.rest_length
        hauled = min(max(0.0, (state.time - self.t_start) * self.speed), self.haul)
        if hauled >= self.haul and not self.reported:
            self.reported = True
            print(f"t = {state.time:.2f} s: hauled {hauled:.2f} m, "
                  f"heave z = {state.body('body1').pos_z:+.3f} m")
        return {"tsda.winch_rope.rest_length": self.rest0 - hauled}


def test_winch_controller() -> None:
    """Offline check of the haul schedule (no SEA-Stack needed)."""
    h = ControllerHarness(
        WinchController(),
        sensors={"bodies": ["body1"], "tsdas": ["winch_rope"]},
        actuators={"tsdas": ["winch_rope"]},
        config={"haul_start_s": 1.0, "haul_length_m": 1.0, "haul_speed_m_per_s": 0.5},
        control_dt=0.5,
    )
    rest = "tsda.winch_rope.rest_length"
    assert h.step(0.0, {rest: 30.0})[rest] == 30.0
    assert h.step(2.0, {rest: 30.0})[rest] == 29.5
    assert h.step(9.0, {rest: 29.0})[rest] == 29.0
    print("winch_controller self-test passed")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        test_winch_controller()
    else:
        run(WinchController())
