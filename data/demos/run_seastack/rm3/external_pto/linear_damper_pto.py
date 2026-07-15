#!/usr/bin/env python3
"""
Linear damper external PTO (RM3 demo) — start here.

Law
    F = -c * v

Same sign convention as SEA-Stack's native LinearPTO (k=0):
extension positive, force opposes motion.

Inputs (from SEA-Stack each accepted step, via PtoState)
    state.velocity       relative rate [m/s] on a TSDA, [rad/s] on an RSDA
    state.displacement   relative extension [m] or [rad] (unused here)
    state.time, state.dt host time and step size

Outputs
    force(state) -> float   force [N] on a TSDA, torque [N·m] on an RSDA

How it works
    1. setup(cfg) reads YAML config once (here: damping c).
    2. force(state) is called each accepted time step.
    3. run(...) connects this class to SEA-Stack over IPC.

How to extend
    Keep this same skeleton. Next step up: adaptive_damping_pto.py adds
    an adapting c(t), force saturation, and reset(). The hydraulic demo
    then adds internal physics states with commit/rollback.

seastack_external.py is only the IPC helper; keep your model in this file.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict

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


class LinearDamperPTO(PtoModule):
    """F = -c v  (extension positive, force opposes motion)."""

    def setup(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        self.c = float(cfg.get("damping", 50.0))
        return {"name": "LinearDamperPTO", "version": "1.0", "n_states": 0}

    def force(self, state: PtoState) -> float:
        return -self.c * state.velocity


if __name__ == "__main__":
    run(LinearDamperPTO())
