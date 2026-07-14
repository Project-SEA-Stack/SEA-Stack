#!/usr/bin/env python3
"""
Linear damper external PTO (RM3 demo).

F = -c * v  (matches seastack::pto::LinearPTO with k=0).
Interesting physics stay in this file; seastack_external.py is only the IPC helper.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, List

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


class LinearDamperPTO(ExternalForceModule):
    def initialize(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        self.c = float(cfg.get("damping", 50.0))
        return {"name": "LinearDamperPTO", "version": "1.0", "n_states": 0}

    def evaluate(self, t: float, dt: float, inputs: List[float]) -> List[float]:
        _disp, vel = inputs[0], inputs[1]
        return [-self.c * vel]


if __name__ == "__main__":
    ExternalForceModule.run(LinearDamperPTO())
