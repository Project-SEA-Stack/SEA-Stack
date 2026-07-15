#!/usr/bin/env python3
"""Chrono-free unit tests for PtoState / PtoModule helpers."""

from __future__ import annotations

import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_PY = _REPO / "libs" / "external" / "python"
sys.path.insert(0, str(_PY))

from seastack_external import (  # noqa: E402
    DEFAULT_PTO_CHANNELS,
    PTO_RSDA_RICH_CHANNELS,
    PTO_TSDA_RICH_CHANNELS,
    PtoModule,
    PtoState,
)


def _assert(cond: bool, msg: str) -> None:
    if not cond:
        raise AssertionError(msg)


def test_pto_state_lean() -> None:
    s = PtoState.from_inputs(1.0, 0.01, [0.5, -2.0])
    _assert(s.displacement == 0.5, "disp")
    _assert(s.velocity == -2.0, "vel")
    _assert(s.names == DEFAULT_PTO_CHANNELS, "default names")
    _assert(s.get("velocity") == -2.0, "get")


def test_pto_state_tsda_rich() -> None:
    vals = [0.0] * 17
    vals[0] = 0.1
    vals[1] = 0.2
    vals[2] = 1.5
    vals[3] = 1.0
    vals[5] = 9.0
    s = PtoState.from_inputs(0.0, 0.01, vals, names=PTO_TSDA_RICH_CHANNELS)
    _assert(s.length == 1.5, "length")
    _assert(s.rest_length == 1.0, "rest_length")
    _assert(s.body1_position == (9.0, 0.0, 0.0), "body1 pos")


def test_pto_state_rsda_rich() -> None:
    vals = [0.0] * 17
    vals[0] = 0.3
    vals[1] = -0.4
    vals[2] = 1.3
    vals[3] = 1.0
    s = PtoState.from_inputs(0.0, 0.01, vals, names=PTO_RSDA_RICH_CHANNELS,
                             link_kind="pto_rsda")
    _assert(s.angle == 1.3, "angle")
    _assert(s.rest_angle == 1.0, "rest_angle")
    _assert(s.link_kind == "pto_rsda", "link_kind")


def test_pto_module_force() -> None:
    class Damper(PtoModule):
        def setup(self, cfg):
            self.c = float(cfg.get("damping", 10.0))
            return {"name": "Damper", "n_states": 0}

        def force(self, state: PtoState) -> float:
            return -self.c * state.velocity

    mod = Damper()
    mod._host_in_names = list(DEFAULT_PTO_CHANNELS)
    mod._host_kind = "pto"
    meta = mod.initialize({"damping": 5.0})
    _assert(meta["name"] == "Damper", "meta name")
    out = mod.evaluate(0.0, 0.01, [0.0, 3.0])
    _assert(abs(out[0] - (-15.0)) < 1e-12, f"force got {out[0]}")


def test_channel_schema_lengths() -> None:
    _assert(len(PTO_TSDA_RICH_CHANNELS) == 17, "tsda 17")
    _assert(len(PTO_RSDA_RICH_CHANNELS) == 17, "rsda 17")
    _assert(PTO_TSDA_RICH_CHANNELS[2] == "length", "tsda length name")
    _assert(PTO_RSDA_RICH_CHANNELS[2] == "angle", "rsda angle name")
    _assert(PTO_TSDA_RICH_CHANNELS[:2] == PTO_RSDA_RICH_CHANNELS[:2] ==
            DEFAULT_PTO_CHANNELS, "universal 0/1")


def main() -> int:
    tests = [
        test_pto_state_lean,
        test_pto_state_tsda_rich,
        test_pto_state_rsda_rich,
        test_pto_module_force,
        test_channel_schema_lengths,
    ]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"  [PASS] {fn.__name__}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"  [FAIL] {fn.__name__}: {exc}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
