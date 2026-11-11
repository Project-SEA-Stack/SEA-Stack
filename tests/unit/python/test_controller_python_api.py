#!/usr/bin/env python3
"""
External controller Python API (preview): chrono-free checks.

  - CONTROLLER_FIELDS / CONTROLLER_WRITABLE match the channel table in
    docs/extending/CONTROLLERS.md (which a C++ test ties to the registry);
  - ControllerHarness applies the host's hold rules (missing key, None, NaN);
  - bad returns and unknown sensor names raise with helpful messages;
  - print() and self.log() are captured; tracebacks hide helper frames;
  - the winch demo's own self-test passes.

Standard library only:  python test_controller_python_api.py
"""

from __future__ import annotations

import math
import re
import subprocess
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO / "libs" / "external" / "python"))

from seastack_external import (  # noqa: E402
    CONTROLLER_FIELDS,
    CONTROLLER_WRITABLE,
    ControllerHarness,
    ControllerModule,
)

FAILURES = []


def check(cond: bool, msg: str) -> None:
    print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILURES.append(msg)


def raises(fn, exc_type, *fragments) -> bool:
    try:
        fn()
    except exc_type as exc:
        text = str(exc)
        missing = [f for f in fragments if f not in text]
        if missing:
            print(f"         message {text!r} lacks {missing}")
        return not missing
    return False


def test_fields_match_docs() -> None:
    doc = (_REPO / "docs" / "extending" / "CONTROLLERS.md").read_text(encoding="utf-8")
    table = doc.split("<!-- controller-channel-table:begin -->")[1].split(
        "<!-- controller-channel-table:end -->")[0]
    fields, writable = {}, {}
    row = re.compile(r"^\| `(\w+)\.<name>\.(\w+)` \| ([\w ]+) \| [^|]+ \| (read(?:/write)?) \|")
    for line in table.splitlines():
        m = row.match(line)
        if not m:
            continue
        _, field, applies, access = m.groups()
        kind = applies[4:] if applies.startswith("any ") else applies
        fields.setdefault(kind, []).append(field)
        if access == "read/write":
            writable.setdefault(kind, []).append(field)
    check({k: tuple(v) for k, v in fields.items()} == CONTROLLER_FIELDS,
          "CONTROLLER_FIELDS matches the docs channel table")
    check({k: tuple(v) for k, v in writable.items()} == CONTROLLER_WRITABLE,
          "CONTROLLER_WRITABLE matches the docs channel table")


class Echo(ControllerModule):
    """Returns whatever the test puts in self.next."""

    def setup(self, cfg):
        print("setup saw", cfg.get("gain"))
        self.next = {}

    def step(self, state):
        self.seen = state
        return self.next


def make() -> ControllerHarness:
    return ControllerHarness(
        Echo(),
        sensors={"bodies": ["float"], "motors": ["slew"], "tsdas": ["rope"]},
        actuators={"motors": ["slew"], "tsdas": ["rope"], "rsdas": ["hinge"]},
        motor_types={"slew": "rotation"},
        config={"gain": 2.0},
        control_dt=0.05,
        initial={"tsda.rope.rest_length": 30.0},
    )


def test_harness_hold_rules() -> None:
    h = make()
    check(h.log == ["setup saw 2.0"], "print() in setup() is captured")
    check(h.actuator_names == ["motor.slew.angle", "motor.slew.angle_dt",
                               "tsda.rope.rest_length", "rsda.hinge.rest_angle"],
          "actuator channel order: motors, tsdas, rsdas")
    check(len(h.sensor_names) == 22 + 3 + 4, "sensor channels: body 22 + rotation motor 3 + tsda 4")

    h.module.next = {"motor.slew.angle": 0.5}
    out = h.step(0.0, {"body.float.pos_z": -0.1})
    check(out["motor.slew.angle"] == 0.5 and out["tsda.rope.rest_length"] == 30.0,
          "commanded channel updates; others keep their initial value")
    check(h.module.seen.body("float").pos == (0.0, 0.0, -0.1), "body() tuple view")
    check(h.module.seen.time == 0.0 and h.module.seen.dt == 0.05, "state.time and state.dt")

    h.module.next = None
    check(h.step(0.05)["motor.slew.angle"] == 0.5, "returning None holds every channel")

    h.module.next = {"motor.slew.angle": math.nan, "tsda.rope.rest_length": 29.0}
    out = h.step(0.1)
    check(out["motor.slew.angle"] == 0.5 and out["tsda.rope.rest_length"] == 29.0,
          "NaN holds the previous value")
    check(any("holding the previous value" in line for line in h.log), "NaN warns")
    n_warn = sum("holding the previous value" in line for line in h.log)
    h.step(0.15)
    check(sum("holding the previous value" in line for line in h.log) == n_warn,
          "NaN warning is reported once per channel")


def test_errors() -> None:
    h = make()
    h.module.next = [1.0]
    check(raises(lambda: h.step(0.0), TypeError, "must return a dict"),
          "list return -> TypeError")
    h.module.next = {"body.float.pos_z": 1.0}
    check(raises(lambda: h.step(0.0), KeyError, "sensor channel", "actuators:"),
          "writing a sensor channel -> KeyError naming the fix")
    h.module.next = {"tsda.rope.rest_lenght": 1.0}
    check(raises(lambda: h.step(0.0), KeyError, "Did you mean 'tsda.rope.rest_length'"),
          "actuator typo -> suggestion")
    h.module.next = {"tsda.rope.rest_length": "30"}
    check(raises(lambda: h.step(0.0), TypeError, "must be a number"),
          "string value -> TypeError")
    h.module.next = {}
    h.step(0.0)
    state = h.module.seen
    check(raises(lambda: state["body.float.pos_q"], KeyError, "Did you mean"),
          "unknown sensor read -> KeyError with suggestion")
    check(raises(lambda: h.step(0.0, {"body.float.posz": 1.0}), KeyError, "Did you mean"),
          "harness rejects unknown sensor values")
    check(raises(lambda: ControllerHarness(Echo(), actuators={"bodies": ["float"]}),
                 KeyError, "unknown element list 'bodies'"),
          "bodies cannot be actuators")


class Boom(ControllerModule):
    def step(self, state):
        self.log("about to fail")
        raise ValueError("bad gain")


def test_traceback_hides_helper() -> None:
    module = Boom()
    h = ControllerHarness(module, actuators={"tsdas": ["rope"]})
    try:
        h.step(0.0)
        check(False, "exception propagates from step()")
        return
    except ValueError as exc:
        text = module._format_error(exc)
    check("bad gain" in text and "in step" in text, "traceback names the user's step()")
    check("seastack_external.py" not in text, "traceback hides seastack_external.py frames")
    check(h.log == ["about to fail"], "self.log() lines are kept when step() raises")


def test_winch_selftest() -> None:
    script = (_REPO / "data" / "demos" / "run_seastack" / "iea_sphere" / "controller_winch"
              / "winch_controller.py")
    proc = subprocess.run([sys.executable, str(script), "--selftest"],
                          capture_output=True, text=True)
    check(proc.returncode == 0 and "self-test passed" in proc.stdout,
          "winch_controller.py --selftest")
    if proc.returncode != 0:
        print(proc.stdout + proc.stderr)


def main() -> int:
    test_fields_match_docs()
    test_harness_hold_rules()
    test_errors()
    test_traceback_hides_helper()
    test_winch_selftest()
    if FAILURES:
        print(f"\n{len(FAILURES)} check(s) failed")
        return 1
    print("\nAll controller Python API checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
