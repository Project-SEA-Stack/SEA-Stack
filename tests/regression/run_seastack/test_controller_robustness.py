#!/usr/bin/env python3
"""
External controller robustness: run_seastack end to end (preview feature).

Each case copies the sphere winch demo to a temporary folder, swaps in a
controller YAML and/or Python script, runs run_seastack, and checks the exit
code and the console text the user would see. Cases cover:

  - YAML mistakes (typos, unknown elements, read-only actuators, bad control_dt,
    missing script);
  - bad step() returns (wrong type, sensor channel, typo, non-number, NaN);
  - process failures (exception in step/setup, sys.exit, hang -> timeout, bad
    interpreter);
  - print() forwarding;
  - cost per tick at control_dt = time_step (reported, loosely bounded).

Standard library only. Usage:
    python test_controller_robustness.py --exe <path/to/run_seastack>
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import List, Optional

_REPO = Path(__file__).resolve().parents[3]
_CASE = _REPO / "data" / "demos" / "run_seastack" / "iea_sphere" / "controller_winch"
_H5 = _REPO / "data" / "demos" / "run_seastack" / "iea_sphere" / "assets" / "hydroData" / "sphere.h5"
_HELPER_DIR = _REPO / "libs" / "external" / "python"

GOOD_CONTROLLER_YAML = """\
command: [{python}, ctl.py]
control_dt: 0.05
timeout_ms: {timeout_ms}
sensors:
  bodies: [body1]
  tsdas: [winch_rope]
actuators:
  tsdas: [winch_rope]
config:
  gain: 1.0
"""

GOOD_STEP = """\
from seastack_external import ControllerModule, run

class Ctl(ControllerModule):
    def setup(self, cfg):
        self.gain = cfg["gain"]

    def step(self, state):
{body}

if __name__ == "__main__":
    run(Ctl())
"""


def _run_env() -> dict:
    """Child environment: DLL dirs from ctest (Windows) and the helper on PYTHONPATH."""
    env = dict(os.environ)
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    env["PYTHONPATH"] = os.pathsep.join([str(_HELPER_DIR), env.get("PYTHONPATH", "")])
    return env


def _yaml_path(p: Path) -> str:
    return p.as_posix()


class Case:
    def __init__(self, name: str, *, controller_yaml: Optional[str] = None,
                 step_body: Optional[str] = None, script: Optional[str] = None,
                 expect_ok: bool = False, expect: tuple = (), forbid: tuple = (),
                 end_time: float = 1.0, timeout_ms: int = 10000,
                 with_controller: bool = True) -> None:
        self.name = name
        self.controller_yaml = controller_yaml
        self.step_body = step_body
        self.script = script
        self.expect_ok = expect_ok
        self.expect = expect
        self.forbid = forbid
        self.end_time = end_time
        self.timeout_ms = timeout_ms
        self.with_controller = with_controller


def _indent(code: str) -> str:
    return "\n".join("        " + line for line in code.strip("\n").splitlines())


def prepare(case: Case, root: Path) -> Path:
    d = root / case.name
    shutil.copytree(_CASE, d, ignore=shutil.ignore_patterns("outputs", "__pycache__"))
    hydro = d / "iea_sphere_controller_winch.hydro.yaml"
    hydro.write_text(hydro.read_text(encoding="utf-8").replace(
        "../assets/hydroData/sphere.h5", _yaml_path(_H5)), encoding="utf-8")
    sim = d / "iea_sphere_controller_winch.simulation.yaml"
    sim.write_text(sim.read_text(encoding="utf-8").replace(
        "end_time: 60.0", f"end_time: {case.end_time}"), encoding="utf-8")
    setup = d / "iea_sphere_controller_winch.setup.yaml"
    if not case.with_controller:
        setup.write_text(setup.read_text(encoding="utf-8").replace(
            "controller_file: winch.controller.yaml", ""), encoding="utf-8")
        return d

    python = f'"{_yaml_path(Path(sys.executable))}"'
    yaml_text = (case.controller_yaml or GOOD_CONTROLLER_YAML).format(
        python=python, timeout_ms=case.timeout_ms)
    (d / "winch.controller.yaml").write_text(yaml_text, encoding="utf-8")
    if case.script is not None:
        (d / "ctl.py").write_text(case.script, encoding="utf-8")
    else:
        body = case.step_body or 'return {"tsda.winch_rope.rest_length": 30.0}'
        (d / "ctl.py").write_text(GOOD_STEP.format(body=_indent(body)), encoding="utf-8")
    return d


def run(exe: str, case_dir: Path, timeout_s: float = 120.0):
    t0 = time.perf_counter()
    proc = subprocess.run([exe, "--nogui", str(case_dir)], capture_output=True, text=True,
                          encoding="utf-8", errors="replace", env=_run_env(),
                          timeout=timeout_s)
    return proc.returncode, proc.stdout + proc.stderr, time.perf_counter() - t0


def cases() -> List[Case]:
    yaml_head = "command: [{python}, ctl.py]\ntimeout_ms: {timeout_ms}\n"
    rope = "actuators:\n  tsdas: [winch_rope]\n"
    return [
        Case("baseline", expect_ok=True,
             expect=("controller 'ctl.py' attached", "writes tsda.winch_rope.rest_length [m]")),
        # --- YAML mistakes -------------------------------------------------
        Case("yaml_key_typo",
             controller_yaml=yaml_head + "control_dtt: 0.05\n" + rope,
             expect=("unknown key 'control_dtt'", "Did you mean 'control_dt'")),
        Case("yaml_unknown_body",
             controller_yaml=yaml_head + "sensors:\n  bodies: [body2]\n" + rope,
             expect=("body2", "body1")),
        Case("yaml_unknown_tsda",
             controller_yaml=yaml_head + "actuators:\n  tsdas: [winch_rop]\n",
             expect=("winch_rop", "Did you mean 'winch_rope'")),
        Case("yaml_body_actuator",
             controller_yaml=yaml_head + "actuators:\n  bodies: [body1]\n",
             expect=("read-only",)),
        Case("yaml_list_typo",
             controller_yaml=yaml_head + "sensors:\n  body: [body1]\n" + rope,
             expect=("unknown key 'body'", "Did you mean 'bodies'")),
        Case("yaml_control_dt_not_multiple",
             controller_yaml=yaml_head + "control_dt: 0.015\n" + rope,
             expect=("control_dt", "multiple")),
        Case("yaml_missing_script",
             controller_yaml="command: [{python}, no_such_script.py]\n" + rope,
             expect=("no_such_script.py",)),
        Case("yaml_bad_interpreter",
             controller_yaml="command: [no_such_python_xyz, ctl.py]\n" + rope,
             expect=("could not be started",)),
        # --- bad step() returns --------------------------------------------
        Case("return_list", step_body="return [30.0]",
             expect=("must return a dict", "failed at t = 0")),
        Case("return_sensor_channel",
             step_body='return {"body.body1.pos_z": 0.0}',
             expect=("sensor channel",)),
        Case("return_channel_typo",
             step_body='return {"tsda.winch_rope.rest_lenght": 29.0}',
             expect=("unknown actuator channel", "Did you mean 'tsda.winch_rope.rest_length'")),
        Case("return_non_number",
             step_body='return {"tsda.winch_rope.rest_length": "thirty"}',
             expect=("must be a number",)),
        Case("return_nan_holds", expect_ok=True,
             step_body='return {"tsda.winch_rope.rest_length": float("nan")}',
             expect=("holding the previous value",)),
        Case("read_unknown_sensor",
             step_body='return {"tsda.winch_rope.rest_length": state["body.body1.pos_q"]}',
             expect=("body.body1.pos_q", "Did you mean")),
        # --- process failures ----------------------------------------------
        Case("exception_in_step",
             step_body=('if state.time >= 0.5:\n'
                        '    raise RuntimeError("boom in step")\n'
                        'return None'),
             expect=("boom in step", "ctl.py", "failed at t = 0.5")),
        Case("exception_in_setup",
             script=GOOD_STEP.format(body=_indent("return None")).replace(
                 'self.gain = cfg["gain"]', 'self.gain = cfg["no_such_key"]'),
             expect=("failed to start", "no_such_key")),
        Case("sys_exit_in_step",
             step_body="import sys\nsys.exit(3)",
             expect=("failed at t = 0", "exited")),
        Case("hang_times_out", timeout_ms=1000,
             step_body="import time\ntime.sleep(30)\nreturn None",
             expect=("timeout",)),
        # --- console output ------------------------------------------------
        Case("print_forwarded", expect_ok=True,
             step_body=('if state.time == 0.0:\n'
                        '    print("hello from step", self.gain)\n'
                        'return None'),
             expect=("[ctl.py] hello from step 1.0",)),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--exe", required=True, help="path to run_seastack")
    parser.add_argument("--keep", action="store_true", help="keep the temporary folder")
    args = parser.parse_args()

    root = Path(tempfile.mkdtemp(prefix="seastack_ctl_"))
    failures: List[str] = []
    try:
        for case in cases():
            d = prepare(case, root)
            code, out, wall = run(args.exe, d)
            problems = []
            if case.expect_ok and code != 0:
                problems.append(f"exit code {code}, expected 0")
            if not case.expect_ok and code == 0:
                problems.append("exit code 0, expected a failure")
            for text in case.expect:
                if text not in out:
                    problems.append(f"missing text: {text!r}")
            status = "PASS" if not problems else "FAIL"
            print(f"  [{status}] {case.name} (exit {code}, {wall:.1f} s)")
            if problems:
                failures.append(case.name)
                for p in problems:
                    print(f"         {p}")
                print("         --- run_seastack output (last 25 lines) ---")
                for line in out.strip().splitlines()[-25:]:
                    print(f"         {line}")

        # Cost per tick: control_dt = time_step, 10 s (1000 steps), vs no controller.
        every_step = GOOD_CONTROLLER_YAML.replace("control_dt: 0.05", "control_dt: 0.01")
        n_steps = 1000
        with_ctl = prepare(Case("cost_with", controller_yaml=every_step, end_time=10.0), root)
        without = prepare(Case("cost_without", with_controller=False, end_time=10.0), root)
        code_a, out_a, wall_a = run(args.exe, with_ctl)
        code_b, out_b, wall_b = run(args.exe, without)
        per_tick_ms = 1e3 * (wall_a - wall_b) / n_steps
        print(f"  cost: {wall_a:.2f} s with controller, {wall_b:.2f} s without, "
              f"~{per_tick_ms:.3f} ms per tick (includes Python start-up)")
        if code_a != 0 or code_b != 0:
            failures.append("cost runs")
            print((out_a + out_b)[-3000:])
        elif per_tick_ms > 5.0:
            failures.append("cost per tick")
            print("  [FAIL] cost per tick above 5 ms")
    finally:
        if args.keep:
            print(f"kept {root}")
        else:
            shutil.rmtree(root, ignore_errors=True)

    if failures:
        print(f"\nFAILED: {', '.join(failures)}")
        return 1
    print("\nAll controller robustness cases passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
