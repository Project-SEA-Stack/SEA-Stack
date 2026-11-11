#!/usr/bin/env python3
"""
Crane demo controller (data/demos/run_seastack/crane/assets/crane_controller.py):
chrono-free checks of its kinematics and joint limits.

  - forward kinematics at the parked pose matches the hold tip in the case YAML;
  - the tip moves with the vessel (translation and rotation);
  - inverse kinematics round-trips to 1e-4 m, upright and on a rolled vessel;
  - one damped correction step reduces the tip error and respects max_step;
  - StepJoint honours the rate, acceleration and travel limits;
  - SchedulePhase switches at the schedule boundaries.

Standard library only:  python test_crane_controller_python.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO / "libs" / "external" / "python"))
sys.path.insert(0, str(_REPO / "data" / "demos" / "run_seastack" / "crane" / "assets"))

import crane_controller as cc  # noqa: E402

FAILURES = []

# Same values as layout: in the lift cases' crane.controller.yaml.
LAYOUT = {
    "pivot_vessel_local": [15.0, 0.0, 9.75],
    "boom_dir0_vessel_local": [0.453990, 0.0, 0.891007],
    "jib_dir0_vessel_local": [0.998630, 0.0, 0.052336],
    "slew_axis_vessel_local": [0.0, 0.0, 1.0],
    "luff_axis_parked_vessel_local": [0.0, 1.0, 0.0],
    "boom_length_m": 10.0,
    "jib_length_m": 14.0,
}
IDENTITY = (1.0, 0.0, 0.0, 0.0)


def check(cond: bool, msg: str) -> None:
    print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILURES.append(msg)


def dist(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def roll_quat(angle_rad: float):
    """Unit quaternion (w, x, y, z) for a rotation about world x."""
    return (math.cos(angle_rad / 2), math.sin(angle_rad / 2), 0.0, 0.0)


def test_forward_kinematics() -> None:
    print("Forward kinematics")
    tip = cc.RigidTip(LAYOUT, (0, 0, 0), IDENTITY, 0.0, 0.0, 0.0)
    check(dist(tip[:2], (33.520718, 0.0)) < 1e-5,
          f"parked tip XY {tip[0]:.6f}, {tip[1]:.6f} = hold_tip_xy_world (33.520718, 0)")
    moved = cc.RigidTip(LAYOUT, (1.0, -2.0, 0.5), IDENTITY, 0.0, 0.0, 0.0)
    check(dist(moved, (tip[0] + 1.0, tip[1] - 2.0, tip[2] + 0.5)) < 1e-12,
          "vessel translation moves the tip by the same amount")
    slewed = cc.RigidTip(LAYOUT, (0, 0, 0), IDENTITY, math.pi / 2, 0.0, 0.0)
    reach = tip[0] - 15.0
    check(dist(slewed, (15.0, reach, tip[2])) < 1e-9,
          "slew +90 deg about vessel z swings the tip from +x to +y about the pivot")
    q = roll_quat(math.radians(5.0))
    rolled = cc.RigidTip(LAYOUT, (0, 0, 0), q, 0.0, 0.0, 0.0)
    c, s = math.cos(math.radians(5.0)), math.sin(math.radians(5.0))
    expect = (tip[0], c * tip[1] - s * tip[2], s * tip[1] + c * tip[2])
    check(dist(rolled, expect) < 1e-9, "vessel roll rotates the tip about world x")


def test_inverse_kinematics() -> None:
    print("Inverse kinematics")
    for label, rot in (("upright", IDENTITY), ("5 deg roll", roll_quat(math.radians(5.0)))):
        pos = (0.3, -0.2, 0.4)
        q_true = (0.8, -0.3, 0.4)
        target = cc.RigidTip(LAYOUT, pos, rot, *q_true)
        q = cc.SolveTipIk3(LAYOUT, pos, rot, *target, 0.7, -0.2, 0.3)
        reached = cc.RigidTip(LAYOUT, pos, rot, *q)
        check(dist(reached, target) < 1e-4,
              f"{label}: IK reaches the target tip to {dist(reached, target):.1e} m (< 1e-4)")
        check(dist(q, q_true) < 1e-3, f"{label}: IK recovers the joint angles")

    pos, rot = (0.0, 0.0, 0.0), IDENTITY
    target = cc.RigidTip(LAYOUT, pos, rot, 0.5, -0.2, 0.3)
    q0 = (0.45, -0.15, 0.25)
    err0 = dist(cc.RigidTip(LAYOUT, pos, rot, *q0), target)
    q1 = cc.CorrectTipIkStep(LAYOUT, pos, rot, *target, *q0, *q0, max_step=0.01)
    err1 = dist(cc.RigidTip(LAYOUT, pos, rot, *q1), target)
    check(err1 < err0, f"correction step reduces tip error ({err0:.3f} -> {err1:.3f} m)")
    check(dist(q1, q0) <= 0.01 + 1e-12, "correction step is no larger than max_step")


def test_step_joint() -> None:
    print("StepJoint limits")
    dt, rate_max, accel_max, travel = 0.01, 0.2, 1.0, 0.5

    angle, rate, sat = cc.StepJoint(1.0, 0.0, 0.0, rate_max, accel_max, travel, 0.0, dt, 0)
    check(abs(rate - accel_max * dt) < 1e-12, "from rest, rate grows by at most accel_max dt")
    check(abs(angle - accel_max * dt * dt) < 1e-12, "angle advances by rate dt")

    angle, rate, sat = cc.StepJoint(1.0, 0.0, rate_max, rate_max, accel_max, travel, 0.0, dt, 0)
    check(abs(rate - rate_max) < 1e-12 and sat == 1,
          "far target at full rate: rate stays at rate_max and the saturation flag is set")

    angle, rate, sat = cc.StepJoint(1.0, 0.499, rate_max, rate_max, accel_max, travel, 0.0, dt, 0)
    check(angle == 0.5 and rate == 0.0 and sat == 1,
          "travel limit: angle clamps to angle0 + travel, rate resets, flag set")

    angle, rate, sat = cc.StepJoint(0.001, 0.0, 0.1, rate_max, accel_max, travel, 0.0, dt, 0)
    check(abs(angle - 0.001) < 1e-12 and sat == 0,
          "small target within one step: reached exactly, no saturation")


def test_schedule() -> None:
    print("SchedulePhase")
    sched = {"hold_end_s": 8.0, "move_end_s": 28.0, "settle_end_s": 33.0,
             "lower_end_s": 42.0, "setdown_end_s": 48.0}
    times = (0.0, 7.999, 8.0, 27.999, 28.0, 33.0, 42.0, 48.0, 60.0)
    phases = [cc.SchedulePhase(sched, t) for t in times]
    check(phases == [0, 0, 1, 1, 2, 3, 4, 5, 5], f"phases at {times}: {phases}")


def main() -> int:
    test_forward_kinematics()
    test_inverse_kinematics()
    test_step_joint()
    test_schedule()
    if FAILURES:
        print(f"\nFAILED: {len(FAILURES)} check(s)")
        return 1
    print("\nCrane controller checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
