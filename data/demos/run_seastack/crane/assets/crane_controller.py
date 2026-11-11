#!/usr/bin/env python3
"""
External crane controller (Python IPC) — 3D-MC / AHC / open-loop lift control
with diagnostics CSV. Attaches via SEA-Stack's external controller interface
(preview, docs/extending/CONTROLLERS.md).

This module is the outer motion-compensation / mission layer. Chrono
ChLinkMotorRotationAngle motors enforce the commanded joint angles; there is
no joint-level PID here. Design notes, IK, modes, and hoist law:

  data/demos/run_seastack/crane/CONTROLLER.md

Sign convention (must match Chrono plant wiring):
  SEA-Stack publishes Chrono-native motor angles (motor.<name>.angle).
  Internal kinematic angles: q = -chrono_angle.
  Outputs write motor.<name>.angle = -q and motor.<name>.angle_dt = -dq/dt.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

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

from seastack_external import ControllerModule, ControllerState, run  # noqa: E402

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]

K_G = 9.81

MODE_OPEN_LOOP = "open_loop"
MODE_AHC = "ahc"
MODE_3DMC = "3dmc"


# ---------------------------------------------------------------------------
# Helpers (same constants / algorithm as crane_subsystem.cpp)
# ---------------------------------------------------------------------------


def Clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def Quintic01(s: float) -> float:
    s = Clamp(s, 0.0, 1.0)
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def Smoothstep(t0: float, t1: float, t: float) -> float:
    if t1 <= t0:
        return 1.0 if t >= t1 else 0.0
    return Quintic01((t - t0) / (t1 - t0))


def _vadd(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vscale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _vdot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vcross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _vlen(a: Vec3) -> float:
    return math.sqrt(_vdot(a, a))


def RotateAbout(axis: Vec3, angle: float, v: Vec3) -> Vec3:
    n = _vlen(axis)
    if n < 1.0e-12:
        return v
    k = _vscale(axis, 1.0 / n)
    c = math.cos(angle)
    s = math.sin(angle)
    # v*c + k.Cross(v)*s + k*(k.Dot(v))*(1-c)
    return _vadd(
        _vadd(_vscale(v, c), _vscale(_vcross(k, v), s)),
        _vscale(k, _vdot(k, v) * (1.0 - c)),
    )


def RotateByQuatWxyz(q: Sequence[float], v: Vec3) -> Vec3:
    """Rotate vector v by unit quaternion q = (w, x, y, z) (Chrono e0..e3)."""
    qw, qx, qy, qz = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    # Chrono ChQuaternion::Rotate: v' = q * (0,v) * q^*
    # Equivalent: t = 2*cross(qvec, v); v' = v + qw*t + cross(qvec, t)
    qvec = (qx, qy, qz)
    t = _vscale(_vcross(qvec, v), 2.0)
    return _vadd(_vadd(v, _vscale(t, qw)), _vcross(qvec, t))


def VesselPoint(pos: Sequence[float], rot: Sequence[float], local: Vec3) -> Vec3:
    return _vadd((float(pos[0]), float(pos[1]), float(pos[2])), RotateByQuatWxyz(rot, local))


def RigidTip(
    layout: Dict[str, Any],
    pos: Sequence[float],
    rot: Sequence[float],
    q_slew: float,
    q_luff: float,
    q_knuckle: float,
) -> Vec3:
    boom = tuple(layout["boom_dir0_vessel_local"])  # type: ignore[assignment]
    jib = tuple(layout["jib_dir0_vessel_local"])  # type: ignore[assignment]
    slew_ax = tuple(layout["slew_axis_vessel_local"])  # type: ignore[assignment]
    boom = RotateAbout(slew_ax, q_slew, boom)
    jib = RotateAbout(slew_ax, q_slew, jib)
    luff_ax = RotateAbout(
        slew_ax, q_slew, tuple(layout["luff_axis_parked_vessel_local"])
    )
    boom = RotateAbout(luff_ax, q_luff, boom)
    jib = RotateAbout(luff_ax, q_luff, jib)
    knuckle_ax = luff_ax
    jib = RotateAbout(knuckle_ax, q_knuckle, jib)
    tip_local = _vadd(
        tuple(layout["pivot_vessel_local"]),
        _vadd(
            _vscale(boom, float(layout["boom_length_m"])),
            _vscale(jib, float(layout["jib_length_m"])),
        ),
    )
    return VesselPoint(pos, rot, tip_local)


def SolveTipIk3(
    layout: Dict[str, Any],
    pos: Sequence[float],
    rot: Sequence[float],
    tx: float,
    ty: float,
    tz: float,
    q_slew: float,
    q_luff: float,
    q_knuckle: float,
) -> Tuple[float, float, float]:
    kH = 1.0e-4
    kIters = 6
    for _ in range(kIters):
        t = RigidTip(layout, pos, rot, q_slew, q_luff, q_knuckle)
        ex = t[0] - tx
        ey = t[1] - ty
        ez = t[2] - tz
        if math.sqrt(ex * ex + ey * ey + ez * ez) < 1.0e-4:
            return q_slew, q_luff, q_knuckle
        ts = RigidTip(layout, pos, rot, q_slew + kH, q_luff, q_knuckle)
        tl = RigidTip(layout, pos, rot, q_slew, q_luff + kH, q_knuckle)
        tk = RigidTip(layout, pos, rot, q_slew, q_luff, q_knuckle + kH)
        a = (ts[0] - t[0]) / kH
        b = (tl[0] - t[0]) / kH
        c = (tk[0] - t[0]) / kH
        d = (ts[1] - t[1]) / kH
        e = (tl[1] - t[1]) / kH
        f = (tk[1] - t[1]) / kH
        g = (ts[2] - t[2]) / kH
        h = (tl[2] - t[2]) / kH
        i = (tk[2] - t[2]) / kH
        det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
        if abs(det) < 1.0e-12:
            return q_slew, q_luff, q_knuckle
        rx, ry, rz = -ex, -ey, -ez
        ds = (rx * (e * i - f * h) - b * (ry * i - f * rz) + c * (ry * h - e * rz)) / det
        dl = (a * (ry * i - f * rz) - rx * (d * i - f * g) + c * (d * rz - ry * g)) / det
        dk = (a * (e * rz - ry * h) - b * (d * rz - ry * g) + rx * (d * h - e * g)) / det
        q_slew += ds
        q_luff += dl
        q_knuckle += dk
    return q_slew, q_luff, q_knuckle


def CorrectTipIkStep(
    layout: Dict[str, Any],
    pos: Sequence[float],
    rot: Sequence[float],
    tx: float,
    ty: float,
    tz: float,
    q_ref_s: float,
    q_ref_l: float,
    q_ref_k: float,
    q_slew: float,
    q_luff: float,
    q_knuckle: float,
    max_step: float,
) -> Tuple[float, float, float]:
    kH = 1.0e-4
    kLambda = 0.05
    t = RigidTip(layout, pos, rot, q_slew, q_luff, q_knuckle)
    ex = t[0] - tx
    ey = t[1] - ty
    ez = t[2] - tz
    ts = RigidTip(layout, pos, rot, q_slew + kH, q_luff, q_knuckle)
    tl = RigidTip(layout, pos, rot, q_slew, q_luff + kH, q_knuckle)
    tk = RigidTip(layout, pos, rot, q_slew, q_luff, q_knuckle + kH)
    Jc = [
        [(ts[0] - t[0]) / kH, (tl[0] - t[0]) / kH, (tk[0] - t[0]) / kH],
        [(ts[1] - t[1]) / kH, (tl[1] - t[1]) / kH, (tk[1] - t[1]) / kH],
        [(ts[2] - t[2]) / kH, (tl[2] - t[2]) / kH, (tk[2] - t[2]) / kH],
    ]
    evec = [-ex, -ey, -ez]
    JTJ = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    JTe = [0.0, 0.0, 0.0]
    for r in range(3):
        for c in range(3):
            for k in range(3):
                JTJ[r][c] += Jc[k][r] * Jc[k][c]
        for k in range(3):
            JTe[r] += Jc[k][r] * evec[k]
    JTJ[0][0] += kLambda
    JTJ[1][1] += kLambda
    JTJ[2][2] += kLambda
    JTe[0] += kLambda * (q_ref_s - q_slew)
    JTe[1] += kLambda * (q_ref_l - q_luff)
    JTe[2] += kLambda * (q_ref_k - q_knuckle)
    a, b, c = JTJ[0][0], JTJ[0][1], JTJ[0][2]
    d, e, f = JTJ[1][0], JTJ[1][1], JTJ[1][2]
    g, h, i = JTJ[2][0], JTJ[2][1], JTJ[2][2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1.0e-12:
        return q_slew, q_luff, q_knuckle
    rx, ry, rz = JTe[0], JTe[1], JTe[2]
    ds = (rx * (e * i - f * h) - b * (ry * i - f * rz) + c * (ry * h - e * rz)) / det
    dl = (a * (ry * i - f * rz) - rx * (d * i - f * g) + c * (d * rz - ry * g)) / det
    dk = (a * (e * rz - ry * h) - b * (d * rz - ry * g) + rx * (d * h - e * g)) / det
    mag = math.sqrt(ds * ds + dl * dl + dk * dk)
    if mag > max_step and mag > 1.0e-12:
        s = max_step / mag
        ds *= s
        dl *= s
        dk *= s
    return q_slew + ds, q_luff + dl, q_knuckle + dk


def StepJoint(
    target: float,
    angle: float,
    rate: float,
    rate_max: float,
    accel_max: float,
    travel: float,
    angle0: float,
    dt: float,
    sat_flag: int,
) -> Tuple[float, float, int]:
    err = target - angle
    desired_rate = err / max(dt, 1.0e-6)
    desired_rate = Clamp(desired_rate, -rate_max, rate_max)
    dr = Clamp(desired_rate - rate, -accel_max * dt, accel_max * dt)
    rate += dr
    rate = Clamp(rate, -rate_max, rate_max)
    da = rate * dt
    if abs(da) + 1.0e-12 < abs(err) and abs(rate) >= rate_max - 1.0e-9:
        sat_flag = 1
    angle += da
    clamped = Clamp(angle, angle0 - travel, angle0 + travel)
    if clamped != angle:
        sat_flag = 1
        angle = clamped
        rate = 0.0
    return angle, rate, sat_flag


def BandClamp(q: float, lo: float, hi: float, clamp_flag: int) -> Tuple[float, int, bool]:
    c = Clamp(q, lo, hi)
    if c != q:
        return c, 1, True
    return q, clamp_flag, False


def SchedulePhase(sched: Dict[str, float], t: float) -> int:
    if t < sched["hold_end_s"]:
        return 0
    if t < sched["move_end_s"]:
        return 1
    if t < sched["settle_end_s"]:
        return 2
    if t < sched["lower_end_s"]:
        return 3
    if t < sched["setdown_end_s"]:
        return 4
    return 5


def PredictVesselPose(
    body_pos: Sequence[float],
    body_vel: Sequence[float],
    body_quat: Sequence[float],
    body_ang_vel: Sequence[float],
    frozen_pos: Sequence[float],
    frozen_rot: Sequence[float],
    frozen: bool,
    t_lead: float,
) -> Tuple[List[float], List[float]]:
    if frozen:
        return list(frozen_pos), list(frozen_rot)
    out_pos = [
        float(body_pos[0]) + float(body_vel[0]) * t_lead,
        float(body_pos[1]) + float(body_vel[1]) * t_lead,
        float(body_pos[2]) + float(body_vel[2]) * t_lead,
    ]
    wx, wy, wz = float(body_ang_vel[0]), float(body_ang_vel[1]), float(body_ang_vel[2])
    qw, qx, qy, qz = (
        float(body_quat[0]),
        float(body_quat[1]),
        float(body_quat[2]),
        float(body_quat[3]),
    )
    dqw = 0.5 * (-wx * qx - wy * qy - wz * qz)
    dqx = 0.5 * (wx * qw + wy * qz - wz * qy)
    dqy = 0.5 * (wy * qw - wx * qz + wz * qx)
    dqz = 0.5 * (wz * qw + wx * qy - wy * qx)
    nw = qw + dqw * t_lead
    nx = qx + dqx * t_lead
    ny = qy + dqy * t_lead
    nz = qz + dqz * t_lead
    n = math.sqrt(nw * nw + nx * nx + ny * ny + nz * nz)
    if n > 1.0e-12:
        nw /= n
        nx /= n
        ny /= n
        nz /= n
    return out_pos, [nw, nx, ny, nz]


# ---------------------------------------------------------------------------
# Config from IPC ExternalInit.config (nested maps; same shape as YAML config:)
# ---------------------------------------------------------------------------


def _f(d: Any, key: str, default: float) -> float:
    if not isinstance(d, dict) or d.get(key) is None:
        return default
    return float(d[key])


def _s(d: Any, key: str, default: str = "") -> str:
    if not isinstance(d, dict) or d.get(key) is None:
        return default
    return str(d[key])


def _b(d: Any, key: str, default: bool) -> bool:
    if not isinstance(d, dict) or d.get(key) is None:
        return default
    return bool(d[key])


def _vec(d: Any, key: str, default: List[float]) -> List[float]:
    if not isinstance(d, dict) or d.get(key) is None:
        return list(default)
    return [float(x) for x in d[key]]


def _parse_mode(raw: Any) -> str:
    s = str(raw or "").lower()
    if s in ("open_loop", "openloop", "baseline"):
        return MODE_OPEN_LOOP
    if s in ("ahc", "heave"):
        return MODE_AHC
    if s in ("3dmc", "threedmc", "full", "compensated"):
        return MODE_3DMC
    raise RuntimeError(f"crane: unknown mode '{raw}' (expected open_loop | ahc | 3dmc)")


def _motor(node: Any) -> Dict[str, str]:
    if not isinstance(node, dict):
        return {"name": "", "type": "rotation"}
    return {"name": _s(node, "name", ""), "type": _s(node, "type", "rotation")}


def NormalizeConfig(cfg: Dict[str, Any]) -> Dict[str, Any]:
    """Fill defaults on the IPC config dict. No file I/O."""
    if not isinstance(cfg, dict):
        raise RuntimeError("crane controller: config must be a dict")
    if cfg.get("file"):
        raise RuntimeError(
            "crane controller: config.file is no longer supported; put parameters "
            "inline under config: in the controller YAML (same as external_pto)."
        )

    motors = cfg.get("motors") if isinstance(cfg.get("motors"), dict) else {}
    hoist_n = cfg.get("hoist") if isinstance(cfg.get("hoist"), dict) else {}
    limits_n = cfg.get("limits") if isinstance(cfg.get("limits"), dict) else {}
    gains_n = cfg.get("gains") if isinstance(cfg.get("gains"), dict) else {}
    sched_n = cfg.get("schedule") if isinstance(cfg.get("schedule"), dict) else {}
    lay_n = cfg.get("layout") if isinstance(cfg.get("layout"), dict) else None

    hoist = {
        "tsda_name": _s(hoist_n, "tsda_name", "crane_hoist"),
        "initial_rest_length_m": _f(hoist_n, "initial_rest_length_m", 5.0),
        "pay_rate_max_m_s": _f(hoist_n, "pay_rate_max_m_s", 1.5),
        "tension_min_frac": _f(hoist_n, "tension_min_frac", 0.0),
        "attach_point_head_local": _vec(
            hoist_n, "attach_point_head_local", [0.0, 0.0, -0.2]
        ),
        "attach_point_payload_local": _vec(
            hoist_n, "attach_point_payload_local", [0.0, 0.0, 0.60]
        ),
    }
    limits = {
        "slew_rate_rad_s": _f(limits_n, "slew_rate_rad_s", 0.18),
        "luff_rate_rad_s": _f(limits_n, "luff_rate_rad_s", 0.20),
        "knuckle_rate_rad_s": _f(limits_n, "knuckle_rate_rad_s", 0.18),
        "joint_accel_rad_s2": _f(limits_n, "joint_accel_rad_s2", 0.40),
        "max_travel_rad": _f(limits_n, "max_travel_rad", 2.20),
        "filter_tau_s": _f(limits_n, "filter_tau_s", 0.10),
        "lead_time_s": _f(limits_n, "lead_time_s", 0.12),
        "antisway_max_m": _f(limits_n, "antisway_max_m", 1.5),
    }
    gains = {
        "antisway_zeta": _f(gains_n, "antisway_zeta", 0.45),
        "antisway_leak_s": _f(gains_n, "antisway_leak_s", 0.10),
    }
    schedule = {
        "hold_end_s": _f(sched_n, "hold_end_s", 8.0),
        "move_end_s": _f(sched_n, "move_end_s", 28.0),
        "settle_end_s": _f(sched_n, "settle_end_s", 34.0),
        "lower_end_s": _f(sched_n, "lower_end_s", 42.0),
        "setdown_end_s": _f(sched_n, "setdown_end_s", 50.0),
        "final_clearance_m": _f(sched_n, "final_clearance_m", 0.25),
        "setdown_depth_m": _f(sched_n, "setdown_depth_m", 0.30),
    }

    layout: Dict[str, Any] = {"present": False}
    if lay_n is not None:
        layout = {
            "present": True,
            "pivot_vessel_local": _vec(lay_n, "pivot_vessel_local", [15.0, 0.0, 9.75]),
            "boom_dir0_vessel_local": _vec(
                lay_n, "boom_dir0_vessel_local", [0.453990, 0.0, 0.891007]
            ),
            "jib_dir0_vessel_local": _vec(
                lay_n, "jib_dir0_vessel_local", [0.998630, 0.0, 0.052336]
            ),
            "slew_axis_vessel_local": _vec(
                lay_n, "slew_axis_vessel_local", [0.0, 0.0, 1.0]
            ),
            "luff_axis_pedestal_local": _vec(
                lay_n, "luff_axis_pedestal_local", [0.0, 1.0, 0.0]
            ),
            "luff_axis_parked_vessel_local": _vec(
                lay_n, "luff_axis_parked_vessel_local", [0.0, 1.0, 0.0]
            ),
            "knuckle_axis_parked_vessel_local": _vec(
                lay_n, "knuckle_axis_parked_vessel_local", [0.0, 1.0, 0.0]
            ),
            "boom_length_m": _f(lay_n, "boom_length_m", 10.0),
            "jib_length_m": _f(lay_n, "jib_length_m", 14.0),
            "hold_tip_xy_world": _vec(lay_n, "hold_tip_xy_world", [33.5, 0.0]),
            "target_tip_xy_world": _vec(lay_n, "target_tip_xy_world", [20.0, 16.0]),
            "payload_z_hold_m": _f(lay_n, "payload_z_hold_m", 11.8),
            "payload_z_land_m": _f(lay_n, "payload_z_land_m", 8.6),
        }
        if "knuckle_axis_parked_vessel_local" not in lay_n:
            layout["knuckle_axis_parked_vessel_local"] = list(
                layout["luff_axis_parked_vessel_local"]
            )
        schedule["hold_end_s"] = _f(lay_n, "hold_end_s", schedule["hold_end_s"])
        schedule["move_end_s"] = _f(lay_n, "move_end_s", schedule["move_end_s"])
        schedule["settle_end_s"] = _f(lay_n, "settle_end_s", schedule["settle_end_s"])
        schedule["lower_end_s"] = _f(lay_n, "lower_end_s", schedule["lower_end_s"])
        schedule["setdown_end_s"] = _f(lay_n, "setdown_end_s", schedule["setdown_end_s"])
        if schedule["settle_end_s"] < schedule["move_end_s"]:
            schedule["settle_end_s"] = schedule["move_end_s"]

    diagnostics_csv = _s(cfg, "diagnostics_csv", "crane_diagnostics.csv")
    diagnostics_dir = _s(cfg, "diagnostics_dir", "")
    if diagnostics_dir and diagnostics_csv and not os.path.isabs(diagnostics_csv):
        diagnostics_csv = os.path.join(diagnostics_dir, diagnostics_csv)

    return {
        "enabled": _b(cfg, "enabled", True),
        "mode": _parse_mode(cfg.get("mode", MODE_OPEN_LOOP)),
        "payload_body": _s(cfg, "payload_body", "payload"),
        "jib_head_body": _s(cfg, "jib_head_body", "jib_head"),
        "vessel_body": _s(cfg, "vessel_body", "body1"),
        "target_world": _vec(cfg, "target_world", [20.0, 16.0, 8.6]),
        "payload_mass_kg": _f(cfg, "payload_mass_kg", 3000.0),
        "slew": _motor(motors.get("slew")),
        "luff": _motor(motors.get("luff")),
        "knuckle": _motor(motors.get("knuckle")),
        "hoist": hoist,
        "limits": limits,
        "gains": gains,
        "schedule": schedule,
        "layout": layout,
        "diagnostics_csv": diagnostics_csv,
    }


CSV_HEADER = (
    "time_s,payload_x_m,payload_y_m,payload_z_m,"
    "target_x_m,target_y_m,target_z_m,"
    "ref_x_m,ref_y_m,ref_z_m,tip_x_m,tip_y_m,tip_z_m,"
    "tip_cmd_x_m,tip_cmd_y_m,tip_cmd_z_m,"
    "err_xy_m,err_z_m,tip_err_3d_m,swing_angle_rad,"
    "slew_angle_rad,luff_angle_rad,knuckle_angle_rad,"
    "rest_length_m,tension_N,hang_m,"
    "sat_slew,sat_luff,sat_knuckle,sat_hoist,slack,"
    "clamp_luff,clamp_knuckle,phase,"
    "antisway_dx_m,antisway_dy_m,mode\n"
)


class CraneController(ControllerModule):
    """3D-MC / AHC / open-loop crane controller with diagnostics CSV."""

    def __init__(self) -> None:
        super().__init__()
        self.config: Dict[str, Any] = {}
        self.payload_mass_kg = 3000.0

        self.initialized_ = False
        self.slew_angle_ = 0.0
        self.luff_angle_ = 0.0
        self.knuckle_angle_ = 0.0
        self.slew0_ = 0.0
        self.luff0_ = 0.0
        self.knuckle0_ = 0.0
        self.slew_rate_ = 0.0
        self.luff_rate_ = 0.0
        self.knuckle_rate_ = 0.0
        self.rest_length_ = 0.0

        self.frozen_pos_: List[float] = [0.0, 0.0, 0.0]
        self.frozen_rot_wxyz_: List[float] = [1.0, 0.0, 0.0, 0.0]

        self.tip_z_hold_ = 0.0
        self.ref_x_ = 0.0
        self.ref_y_ = 0.0
        self.ref_z_ = 0.0
        self.tip_cmd_x_ = 0.0
        self.tip_cmd_y_ = 0.0
        self.tip_cmd_z_ = 0.0
        self.tip_x_ = 0.0
        self.tip_y_ = 0.0
        self.tip_z_ = 0.0
        self.hang_m_ = 0.0
        self.swing_angle_rad_ = 0.0

        self.as_dx_ = 0.0
        self.as_dy_ = 0.0
        self.antisway_frozen_ = False

        self.sat_slew_ = 0
        self.sat_luff_ = 0
        self.sat_knuckle_ = 0
        self.sat_hoist_ = 0
        self.slack_ = 0
        self.clamp_luff_ = 0
        self.clamp_knuckle_ = 0
        self.phase_ = 0

        self.touchdown_logged_ = False
        self.touchdown_time_s_ = -1.0
        self.touchdown_vz_mps_ = 0.0
        self.payload_z_prev_ = 0.0
        self.payload_z_prev_valid_ = False

        self._csv_fp: Optional[Any] = None
        self._csv_open_attempted_ = False
        self.last_tension_N_ = 0.0
        self.clamp_warned_ = False

        self._have_slew = False
        self._have_luff = False
        self._have_knuckle = False
        self._have_hoist = False
        self._have_vessel = False

    def setup(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        self.config = NormalizeConfig(cfg if isinstance(cfg, dict) else {})
        self.payload_mass_kg = float(self.config["payload_mass_kg"])

        self._have_slew = bool(self.config["slew"]["name"])
        self._have_luff = bool(self.config["luff"]["name"])
        self._have_knuckle = bool(self.config["knuckle"]["name"])
        self._have_hoist = bool(self.config["hoist"]["tsda_name"])
        self._have_vessel = bool(self.config["vessel_body"])

        self.rest_length_ = float(self.config["hoist"]["initial_rest_length_m"])
        self.hang_m_ = self.rest_length_
        return {"name": "CraneController", "version": "1.0", "n_states": 0}

    def _open_csv(self) -> None:
        if self._csv_open_attempted_:
            return
        self._csv_open_attempted_ = True
        path = self.config.get("diagnostics_csv") or ""
        if not path:
            return
        try:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            self._csv_fp = open(path, "w", encoding="utf-8", newline="")
            self._csv_fp.write(CSV_HEADER)
            self._csv_fp.flush()
        except OSError as exc:
            sys.stderr.write(f"crane: could not open diagnostics CSV '{path}': {exc}\n")
            self._csv_fp = None

    def step(self, state: ControllerState) -> Dict[str, float]:
        if not self.config.get("enabled", True) or not self._have_hoist:
            return {}

        time = float(state.time)
        dt = float(state.dt)
        cfg = self.config
        hoist = cfg["hoist"]
        sched = cfg["schedule"]
        L = cfg["layout"]
        limits = cfg["limits"]
        gains = cfg["gains"]
        mode = cfg["mode"]
        have_ik = bool(L.get("present"))

        vessel = state.body(cfg["vessel_body"]) if self._have_vessel else None
        jib_head = state.body(cfg["jib_head_body"])
        payload = state.body(cfg["payload_body"])
        hoist_tsda = state.tsda(hoist["tsda_name"])

        slew_name = cfg["slew"]["name"]
        luff_name = cfg["luff"]["name"]
        knuckle_name = cfg["knuckle"]["name"]

        # --- first-call init (mirror OnBeforeStep initialized_ block) ---
        if not self.initialized_:
            if self._have_slew:
                self.slew_angle_ = -state.motor(slew_name).angle
                self.slew0_ = self.slew_angle_
            if self._have_luff:
                self.luff_angle_ = -state.motor(luff_name).angle
                self.luff0_ = self.luff_angle_
            if self._have_knuckle:
                self.knuckle_angle_ = -state.motor(knuckle_name).angle
                self.knuckle0_ = self.knuckle_angle_
            if vessel is not None:
                self.frozen_pos_ = [vessel.pos[0], vessel.pos[1], vessel.pos[2]]
                self.frozen_rot_wxyz_ = [
                    vessel.quat_wxyz[0],
                    vessel.quat_wxyz[1],
                    vessel.quat_wxyz[2],
                    vessel.quat_wxyz[3],
                ]
            self.rest_length_ = float(hoist["initial_rest_length_m"])
            self.hang_m_ = self.rest_length_
            if have_ik:
                self.tip_z_hold_ = RigidTip(
                    L, self.frozen_pos_, self.frozen_rot_wxyz_, 0.0, 0.0, 0.0
                )[2]
            else:
                self.tip_z_hold_ = float(L.get("payload_z_hold_m", 11.8)) + self.hang_m_
            self.initialized_ = True

        self.phase_ = SchedulePhase(sched, time)

        t_sched = time + float(limits["filter_tau_s"]) + dt

        if have_ik:
            hold_x = float(L["hold_tip_xy_world"][0])
            hold_y = float(L["hold_tip_xy_world"][1])
            tgt_x = float(L["target_tip_xy_world"][0])
            tgt_y = float(L["target_tip_xy_world"][1])
        else:
            hold_x = float(cfg["target_world"][0])
            hold_y = float(cfg["target_world"][1])
            tgt_x = hold_x
            tgt_y = hold_y

        u_xy = Smoothstep(sched["hold_end_s"], sched["move_end_s"], t_sched)
        self.ref_x_ = (1.0 - u_xy) * hold_x + u_xy * tgt_x
        self.ref_y_ = (1.0 - u_xy) * hold_y + u_xy * tgt_y

        z_hold = (
            float(L["payload_z_hold_m"])
            if have_ik
            else (float(cfg["target_world"][2]) + 4.0)
        )
        z_rest = (
            float(L["payload_z_land_m"]) if have_ik else float(cfg["target_world"][2])
        )
        z_approach = z_rest + float(sched["final_clearance_m"])
        z_setdown = z_rest - float(sched["setdown_depth_m"])

        if t_sched < sched["settle_end_s"]:
            self.ref_z_ = z_hold
        elif t_sched < sched["lower_end_s"]:
            u = Smoothstep(sched["settle_end_s"], sched["lower_end_s"], t_sched)
            self.ref_z_ = (1.0 - u) * z_hold + u * z_approach
        elif t_sched < sched["setdown_end_s"]:
            u = Clamp(
                (t_sched - sched["lower_end_s"])
                / max(1.0e-6, sched["setdown_end_s"] - sched["lower_end_s"]),
                0.0,
                1.0,
            )
            self.ref_z_ = (1.0 - u) * z_approach + u * z_setdown
        else:
            self.ref_z_ = z_setdown

        self.sat_slew_ = self.sat_luff_ = self.sat_knuckle_ = self.sat_hoist_ = 0
        self.clamp_luff_ = self.clamp_knuckle_ = 0

        joints_live = mode == MODE_3DMC
        ik_pos: List[float] = list(self.frozen_pos_)
        ik_rot: List[float] = list(self.frozen_rot_wxyz_)
        if joints_live and vessel is not None:
            ik_pos, ik_rot = PredictVesselPose(
                vessel.pos,
                vessel.vel,
                vessel.quat_wxyz,
                vessel.ang_vel,
                self.frozen_pos_,
                self.frozen_rot_wxyz_,
                frozen=False,
                t_lead=float(limits["lead_time_s"]),
            )

        self.tip_cmd_x_ = self.ref_x_
        self.tip_cmd_y_ = self.ref_y_
        self.tip_cmd_z_ = self.tip_z_hold_

        attach_z = float(hoist["attach_point_payload_local"][2])
        head_attach_z = float(hoist["attach_point_head_local"][2])
        self.hang_m_ = max(
            0.5, self.tip_cmd_z_ + head_attach_z - (self.ref_z_ + attach_z)
        )

        # Anti-sway
        p = payload.pos
        tip = jib_head.pos
        sx = p[0] - tip[0]
        sy = p[1] - tip[1]
        sz = max(0.1, tip[2] - p[2])
        self.swing_angle_rad_ = math.atan2(math.hypot(sx, sy), sz)

        static_T = self.payload_mass_kg * K_G
        T = max(0.0, -float(hoist_tsda.force))
        if (
            not self.antisway_frozen_
            and self.phase_ >= 4
            and T < 0.10 * static_T
            and self.ref_z_ < z_rest
        ):
            self.antisway_frozen_ = True

        if (
            mode == MODE_3DMC
            and float(gains["antisway_zeta"]) > 0.0
            and not self.antisway_frozen_
        ):
            Lhang = max(1.0, math.hypot(sx, sy, sz))
            w0 = math.sqrt(K_G / Lhang)
            k = 2.0 * float(gains["antisway_zeta"]) * w0
            leak = max(0.0, float(gains["antisway_leak_s"]))
            self.as_dx_ += (k * sx - leak * self.as_dx_) * dt
            self.as_dy_ += (k * sy - leak * self.as_dy_) * dt
            lim = float(limits["antisway_max_m"])
            mag = math.hypot(self.as_dx_, self.as_dy_)
            if mag > lim and mag > 1.0e-9:
                self.as_dx_ *= lim / mag
                self.as_dy_ *= lim / mag
            self.tip_cmd_x_ += self.as_dx_
            self.tip_cmd_y_ += self.as_dy_

        if have_ik:
            q_ol_s, q_ol_l, q_ol_k = SolveTipIk3(
                L,
                self.frozen_pos_,
                self.frozen_rot_wxyz_,
                self.tip_cmd_x_,
                self.tip_cmd_y_,
                self.tip_cmd_z_,
                self.slew_angle_,
                self.luff_angle_,
                self.knuckle_angle_,
            )
            q_s, q_l, q_k = q_ol_s, q_ol_l, q_ol_k
            if joints_live:
                q_s = self.slew_angle_
                q_l = self.luff_angle_
                q_k = self.knuckle_angle_
                for _ in range(3):
                    q_s, q_l, q_k = CorrectTipIkStep(
                        L,
                        ik_pos,
                        ik_rot,
                        self.tip_cmd_x_,
                        self.tip_cmd_y_,
                        self.tip_cmd_z_,
                        q_ol_s,
                        q_ol_l,
                        q_ol_k,
                        q_s,
                        q_l,
                        q_k,
                        0.08,
                    )

            q_ol_l, self.clamp_luff_, _ = BandClamp(q_ol_l, -0.80, 1.20, self.clamp_luff_)
            q_ol_k, self.clamp_knuckle_, _ = BandClamp(
                q_ol_k, -0.90, 1.00, self.clamp_knuckle_
            )
            q_l, self.clamp_luff_, _ = BandClamp(q_l, -0.80, 1.20, self.clamp_luff_)
            q_k, self.clamp_knuckle_, _ = BandClamp(q_k, -0.90, 1.00, self.clamp_knuckle_)
            if (self.clamp_luff_ or self.clamp_knuckle_) and not self.clamp_warned_:
                self.clamp_warned_ = True
                sys.stderr.write(
                    "crane: joint band clamp engaged — tip target may be near "
                    "workspace edge.\n"
                )

            tau = max(1.0e-3, float(limits["filter_tau_s"]))
            a = dt / (tau + dt)
            q_s = self.slew_angle_ + a * (q_s - self.slew_angle_)
            q_l = self.luff_angle_ + a * (q_l - self.luff_angle_)
            q_k = self.knuckle_angle_ + a * (q_k - self.knuckle_angle_)

            rmax_s = float(limits["slew_rate_rad_s"])
            rmax_l = float(limits["luff_rate_rad_s"])
            rmax_k = float(limits["knuckle_rate_rad_s"])
            amax = max(1.0e-3, float(limits["joint_accel_rad_s2"]))
            travel = float(limits["max_travel_rad"])
            self.slew_angle_, self.slew_rate_, self.sat_slew_ = StepJoint(
                q_s,
                self.slew_angle_,
                self.slew_rate_,
                rmax_s,
                amax,
                travel,
                self.slew0_,
                dt,
                self.sat_slew_,
            )
            self.luff_angle_, self.luff_rate_, self.sat_luff_ = StepJoint(
                q_l,
                self.luff_angle_,
                self.luff_rate_,
                rmax_l,
                amax,
                travel,
                self.luff0_,
                dt,
                self.sat_luff_,
            )
            self.knuckle_angle_, self.knuckle_rate_, self.sat_knuckle_ = StepJoint(
                q_k,
                self.knuckle_angle_,
                self.knuckle_rate_,
                rmax_k,
                amax,
                travel,
                self.knuckle0_,
                dt,
                self.sat_knuckle_,
            )

        # Hoist rest length from kinematic tip height vs payload Z reference.
        head_z = jib_head.pos[2]
        if have_ik:
            hoist_frozen = mode == MODE_OPEN_LOOP
            hp = list(self.frozen_pos_)
            hr = list(self.frozen_rot_wxyz_)
            if not hoist_frozen and vessel is not None:
                hp = [vessel.pos[0], vessel.pos[1], vessel.pos[2]]
                hr = [
                    vessel.quat_wxyz[0],
                    vessel.quat_wxyz[1],
                    vessel.quat_wxyz[2],
                    vessel.quat_wxyz[3],
                ]
            tip_kin = RigidTip(
                L, hp, hr, self.slew_angle_, self.luff_angle_, self.knuckle_angle_
            )
            head_z = tip_kin[2]

        z_cmd = self.ref_z_
        min_hang = 0.5
        if head_z - min_hang < z_cmd:
            z_cmd = head_z - min_hang
        target_rest = head_z + head_attach_z - (z_cmd + attach_z)
        if target_rest < 0.5:
            target_rest = 0.5

        in_setdown = self.phase_ >= 4
        static_T = self.payload_mass_kg * K_G
        T = max(0.0, -float(hoist_tsda.force))
        self.slack_ = 1 if T < 1.0 else 0
        if (
            not in_setdown
            and float(hoist["tension_min_frac"]) > 0.0
            and T < float(hoist["tension_min_frac"]) * static_T
        ):
            deficit = (float(hoist["tension_min_frac"]) * static_T - T) / max(
                1.0, float(hoist["tension_min_frac"]) * static_T
            )
            pull_in = (
                0.5 * float(hoist["pay_rate_max_m_s"]) * dt * Clamp(deficit, 0.0, 1.0)
            )
            floor_rest = target_rest - 0.5
            target_rest = max(floor_rest, min(target_rest, self.rest_length_ - pull_in))

        max_delta = float(hoist["pay_rate_max_m_s"]) * dt
        unbounded = target_rest - self.rest_length_
        delta = Clamp(unbounded, -max_delta, max_delta)
        if abs(delta) + 1.0e-12 < abs(unbounded):
            self.sat_hoist_ = 1
        self.rest_length_ += delta
        if self.rest_length_ < 0.5:
            self.rest_length_ = 0.5

        self.tip_x_ = jib_head.pos[0]
        self.tip_y_ = jib_head.pos[1]
        self.tip_z_ = jib_head.pos[2]

        # --- OnAfterStep diagnostics ---
        self.last_tension_N_ = max(0.0, -float(hoist_tsda.force))
        self.slack_ = 1 if self.last_tension_N_ < 1.0 else 0

        pz = payload.pos[2]
        z_rest_td = (
            float(L["payload_z_land_m"]) if have_ik else float(cfg["target_world"][2])
        )
        if not self.touchdown_logged_ and pz <= z_rest_td + 0.05:
            self.touchdown_logged_ = True
            self.touchdown_time_s_ = time
            if self.payload_z_prev_valid_ and dt > 1.0e-9:
                self.touchdown_vz_mps_ = (pz - self.payload_z_prev_) / dt
            sys.stderr.write(
                f"crane: touchdown at t={self.touchdown_time_s_} s, "
                f"vz={self.touchdown_vz_mps_} m/s\n"
            )
        self.payload_z_prev_ = pz
        self.payload_z_prev_valid_ = True

        if cfg.get("diagnostics_csv"):
            self._open_csv()
            if self._csv_fp is not None:
                err_xy = math.hypot(payload.pos[0] - self.ref_x_, payload.pos[1] - self.ref_y_)
                err_z = payload.pos[2] - self.ref_z_
                tip_err = math.sqrt(
                    (self.tip_x_ - self.tip_cmd_x_) ** 2
                    + (self.tip_y_ - self.tip_cmd_y_) ** 2
                    + (self.tip_z_ - self.tip_cmd_z_) ** 2
                )
                tw = cfg["target_world"]
                row = (
                    f"{time},{payload.pos[0]},{payload.pos[1]},{payload.pos[2]},"
                    f"{tw[0]},{tw[1]},{tw[2]},"
                    f"{self.ref_x_},{self.ref_y_},{self.ref_z_},"
                    f"{self.tip_x_},{self.tip_y_},{self.tip_z_},"
                    f"{self.tip_cmd_x_},{self.tip_cmd_y_},{self.tip_cmd_z_},"
                    f"{err_xy},{err_z},{tip_err},{self.swing_angle_rad_},"
                    f"{self.slew_angle_},{self.luff_angle_},{self.knuckle_angle_},"
                    f"{self.rest_length_},{self.last_tension_N_},{self.hang_m_},"
                    f"{self.sat_slew_},{self.sat_luff_},{self.sat_knuckle_},"
                    f"{self.sat_hoist_},{self.slack_},"
                    f"{self.clamp_luff_},{self.clamp_knuckle_},{self.phase_},"
                    f"{self.as_dx_},{self.as_dy_},{mode}\n"
                )
                self._csv_fp.write(row)
                self._csv_fp.flush()

        out: Dict[str, float] = {}
        if self._have_slew:
            out[f"motor.{slew_name}.angle"] = -self.slew_angle_
            out[f"motor.{slew_name}.angle_dt"] = -self.slew_rate_
        if self._have_luff:
            out[f"motor.{luff_name}.angle"] = -self.luff_angle_
            out[f"motor.{luff_name}.angle_dt"] = -self.luff_rate_
        if self._have_knuckle:
            out[f"motor.{knuckle_name}.angle"] = -self.knuckle_angle_
            out[f"motor.{knuckle_name}.angle_dt"] = -self.knuckle_rate_
        if self._have_hoist:
            out[f"tsda.{hoist['tsda_name']}.rest_length"] = self.rest_length_
        return out

    def shutdown(self) -> None:
        if self._csv_fp is not None:
            try:
                self._csv_fp.close()
            except OSError:
                pass
            self._csv_fp = None


if __name__ == "__main__":
    run(CraneController())
