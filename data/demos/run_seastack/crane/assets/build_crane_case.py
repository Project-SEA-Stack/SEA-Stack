"""Author crane kinematics, body-frame meshes, and YAML from one layout.

Run from this directory or the SEA-Stack root:

    python data/demos/run_seastack/crane/assets/build_crane_case.py

Writes:
  geometry/pedestal.obj, boom.obj, hook.obj, payload.obj, jib_ref.obj
  common/crane.model.yaml, common/crane.ropes.yaml, common/crane.simulation.yaml
  lift_*/crane.setup.yaml and lift_*/crane.controller.yaml

BEM meshes and the platform OBJ are left to generate_crane_platform.py /
generate_vessel.py (hull coefficients must not drift).
Hand-edited: common/crane.hydro.yaml.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

ASSETS = Path(__file__).resolve().parent
CRANE_ROOT = ASSETS.parent
COMMON = CRANE_ROOT / "common"
GEOM = ASSETS / "geometry"

# ---------------------------------------------------------------------------
# Layout inputs (metres, radians, z-up). Edit here, not the generated YAML.
# ---------------------------------------------------------------------------
VESSEL_COG_WORLD = np.array([0.0, 0.0, -1.75])
SEAT_BODY_LOCAL = np.array([15.0, 0.0, 5.25])
PEDESTAL_H = 4.5
# Longer boom/jib + lower parked elevation keep the landing tip mid-workspace
# (landing reach ~81% of Lb+Lj at fixed hover tip z; previous 8+12 @ 70 deg was outside).
BOOM_LEN = 10.0
JIB_LEN = 14.0
BOOM_ELEV0_DEG = 63.0  # parked over the stern, mid-workspace
KNUCKLE_DEG = 60.0  # boom_elev - jib_elev
SLEW0_DEG = 0.0  # boom in vessel +x (aft); lifts slew to the cradle

BOOM_W, BOOM_H = 0.90, 0.70
PEDESTAL_R = 1.15
JIB_BOX = 0.32
JIB_WALL = 0.010
# The jib is rigid; its steel box section sets the jib mass.
PAYLOAD_MASS = 3000.0
PAYLOAD_SIZE = 1.2
JIB_HEAD_MASS = 150.0
BOOM_MASS = 3500.0
PEDESTAL_MASS = 4000.0

PLATFORM_DECK = np.array([20.0, 16.0, 8.0])
RAIL_TOP_Z = PLATFORM_DECK[2] + 1.0  # 1 m perimeter rails on the deck
# Resting payload centre = deck + half cube; land onto the cradle.
TARGET_WORLD = np.array([20.0, 16.0, PLATFORM_DECK[2] + PAYLOAD_SIZE / 2.0])
HANG0 = 5.0  # hover rope hang [m]; payload_z_hold derived from parked tip

STEEL_E = 2.1e11
STEEL_G = 8.1e10
STEEL_RHO = 7850.0
STEEL_FY = 4.60e8  # S460

# Schedule: hold → move → settle hover → lower → set-down → hold.
HOLD_END_S = 8.0
MOVE_END_S = 28.0  # 20 s traverse (peak slew ~66% of rate limit)
SETTLE_END_S = 34.0
LOWER_END_S = 42.0
SETDOWN_END_S = 50.0  # slower set-down for soft landing
END_S = 54.0
APPROACH_CLEARANCE_M = 0.25  # hover above deck before final set-down
SETDOWN_DEPTH_M = 0.30  # pay below resting centre so rope goes slack on deck
SETDOWN_RATE_M_S = 0.08  # documentation only; actual rate from schedule window

HOIST_EA = 8.0e6
HOIST_C = 8.0e4
HEAD_ATTACH = np.array([0.0, 0.0, -0.20])
PAYLOAD_ATTACH = np.array([0.0, 0.0, 0.60])  # top of 1.2 m cube

# 3D-MC class actuator limits (assumptions; see README).
SLEW_RATE = 0.18
LUFF_RATE = 0.20
KNUCKLE_RATE = 0.18
JOINT_ACCEL = 0.40  # rad/s^2
HOIST_RATE = 1.5  # m/s AHC-class winch
MAX_TRAVEL = 2.20

# DP heading-hold proxy: torsional RSDA about world +z at vessel CoG.
# Izz ~ 1.5e8 kg.m^2; k,c sized for ~1 deg yaw std in Hs=2 seas.
YAW_K_NM_PER_RAD = 5.0e7
YAW_C_NMS_PER_RAD = 5.0e7


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_axis(axis: np.ndarray, a: float) -> np.ndarray:
    k = axis / np.linalg.norm(axis)
    c, s = math.cos(a), math.sin(a)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def _mat_to_quat_wxyz(R: np.ndarray) -> list[float]:
    """Rotation matrix (columns = body axes in world) -> Chrono (w, x, y, z)."""
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    q /= np.linalg.norm(q)
    return [float(v) for v in q]


def _frame_from_x(x_axis: np.ndarray) -> np.ndarray:
    x = x_axis / np.linalg.norm(x_axis)
    up = np.array([0.0, 0.0, 1.0])
    y = np.cross(up, x)
    if np.linalg.norm(y) < 1.0e-6:
        y = np.array([0.0, 1.0, 0.0])
    else:
        y /= np.linalg.norm(y)
    z = np.cross(x, y)
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))


def _fmt(v) -> str:
    if isinstance(v, (list, tuple, np.ndarray)):
        return "[" + ", ".join(f"{float(x):.6f}" for x in v) + "]"
    return f"{float(v):.6f}"


def _box_section_I(a: float, t: float) -> tuple[float, float]:
    inner = a - 2.0 * t
    area = a * a - inner * inner
    i = (a**4 - inner**4) / 12.0
    return area, i


def compute_layout() -> dict:
    boom_e0 = math.radians(BOOM_ELEV0_DEG)
    knuckle = math.radians(KNUCKLE_DEG)
    slew0 = math.radians(SLEW0_DEG)
    jib_e0 = boom_e0 - knuckle

    boom_dir0 = np.array([math.cos(boom_e0), 0.0, math.sin(boom_e0)])
    jib_dir0 = np.array([math.cos(jib_e0), 0.0, math.sin(jib_e0)])
    rz = _rot_z(slew0)
    boom_dir0 = rz @ boom_dir0
    jib_dir0 = rz @ jib_dir0

    pivot_local = SEAT_BODY_LOCAL + np.array([0.0, 0.0, PEDESTAL_H])
    seat_world = VESSEL_COG_WORLD + SEAT_BODY_LOCAL
    pivot_world = VESSEL_COG_WORLD + pivot_local
    knuckle_world = pivot_world + BOOM_LEN * boom_dir0
    tip_world = knuckle_world + JIB_LEN * jib_dir0
    boom_mid = pivot_world + 0.5 * BOOM_LEN * boom_dir0
    pedestal_com = seat_world + np.array([0.0, 0.0, PEDESTAL_H / 2.0])

    boom_R = _frame_from_x(boom_dir0)
    boom_q = _mat_to_quat_wxyz(boom_R)
    jib_R = _frame_from_x(jib_dir0)
    jib_q = _mat_to_quat_wxyz(jib_R)
    ped_R = _rot_z(slew0)
    ped_q = _mat_to_quat_wxyz(ped_R)
    luff_axis_pedestal = np.array([0.0, 1.0, 0.0])
    luff_axis_parked = ped_R @ luff_axis_pedestal
    jib_mid = knuckle_world + 0.5 * JIB_LEN * jib_dir0

    head_attach_world = tip_world + HEAD_ATTACH
    # Hover hang is fixed; payload centre hangs under the parked tip.
    payload_z_hold = float(head_attach_world[2] - HANG0 - PAYLOAD_ATTACH[2])
    payload_com = np.array([tip_world[0], tip_world[1], payload_z_hold])
    rest0 = float(head_attach_world[2] - (payload_com[2] + PAYLOAD_ATTACH[2]))
    if rest0 < 0.5:
        raise RuntimeError(f"initial rest length {rest0:.3f} m is too small")

    area, ixx = _box_section_I(JIB_BOX, JIB_WALL)
    jib_mass = STEEL_RHO * area * JIB_LEN
    p = PAYLOAD_MASS * 9.81
    sag_tip = p * JIB_LEN**3 / (3.0 * STEEL_E * ixx)
    w = jib_mass * 9.81 / JIB_LEN
    sag_self = w * JIB_LEN**4 / (8.0 * STEEL_E * ixx)
    sigma = (p * JIB_LEN) * (JIB_BOX / 2.0) / ixx
    util = sigma / STEEL_FY

    return {
        "seat_body_local": SEAT_BODY_LOCAL,
        "pivot_vessel_local": pivot_local,
        "boom_dir0_vessel_local": boom_dir0,
        "jib_dir0_vessel_local": jib_dir0,
        "slew_axis_vessel_local": np.array([0.0, 0.0, 1.0]),
        "luff_axis_pedestal_local": luff_axis_pedestal,
        "luff_axis_parked_vessel_local": luff_axis_parked,
        "boom_length": BOOM_LEN,
        "jib_length": JIB_LEN,
        "seat_world": seat_world,
        "pivot_world": pivot_world,
        "knuckle_world": knuckle_world,
        "tip_world": tip_world,
        "boom_mid": boom_mid,
        "jib_mid": jib_mid,
        "pedestal_com": pedestal_com,
        "boom_q": boom_q,
        "jib_q": jib_q,
        "ped_q": ped_q,
        "payload_com": payload_com,
        "payload_z_hold": payload_z_hold,
        "rest0": rest0,
        "hold_tip_xy": tip_world[:2],
        "target_tip_xy": TARGET_WORLD[:2],
        "jib_mass": jib_mass,
        "sag_tip": sag_tip,
        "sag_self": sag_self,
        "sigma": sigma,
        "util": util,
        "ixx": ixx,
        "area": area,
    }


def _planar_ik(Lb: float, Lj: float, r: float, dz: float) -> tuple[float, float, float]:
    """2-link planar IK. Returns (reach, knuckle_bend_deg, boom_elev_deg).

    knuckle_bend is the interior fold angle (0 = straight boom+jib).
    """
    R = math.hypot(r, dz)
    if R > Lb + Lj - 1.0e-9 or R < abs(Lb - Lj) + 1.0e-9:
        raise RuntimeError(f"unreachable: r={r:.3f} dz={dz:.3f} R={R:.3f} vs Lb+Lj={Lb+Lj:.3f}")
    ck = (R * R - Lb * Lb - Lj * Lj) / (2.0 * Lb * Lj)
    ck = max(-1.0, min(1.0, ck))
    # Angle between boom and jib vectors (interior complementary of bend).
    between = math.degrees(math.acos(ck))
    knuckle_bend = 180.0 - between
    a = math.atan2(dz, r)
    b = math.acos(max(-1.0, min(1.0, (Lb * Lb + R * R - Lj * Lj) / (2.0 * Lb * R))))
    boom_elev = math.degrees(a + b)
    return R, knuckle_bend, boom_elev


def assert_workspace(L: dict) -> None:
    """Fail the build if the hold→target tip arc leaves the mid-workspace."""
    pivot = L["pivot_world"]
    tip0 = L["tip_world"]
    tip_z = float(tip0[2])  # tip held at parked height throughout
    Lb, Lj = BOOM_LEN, JIB_LEN
    max_reach = Lb + Lj
    payload_bottom = L["payload_z_hold"] - PAYLOAD_SIZE / 2.0
    clear = payload_bottom - RAIL_TOP_Z
    print("Workspace check (tip z fixed at parked height)")
    print(f"  tip_z={tip_z:.2f} m  hang0={HANG0:.2f} m  payload_z_hold={L['payload_z_hold']:.2f} m")
    print(f"  payload bottom vs rail top: {payload_bottom:.2f} vs {RAIL_TOP_Z:.2f}  clear={clear:.2f} m")
    if clear < 1.0:
        raise RuntimeError(f"hover clearance above rails is {clear:.2f} m (< 1.0 m)")

    samples = []
    for s in np.linspace(0.0, 1.0, 9):
        xy = (1.0 - s) * tip0[:2] + s * TARGET_WORLD[:2]
        d = np.array([xy[0], xy[1], tip_z]) - pivot
        r = float(math.hypot(d[0], d[1]))
        dz = float(d[2])
        R, knk, boom_e = _planar_ik(Lb, Lj, r, dz)
        frac = R / max_reach
        samples.append((s, r, dz, R, frac, knk, boom_e))
        print(
            f"  s={s:.2f}  horiz={r:.2f} dz={dz:.2f} reach={R:.2f} "
            f"({100*frac:.0f}%) knuckle={knk:.0f}° boom_elev={boom_e:.0f}°"
        )
        if frac > 0.90:
            raise RuntimeError(
                f"reach {R:.2f} m is {100*frac:.0f}% of {max_reach:.2f} m at s={s:.2f} (limit 90%)"
            )
        # Landing half of the arc must stay mid-workspace (authority + singularity margin).
        if s >= 0.5 and frac > 0.85:
            raise RuntimeError(
                f"landing-side reach {R:.2f} m is {100*frac:.0f}% of {max_reach:.2f} m "
                f"at s={s:.2f} (limit 85%)"
            )
        if not (40.0 <= knk <= 120.0):
            raise RuntimeError(
                f"knuckle bend {knk:.1f}° outside [40, 120] at s={s:.2f}"
            )
    print("  workspace OK")


# ---------------------------------------------------------------------------
# OBJ writers (body-local frames)
# ---------------------------------------------------------------------------


def _write_obj(path: Path, verts: list, quads: list, header: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# {header}\n")
        for x, y, z in verts:
            f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for a, b, c, d in quads:
            f.write(f"f {a + 1} {b + 1} {c + 1} {d + 1}\n")


def _add_box(verts, quads, xmin, xmax, ymin, ymax, zmin, zmax) -> None:
    base = len(verts)
    verts.extend(
        [
            (xmin, ymin, zmin),
            (xmax, ymin, zmin),
            (xmax, ymax, zmin),
            (xmin, ymax, zmin),
            (xmin, ymin, zmax),
            (xmax, ymin, zmax),
            (xmax, ymax, zmax),
            (xmin, ymax, zmax),
        ]
    )
    quads.extend(
        [
            (base + 0, base + 1, base + 2, base + 3),
            (base + 4, base + 5, base + 6, base + 7),
            (base + 0, base + 1, base + 5, base + 4),
            (base + 1, base + 2, base + 6, base + 5),
            (base + 2, base + 3, base + 7, base + 6),
            (base + 3, base + 0, base + 4, base + 7),
        ]
    )


def _add_cyl(verts, quads, z0, z1, r, n=20) -> None:
    base = len(verts)
    for z in (z0, z1):
        for k in range(n):
            th = 2.0 * math.pi * k / n
            verts.append((r * math.cos(th), r * math.sin(th), z))
    for k in range(n):
        k1 = (k + 1) % n
        quads.append((base + k, base + k1, base + n + k1, base + n + k))


def write_crane_objs() -> None:
    GEOM.mkdir(parents=True, exist_ok=True)

    # Pedestal: origin at COM. Column z in [-H/2, +H/2], slew ring, port cab.
    v, q = [], []
    h2 = PEDESTAL_H / 2.0
    _add_cyl(v, q, -h2, h2, PEDESTAL_R, n=24)
    _add_cyl(v, q, h2 - 0.18, h2 + 0.12, PEDESTAL_R + 0.22, n=24)  # slew ring
    _add_box(v, q, -0.55, 0.85, -PEDESTAL_R - 1.35, -PEDESTAL_R + 0.05, -0.4, 1.1)  # cab
    _add_box(v, q, -0.35, 0.65, -PEDESTAL_R - 1.30, -PEDESTAL_R - 0.15, 0.55, 1.05)  # glazing block
    _write_obj(GEOM / "pedestal.obj", v, q, "Pedestal + slew ring + cab (COM origin, +z up)")

    # Boom: origin at COM, +x toward knuckle. Lattice-ish side plates.
    v, q = [], []
    hx, hy, hz = BOOM_LEN / 2.0, BOOM_W / 2.0, BOOM_H / 2.0
    _add_box(v, q, -hx, hx, -hy, hy, -hz, hz)
    _add_box(v, q, -hx, hx, -hy - 0.04, -hy + 0.02, -hz - 0.08, hz + 0.08)
    _add_box(v, q, -hx, hx, hy - 0.02, hy + 0.04, -hz - 0.08, hz + 0.08)
    _add_box(v, q, hx - 0.35, hx + 0.25, -hy - 0.12, hy + 0.12, -hz - 0.12, hz + 0.12)  # knuckle
    for xi in np.linspace(-hx + 1.0, hx - 1.0, 4):
        _add_box(v, q, xi - 0.08, xi + 0.08, -hy, hy, -hz - 0.06, hz + 0.06)
    _write_obj(GEOM / "boom.obj", v, q, "Rigid boom, COM origin, +x to knuckle")

    # Jib reference (body-local along +x, COM origin) — not used as Chrono viz.
    v, q = [], []
    jh, jw = JIB_BOX / 2.0, JIB_BOX / 2.0
    _add_box(v, q, -JIB_LEN / 2.0, JIB_LEN / 2.0, -jw, jw, -jh, jh)
    _write_obj(GEOM / "jib_ref.obj", v, q, "Reference jib box (not the FE mesh)")

    hs = PAYLOAD_SIZE / 2.0
    v, q = [], []
    _add_box(v, q, -hs, hs, -hs, hs, -hs, hs)
    _write_obj(GEOM / "payload.obj", v, q, f"Payload {PAYLOAD_SIZE} m cube")

    v, q = [], []
    _add_cyl(v, q, -0.12, 0.12, 0.28, n=16)
    _add_box(v, q, -0.08, 0.08, -0.08, 0.08, -0.45, 0.05)
    _write_obj(GEOM / "hook.obj", v, q, "Jib-head sheave")


def _model_yaml_common(L: dict, *, name: str, i_pay: float, i_boom_x: float, i_boom_y: float,
                       extra_bodies: str, extra_links: str) -> str:
    return f"""# Generated by assets/build_crane_case.py — parked pose, identity vessel.
# Boom OBJ is body-local (+x to knuckle). Pedestal OBJ origin at COM.
chrono-version: 10.0

model:
  name: {name}
  angle_degrees: false

  data_path:
    type: RELATIVE
    root: "."

  bodies:
    - name: ground
      location: [0.0, 0.0, 0.0]
      fixed: true

    - name: body1
      location: {_fmt(VESSEL_COG_WORLD)}
      orientation: [0.0, 0.0, 0.0]
      mass: 944467.0
      fixed: false
      inertia:
        moments: [16875600.0, 149479167.0, 149479167.0]
        products: [0.0, 0.0, 0.0]
      com:
        location: [0.0, 0.0, 0.0]
        orientation: [0.0, 0.0, 0.0]
      visualization:
        model_file: ../assets/geometry/vessel_hull.obj
        shape_location: [0.0, 0.0, 1.75]
        color: [0.12, 0.18, 0.28]

    - name: pedestal
      location: {_fmt(L['pedestal_com'])}
      orientation: {_fmt(L['ped_q'])}
      mass: {PEDESTAL_MASS}
      fixed: false
      inertia:
        moments: [4500.0, 4500.0, 2900.0]
        products: [0.0, 0.0, 0.0]
      visualization:
        model_file: ../assets/geometry/pedestal.obj
        shape_location: [0.0, 0.0, 0.0]
        color: [0.92, 0.55, 0.12]

    - name: boom
      location: {_fmt(L['boom_mid'])}
      orientation: {_fmt(L['boom_q'])}
      mass: {BOOM_MASS}
      fixed: false
      inertia:
        moments: [{i_boom_x:.1f}, {i_boom_y:.1f}, {i_boom_y:.1f}]
        products: [0.0, 0.0, 0.0]
      visualization:
        model_file: ../assets/geometry/boom.obj
        shape_location: [0.0, 0.0, 0.0]
        color: [0.95, 0.62, 0.15]
{extra_bodies}
    - name: jib_head
      location: {_fmt(L['tip_world'])}
      orientation: [0.0, 0.0, 0.0]
      mass: {JIB_HEAD_MASS}
      fixed: false
      inertia:
        moments: [20.0, 20.0, 20.0]
        products: [0.0, 0.0, 0.0]
      visualization:
        model_file: ../assets/geometry/hook.obj
        shape_location: [0.0, 0.0, 0.0]
        color: [0.85, 0.85, 0.88]

    - name: payload
      location: {_fmt(L['payload_com'])}
      orientation: [0.0, 0.0, 0.0]
      mass: {PAYLOAD_MASS}
      fixed: false
      inertia:
        moments: [{i_pay:.1f}, {i_pay:.1f}, {i_pay:.1f}]
        products: [0.0, 0.0, 0.0]
      contact:
        materials:
          - name: payload_mat
            coefficient_of_friction: 0.6
            coefficient_of_restitution: 0.05
            physical_properties:
              Young_modulus: 2.0e8
              Poisson_ratio: 0.3
            coefficients:
              normal_stiffness: 1.0e7
              normal_damping: 2.0e4
              tangential_stiffness: 5.0e6
              tangential_damping: 1.0e4
        shapes:
          - type: BOX
            material: payload_mat
            dimensions: [{PAYLOAD_SIZE}, {PAYLOAD_SIZE}, {PAYLOAD_SIZE}]
            location: [0.0, 0.0, 0.0]
            orientation: [0.0, 0.0, 0.0]
        collision_family: 5
      visualization:
        model_file: ../assets/geometry/payload.obj
        shape_location: [0.0, 0.0, 0.0]
        color: [0.85, 0.25, 0.18]

    - name: platform
      location: {_fmt(PLATFORM_DECK)}
      orientation: [0.0, 0.0, 0.0]
      mass: 1.0
      fixed: true
      contact:
        materials:
          - name: deck_mat
            coefficient_of_friction: 0.7
            coefficient_of_restitution: 0.02
            physical_properties:
              Young_modulus: 2.0e8
              Poisson_ratio: 0.3
            coefficients:
              normal_stiffness: 1.0e7
              normal_damping: 2.0e4
              tangential_stiffness: 5.0e6
              tangential_damping: 1.0e4
        shapes:
          - type: BOX
            material: deck_mat
            dimensions: [8.0, 6.0, 0.25]
            location: [0.0, 0.0, -0.125]
            orientation: [0.0, 0.0, 0.0]
          - type: BOX
            material: deck_mat
            dimensions: [0.3, 0.3, 1.0]
            location: [-1.2, -1.2, 0.5]
            orientation: [0.0, 0.0, 0.0]
          - type: BOX
            material: deck_mat
            dimensions: [0.3, 0.3, 1.0]
            location: [1.2, -1.2, 0.5]
            orientation: [0.0, 0.0, 0.0]
          - type: BOX
            material: deck_mat
            dimensions: [0.3, 0.3, 1.0]
            location: [-1.2, 1.2, 0.5]
            orientation: [0.0, 0.0, 0.0]
          - type: BOX
            material: deck_mat
            dimensions: [0.3, 0.3, 1.0]
            location: [1.2, 1.2, 0.5]
            orientation: [0.0, 0.0, 0.0]
        collision_family: 6
      visualization:
        model_file: ../assets/geometry/platform.obj
        shape_location: [0.0, 0.0, 0.0]
        color: [0.55, 0.58, 0.62]

  tsdas:
    - name: station_keeping_surge
      type: TSDA
      body1: ground
      body2: body1
      point1: {_fmt(VESSEL_COG_WORLD)}
      point2: {_fmt(VESSEL_COG_WORLD)}
      spring_coefficient: 8.0e4
      damping_coefficient: 8.0e4
      free_length: 0.0
    - name: station_keeping_sway
      type: TSDA
      body1: ground
      body2: body1
      point1: [0.0, 1.0, {VESSEL_COG_WORLD[2]}]
      point2: [0.0, 1.0, {VESSEL_COG_WORLD[2]}]
      spring_coefficient: 8.0e4
      damping_coefficient: 8.0e4
      free_length: 0.0
    # Hoist rope (tension-only force law via setup ropes: override).
    # Chrono MBS YAML Initialize uses absolute points (local=false).
    - name: crane_hoist
      type: TSDA
      body1: jib_head
      body2: payload
      point1: {_fmt(np.asarray(L['tip_world']) + HEAD_ATTACH)}
      point2: {_fmt(np.asarray(L['payload_com']) + PAYLOAD_ATTACH)}
      spring_coefficient: 0.0
      damping_coefficient: 0.0
      free_length: {L['rest0']:.4f}

  # DP heading-hold proxy (torsional spring about world +z at CoG).
  rsdas:
    - name: station_keeping_yaw
      type: RSDA
      body1: ground
      body2: body1
      location: {_fmt(VESSEL_COG_WORLD)}
      axis: [0.0, 0.0, 1.0]
      spring_coefficient: {YAW_K_NM_PER_RAD:.3e}
      damping_coefficient: {YAW_C_NMS_PER_RAD:.3e}
      free_angle: 0.0

  motors:
    - name: crane_slew
      type: ROTATION
      body1: body1
      body2: pedestal
      location: {_fmt(L['seat_world'])}
      axis: [0.0, 0.0, 1.0]
      spindle: REVOLUTE
      actuation_type: POSITION
      actuation_function:
        type: CONTROLLER

    - name: crane_luff
      type: ROTATION
      body1: pedestal
      body2: boom
      location: {_fmt(L['pivot_world'])}
      axis: {_fmt(L['luff_axis_parked_vessel_local'])}
      spindle: REVOLUTE
      actuation_type: POSITION
      actuation_function:
        type: CONTROLLER
{extra_links}
"""


def write_model_yaml(L: dict) -> None:
    i_pay = PAYLOAD_MASS * PAYLOAD_SIZE**2 / 6.0
    i_boom_x = BOOM_MASS * (BOOM_W**2 + BOOM_H**2) / 12.0
    i_boom_y = BOOM_MASS * BOOM_LEN**2 / 12.0
    i_jib = L["jib_mass"] * JIB_LEN**2 / 12.0

    # Rigid lift plant: actuated knuckle + tip head welded to the jib.
    rigid_jib = f"""
    - name: jib
      location: {_fmt(L['jib_mid'])}
      orientation: {_fmt(L['jib_q'])}
      mass: {L['jib_mass']:.1f}
      fixed: false
      inertia:
        moments: [{i_jib:.1f}, {i_jib:.1f}, {i_jib:.1f}]
        products: [0.0, 0.0, 0.0]
      visualization:
        model_file: ../assets/geometry/jib_ref.obj
        shape_location: [0.0, 0.0, 0.0]
        color: [0.95, 0.55, 0.12]
"""
    # Knuckle motor is appended under motors: (extra_links continues that section).
    rigid_links = f"""
    - name: crane_knuckle
      type: ROTATION
      body1: boom
      body2: jib
      location: {_fmt(L['knuckle_world'])}
      axis: {_fmt(L['luff_axis_parked_vessel_local'])}
      spindle: REVOLUTE
      actuation_type: POSITION
      actuation_function:
        type: CONTROLLER

  joints:
    - name: jib_tip_lock
      type: LOCK
      body1: jib
      body2: jib_head
      location: {_fmt(L['tip_world'])}
"""
    (COMMON / "crane.model.yaml").write_text(
        _model_yaml_common(
            L,
            name="crane_rigid_jib",
            i_pay=i_pay,
            i_boom_x=i_boom_x,
            i_boom_y=i_boom_y,
            extra_bodies=rigid_jib,
            extra_links=rigid_links,
        ),
        encoding="utf-8",
    )


def _controller_yaml(L: dict, *, mode: str) -> str:
    """Emit per-case crane.controller.yaml (docs/extending/CONTROLLERS.md)."""
    rest0 = L["rest0"]
    slew_rate, luff_rate, knuckle_rate = SLEW_RATE, LUFF_RATE, KNUCKLE_RATE
    accel = JOINT_ACCEL
    travel = MAX_TRAVEL
    if mode == "3dmc":
        antisway_zeta, antisway_leak = 0.45, 0.10
    else:
        antisway_zeta = antisway_leak = 0.0
    tension_min = 0.0
    return f"""# Generated by assets/build_crane_case.py -- edit the generator, not this file.
# External controller (preview); see docs/extending/CONTROLLERS.md.
#
# Every tick (1 kHz) SEA-Stack sends the sensor channels below to
# crane_controller.py and applies the actuator values it returns.
command: [python, ../assets/crane_controller.py]   # relative to this file
control_dt: 0.001     # [s] = time_step: the IK and winch run every step
timeout_ms: 60000

sensors:
  bodies: [body1, jib_head, payload]
  # body.<name>.pos_* [m], quat_* [-], vel_* [m/s], ang_vel_* [rad/s], world frame
  motors: [crane_slew, crane_luff, crane_knuckle]
  # motor.<name>.angle [rad], angle_dt [rad/s], torque [N*m] (Chrono motor angle)
  tsdas: [crane_hoist]
  # tsda.crane_hoist.length / rest_length [m], length_dt [m/s],
  # force [N] (negative in tension; the rope law is in common/crane.ropes.yaml)

actuators:
  motors: [crane_slew, crane_luff, crane_knuckle]
  # motor.<name>.angle [rad] and angle_dt [rad/s]: set-points sent as -q, -dq/dt
  # (the controller's joint angle q is minus the Chrono motor angle)
  tsdas: [crane_hoist]
  # tsda.crane_hoist.rest_length [m]: winch pay-out

config:
  enabled: true
  mode: {mode}
  diagnostics_csv: outputs/crane_diagnostics.csv   # relative to this file
  payload_mass_kg: {PAYLOAD_MASS}
  payload_body: payload
  jib_head_body: jib_head
  vessel_body: body1
  target_world: {_fmt(TARGET_WORLD)}
  layout:
    present: true
    pivot_vessel_local: {_fmt(L['pivot_vessel_local'])}
    boom_dir0_vessel_local: {_fmt(L['boom_dir0_vessel_local'])}
    jib_dir0_vessel_local: {_fmt(L['jib_dir0_vessel_local'])}
    slew_axis_vessel_local: [0.0, 0.0, 1.0]
    luff_axis_parked_vessel_local: {_fmt(L['luff_axis_parked_vessel_local'])}
    boom_length_m: {BOOM_LEN}
    jib_length_m: {JIB_LEN}
    hold_tip_xy_world: {_fmt(L['hold_tip_xy'])}
    target_tip_xy_world: {_fmt(TARGET_WORLD[:2])}
    payload_z_hold_m: {L['payload_z_hold']:.6f}
    payload_z_land_m: {TARGET_WORLD[2]}
  motors:
    slew: {{ name: crane_slew }}
    luff: {{ name: crane_luff }}
    knuckle: {{ name: crane_knuckle }}
  hoist:
    tsda_name: crane_hoist
    initial_rest_length_m: {rest0:.4f}
    pay_rate_max_m_s: {HOIST_RATE}
    tension_min_frac: {tension_min}
    attach_point_head_local: {_fmt(HEAD_ATTACH)}
    attach_point_payload_local: {_fmt(PAYLOAD_ATTACH)}
  limits:
    slew_rate_rad_s: {slew_rate}
    luff_rate_rad_s: {luff_rate}
    knuckle_rate_rad_s: {knuckle_rate}
    joint_accel_rad_s2: {accel}
    max_travel_rad: {travel}
    filter_tau_s: 0.10
    lead_time_s: 0.12
    antisway_max_m: 1.5
  gains:
    antisway_zeta: {antisway_zeta}
    antisway_leak_s: {antisway_leak}
  schedule:
    hold_end_s: {HOLD_END_S}
    move_end_s: {MOVE_END_S}
    settle_end_s: {SETTLE_END_S}
    lower_end_s: {LOWER_END_S}
    setdown_end_s: {SETDOWN_END_S}
    final_clearance_m: {APPROACH_CLEARANCE_M}
    setdown_depth_m: {SETDOWN_DEPTH_M}
"""


def write_case_yamls(L: dict) -> None:
    cases = [
        ("lift_baseline", "open_loop"),
        ("lift_3dmc", "3dmc"),
    ]
    for folder, mode in cases:
        path = CRANE_ROOT / folder
        path.mkdir(parents=True, exist_ok=True)
        (path / "crane.controller.yaml").write_text(
            _controller_yaml(L, mode=mode), encoding="utf-8"
        )

    setup_tpl = """# {title}
# Generated by assets/build_crane_case.py. The controller runs in Python
# (crane.controller.yaml, preview); needs run_seastack built with
# -DSEASTACK_ENABLE_EXTERNAL=ON (release packages are) and Python 3.
model_file: ../common/crane.model.yaml
simulation_file: ../common/crane.simulation.yaml
hydro_file: ../common/crane.hydro.yaml
controller_file: crane.controller.yaml
ropes_file: ../common/crane.ropes.yaml
output_directory: outputs

output:
  level: standard
  decimation: 33
"""
    (CRANE_ROOT / "lift_baseline" / "crane.setup.yaml").write_text(
        setup_tpl.format(title="Baseline lift — open-loop joints on mean vessel pose."),
        encoding="utf-8",
    )
    (CRANE_ROOT / "lift_3dmc" / "crane.setup.yaml").write_text(
        setup_tpl.format(
            title="3D-MC lift — tip XYZ IK + MRU feedforward + anti-sway + AHC winch."
        ),
        encoding="utf-8",
    )

    (COMMON / "crane.ropes.yaml").write_text(
        f"""# Generated by assets/build_crane_case.py -- tension-only hoist rope (preview).
# Replaces the force law of the model TSDA crane_hoist:
#   T = max(0, EA/L0 (L - L0) + c dL/dt)
# where L0 is the TSDA rest length (the controller's winch pay-out).
ropes:
  - link: crane_hoist
    law: tension_only
    ea_N: {HOIST_EA}               # axial stiffness EA [N]
    damping_Ns_per_m: {HOIST_C}    # axial damping c [N*s/m]
""",
        encoding="utf-8",
    )


def write_simulation_yaml() -> None:
    text = f"""# Generated by assets/build_crane_case.py
# SMC + EulerImplicitLinearized + SparseLU. 3D tip IK + AHC winch on the
# rigid knuckle-boom plant.
chrono-version: 10.0

simulation:
  contact_method: SMC

  time_step: 0.001
  end_time: {END_S}
  enforce_realtime: false
  gravity: [0, 0, -9.81]

  integrator:
    type: EULER_IMPLICIT_LINEARIZED
    time_step: 0.001

  solver:
    type: SPARSE_LU
    lock_sparsity_pattern: true

  visualization:
    type: MODEL_FILE
    render_fps: 30
    enable_shadows: true
    camera:
      vertical: Z
      location: [38.0, -42.0, 26.0]
      target: [30.0, 3.0, 12.0]
"""
    (COMMON / "crane.simulation.yaml").write_text(text, encoding="utf-8")


def main() -> None:
    L = compute_layout()
    print("Crane layout")
    print(f"  pivot world     {_fmt(L['pivot_world'])}")
    print(f"  knuckle world   {_fmt(L['knuckle_world'])}")
    print(f"  tip world       {_fmt(L['tip_world'])}")
    print(f"  payload         {_fmt(L['payload_com'])}  rest0={L['rest0']:.3f} m")
    print(f"  jib mass        {L['jib_mass']:.0f} kg")
    print(f"  sag payload/self {L['sag_tip']:.3f} / {L['sag_self']:.3f} m  util={L['util']:.2f}")
    assert_workspace(L)
    write_crane_objs()
    write_model_yaml(L)
    write_case_yamls(L)
    write_simulation_yaml()
    print("Wrote meshes + common/*.yaml + lift_*/crane.setup.yaml + crane.controller.yaml")


if __name__ == "__main__":
    main()
