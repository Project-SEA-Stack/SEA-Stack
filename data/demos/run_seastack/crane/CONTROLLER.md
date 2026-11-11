# Crane controller (`crane_controller.py`)

This note documents the Python IPC controller used by `lift_baseline`,
`lift_ahc`, and `lift_3dmc`. It is the outer motion-compensation / mission
layer. Joint tracking is not soft PID in Python: Chrono
`ChLinkMotorRotationAngle` motors enforce the commanded angles.

Source: [`assets/crane_controller.py`](assets/crane_controller.py).  
External controller interface (preview):
[`docs/extending/CONTROLLERS.md`](../../../../docs/extending/CONTROLLERS.md).

---

## Role in the stack

```text
  schedule + tip / payload references
            │
            ▼
  world tip command  (+ anti-sway in 3D-MC)
            │
            ▼
  inverse kinematics  →  kinematic joint angles q
            │
            ▼
  StepJoint rate / accel / travel limits
            │
            ▼
  actuator channels (Chrono motor angles)
            │
            ▼
  position motors (ChLinkMotorRotationAngle)
  + TSDA rest_length on the hoist
```

| Layer | What it does in this demo |
|-------|---------------------------|
| Outer (this module) | Mission schedule, tip/payload references, IK, anti-sway, hoist length law |
| Inner (Chrono motors) | Enforce motor angle / rate as a prescribed motion |
| Plant | Vessel hydrodynamics, multibody crane, payload, tension-only hoist TSDA |

On a real crane the outer layer would still look similar; you would add
actuator dynamics and drive servos (often PID-like cascades) under the
motor setpoints. That inner loop is idealised away here.

---

## Attach and I/O

Per-case file: `crane.controller.yaml` (`controller_file` in the setup file).
The hoist rope law comes from `common/crane.ropes.yaml` (`ropes_file`).

- SEA-Stack passes the YAML `config:` block to `setup()`.
- The module only calls `NormalizeConfig(cfg)`. It does **not** re-parse a
  second YAML file (`config.file` is rejected).
- `control_dt`: `0.001` s (1 kHz). Between ticks the host holds the
  last commands.

**Sensors (each tick):** vessel (`body1`), `jib_head` and `payload` body
state; motor angles and rates (Chrono motor angle); hoist TSDA length,
rest length and force.

**Actuators:**

| Channel | Meaning |
|---------|---------|
| `motor.<name>.angle` | Chrono motor angle set-point [rad] |
| `motor.<name>.angle_dt` | Chrono motor rate set-point [rad/s] |
| `tsda.<name>.rest_length` | Hoist rope rest length (winch pay-out) [m] |

Missing or NaN values hold the previous command (host behaviour).

---

## Sign convention

Must match the Chrono plant wiring:

| Channel | Convention |
|---------|------------|
| Motor angle sensors | Chrono motor angle |
| Internal FK / IK angles `q` | `q = -chrono_angle` |
| Motor actuators | `angle = -q`, `angle_dt = -dq/dt` |

Vessel-frame boom and jib directions in `layout:` are authored for the
kinematic `q` convention, not the Chrono motor sign.

---

## Modes

`config.mode` selects how joints and hoist use vessel motion.

| Mode | Case | Joint IK vessel pose | Hoist tip height | Anti-sway |
|------|------|----------------------|------------------|-----------|
| `open_loop` | `lift_baseline` | Frozen park pose (first tick) | Frozen park FK | Off |
| `ahc` | `lift_ahc` | Frozen park pose | **Live** vessel FK | Off |
| `3dmc` | `lift_3dmc` | **Predicted** live pose + correction | Live vessel FK | On |

Intuition:

- **Open loop** — planned path in the parked vessel frame; waves move the tip.
- **AHC** — joints still open-loop; winch tracks live tip height so payload
  heave is reduced.
- **3D-MC** — tip held in world XYZ (fixed tip *z* for the lift); winch still
  sets hang length; anti-sway nudges tip XY.

Industry practice used here: tip *z* stays at parked height for the whole
lift; vertical payload motion is winch-only. Lowering the tip with the joints
excites the pendulum.

---

## Per-tick algorithm

```text
1. Read bodies / motors / TSDA
2. First tick: seed q from Chrono, snapshot frozen vessel pose, tip_z_hold
3. phase = SchedulePhase(t)
4. Build ref_x, ref_y, ref_z from schedule (time advanced by filter_tau + dt)
5. tip_cmd = (ref_x, ref_y, tip_z_hold)
6. If 3D-MC and anti-sway active: tip_cmd_xy += antisway offset
7. If layout present:
     q_ol = SolveTipIk3(frozen pose, tip_cmd)
     if 3D-MC:
       predict vessel pose (lead_time)
       q = CorrectTipIkStep × 3 (predicted pose, pull toward q_ol)
     else:
       q = q_ol
     BandClamp → low-pass → StepJoint → update q, rates
8. Hoist: target rest length from tip z vs ref_z; rate-limit pay-out/in
9. Write diagnostics CSV (optional)
10. Return motor angle / angle_dt (Chrono sign) + hoist rest_length
```

---

## Mission schedule

Phase boundaries come from `config.schedule` (defaults shown):

| Phase | Until | Tip XY | Payload Z reference |
|-------|--------|--------|---------------------|
| 0 hold | 8 s | hold | `payload_z_hold_m` |
| 1 move | 28 s | quintic blend hold → target | hold Z |
| 2 settle | 34 s | target | hold Z |
| 3 lower | 42 s | target | blend to land + clearance |
| 4 setdown | 50 s | target | blend past land by `setdown_depth_m` |
| 5 done | after | target | setdown Z |

XY and Z use smoothstep / quintic blends so the reference is C²-ish for the
joint filter. Schedule evaluation uses `t + filter_tau_s + dt` so filtering
does not lag the plan.

Anti-sway freezes once phase ≥ 4, tension falls below 10% of static weight,
and the Z reference is below the land height (set-down / slack).

---

## Forward kinematics

`RigidTip(layout, vessel_pos, vessel_rot, q_slew, q_luff, q_knuckle)`:

1. Take parked boom and jib unit directions in the vessel frame.
2. Rotate about slew → luff → knuckle axes (vessel-local, then slewed).
3. Tip in vessel frame:
   `pivot + boom_dir * boom_length + jib_dir * jib_length`.
4. Map to world with vessel position and quaternion.

Knuckle axis is taken equal to the slewed luff axis (pitch-pitch knuckle
boom). Geometry lengths and parked directions are generated by
`build_crane_case.py` into `config.layout`.

---

## Inverse kinematics

Both solvers invert the same `RigidTip` map. There is no analytic closed form;
Jacobians are finite-differenced (`h = 1e-4` rad).

### `SolveTipIk3` — Newton on a fixed vessel pose

Used for:

- open-loop / AHC joint commands, and
- the 3D-MC open-loop reference `q_ol` (always on the **frozen** park pose).

```text
repeat up to 6 times:
  t = RigidTip(q)
  e = t − tip_cmd
  if ||e|| < 1e-4: return q
  J[:,i] ≈ [RigidTip(q + h e_i) − t] / h
  solve J Δq = −e
  q ← q + Δq
```

Warm-started from the current internal angles.

### `CorrectTipIkStep` — damped least squares (3D-MC only)

Run three times on the **predicted** vessel pose. Each step:

```text
t = RigidTip(q) on predicted pose
e = t − tip_cmd
J from finite differences
solve (JᵀJ + λI) Δq = Jᵀ(−e) + λ (q_ol − q)     # λ = 0.05
cap ||Δq|| ≤ 0.08 rad
q ← q + Δq
```

The `λ (q_ol − q)` term pulls toward the parked-pose IK solution so the
joints do not drift into odd configurations while chasing the tip on a
moving base.

### Vessel prediction (`PredictVesselPose`)

For 3D-MC only:

```text
pos'  = pos + vel * lead_time
quat' = integrate quat from world angular velocity over lead_time
```

`lead_time_s` defaults to 0.12 s (MRU-style feedforward from the vessel
body channels already in the IPC state — not a separate sensor model).

### After IK

1. `BandClamp` on luff / knuckle bands (workspace guard).
2. First-order blend toward the IK target with time constant `filter_tau_s`.
3. `StepJoint`: enforce rate max, accel max, and travel from the parked
   angles. Saturation flags go to the diagnostics CSV.

---

## Anti-sway (3D-MC)

Horizontal tip–payload offset:

```text
s_x = payload_x − tip_x
s_y = payload_y − tip_y
L   = hang length (from tip to payload, floored)
ω₀  = sqrt(g / L)
```

Update (leaky velocity law on tip command offset `δ`):

```text
δ̇ = 2 ζ ω₀ s − leak · δ
||δ|| capped at antisway_max_m
tip_cmd_xy += δ
```

Defaults: `antisway_zeta = 0.45`, `antisway_leak_s = 0.1` 1/s,
`antisway_max_m = 1.5` m.

This is intentional damping of the pendulum (`tip_dot ∝ s`), not a PD on
swing that injects positive velocity feedback. See the demo README for the
failure mode of the earlier anti-damping law.

---

## Hoist rest-length law

Vertical payload control is almost entirely the winch:

```text
head_z     = RigidTip(...).z   # frozen or live vessel, per mode
z_cmd      = ref_z             # from schedule (clamped so hang ≥ 0.5 m)
target_L0  = head_z + head_attach_z − (z_cmd + payload_attach_z)
L0        ← L0 + clamp(target_L0 − L0, ± pay_rate_max * dt)
```

Optional `tension_min_frac` can pull in slack before set-down (unused at 0
in the generated cases). During set-down, tension collapse freezes
anti-sway; the rest length still follows the Z schedule into the cradle.

The hoist TSDA force law comes from `ropes_file`: a tension-only rope,
`T = max(0, EA/L0 (L − L0) + c dL/dt)`. Rest length `L0` is the commanded
free length, not a motor angle.

---

## What this is not

- Not joint-level PID / torque control.
- Not a full actuator, winch drum, or hydraulic model.
- Not a certified DP or MRU stack — vessel pose/velocity come from the
  simulation bodies.
- Not a general-purpose crane library — geometry and schedule are this
  demo’s layout.

Use it as a worked example of `ControllerModule` + floating-base tip
tracking, and as a baseline for outer-loop research.

---

## Config map

Inline under `config:` in `crane.controller.yaml` (filled by
`NormalizeConfig` with defaults). Important keys:

| Key | Role |
|-----|------|
| `mode` | `open_loop` \| `ahc` \| `3dmc` |
| `layout` | Pivot, axes, lengths, hold/target tip XY, hold/land Z |
| `motors` | Chrono link names for slew / luff / knuckle |
| `hoist` | TSDA name, initial rest length, pay rate, attach offsets |
| `limits` | Joint rate/accel/travel, filter τ, lead time, anti-sway cap |
| `gains` | `antisway_zeta`, `antisway_leak_s` |
| `schedule` | Phase times, clearance, set-down depth |
| `diagnostics_csv` | Optional CSV path, relative to the controller file |

Regenerate case YAMLs with `assets/build_crane_case.py` after layout or
gain changes.

---

## Diagnostics

When a CSV path is set, each tick appends payload vs reference, tip vs
`tip_cmd`, swing angle, joint angles, rest length, tension, saturation /
clamp flags, phase, anti-sway offsets, and mode. Touchdown is logged to
stderr when payload Z crosses the land height.

`summarize_runs.py` reads the three lift-case CSVs for the comparison table
in the demo README.

---

## Code map

| Symbol | Role |
|--------|------|
| `NormalizeConfig` | Defaults + validation for IPC `config` |
| `CraneController.setup` | Store config, seed hoist length |
| `CraneController.step` | Full outer law |
| `RigidTip` | Forward kinematics |
| `SolveTipIk3` | Newton IK on a fixed vessel pose |
| `CorrectTipIkStep` | Damped LS correction on predicted pose |
| `PredictVesselPose` | Lead-time vessel pose |
| `StepJoint` | Rate / accel / travel limiting |
| `SchedulePhase` | Integer phase from schedule times |
