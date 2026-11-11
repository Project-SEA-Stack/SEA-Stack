# Motion-compensated crane (3D-MC control demo, preview)

An offshore workboat (Wigley hull, 50 m) with a rigid knuckle-boom crane and an
active-heave-compensated (AHC) hoist sets a 3 t payload onto a fixed,
pile-supported platform in irregular waves. The Capytaine BEM data include
diffraction from the vessel and the fixed piles.

The crane is driven by an external Python controller
([`assets/crane_controller.py`](assets/crane_controller.py)) through the
external controller interface (preview,
[`docs/extending/CONTROLLERS.md`](../../../../docs/extending/CONTROLLERS.md)).
Two cases run the same sea:

| Case | Mode | What it does |
|------|------|--------------|
| `lift_baseline` | open loop | Joints follow the planned path in the parked vessel frame; hoist uses the parked tip height |
| `lift_3dmc` | 3D-MC | Live tip XYZ inverse kinematics, vessel-motion feedforward, anti-sway, AHC hoist, controlled set-down |

The controller also has an AHC-only mode (`mode: ahc`: open-loop joints, hoist
tracks the live tip height); see [`CONTROLLER.md`](CONTROLLER.md).

**Requires** `SEASTACK_ENABLE_EXTERNAL=ON` and `python` (3.8+, standard
library only) on `PATH`.

**Files.** The plant (vessel, crane bodies, motors, hoist TSDA) is in
`common/crane.model.yaml`. Each case's `crane.setup.yaml` names a
`controller_file` (`crane.controller.yaml`: sensors, actuators and the
controller's inline `config:`) and the shared `ropes_file`
(`common/crane.ropes.yaml`: tension-only hoist rope). Layout, crane meshes and
most YAML come from `assets/build_crane_case.py`; `common/crane.hydro.yaml` is
hand-edited. Platform OBJ and pile BEM mesh: `assets/generate_crane_platform.py`.
Hull mesh: `assets/generate_vessel.py`. BEM: `assets/run_bem.py`. The hull and
BEM scripts need a source checkout (they use `scripts/bem` and the Wigley
demo's generator); the release package ships their outputs.

**Controller design:** [`CONTROLLER.md`](CONTROLLER.md) covers modes,
schedule, kinematics, anti-sway, the hoist law and the sign convention.
Chrono position motors enforce the joint angles; there is no joint PID in
Python.

## Run

```powershell
.\build\bin\Release\run_seastack.exe data\demos\run_seastack\crane\lift_baseline --nogui
.\build\bin\Release\run_seastack.exe data\demos\run_seastack\crane\lift_3dmc --nogui
python data\demos\run_seastack\crane\summarize_runs.py
```

Each run simulates 54 s at 1 kHz (about 15–20 s wall time on a desktop) and
writes `outputs/results.irregular.h5` plus the controller's
`outputs/crane_diagnostics.csv`. `summarize_runs.py` reads both CSVs.

To regenerate the case files after a layout or gain change:

```powershell
python data\demos\run_seastack\crane\assets\build_crane_case.py
```

## Layout

| Item | Value |
|------|-------|
| Vessel | Wigley L = 50 m, B = 12 m, T = 3.5 m, CoG z = −1.75 m |
| Water depth | 40 m |
| Crane seat | (15, 0, 5.25) m in the vessel body; 4.5 m pedestal |
| Parked pose | slew 0° (aft), boom elevation 63°, knuckle 60°; tip ≈ (33.5, 0, 17.6) m in the world frame |
| Cradle / target | (20, 16, 8.6) m (resting payload centre) |
| Boom + jib | 10 m + 14 m, rigid; actuated knuckle motor |
| Workspace | landing reach ≈ 81% of boom + jib at fixed tip z (asserted in the builder) |
| Payload | 3 t; hang 5 m, hover z ≈ 11.8 m |
| Hoist rope | EA and damping in `common/crane.ropes.yaml`; `T = max(0, EA/L0 (L − L0) + c dL/dt)` |
| Waves | JONSWAP Hs = 2.0 m, Tp = 8 s, γ = 3.3, heading 45°, seed 42 |
| Station keeping | surge/sway TSDAs + yaw RSDA (a proxy for DP heading hold) |

Schedule: hold 0–8 s, move 8–28 s, settle hover 28–34 s, lower 34–42 s,
set-down 42–50 s, hold to 54 s.

## Controller assumptions (3D-MC)

Details and derivation: [`CONTROLLER.md`](CONTROLLER.md). In short:

- The tip stays at its parked height for the whole lift; the winch does all
  vertical payload motion.
- Joint rate limits: slew 0.18, luff 0.20, knuckle 0.18 rad/s; acceleration
  limit 0.40 rad/s².
- Hoist pay rate 1.5 m/s; the constant-tension floor is off
  (`tension_min_frac = 0`).
- Joint filter τ = 0.10 s with schedule lead τ + dt; vessel-motion lead 0.12 s.
- Anti-sway: `delta_dot = 2 zeta sqrt(g/L) s − leak delta`, zeta = 0.45,
  leak = 0.1 1/s, offset limit 1.5 m.
- Live IK: warm start from the current angles, then damped Newton steps on the
  predicted vessel pose.
- The controller's joint angle q is minus the Chrono motor angle; it sends
  `motor.<name>.angle = −q`.

## Measured (Hs = 2 m, Tp = 8 s, seed 42)

From `summarize_runs.py`, SEA-Stack v1.1 (dev, November 2026), Windows,
Release build. Swing and peak tension are taken in flight, before set-down:

| Case | Mean tip error, 10–48 s | Settle payload XY, 28–34 s | Max swing, 30–42 s | Peak tension / static, before 42 s | Touchdown speed / XY error |
|------|------|------|------|------|------|
| Open loop | 1.14 m | 1.96 m | 37.5° | 1.28 | 2.0 m/s / 3.9 m (misses the cradle) |
| 3D-MC + anti-sway | **0.079 m** | **0.46 m** | **0.4°** | **1.26** | **0.07 m/s / 0.10 m** |

3D-MC holds the tip fixed in space, damps the pendulum and sets the payload
into the cradle. Open loop shows the plain wave-driven response.

**Luff rate limit.** Compensating vessel roll keeps the luff joint close to
its rate limit. With an earlier limit of 0.16 rad/s the luff saturated during
set-down: the tip rose about 0.3 m above its command and touchdown was
0.31 m/s. At 0.20 rad/s the luff stays below its limit until the payload has
landed. The joint limits are assumptions of this demo, not data for a real
crane.

## How the first 3D-MC version failed

An earlier compensated version held the tip for about 16 s (centimetre tip
error, swing under 5°) and then collapsed. The causes, kept here because they
are typical:

1. **Landing tip outside the workspace.** With the pivot at (15, 0, 8), a tip
   at (20, 16) at the parked tip height needed 20.4 m of reach from a 20 m
   boom + jib. The IK hit the joint band limits, the tip dropped about 5 m, and
   the AHC winch reeled the rope to its 0.5 m floor (tension 0 → 350 kN,
   swing 85°).
2. **Tip lowered by the joints.** Commanding tip z down with the payload put
   the joint motion in the pendulum's resonance band. Industrial 3D-MC holds
   tip z and lowers with the winch only.
3. **Anti-sway law was anti-damping.** An offset `kp s + kd s_dot` feeds
   positive velocity feedback into `s_ddot + (g/L) s = −tip_ddot`. The stable
   law sets the tip velocity proportional to the swing, `tip_dot = k s`, giving
   `s_ddot + k s_dot + (g/L) s = 0` with `zeta = k / (2 sqrt(g/L))`.
4. **No real set-down.** The target z sat 0.15 m above the resting centre, so
   the payload hovered and swung above the deck.

## Tests

- `test_crane_lift` (labels `external;chrono;execute`): 15 s runs of
  `lift_baseline` and `lift_3dmc`; finite output, rope tension never negative,
  no slack after 1 s, 3D-MC tip error below 0.2 m and below a quarter of the
  open-loop error.
- `test_crane_controller_python` (chrono-free): forward and inverse
  kinematics, joint rate/acceleration/travel limits, schedule phases.
