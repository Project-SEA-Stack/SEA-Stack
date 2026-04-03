# RM3 C++ demos

Reference Model 3: float + submerged plate on a heave prismatic
joint. Two executables share the same masses, meshes, and `rm3.h5`
hydrodynamics; they differ in how the PTO is represented along the
relative heave axis.

## Key parameters (shared geometry)

| Item | Value |
|------|--------|
| Float mass | 725,834 kg |
| Plate mass | 886,691 kg |
| Float CG z | −0.72 m |
| Plate CG z | −21.29 m |
| Heave joint | `ChLinkLockPrismatic` (float–plate) |
| Hydro database | `demos/rm3/hydroData/rm3.h5` |
| Meshes | `float_cog.obj`, `plate_cog.obj` |

## Available demos

| Executable | PTO / coupling | Waves | Notes |
|------------|----------------|-------|--------|
| `demo_rm3_hydraulic_pto` | `RectifiedHydraulicPTO` + `PIController` via `PTOForceFunctor` on `ChLinkTSDA` | JONSWAP irregular (Hs 2.5 m, Tp 8 s, γ 3.3, 200 components, seed 42) | Writes **`rm3_hydraulic_pto.csv`** with PTO diagnostics every step |
| `demo_rm3_irreg_mooring`¹ | Native Chrono `ChLinkTSDA` linear damper **c = 1.2×10⁶** N·s/m | Same spectrum as above | MoorDyn on the plate (hydro/Chrono body index 1, `body2`); `lines_rm3.txt`; 300 s duration |

¹ Requires `SEASTACK_ENABLE_MOORING`. Compare with the YAML mooring case under
`data/demos/run_seastack/rm3/mooring/`.

---

## How PTO + control work (`demo_rm3_hydraulic_pto`)

This demo is the clearest place in the tree to see domain PTO and control
code wired into Chrono.

### 1. Chrono link

The relative motion between float and plate is still constrained by the
prismatic*joint. The PTO is modelled as a **`ChLinkTSDA`**.

Instead of calling `SetDampingCoefficient` on the link, the demo registers a
**`PTOForceFunctor`** (`adapters/chrono/pto_chrono_adapter.h`). On each
evaluation, Chrono passes:

- **`length`**, **`rest_length`**, **`vel`** (extension rate; positive =
  lengthening link)

The functor converts that to **`displacement = length − rest_length`** and
**`velocity = vel`**, then calls the solver-agnostic interface:

```text
IPTOModel::ComputeForce(displacement, velocity, time) → axial force [N]
```

That scalar is the axial force along the TSDA returned to Chrono.

### 2. What `RectifiedHydraulicPTO` does

`RectifiedHydraulicPTO` (`libs/pto/…/rectified_hydraulic_pto.h`) implements
**`IPTOModel`**. It is a simple model of a hydraulic drivetrain:

1. **Cylinder** — piston velocity maps to signed oil flow.
2. **Smoothed rectifier** — flow direction follows a smooth sign of piston
   velocity (no hard switching at v = 0), so the axial force does not jump
   by MN-scale spikes when the relative velocity crosses zero.
3. **HP / LP accumulators** — gas springs set working pressures;
   volumes update from net flow into each side.
4. **Fixed-displacement motor** — differential pressure produces motor
   torque on the hydraulic shaft.
5. **Generator side** — inertia **J**, viscous damping **B**, and an
   optional control torque **`T_gen`** (see below) integrated with forward Euler sub-steps (`num_substeps`) per Chrono step.

The **axial PTO force** returned to Chrono is set by the **HP−LP pressure
difference** and the **cylinder area**, in the direction that opposes extension
for positive Δp (see implementation in `rectified_hydraulic_pto.cpp`).

**Caching:** `ComputeForce` advances internal hydraulic state only when
**`time`** moves forward; repeated calls at the same time (e.g. HHT internal
evaluations) reuse the cached force so the ODE state stays consistent with
one step.

### 3. Where the PI controller sits

The constructor takes an optional **`IController`**:

```cpp
RectifiedHydraulicPTO(params, speed_controller);
```

In this demo, **`speed_controller`** is a **`PIController`**
(`libs/control/pi_controller.h`):

- **Measurement** passed each step: **`motor_speed_`** [rad/s] (hydraulic
  motor / generator shaft in the PTO model).
- **Output**: **`generator_torque`** [N·m] (**`T_gen`**).

The shaft dynamics use:

```text
J · dω/dt = T_motor − T_gen − B·ω
```

So larger positive `T_gen` loads the shaft more and tends to slow the
motor when it is running faster than desired.

The PI law uses **`error = measurement − setpoint`** (here
**ω − ω_setpoint**). The demo sets **ω_setpoint ≈ 104.72 rad/s** (~1000 RPM),
with **output clamped to [0, 1000] N·m** (only **non-negative** braking/load
torque in this setup). **Anti-windup** is conditional integration when
saturated.

The controller is evaluated **once per outer time step** before the hydraulic
sub-stepping loop; generator torque is **held fixed** across those
sub-steps (documented v1 choice in the PTO implementation).

### 4. End-to-end signal flow (one time step)

1. Chrono advances multibody dynamics; TSDA **velocity** is known.
2. `PTOForceFunctor` → `ComputeForce` with that **velocity** and **time**.
3. `RectifiedHydraulicPTO` updates **`T_gen`** from **`PIController::Compute(ω, t)`**.
4. Internal hydraulic states (pressures, volumes, **ω**) advance over
   **`num_substeps`**.
5. Resulting **cylinder force** is returned to Chrono as the TSDA axial force.
6. **`HydroSystem`** still applies radiation, excitation, hydrostatics, etc.,
   on the two bodies as in any other demo.

### 5. What gets logged

The demo records **`PTODiagnostics`** (`GetDiagnostics()`): HP/LP pressure,
oil volumes, motor speed, motor and generator torques, cylinder force, flow,
mechanical and electrical power estimates, plus link kinematics and
**`controller->last_error()`** in the CSV.

---

## Ideas to experiment with

**Hydraulics (same PI gains)**

- **Accumulators:** change `total_volume` or `precharge_pressure` (HP/LP) and
  watch pressure bands and mean power in the CSV.
- **Motor:** `displacement`, `mech_efficiency`, `vol_efficiency` — affect how
  fast **ω** builds for a given Δp.
- **Cylinder:** `piston_area` — trades force vs flow for the same sea state.
- **Sub-steps:** `num_substeps` — coarser sub-stepping can destabilize the
  fast hydraulic transients; finer is costlier.
- **Smoothing:** `velocity_smoothing` (ε for the rectifier) — too small may
  stiffen zero-crossings; too large smears direction changes.

**Controller**

- **Setpoint** — raise/lower target RPM; observe saturation on `T_gen` and
  `last_error`.
- **`kp` / `ki`** — classic trade-off: faster tracking vs oscillation /
  overshoot; watch integral windup when clamped.
- **`output_max`** — if too low, the motor cannot be loaded enough to regulate
  speed; if very high, check **HP pressure** (a warning logs if HP exceeds
  ~3× precharge).

**Simulation**

- **Sea state** in `demo_rm3_hydraulic_pto.cpp` — `Hs`, `Tp`, `gamma`, `seed`,
  `n_omega`, **`SetRampDuration`**.
- **`timestep`**, **`simulationDuration`**, HHT vs solver type — same as other
  Chrono demos.

**Contrast**

- Run **`demo_rm3_irreg_mooring`** with the **linear damper** TSDA and compare
  heave/PTO behavior to the hydraulic model under similar waves.
- Compare to **`run_seastack`** RM3 YAML cases (linear PTO or hydraulic where
  configured) for workflow differences.

## Related code and docs

- `PTOForceFunctor` — `adapters/chrono/src/pto_chrono_adapter.cpp`
- `RectifiedHydraulicPTO` — `libs/pto/src/hydraulic/rectified_hydraulic_pto.cpp`
- `PIController` — `libs/control/include/seastack/control/pi_controller.h`
- YAML / packaged demos — [`data/demos/run_seastack/rm3/README.md`](../../data/demos/run_seastack/rm3/README.md)
