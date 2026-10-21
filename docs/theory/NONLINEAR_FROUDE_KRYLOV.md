# Nonlinear Froude-Krylov and hydrostatic forces

**Status:** design note for review (Stage 2a). No code implements this yet.
Sections 9 and 10 fix the verification cases and pass criteria before any
results exist.

## 1. Summary

SEA-Stack will compute the static and incident-wave pressure on the
**instantaneous wetted surface** of a body mesh. Clipping uses the incident
wave elevation. The result replaces calm-water nonlinear buoyancy for the
bodies that enable it. Radiation and diffraction stay linear.

- **Method.** Weakly nonlinear Froude-Krylov (NL FK), as in WEC-Sim
  `nonlinearHydro = 2` (Lawson et al. 2014), building on Gilloteaux (2007).
- **Pressure models.** Two choices, per body:
  - `airy_wheeler` (default): linear Airy pressure, Wheeler-stretched above
    the mean waterline.
  - `hydrostatic`: long-wave pressure $\rho g(\eta - z)$, with no decay of
    the wave pressure with depth.
- **No double counting.** The BEM excitation already contains the linear
  Froude-Krylov force. The component subtracts the linear FK it would compute
  itself on the mean wetted mesh, so the total is the BEM excitation plus the
  nonlinear correction.
- **Opt-in per body.** It requires nonlinear hydrostatics on the same body,
  because static and wave pressure must be integrated over the same surface.

## 2. Why calm-water nonlinear buoyancy fails in waves

`NonlinearHydrostaticsComponent` integrates buoyancy below the calm waterplane
$z = 0$. The linear excitation contains the linear FK force on the mean hull,
which grows in proportion to wave amplitude. Once the relative motion
approaches the draft or freeboard, the buoyancy saturates (it cannot exceed the
full hull volume or fall below zero), but the excitation does not. Nothing
balances it, and the body is thrown clear of the water.

The IEA sphere in regular waves at T = 8 s shows this
(`test_compare_linear_vs_nonlinear_hs_sphere_reg`). At wave amplitude 0.1 m the
nonlinear heave is 0.3% above linear, at 1.0 m it is 2% above, and at 2.5 m it
is 16% above. At H = 10 m the sphere leaves the water and then dives 33 m.

**Buoyancy below the wave surface is not a fix on its own.** For a wall-sided
body with waterplane area $A$, heave $z$ and local elevation $\eta$:

$$\rho g V_{\text{below }\eta} \approx \rho g (V_0 - A z) + \rho g A \eta .$$

The last term is the long-wave Froude-Krylov force. Adding it to the BEM
excitation counts the wave force about twice in small waves, which is the
symptom of WEC-Sim issue #1485. Any free-surface buoyancy therefore needs the
linear-FK subtraction in section 6.

## 3. Methods considered

| Method | Captures | Status |
|---|---|---|
| Linear hydrostatics + linear excitation | Small motion only | Existing default |
| Nonlinear hydrostatics, calm waterplane (WEC-Sim `nonlinearHydro = 1` analogue) | Hull shape in still water | Existing (`hydrostatics: nonlinear`) |
| **NL FK, Airy + Wheeler pressure on the instantaneous wetted surface** (WEC-Sim `nonlinearHydro = 2`; Lawson et al. 2014; Gilloteaux 2007) | Hull shape and wave pressure up to the incident surface | **Chosen**, default pressure model |
| **NL FK, long-wave (hydrostatic) pressure** up to the incident surface | As above, but the wave pressure does not decay with depth | **Option** (`froude_krylov_pressure: hydrostatic`) |
| Analytical NL FK for axisymmetric bodies (Giorgi & Ringwood 2017, 2018) | Closed-form integrals for simple shapes, with the waterline flat at $\eta$ on the body axis | Independent verification reference only |
| Nonlinear incident waves, e.g. Rienecker-Fenton (Penalba et al. 2017) | Steep-wave kinematics | Out of scope |
| Fully nonlinear potential flow or CFD | Everything above, plus nonlinear diffraction and radiation | Out of scope |

**Why mesh pressure integration.** It works for any closed mesh, all 6 DOF and
multiple bodies. It is the most widely used approach in WEC codes, and
published results exist to compare against (section 10).

**Why offer the long-wave option.** It needs only the wave elevation, and it is
accurate when the draft is small compared with the wavelength. The pressure
ratio $e^{-kd}$ at keel depth $d$ shows the error. For the IEA sphere
($d = 5$ m) the ratio is 0.73 at T = 8 s and 0.57 at T = 6 s, so the long-wave
model overpredicts the wave pressure there. Ship-stability work uses the same
idea for righting levers in waves; a reference will be added after review.
Its cost is close to the Airy model's, because evaluating $\eta$ at the
vertices dominates.

## 4. Conventions

- **World frame:** $Z$ up, origin on the mean water level, $x$ along wave
  heading 0. Units are SI.
- **BEM frame = world frame.** This is already assumed by the existing
  excitation components, which reference wave phase to the world origin and
  ignore the body's horizontal position. The mesh linear FK in section 6 uses
  the same assumption, so the subtraction is consistent with the BEM data.
  Bodies placed away from their BEM position get a startup warning (section 7).
- **Mesh contract** (unchanged from nonlinear hydrostatics):
  - a closed OBJ triangle mesh with outward normals, in the BEM frame;
  - at build time it is translated by $-\mathbf{r}_{CG}$ (`HydroData::cg`)
    into the body CG frame.
- **Body pose:** `BodyState::position` is the CG in world coordinates.
  `orientation_rpy` gives $R = R_z(\psi) R_y(\theta) R_x(\phi)$ (Chrono Cardan
  XYZ).
- **Force and moment:** world axes, moment about the instantaneous CG. This
  matches `NonlinearHydrostaticsComponent` and the Chrono force attacher.
- **Incident wave**, as in `LinearDirectionalWaveField`:

  $$\eta(\mathbf{x},t) = r(t)\sum_i a_i \cos(\mathbf{k}_i\cdot\mathbf{x} - \omega_i t + \phi_i),
  \qquad \mathbf{k}_i = k_i(\cos\beta_i, \sin\beta_i).$$

  - $r(t)$ is the excitation ramp, with the same shape as the excitation
    component it corrects: $\tfrac12(1-\cos(\pi t/T_r))$ for
    frequency-domain excitation, $t/T_r$ for excitation IRF convolution, and 1
    for $t \ge T_r$. The IRF component ramps $\eta(t-\tau)$ inside the
    convolution, while NL FK applies $r(t)$; the two differ only during the
    ramp.
  - **The ramp is applied to both $\eta$ and the wave pressure**, so clipping,
    pressure and the subtracted linear FK all follow the same envelope as the
    BEM excitation. (`WaveBase::GetElevation` itself stays unramped; the
    component applies $r(t)$.)
- **Pressure:** gauge pressure, zero at the free surface. Static part
  $-\rho g z$.

## 5. Pressure models

The total pressure at a wetted point is

$$p(\mathbf{x},t) = -\rho g z + p_d(\mathbf{x},t).$$

### 5.1 `airy_wheeler` (default)

$$p_d = \rho g\, r(t)\sum_i a_i\, Z_i(z_s)\cos(\mathbf{k}_i\cdot\mathbf{x} - \omega_i t + \phi_i)$$

- **Depth factor:**
  - finite depth: $Z_i(z) = \cosh k_i(z+h) / \cosh k_i h$;
  - deep water: $Z_i(z) = e^{k_i z}$, used when the existing `is_in_deep_water`
    test holds ($kh > 89.4$, or $h \le 0$ or infinite).
- **Wheeler stretching** of the vertical coordinate for $z \le \eta$:

  $$z_s = h\,\frac{z - \eta}{h + \eta}$$

  - In deep water this becomes $z_s = z - \eta$, the $h \to \infty$ limit.
    The existing `GetWheelerStretchedPosition` returns the unstretched
    position in deep water. That suits velocities, but here it would leave
    $p \ne 0$ at the surface, so the pressure function implements the limit
    itself.
  - Stretching maps $z = \eta$ to $z_s = 0$, so $p_d = \rho g \eta$ there and
    the total pressure is exactly zero on the free surface. A unit test
    checks this.
  - Below the trough ($z \ll \eta$) stretching changes $p_d$ only slightly.

### 5.2 `hydrostatic` (long-wave)

$$p_d = \rho g\, \eta(\mathbf{x}, t), \qquad p = \rho g\,(\eta - z).$$

This is the $kh \to 0$, $kd \to 0$ limit of 5.1. It is also zero on the free
surface.

## 6. Force decomposition and the linear-FK subtraction

For a body with `froude_krylov: nonlinear`:

$$\mathbf{F}_{\text{hydro}} = \mathbf{F}_{\text{rad}} + \mathbf{F}_{\text{exc,BEM}}(t)
 + \underbrace{\Big[\mathbf{F}_{p}(S_w(t)) - \mathbf{F}^{\text{lin}}_{FK,\text{mesh}}(t)\Big]}_{\texttt{NonlinearFroudeKrylovComponent}}$$

- $\mathbf{F}_{\text{rad}}$ and $\mathbf{F}_{\text{exc,BEM}}$ come from the
  existing radiation and excitation components, unchanged. The excitation
  is FK plus diffraction from the H5 file.
- $\mathbf{F}_{p}(S_w) = -\int_{S_w(t)} p\,\mathbf{n}\,dS$ and
  $\mathbf{M}_{p} = -\int_{S_w(t)} p\,(\mathbf{r}-\mathbf{r}_{CG})\times\mathbf{n}\,dS$.
  The integral covers the part of the hull below $\eta$ at the current pose.
  It contains the static buoyancy, so this component **replaces**
  `NonlinearHydrostaticsComponent` for the body.
- **Mesh linear FK.** Same pressure model, on the mean wetted mesh $S_0$: the
  mesh at its BEM pose, clipped at $z = 0$, with no stretching and no
  dependence on body motion. Writing $\theta_i = \phi_i - \omega_i t$:

  $$\mathbf{F}^{\text{lin}}_{FK,\text{mesh}}(t) = r(t)\sum_i a_i\big(\mathbf{A}_i\cos\theta_i + \mathbf{B}_i\sin\theta_i\big)$$

  with the 6-vectors (force, then moment about the CG at its BEM position)

  $$\mathbf{A}_i = -\rho g \int_{S_0} Z_i(z)\cos(\mathbf{k}_i\cdot\mathbf{x})\,\hat{\mathbf{n}}\,dS,
  \qquad
  \mathbf{B}_i = \rho g \int_{S_0} Z_i(z)\sin(\mathbf{k}_i\cdot\mathbf{x})\,\hat{\mathbf{n}}\,dS .$$

  - Here $\hat{\mathbf{n}}$ is the generalised normal: $\mathbf{n}$ for force,
    $(\mathbf{r}-\mathbf{r}_{CG})\times\mathbf{n}$ for moment.
  - For `hydrostatic`, $Z_i = 1$.
  - $\mathbf{A}_i$ and $\mathbf{B}_i$ are computed once at build time, with the
    same quadrature as $\mathbf{F}_p$. They then form the exact linearisation
    of the discrete operator, so discretisation error cancels in the
    small-amplitude limit.
  - The form matches `ExcitationComponent` (real part times $\cos\theta$,
    imaginary part times $\sin\theta$).

**Limits** (all checked by tests):

- **Still water** ($\eta \equiv 0$): $S_w$ is the hull below $z = 0$,
  $p = -\rho g z$, and the correction term is zero. The component returns the
  same force as `NonlinearHydrostaticsComponent`: $\rho g V \mathbf{e}_z$ at
  the centre of buoyancy, to round-off (section 8).
- **Small amplitude and motion:** $\mathbf{F}_p \approx \rho g V_0\mathbf{e}_z
  - \mathbf{C}\,\boldsymbol{\xi} + \mathbf{F}^{\text{lin}}_{FK,\text{mesh}}
  + O(a^2, a\xi, \xi^2)$. Here $\boldsymbol{\xi}$ is the body displacement
  from equilibrium and $\mathbf{C}$ the hydrostatic stiffness of the mesh. So
  the total tends to the linear model, with the mesh stiffness in place of the
  H5 stiffness.
- **Fixed body:** the correction is $O(a^2)$. A doubled wave force (WEC-Sim
  #1485) would leave an $O(a)$ residual.

**What stays linear:** diffraction (inside the BEM excitation) and radiation,
both about the mean position. This is the standard weakly nonlinear
assumption; section 11 lists the validity limits.

## 7. Wiring per body

| `hydrostatics` | `froude_krylov` | Components for this body |
|---|---|---|
| linear | linear | `HydrostaticsComponent` (unchanged) |
| nonlinear | linear | `NonlinearHydrostaticsComponent` (unchanged; Stage 1 warning in waves) |
| nonlinear | nonlinear | `NonlinearFroudeKrylovComponent` (static + wave pressure, minus mesh linear FK) |
| linear | nonlinear | Error: static and wave pressure must be integrated over the same instantaneous wetted surface |

Radiation and excitation components are unchanged and cover all bodies.

**Build-time checks:**

- `froude_krylov: nonlinear` needs a `mesh_file`.
- It needs `include_excitation` true, because the subtraction assumes the BEM
  excitation is applied.
- The wave field must expose discrete components. Eta-table waves without
  components are rejected.
- In still water, the run logs that NL FK reduces to nonlinear hydrostatics.
- **Self-check:** where the H5 file has a finite `excitation/froude-krylov`
  dataset (Capytaine), compare it with the mesh linear FK at the wave
  frequencies and warn above 10% in the dominant DOF. This catches mesh scale
  and frame errors. sphere.h5 (WAMIT) stores NaN there, so no check runs.
- **Position check:** warn if a body's initial horizontal position differs from
  its BEM CG by more than 1% of the shortest wavelength (BEM frame = world
  frame assumption).

**YAML** (body level, flat keys):

```yaml
- name: body1
  h5_file: ../assets/hydroData/sphere.h5
  mesh_file: ../assets/geometry/sphere.obj   # closed hull, BEM frame
  hydrostatics: nonlinear                    # linear (default) | nonlinear
  froude_krylov: nonlinear                   # linear (default) | nonlinear
  froude_krylov_pressure: airy_wheeler       # airy_wheeler (default) | hydrostatic
```

## 8. Discretisation

**Clipping.**

- At each vertex, evaluate $\eta_v = \eta(x_v, y_v, t)$ and the signed height
  $d_v = z_v - \eta_v$.
- Triangles with all $d_v < 0$ are wet; those with all $d_v \ge 0$ are dry.
  Cut triangles are split on their edges by linear interpolation of $d$.
  `WettedSurfacePressureIntegrator` does this with its own clipping.
  `MeshSubmergedVolume` is left untouched so that nonlinear hydrostatics
  results stay bit-identical.
- This is exact when $\eta$ is linear along each edge, and second order in edge
  length otherwise.
- No waterplane cap is needed, because the pressure is zero on the free
  surface.

**Pressure quadrature.** Pressure is evaluated at wet vertices. At clip points
it is set to zero, since they lie on the free surface by construction. Over
each wet (sub-)triangle $T$ with area $A_T$, normal $\mathbf{n}_T$, vertices
$\mathbf{r}_j$ and pressures $p_j$, the pressure is taken as linear:

$$\mathbf{F}_T = -A_T\,\mathbf{n}_T\,\frac{p_1+p_2+p_3}{3},\qquad
\int_T p\,\mathbf{r}\,dS = \frac{A_T}{12}\sum_{j}\mathbf{r}_j\Big(p_j + \sum_k p_k\Big),$$

with $\mathbf{M}_T = -\big(\int_T p\,(\mathbf{r} - \mathbf{r}_{CG})\,dS\big)\times\mathbf{n}_T$.

- **Static part exact.** $-\rho g z$ is linear, so for a closed mesh the static
  force and moment equal $\rho g V\mathbf{e}_z$ at the centre of buoyancy to
  round-off. This is the still-water identity.
- **Dynamic part second order** in panel size divided by wavelength and by the
  decay length $1/k$. The mesh convergence test measures it.
- **Cost:** each wave component is evaluated once per vertex per force call,
  and the values are shared by clipping and pressure. Every vertex needs its
  true $\eta_v$, wet or dry, because a dry vertex at the end of a cut edge
  sets where the clip point falls.

**Precomputed linear FK.** $A_i$ and $B_i$ use the same integrator on the
mesh at its BEM position, clipped at $z = 0$. Wet vertices carry the linear
pressure shape $\rho g Z_i(z_v)\cos(\mathbf{k}_i\cdot\mathbf{x}_v)$ (and the
sine). Clip points, which lie on $z = 0$, carry the surface value
$\rho g\cos(\mathbf{k}_i\cdot\mathbf{x})$, interpolated along the cut edge. This
is the first-order change of the nonlinear integral as the clip points move
with $\eta$, so the mesh linear FK is the exact linearisation of the discrete
nonlinear force and the difference is $O(a^2)$ (U5). Setting zero at the clip
points instead would leave an $O(a)$ error of order panel size times
waterline length.

**Deviations from WEC-Sim `nonlinearHydro = 2`, and why:**

| Item | WEC-Sim | SEA-Stack | Reason |
|---|---|---|---|
| Partly wet panels | Whole panel wet or dry by centroid | Clipped at vertex $\eta$ | No force jumps as panels cross the surface; faster convergence |
| Pressure on a panel | Centroid value | Linear between vertices | Static part exact; reuses vertex evaluations |
| Linear FK subtracted | Mesh linear FK | Mesh linear FK, same quadrature, precomputed | Exact linearisation of the discrete operator |
| Deep-water stretching | Not specified here | $z_s = z - \eta$ | Keeps $p = 0$ at the surface |
| Ramp | Applied to excitation | Applied to $\eta$, $p_d$ and linear FK | Same envelope as the BEM excitation |
| Update interval | `nonlinearDt` option | Every force evaluation | Add an interval only if the benchmark needs it |

## 9. Verification plan and pass criteria

Tolerances are fixed here, before any results. Relaxing one later needs a
written reason in this file.

**Unit** (`ctest -L unit`):

| # | Test | Pass criterion |
|---|---|---|
| U1 | `GetDynamicPressure` vs closed-form Airy, deep and finite depth | relative error ≤ 1e-12 |
| U2 | Total pressure at $z = \eta$, both models | $\lvert p\rvert \le 10^{-9}\,\rho g a$ |
| U3 | Still-water identity vs `NonlinearHydrostaticsComponent`, box and sphere, several poses, both models | force and moment relative difference ≤ 1e-9 |
| U4 | Frame test: body offset in $x, y$ and rotated, with a wave (WEC-Sim #886) | equals the translated or rotated reference to ≤ 1e-9 |
| U5 | Fixed body: correction scales as $a^2$ (WEC-Sim #1485). Use a pose where the waterplane area changes with height (for a sphere, not at the equator; there the long-wave correction is $O(a^3)$) | fitted log-log slope 2 ± 0.1 for $a$ = 1e-3 to 1e-1 m |
| U6 | Mesh linear FK, truncated vertical cylinder in heave, vs $\rho g a\,e^{-kd}\pi R^2\,2J_1(kR)/(kR)$ | ≤ 1% on the finest mesh |
| U7 | Clipping under a sloping planar surface, `hydrostatic` model (exact case) | force ≤ 1e-9 relative |
| U8 | Mesh convergence, sphere, three resolutions | error decreases monotonically, observed order ≥ 1.5 |
| U9 | Per-body toggle, two bodies | forces on the linear body bit-identical to an all-linear run |
| U9a | Long-wave: wall-sided box in a long wave, heave force minus still-water force vs $\rho g A\eta$; both models agree as $k \to 0$ (sphere, deep water, linear FK heave) | ≤ 1e-9; ≤ 1% at T = 120 s, and the difference falls in proportion to $k$ |
| U10 | YAML: every accepted form and every error case | all forms parse; all errors raised with the stated message |

**Comparison** (`ctest -L comparison`, internal references):

| # | Case | Pass criterion |
|---|---|---|
| C1 | Sphere RAO at steepness 0.0005, NL FK vs linear | amplitude ≤ 1%, phase ≤ 1° |
| C2 | Sphere regular waves vs the independent Python reference, heave only, constant-coefficient radiation: free body at ω = 0.8 rad/s; optimum PTO at ω = 0.8 rad/s; free body at ω = 1.25 rad/s (cases in the table below) | steady amplitude ≤ 1%, phase ≤ 2°; where the reference NL FK differs from linear by more than 2%, the SEA-Stack difference has the same sign and is within 30% of the reference difference |
| C3 | Prescribed heave of sphere and cylinder, force vs Giorgi & Ringwood analytical NL FK, $kR \le 0.15$, $a \le 1.5$ m, deep water | RMS force difference ≤ 2% of the force range |
| C4 | Runaway case (H = 10 m, T = 8 s) | NL FK heave stays within ±(draft + amplitude); calm-water NL case still warns |
| C5 | RM3, NL FK on the float only, spar linear, small steepness | ≤ 1% of all-linear |
| C6 | Four-model regular-wave sweep (linear, calm-water NL, NL FK long-wave, NL FK Airy) | informational; long-wave minus Airy difference grows with $kd$ |
| C7 | Still-water decay, all three NL variants | NL FK Airy and long-wave bit-identical; NL FK vs calm-water NL hydrostatics: max heave difference ≤ 1e-6 m |

**Changes after the Python reference (stage 2a).** The reference
(`tests/verification/utilities/nlfk_reference.py`) showed three criteria were
mis-set. No C++ results existed when these were changed.

- **U5:** at a sphere's equator $dA_w/dz = 0$, so the long-wave correction is
  $O(a^3)$, not $O(a^2)$ (reference slope 3.000). The criterion is unchanged;
  the test pose must avoid that case.
- **U9a:** the Airy and long-wave linear FK differ by about $k$ times the
  draft, because $e^{kz}$ decays over the hull. That is 1.5% at T = 30 s in
  200 m of water, not under 1%. The test now uses T = 120 s in deep water
  (0.09%) and checks first-order convergence in $k$ (observed order 0.999).
- **C3:** Giorgi & Ringwood (2017) take the waterline flat at $\eta$ on the
  body axis, use unstretched deep-water Airy pressure, and approximate the
  horizontal phase. Their model is therefore not ours, and the comparison is
  only meaningful where the two agree. For the IEA sphere under prescribed
  heave (amplitude 1.5 m), the reference gives an RMS gap between the two
  models of 0.10% / 0.62% at $kR$ = 0.14 ($a$ = 0.5 / 1.5 m) and 1.31% at
  $kR$ = 0.31, $a$ = 1.5 m. The criterion is limited to $kR \le 0.15$ so the
  model gap uses at most a third of the 2% budget.
- **C7 (changed in stage 2b, before any C7 run):** "bit-identical" assumed
  NL FK reuses the volume calculation of nonlinear hydrostatics. It does not:
  NL FK integrates the static pressure over the surface, while nonlinear
  hydrostatics uses the divergence theorem on the clipped volume. The two
  agree to round-off (U3, ≤ 1e-9 relative), not bit for bit. In still water
  both NL FK pressure models take the same code path, so they stay
  bit-identical. Against nonlinear hydrostatics the decay must agree to
  1e-6 m: far above round-off growth in a damped decay, far below any
  physical effect.
- **C2:** the free sphere at ω = 0.8 rad/s follows the wave (RAO ≈ 1.03), so
  its wetted surface barely changes and NL FK stays within 0.1% of linear. A
  1% amplitude check alone could not tell a working NL FK from a linear one.
  Two cases with larger relative motion and a check on the nonlinear
  difference were added.

**Reference results (stage 2a).** IEA sphere (R = 5 m, CG 2 m below the
centre, mass 261.8 t), `sphere.h5`, heave only, constant-coefficient
radiation, 20 s ramp, 140 s, harmonic fit over the last 10 periods. Steady
heave amplitude in metres, phase relative to the incident wave at the origin:

| Case | $a$ (m) | Linear | Calm-water NL | NL FK, Airy + Wheeler | NL FK, long-wave |
|---|---|---|---|---|---|
| Free, ω = 0.8 | 0.1 | 0.1034 (−0.5°) | 0.1034 (−0.5°) | 0.1034 (−0.5°) | 0.1034 (−0.5°) |
| Free, ω = 0.8 | 1.0 | 1.0343 (−0.5°) | 1.0523 (−0.6°) | 1.0340 (−0.5°) | 1.0341 (−0.5°) |
| Free, ω = 0.8 | 2.5 | 2.5856 (−0.5°) | 3.0033 (−1.6°) | 2.5845 (−0.5°) | 2.5861 (−0.5°) |
| PTO 6.1e5 N s/m, ω = 0.8 | 1.0 | 0.6940 (−42.2°) | 0.6963 (−42.4°) | 0.6911 (−42.4°) | 0.6913 (−42.4°) |
| PTO 6.1e5 N s/m, ω = 0.8 | 2.5 | 1.7351 (−42.2°) | 1.7735 (−43.6°) | 1.6874 (−43.3°) | 1.6910 (−43.6°) |
| Free, ω = 1.25 | 0.5 | 0.7344 (−10.2°) | 0.7468 (−10.8°) | 0.7364 (−10.4°) | 0.7354 (−10.3°) |
| Free, ω = 1.25 | 1.0 | 1.4688 (−10.2°) | 1.5877 (−13.5°) | 1.4869 (−11.0°) | 1.4786 (−10.9°) |

Runaway case (H = 10 m, T = 8 s, RIRF radiation, 80 s): calm-water NL
hydrostatics heaves from −31 m to +34 m; NL FK stays within −5.4 m to +5.2 m
for both pressure models (C4 limit ±10 m); linear gives ±5.3 m.

Reproduce with `nlfk_reference.py selftest | regular | runaway | gr-compare`.

**C++ results (stage 2b).** All unit and comparison criteria pass. Run
`ctest -C Release -R "nonlinear_froude_krylov|compare_nlfk"`.

| # | Result |
|---|---|
| C1 | NL FK − linear ≤ +0.46% amplitude, ≤ 0.19° phase. The +0.2% offset at small amplitude is the mesh waterplane stiffness against the H5 value |
| C2 | All checked models within 0.6% and 0.3° of the reference. PTO, $a$ = 2.5 m: NL FK − linear −2.73% (reference −2.75%) Airy, −2.47% (reference −2.54%) long-wave |
| C3 | RMS vs Giorgi & Ringwood ≤ 0.74% of range; vs the Python Airy + Wheeler reference ≤ 0.19% |
| C4 | NL FK −5.41 to +5.04 m (Airy), −5.30 to +5.18 m (long-wave). Calm-water NL reaches −31.3 m and trips the divergence guard at 24.7 s; the guard then zeroes all hydro forces, so later heave is not reported |
| C5 | Float and plate within 0.39% and 0.49° of all-linear |
| C6 | Long-wave − Airy: +0.03% at $kd$ = 0.18, −0.40% at 0.80, −3.8% at 1.15 |
| C7 | Airy and long-wave bit-identical; NL FK vs calm-water NL identical to the 1e-9 m print resolution |

## 10. External verification

| Case | Data | Pass criterion |
|---|---|---|
| `sphere_rao_sweep_nl` | IEA OES Task 10 sphere, steepness 0.01, free and optimum-PTO; MARIN aNySIM and NREL WEC-Sim nonlinear results (Wendt et al. 2019, `RAO_dat.xlsx`) | at each period, the SEA-Stack RAO lies within the range of the two nonlinear codes widened by 5% of their mean |
| Sphere 5 m decay multicode | Existing weakly nonlinear decay result | same criterion as the existing NL hydrostatics case |
| `sphere_decay_aau_experiment` | Kramer et al. 2021 heave decay, 3 drop heights, CC BY | first two periods within 2% of the experiment mean; first-trough amplitude within the spread of the published LPF2-type participants |
| `ccpwsi_bt3_float` | CCP-WSI Blind Test 3, Geometry 1 (Ransley et al. 2020) | **after a licence check**; heave and surge peaks within the published NL FK participant's error band |
| WEC-Sim ellipsoid | Public WEC-Sim inputs (H = 4 m, T = 6 s, PTO 1.2e6 N s/m) | demo, plus comparison with the Python reference (C2 criterion); no WEC-Sim output exists to compare with |

The experimental criteria are provisional until the data has been inspected.
Any change goes through review and is recorded here.

**Status (stage 2b).**

- **`sphere_rao_sweep_nl`: pass, all 20 points.** Test
  `tests/verification/sphere_rao_sweep_nl`, data
  `data/verification/sphere_rao_sweep_nl` (normalised from the existing
  `RAO_dat.xlsx`, sheets `noPTO_S=0.01` and `PTO_S=0.01`). Wendt et al.
  define steepness as $S = H/(gT^2)$ (their eq. 5), so $H = 0.01\,gT^2$ and
  $a = H/2$, from 0.44 m at 3 s to 5.94 m at 11 s. With optimum PTO, both
  nonlinear codes fall well below linear at long periods. NL FK follows them:
  at 11 s the RAO is 0.577 (MARIN 0.575, NREL 0.574, SEA-Stack linear 0.700).
  11 of the 20 points lie between MARIN and NREL; the largest distance
  outside their range is 0.95% (optimum PTO, 7 s), against the 5% allowed.
- **Sphere 5 m decay:** covered by C7. In still water NL FK reproduces
  nonlinear hydrostatics to 1e-9 m, so the existing NL hydrostatics decay
  cases (`iea_sphere/decay_nl_5m` regression baseline) apply unchanged.
- **`sphere_decay_aau_experiment`:** not done. The dataset is not in the
  repository. In still water NL FK equals nonlinear hydrostatics, so this case
  tests the hydrostatics and radiation, not the FK term.
- **`ccp_wsi_bt3_float`:** not done; waiting on the licence check.
- **WEC-Sim ellipsoid:** not done here; planned as a demo with the user
  documentation.

## 11. Validity and limits

- **Incident wave:** linear, undisturbed by the body. Clipping uses the
  incident elevation, not the local diffracted or radiated surface.
- **Radiation and diffraction:** linear, about the mean position.
- **Not modelled:** steep-wave kinematics, breaking, slamming, green water,
  and wetting of surfaces the mesh does not include.
- **Mesh:** must be closed and fine enough near the waterline to resolve
  $\eta$ along the hull. Guidance will follow from U8.
- **Long-wave model:** overpredicts wave pressure at depth by $e^{kd}$.
- **Position:** bodies must sit at their BEM positions (section 4).

## 12. References

Page numbers and author lists were written from memory. Check them against the
sources before release. Checked so far: Giorgi & Ringwood (2017) and the DOI
of Giorgi & Ringwood (2018).

- Cummins, W. E. (1962). *The impulse response function and ship motions.*
  Schiffstechnik 9, 101-109.
- Faltinsen, O. M. (1990). *Sea Loads on Ships and Offshore Structures.*
  Cambridge University Press.
- Gilloteaux, J.-C. (2007). *Mouvements de grande amplitude d'un corps flottant
  en fluide parfait. Application à la récupération de l'énergie des vagues.*
  PhD thesis, École Centrale de Nantes.
- Giorgi, G., Ringwood, J. V. (2017). Computationally efficient nonlinear
  Froude-Krylov force calculations for heaving axisymmetric wave energy point
  absorbers. *J. Ocean Eng. Mar. Energy* 3, 21-33.
  doi:10.1007/s40722-016-0066-2
- Giorgi, G., Ringwood, J. V. (2018). Analytical representation of nonlinear
  Froude-Krylov forces for 3-DoF point absorbing wave energy devices.
  *Ocean Engineering* 164, 749-759. doi:10.1016/j.oceaneng.2018.07.020
  (adds Wheeler stretching; the waterline is still flat at the axis).
- Kramer, M. B., et al. (2021). Highly accurate
  experimental heave decay tests with a floating sphere: a public benchmark
  dataset for model validation of fluid-structure interaction. *Energies*
  14(2), 269.
- Lawson, M., Yu, Y.-H., Nelessen, A., Ruehl, K., Michelen, C. (2014).
  Implementing nonlinear buoyancy and excitation forces in the WEC-Sim wave
  energy converter modeling tool. *Proc. OMAE 2014*, OMAE2014-24445.
- Penalba, M., Mérigaud, A., Gilloteaux, J.-C., Ringwood, J. V. (2017).
  Influence of nonlinear Froude-Krylov forces on the performance of two wave
  energy points absorbers. *J. Ocean Eng. Mar. Energy* 3, 209-220.
- Ransley, E., et al. (2020). Blind comparative study of focused wave
  interactions with floating structures (CCP-WSI Blind Test Series 3).
  *Int. J. Offshore Polar Eng.* 30(1). Dataset doi:10.24382/5025.
- Wendt, F., et al. (2019). Ocean Energy Systems wave energy modelling task:
  modelling, verification and validation of wave energy converters.
  *J. Mar. Sci. Eng.* 7(11), 379.
- Wheeler, J. D. (1970). Method for calculating forces produced by irregular
  waves. *J. Petroleum Technology* 22(3), 359-367.
- WEC-Sim issue #886 (frames and static pressure datum) and issue #1485 /
  PR #1527 (wave force doubled in v6.1.2):
  <https://github.com/WEC-Sim/WEC-Sim/issues/886>,
  <https://github.com/WEC-Sim/WEC-Sim/issues/1485>.
