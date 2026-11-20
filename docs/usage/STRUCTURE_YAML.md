# Structure YAML (`*.structure.yaml`, preview)

A structure file describes a Chrono::FEA Euler-beam frame: its beams, the
mates that clamp beam nodes to model bodies, and optional rigid attachments.
Name it from a setup YAML with `structure_file`; `StructureSubsystem` builds it
at run time, so a new topology needs no C++ changes.

The parser is strict. An unknown key, a missing required key, a wrong-sized
vector, a non-physical value or a reference to an undefined material, section,
beam or node set stops the run with the file name and key path in the error.
Body names are checked when the structure is attached, because bodies come
from the model YAML.

## Conventions

- z up, free surface at z = 0, SI units throughout (m, kg, s, N, Pa).
- `start`, `end`, `offset` and mate positions are in **world** coordinates.
- A beam's `y_direction` sets the section local y axis (Chrono `Ydir`). The
  section `depth` is measured along it, and its strong second moment
  (depth³) goes to **Izz**. With `y_direction: [0, 0, 1]` a horizontal beam
  resists vertical load with Izz, and the matching bending moment is **Mz**
  (`ELEM_BEAM_MZ`), not My.
- Mate frames are world-aligned and sit at the node, so `constrain` reads as
  `[x, y, z, rot_x, rot_y, rot_z]` in **world** axes at the start. The frames
  then move with the body, so on a rolling hull a `z`-only bearing restrains
  the hull's tilted vertical (and its reaction has a horizontal world
  component).
- Mates are two-way constraints: a bearing can pull as well as push. Check
  that reactions keep the sign a real support would allow.
- Mate reactions (diagnostics CSV, load-path check) are the force and torque
  the mate applies to the support body, in world axes.
- Element section forces (HDF5, colour plot) are in the **element local**
  frame: Fx axial, Fy/Fz shear, Mx torsion, My/Mz bending.

## Using it from a setup YAML

```yaml
model_file: cantilever.model.yaml
simulation_file: cantilever.simulation.yaml
structure_file: cantilever.structure.yaml   # relative to the setup file
output_directory: outputs

checks:                                     # optional
  - type: structure_static_load_path        # alias: bridge_static_load_path
    time_s: 4.0                             # s, default 2.0
    tolerance_percent: 2.0                  # %, default 10.0
```

`structure_static_load_path` sums the mate reactions along gravity at `time_s`
and compares them with the structure's own weight (beams plus rigid
attachments). Bodies the structure is mated to are not included: their weight
cancels in the sum. A failed check sets exit code 1. A check that cannot be
judged also fails: no structure, no mates, gravity off, or a run that ends
before `time_s`. Pick `time_s` after the structure has settled; ringing beam
inertia shows up as error (see [Damping](#damping)).

The solver in the simulation YAML must handle FEA stiffness: use
`SPARSE_LU`, `SPARSE_QR`, `PARDISO_MKL` or `MINRES`. Variational-inequality
solvers (`PSOR`, `APGD`, `BB`, …) are rejected at start-up.

## Minimal example

```yaml
structure:
  automatic_gravity: true
  materials:
    - { name: alu, density: 2700.0, youngs_modulus: 7.0e10, shear_modulus: 2.63e10 }
  sections:
    - { name: rhs, type: BOX, material: alu, width: 0.30, depth: 0.30, wall: 0.010,
        rayleigh_beta: 5.0e-3, torsion_constant: 2.4389e-4 }
  beams:
    - name: connector
      section: rhs
      start: [0.0, 0.0, 3.5]
      end:   [6.0, 0.0, 3.5]
      elements: 6
      y_direction: [0.0, 0.0, 1.0]
  mates:
    - { name: root, node: connector.first, body: ground,
        constrain: [true, true, true, true, true, true] }
  diagnostics:
    file: structure_diagnostics.csv
    every_n_steps: 10
```

## Keys

**Bold** keys are required. `materials`, `sections` and `beams` are required
and must each have at least one entry; every other block is optional. Names
must be unique within their list.

### Top level (`structure:`)

| Key | Default | Notes |
|-----|---------|-------|
| `automatic_gravity` | `true` | Beam self-weight from the simulation gravity |
| **`materials`**, **`sections`**, **`beams`** | — | See below |
| `cross_beams`, `node_sets`, `mates`, `rigid_attachments`, `visualization`, `diagnostics` | — | See below |

### `materials`

| Key | Unit | Notes |
|-----|------|-------|
| **`name`** | — | Referenced by sections |
| **`density`** | kg/m³ | > 0 |
| **`youngs_modulus`** | Pa | > 0 |
| **`shear_modulus`** | Pa | > 0; typically E / 2(1+ν) |

### `sections`

Only rectangular hollow sections (`type: BOX`) are supported.

| Key | Unit | Default | Notes |
|-----|------|---------|-------|
| **`name`** | — | | Referenced by beams |
| `type` | — | `BOX` | Only `BOX` |
| **`material`** | — | | A material name |
| **`width`** | m | | Extent across local z; > 0 |
| **`depth`** | m | | Extent across local y (`y_direction`); > 0 |
| **`wall`** | m | | > 0 and less than half of width and depth |
| `rayleigh_beta` | s | 0 | Stiffness-proportional damping; see [Damping](#damping) |
| `rayleigh_alpha` | 1/s | 0 | Mass-proportional damping |
| `torsion_constant` | m⁴ | Iyy + Izz | Saint-Venant J; see [Limitations](#limitations) |

### `beams`

| Key | Unit | Default | Notes |
|-----|------|---------|-------|
| **`name`** | — | | Must not contain `.` |
| **`section`** | — | | A section name |
| **`start`**, **`end`** | m | | World coordinates |
| **`elements`** | — | | Number of `ChElementBeamEuler`, ≥ 1; the beam has `elements + 1` nodes |
| `y_direction` | — | `[0, 0, 1]` | Local y axis; must not be parallel to the beam |

### Node references

Mates and node sets point at beam nodes as `beam.first`, `beam.last`,
`beam.mid` or `beam.N` (0-based, `0 … elements`). `mid` is node
`(elements + 1) / 2` rounded down: the centre node for an even element count,
the node just past the centre for an odd one.
An index past the end of the beam is rejected.

### `node_sets`

| Key | Notes |
|-----|-------|
| **`name`** | Referenced by `rigid_attachments` |
| **`nodes`** | Non-empty list of node references |

### `cross_beams`

Builds a transverse ladder between two parallel rail beams: one cross-beam per
rail node station. The cross-beams share the rail nodes, and each mid node is
added to `node_set`.

| Key | Default | Notes |
|-----|---------|-------|
| `name_prefix` | `cross` | |
| **`section`** | | A section name |
| **`between`** | | Two different beam names |
| `elements` | 2 | Per cross-beam; ≥ 2 so there is a mid node |
| `y_direction` | `[0, 0, 1]` | |
| `node_set` | `cross_centres` | Name given to the mid nodes |

### `mates`

| Key | Notes |
|-----|-------|
| **`name`** | Used in the diagnostics CSV columns |
| **`node`** | Node reference |
| **`body`** | A body from the model YAML (for example `ground` or `body1`) |
| **`constrain`** | Six booleans `[x, y, z, rot_x, rot_y, rot_z]`, world axes |

### `rigid_attachments`

One rigid box per node in `node_set`, clamped in all six DOF (deck planks).
Inertia is that of a uniform box of the given mass and extents.

| Key | Unit | Default | Notes |
|-----|------|---------|-------|
| `name_prefix` | — | `plank` | |
| **`node_set`** | — | | From `node_sets` or `cross_beams` |
| **`mass`** | kg | | Per body; > 0 |
| **`box`** | m | | Extents; each > 0 |
| `offset` | m | `[0, 0, 0]` | Body centre relative to its node, world axes |
| `collide` | — | `true` | `false` gives a visual-only body with no contact |
| `collision_family` | — | 7 | 0–15 |
| `opacity` | — | 0.35 | 0–1 |
| `color` | — | `[0.75, 0.78, 0.82]` | RGB, 0–1 |
| `material` | — | steel-deck SMC values | `friction`, `restitution`, `Young_modulus`, `Poisson_ratio`, `normal_stiffness`, `normal_damping`, `tangential_stiffness`, `tangential_damping` |

### `visualization`

| Key | Default | Notes |
|-----|---------|-------|
| `data_type` | `ELEM_BEAM_MZ` | `ELEM_BEAM_MY`, `ELEM_BEAM_MZ`, `ELEM_BEAM_TX`, `NODE_SPEED_NORM`, `NONE` (case-insensitive) |
| `colormap` | `JET` | `JET`, `INFERNO`, `VIRIDIS`, `PLASMA` |
| `range` | `[-7e4, 7e4]` | Colour range in field units; min < max |
| `beam_resolution` | 12 | Colour bands along each element; ≥ 1 |
| `beam_resolution_section` | 8 | Facets around the section; ≥ 1 |

The SEA-Stack viewer (the default) draws the beams with their section outline.
It uses `data_type` only, to pick the field shown at start-up. It colours by
the field's magnitude (for example \|Mz\|) on the Turbo colour map, with an
auto, locked or fixed range set under Scene → FEA beams in the Inspector.
`NODE_SPEED_NORM` shows plain beams. `colormap`, `range` and the resolutions
apply to the Chrono viewer (`--viewer=chrono`).

### `diagnostics`

| Key | Default | Notes |
|-----|---------|-------|
| `file` | empty (off) | Relative paths resolve to the run output directory |
| `every_n_steps` | 10 | Sample every N steps; ≥ 1 |

CSV columns: `time_s`; per mate `<mate>_Fx_N … <mate>_Mz_Nm`; per beam
`<beam>_max_abs_Mx/My/Mz_Nm` (sampled at η ∈ {−1, 0, 1}) and
`<beam>_rel_dx/dy/dz_m` (change in the last-node-minus-first-node vector
since the start, world axes, so rigid-body translation drops out). The CSV runs at a
higher rate than the HDF5 output, so it resolves structural peaks.

## HDF5 output

| Group | Datasets |
|-------|----------|
| `/inputs/structure/<beam>` | `undeformed_node_positions` [nodes, 3], `connectivity` [elements, 2]; attributes `section_width`, `section_depth` |
| `/results/structure/<beam>` | `node_positions` [frames, nodes, 3] (world, m), `node_orientations` [frames, nodes, 4] (wxyz), `element_forces` [frames, elements, 6] (Fx Fy Fz Mx My Mz at the element mid-point, element local frame, N and N·m) |

Frames share `/results/time/time`.

## Damping

An omitted `rayleigh_beta` means **no** stiffness damping. (Chrono's own
section default is β = 0.01 s; SEA-Stack always overrides it with the YAML
value.)

With Chrono's `ChElementBeamEuler`, the measured first-mode damping ratio is
ζ₁ ≈ β·ω₁, twice the textbook β·ω/2. It does not depend on the time step. The
implicit Euler integrator adds about ω·dt/2 on top. To target a damping ratio
ζ, choose β ≈ ζ / ω₁.

## Verification

The unit tests check the structure against closed-form beam theory
(`test_structure_fea_analytic`, static linear solve unless noted):

| Case | Check | Result | Test tolerance |
|------|-------|--------|----------------|
| Cantilever, tip load | Tip sag PL³/3EI, tip rotation, root reaction and moment signs | exact | 0.5 % |
| Cantilever, self-weight | Sag wL⁴/8EI | 0.13 % | 1 % |
| Simply supported, pin + roller | Mid-span sag 5wL⁴/384EI, end reactions wL/2 | 0.3 % | 1 % |
| Torsion, tip torque | Twist TL/GJ, default and Bredt J | exact | 0.5 % |
| Rotated beams, strong and weak axis | Sag direction and size with the right I | exact | 0.5 % |
| Mesh convergence, n = 1…16 | Self-weight sag error falls as 1/(3n²) | 0.13 % at n = 16 | 0.5 %, monotonic |
| First natural frequency (tip mass) | 9.361 Hz vs 9.376 Hz analytic | 0.2 % | 2 % |
| Damping | Undamped amplitude kept; damped ζ vs β·ω₁ + ω₁·dt/2 | | > 85 % kept; 10 % |
| Load-path check | Reactions vs beam + 500 kg attachment weight; gravity off is invalid | < 0.01 % | 0.01 % |

`test_structure_config` covers the parser (defaults and about 40 rejection
cases). `test_structure_run` runs the cantilever demo end to end and checks
the HDF5 datasets, the check pass and fail paths, and the solver guard.

## Limitations

- **Euler–Bernoulli beams only.** No shear deformation, so stubby members
  (length under about 10× depth) are too stiff.
- **BOX sections only**, with one linear-elastic isotropic material each.
- **Default torsion constant.** J = Iyy + Izz when `torsion_constant` is
  unset. For a closed thin-wall box this is about 33 % stiffer than the Bredt
  value 4A_m²t / P (for example 2.4389e-4 m⁴ for a 300 × 300 × 10 mm box). Set
  `torsion_constant` when torsion matters.
- **Beams do not share nodes.** Two beams meeting at a point have separate
  nodes, and mates connect nodes to bodies only. Portal frames and trusses
  cannot be built yet. The only built-in shared-node topology is the
  `cross_beams` ladder.
- **No hydrodynamic loads on beams.** Beams carry gravity, mate loads and
  attachment loads only. Wave and current loads act on the hydro bodies.
- Hydrodynamic bodies must be named `body1` … `bodyN` (the hydro coupler
  parses the number).
- Needs an FEA-capable solver (`SPARSE_LU`, `SPARSE_QR`, `PARDISO_MKL` or
  `MINRES`).
- Campaign runs pass `structure_file` through, but not the setup `checks:`
  block.

## Worked demos

- [`data/demos/run_seastack/ship2ship/bridge/`](../../data/demos/run_seastack/ship2ship/bridge/):
  linkspan between two floating hulls. It is the example for `cross_beams`,
  `rigid_attachments`, explicit `torsion_constant`, and partial-`constrain`
  bearings (pins with sliding releases). `bridge_static` checks it against a
  hand calculation.
- [`data/demos/run_seastack/trimaran/fea_arms/`](../../data/demos/run_seastack/trimaran/):
  steel cross-arms clamped between three floating hulls in irregular waves.
- [`data/demos/run_seastack/fea/cantilever/`](../../data/demos/run_seastack/fea/cantilever/):
  clamped cantilever with a tip mass, compared with beam theory.
- [`data/demos/run_seastack/crane/beam_check/`](../../data/demos/run_seastack/crane/):
  flexible crane jib on a floating vessel, with the payload on the hoist rope.
