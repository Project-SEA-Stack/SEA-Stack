# Ship-to-ship transfer (YAML case)

Drive an HMMWV from one 50 m Wigley hull to another across a flexible linkspan
bridge while both vessels respond to beam seas.

```
        z
        |            HMMWV drives along +y
        |                    ==>
   deck +3.5 ____________       ___________       ____________
             |          |=======|  bridge  |======|          |
             |  hull 1  |       (8 m gap)          |  hull 2  |
    SWL 0 ---|  body1   |--------------------------|  body2   |---
             |  y=-10   |                          |  y=+10   |
   keel -3.5 |__________|                          |__________|
                                  --> +y
```

## Running

```bash
# One-off: generate the meshes and the two-body BEM database
cd data/demos/run_seastack/ship2ship/assets
python generate_meshes.py
python run_bem.py                     # ~200 frequencies x 24 headings, a few minutes

# Then, from the build tree (Windows)
build\bin\Release\run_seastack.exe data\demos\run_seastack\ship2ship\transfer --nogui

# Or on Linux/macOS
./build/bin/run_seastack data/demos/run_seastack/ship2ship/transfer --nogui
```

The `--nogui` flag runs a 30 s headless simulation. Omit it for the interactive
GUI with keyboard vehicle control (`W`/`S` throttle, `A`/`D` steer).

## Model

| Item | Value |
|------|-------|
| Hull | Wigley, L = 50 m, B = 12 m, T = 3.5 m, freeboard 3.5 m |
| Displacement | 933.3 m³, 956.7 t per hull (floats at z = 0) |
| Hull separation | 20 m centre to centre, 8 m clear gap |
| Water depth | 50 m (BEM and wave field agree) |
| Sea state | regular beam wave, H = 2.0 m, T = 8 s, heading 90°, 20 s ramp |
| Bridge | light aluminium ladder frame: 2 girders 0.15 × 0.30 m RHS + 9 cross-beams, 12 m span |
| Bridge deck | 9 thin translucent planks, 4.5 m wide, 2.43 t total |
| Vehicle | HMMWV, rigid tires, interactive driver |
| Time step | 1e-3 s, EULER_IMPLICIT_LINEARIZED + SPARSE_LU |

### Hydrodynamics

One **two-body coupled** BEM database (`ship2ship.h5`, 12 DOF), not two
single-body files, so hull-to-hull radiation and diffraction interaction across
the gap is represented.

### Gap resonance, and why T = 8 s

The two-body database shows a sharp resonance at **omega = 1.12–1.19 rad/s**
(T = 5.3–5.6 s). This is the piston mode of the channel between the hulls. The
demo runs at T = 8 s, clear of the peak. **If you re-tune the sea state, stay
away from 5.3–5.6 s** unless you also add gap damping.

At T = 8 s the wavelength is about 100 m, so the bridge sees about **1.2 m of
relative heave**.

### Bridge: ladder frame with Euler beams

A light aluminium portable linkspan: two edge girders at x = ±2 m joined by
transverse cross-beams, with thin translucent planks on top. Members are
false-coloured by **vertical bending moment** `Mz` (jet colormap, ±70 kN·m).

The girders are sized so a midspan HMMWV adds about **80 mm** of sag (L/150).

### Boundary conditions

Four pinned bearings with releases so differential hull motion does not lock
into the frame:

| Bearing | Hull | Constrained |
|---------|------|-------------|
| stbd girder A | `body1` | x, y, z |
| stbd girder B | `body1` | y, z |
| port girder A | `body2` | z |
| port girder B | `body2` | z |

## Validation

The simulation prints a static load-path check at t = 2 s:

```
Bridge weight:            30.976 kN
Sum of bearing reactions: 32.214 kN (4.0%)
Mid-span sag, measured:   -57.4 mm
```

Treat these as the case's expected static validation targets.

## Files

| File | Description |
|------|-------------|
| `ship2ship_transfer.setup.yaml` | Entry point for run_seastack |
| `ship2ship_transfer.model.yaml` | Chrono MBS (two hulls) |
| `ship2ship_transfer.simulation.yaml` | Solver and time settings |
| `ship2ship_transfer.hydro.yaml` | Hydrodynamics (two-body coupled, beam sea) |
| `ship2ship_transfer.vehicle.yaml` | HMMWV on deck with terrain patches |
| `ship2ship_transfer.structure.yaml` | FEA linkspan bridge |
