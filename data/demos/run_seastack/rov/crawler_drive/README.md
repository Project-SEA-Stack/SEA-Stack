# ROV umbilical (YAML case)

A 50 m Wigley support vessel works in head seas while a tracked seabed crawler
drives away from it 40 m below, the two joined by a lazy-wave umbilical with a
buoyancy-module arch.

```
      fairlead (vessel stern A-frame, z = +5 m)
         \                 _.-'''-._      hog bend, lifted by the buoyant section
    SWL 0 \   upper 40 m _-'         '-_
        ---\-._        _-'               '-._   lower 28 m
               '--''''                        '-.__ crawler ==> +x
                                                    ~12 m lying on the bottom
  seabed -40 m  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  1.5 m relief
```

The arch is the point of the demo. Without it, every heave cycle of the ship
would snatch the vehicle off the bottom. With it, vessel heave is absorbed by
the arch changing shape and the crawler keeps driving.

## Running

```bash
# One-off: generate the mesh and the 40 m-depth BEM database
cd data/demos/run_seastack/rov/assets
python generate_meshes.py
python run_bem.py

# Then, from the build tree (Windows)
build\bin\Release\run_seastack.exe data\demos\run_seastack\rov\crawler_drive --nogui

# Or on Linux/macOS
./build/bin/run_seastack data/demos/run_seastack/rov/crawler_drive --nogui
```

The headless run takes about 3–4 minutes for 45 s of simulated time. Neither
mode is real-time: the M113's track contact forces a 5e-4 s step.

The crawler is driven by a path follower, not by the keyboard, so the run is
repeatable. In GUI mode, mouse drag and scroll set the viewpoint.

## Model

| Item | Value |
|------|-------|
| Vessel | Wigley, L = 50 m, B = 12 m, T = 3.5 m, 956.7 t, CoG z = −1.75 m |
| Water depth | 40 m to the seabed crests (BEM, wave field and MoorDyn agree) |
| Seabed | RigidTerrain height map, 100 × 64 m, 1.5 m of relief |
| Sea state | regular head wave, H = 2.0 m, T = 8 s, heading 0° (along +x) |
| Crawler | M113 single-pin tracks, 11.3 t, path follower at 1.0 m/s |
| Umbilical | 80 m total: 40 m plain + 12 m buoyant + 28 m plain, MoorDyn v2 |
| Time step | 5e-4 s Chrono, 5e-5 s MoorDyn, EULER_IMPLICIT_LINEARIZED + SPARSE_LU |

### Water depth consistency

40 m has to agree across the BEM solve (finite depth), the wave field, and
MoorDyn's `WtrDpth`. The setup YAML can include a `water_depth_consistency`
check that verifies this at startup:

```yaml
checks:
  - type: water_depth_consistency
    tolerance_m: 0.01
```

### Hydrodynamics

Single-body `rov_vessel.h5`, solved at 40 m finite depth. Radiation uses
state-space at this small time step to avoid convolution cost.

### Umbilical: MoorDyn body to body

The umbilical connects the hull (a hydro body) to the crawler chassis (a
vehicle body) using MoorDyn v2. The lazy-wave configuration with a buoyancy
arch decouples vessel heave from the crawler.

## Validation

Start-up reports the umbilical's initial tension:

```
--- Umbilical initial tension ---
  Expected fairlead tension (order of): 0.994 kN
  MoorDyn fairlead tension:             1.445 kN
```

After the 45 s run:

```
  Peak fairlead tension:     2.497 kN
  Peak tension at crawler:   2.196 kN
  Crawler travelled to x =   81.0 m
  Peak lateral path error:   0.547 m
```

## Files

| File | Description |
|------|-------------|
| `crawler_drive.setup.yaml` | Entry point for run_seastack |
| `crawler_drive.model.yaml` | Chrono MBS (vessel only; crawler from vehicle YAML) |
| `crawler_drive.simulation.yaml` | Solver and time settings |
| `crawler_drive.hydro.yaml` | Hydrodynamics (single body, head sea, MoorDyn umbilical) |
| `crawler_drive.vehicle.yaml` | M113 tracked crawler + seabed terrain |
