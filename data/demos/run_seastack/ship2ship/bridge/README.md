# Ship-to-ship linkspan bridge (FEA preview)

A light aluminium linkspan spans the 8 m gap between the two hulls in a
regular beam sea (H = 2 m, T = 8 s). The bridge is a Chrono::FEA Euler-beam
ladder frame with rigid deck planks, resting on four pinned bearings. It is
defined entirely in `ship2ship_bridge.structure.yaml`
(schema: [`docs/usage/STRUCTURE_YAML.md`](../../../../../docs/usage/STRUCTURE_YAML.md)).
Members are coloured by vertical bending moment Mz (±70 kN·m).

```
        z
        |
   deck +3.5 ____________     bridge (12 m)     ____________
             |          |=======================|          |
             |  body1   |       (8 m gap)       |  body2   |
    SWL 0 ---|  y=-10   |-----------------------|  y=+10   |---
             |__________|                       |__________|
                                  --> +y
```

```powershell
.\build\bin\Release\run_seastack.exe data\demos\run_seastack\ship2ship\bridge
.\build\bin\Release\run_seastack.exe data\demos\run_seastack\ship2ship\bridge_static --nogui
```

`bridge` runs 30 s in about 6 s of wall time headless; `bridge_static` runs
15 s in about 3.5 s. Both write `outputs/structure_diagnostics.csv` (bearing
reactions and member moment extremes every 10 steps) and the deformed bridge to
the HDF5.

## Bridge

| Item | Value |
|------|-------|
| Girders | 2, at x = ±2 m, y = −6 … +6 m, z = 3.34 m; 150 × 300 × 8 mm RHS (depth vertical), 8 elements |
| Cross-beams | 9, one per girder node station; 100 × 150 × 6 mm RHS, 2 elements |
| Deck planks | 9 rigid bodies, 270 kg each, 4.5 × 1.5 × 0.06 m, clamped to the cross-beam centres, no collision |
| Material | aluminium 5083: E = 70 GPa, G = 26.3 GPa, ρ = 2700 kg/m³ |
| Torsion constant | Bredt thin-wall J: 6.338e-5 m⁴ (girder), 9.238e-6 m⁴ (cross-beam) |
| Mass | 450 kg girders + 278 kg cross-beams + 2430 kg planks = 3158 kg (30.98 kN) |

The torsion constants are set explicitly. The default J = Iyy + Izz would be
72 % too stiff for the girder and 46 % for the cross-beam, and twist matters in
oblique seas (below).

## Bearings

`constrain` is in world axes at the start. The constrained directions then
turn with the hull.

| Bearing | Node | Hull | Constrained |
|---------|------|------|-------------|
| `bearing_stbd_a` | `girder_a.first` | `body1` | x, y, z |
| `bearing_stbd_b` | `girder_b.first` | `body1` | y, z |
| `bearing_port_a` | `girder_a.last` | `body2` | z |
| `bearing_port_b` | `girder_b.last` | `body2` | z |

All rotations are free, and the `body2` end slides freely in x and y, so the
gap can open and close as the hulls roll. That is 7 restraints for 6
rigid-body degrees of freedom. The extra one is the fourth vertical support,
as for a bridge resting on four pads. Hull roll moves both bearings on a hull
together and does not load the frame. Differential hull pitch does: it twists
the ladder frame and moves load between diagonal bearings.

The bearings are two-way constraints: they would hold the bridge down where
real pads would lift off. Check that the vertical reactions stay compressive.

## Damping

The first vertical mode was measured at 2.23–2.25 Hz (ω₁ = 14.0 rad/s) by
dropping the bridge in still water. A simply supported hand estimate gives
2.27 Hz. Chrono's Euler beam damps at ζ₁ ≈ β·ω₁, so `rayleigh_beta: 0.003`
gives 4.2 %. Implicit Euler adds ω·dt/2 and some energy leaves through the
hulls:

| dt | Measured ζ₁ |
|----|-------------|
| 1 ms | 5.4 % |
| 5 ms (this case) | 8.3 % |

The ported case had Chrono's hidden default β = 0.01 s (ζ₁ = 18 %). The
bridge's 2.25 Hz mode is far above the 0.125 Hz wave, so its wave response is
quasi-static. The damping mainly sets how fast the bridge settles at start-up.

## Verification

**Statics (`bridge_static`, still water, t = 10 s):**

| Quantity | Hand calculation | Run |
|----------|------------------|-----|
| Bridge weight | 30.976 kN | 30.976 kN |
| Sum of bearing reactions along gravity | 30.976 kN | 31.020 kN (+0.14 %, check limit 2 %) |
| Girder mid-span sag (relative to the end chord) | 54.54 mm | 54.50 mm |

The sag hand calculation treats each girder as simply supported over 12 m. It
carries its own weight (5wL⁴/384EI = 8.68 mm) plus half of each interior
cross-beam and plank as point loads at the 7 interior node stations
(45.86 mm). The old 4 % load-path error and 57.4 mm sag were sampled at 2 s,
while the bridge was still ringing.

**Beam sea (`bridge`, t ≥ 20 s):**

| Quantity | Value |
|----------|-------|
| Peak relative heave, bridge vs hulls only | 1.2066 m vs 1.2102 m (−0.3 %, from dt) |
| Peak girder \|Mz\| | 22.5 kN·m (about 41 MPa) |
| Bearing vertical reaction on each hull | −8.28 … −7.24 kN (static −7.7 kN): always compressive |
| Girder torsion | < 0.1 kN·m |

**Time step.** Against 1 ms, 5 ms changes relative heave by 0.3 %, and peak
girder moment, bearing reactions and the load-path check by less than 0.1 %.
10 ms changes relative heave by 0.65 %. The case uses 5 ms. A vehicle case
needs 1 ms for tyre contact.

**Oblique sea (heading 45°, H = 2 m, not a shipped case).** Differential hull
pitch reaches 2.3°, and the frame twists:
- girder torsion 5.3 kN·m (8.4 kN·m with the default J);
- bearing vertical reactions −11.1 … −4.6 kN, still compressive.

Pads would start to lift at roughly H ≈ 4.8 m in that heading (linear
extrapolation).

## Not modelled

- No wave, wind or current load on the bridge itself; it is loaded only
  through its bearings and its weight.
- No bearing lift-off, friction or pad stiffness.
- Euler beams (no shear deformation); joints between girders and cross-beams
  are rigid.

## Files

| File | Description |
|------|-------------|
| `ship2ship_bridge.setup.yaml` | Entry point |
| `ship2ship_bridge.model.yaml` | Two hulls and station keeping (shared with `bridge_static`) |
| `ship2ship_bridge.structure.yaml` | FEA bridge (shared with `bridge_static`) |
| `ship2ship_bridge.hydro.yaml` | Two-body BEM, regular beam sea |
| `ship2ship_bridge.simulation.yaml` | dt = 5 ms, implicit Euler, SPARSE_LU |
| `../bridge_static/*.setup.yaml`, `*.hydro.yaml`, `*.simulation.yaml` | Still water, 15 s, load-path check at 10 s |
