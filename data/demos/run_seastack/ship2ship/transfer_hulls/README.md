# Ship-to-ship transfer: hulls only (YAML case)

Hydro-only precursor to the full ship-to-ship transfer demo. Two 50 m Wigley
hulls respond to beam seas without the linkspan bridge, vehicle, or FEA
structure subsystem.

This is a lightweight smoke test (~1 s wall time for 30 s of simulation) to
verify the two-body hydrodynamic coupling is working before running the full
transfer case with vehicle and structure.

## Running

```bash
# From the build tree (Windows)
build\bin\Release\run_seastack.exe data\demos\run_seastack\ship2ship\transfer_hulls --nogui

# Or on Linux/macOS
./build/bin/run_seastack data/demos/run_seastack/ship2ship/transfer_hulls --nogui
```

## Model

| Item | Value |
|------|-------|
| Hull | Wigley, L = 50 m, B = 12 m, T = 3.5 m, freeboard 3.5 m |
| Displacement | 933.3 m³, 956.7 t per hull |
| Hull separation | 20 m centre to centre, 8 m clear gap |
| Water depth | 50 m |
| Sea state | regular beam wave, H = 2.0 m, T = 8 s, heading 90° |
| Time step | 1e-3 s |

## Validation

Expected peak relative heave (port hull z − stbd hull z): ~1.23 m.

## Files

| File | Description |
|------|-------------|
| `ship2ship_transfer_hulls.setup.yaml` | Entry point for run_seastack |
| `ship2ship_transfer_hulls.model.yaml` | Chrono MBS (two hulls) |
| `ship2ship_transfer_hulls.simulation.yaml` | Solver and time settings |
| `ship2ship_transfer_hulls.hydro.yaml` | Hydrodynamics (two-body coupled) |
