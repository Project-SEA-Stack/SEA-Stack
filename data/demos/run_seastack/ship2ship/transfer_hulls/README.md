# Ship to ship: hulls only

Two 50 m Wigley hulls respond to a regular beam sea, with no bridge. This
checks the two-body coupled hydrodynamics on its own; `../bridge` adds the
FEA linkspan. 30 s of simulation takes about 2 s of wall time.

```powershell
.\build\bin\Release\run_seastack.exe data\demos\run_seastack\ship2ship\transfer_hulls --nogui
```

The model is described in [`../README.md`](../README.md). Time step: 1 ms.

The deck collision boxes in `ship2ship_transfer_hulls.model.yaml` are only for
a later vehicle case; nothing touches them here.

## Expected result

Peak relative heave |z(`body2`) − z(`body1`)| after the 20 s ramp: 1.21 m.
`test_ship2ship_bridge` checks it within 5 %.

## Files

| File | Description |
|------|-------------|
| `ship2ship_transfer_hulls.setup.yaml` | Entry point for run_seastack |
| `ship2ship_transfer_hulls.model.yaml` | Chrono MBS (two hulls) |
| `ship2ship_transfer_hulls.simulation.yaml` | Solver and time settings |
| `ship2ship_transfer_hulls.hydro.yaml` | Hydrodynamics (two-body coupled) |
