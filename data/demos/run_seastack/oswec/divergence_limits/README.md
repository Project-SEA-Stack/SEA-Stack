# OSWEC — configurable divergence limits

Short free-flap (no PTO) case that shows the optional hydro YAML
`divergence:` block. The flap is free about its revolute hinge; a
moderately energetic irregular sea can drive large pitch. With the
**default** 90° roll/pitch guard, that can abort the run as “diverged”
even though the motion is physically allowed. This case raises the
threshold to 180°.

This is a **blow-up detector** on hydrodynamic force attachment, not a
mechanical joint stop. It only applies when a hydro YAML is used (no
hydro → no `HydroSystem` → no guard).

## What to look at

In `oswec_divergence_limits.hydro.yaml`:

```yaml
divergence:
  enabled: true
  max_roll_pitch: 180.0   # deg; default is 90
```

Try the same case with the `divergence:` block removed (or
`max_roll_pitch: 90`) if you want to see the default guard trip on a
large pitch excursion. Or set `enabled: false` to skip magnitude checks
entirely (NaN/Inf still abort).

## Run

```bash
# From a source build (Windows example)
build\bin\Release\run_seastack.exe data\demos\run_seastack\oswec\divergence_limits --nogui
```

Packaged release:

```bash
bin\run_seastack.exe demos\oswec\divergence_limits --nogui
```

## Related

- Defaults and units: [TECHNICAL_OVERVIEW.md](../../../../TECHNICAL_OVERVIEW.md) (error handling / divergence)
- Sibling free-flap sea: [`../irregular_waves/`](../irregular_waves/)
