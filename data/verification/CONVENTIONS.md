# SEA-Stack Verification Data Conventions

This document defines the canonical conventions for all normalized verification
data in SEA-Stack. Every file under `data/verification/*/normalized/` must
conform to these rules.

## Units

All quantities use SI units:

| Quantity        | Unit   | Notes                                  |
|-----------------|--------|----------------------------------------|
| Time            | s      | Starting at 0                          |
| Length / heave  | m      | Displacement relative to equilibrium   |
| Angle / pitch   | rad    | Right-hand rule about Y axis           |
| Force           | N      |                                        |
| RAO (linear)    | m/m    | Dimensionless                          |
| RAO (angular)   | rad/m  | **Not** deg/m                          |
| Frequency       | rad/s  |                                        |

## Reference Frame

- Global, right-handed, Z-up.
- Origin at mean water level (MWL), centred on the device.
- Displacements are **relative to equilibrium**, not absolute CG position.
  For example, sphere decay heave is "metres above equilibrium", not the
  global Z coordinate of the CG.

## Sign Conventions

| DOF    | Positive direction                              |
|--------|-------------------------------------------------|
| Heave  | Upward (+Z)                                     |
| Surge  | Forward (+X, wave propagation direction)        |
| Pitch  | Bow-up (right-hand rotation about +Y)           |
| Tension| Positive = tensile                              |

## Phase Convention (RAO)

Phase is measured relative to `cos(omega * t)`, positive = lead.

## File Format

Every normalized `.txt` file must use the standard `#`-prefixed metadata header:

```
# SEA-Stack Verification Data
# format: timeseries | rao
# source: <tool_identifier>
# model: <case_name>
# quantity: <what_is_stored>
# units: <comma-separated SI units>
# columns: <space-separated column names with units>
```

Optional additional `#` comment lines may follow.

Numeric data is space-delimited, one row per time step (or frequency point).

## Manifest Requirements

Every `manifest.json` must include for each normalized dataset:
- `coordinate_frame` (e.g. "global, Z-up")
- `sign_convention` (e.g. "positive up")
- `units` dict mapping column names to SI units
