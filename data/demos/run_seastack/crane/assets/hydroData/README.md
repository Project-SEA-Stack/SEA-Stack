# Crane demo hydrodynamic data

## `crane_vessel.h5`

BEMIO-format first-order coefficients for the 50 m Wigley support vessel at
40 m water depth, solved with Capytaine including diffraction from four fixed
pile columns near the offshore platform.

Generate:

```bash
cd data/demos/run_seastack/crane/assets
python generate_vessel.py
python generate_crane_platform.py
python run_bem.py              # full solve
python run_bem.py --quick -j 4   # smoke test (writes a local quick H5; not committed)
```
