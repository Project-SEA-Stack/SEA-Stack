"""Capytaine BEM for the ROV umbilical demo: one 50 m Wigley hull at 40 m depth.

Water depth appears in three places and they must all agree: this BEM solve, the
demo's wave field, and MoorDyn's ``WtrDpth`` in ``mooring/umbilical.txt``. The
demo asserts that at start-up. 40 m is shallow for real ROV work; it is chosen so
the vessel, the umbilical and the seabed crawler all fit in one camera shot.

Conventions (see scripts/bem/README.md):
  * Moments are taken about the hull's centre of gravity, so
    ``rotation_center == center_of_mass == (0, 0, COG_Z)``. SEA-Stack applies the
    coefficients at the Chrono body frame, which sits at the CoG, and treats the
    H5 ``cg`` as the body's equilibrium position.
  * A waterplane lid is passed to Capytaine to suppress irregular frequencies.
  * The HDF5 is written by scripts/bem/bemio_from_dataset.py, which applies the
    Capytaine e^-iwt -> BEMIO e^+iwt imaginary-sign flip.

Dependencies:
    pip install capytaine numpy scipy h5py xarray

Usage:
    python run_bem.py              # full solve
    python run_bem.py --quick      # coarse solve for a smoke test
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import xarray as xr

try:
    import capytaine as cpt
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "Capytaine is required: pip install capytaine\n"
        "See https://capytaine.org/stable/"
    ) from exc

_BEM_TOOLS = Path(__file__).resolve().parents[5] / "scripts" / "bem"
if str(_BEM_TOOLS) not in sys.path:
    sys.path.insert(0, str(_BEM_TOOLS))

from bemio_from_dataset import (  # noqa: E402
    BodyHydrostatics,
    build_hydro_from_dataset,
    excitation_irf,
    radiation_irf,
    write_bemio_h5,
)

# ---------------------------------------------------------------------------
# Configuration -- must match mooring/umbilical.txt and the model YAML
# ---------------------------------------------------------------------------
LENGTH = 50.0
BEAM = 12.0
DRAFT = 3.5
COG_Z = -1.75          # m, approx -T/2; also the Chrono body origin

RHO = 1025.0           # kg/m^3
G = 9.81               # m/s^2
WATER_DEPTH = 40.0     # m -- MUST equal MoorDyn WtrDpth and the hydro YAML sea state

NAME = "vessel"
HEADINGS_DEG = np.arange(0.0, 360.0, 15.0)

RA_T_END, RA_N_DT = 60.0, 1201
EX_T_END, EX_N_DT = 60.0, 2401
N_DW = 1001


def load_body(mesh_path: Path) -> cpt.FloatingBody:
    if not mesh_path.is_file():
        raise FileNotFoundError(
            f"Mesh not found: {mesh_path}\nRun generate_meshes.py first."
        )
    mesh = cpt.load_mesh(str(mesh_path), file_format="nemoh")
    wetted = mesh.immersed_part()
    lid = wetted.generate_lid()
    centre = (0.0, 0.0, COG_Z)
    body = cpt.FloatingBody(
        mesh=wetted,
        lid_mesh=lid,
        dofs=cpt.rigid_body_dofs(rotation_center=centre),
        center_of_mass=np.array(centre),
        name=NAME,
    )
    n_lid = body.lid_mesh.nb_faces if body.lid_mesh is not None else 0
    print(f"  {NAME}: {body.mesh.nb_faces} wet panels, {n_lid} lid panels")
    return body


def body_hydrostatics(body: cpt.FloatingBody) -> BodyHydrostatics:
    vol_analytic = 4.0 * LENGTH * BEAM * DRAFT / 9.0
    awp_analytic = 2.0 * LENGTH * BEAM / 3.0

    hs = body.compute_hydrostatics(rho=RHO, g=G)
    stiffness = np.asarray(body.compute_hydrostatic_stiffness(rho=RHO, g=G), dtype=float)
    cb = np.asarray(hs["center_of_buoyancy"], dtype=float).ravel()[:3]
    vol = float(hs["disp_volume"])
    cg = np.asarray(body.center_of_mass, dtype=float).ravel()[:3]
    awp = stiffness[2, 2] / (RHO * G)

    print(f"  V={vol:.2f} m^3 ({100 * (vol / vol_analytic - 1):+.2f} % vs analytic "
          f"{vol_analytic:.2f})")
    print(f"  Awp={awp:.2f} m^2 ({100 * (awp / awp_analytic - 1):+.2f} % vs analytic "
          f"{awp_analytic:.2f})")
    print(f"  cb_z={cb[2]:+.3f} m   cg_z={cg[2]:+.3f} m")

    return BodyHydrostatics(
        name=NAME,
        disp_volume=vol,
        center_of_gravity=cg,
        center_of_buoyancy=cb,
        hydrostatic_stiffness=stiffness,
    )


def solve_bem(body: cpt.FloatingBody, omega: np.ndarray, n_jobs: int) -> xr.Dataset:
    print(f"\n  {body.mesh.nb_faces} panels, {len(body.dofs)} DOFs")
    print(f"  omega: {omega[0]:.3f} .. {omega[-1]:.3f} rad/s ({omega.size} points)")
    print(f"  headings: {len(HEADINGS_DEG)} from {HEADINGS_DEG[0]:.0f} to "
          f"{HEADINGS_DEG[-1]:.0f} deg")
    print(f"  water depth: {WATER_DEPTH} m (finite)")

    if n_jobs != 1:
        try:
            import joblib  # noqa: F401
        except ImportError:
            print("  NOTE: joblib not installed -- solving sequentially "
                  "(pip install joblib for a large speed-up).")
            n_jobs = 1

    test_matrix = xr.Dataset(coords={
        "omega": omega,
        "radiating_dof": list(body.dofs),
        "wave_direction": np.deg2rad(HEADINGS_DEG),
        "water_depth": [WATER_DEPTH],
        "rho": [RHO],
    })

    t0 = time.time()
    dataset = cpt.BEMSolver().fill_dataset(test_matrix, body, n_jobs=n_jobs)
    print(f"  BEM solve completed in {time.time() - t0:.1f} s")
    return dataset


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true", help="Coarse solve for a smoke test")
    ap.add_argument("--n-freq", type=int, default=None, help="Frequency points (default 200)")
    ap.add_argument("--freq-min", type=float, default=0.05, help="Min omega [rad/s]")
    ap.add_argument("--freq-max", type=float, default=3.0, help="Max omega [rad/s]")
    ap.add_argument("--heading-step", type=float, default=None,
                    help="Heading increment [deg] (default 15, quick 45)")
    ap.add_argument("--jobs", "-j", type=int, default=-1,
                    help="Parallel workers (-1 = all cores, 1 = sequential)")
    ap.add_argument("--output", type=Path, default=None)
    ap.add_argument("--meshes-dir", type=Path, default=None)
    args = ap.parse_args()

    global HEADINGS_DEG
    step = args.heading_step if args.heading_step is not None else (45.0 if args.quick else 15.0)
    HEADINGS_DEG = np.arange(0.0, 360.0, step)

    script_dir = Path(__file__).resolve().parent
    mesh_path = (args.meshes_dir or script_dir / "meshes") / "vessel.nemoh"
    output = args.output or script_dir / "hydroData" / "rov_vessel.h5"
    output.parent.mkdir(parents=True, exist_ok=True)

    n_freq = args.n_freq if args.n_freq is not None else (30 if args.quick else 200)
    omega = np.linspace(args.freq_min, args.freq_max, n_freq)

    print("=" * 68)
    print(f"ROV support vessel BEM (single body, {WATER_DEPTH} m depth)")
    print("=" * 68)

    print("\nLoading mesh...")
    body = load_body(mesh_path)

    print("\nHydrostatics...")
    hydrostatics = body_hydrostatics(body)

    print("\nRunning BEM...")
    dataset = solve_bem(body, omega, args.jobs)

    print("\nPost-processing...")
    hydro = build_hydro_from_dataset(dataset, [hydrostatics])
    print("  radiation IRF...")
    radiation_irf(hydro, t_end=RA_T_END, n_dt=RA_N_DT, n_dw=N_DW)
    print("  excitation IRF...")
    excitation_irf(hydro, t_end=EX_T_END, n_dt=EX_N_DT, n_dw=N_DW)

    write_bemio_h5(hydro, output)
    print(f"\nWrote {output}")
    print(f"  Nf={hydro['Nf']}  Nh={hydro['Nh']}  depth={WATER_DEPTH} m")
    print("\nSanity check with:")
    print(f"  python ../../../../../scripts/bem/check_bemio_h5.py {output.name} "
          f"--waterplane-area {2.0 * LENGTH * BEAM / 3.0:.4f} "
          f"--disp-vol {4.0 * LENGTH * BEAM * DRAFT / 9.0:.4f}")


if __name__ == "__main__":
    main()
