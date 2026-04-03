"""Run Capytaine BEM analysis for the 5SA with multiple wave headings.

This script re-uses the mesh loading, BEM solver, and H5 writer from
run_bem.py but solves for a range of wave directions so that the
excitation transfer functions in the output H5 file are heading-
dependent.  The resulting dataset is required by the ``spreading/``
and ``bimodal/`` demo cases that use LinearDirectionalWaveField.

Default heading range: 0-350 deg in 10-deg steps (36 headings).

Dependencies:
    pip install capytaine numpy h5py scipy
    pip install joblib          # optional, for parallel BEM solve

Usage:
    python run_bem_directional.py                # full solve (36 headings)
    python run_bem_directional.py --quick        # reduced resolution
    python run_bem_directional.py --dir-step 30  # every 30 deg (12 headings)
    python run_bem_directional.py --from-nc hydroData/5sa_directional_capytaine.nc
    python run_bem_directional.py --help
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from run_bem import (
    N_SEGMENTS,
    load_bodies,
    compute_body_hydrostatics,
    solve_bem,
    write_bemio_h5,
)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Run Capytaine BEM for 5SA with multiple wave headings",
    )
    ap.add_argument(
        "--quick", action="store_true",
        help="Reduced resolution for quick testing (fewer frequencies, larger heading step)",
    )
    ap.add_argument("--n-freq", type=int, default=None,
                    help="Number of frequency points (default: 400, quick: 30)")
    ap.add_argument("--freq-min", type=float, default=0.05,
                    help="Minimum angular frequency [rad/s]")
    ap.add_argument("--freq-max", type=float, default=5.0,
                    help="Maximum angular frequency [rad/s]")
    ap.add_argument("--dir-min", type=float, default=0.0,
                    help="Minimum wave direction [deg]")
    ap.add_argument("--dir-max", type=float, default=350.0,
                    help="Maximum wave direction [deg]")
    ap.add_argument("--dir-step", type=float, default=None,
                    help="Wave direction step [deg] (default: 10, quick: 30)")
    ap.add_argument("--irf-t-end", type=float, default=60.0,
                    help="IRF time range [s]")
    ap.add_argument("--irf-dt", type=float, default=0.05,
                    help="IRF time step [s]")
    ap.add_argument("--output", type=Path, default=None,
                    help="Output H5 file path")
    ap.add_argument("--meshes-dir", type=Path, default=None,
                    help="Directory containing .nemoh meshes")
    ap.add_argument("--jobs", "-j", type=int, default=-1,
                    help="Parallel workers for BEM solve (-1 = all cores)")
    ap.add_argument("--from-nc", type=Path, default=None,
                    help="Skip BEM solve; re-run post-processing from saved .nc file")
    args = ap.parse_args()

    script_dir = Path(__file__).resolve().parent
    meshes_dir = args.meshes_dir or script_dir / "meshes"
    output = args.output or script_dir / "hydroData" / "5sa_directional.h5"
    output.parent.mkdir(parents=True, exist_ok=True)
    nc_path = output.parent / "5sa_directional_capytaine.nc"

    print("=" * 60)
    print("5SA Multi-Directional BEM Analysis")
    print("=" * 60)

    print("\nLoading meshes...")
    bodies = load_bodies(meshes_dir)

    print("\nComputing hydrostatics from meshes...")
    body_hydrostatics = compute_body_hydrostatics(bodies)

    if args.from_nc:
        from run_bem import _load_nc
        dataset = _load_nc(args.from_nc)
    else:
        n_freq = args.n_freq
        if n_freq is None:
            n_freq = 30 if args.quick else 400

        dir_step = args.dir_step
        if dir_step is None:
            dir_step = 30.0 if args.quick else 10.0

        freq_range = np.linspace(args.freq_min, args.freq_max, n_freq)
        dir_deg = np.arange(args.dir_min, args.dir_max + dir_step / 2, dir_step)
        wave_directions = np.radians(dir_deg)

        n_problems = len(freq_range) * (len(bodies[0].dofs) * N_SEGMENTS + len(wave_directions))
        print(f"\n  Headings: {dir_deg} ({len(dir_deg)} directions)")
        print(f"  Estimated BEM problems: ~{n_problems}")
        print(f"  This will take significantly longer than the single-heading run.\n")

        print("Running BEM...")
        dataset = solve_bem(bodies, freq_range, wave_directions, n_jobs=args.jobs)

        print("\nSaving Capytaine NetCDF (checkpoint)...")
        try:
            from run_bem import _save_nc
            _save_nc(dataset, nc_path)
        except Exception as e:
            print(f"  WARNING: NetCDF export failed: {e}")

    print("\nPost-processing and writing H5...")
    write_bemio_h5(output, dataset, N_SEGMENTS,
                   body_hydrostatics=body_hydrostatics,
                   irf_t_end=args.irf_t_end, irf_dt=args.irf_dt)

    print(f"\nDirectional H5 written to: {output}")
    print("Update the hydro YAML files to point to this file if needed:")
    print(f"  h5_file: ../assets/hydroData/{output.name}")
    print("\nDone.")


if __name__ == "__main__":
    main()
