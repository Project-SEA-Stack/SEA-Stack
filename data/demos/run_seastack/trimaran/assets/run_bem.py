"""Run Capytaine BEM analysis for the trimaran WEC and export BEMIO H5.

This script:
  1. Loads the center-hull and outrigger Nemoh meshes from generate_meshes.py
  2. Positions three FloatingBody instances (center, port outrigger, stbd outrigger)
  3. Solves radiation and diffraction BEM problems (18 DOF total)
  4. Computes impulse response functions (radiation RIRF and excitation IRF)
  5. Writes the result in BEMIO HDF5 format for SEA-Stack

The multi-body BEM pattern follows the 5SA demo
(data/demos/run_seastack/5sa/assets/run_bem.py).

Dependencies:
    pip install capytaine numpy h5py scipy

Usage:
    python run_bem.py                 # full BEM solve
    python run_bem.py --quick         # reduced resolution for testing
    python run_bem.py --help
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np
import h5py

try:
    import capytaine as cpt
except ImportError:
    raise ImportError(
        "Capytaine is required: pip install capytaine\n"
        "See https://capytaine.github.io/stable/user_manual/installation.html"
    )


# ---------------------------------------------------------------------------
# Layout  (must match generate_meshes.py defaults and C++ demo)
# ---------------------------------------------------------------------------
N_BODIES = 3
OUTRIGGER_Y_OFFSET = 15.0   # m, center-to-center lateral spacing

# Center hull
CENTER_LENGTH = 50.0
CENTER_DRAFT = 3.5
CENTER_COG_Z = -1.75

# Outrigger
OUTRIGGER_LENGTH = 25.0
OUTRIGGER_DRAFT = 1.5
OUTRIGGER_COG_Z = -0.75

RHO = 1025.0
G = 9.81
WATER_DEPTH = 250.0


# ---------------------------------------------------------------------------
# Body loading
# ---------------------------------------------------------------------------

def load_bodies(meshes_dir: Path) -> list[cpt.FloatingBody]:
    """Load Nemoh meshes and configure the three floating bodies."""
    bodies: list[cpt.FloatingBody] = []

    # Body 1: center hull at origin
    center_mesh_path = meshes_dir / "center.nemoh"
    if not center_mesh_path.exists():
        raise FileNotFoundError(
            f"Mesh not found: {center_mesh_path}\nRun generate_meshes.py first."
        )
    mesh = cpt.load_mesh(str(center_mesh_path), file_format="nemoh")
    body = cpt.FloatingBody(mesh=mesh)
    body.center_of_mass = np.array([0.0, 0.0, CENTER_COG_Z])
    body.rotation_center = np.array([0.0, 0.0, 0.0])
    body.add_all_rigid_body_dofs()
    body.keep_immersed_part()
    body.name = "center"
    bodies.append(body)
    print(f"  Loaded {body.name}: {body.mesh.nb_faces} wet panels, "
          f"center_of_mass={body.center_of_mass}")

    # Body 2 & 3: outriggers at +/- Y offset
    outrigger_mesh_path = meshes_dir / "outrigger.nemoh"
    if not outrigger_mesh_path.exists():
        raise FileNotFoundError(
            f"Mesh not found: {outrigger_mesh_path}\nRun generate_meshes.py first."
        )

    for side, y_offset, name in [("port", -OUTRIGGER_Y_OFFSET, "port_outrigger"),
                                  ("stbd", +OUTRIGGER_Y_OFFSET, "stbd_outrigger")]:
        mesh = cpt.load_mesh(str(outrigger_mesh_path), file_format="nemoh")
        body = cpt.FloatingBody(mesh=mesh)
        body.mesh.translate([0.0, y_offset, 0.0])
        body.center_of_mass = np.array([0.0, y_offset, OUTRIGGER_COG_Z])
        body.rotation_center = np.array([0.0, y_offset, 0.0])
        body.add_all_rigid_body_dofs()
        body.keep_immersed_part()
        body.name = name
        bodies.append(body)
        print(f"  Loaded {body.name}: {body.mesh.nb_faces} wet panels, "
              f"center_of_mass={body.center_of_mass}")

    return bodies


def compute_body_hydrostatics(bodies: list[cpt.FloatingBody]) -> list[dict]:
    """Compute hydrostatic properties for each body from its mesh."""
    results = []
    for body in bodies:
        hs = body.compute_hydrostatics(rho=RHO, g=G)
        C = np.array(hs["hydrostatic_stiffness"])
        dv = float(hs["disp_volume"])
        cb = np.array(hs["center_of_buoyancy"]).flatten()[:3]
        results.append({"stiffness": C, "disp_vol": dv, "cb": cb})
        print(f"  {body.name}: V_disp={dv:.1f} m^3, cb_z={cb[2]:.3f} m, "
              f"C33={C[2,2]:.0f} N/m, C55={C[4,4]:.0f} N-m/rad")
    return results


# ---------------------------------------------------------------------------
# BEM solver
# ---------------------------------------------------------------------------

def solve_bem(
    bodies: list[cpt.FloatingBody],
    freq_range: np.ndarray,
    wave_directions: np.ndarray,
    n_jobs: int = 1,
) -> "xarray.Dataset":
    """Assemble multi-body and solve BEM problems."""
    import xarray as xr

    all_bodies = bodies[0]
    for b in bodies[1:]:
        all_bodies = all_bodies + b
    all_bodies.keep_immersed_part()

    n_problems = len(freq_range) * (len(all_bodies.dofs) + len(wave_directions))
    print(f"\n  Combined body: {all_bodies.mesh.nb_faces} panels, "
          f"{len(all_bodies.dofs)} DOFs")
    print(f"  Frequency range: {freq_range[0]:.3f} .. {freq_range[-1]:.3f} rad/s "
          f"({len(freq_range)} points)")
    print(f"  Wave directions: {np.degrees(wave_directions)} deg")
    print(f"  ~{n_problems} BEM problems to solve")

    if n_jobs != 1:
        try:
            import joblib  # noqa: F401
        except ImportError:
            print("  WARNING: joblib not installed -- falling back to sequential solve.")
            n_jobs = 1

    if n_jobs == 1:
        print("\n  Solving BEM problems sequentially...")
    elif n_jobs == -1:
        import os as _os
        print(f"\n  Solving BEM problems in parallel (all {_os.cpu_count()} cores)...")
    else:
        print(f"\n  Solving BEM problems in parallel ({n_jobs} workers)...")

    test_matrix = xr.Dataset({
        "omega": freq_range,
        "wave_direction": wave_directions,
        "radiating_dof": list(all_bodies.dofs),
        "water_depth": [WATER_DEPTH],
        "rho": [RHO],
    })

    solver = cpt.BEMSolver()
    t0 = time.time()
    dataset = solver.fill_dataset(test_matrix, all_bodies, n_jobs=n_jobs)
    elapsed = time.time() - t0
    print(f"  BEM solve completed in {elapsed:.1f} s")

    return dataset


# ---------------------------------------------------------------------------
# Impulse response functions  (same as 5SA / Wigley run_bem.py)
# ---------------------------------------------------------------------------

def compute_radiation_irf(
    omega: np.ndarray,
    B: np.ndarray,
    t_end: float = 60.0,
    dt: float = 0.05,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute radiation IRF K(t) from damping B(omega).

    K(t) = (2/pi) * integral_0^inf B(omega) * cos(omega*t) d_omega
    """
    t = np.arange(0, t_end + dt / 2, dt)
    n_t = len(t)
    trailing = B.shape[1:]
    K = np.zeros(trailing + (n_t,))
    d_omega = np.gradient(omega)
    extra = (slice(None),) + (np.newaxis,) * len(trailing)
    for k in range(n_t):
        w = (np.cos(omega * t[k]) * d_omega)[extra]
        K[..., k] = (2.0 / math.pi) * np.sum(B * w, axis=0)
    return t, K


def compute_excitation_irf(
    omega: np.ndarray,
    F_re: np.ndarray,
    F_im: np.ndarray,
    t_end: float = 60.0,
    dt: float = 0.05,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute excitation IRF f(t) from complex excitation force.

    f(t) = (1/pi) * integral_0^inf [Re(F)*cos(wt) - Im(F)*sin(wt)] d_omega
    """
    t = np.arange(-t_end, t_end + dt / 2, dt)
    n_t = len(t)
    trailing = F_re.shape[1:]
    f = np.zeros(trailing + (n_t,))
    d_omega = np.gradient(omega)
    extra = (slice(None),) + (np.newaxis,) * len(trailing)
    for k in range(n_t):
        c = (np.cos(omega * t[k]) * d_omega)[extra]
        s = (np.sin(omega * t[k]) * d_omega)[extra]
        f[..., k] = (1.0 / math.pi) * np.sum(F_re * c - F_im * s, axis=0)
    return t, f


# ---------------------------------------------------------------------------
# BEMIO H5 writer  (adapted from 5SA for 3 bodies with different geometry)
# ---------------------------------------------------------------------------

# Per-body CoG and reference data for BEMIO export.
# Order MUST match: (1) load_bodies() append order, (2) Capytaine combined DOF blocks,
# (3) C++ trimaran_hulls.h body1/body2/body3 and hydro_bodies vector.
# Convention: z up; **body2 = port at y = -OUTRIGGER_Y_OFFSET**, **body3 = stbd at y = +OFFSET**.
_BODY_INFO = [
    {"cog": [0.0, 0.0, CENTER_COG_Z], "label": "center"},
    {"cog": [0.0, -OUTRIGGER_Y_OFFSET, OUTRIGGER_COG_Z], "label": "port_minus_y"},
    {"cog": [0.0, +OUTRIGGER_Y_OFFSET, OUTRIGGER_COG_Z], "label": "stbd_plus_y"},
]


def write_bemio_h5(
    output_path: Path,
    dataset: "xarray.Dataset",
    n_bodies: int,
    body_hydrostatics: list[dict] | None = None,
    irf_t_end: float = 60.0,
    irf_dt: float = 0.05,
) -> None:
    """Convert Capytaine xarray Dataset to BEMIO H5 format for SEA-Stack."""
    rho = float(dataset.attrs.get("rho", RHO))
    g = G
    omega = dataset["omega"].values
    n_freq = len(omega)

    dof_names = list(dataset["radiating_dof"].values)
    n_dof_total = len(dof_names)
    dof_per_body = n_dof_total // n_bodies

    wave_dirs = dataset["wave_direction"].values
    n_dir = len(wave_dirs)

    added_mass_full = dataset["added_mass"].values
    rad_damping_full = dataset["radiation_damping"].values

    Fk = dataset["Froude_Krylov_force"].values
    Fd = dataset["diffraction_force"].values
    F_exc = Fk + Fd

    if "hydrostatic_stiffness" in dataset:
        C_full = dataset["hydrostatic_stiffness"].values
    else:
        C_full = np.zeros((n_dof_total, n_dof_total))

    A_inf = added_mass_full[-1, :, :]

    print("  Computing radiation IRF...")
    rirf_t, rirf_K = compute_radiation_irf(omega, rad_damping_full,
                                           t_end=irf_t_end, dt=irf_dt)
    n_rirf_t = len(rirf_t)

    print("  Computing excitation IRF...")
    F_exc_re = np.real(F_exc)
    F_exc_im = np.imag(F_exc)
    exc_irf_t, exc_irf_f = compute_excitation_irf(omega, F_exc_re, F_exc_im,
                                                   t_end=irf_t_end, dt=irf_dt)
    n_exc_t = len(exc_irf_t)

    def _col(arr):
        return np.asarray(arr).reshape(-1, 1)

    def _scalar(v):
        return np.array([[float(v)]])

    print(f"  Writing BEMIO H5 to {output_path}")
    with h5py.File(str(output_path), "w") as f:
        sp = f.create_group("simulation_parameters")
        sp.create_dataset("rho", data=_scalar(rho))
        sp.create_dataset("g", data=_scalar(g))
        if np.isinf(WATER_DEPTH):
            sp.create_dataset("water_depth", data="infinite")
        else:
            sp.create_dataset("water_depth", data=_scalar(WATER_DEPTH))
        sp.create_dataset("w", data=_col(omega))
        sp.create_dataset("wave_dir", data=_col(np.degrees(wave_dirs)))
        # Machine-readable indexing for SEA-Stack trimaran demos (ignored by standard BEMIO readers).
        sp.attrs["trimaran_body_order"] = (
            "body1=center@y=0; "
            f"body2=port@y={-OUTRIGGER_Y_OFFSET:g}; "
            f"body3=stbd@y={+OUTRIGGER_Y_OFFSET:g} "
            "(world z up, lateral: -Y port, +Y stbd)"
        )

        for b in range(n_bodies):
            body_name = f"body{b + 1}"
            bg = f.create_group(body_name)

            i0 = b * dof_per_body
            i1 = i0 + dof_per_body

            props = bg.create_group("properties")
            cg = _BODY_INFO[b]["cog"]

            if body_hydrostatics and b < len(body_hydrostatics):
                bhs = body_hydrostatics[b]
                disp_vol = bhs["disp_vol"]
                cb = bhs["cb"]
                C_body = bhs["stiffness"] / (rho * g)
            else:
                disp_vol = 0.0
                cb = np.array(cg)
                C_body = C_full[i0:i1, i0:i1] / (rho * g)

            props.create_dataset("disp_vol", data=_scalar(disp_vol))
            props.create_dataset("cg", data=_col(cg))
            props.create_dataset("cb", data=_col(cb))

            hc = bg.create_group("hydro_coeffs")
            hc.create_dataset("linear_restoring_stiffness", data=C_body)

            A_body = A_inf[i0:i1, :] / rho
            am = hc.create_group("added_mass")
            am.create_dataset("inf_freq", data=A_body)

            K_body = rirf_K[i0:i1, :, :] / rho
            rd = hc.create_group("radiation_damping")
            irf_group = rd.create_group("impulse_response_fun")
            irf_group.create_dataset("K", data=K_body)
            irf_group.create_dataset("t", data=_col(rirf_t))

            exc = hc.create_group("excitation")

            F_body = F_exc[:, :, i0:i1]
            mag = np.abs(F_body) / (rho * g)
            phase = np.angle(F_body)

            exc.create_dataset("mag", data=np.transpose(mag, (2, 1, 0)))
            exc.create_dataset("phase", data=np.transpose(phase, (2, 1, 0)))

            f_body = exc_irf_f[:, i0:i1, :] / (rho * g)
            exc_irf_group = exc.create_group("impulse_response_fun")
            exc_irf_group.create_dataset("f", data=np.transpose(f_body, (1, 0, 2)))
            exc_irf_group.create_dataset("t", data=_col(exc_irf_t))

    print(f"  BEMIO H5 written: {output_path}")
    print(f"    Bodies: {n_bodies}, DOF/body: {dof_per_body}")
    print(f"    Frequencies: {n_freq}, Wave directions: {n_dir}")
    print(f"    RIRF time steps: {n_rirf_t}, Excitation IRF time steps: {n_exc_t}")


# ---------------------------------------------------------------------------
# NetCDF checkpoint helpers
# ---------------------------------------------------------------------------

def _save_nc(dataset, nc_path: Path) -> None:
    from capytaine.io.xarray import separate_complex_values
    nc_path.parent.mkdir(parents=True, exist_ok=True)
    ds = separate_complex_values(dataset)
    for coord in ("radiating_dof", "influenced_dof"):
        if coord in ds.coords and hasattr(ds[coord].dtype, "categories"):
            ds[coord] = ds[coord].astype(str)
    ds.to_netcdf(
        str(nc_path),
        encoding={"radiating_dof": {"dtype": "U"}, "influenced_dof": {"dtype": "U"}},
    )
    print(f"  Capytaine NetCDF saved: {nc_path}")


def _load_nc(nc_path: Path):
    import xarray as xr
    from capytaine.io.xarray import merge_complex_values
    print(f"  Loading dataset from {nc_path} ...")
    return merge_complex_values(xr.open_dataset(str(nc_path)))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="Run Capytaine BEM for trimaran WEC")
    ap.add_argument("--quick", action="store_true",
                    help="Reduced resolution for quick testing")
    ap.add_argument("--n-freq", type=int, default=None,
                    help="Number of frequency points (default: 200, quick: 30)")
    ap.add_argument("--freq-min", type=float, default=0.05,
                    help="Minimum angular frequency [rad/s]")
    ap.add_argument("--freq-max", type=float, default=3.0,
                    help="Maximum angular frequency [rad/s]")
    ap.add_argument("--irf-t-end", type=float, default=60.0,
                    help="IRF time range [s]")
    ap.add_argument("--irf-dt", type=float, default=0.05,
                    help="IRF time step [s]")
    ap.add_argument("--output", type=Path, default=None)
    ap.add_argument("--meshes-dir", type=Path, default=None)
    ap.add_argument("--jobs", "-j", type=int, default=-1,
                    help="Parallel workers (-1 = all cores, 1 = sequential)")
    ap.add_argument("--from-nc", type=Path, default=None,
                    help="Skip BEM solve; reload from saved .nc file")
    args = ap.parse_args()

    script_dir = Path(__file__).resolve().parent
    meshes_dir = args.meshes_dir or script_dir / "meshes"
    output = args.output or script_dir / "hydroData" / "trimaran.h5"
    output.parent.mkdir(parents=True, exist_ok=True)
    nc_path = output.parent / "trimaran_capytaine.nc"

    print("=" * 60)
    print("Trimaran WEC BEM Analysis")
    print("=" * 60)

    print("\nLoading meshes...")
    bodies = load_bodies(meshes_dir)

    print("\nComputing hydrostatics from meshes...")
    body_hydrostatics = compute_body_hydrostatics(bodies)

    if args.from_nc:
        dataset = _load_nc(args.from_nc)
    else:
        n_freq = args.n_freq
        if n_freq is None:
            n_freq = 30 if args.quick else 200

        freq_range = np.linspace(args.freq_min, args.freq_max, n_freq)
        wave_directions = np.array([0.0])

        print("\nRunning BEM...")
        dataset = solve_bem(bodies, freq_range, wave_directions, n_jobs=args.jobs)

        print("\nSaving Capytaine NetCDF (checkpoint)...")
        try:
            _save_nc(dataset, nc_path)
        except Exception as e:
            print(f"  WARNING: NetCDF export failed: {e}")

    print("\nPost-processing and writing H5...")
    write_bemio_h5(output, dataset, N_BODIES,
                   body_hydrostatics=body_hydrostatics,
                   irf_t_end=args.irf_t_end, irf_dt=args.irf_dt)

    print("\nDone.")


if __name__ == "__main__":
    main()
