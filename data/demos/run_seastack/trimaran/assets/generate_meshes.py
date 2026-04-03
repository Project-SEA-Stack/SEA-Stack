"""Generate parametric Wigley hull meshes for a trimaran WEC demo.

Produces two hull shapes at different scales:
  - Center hull:  L=50 m, B=12 m, T=3.5 m, F=3.5 m
  - Outrigger:    L=25 m, B=4 m,  T=1.5 m, F=1.5 m

Each hull is written as a Nemoh (.nemoh) mesh for Capytaine BEM and a
Wavefront OBJ (.obj) mesh for SEA-Stack 3-D visualization.  Meshes are
in body-local coordinates (origin at waterline midship, z=0 at WL).

The geometry functions are copied from the single-hull Wigley demo
(data/demos/run_seastack/wigley/assets/generate_meshes.py) to avoid
cross-demo import-path coupling.

No external dependencies required -- uses only the Python standard library.

Usage:
    python generate_meshes.py            # defaults
    python generate_meshes.py --help     # show all options
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path


# ---------------------------------------------------------------------------
# Center hull defaults
# ---------------------------------------------------------------------------
CENTER_LENGTH = 50.0
CENTER_BEAM = 12.0
CENTER_DRAFT = 3.5
CENTER_FREEBOARD = 3.5

# ---------------------------------------------------------------------------
# Outrigger hull defaults
# ---------------------------------------------------------------------------
OUTRIGGER_LENGTH = 25.0
OUTRIGGER_BEAM = 4.0
OUTRIGGER_DRAFT = 1.5
OUTRIGGER_FREEBOARD = 1.5

# Mesh resolution
N_LONGITUDINAL = 48
N_UNDERWATER = 12
N_TOPSIDES = 6
N_DECK_TRANSVERSE = 8

RHO = 1025.0


# ---------------------------------------------------------------------------
# Geometry helpers  (copied from wigley/assets/generate_meshes.py)
# ---------------------------------------------------------------------------

def wigley_half_beam(
    x: float, z: float,
    L: float, B: float, T: float,
) -> float:
    """Half-beam y >= 0 at hull station (x, z).

    Below waterline (z <= 0): classical Wigley parabolic form.
    Above waterline (z > 0):  vertical topsides (waterline beam).
    """
    y_wl = (B / 2.0) * max(1.0 - (2.0 * x / L) ** 2, 0.0)
    if z <= 0.0:
        return max(y_wl * (1.0 - (z / T) ** 2), 0.0)
    return y_wl


def cosine_space(a: float, b: float, n: int) -> list[float]:
    """Return n+1 values from *a* to *b* with cosine clustering at both ends."""
    mid = (a + b) / 2.0
    half = (b - a) / 2.0
    return [mid - half * math.cos(math.pi * i / n) for i in range(n + 1)]


# ---------------------------------------------------------------------------
# Mesh construction  (copied from wigley/assets/generate_meshes.py)
# ---------------------------------------------------------------------------

def build_hull_mesh(
    L: float,
    B: float,
    T: float,
    F: float,
    n_x: int,
    n_zs: int,
    n_zt: int,
    n_y: int,
    cosine_x: bool = True,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int, int]]]:
    """Build a closed Wigley hull mesh (quad panels).

    Returns
    -------
    vertices : list of (x, y, z)
    quads    : list of (v0, v1, v2, v3) -- 0-based indices, outward normals
    """
    if cosine_x:
        x_stations = cosine_space(-L / 2.0, L / 2.0, n_x)
    else:
        x_stations = [(-L / 2.0) + L * i / n_x for i in range(n_x + 1)]

    z_under = cosine_space(-T, 0.0, n_zs)
    z_above = cosine_space(0.0, F, n_zt)

    n_ring = 2 * n_zt + 2 * n_zs + n_y

    vertices: list[tuple[float, float, float]] = []

    for xi in x_stations:
        y_wl = (B / 2.0) * max(1.0 - (2.0 * xi / L) ** 2, 0.0)

        for j in range(n_zt, -1, -1):
            z = z_above[j]
            vertices.append((xi, -y_wl, z))

        for j in range(n_zs - 1, -1, -1):
            z = z_under[j]
            y = wigley_half_beam(xi, z, L, B, T)
            vertices.append((xi, -y, z))

        for j in range(1, n_zs + 1):
            z = z_under[j]
            y = wigley_half_beam(xi, z, L, B, T)
            vertices.append((xi, y, z))

        for j in range(1, n_zt + 1):
            z = z_above[j]
            vertices.append((xi, y_wl, z))

        for m in range(1, n_y):
            frac = m / n_y
            y_deck = y_wl * (1.0 - 2.0 * frac)
            vertices.append((xi, y_deck, F))

    quads: list[tuple[int, int, int, int]] = []
    for i in range(n_x):
        base_i = i * n_ring
        base_next = (i + 1) * n_ring
        for k in range(n_ring):
            k1 = (k + 1) % n_ring
            v0 = base_i + k
            v1 = base_i + k1
            v2 = base_next + k1
            v3 = base_next + k
            quads.append((v0, v1, v2, v3))

    return vertices, quads


# ---------------------------------------------------------------------------
# Mesh writers  (copied from wigley/assets/generate_meshes.py)
# ---------------------------------------------------------------------------

def write_nemoh(
    path: Path,
    vertices: list[tuple[float, float, float]],
    quads: list[tuple[int, int, int, int]],
) -> None:
    """Write a mesh in Nemoh format."""
    with open(path, "w") as f:
        f.write(f"     {len(vertices)}     {len(quads)}\n")
        for i, (x, y, z) in enumerate(vertices):
            f.write(f"     {i+1}      {x:16.6f}      {y:16.6f}      {z:16.6f}\n")
        f.write("     0       0.000000       0.000000       0.000000\n")
        for v0, v1, v2, v3 in quads:
            f.write(f"     {v0+1}     {v1+1}     {v2+1}     {v3+1}\n")
        f.write("     0     0     0     0\n")


def write_obj(
    path: Path,
    vertices: list[tuple[float, float, float]],
    quads: list[tuple[int, int, int, int]],
    header: str = "Wigley hull mesh",
) -> None:
    """Write a Wavefront OBJ file."""
    with open(path, "w") as f:
        f.write(f"# {header}\n")
        for x, y, z in vertices:
            f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for v0, v1, v2, v3 in quads:
            f.write(f"f {v0+1} {v1+1} {v2+1} {v3+1}\n")


# ---------------------------------------------------------------------------
# Hydrostatic summary
# ---------------------------------------------------------------------------

def print_hull_summary(
    name: str,
    L: float, B: float, T: float, F: float,
    n_panels: int,
) -> None:
    """Print analytical hydrostatic estimates for a Wigley hull."""
    vol_displaced = 4.0 * L * B * T / 9.0
    wp_area = 2.0 * L * B / 3.0
    mass = RHO * vol_displaced
    cog_z = -T / 2.0

    kxx = 0.35 * B
    kyy = 0.25 * L
    kzz = 0.25 * L
    Ixx = mass * kxx * kxx
    Iyy = mass * kyy * kyy
    Izz = mass * kzz * kzz

    print(f"\n--- {name} ---")
    print(f"  L={L:.1f} m, B={B:.1f} m, T={T:.1f} m, F={F:.1f} m")
    print(f"  Panels:            {n_panels}")
    print(f"  Displaced volume:  {vol_displaced:.1f} m^3")
    print(f"  Waterplane area:   {wp_area:.1f} m^2")
    print(f"  Equilibrium mass:  {mass:.0f} kg  ({mass/1000:.0f} t)")
    print(f"  CoG (approx):      z = {cog_z:.2f} m")
    print(f"  Ixx (roll):        {Ixx:.0f} kg-m^2  (kxx = {kxx:.1f} m)")
    print(f"  Iyy (pitch):       {Iyy:.0f} kg-m^2  (kyy = {kyy:.1f} m)")
    print(f"  Izz (yaw):         {Izz:.0f} kg-m^2  (kzz = {kzz:.1f} m)")


# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------

def generate(
    output_dir: Path | None = None,
    n_x: int = N_LONGITUDINAL,
    n_zs: int = N_UNDERWATER,
    n_zt: int = N_TOPSIDES,
    n_y: int = N_DECK_TRANSVERSE,
) -> None:
    if output_dir is None:
        output_dir = Path(__file__).resolve().parent

    meshes_dir = output_dir / "meshes"
    geom_dir = output_dir / "geometry"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    geom_dir.mkdir(parents=True, exist_ok=True)

    # --- Center hull ---
    print("Center hull...")
    c_verts, c_quads = build_hull_mesh(
        CENTER_LENGTH, CENTER_BEAM, CENTER_DRAFT, CENTER_FREEBOARD,
        n_x, n_zs, n_zt, n_y,
    )
    write_nemoh(meshes_dir / "center.nemoh", c_verts, c_quads)
    write_obj(geom_dir / "center.obj", c_verts, c_quads,
              header="Trimaran center hull (Wigley)")
    print(f"  Nemoh: center.nemoh  ({len(c_verts)} verts, {len(c_quads)} panels)")
    print(f"  OBJ:   center.obj")

    print_hull_summary("Center hull",
                       CENTER_LENGTH, CENTER_BEAM, CENTER_DRAFT, CENTER_FREEBOARD,
                       len(c_quads))

    # --- Outrigger hull (single mesh; BEM places instances at y=-/+ OUTRIGGER spacing in run_bem.py) ---
    print("\nOutrigger hull...")
    o_verts, o_quads = build_hull_mesh(
        OUTRIGGER_LENGTH, OUTRIGGER_BEAM, OUTRIGGER_DRAFT, OUTRIGGER_FREEBOARD,
        n_x, n_zs, n_zt, n_y,
    )
    write_nemoh(meshes_dir / "outrigger.nemoh", o_verts, o_quads)
    write_obj(geom_dir / "outrigger.obj", o_verts, o_quads,
              header="Trimaran outrigger hull (Wigley)")
    print(f"  Nemoh: outrigger.nemoh  ({len(o_verts)} verts, {len(o_quads)} panels)")
    print(f"  OBJ:   outrigger.obj")

    print_hull_summary("Outrigger hull",
                       OUTRIGGER_LENGTH, OUTRIGGER_BEAM, OUTRIGGER_DRAFT, OUTRIGGER_FREEBOARD,
                       len(o_quads))


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate trimaran Wigley hull meshes")
    ap.add_argument("--n-x", type=int, default=N_LONGITUDINAL)
    ap.add_argument("--n-zs", type=int, default=N_UNDERWATER)
    ap.add_argument("--n-zt", type=int, default=N_TOPSIDES)
    ap.add_argument("--n-y", type=int, default=N_DECK_TRANSVERSE)
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    print("Generating trimaran hull meshes...\n")
    generate(
        output_dir=args.output_dir,
        n_x=args.n_x,
        n_zs=args.n_zs,
        n_zt=args.n_zt,
        n_y=args.n_y,
    )
    print("\nDone.")


if __name__ == "__main__":
    main()
