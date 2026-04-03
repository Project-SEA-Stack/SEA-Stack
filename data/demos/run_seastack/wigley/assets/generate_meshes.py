"""Generate a parametric Wigley hull mesh for BEM and visualization.

The hull is defined by the classical Wigley parabolic formula below the
waterline, with vertical topsides above and a flat deck cap.  The result
is a watertight quad-panel surface mesh written in both Nemoh (.nemoh) format
for Capytaine BEM analysis and Wavefront OBJ (.obj) format for SEA-Stack
3D visualization.

Coordinate convention (body-local, same as Nemoh output):
    x  -- longitudinal, bow at -L/2, stern at +L/2
    y  -- transverse, starboard positive
    z  -- vertical, waterline at 0, keel at -T, deck at +F

In Capytaine the mesh is loaded as-is (waterline at z=0) and
``keep_immersed_part()`` clips the above-water panels.

No external dependencies required -- uses only the Python standard library.

Usage:
    python generate_meshes.py            # defaults: 50 m ship
    python generate_meshes.py --help     # show all options
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path


# ---------------------------------------------------------------------------
# Wigley hull defaults (ship-scale, car-on-deck)
# ---------------------------------------------------------------------------
LENGTH = 50.0           # m -- waterline length
BEAM = 12.0             # m -- maximum beam at waterline
DRAFT = 3.5             # m -- draft below waterline
FREEBOARD = 3.5         # m -- deck height above waterline

# Mesh resolution
N_LONGITUDINAL = 48     # stations along hull length (x)
N_UNDERWATER = 12       # z-stations per side, keel to waterline
N_TOPSIDES = 6          # z-stations per side, waterline to deck
N_DECK_TRANSVERSE = 8   # transverse strips across deck


# ---------------------------------------------------------------------------
# Geometry helpers
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
# Mesh construction
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

    Each longitudinal station has a closed cross-section ring that traverses:

        port deck edge -> port topsides (down) -> port underwater (down to
        keel) -> starboard underwater (up) -> starboard topsides (up) ->
        starboard deck edge -> deck interior (back to port deck edge)

    Adjacent rings are connected by quad panels.  At bow and stern the ring
    degenerates (y -> 0 everywhere) producing triangular panels, which is
    standard for Nemoh / Capytaine.

    Returns
    -------
    vertices : list of (x, y, z)
    quads    : list of (v0, v1, v2, v3) -- 0-based indices, outward normals
    """
    # Longitudinal stations
    if cosine_x:
        x_stations = cosine_space(-L / 2.0, L / 2.0, n_x)
    else:
        x_stations = [(-L / 2.0) + L * i / n_x for i in range(n_x + 1)]

    # Vertical stations (cosine-spaced, ascending)
    z_under = cosine_space(-T, 0.0, n_zs)   # keel -> waterline
    z_above = cosine_space(0.0, F, n_zt)     # waterline -> deck

    # Vertices per ring (closed loop around the section)
    #   Seg A : port topsides      n_zt + 1
    #   Seg B : port underwater    n_zs      (skip shared WL vertex)
    #   Seg C : stbd underwater    n_zs      (skip shared keel vertex)
    #   Seg D : stbd topsides      n_zt      (skip shared WL vertex)
    #   Seg E : deck interior      n_y - 1   (skip both deck-edge vertices)
    n_ring = 2 * n_zt + 2 * n_zs + n_y

    vertices: list[tuple[float, float, float]] = []

    for xi in x_stations:
        y_wl = (B / 2.0) * max(1.0 - (2.0 * xi / L) ** 2, 0.0)

        # Segment A: port topsides (deck -> waterline)
        for j in range(n_zt, -1, -1):
            z = z_above[j]
            vertices.append((xi, -y_wl, z))

        # Segment B: port underwater (just below WL -> keel)
        for j in range(n_zs - 1, -1, -1):
            z = z_under[j]
            y = wigley_half_beam(xi, z, L, B, T)
            vertices.append((xi, -y, z))

        # Segment C: starboard underwater (just above keel -> WL)
        for j in range(1, n_zs + 1):
            z = z_under[j]
            y = wigley_half_beam(xi, z, L, B, T)
            vertices.append((xi, y, z))

        # Segment D: starboard topsides (just above WL -> deck)
        for j in range(1, n_zt + 1):
            z = z_above[j]
            vertices.append((xi, y_wl, z))

        # Segment E: deck interior (stbd edge -> port edge, skip endpoints)
        for m in range(1, n_y):
            frac = m / n_y
            y_deck = y_wl * (1.0 - 2.0 * frac)
            vertices.append((xi, y_deck, F))

    # Quad panels connecting adjacent rings
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
# Nemoh mesh writer  (same format as 5SA generate_meshes.py)
# ---------------------------------------------------------------------------

def write_nemoh(
    path: Path,
    vertices: list[tuple[float, float, float]],
    quads: list[tuple[int, int, int, int]],
) -> None:
    """Write a mesh in Nemoh format.

    Format:
        Line 1: <n_vertices> <n_panels>
        Vertex block: <1-based-id> <x> <y> <z>
        Vertex terminator: 0  0.0  0.0  0.0
        Panel block: <v1> <v2> <v3> <v4>  (1-based)
        Panel terminator: 0  0  0  0
    """
    with open(path, "w") as f:
        f.write(f"     {len(vertices)}     {len(quads)}\n")
        for i, (x, y, z) in enumerate(vertices):
            f.write(f"     {i+1}      {x:16.6f}      {y:16.6f}      {z:16.6f}\n")
        f.write("     0       0.000000       0.000000       0.000000\n")
        for v0, v1, v2, v3 in quads:
            f.write(f"     {v0+1}     {v1+1}     {v2+1}     {v3+1}\n")
        f.write("     0     0     0     0\n")


# ---------------------------------------------------------------------------
# OBJ mesh writer  (same format as 5SA generate_meshes.py)
# ---------------------------------------------------------------------------

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
# Main generation
# ---------------------------------------------------------------------------

def generate(
    length: float = LENGTH,
    beam: float = BEAM,
    draft: float = DRAFT,
    freeboard: float = FREEBOARD,
    n_x: int = N_LONGITUDINAL,
    n_zs: int = N_UNDERWATER,
    n_zt: int = N_TOPSIDES,
    n_y: int = N_DECK_TRANSVERSE,
    cosine_x: bool = True,
    output_dir: Path | None = None,
) -> None:
    if output_dir is None:
        output_dir = Path(__file__).resolve().parent

    meshes_dir = output_dir / "meshes"
    geom_dir = output_dir / "geometry"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    geom_dir.mkdir(parents=True, exist_ok=True)

    vertices, quads = build_hull_mesh(
        length, beam, draft, freeboard,
        n_x, n_zs, n_zt, n_y,
        cosine_x=cosine_x,
    )

    # Nemoh mesh -- coordinates are already in the global/BEM frame
    # (waterline at z=0, same convention as 5SA after its z-draft shift).
    nemoh_path = meshes_dir / "wigley.nemoh"
    write_nemoh(nemoh_path, vertices, quads)
    print(f"  Nemoh: {nemoh_path.name}  ({len(vertices)} verts, {len(quads)} panels)")

    # OBJ mesh -- body-local frame (same coords; body origin at waterline midship)
    obj_path = geom_dir / "wigley.obj"
    write_obj(obj_path, vertices, quads, header="Wigley hull")
    print(f"  OBJ:   {obj_path.name}")

    # --- Hydrostatic estimates (analytical Wigley integrals) ---
    rho = 1025.0
    vol_displaced = 4.0 * length * beam * draft / 9.0    # V = (4/9)*L*B*T
    wp_area = 2.0 * length * beam / 3.0                  # A = (2/3)*L*B
    mass = rho * vol_displaced
    cog_z = -draft / 2.0                                  # approximate CoG

    # Gyradius-based inertia (typical ship coefficients)
    kxx = 0.35 * beam
    kyy = 0.25 * length
    kzz = 0.25 * length
    Ixx = mass * kxx * kxx
    Iyy = mass * kyy * kyy
    Izz = mass * kzz * kzz

    n_ring = 2 * n_zt + 2 * n_zs + n_y
    spacing_type = "cosine" if cosine_x else "uniform"

    print(f"\n--- Hull summary ---")
    print(f"  Length (WL):       {length:.1f} m")
    print(f"  Beam:              {beam:.1f} m")
    print(f"  Draft:             {draft:.1f} m")
    print(f"  Freeboard:         {freeboard:.1f} m")
    print(f"  B/L:               {beam/length:.3f}")
    print(f"  T/L:               {draft/length:.3f}")
    print(f"  Mesh resolution:   {n_x} x {n_ring} = {n_x * n_ring} panels ({spacing_type} longitudinal)")

    print(f"\n--- Hydrostatics (analytical) ---")
    print(f"  Displaced volume:  {vol_displaced:.1f} m^3")
    print(f"  Waterplane area:   {wp_area:.1f} m^2")
    print(f"  Equilibrium mass:  {mass:.0f} kg  ({mass/1000:.0f} t)")
    print(f"  CoG (approx):      z = {cog_z:.2f} m")

    print(f"\n--- Inertia estimates (gyradius) ---")
    print(f"  Ixx (roll):        {Ixx:.0f} kg-m^2  (kxx = {kxx:.1f} m)")
    print(f"  Iyy (pitch):       {Iyy:.0f} kg-m^2  (kyy = {kyy:.1f} m)")
    print(f"  Izz (yaw):         {Izz:.0f} kg-m^2  (kzz = {kzz:.1f} m)")


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate Wigley hull meshes")
    ap.add_argument("--length", type=float, default=LENGTH)
    ap.add_argument("--beam", type=float, default=BEAM)
    ap.add_argument("--draft", type=float, default=DRAFT)
    ap.add_argument("--freeboard", type=float, default=FREEBOARD)
    ap.add_argument("--n-x", type=int, default=N_LONGITUDINAL)
    ap.add_argument("--n-zs", type=int, default=N_UNDERWATER)
    ap.add_argument("--n-zt", type=int, default=N_TOPSIDES)
    ap.add_argument("--n-y", type=int, default=N_DECK_TRANSVERSE)
    ap.add_argument("--uniform-spacing", action="store_true",
                    help="Use uniform longitudinal spacing instead of cosine")
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    print("Generating Wigley hull meshes...\n")
    generate(
        length=args.length,
        beam=args.beam,
        draft=args.draft,
        freeboard=args.freeboard,
        n_x=args.n_x,
        n_zs=args.n_zs,
        n_zt=args.n_zt,
        n_y=args.n_y,
        cosine_x=not args.uniform_spacing,
        output_dir=args.output_dir,
    )
    print("\nDone.")


if __name__ == "__main__":
    main()
