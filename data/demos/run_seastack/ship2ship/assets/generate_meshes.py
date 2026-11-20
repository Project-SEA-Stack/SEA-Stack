"""Generate the Wigley hull meshes for the ship-to-ship transfer demo.

Both vessels are identical 50 m Wigley hulls, so a single mesh is written and
``run_bem.py`` instantiates it twice at y = -10 m and y = +10 m. The hull
parametrisation is imported from the wigley demo assets rather than duplicated,
so the two demos cannot drift apart.

Coordinate convention (BEM frame, shared by the .nemoh, the .obj and the .h5):
    x  longitudinal, bow at -L/2, stern at +L/2
    y  transverse, starboard positive
    z  vertical, undisturbed free surface at 0, keel at -T, deck at +F

Usage:
    python generate_meshes.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Single source of truth for the Wigley hull geometry.
_WIGLEY_ASSETS = (Path(__file__).resolve().parents[2] / "wigley" / "assets")
if str(_WIGLEY_ASSETS) not in sys.path:
    sys.path.insert(0, str(_WIGLEY_ASSETS))

try:
    from generate_meshes import build_hull_mesh, write_nemoh, write_obj  # type: ignore
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        f"Could not import the Wigley hull builder from {_WIGLEY_ASSETS}.\n"
        "The ship2ship demo reuses data/demos/run_seastack/wigley/assets/generate_meshes.py."
    ) from exc

# ---------------------------------------------------------------------------
# Hull geometry -- must match run_bem.py and the model YAML
# ---------------------------------------------------------------------------
LENGTH = 50.0       # m, waterline length
BEAM = 12.0         # m, maximum beam at the waterline
DRAFT = 3.5         # m, below the waterline
FREEBOARD = 3.5     # m, deck above the waterline

# Mesh resolution (same as the wigley demo defaults).
N_LONGITUDINAL = 48
N_UNDERWATER = 12
N_TOPSIDES = 6
N_DECK_TRANSVERSE = 8


def generate(output_dir: Path) -> None:
    meshes_dir = output_dir / "meshes"
    geom_dir = output_dir / "geometry"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    geom_dir.mkdir(parents=True, exist_ok=True)

    vertices, quads = build_hull_mesh(
        LENGTH, BEAM, DRAFT, FREEBOARD,
        N_LONGITUDINAL, N_UNDERWATER, N_TOPSIDES, N_DECK_TRANSVERSE,
        cosine_x=True,
    )

    nemoh_path = meshes_dir / "hull.nemoh"
    write_nemoh(nemoh_path, vertices, quads)
    print(f"  Nemoh: {nemoh_path.name}  ({len(vertices)} verts, {len(quads)} panels)")

    obj_path = geom_dir / "hull.obj"
    write_obj(obj_path, vertices, quads, header="Ship-to-ship Wigley hull (50 m)")
    print(f"  OBJ:   {obj_path.name}")

    # Analytic Wigley hydrostatics; run_bem.py checks the panel mesh against these
    # and the model YAML uses the same mass so the hulls float at z = 0.
    rho = 1025.0
    vol = 4.0 * LENGTH * BEAM * DRAFT / 9.0
    awp = 2.0 * LENGTH * BEAM / 3.0
    mass = rho * vol
    kxx, kyy = 0.35 * BEAM, 0.25 * LENGTH

    print("\n--- Hull summary (per vessel) ---")
    print(f"  L x B x T x F:      {LENGTH} x {BEAM} x {DRAFT} x {FREEBOARD} m")
    print(f"  Displaced volume:   {vol:.2f} m^3   (4/9 L B T)")
    print(f"  Waterplane area:    {awp:.2f} m^2   (2/3 L B)")
    print(f"  Equilibrium mass:   {mass:.0f} kg  ({mass / 1000.0:.0f} t)")
    print(f"  Ixx (roll):         {mass * kxx * kxx:.0f} kg-m^2  (kxx = {kxx:.2f} m)")
    print(f"  Iyy = Izz:          {mass * kyy * kyy:.0f} kg-m^2  (kyy = {kyy:.2f} m)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    print("Generating ship-to-ship Wigley hull mesh...\n")
    generate(args.output_dir or Path(__file__).resolve().parent)
    print("\nDone.")


if __name__ == "__main__":
    main()
