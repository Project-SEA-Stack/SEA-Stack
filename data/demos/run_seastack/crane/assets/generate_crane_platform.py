"""Generate platform geometry and BEM pile mesh for the crane demo.

Crane-part OBJs (pedestal, boom, jib, payload, hook) are authored by
assets/build_crane_case.py so dimensions stay in one place.

Outputs under geometry/ and meshes/:
  platform.obj       -- deck, piles, railings, cradle (platform-local)
  piles_bem.nemoh    -- immersed pile panels only (z = 0 .. -40 m)

Platform local frame: deck at z = 0; world placement (20, 16, 8) m.
Pile centres on a 6 x 4 m rectangle: (±3, ±2) m from platform centre.

Usage:
    python generate_crane_platform.py
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

# ---------------------------------------------------------------------------
# Platform dimensions (metres, z-up)
# ---------------------------------------------------------------------------
DECK_LX = 8.0
DECK_LY = 6.0
PILE_RECT_LX = 6.0
PILE_RECT_LY = 4.0
PILE_RADIUS = 0.45
PILE_Z_TOP = 0.0
PILE_Z_BOT = -40.0
WATER_DEPTH = 40.0

PLATFORM_WORLD = (20.0, 16.0, 8.0)

N_CYL_THETA = 16
N_CYL_Z = 24


def box_quads(
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
    zmin: float,
    zmax: float,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int, int]]]:
    c = [
        (xmin, ymin, zmin),
        (xmax, ymin, zmin),
        (xmax, ymax, zmin),
        (xmin, ymax, zmin),
        (xmin, ymin, zmax),
        (xmax, ymin, zmax),
        (xmax, ymax, zmax),
        (xmin, ymax, zmax),
    ]
    q = [
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ]
    return c, q


def cylinder_quads(
    cx: float,
    cy: float,
    z0: float,
    z1: float,
    radius: float,
    n_theta: int,
    n_z: int,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int, int]]]:
    """Panelized open cylinder (side surface only), outward normals."""
    if n_z < 1:
        raise ValueError("n_z must be >= 1")
    zs = [z0 + (z1 - z0) * i / n_z for i in range(n_z + 1)]
    vertices: list[tuple[float, float, float]] = []
    for z in zs:
        for k in range(n_theta):
            th = 2.0 * math.pi * k / n_theta
            vertices.append(
                (cx + radius * math.cos(th), cy + radius * math.sin(th), z)
            )

    quads: list[tuple[int, int, int, int]] = []
    ring = n_theta
    for iz in range(n_z):
        for k in range(n_theta):
            k1 = (k + 1) % n_theta
            v00 = iz * ring + k
            v01 = iz * ring + k1
            v11 = (iz + 1) * ring + k1
            v10 = (iz + 1) * ring + k
            quads.append((v00, v01, v11, v10))
    return vertices, quads


def write_nemoh(
    path: Path,
    vertices: list[tuple[float, float, float]],
    quads: list[tuple[int, int, int, int]],
) -> None:
    with open(path, "w") as f:
        f.write(f"     {len(vertices)}     {len(quads)}\n")
        for i, (x, y, z) in enumerate(vertices):
            f.write(f"     {i + 1}      {x:16.6f}      {y:16.6f}      {z:16.6f}\n")
        f.write("     0       0.000000       0.000000       0.000000\n")
        for v0, v1, v2, v3 in quads:
            f.write(f"     {v0 + 1}     {v1 + 1}     {v2 + 1}     {v3 + 1}\n")
        f.write("     0     0     0     0\n")


def pile_centres_local() -> list[tuple[float, float]]:
    hx, hy = PILE_RECT_LX / 2.0, PILE_RECT_LY / 2.0
    return [(-hx, -hy), (hx, -hy), (-hx, hy), (hx, hy)]


def generate_platform_obj(geom_dir: Path) -> None:
    """Deck, piles to z=-40, railings, raised U-shape cradle."""
    lx, ly = DECK_LX / 2.0, DECK_LY / 2.0
    deck_t = 0.25

    all_v: list[tuple[float, float, float]] = []
    all_q: list[tuple[int, int, int, int]] = []

    def add_part(part: tuple[list, list]) -> None:
        c, q = part
        base = len(all_v)
        all_v.extend(c)
        all_q.extend(
            (v0 + base, v1 + base, v2 + base, v3 + base) for v0, v1, v2, v3 in q
        )

    add_part(box_quads(-lx, lx, -ly, ly, -deck_t, 0.0))

    for px, py in pile_centres_local():
        add_part(
            cylinder_quads(px, py, -WATER_DEPTH, 0.0, PILE_RADIUS, N_CYL_THETA, N_CYL_Z)
        )

    rail_h = 1.0
    rail_t = 0.08
    add_part(box_quads(-lx, lx, ly - rail_t, ly, 0.0, rail_h))
    add_part(box_quads(-lx, lx, -ly, -ly + rail_t, 0.0, rail_h))
    add_part(box_quads(-lx, -lx + rail_t, -ly, ly, 0.0, rail_h))
    add_part(box_quads(lx - rail_t, lx, -ly, ly, 0.0, rail_h))

    cradle_w = 2.0
    cradle_h = 0.8
    cradle_leg = 1.5
    cx0 = -1.0
    add_part(
        box_quads(
            cx0, cx0 + cradle_leg, -cradle_w / 2, -cradle_w / 2 + 0.15, 0.0, cradle_h
        )
    )
    add_part(
        box_quads(
            cx0, cx0 + cradle_leg, cradle_w / 2 - 0.15, cradle_w / 2, 0.0, cradle_h
        )
    )
    add_part(
        box_quads(
            cx0, cx0 + cradle_leg, -cradle_w / 2, cradle_w / 2, cradle_h, cradle_h + 0.15
        )
    )

    path = geom_dir / "platform.obj"
    with open(path, "w") as f:
        f.write("# Offshore platform: deck, piles, railings, cradle\n")
        for x, y, z in all_v:
            f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for v0, v1, v2, v3 in all_q:
            f.write(f"f {v0 + 1} {v1 + 1} {v2 + 1} {v3 + 1}\n")
    print(f"  OBJ:   {path.name}  ({len(all_v)} verts, {len(all_q)} faces)")


def generate_piles_bem(meshes_dir: Path) -> None:
    """Immersed pile panels only, z = 0 .. -40, XY at pile centres."""
    all_v: list[tuple[float, float, float]] = []
    all_q: list[tuple[int, int, int, int]] = []

    for px, py in pile_centres_local():
        cv, qv = cylinder_quads(
            px, py, PILE_Z_BOT, PILE_Z_TOP, PILE_RADIUS, N_CYL_THETA, N_CYL_Z
        )
        base = len(all_v)
        all_v.extend(cv)
        all_q.extend(
            (v0 + base, v1 + base, v2 + base, v3 + base) for v0, v1, v2, v3 in qv
        )

    path = meshes_dir / "piles_bem.nemoh"
    write_nemoh(path, all_v, all_q)
    print(f"  Nemoh: {path.name}  ({len(all_v)} verts, {len(all_q)} panels, 4 piles)")


def generate(output_dir: Path) -> None:
    geom_dir = output_dir / "geometry"
    meshes_dir = output_dir / "meshes"
    geom_dir.mkdir(parents=True, exist_ok=True)
    meshes_dir.mkdir(parents=True, exist_ok=True)

    print("Generating platform geometry...")
    generate_platform_obj(geom_dir)

    print("\nGenerating BEM pile mesh...")
    generate_piles_bem(meshes_dir)

    wx, wy, wz = PLATFORM_WORLD
    print("\n--- Platform summary ---")
    print(f"  Deck:           {DECK_LX} x {DECK_LY} m")
    print(f"  Pile rectangle: {PILE_RECT_LX} x {PILE_RECT_LY} m  (R={PILE_RADIUS} m)")
    print(f"  World placement: ({wx}, {wy}, {wz}) m")
    print("  BEM pile world XY (after translate):")
    for px, py in pile_centres_local():
        print(f"    ({wx + px:.1f}, {wy + py:.1f})")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    print("Generating crane platform assets...\n")
    generate(args.output_dir or Path(__file__).resolve().parent)
    print("\nDone.")


if __name__ == "__main__":
    main()
