"""Generate 5SA (Attenuator - 5 Segments) panel meshes for BEM and visualization.

Each segment is defined by a longitudinal profile of (x, radius) stations that are
revolved around the x-axis. The resulting quad-panel surface mesh is written in both
Nemoh (.nemoh) format for Capytaine BEM analysis and Wavefront OBJ (.obj) format for
SEA-Stack 3D visualization.

No external dependencies required -- uses only the Python standard library.

Usage:
    python generate_meshes.py            # defaults: 4 m diameter, 36 m segment, no gap
    python generate_meshes.py --help     # show all options
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path


# ---------------------------------------------------------------------------
# 5SA defaults
# ---------------------------------------------------------------------------
DIAMETER = 4.0          # m
SEGMENT_LENGTH = 36.0   # m
GAP = 0.0               # m between segment ends (0 = segments touch)
N_SEGMENTS = 5
DRAFT = 0.259199             # m  (CoG depth below waterline)

# Mesh resolution
N_LONGITUDINAL = 40     # panels along segment length (x)
N_CIRCUMFERENTIAL = 24  # panels around the circumference


def compute_segment_centers(
    n_segments: int,
    segment_length: float,
    gap: float,
) -> list[float]:
    """Return x-coordinates of each segment center.

    Segment 0's nose starts at x = 0, so its center is at segment_length / 2.
    """
    spacing = segment_length + gap
    return [segment_length / 2.0 + i * spacing for i in range(n_segments)]


def make_segment_profile(
    length: float,
    radius: float,
    n_x: int,
    cap_fraction: float = 0.05,
    cosine_spacing: bool = True,
) -> list[tuple[float, float]]:
    """Return a list of (x_local, r) stations for one segment.

    The segment runs from x_local = -length/2 to +length/2 with hemispherical
    end-caps over the first/last *cap_fraction* of the length.

    When *cosine_spacing* is True the longitudinal stations follow a cosine
    distribution, clustering points near each end of the segment for better
    resolution of the end-cap geometry and hydrodynamic pressure gradients.
    """
    half = length / 2.0
    cap_len = cap_fraction * length
    profile = []
    for i in range(n_x + 1):
        if cosine_spacing:
            # Maps i in [0, n_x] to x in [-half, half] with clustering at ends
            theta = math.pi * i / n_x
            xi = -half * math.cos(theta)
        else:
            xi = -half + (length * i) / n_x
        dist_from_end = min(xi - (-half), half - xi)
        if dist_from_end < cap_len and cap_len > 0:
            t = dist_from_end / cap_len
            ri = radius * math.sqrt(max(t * (2.0 - t), 0.0))
        else:
            ri = radius
        profile.append((xi, ri))
    return profile


def revolve_profile(
    profile: list[tuple[float, float]],
    n_theta: int,
    z_center: float = 0.0,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int, int]]]:
    """Revolve a 2-D profile (x, r) around the x-axis.

    Parameters
    ----------
    profile : list of (x_local, radius)
    n_theta : int
        Number of circumferential divisions.
    z_center : float
        Vertical offset of the revolution axis.

    Returns
    -------
    vertices : list of (x, y, z)
    quads : list of (v0, v1, v2, v3)  -- 0-based indices
    """
    n_x = len(profile)
    theta_vals = [2.0 * math.pi * j / n_theta for j in range(n_theta)]

    vertices: list[tuple[float, float, float]] = []
    for xi, ri in profile:
        for th in theta_vals:
            y = ri * math.sin(th)
            z = ri * math.cos(th) + z_center
            vertices.append((xi, y, z))

    quads: list[tuple[int, int, int, int]] = []
    for i in range(n_x - 1):
        for j in range(n_theta):
            j_next = (j + 1) % n_theta
            v0 = i * n_theta + j
            v1 = i * n_theta + j_next
            v2 = (i + 1) * n_theta + j_next
            v3 = (i + 1) * n_theta + j
            quads.append((v0, v1, v2, v3))

    return vertices, quads


# ---------------------------------------------------------------------------
# Nemoh mesh writer
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
# OBJ mesh writer
# ---------------------------------------------------------------------------

def write_obj(
    path: Path,
    vertices: list[tuple[float, float, float]],
    quads: list[tuple[int, int, int, int]],
    header: str = "5SA segment mesh",
) -> None:
    """Write a Wavefront OBJ file."""
    with open(path, "w") as f:
        f.write(f"# {header}\n")
        for x, y, z in vertices:
            f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for v0, v1, v2, v3 in quads:
            f.write(f"f {v0+1} {v1+1} {v2+1} {v3+1}\n")


# ---------------------------------------------------------------------------
# Computed properties
# ---------------------------------------------------------------------------

def estimate_submerged_volume(
    profile: list[tuple[float, float]],
    draft: float,
) -> tuple[float, float]:
    """Estimate displaced volume and waterplane area using trapezoidal integration.

    Assumes the cylinder axis is at z = -draft (center of circular cross-section).
    Waterline is at z = 0.
    """
    total_volume = 0.0
    total_wp_area = 0.0

    for i in range(len(profile) - 1):
        x0, r0 = profile[i]
        x1, r1 = profile[i + 1]
        dx = x1 - x0

        for r in (r0, r1):
            if r <= 0:
                a_sub = 0.0
                wp_width = 0.0
            elif draft >= r:
                a_sub = math.pi * r * r
                wp_width = 0.0
            else:
                h_above = r - draft
                if h_above <= 0:
                    a_sub = math.pi * r * r
                    wp_width = 0.0
                else:
                    a_seg = r * r * math.acos((r - h_above) / r) - (r - h_above) * math.sqrt(2 * r * h_above - h_above * h_above)
                    a_sub = math.pi * r * r - a_seg
                    half_chord = math.sqrt(r * r - (r - h_above) ** 2)
                    wp_width = 2.0 * half_chord
            if r == r0:
                a_sub_0, wp_0 = a_sub, wp_width
            else:
                a_sub_1, wp_1 = a_sub, wp_width

        total_volume += 0.5 * (a_sub_0 + a_sub_1) * dx
        total_wp_area += 0.5 * (wp_0 + wp_1) * dx

    return total_volume, total_wp_area


# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------

def generate(
    n_segments: int = N_SEGMENTS,
    segment_length: float = SEGMENT_LENGTH,
    gap: float = GAP,
    diameter: float = DIAMETER,
    draft: float = DRAFT,
    n_x: int = N_LONGITUDINAL,
    n_theta: int = N_CIRCUMFERENTIAL,
    cap_fraction: float = 0.05,
    cosine_spacing: bool = True,
    output_dir: Path | None = None,
) -> None:
    radius = diameter / 2.0
    centers = compute_segment_centers(n_segments, segment_length, gap)

    if output_dir is None:
        output_dir = Path(__file__).resolve().parent

    meshes_dir = output_dir / "meshes"
    geom_dir = output_dir / "geometry"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    geom_dir.mkdir(parents=True, exist_ok=True)

    profile = make_segment_profile(segment_length, radius, n_x, cap_fraction,
                                   cosine_spacing=cosine_spacing)
    verts_template, quads = revolve_profile(profile, n_theta, z_center=0.0)

    vol_sub, wp_area = estimate_submerged_volume(profile, draft)
    rho = 1025.0
    mass_equil = rho * vol_sub

    for seg_idx in range(n_segments):
        cx = centers[seg_idx]
        seg_name = f"segment_{seg_idx + 1}"

        # Nemoh mesh: global frame (translate x to segment position, z down by draft)
        verts_global = [(x + cx, y, z - draft) for x, y, z in verts_template]
        nemoh_path = meshes_dir / f"{seg_name}.nemoh"
        write_nemoh(nemoh_path, verts_global, quads)
        print(f"  Nemoh: {nemoh_path.name}  ({len(verts_global)} verts, {len(quads)} panels)")

        # OBJ mesh: body-local frame (centered at CoG = origin)
        obj_path = geom_dir / f"{seg_name}.obj"
        write_obj(obj_path, verts_template, quads, header=f"5SA {seg_name}")
        print(f"  OBJ:   {obj_path.name}")

    # Template OBJ
    obj_template = geom_dir / "segment.obj"
    write_obj(obj_template, verts_template, quads, header="5SA template segment")

    print(f"\n--- Layout summary ---")
    print(f"  Segments:          {n_segments}")
    print(f"  Segment length:    {segment_length:.1f} m")
    print(f"  Diameter:          {diameter:.1f} m")
    print(f"  Gap:               {gap:.1f} m")
    print(f"  Draft (CoG z):     {draft:.1f} m")
    nose_x = centers[0] - segment_length / 2.0
    tail_x = centers[-1] + segment_length / 2.0
    print(f"  Overall span:      {tail_x - nose_x:.1f} m  (nose x={nose_x:.1f}, tail x={tail_x:.1f})")
    print(f"  Segment centers:   {['%.1f' % c for c in centers]}")
    spacing_type = "cosine" if cosine_spacing else "uniform"
    print(f"  Mesh resolution:   {n_x} x {n_theta} = {n_x * n_theta} panels/segment ({spacing_type} longitudinal spacing)")

    print(f"\n--- Hydrostatics (per segment) ---")
    print(f"  Submerged volume:  {vol_sub:.2f} m^3")
    print(f"  Waterplane area:   {wp_area:.2f} m^2")
    print(f"  Equilibrium mass:  {mass_equil:.1f} kg  ({mass_equil/1000:.1f} t)")
    print(f"  Total device mass: {mass_equil * n_segments / 1000:.1f} t")

    # Rough inertia estimates (solid cylinder approximation at equilibrium mass)
    Ixx = 0.5 * mass_equil * radius * radius
    Iyy = mass_equil * (3.0 * radius * radius + segment_length * segment_length) / 12.0
    Izz = Iyy
    print(f"  Ixx (approx):      {Ixx:.0f} kg-m^2")
    print(f"  Iyy (approx):      {Iyy:.0f} kg-m^2")
    print(f"  Izz (approx):      {Izz:.0f} kg-m^2")


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate 5SA meshes")
    ap.add_argument("--diameter", type=float, default=DIAMETER)
    ap.add_argument("--length", type=float, default=SEGMENT_LENGTH)
    ap.add_argument("--gap", type=float, default=GAP)
    ap.add_argument("--draft", type=float, default=DRAFT)
    ap.add_argument("--n-segments", type=int, default=N_SEGMENTS)
    ap.add_argument("--n-x", type=int, default=N_LONGITUDINAL)
    ap.add_argument("--n-theta", type=int, default=N_CIRCUMFERENTIAL)
    ap.add_argument("--cap-fraction", type=float, default=0.05,
                    help="Fraction of segment length for hemispherical end-caps (default: 0.05)")
    ap.add_argument("--uniform-spacing", action="store_true",
                    help="Use uniform longitudinal spacing instead of cosine (default: cosine)")
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    print("Generating 5SA meshes...\n")
    generate(
        n_segments=args.n_segments,
        segment_length=args.length,
        gap=args.gap,
        diameter=args.diameter,
        draft=args.draft,
        n_x=args.n_x,
        n_theta=args.n_theta,
        cap_fraction=args.cap_fraction,
        cosine_spacing=not args.uniform_spacing,
        output_dir=args.output_dir,
    )
    print("\nDone.")


if __name__ == "__main__":
    main()
