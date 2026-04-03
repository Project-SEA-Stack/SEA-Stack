/*********************************************************************
 * @file  submerged_volume.cpp
 * @brief MeshSubmergedVolume: divergence theorem with waterplane clipping.
 *
 * Theory (Faltinsen 1990, Ch. 3):
 *   Submerged volume via the divergence theorem. Each triangle (a,b,c)
 *   contributes a signed tetrahedron volume V_tet = a . (b x c) / 6
 *   and centroid moment M_tet = (a+b+c) * V_tet / 4 to the enclosed
 *   region's volume integral.
 *
 *   For the integral to be correct the surface must be closed.
 *   Triangles intersecting the waterplane are clipped, and the
 *   resulting waterplane cap polygon is triangulated to close the
 *   clipped region. This explicit cap ensures the divergence theorem
 *   yields the correct submerged volume.
 *********************************************************************/

#include <seastack/hydro/geometry/submerged_volume.h>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace seastack::hydro::geometry {

// ═══════════════════════════════════════════════════════════════════════
// Construction + validation
// ═══════════════════════════════════════════════════════════════════════

MeshSubmergedVolume::MeshSubmergedVolume(TriangleMesh mesh)
    : mesh_(std::move(mesh)) {
    auto topo = mesh_.AnalyzeTopology();
    if (!topo.is_closed) {
        std::ostringstream oss;
        oss << "Mesh has " << topo.num_boundary_edges
            << " boundary edges (mean boundary z = "
            << topo.boundary_z_mean << "). "
            << "Nonlinear hydrostatics requires a closed (watertight) mesh. "
            << "If this mesh represents only the wetted surface below the "
            << "waterline, provide a full-hull mesh instead. "
            << "See documentation for mesh requirements.";
        throw std::runtime_error(oss.str());
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Signed tetrahedron primitives
// ═══════════════════════════════════════════════════════════════════════

namespace {

constexpr double kVolumeEpsilon = 1.0e-15;

// Signed volume of the tetrahedron formed by triangle (a,b,c) and the origin.
inline double SignedTetVolume(const Eigen::Vector3d& a,
                              const Eigen::Vector3d& b,
                              const Eigen::Vector3d& c) {
    return a.dot(b.cross(c)) / 6.0;
}

// Accumulate a triangle's contribution to volume and first moment.
inline void AccumulateTriangle(const Eigen::Vector3d& a,
                               const Eigen::Vector3d& b,
                               const Eigen::Vector3d& c,
                               double& volume,
                               Eigen::Vector3d& moment) {
    double v = SignedTetVolume(a, b, c);
    volume += v;
    // Centroid of the tetrahedron (origin, a, b, c) is (a+b+c)/4
    moment += v * (a + b + c) / 4.0;
}

// Linearly interpolate between two points at a given z-level.
inline Eigen::Vector3d LerpAtZ(const Eigen::Vector3d& below,
                                const Eigen::Vector3d& above,
                                double z) {
    double t = (z - below.z()) / (above.z() - below.z());
    return below + t * (above - below);
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// Compute submerged volume
// ═══════════════════════════════════════════════════════════════════════

SubmergedVolumeResult MeshSubmergedVolume::Compute(
    const Eigen::Affine3d& body_to_world,
    double waterplane_z) const {

    const int nv = mesh_.num_vertices();
    const int nf = mesh_.num_faces();

    // Transform all vertices to world frame
    Eigen::MatrixXd world_verts(nv, 3);
    for (int i = 0; i < nv; ++i) {
        Eigen::Vector3d v = mesh_.vertices.row(i).transpose();
        world_verts.row(i) = (body_to_world * v).transpose();
    }

    double total_volume = 0.0;
    Eigen::Vector3d total_moment = Eigen::Vector3d::Zero();

    // Collect waterplane intersection edges for cap polygon
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> cap_edges;

    for (int f = 0; f < nf; ++f) {
        const Eigen::Vector3d v0 = world_verts.row(mesh_.faces(f, 0));
        const Eigen::Vector3d v1 = world_verts.row(mesh_.faces(f, 1));
        const Eigen::Vector3d v2 = world_verts.row(mesh_.faces(f, 2));

        const bool b0 = v0.z() <= waterplane_z;
        const bool b1 = v1.z() <= waterplane_z;
        const bool b2 = v2.z() <= waterplane_z;

        const int below_count = static_cast<int>(b0) + static_cast<int>(b1) + static_cast<int>(b2);

        if (below_count == 0) {
            // Entirely above waterplane -- skip
            continue;
        }

        if (below_count == 3) {
            // Entirely below waterplane -- accumulate full triangle
            AccumulateTriangle(v0, v1, v2, total_volume, total_moment);
            continue;
        }

        // Mixed: clip triangle against waterplane z = waterplane_z
        // Separate vertices into below/above sets while preserving winding order.
        // We iterate the three edges; when an edge crosses the waterplane,
        // we compute the intersection point.

        const Eigen::Vector3d* verts[3] = {&v0, &v1, &v2};
        const bool below[3] = {b0, b1, b2};

        std::vector<Eigen::Vector3d> submerged_poly;
        Eigen::Vector3d clip_a, clip_b;
        int clip_count = 0;

        for (int e = 0; e < 3; ++e) {
            int next = (e + 1) % 3;
            const Eigen::Vector3d& va = *verts[e];
            const Eigen::Vector3d& vb = *verts[next];

            if (below[e]) {
                submerged_poly.push_back(va);
            }

            // Check if edge crosses waterplane
            if (below[e] != below[next]) {
                const Eigen::Vector3d& lo = below[e] ? va : vb;
                const Eigen::Vector3d& hi = below[e] ? vb : va;
                Eigen::Vector3d ip = LerpAtZ(lo, hi, waterplane_z);
                submerged_poly.push_back(ip);

                // Track clipping points for cap polygon (ordered: first is
                // when we go from below to above, second from above to below)
                if (clip_count < 2) {
                    if (clip_count == 0) clip_a = ip;
                    else clip_b = ip;
                    clip_count++;
                }
            }
        }

        // Fan-triangulate the submerged polygon and accumulate
        for (size_t i = 1; i + 1 < submerged_poly.size(); ++i) {
            AccumulateTriangle(submerged_poly[0], submerged_poly[i],
                               submerged_poly[i + 1],
                               total_volume, total_moment);
        }

        // Record the waterplane clipping edge (for cap polygon)
        if (clip_count == 2) {
            cap_edges.emplace_back(clip_a, clip_b);
        }
    }

    // Close the submerged region with waterplane cap triangles.
    // The cap is a planar polygon at z = waterplane_z formed by the clipping
    // edges. We fan-triangulate from the centroid of all clipping points.
    if (!cap_edges.empty()) {
        Eigen::Vector3d cap_centroid = Eigen::Vector3d::Zero();
        int cap_point_count = 0;
        for (const auto& [a, b] : cap_edges) {
            cap_centroid += a + b;
            cap_point_count += 2;
        }
        cap_centroid /= static_cast<double>(cap_point_count);
        cap_centroid.z() = waterplane_z;

        // Each clipping edge forms a cap triangle with the centroid.
        // Winding: cap normal should point upward (+z) to close the volume
        // from below. The clipping edges come from the hull surface where
        // the below-to-above crossing direction is consistent, so we use
        // (centroid, b, a) ordering to get outward (+z) cap normals.
        for (const auto& [a, b] : cap_edges) {
            AccumulateTriangle(cap_centroid, b, a, total_volume, total_moment);
        }
    }

    SubmergedVolumeResult result;
    if (std::abs(total_volume) < kVolumeEpsilon) {
        result.volume = 0.0;
        result.centroid = Eigen::Vector3d::Zero();
    } else {
        result.volume = total_volume;
        result.centroid = total_moment / total_volume;
    }

    return result;
}

}  // namespace seastack::hydro::geometry
