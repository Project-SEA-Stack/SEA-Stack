/*********************************************************************
 * @file  wetted_surface_pressure.cpp
 * @brief WettedSurfacePressureIntegrator implementation.
 *
 * Clipping follows MeshSubmergedVolume (walk the face edges in winding
 * order, keep wet vertices, insert a point where an edge changes sign), but
 * on d = z - eta instead of z. MeshSubmergedVolume is left untouched so
 * nonlinear hydrostatics results stay bit-identical.
 *********************************************************************/

#include <seastack/hydro/geometry/wetted_surface_pressure.h>

#include <array>
#include <sstream>
#include <stdexcept>
#include <string>

namespace seastack::hydro::geometry {

WettedSurfacePressureIntegrator::WettedSurfacePressureIntegrator(TriangleMesh mesh)
    : mesh_(std::move(mesh)) {
    const auto topo = mesh_.AnalyzeTopology();
    if (!topo.is_closed) {
        std::ostringstream oss;
        oss << "Mesh has " << topo.num_boundary_edges
            << " boundary edges (mean boundary z = " << topo.boundary_z_mean << "). "
            << "Nonlinear Froude-Krylov requires a closed (watertight) hull mesh, "
            << "because the wetted surface moves with the waves.";
        throw std::runtime_error(oss.str());
    }
}

Eigen::Matrix3Xd WettedSurfacePressureIntegrator::TransformVertices(
    const Eigen::Affine3d& body_to_world) const {
    Eigen::Matrix3Xd world;
    TransformVertices(body_to_world, world);
    return world;
}

void WettedSurfacePressureIntegrator::TransformVertices(const Eigen::Affine3d& body_to_world,
                                                        Eigen::Matrix3Xd& world) const {
    const int nv = mesh_.num_vertices();
    world.resize(3, nv);
    for (int i = 0; i < nv; ++i) {
        world.col(i) = body_to_world * Eigen::Vector3d(mesh_.vertices.row(i).transpose());
    }
}

namespace {

struct PolygonPoint {
    Eigen::Vector3d r;  // position relative to the moment reference point [m]
    double p;           // pressure [Pa]
};

/// Force, moment and area of one triangle with linear pressure (positions relative to r_ref).
void AccumulateTriangle(const PolygonPoint& a, const PolygonPoint& b, const PolygonPoint& c,
                        PressureIntegral& out) {
    const Eigen::Vector3d area_vector = 0.5 * (b.r - a.r).cross(c.r - a.r);  // A n
    const double p_sum = a.p + b.p + c.p;
    out.force -= area_vector * (p_sum / 3.0);
    const Eigen::Vector3d first_moment = (a.r * (a.p + p_sum) + b.r * (b.p + p_sum) + c.r * (c.p + p_sum)) / 12.0;
    out.moment -= first_moment.cross(area_vector);
    out.wetted_area += area_vector.norm();
}

}  // namespace

PressureIntegral WettedSurfacePressureIntegrator::Integrate(
    const Eigen::Matrix3Xd& world_vertices,
    const Eigen::VectorXd& height_above_surface,
    const Eigen::VectorXd& vertex_pressure,
    const Eigen::VectorXd& surface_pressure,
    const Eigen::Vector3d& reference_point) const {
    const int nv = mesh_.num_vertices();
    if (world_vertices.cols() != nv || height_above_surface.size() != nv ||
        vertex_pressure.size() != nv || surface_pressure.size() != nv) {
        throw std::invalid_argument("WettedSurfacePressureIntegrator::Integrate: per-vertex arrays must have " +
                                    std::to_string(nv) + " entries");
    }

    PressureIntegral result;
    std::array<PolygonPoint, 4> poly;  // a clipped triangle has at most 4 wet points

    for (int f = 0; f < mesh_.num_faces(); ++f) {
        const std::array<int, 3> idx = {mesh_.faces(f, 0), mesh_.faces(f, 1), mesh_.faces(f, 2)};
        const std::array<bool, 3> wet = {height_above_surface[idx[0]] <= 0.0,
                                         height_above_surface[idx[1]] <= 0.0,
                                         height_above_surface[idx[2]] <= 0.0};
        const int wet_count = static_cast<int>(wet[0]) + static_cast<int>(wet[1]) + static_cast<int>(wet[2]);
        if (wet_count == 0) continue;

        int n = 0;
        for (int e = 0; e < 3; ++e) {
            const int i = idx[e];
            const int j = idx[(e + 1) % 3];
            if (wet[e]) {
                poly[n++] = {world_vertices.col(i) - reference_point, vertex_pressure[i]};
            }
            if (wet[e] != wet[(e + 1) % 3]) {
                // Point on edge i-j where d = 0, measured from the wet end.
                const int lo = wet[e] ? i : j;
                const int hi = wet[e] ? j : i;
                const double t = height_above_surface[lo] / (height_above_surface[lo] - height_above_surface[hi]);
                const Eigen::Vector3d r = world_vertices.col(lo) + t * (world_vertices.col(hi) - world_vertices.col(lo));
                const double p = surface_pressure[lo] + t * (surface_pressure[hi] - surface_pressure[lo]);
                poly[n++] = {r - reference_point, p};
            }
        }

        for (int k = 1; k + 1 < n; ++k) {
            AccumulateTriangle(poly[0], poly[k], poly[k + 1], result);
        }
    }
    return result;
}

}  // namespace seastack::hydro::geometry
