/*********************************************************************
 * @file  wetted_surface_pressure.h
 * @brief WettedSurfacePressureIntegrator: pressure force and moment on the
 *        part of a closed triangle mesh below a (possibly wavy) free surface.
 *
 * Used by nonlinear Froude-Krylov. Theory and pass criteria:
 * docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 8.
 *
 * The caller supplies per-vertex values in the world frame; the integrator
 * knows nothing about waves. That keeps it testable against analytic
 * surfaces and reusable for the linear Froude-Krylov coefficients.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_GEOMETRY_WETTED_SURFACE_PRESSURE_H
#define SEASTACK_HYDRO_GEOMETRY_WETTED_SURFACE_PRESSURE_H

#include <seastack/hydro/geometry/triangle_mesh.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace seastack::hydro::geometry {

/// Pressure resultant on the wetted surface (world axes).
struct PressureIntegral {
    Eigen::Vector3d force  = Eigen::Vector3d::Zero();  ///< -int p n dS [N]
    Eigen::Vector3d moment = Eigen::Vector3d::Zero();  ///< -int p (r - r_ref) x n dS [N m]
    double wetted_area     = 0.0;                      ///< Area of the wet surface [m^2]
};

/**
 * @brief Integrates a vertex pressure field over the wet part of a closed mesh.
 *
 * Wet/dry split: a vertex is wet when its height above the free surface
 * d_v = z_v - eta(x_v, y_v) is <= 0. Triangles with mixed signs are clipped
 * where d, interpolated linearly along each edge, is zero.
 *
 * Pressure: linear over each wet (sub-)triangle. Wet vertices carry the
 * supplied vertex pressure; each clip point carries the supplied surface
 * value interpolated linearly along its edge (zero for the total pressure,
 * which vanishes on the free surface). Per sub-triangle T with area vector
 * S = A n and vertex pressures p_j:
 *   F_T = -S (p_0 + p_1 + p_2) / 3
 *   M_T = -(1/12) sum_j (r_j - r_ref)(p_j + p_0 + p_1 + p_2) x S
 * These are exact for a linear pressure, so the hydrostatic part -rho g z is
 * integrated exactly.
 *
 * No waterplane cap is added: the pressure is zero on the free surface.
 */
class WettedSurfacePressureIntegrator {
  public:
    /// @param mesh  Closed triangle mesh, outward normals (CCW faces).
    /// @throws std::runtime_error if the mesh has boundary edges.
    explicit WettedSurfacePressureIntegrator(TriangleMesh mesh);

    const TriangleMesh& GetMesh() const { return mesh_; }
    int NumVertices() const { return mesh_.num_vertices(); }

    /// Vertex positions in the world frame, one per column.
    Eigen::Matrix3Xd TransformVertices(const Eigen::Affine3d& body_to_world) const;

    /**
     * @param world_vertices    3 x N vertex positions, world frame [m].
     * @param height_above_surface  d_v = z_v - eta_v [m]; wet where d_v <= 0.
     * @param vertex_pressure   Pressure at each vertex [Pa]; read only for wet vertices.
     * @param surface_pressure  Value at the free surface [Pa], interpolated along a
     *                          cut edge to give the clip-point pressure. Pass zeros
     *                          for the total pressure.
     * @param reference_point   Moment reference point, world frame [m].
     */
    PressureIntegral Integrate(const Eigen::Matrix3Xd& world_vertices,
                               const Eigen::VectorXd& height_above_surface,
                               const Eigen::VectorXd& vertex_pressure,
                               const Eigen::VectorXd& surface_pressure,
                               const Eigen::Vector3d& reference_point) const;

  private:
    TriangleMesh mesh_;
};

}  // namespace seastack::hydro::geometry

#endif  // SEASTACK_HYDRO_GEOMETRY_WETTED_SURFACE_PRESSURE_H
