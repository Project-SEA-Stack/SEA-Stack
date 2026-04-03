/*********************************************************************
 * @file  triangle_mesh.h
 * @brief Lightweight Eigen-based triangle mesh type with topology analysis.
 *
 * MAIN TYPES:
 *   - TriangleMesh: vertices (Nx3) + faces (Mx3) with topology diagnostics
 *   - TopologyInfo: closure check results (boundary edges, Euler characteristic)
 *
 * Mesh convention: vertices in body-local frame, faces use CCW winding
 * with outward-pointing normals.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_GEOMETRY_TRIANGLE_MESH_H
#define SEASTACK_HYDRO_GEOMETRY_TRIANGLE_MESH_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace seastack::hydro::geometry {

/**
 * @brief Lightweight triangle mesh for hydrostatic geometry computations.
 */
struct TriangleMesh {
    Eigen::MatrixXd vertices;   ///< Nx3, each row = vertex (x,y,z) in body frame
    Eigen::MatrixXi faces;      ///< Mx3, each row = 3 vertex indices (CCW outward normals)

    int num_vertices() const { return static_cast<int>(vertices.rows()); }
    int num_faces() const { return static_cast<int>(faces.rows()); }

    Eigen::AlignedBox3d bounding_box() const;

    /**
     * @brief Mesh topology diagnostics.
     */
    struct TopologyInfo {
        bool is_closed;             ///< True if no boundary edges (manifold, V - E + F = 2)
        int num_boundary_edges;     ///< 0 for closed meshes
        double boundary_z_mean;     ///< Mean z of boundary-edge vertices (useful for open meshes)
        double boundary_z_stddev;   ///< Spread of boundary-edge vertex z-values
    };

    /**
     * @brief Analyze mesh topology for closure/watertightness.
     *
     * Builds a half-edge structure to detect boundary edges (edges shared by
     * only one face). For a closed mesh, num_boundary_edges == 0.
     *
     * @return TopologyInfo with closure status and boundary diagnostics
     */
    TopologyInfo AnalyzeTopology() const;
};

}  // namespace seastack::hydro::geometry

#endif  // SEASTACK_HYDRO_GEOMETRY_TRIANGLE_MESH_H
