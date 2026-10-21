/*********************************************************************
 * @file  test_mesh_shapes.h
 * @brief Closed triangle meshes (box, UV sphere, vertical cylinder) built in
 *        memory for geometry and nonlinear hydrostatics / Froude-Krylov tests.
 *
 * All meshes are watertight with CCW winding and outward normals, matching
 * the TriangleMesh convention. Coordinates in metres, z up.
 *********************************************************************/

#ifndef SEASTACK_TESTS_UNIT_TEST_MESH_SHAPES_H
#define SEASTACK_TESTS_UNIT_TEST_MESH_SHAPES_H

#include <seastack/hydro/geometry/triangle_mesh.h>

#include <Eigen/Dense>

#include <cmath>
#include <vector>

namespace seastack::test {

using seastack::hydro::geometry::TriangleMesh;

/// Axis-aligned box of size (lx, ly, lz) centred at `center` (12 triangles).
inline TriangleMesh MakeBox(double lx, double ly, double lz,
                            const Eigen::Vector3d& center = Eigen::Vector3d::Zero()) {
    TriangleMesh m;
    m.vertices.resize(8, 3);
    for (int i = 0; i < 8; ++i) {
        const double x = (i & 1) ? 0.5 * lx : -0.5 * lx;
        const double y = (i & 2) ? 0.5 * ly : -0.5 * ly;
        const double z = (i & 4) ? 0.5 * lz : -0.5 * lz;
        m.vertices.row(i) = (Eigen::Vector3d(x, y, z) + center).transpose();
    }
    m.faces.resize(12, 3);
    m.faces << 0, 2, 3,  0, 3, 1,   // bottom (-z)
               4, 5, 7,  4, 7, 6,   // top (+z)
               0, 1, 5,  0, 5, 4,   // -y
               2, 6, 7,  2, 7, 3,   // +y
               0, 4, 6,  0, 6, 2,   // -x
               1, 3, 7,  1, 7, 5;   // +x
    return m;
}

/// UV sphere of radius r centred at `center`; n_lat >= 2 bands, n_lon >= 3.
inline TriangleMesh MakeUvSphere(double r, int n_lat, int n_lon,
                                 const Eigen::Vector3d& center = Eigen::Vector3d::Zero()) {
    const double pi = std::acos(-1.0);
    const int n_rings = n_lat - 1;
    TriangleMesh m;
    m.vertices.resize(2 + n_rings * n_lon, 3);
    m.vertices.row(0) = (center + Eigen::Vector3d(0, 0, r)).transpose();
    for (int i = 0; i < n_rings; ++i) {
        const double theta = pi * (i + 1) / n_lat;
        for (int j = 0; j < n_lon; ++j) {
            const double phi = 2.0 * pi * j / n_lon;
            m.vertices.row(1 + i * n_lon + j) =
                (center + r * Eigen::Vector3d(std::sin(theta) * std::cos(phi),
                                              std::sin(theta) * std::sin(phi),
                                              std::cos(theta))).transpose();
        }
    }
    const int south = 1 + n_rings * n_lon;
    m.vertices.row(south) = (center + Eigen::Vector3d(0, 0, -r)).transpose();

    auto ring = [n_lon](int i, int j) { return 1 + i * n_lon + (j % n_lon); };
    std::vector<Eigen::Vector3i> faces;
    for (int j = 0; j < n_lon; ++j) faces.emplace_back(0, ring(0, j), ring(0, j + 1));
    for (int i = 0; i + 1 < n_rings; ++i) {
        for (int j = 0; j < n_lon; ++j) {
            const int a = ring(i, j), b = ring(i, j + 1);
            const int c = ring(i + 1, j + 1), d = ring(i + 1, j);
            faces.emplace_back(a, d, c);
            faces.emplace_back(a, c, b);
        }
    }
    for (int j = 0; j < n_lon; ++j)
        faces.emplace_back(ring(n_rings - 1, j), south, ring(n_rings - 1, j + 1));

    m.faces.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f)
        m.faces.row(static_cast<Eigen::Index>(f)) = faces[f].transpose();
    return m;
}

/// Closed vertical cylinder of radius r between z_bottom and z_top (axis
/// through x = y = 0), n_theta segments and n_z side bands, flat end caps.
inline TriangleMesh MakeVerticalCylinder(double r, double z_bottom, double z_top,
                                         int n_theta, int n_z) {
    const double pi = std::acos(-1.0);
    const int n_rings = n_z + 1;
    TriangleMesh m;
    m.vertices.resize(n_rings * n_theta + 2, 3);
    for (int i = 0; i < n_rings; ++i) {
        const double z = z_top + (z_bottom - z_top) * i / n_z;  // ring 0 at top
        for (int j = 0; j < n_theta; ++j) {
            const double phi = 2.0 * pi * j / n_theta;
            m.vertices.row(i * n_theta + j) << r * std::cos(phi), r * std::sin(phi), z;
        }
    }
    const int c_top = n_rings * n_theta;
    const int c_bot = c_top + 1;
    m.vertices.row(c_top) << 0.0, 0.0, z_top;
    m.vertices.row(c_bot) << 0.0, 0.0, z_bottom;

    auto ring = [n_theta](int i, int j) { return i * n_theta + (j % n_theta); };
    std::vector<Eigen::Vector3i> faces;
    for (int j = 0; j < n_theta; ++j) faces.emplace_back(c_top, ring(0, j), ring(0, j + 1));
    for (int i = 0; i < n_z; ++i) {
        for (int j = 0; j < n_theta; ++j) {
            const int a = ring(i, j), b = ring(i, j + 1);
            const int c = ring(i + 1, j + 1), d = ring(i + 1, j);
            faces.emplace_back(a, d, c);
            faces.emplace_back(a, c, b);
        }
    }
    for (int j = 0; j < n_theta; ++j) faces.emplace_back(c_bot, ring(n_z, j + 1), ring(n_z, j));

    m.faces.resize(static_cast<Eigen::Index>(faces.size()), 3);
    for (std::size_t f = 0; f < faces.size(); ++f)
        m.faces.row(static_cast<Eigen::Index>(f)) = faces[f].transpose();
    return m;
}

/// Cyclically rotate every face's vertex order by `shift` (0, 1 or 2).
/// Geometry and winding are unchanged; only which vertex is listed first.
inline TriangleMesh RotateFaceIndices(TriangleMesh m, int shift) {
    for (Eigen::Index f = 0; f < m.faces.rows(); ++f) {
        const Eigen::Vector3i v = m.faces.row(f).transpose();
        for (int k = 0; k < 3; ++k) m.faces(f, k) = v[(k + shift) % 3];
    }
    return m;
}

}  // namespace seastack::test

#endif  // SEASTACK_TESTS_UNIT_TEST_MESH_SHAPES_H
