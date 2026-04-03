/*********************************************************************
 * @file  triangle_mesh.cpp
 * @brief TriangleMesh topology analysis implementation.
 *********************************************************************/

#include <seastack/hydro/geometry/triangle_mesh.h>

#include <cmath>
#include <map>
#include <utility>

namespace seastack::hydro::geometry {

Eigen::AlignedBox3d TriangleMesh::bounding_box() const {
    Eigen::AlignedBox3d box;
    for (int i = 0; i < num_vertices(); ++i) {
        box.extend(vertices.row(i).transpose());
    }
    return box;
}

TriangleMesh::TopologyInfo TriangleMesh::AnalyzeTopology() const {
    TopologyInfo info{};
    info.is_closed = false;
    info.num_boundary_edges = 0;
    info.boundary_z_mean = 0.0;
    info.boundary_z_stddev = 0.0;

    if (num_faces() == 0 || num_vertices() == 0) {
        return info;
    }

    // Build edge-face adjacency using ordered edge pairs.
    // An edge (a,b) with a < b that appears in only one face is a boundary edge.
    using Edge = std::pair<int, int>;
    std::map<Edge, int> edge_count;

    for (int f = 0; f < num_faces(); ++f) {
        for (int e = 0; e < 3; ++e) {
            int v0 = faces(f, e);
            int v1 = faces(f, (e + 1) % 3);
            Edge edge = (v0 < v1) ? Edge{v0, v1} : Edge{v1, v0};
            edge_count[edge]++;
        }
    }

    // Collect boundary edges and their vertex z-coordinates
    std::vector<double> boundary_z_values;
    for (const auto& [edge, count] : edge_count) {
        if (count == 1) {
            info.num_boundary_edges++;
            boundary_z_values.push_back(vertices(edge.first, 2));
            boundary_z_values.push_back(vertices(edge.second, 2));
        }
    }

    info.is_closed = (info.num_boundary_edges == 0);

    if (!boundary_z_values.empty()) {
        double sum = 0.0;
        for (double z : boundary_z_values) sum += z;
        info.boundary_z_mean = sum / static_cast<double>(boundary_z_values.size());

        double var = 0.0;
        for (double z : boundary_z_values) {
            double d = z - info.boundary_z_mean;
            var += d * d;
        }
        info.boundary_z_stddev = std::sqrt(var / static_cast<double>(boundary_z_values.size()));
    }

    return info;
}

}  // namespace seastack::hydro::geometry
