#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <tuple>
#include <vector>

namespace seastack::viz {

/// Per-corner vertex normals for display, smoothed across edges whose
/// dihedral angle is below `crease_angle_deg` and kept sharp above it, so
/// curved hulls shade smoothly while real edges (deck rims, plate edges)
/// stay crisp. Display only: vertex positions are not changed.
///
/// Coincident vertices (unwelded meshes, common for BEM panel meshes) are
/// treated as one vertex when finding neighbours, using a tolerance relative
/// to the mesh size.
///
/// Output: one normal per triangle corner; corner k of face f uses normal
/// index 3 * f + k. Face orientation (winding) defines the normal side.
struct CornerNormals {
    std::vector<std::array<double, 3>> normals;  ///< 3 * num_faces unit normals
    std::vector<std::array<int, 3>> face_normal_indices;
};

inline CornerNormals ComputeCreaseNormals(const std::vector<std::array<double, 3>>& vertices,
                                          const std::vector<std::array<int, 3>>& faces,
                                          double crease_angle_deg) {
    using V3 = std::array<double, 3>;
    CornerNormals out;
    const size_t nf = faces.size();
    out.normals.resize(3 * nf, V3{0.0, 0.0, 1.0});
    out.face_normal_indices.resize(nf);
    if (nf == 0 || vertices.empty()) {
        return out;
    }

    // Weld coincident vertices: tolerance 1e-6 of the bounding-box diagonal.
    V3 lo = vertices[0], hi = vertices[0];
    for (const auto& v : vertices) {
        for (int i = 0; i < 3; ++i) {
            lo[i] = std::min(lo[i], v[i]);
            hi[i] = std::max(hi[i], v[i]);
        }
    }
    const double diag = std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                                  (hi[2] - lo[2]) * (hi[2] - lo[2]));
    const double tol = std::max(diag * 1e-6, 1e-12);
    std::map<std::tuple<int64_t, int64_t, int64_t>, int> cell_to_id;
    std::vector<int> welded(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto key = std::make_tuple(static_cast<int64_t>(std::llround(vertices[i][0] / tol)),
                                         static_cast<int64_t>(std::llround(vertices[i][1] / tol)),
                                         static_cast<int64_t>(std::llround(vertices[i][2] / tol)));
        auto it = cell_to_id.emplace(key, static_cast<int>(cell_to_id.size())).first;
        welded[i] = it->second;
    }

    // Face normals: unit direction plus area weight (|cross| = 2 x area).
    std::vector<V3> face_unit(nf, V3{0.0, 0.0, 0.0});
    std::vector<double> face_area2(nf, 0.0);
    std::vector<std::vector<int>> faces_at(cell_to_id.size());
    for (size_t f = 0; f < nf; ++f) {
        const auto& a = vertices[static_cast<size_t>(faces[f][0])];
        const auto& b = vertices[static_cast<size_t>(faces[f][1])];
        const auto& c = vertices[static_cast<size_t>(faces[f][2])];
        const V3 e1{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const V3 e2{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        const V3 n{e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        face_area2[f] = len;
        if (len > 0.0) {
            face_unit[f] = {n[0] / len, n[1] / len, n[2] / len};
        }
        for (int k = 0; k < 3; ++k) {
            faces_at[static_cast<size_t>(welded[static_cast<size_t>(faces[f][k])])].push_back(static_cast<int>(f));
        }
    }

    // Corner normal: area-weighted sum of neighbouring faces within the crease angle.
    const double cos_crease = std::cos(crease_angle_deg * 3.14159265358979323846 / 180.0);
    for (size_t f = 0; f < nf; ++f) {
        for (int k = 0; k < 3; ++k) {
            const auto w = static_cast<size_t>(welded[static_cast<size_t>(faces[f][k])]);
            V3 sum{0.0, 0.0, 0.0};
            for (int g : faces_at[w]) {
                const V3& ng = face_unit[static_cast<size_t>(g)];
                const V3& nf_ = face_unit[f];
                const double d = ng[0] * nf_[0] + ng[1] * nf_[1] + ng[2] * nf_[2];
                if (static_cast<size_t>(g) == f || d >= cos_crease) {
                    const double a2 = face_area2[static_cast<size_t>(g)];
                    sum = {sum[0] + a2 * ng[0], sum[1] + a2 * ng[1], sum[2] + a2 * ng[2]};
                }
            }
            const double len = std::sqrt(sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]);
            const size_t idx = 3 * f + static_cast<size_t>(k);
            out.normals[idx] = (len > 0.0) ? V3{sum[0] / len, sum[1] / len, sum[2] / len} : face_unit[f];
            out.face_normal_indices[f][static_cast<size_t>(k)] = static_cast<int>(idx);
        }
    }
    return out;
}

}  // namespace seastack::viz
