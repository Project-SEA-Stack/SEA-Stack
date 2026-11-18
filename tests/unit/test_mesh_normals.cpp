// Unit tests for crease-angle display normals (Chrono-free, VSG-free).

#include <seastack/viz_core/mesh_normals.h>

#include "test_macros.h"

#include <cmath>

using seastack::viz::ComputeCreaseNormals;
using V3 = std::array<double, 3>;
using F3 = std::array<int, 3>;

namespace {

double Dot(const V3& a, const V3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

}  // namespace

int main() {
    TestResults test_results;

    // Two coplanar triangles: every corner normal is +Z.
    {
        const std::vector<V3> v{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
        const std::vector<F3> f{{0, 1, 2}, {0, 2, 3}};
        const auto out = ComputeCreaseNormals(v, f, 40.0);
        TEST_ASSERT(out.normals.size() == 6, "one normal per corner");
        bool all_up = true;
        for (const auto& n : out.normals) {
            all_up = all_up && std::abs(n[2] - 1.0) < 1e-12;
        }
        TEST_ASSERT(all_up, "flat quad keeps +Z normals");
        TEST_ASSERT(out.face_normal_indices[1][2] == 5, "corner k of face f uses index 3f+k");
    }

    // Shallow fold (20 deg between faces, below a 40 deg crease): shared edge
    // normals are averaged; the shared corners of both faces agree.
    {
        const double a = 20.0 * 3.14159265358979323846 / 180.0;
        const std::vector<V3> v{{0, 0, 0}, {0, 1, 0}, {1, 0, 0}, {-std::cos(a), 0, std::sin(a)}};
        const std::vector<F3> f{{0, 2, 1}, {0, 1, 3}};
        const auto out = ComputeCreaseNormals(v, f, 40.0);
        const V3& n_face0 = out.normals[0];  // vertex 0 in face 0
        const V3& n_face1 = out.normals[3];  // vertex 0 in face 1
        TEST_NEAR(Dot(n_face0, n_face1), 1.0, 1e-12, "smoothed edge: both faces share the corner normal");
        TEST_NEAR(std::sqrt(Dot(n_face0, n_face0)), 1.0, 1e-12, "normals are unit length");
    }

    // Right-angle edge (90 deg > 40 deg crease): stays sharp.
    {
        const std::vector<V3> v{{0, 0, 0}, {0, 1, 0}, {1, 0, 0}, {0, 0, 1}};
        const std::vector<F3> f{{0, 2, 1}, {0, 1, 3}};
        const auto out = ComputeCreaseNormals(v, f, 40.0);
        TEST_NEAR(Dot(out.normals[0], out.normals[3]), 0.0, 1e-12, "crease edge keeps face normals");
    }

    // Unwelded mesh (duplicate vertices along the shared edge) smooths the same way.
    {
        const double a = 20.0 * 3.14159265358979323846 / 180.0;
        const std::vector<V3> v{{0, 0, 0}, {0, 1, 0}, {1, 0, 0}, {0, 0, 0}, {0, 1, 0}, {-std::cos(a), 0, std::sin(a)}};
        const std::vector<F3> f{{0, 2, 1}, {3, 4, 5}};
        const auto out = ComputeCreaseNormals(v, f, 40.0);
        TEST_NEAR(Dot(out.normals[0], out.normals[3]), 1.0, 1e-12, "coincident vertices are welded for smoothing");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
