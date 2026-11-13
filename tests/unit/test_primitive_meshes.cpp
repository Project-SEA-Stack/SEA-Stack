// Unit tests for primitive shape meshes (Chrono-free, VSG-free).

#include <seastack/viz_core/primitive_meshes.h>

#include "test_macros.h"

#include <cmath>

using namespace seastack::viz;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Signed enclosed volume [m^3]; positive when all faces wind outward.
double SignedVolume(const TriangleMesh& m) {
    double v = 0.0;
    for (const auto& f : m.faces) {
        const Vec3& a = m.vertices[f[0]];
        const Vec3& b = m.vertices[f[1]];
        const Vec3& c = m.vertices[f[2]];
        v += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
              a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0;
    }
    return v;
}

/// True if every corner normal is unit length and on the same side as its face.
bool NormalsAgreeWithWinding(const TriangleMesh& m) {
    if (m.corner_normals.size() != 3 * m.faces.size()) {
        return false;
    }
    for (size_t f = 0; f < m.faces.size(); ++f) {
        const Vec3& a = m.vertices[m.faces[f][0]];
        const Vec3& b = m.vertices[m.faces[f][1]];
        const Vec3& c = m.vertices[m.faces[f][2]];
        const Vec3 e1{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const Vec3 e2{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        const Vec3 n{e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        for (int k = 0; k < 3; ++k) {
            const Vec3& cn = m.corner_normals[3 * f + k];
            const double len = std::sqrt(cn[0] * cn[0] + cn[1] * cn[1] + cn[2] * cn[2]);
            if (std::abs(len - 1.0) > 1e-9 || cn[0] * n[0] + cn[1] * n[1] + cn[2] * n[2] <= 0.0) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

int main() {
    TestResults test_results;

    const TriangleMesh box = MakeBoxMesh({2.0, 3.0, 4.0});
    TEST_ASSERT(box.faces.size() == 12, "box has 12 triangles");
    TEST_NEAR(SignedVolume(box), 24.0, 1e-9, "box volume, outward winding");
    TEST_ASSERT(NormalsAgreeWithWinding(box), "box normals outward");

    const TriangleMesh sphere = MakeSphereMesh(2.0, 64, 32);
    TEST_NEAR(SignedVolume(sphere), 4.0 / 3.0 * kPi * 8.0, 0.02 * 4.0 / 3.0 * kPi * 8.0, "sphere volume within 2%");
    TEST_ASSERT(NormalsAgreeWithWinding(sphere), "sphere normals outward");

    const TriangleMesh cyl = MakeCylinderMesh(0.5, 3.0, 64);
    TEST_NEAR(SignedVolume(cyl), kPi * 0.25 * 3.0, 0.01 * kPi * 0.25 * 3.0, "cylinder volume within 1%");
    TEST_ASSERT(NormalsAgreeWithWinding(cyl), "cylinder normals outward");
    double zmax = -1e9;
    for (const auto& v : cyl.vertices) {
        zmax = std::max(zmax, v[2]);
    }
    TEST_NEAR(zmax, 1.5, 1e-12, "cylinder axis along z, centred");

    VisualShape s;
    s.kind = ShapeKind::kSphere;
    s.size = {1.0, 0.0, 0.0};
    TEST_ASSERT(!MakeShapeMesh(s).faces.empty(), "dispatch by shape kind");

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
