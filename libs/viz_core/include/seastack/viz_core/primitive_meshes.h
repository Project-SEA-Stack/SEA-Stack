#pragma once

#include <seastack/viz_core/scene_model.h>

#include <cmath>

namespace seastack::viz {

/// Triangle meshes for the primitive shapes of a VisualShape, centred on the
/// shape frame origin, with outward per-corner normals and counter-clockwise
/// winding seen from outside. Sizes follow ShapeKind (full box lengths,
/// sphere radius, cylinder radius and length along local z), all in metres.

namespace detail {

inline void AddTriangle(TriangleMesh* m, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& na,
                        const Vec3& nb, const Vec3& nc) {
    const int i = static_cast<int>(m->vertices.size());
    m->vertices.push_back(a);
    m->vertices.push_back(b);
    m->vertices.push_back(c);
    m->faces.push_back({i, i + 1, i + 2});
    m->corner_normals.push_back(na);
    m->corner_normals.push_back(nb);
    m->corner_normals.push_back(nc);
}

constexpr double kTwoPi = 6.28318530717958647692;

}  // namespace detail

/// Box with full side lengths (lx, ly, lz) [m]; flat faces.
inline TriangleMesh MakeBoxMesh(const Vec3& lengths) {
    const double hx = 0.5 * lengths[0], hy = 0.5 * lengths[1], hz = 0.5 * lengths[2];
    TriangleMesh m;
    // Each face: outward normal n and two in-plane axes u, v with u x v = n.
    struct Face {
        Vec3 n, u, v;
        double dn, du, dv;
    };
    const Face faces[6] = {
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, hx, hy, hz},  {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}, hx, hz, hy},
        {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}, hy, hz, hx},  {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}, hy, hx, hz},
        {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}, hz, hx, hy},  {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}, hz, hy, hx},
    };
    for (const Face& f : faces) {
        Vec3 c[4];
        const double su[4] = {-1, 1, 1, -1};
        const double sv[4] = {-1, -1, 1, 1};
        for (int k = 0; k < 4; ++k) {
            for (int i = 0; i < 3; ++i) {
                c[k][i] = f.n[i] * f.dn + f.u[i] * su[k] * f.du + f.v[i] * sv[k] * f.dv;
            }
        }
        detail::AddTriangle(&m, c[0], c[1], c[2], f.n, f.n, f.n);
        detail::AddTriangle(&m, c[0], c[2], c[3], f.n, f.n, f.n);
    }
    return m;
}

/// UV sphere of the given radius [m]; smooth normals.
inline TriangleMesh MakeSphereMesh(double radius, int segments = 32, int rings = 16) {
    TriangleMesh m;
    auto point = [&](int s, int r) {
        const double theta = detail::kTwoPi * s / segments;      // azimuth
        const double phi = 0.5 * detail::kTwoPi * r / rings;     // from +z pole
        return Vec3{std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta), std::cos(phi)};
    };
    auto scaled = [radius](const Vec3& n) { return Vec3{radius * n[0], radius * n[1], radius * n[2]}; };
    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < segments; ++s) {
            const Vec3 a = point(s, r), b = point(s, r + 1), c = point(s + 1, r + 1), d = point(s + 1, r);
            if (r > 0) {
                detail::AddTriangle(&m, scaled(a), scaled(b), scaled(d), a, b, d);
            }
            if (r < rings - 1) {
                detail::AddTriangle(&m, scaled(b), scaled(c), scaled(d), b, c, d);
            }
        }
    }
    return m;
}

/// Cylinder along local z, centred at the origin; smooth sides, flat caps.
inline TriangleMesh MakeCylinderMesh(double radius, double length, int segments = 32) {
    TriangleMesh m;
    const double hz = 0.5 * length;
    const Vec3 up{0, 0, 1}, down{0, 0, -1};
    for (int s = 0; s < segments; ++s) {
        const double t0 = detail::kTwoPi * s / segments;
        const double t1 = detail::kTwoPi * (s + 1) / segments;
        const Vec3 n0{std::cos(t0), std::sin(t0), 0.0};
        const Vec3 n1{std::cos(t1), std::sin(t1), 0.0};
        const Vec3 b0{radius * n0[0], radius * n0[1], -hz}, b1{radius * n1[0], radius * n1[1], -hz};
        const Vec3 t0p{b0[0], b0[1], hz}, t1p{b1[0], b1[1], hz};
        detail::AddTriangle(&m, b0, b1, t1p, n0, n1, n1);
        detail::AddTriangle(&m, b0, t1p, t0p, n0, n1, n0);
        detail::AddTriangle(&m, {0, 0, hz}, t0p, t1p, up, up, up);
        detail::AddTriangle(&m, {0, 0, -hz}, b1, b0, down, down, down);
    }
    return m;
}

/// Mesh for any primitive shape kind (kMesh returns the shape's own mesh copy).
inline TriangleMesh MakeShapeMesh(const VisualShape& shape) {
    switch (shape.kind) {
        case ShapeKind::kBox:
            return MakeBoxMesh(shape.size);
        case ShapeKind::kSphere:
            return MakeSphereMesh(shape.size[0]);
        case ShapeKind::kCylinder:
            return MakeCylinderMesh(shape.size[0], shape.size[1]);
        case ShapeKind::kMesh:
        default:
            return shape.mesh ? *shape.mesh : TriangleMesh{};
    }
}

}  // namespace seastack::viz
