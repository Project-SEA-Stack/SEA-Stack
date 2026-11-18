#pragma once

#include <seastack/viz_core/scene_model.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace seastack::viz {

/// Mouse picking: which body a ray from the camera hits first. Exact for
/// boxes, spheres, cylinders and triangle meshes (VisualShape kinds), in
/// each shape's own frame. World frame, SI units.

struct Ray {
    Vec3 origin{0.0, 0.0, 0.0};     ///< [m]
    Vec3 direction{0.0, 0.0, 1.0};  ///< Unit vector
};

inline constexpr double kNoHit = std::numeric_limits<double>::infinity();

/// Ray through a point of the image of a perspective camera at `eye` looking
/// at `target` with `up`, vertical field of view fov_deg and aspect
/// width / height. ndc_x in [-1, 1] runs left to right and ndc_y in [-1, 1]
/// top to bottom (window pixel convention).
inline Ray CameraRay(const Vec3& eye, const Vec3& target, const Vec3& up, double fov_deg, double aspect,
                     double ndc_x, double ndc_y) {
    auto sub = [](const Vec3& a, const Vec3& b) { return Vec3{a[0] - b[0], a[1] - b[1], a[2] - b[2]}; };
    auto cross = [](const Vec3& a, const Vec3& b) {
        return Vec3{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    };
    auto normalize = [](Vec3 v) {
        const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n > 0.0) {
            for (double& c : v) c /= n;
        }
        return v;
    };
    const Vec3 forward = normalize(sub(target, eye));
    const Vec3 right = normalize(cross(forward, up));
    const Vec3 true_up = cross(right, forward);
    const double tan_half = std::tan(0.5 * fov_deg * 3.14159265358979323846 / 180.0);
    const double sx = ndc_x * tan_half * aspect;
    const double sy = -ndc_y * tan_half;
    Ray ray;
    ray.origin = eye;
    ray.direction = normalize({forward[0] + sx * right[0] + sy * true_up[0], forward[1] + sx * right[1] + sy * true_up[1],
                               forward[2] + sx * right[2] + sy * true_up[2]});
    return ray;
}

namespace picking_detail {

inline double Dot(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

/// Ray in the local frame of `pose` (rigid: distances along the ray are kept).
inline Ray ToLocal(const Pose& pose, const Ray& ray) {
    const Pose inv = Inverse(pose);
    Ray out;
    out.origin = Transform(inv, ray.origin);
    out.direction = Rotate(inv.rotation, ray.direction);
    return out;
}

/// Slab test against the box [lo, hi]; nearest t >= 0 or kNoHit.
inline double HitBox(const Ray& r, const Vec3& lo, const Vec3& hi) {
    double t_min = 0.0;
    double t_max = kNoHit;
    for (int k = 0; k < 3; ++k) {
        if (std::abs(r.direction[k]) < 1e-300) {
            if (r.origin[k] < lo[k] || r.origin[k] > hi[k]) return kNoHit;
            continue;
        }
        double t1 = (lo[k] - r.origin[k]) / r.direction[k];
        double t2 = (hi[k] - r.origin[k]) / r.direction[k];
        if (t1 > t2) std::swap(t1, t2);
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
        if (t_min > t_max) return kNoHit;
    }
    return t_min;
}

/// Smallest root t >= 0 of a t^2 + b t + c = 0, or kNoHit.
inline double SmallestRoot(double a, double b, double c) {
    if (a <= 0.0) return kNoHit;
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) return kNoHit;
    const double s = std::sqrt(disc);
    const double t1 = (-b - s) / (2.0 * a);
    const double t2 = (-b + s) / (2.0 * a);
    if (t1 >= 0.0) return t1;
    return t2 >= 0.0 ? t2 : kNoHit;
}

inline double HitSphere(const Ray& r, double radius) {
    return SmallestRoot(1.0, 2.0 * Dot(r.origin, r.direction), Dot(r.origin, r.origin) - radius * radius);
}

/// Solid cylinder of `radius` along local z, z in [-h, h].
inline double HitCylinder(const Ray& r, double radius, double h) {
    double best = kNoHit;
    const double a = r.direction[0] * r.direction[0] + r.direction[1] * r.direction[1];
    const double b = 2.0 * (r.origin[0] * r.direction[0] + r.origin[1] * r.direction[1]);
    const double c = r.origin[0] * r.origin[0] + r.origin[1] * r.origin[1] - radius * radius;
    if (c <= 0.0 && std::abs(r.origin[2]) <= h) return 0.0;  // starts inside
    const double t_side = SmallestRoot(a, b, c);
    if (t_side < kNoHit && std::abs(r.origin[2] + t_side * r.direction[2]) <= h) best = t_side;
    if (std::abs(r.direction[2]) > 1e-300) {
        for (double z : {-h, h}) {
            const double t = (z - r.origin[2]) / r.direction[2];
            const double x = r.origin[0] + t * r.direction[0];
            const double y = r.origin[1] + t * r.direction[1];
            if (t >= 0.0 && x * x + y * y <= radius * radius) best = std::min(best, t);
        }
    }
    return best;
}

/// Moller-Trumbore, both faces; t >= 0 or kNoHit.
inline double HitTriangle(const Ray& r, const Vec3& p0, const Vec3& p1, const Vec3& p2) {
    const Vec3 e1{p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
    const Vec3 e2{p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]};
    const Vec3& d = r.direction;
    const Vec3 p{d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0]};
    const double det = Dot(e1, p);
    if (std::abs(det) < 1e-300) return kNoHit;
    const double inv = 1.0 / det;
    const Vec3 s{r.origin[0] - p0[0], r.origin[1] - p0[1], r.origin[2] - p0[2]};
    const double u = Dot(s, p) * inv;
    if (u < 0.0 || u > 1.0) return kNoHit;
    const Vec3 q{s[1] * e1[2] - s[2] * e1[1], s[2] * e1[0] - s[0] * e1[2], s[0] * e1[1] - s[1] * e1[0]};
    const double v = Dot(d, q) * inv;
    if (v < 0.0 || u + v > 1.0) return kNoHit;
    const double t = Dot(e2, q) * inv;
    return t >= 0.0 ? t : kNoHit;
}

inline double HitMesh(const Ray& r, const TriangleMesh& mesh) {
    if (mesh.vertices.empty()) return kNoHit;
    Vec3 lo = mesh.vertices.front();
    Vec3 hi = lo;
    for (const Vec3& v : mesh.vertices) {
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], v[k]);
            hi[k] = std::max(hi[k], v[k]);
        }
    }
    if (HitBox(r, lo, hi) == kNoHit) return kNoHit;
    double best = kNoHit;
    const int n = static_cast<int>(mesh.vertices.size());
    for (const auto& f : mesh.faces) {
        if (f[0] < 0 || f[1] < 0 || f[2] < 0 || f[0] >= n || f[1] >= n || f[2] >= n) continue;
        best = std::min(best, HitTriangle(r, mesh.vertices[f[0]], mesh.vertices[f[1]], mesh.vertices[f[2]]));
    }
    return best;
}

}  // namespace picking_detail

/// Distance [m] along `ray` (world) to `shape` on a body posed at
/// `body_pose`, or kNoHit.
inline double IntersectShape(const VisualShape& shape, const Pose& body_pose, const Ray& ray) {
    using namespace picking_detail;
    const Ray r = ToLocal(Compose(body_pose, shape.local), ray);
    switch (shape.kind) {
        case ShapeKind::kBox: {
            const Vec3 h{0.5 * shape.size[0], 0.5 * shape.size[1], 0.5 * shape.size[2]};
            return HitBox(r, {-h[0], -h[1], -h[2]}, h);
        }
        case ShapeKind::kSphere:
            return HitSphere(r, shape.size[0]);
        case ShapeKind::kCylinder:
            return HitCylinder(r, shape.size[0], 0.5 * shape.size[1]);
        case ShapeKind::kMesh:
            return shape.mesh ? HitMesh(r, *shape.mesh) : kNoHit;
    }
    return kNoHit;
}

/// Index of the body `ray` hits first, or -1. Bodies with visible[i] == 0
/// are skipped (an empty `visible` means all are visible).
inline int PickBody(const SceneDescription& scene, const VizFrame& frame, const Ray& ray,
                    const std::vector<char>& visible = {}) {
    int best_body = -1;
    double best_t = kNoHit;
    for (size_t b = 0; b < scene.bodies.size() && b < frame.bodies.size(); ++b) {
        if (b < visible.size() && !visible[b]) continue;
        for (const VisualShape& shape : scene.bodies[b].shapes) {
            const double t = IntersectShape(shape, frame.bodies[b].pose, ray);
            if (t < best_t) {
                best_t = t;
                best_body = static_cast<int>(b);
            }
        }
    }
    return best_body;
}

}  // namespace seastack::viz
