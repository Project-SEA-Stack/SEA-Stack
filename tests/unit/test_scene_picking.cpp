/*********************************************************************
 * @file  test_scene_picking.cpp
 * @brief Unit tests for mouse picking in the SEA-Stack viewer: camera rays
 *        and ray / shape intersection.
 *
 * Pass criteria:
 *   P1  CameraRay: the image centre looks at the target; the image edges are
 *       half the field of view away; top of the image is +up.
 *   P2  Exact hit distances for box, sphere, cylinder (side and cap) and a
 *       mesh triangle; misses return kNoHit; shape offsets and body poses
 *       (rotation + translation) are applied.
 *   P3  PickBody returns the nearest body and skips hidden ones.
 *
 * Self-contained: viz_core only.
 *********************************************************************/

#include <seastack/viz_core/scene_picking.h>

#include "test_macros.h"

#include <cmath>
#include <memory>
#include <vector>

using namespace seastack::viz;

namespace {

constexpr double kPi = 3.14159265358979323846;

VisualShape Shape(ShapeKind kind, Vec3 size, Vec3 offset = {0.0, 0.0, 0.0}) {
    VisualShape s;
    s.kind = kind;
    s.size = size;
    s.local.position = offset;
    return s;
}

Ray RayFrom(Vec3 origin, Vec3 direction) {
    Ray r;
    r.origin = origin;
    const double n = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    r.direction = {direction[0] / n, direction[1] / n, direction[2] / n};
    return r;
}

/// Rotation by angle [rad] about z.
Quat AboutZ(double angle) { return {std::cos(0.5 * angle), 0.0, 0.0, std::sin(0.5 * angle)}; }

}  // namespace

int main() {
    TestResults test_results;

    // ---- P1. Camera rays ----
    {
        const Vec3 eye{0.0, -10.0, 0.0}, target{0.0, 0.0, 0.0}, up{0.0, 0.0, 1.0};
        const double fov = 40.0, aspect = 2.0;
        const Ray centre = CameraRay(eye, target, up, fov, aspect, 0.0, 0.0);
        TEST_NEAR(centre.direction[1], 1.0, 1e-15, "centre looks at the target");
        const Ray top = CameraRay(eye, target, up, fov, aspect, 0.0, -1.0);
        TEST_NEAR(std::atan2(top.direction[2], top.direction[1]), 0.5 * fov * kPi / 180.0, 1e-12,
                  "top edge: +half the vertical field of view, towards +up");
        const Ray right = CameraRay(eye, target, up, fov, aspect, 1.0, 0.0);
        TEST_NEAR(right.direction[0] / right.direction[1], aspect * std::tan(0.5 * fov * kPi / 180.0), 1e-12,
                  "right edge: aspect x tan(half fov), towards +x (right of the view)");
        TEST_ASSERT(right.direction[0] > 0.0, "looking along +y with z up, the right is +x");
    }

    // ---- P2. Shapes ----
    {
        const Pose identity;
        const Ray down = RayFrom({0.0, 0.0, 10.0}, {0.0, 0.0, -1.0});
        TEST_NEAR(IntersectShape(Shape(ShapeKind::kBox, {2.0, 2.0, 4.0}), identity, down), 8.0, 1e-12,
                  "box: top face at z = 2");
        TEST_NEAR(IntersectShape(Shape(ShapeKind::kSphere, {3.0, 0.0, 0.0}), identity, down), 7.0, 1e-12, "sphere");
        TEST_NEAR(IntersectShape(Shape(ShapeKind::kCylinder, {1.0, 6.0, 0.0}), identity, down), 7.0, 1e-12,
                  "cylinder cap at z = 3");
        const Ray side = RayFrom({-10.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
        TEST_NEAR(IntersectShape(Shape(ShapeKind::kCylinder, {1.0, 6.0, 0.0}), identity, side), 9.0, 1e-12,
                  "cylinder side at x = -1");
        const Ray past_end = RayFrom({-10.0, 0.0, 3.5}, {1.0, 0.0, 0.0});
        TEST_ASSERT(IntersectShape(Shape(ShapeKind::kCylinder, {1.0, 6.0, 0.0}), identity, past_end) == kNoHit,
                    "cylinder: beyond its length, miss");
        const Ray away = RayFrom({0.0, 0.0, 10.0}, {0.0, 0.0, 1.0});
        TEST_ASSERT(IntersectShape(Shape(ShapeKind::kBox, {2.0, 2.0, 2.0}), identity, away) == kNoHit,
                    "box behind the ray origin: miss");

        auto tri = std::make_shared<TriangleMesh>();
        tri->vertices = {{0.0, 0.0, 1.0}, {2.0, 0.0, 1.0}, {0.0, 2.0, 1.0}};
        tri->faces = {{0, 1, 2}};
        VisualShape mesh = Shape(ShapeKind::kMesh, {1.0, 1.0, 1.0});
        mesh.mesh = tri;
        TEST_NEAR(IntersectShape(mesh, identity, RayFrom({0.5, 0.5, 10.0}, {0.0, 0.0, -1.0})), 9.0, 1e-12,
                  "mesh triangle hit");
        TEST_ASSERT(IntersectShape(mesh, identity, RayFrom({1.5, 1.5, 10.0}, {0.0, 0.0, -1.0})) == kNoHit,
                    "outside the triangle: miss");

        // Body at x = 20, turned 90 deg about z; a 4 x 1 x 1 box offset +3 m in body x
        // lies along world y, centred at (20, 3, 0).
        Pose body;
        body.position = {20.0, 0.0, 0.0};
        body.rotation = AboutZ(0.5 * kPi);
        const VisualShape bar = Shape(ShapeKind::kBox, {4.0, 1.0, 1.0}, {3.0, 0.0, 0.0});
        TEST_NEAR(IntersectShape(bar, body, RayFrom({20.0, 3.0, 10.0}, {0.0, 0.0, -1.0})), 9.5, 1e-12,
                  "posed body: shape offset rotated with the body");
        TEST_NEAR(IntersectShape(bar, body, RayFrom({20.0, 4.9, 10.0}, {0.0, 0.0, -1.0})), 9.5, 1e-12,
                  "posed body: long side along world y");
        TEST_ASSERT(IntersectShape(bar, body, RayFrom({22.0, 3.0, 10.0}, {0.0, 0.0, -1.0})) == kNoHit,
                    "posed body: short side along world x");
    }

    // ---- P3. PickBody ----
    {
        SceneDescription scene;
        scene.bodies.resize(2);
        scene.bodies[0].shapes = {Shape(ShapeKind::kSphere, {1.0, 0.0, 0.0})};
        scene.bodies[1].shapes = {Shape(ShapeKind::kBox, {1.0, 1.0, 1.0})};
        VizFrame frame;
        frame.bodies.resize(2);
        frame.bodies[0].pose.position = {0.0, 0.0, 0.0};
        frame.bodies[1].pose.position = {0.0, 0.0, 5.0};  // above the sphere
        const Ray down = RayFrom({0.0, 0.0, 10.0}, {0.0, 0.0, -1.0});
        TEST_ASSERT(PickBody(scene, frame, down) == 1, "nearest body along the ray");
        TEST_ASSERT(PickBody(scene, frame, down, {1, 0}) == 0, "hidden body skipped");
        TEST_ASSERT(PickBody(scene, frame, RayFrom({5.0, 5.0, 10.0}, {0.0, 0.0, -1.0})) == -1, "empty space: -1");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
