// Unit tests for the renderer-independent scene model and frame history
// (Chrono-free, VSG-free).

#include <seastack/viz_core/frame_history.h>
#include <seastack/viz_core/scene_model.h>

#include "test_macros.h"

#include <cmath>

using namespace seastack::viz;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Rotation of angle [rad] about the unit axis.
Quat AxisAngle(const Vec3& axis, double angle) {
    const double s = std::sin(0.5 * angle);
    return {std::cos(0.5 * angle), s * axis[0], s * axis[1], s * axis[2]};
}

VizFrame FrameAt(double t) {
    VizFrame f;
    f.time = t;
    return f;
}

}  // namespace

int main() {
    TestResults test_results;

    // --- Pose maths ---
    {
        const Quat q = AxisAngle({0.0, 0.0, 1.0}, 0.5 * kPi);  // +90 deg about Z
        const Vec3 r = Rotate(q, {1.0, 0.0, 0.0});
        TEST_NEAR(r[0], 0.0, 1e-12, "Rz(90) x -> y: x");
        TEST_NEAR(r[1], 1.0, 1e-12, "Rz(90) x -> y: y");

        Pose body;
        body.position = {10.0, 0.0, 0.0};
        body.rotation = q;
        const Vec3 p = Transform(body, {1.0, 0.0, 2.0});
        TEST_NEAR(p[0], 10.0, 1e-12, "transform: x");
        TEST_NEAR(p[1], 1.0, 1e-12, "transform: y");
        TEST_NEAR(p[2], 2.0, 1e-12, "transform: z");

        Pose shape;
        shape.position = {1.0, 0.0, 0.0};
        shape.rotation = q;
        const Pose world = Compose(body, shape);
        const Vec3 a = Transform(world, {1.0, 0.0, 0.0});
        const Vec3 b = Transform(body, Transform(shape, {1.0, 0.0, 0.0}));
        TEST_NEAR(a[0], b[0], 1e-12, "compose matches nested transform: x");
        TEST_NEAR(a[1], b[1], 1e-12, "compose matches nested transform: y");
        TEST_NEAR(world.rotation[0], std::cos(0.5 * kPi), 1e-12, "compose: two 90 deg turns = 180 deg");
    }

    // --- Consistency check ---
    {
        SceneDescription scene;
        scene.bodies.resize(2);
        scene.mooring_lines.push_back({3, MooringEndType::kFixed, MooringEndType::kCoupled});
        VizFrame frame;
        frame.bodies.resize(2);
        frame.mooring_lines.resize(1);
        frame.mooring_lines[0].node_positions.resize(3);
        TEST_ASSERT(IsConsistent(scene, frame), "tensions optional");
        frame.mooring_lines[0].node_tensions.resize(2);
        TEST_ASSERT(!IsConsistent(scene, frame), "tension count must match nodes");
        frame.mooring_lines[0].node_tensions.resize(3);
        frame.bodies.resize(1);
        TEST_ASSERT(!IsConsistent(scene, frame), "body count must match");
    }

    // --- Frame history ---
    {
        FrameHistory h(3);
        TEST_ASSERT(h.Empty(), "starts empty");
        h.Push(FrameAt(0.1));
        h.Push(FrameAt(0.2));
        TEST_ASSERT(h.Size() == 2, "size grows");
        TEST_NEAR(h.Oldest().time, 0.1, 1e-12, "oldest before wrap");
        h.Push(FrameAt(0.3));
        h.Push(FrameAt(0.4));
        h.Push(FrameAt(0.5));
        TEST_ASSERT(h.Size() == 3, "bounded by capacity");
        TEST_NEAR(h.Oldest().time, 0.3, 1e-12, "oldest dropped on wrap");
        TEST_NEAR(h.At(1).time, 0.4, 1e-12, "time order after wrap");
        TEST_NEAR(h.Newest().time, 0.5, 1e-12, "newest after wrap");

        TEST_ASSERT(h.NearestIndex(0.0) == 0, "nearest clamps below");
        TEST_ASSERT(h.NearestIndex(9.0) == 2, "nearest clamps above");
        TEST_ASSERT(h.NearestIndex(0.36) == 1, "nearest picks closer frame");
        TEST_ASSERT(h.NearestIndex(0.34) == 0, "nearest picks closer earlier frame");
        TEST_ASSERT(h.NearestIndex(0.35) == 0, "tie goes to the earlier frame");

        h.Push(FrameAt(0.2));  // simulation reset / rewind
        TEST_ASSERT(h.Size() == 1, "non-increasing time restarts history");
        TEST_NEAR(h.Newest().time, 0.2, 1e-12, "restarted history holds the new frame");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
