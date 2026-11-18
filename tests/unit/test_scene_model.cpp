// Unit tests for the renderer-independent scene model and frame history
// (Chrono-free, VSG-free).

#include <seastack/viz_core/frame_history.h>
#include <seastack/viz_core/frame_interpolation.h>
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

        const Pose round_trip = Compose(Inverse(body), Compose(body, shape));
        TEST_NEAR(round_trip.position[0], shape.position[0], 1e-12, "inverse undoes compose: position");
        TEST_NEAR(round_trip.rotation[3], shape.rotation[3], 1e-12, "inverse undoes compose: rotation");
        const Vec3 back = Transform(Inverse(body), Transform(body, {1.0, 2.0, 3.0}));
        TEST_NEAR(back[1], 2.0, 1e-12, "inverse maps a point back");
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

    // --- Interpolation (visual only) ---
    {
        const Quat a = AxisAngle({0.0, 0.0, 1.0}, 0.0);
        const Quat b = AxisAngle({0.0, 0.0, 1.0}, 0.5 * kPi);
        const Quat mid = Slerp(a, b, 0.5);
        const Quat expected = AxisAngle({0.0, 0.0, 1.0}, 0.25 * kPi);
        TEST_NEAR(mid[0], expected[0], 1e-12, "slerp halfway: w");
        TEST_NEAR(mid[3], expected[3], 1e-12, "slerp halfway: z");
        const Quat b_neg{-b[0], -b[1], -b[2], -b[3]};  // same rotation as b
        const Quat mid_neg = Slerp(a, b_neg, 0.5);
        TEST_NEAR(std::abs(mid_neg[0]), expected[0], 1e-12, "slerp takes the shorter arc");
        TEST_NEAR(std::abs(mid_neg[3]), expected[3], 1e-12, "slerp shorter arc: z");

        VizFrame f0 = FrameAt(1.0);
        VizFrame f1 = FrameAt(2.0);
        f0.step = 10;
        f1.step = 11;
        f0.bodies.resize(1);
        f1.bodies.resize(1);
        f1.bodies[0].pose.position = {4.0, 0.0, -2.0};
        f1.bodies[0].pose.rotation = b;
        f0.mooring_lines.resize(1);
        f1.mooring_lines.resize(1);
        f0.mooring_lines[0].node_positions = {{0.0, 0.0, 0.0}};
        f1.mooring_lines[0].node_positions = {{0.0, 0.0, -10.0}};
        f0.mooring_lines[0].node_tensions = {100.0};
        f1.mooring_lines[0].node_tensions = {300.0};

        const VizFrame q = InterpolateFrames(f0, f1, 1.25);
        TEST_NEAR(q.time, 1.25, 1e-12, "interpolated time");
        TEST_ASSERT(q.step == 10, "step from the nearer frame");
        TEST_NEAR(q.bodies[0].pose.position[0], 1.0, 1e-12, "position blended linearly");
        TEST_NEAR(q.bodies[0].pose.position[2], -0.5, 1e-12, "position blended linearly: z");
        TEST_NEAR(q.bodies[0].pose.rotation[3], std::sin(0.0625 * kPi), 1e-12, "rotation slerped");
        TEST_NEAR(q.mooring_lines[0].node_positions[0][2], -2.5, 1e-12, "mooring node blended");
        TEST_NEAR(q.mooring_lines[0].node_tensions[0], 150.0, 1e-12, "tension blended");

        VizFrame f2 = f1;
        f2.mooring_lines[0].node_positions.push_back({1.0, 1.0, 1.0});
        f2.mooring_lines[0].node_tensions.push_back(0.0);
        TEST_ASSERT(InterpolateFrames(f0, f2, 1.75).step == 11, "layout change: nearer frame unchanged");

        FrameHistory h(8);
        h.Push(f0);
        h.Push(f1);
        VizFrame s;
        TEST_ASSERT(h.Sample(1.5, &s), "between frames: interpolated");
        TEST_NEAR(s.bodies[0].pose.position[0], 2.0, 1e-12, "sampled position");
        TEST_ASSERT(!h.Sample(2.0, &s) && s.step == 11, "on a recorded frame: exact");
        TEST_ASSERT(!h.Sample(9.0, &s) && s.step == 11, "after the range: newest");
        TEST_ASSERT(!h.Sample(0.0, &s) && s.step == 10, "before the range: oldest");
    }

    // --- Memory budget ---
    {
        VizFrame f;
        f.bodies.resize(10);
        f.mooring_lines.resize(4);
        for (auto& line : f.mooring_lines) {
            line.node_positions.resize(40);
            line.node_tensions.resize(40);
        }
        const size_t bytes = EstimateFrameBytes(f);
        TEST_ASSERT(bytes > 4 * 40 * 32 && bytes < 8000, "frame size estimate is a few kB");
        TEST_ASSERT(FrameCapacityForBudget(bytes, 256u << 20, 1000000) == (256u << 20) / bytes, "budget fit");
        TEST_ASSERT(FrameCapacityForBudget(bytes, 256u << 20, 1000) == 1000, "frame cap");
        TEST_ASSERT(FrameCapacityForBudget(bytes, 10, 1000) == 2, "at least two frames");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
