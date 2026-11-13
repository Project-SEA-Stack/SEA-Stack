// Unit tests for the model-scaled view defaults in scene_framing.h
// (Chrono-free, VSG-free): bounds, camera framing, water extent, clip planes
// and mooring marker radii.

#include <seastack/viz_core/scene_framing.h>

#include "test_macros.h"

#include <cmath>

using namespace seastack::viz;

namespace {

constexpr double kPi = 3.14159265358979323846;

double Distance(const Vec3& a, const Vec3& b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Bounds3 Box(Vec3 lo, Vec3 hi) {
    Bounds3 b;
    b.Extend(lo);
    b.Extend(hi);
    return b;
}

}  // namespace

int main() {
    TestResults test_results;

    // Bounds basics.
    {
        Bounds3 empty;
        TEST_ASSERT(!empty.IsValid(), "default bounds are empty");
        Bounds3 b = Box({-10, -5, -30}, {10, 5, 2});
        TEST_ASSERT(b.IsValid(), "extended bounds are valid");
        TEST_NEAR(b.Center()[2], -14.0, 1e-12, "center z");
        TEST_NEAR(ModelLengthScale(b), 20.0, 1e-12, "length scale = larger plan side");
        empty.Extend(b);
        TEST_NEAR(empty.Size()[0], 20.0, 1e-12, "extend by box");
        Bounds3 point = Box({1, 1, 1}, {1, 1, 1});
        TEST_NEAR(ModelLengthScale(point), kMinModelLength, 1e-12, "point model uses minimum length");
    }

    // Deep-water wavelength: T = 8 s gives ~99.9 m.
    {
        TEST_NEAR(DeepWaterWavelength(8.0, 9.81), 9.81 * 64.0 / (2.0 * kPi), 1e-9, "deep-water wavelength");
        TEST_NEAR(DeepWaterWavelength(0.0), 0.0, 0.0, "no period gives zero wavelength");
    }

    // Framing: the camera looks at the box centre from the requested direction
    // and the bounding sphere fits inside the field of view.
    {
        const Bounds3 b = Box({-10, -10, -30}, {10, 10, 5});
        const double vfov = 40.0;
        const double aspect = 16.0 / 9.0;
        const CameraPose front = FrameBounds(b, CameraView::kFront, vfov, aspect);
        TEST_NEAR(front.target[2], b.Center()[2], 1e-12, "target at box centre");
        TEST_ASSERT(front.eye[1] < front.target[1], "front view camera on -Y side");
        TEST_NEAR(front.eye[0], front.target[0], 1e-12, "front view has no X offset");

        const double dist = Distance(front.eye, front.target);
        const double half_v = 0.5 * vfov * kPi / 180.0;
        TEST_ASSERT(b.Radius() / dist <= std::sin(half_v) + 1e-12, "bounding sphere fits the vertical FOV");
        TEST_NEAR(b.Radius() / dist, std::sin(half_v) / kFramingMargin, 1e-9, "framing keeps the margin");

        // Portrait viewport: horizontal FOV limits, so the camera moves back.
        const CameraPose narrow = FrameBounds(b, CameraView::kFront, vfov, 0.5);
        TEST_ASSERT(Distance(narrow.eye, narrow.target) > dist, "narrow viewport frames from further away");

        // Distance scales linearly with model size (lab vs full scale).
        const Bounds3 lab = Box({-0.1, -0.1, -0.3}, {0.1, 0.1, 0.05});
        const CameraPose lab_pose = FrameBounds(lab, CameraView::kFront, vfov, aspect);
        TEST_NEAR(Distance(lab_pose.eye, lab_pose.target), dist / 100.0, 1e-9, "1:100 model frames 100x closer");

        const CameraPose top = FrameBounds(b, CameraView::kTop, vfov, aspect);
        TEST_ASSERT(top.eye[2] > top.target[2], "top view camera above");
        TEST_NEAR(top.up[1], 1.0, 1e-12, "top view uses +Y as screen up");
        const CameraPose iso = FrameBounds(b, CameraView::kIsometric, vfov, aspect);
        TEST_ASSERT(iso.eye[0] > iso.target[0] && iso.eye[1] < iso.target[1] && iso.eye[2] > iso.target[2],
                    "isometric view from -Y/+X quadrant, above");
        TEST_NEAR(iso.up[2], 1.0, 1e-12, "isometric view keeps Z up");

        // Zero direction falls back to isometric rather than producing NaNs.
        const CameraPose fallback = FrameBounds(b, Vec3{0, 0, 0}, vfov, aspect);
        TEST_ASSERT(std::isfinite(fallback.eye[0]) && fallback.eye[2] > fallback.target[2],
                    "zero direction falls back to isometric");
    }

    // Water patch: covers the model and at least 1.5 wavelengths, centred on the model.
    {
        const Bounds3 rm3 = Box({-10, -10, -35}, {10, 10, 3});
        const WaterExtent short_waves = ComputeWaterExtent(rm3, 10.0);
        TEST_NEAR(short_waves.width, kWaterModelFactor * 20.0, 1e-12, "model length dominates for short waves");
        const WaterExtent long_waves = ComputeWaterExtent(rm3, 100.0);
        TEST_NEAR(long_waves.width, 150.0, 1e-12, "1.5 wavelengths for long waves");
        TEST_NEAR(long_waves.length, long_waves.width, 1e-12, "square patch");

        const Bounds3 offset = Box({100, 50, -1}, {110, 60, 1});
        const WaterExtent w = ComputeWaterExtent(offset, 0.0);
        TEST_NEAR(w.center_x, 105.0, 1e-12, "patch centred on model (x)");
        TEST_NEAR(w.center_y, 55.0, 1e-12, "patch centred on model (y)");

        // Still water: at least kStillWaterPlanFactor mean plan sizes.
        const Bounds3 sphere = Box({-5, -5, -6}, {5, 5, 4});
        TEST_NEAR(ComputeWaterExtent(sphere, 0.0).width, kStillWaterPlanFactor * 10.0, 1e-12,
                  "still water: 10 m sphere gets a 50 m patch");
        TEST_NEAR(ComputeWaterExtent(sphere, 20.0).width, 30.0, 1e-12,
                  "with waves the still-water minimum does not apply");
        const Bounds3 slender = Box({0, -1.5, -2}, {180, 1.5, 2});
        TEST_NEAR(ComputeWaterExtent(slender, 0.0).width, kWaterModelFactor * 180.0, 1e-9,
                  "slender model in still water: length term still covers the device");
    }

    // Clip planes scale with the view.
    {
        const ClipPlanes full = ComputeClipPlanes(80.0, 150.0);
        const ClipPlanes lab = ComputeClipPlanes(0.8, 1.5);
        TEST_ASSERT(full.near_m > lab.near_m, "near plane scales with view distance");
        TEST_ASSERT(full.far_m > 80.0 + 150.0, "far plane covers camera distance plus water");
        TEST_ASSERT(lab.near_m < 0.8 * 0.1, "lab near plane well inside the framing distance");
    }

    // Mooring radii: RM3-size model keeps the previous defaults.
    {
        const MooringVizRadii r = DefaultMooringVizRadii(20.0);
        TEST_NEAR(r.line, 0.15, 1e-12, "20 m model line radius");
        TEST_NEAR(r.endpoint, 0.5, 1e-12, "20 m model endpoint radius");
        TEST_NEAR(r.node_marker, 0.2, 1e-12, "20 m model node radius");
        TEST_NEAR(MeanPlanLength(Box({0, -1.5, -2}, {180, 1.5, 2})), std::sqrt(180.0 * 3.0), 1e-9,
                  "slender model: geometric mean of plan sides");
        TEST_NEAR(MeanPlanLength(Box({-15, -15, -30}, {15, 15, 5})), 30.0, 1e-9,
                  "square plan: same as ModelLengthScale");
        const MooringVizRadii lab = DefaultMooringVizRadii(0.2);
        TEST_NEAR(lab.line, 0.0015, 1e-12, "lab-scale line radius scales down");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
