/*********************************************************************
 * @file  test_frame_series.cpp
 * @brief Unit tests for the time series read from recorded frames (plots,
 *        trajectories, force arrows) and for body loads in VizFrame.
 *
 * Pass criteria:
 *   F1  SampleIndices returns the frames in [t_begin, t_end], oldest first,
 *       at most max_points, always ending with the last frame in range.
 *   F2  CardanAnglesXYZ inverts Chrono's SetFromCardanAnglesXYZ (formula
 *       copied here) for general angles.
 *   F3  BodyWrench / TotalWrench index sources and bodies as documented.
 *   F4  Loads are interpolated linearly; a frame with loads of the wrong
 *       size is inconsistent with the scene.
 *
 * Self-contained: viz_core only.
 *********************************************************************/

#include <seastack/viz_core/frame_history.h>
#include <seastack/viz_core/frame_interpolation.h>
#include <seastack/viz_core/frame_series.h>

#include "test_macros.h"

#include <cmath>
#include <vector>

using namespace seastack::viz;

namespace {

/// Chrono's ChQuaternion::SetFromCardanAnglesXYZ (roll, pitch, yaw) [rad].
Quat FromCardanXYZ(double roll, double pitch, double yaw) {
    const double t0 = std::cos(yaw * 0.5), t1 = std::sin(yaw * 0.5);
    const double t2 = std::cos(roll * 0.5), t3 = std::sin(roll * 0.5);
    const double t4 = std::cos(pitch * 0.5), t5 = std::sin(pitch * 0.5);
    return {t0 * t2 * t4 + t1 * t3 * t5, t0 * t3 * t4 - t1 * t2 * t5, t0 * t2 * t5 + t1 * t3 * t4,
            t1 * t2 * t4 - t0 * t3 * t5};
}

VizFrame Frame(double t, size_t n_bodies, size_t n_sources) {
    VizFrame f;
    f.time = t;
    f.bodies.resize(n_bodies);
    f.body_wrenches.resize(n_bodies * n_sources);
    return f;
}

}  // namespace

int main() {
    TestResults test_results;

    // ---- F1. SampleIndices ----
    {
        FrameHistory h(100);
        for (int i = 0; i < 50; ++i) {
            h.Push(Frame(0.1 * i, 1, 0));  // t = 0 .. 4.9 s
        }
        auto all = SampleIndices(h, -1.0, 10.0, 1000);
        TEST_ASSERT(all.size() == 50 && all.front() == 0 && all.back() == 49, "whole range, no decimation");
        auto mid = SampleIndices(h, 1.05, 2.0, 1000);
        TEST_ASSERT(!mid.empty() && h.At(mid.front()).time >= 1.05 && std::abs(h.At(mid.back()).time - 2.0) < 1e-12,
                    "range [1.05, 2.0]: first frame inside, last at 2.0");
        TEST_ASSERT(mid.size() == 10, "range [1.05, 2.0]: 10 frames (1.1 .. 2.0)");
        auto few = SampleIndices(h, 0.0, 4.9, 7);
        bool increasing = true;
        for (size_t k = 1; k < few.size(); ++k) increasing = increasing && few[k] > few[k - 1];
        TEST_ASSERT(few.size() <= 7 && few.back() == 49 && increasing, "decimated: at most 7, ends at the last frame");
        TEST_ASSERT(SampleIndices(h, 6.0, 7.0, 10).empty(), "range after the recording: empty");
        TEST_ASSERT(SampleIndices(h, 2.0, 1.0, 10).empty(), "reversed range: empty");
        TEST_ASSERT(SampleIndices(FrameHistory(4), 0.0, 1.0, 10).empty(), "empty history: empty");
    }

    // ---- F2. Cardan angles ----
    {
        double worst = 0.0;
        for (double roll : {-2.5, -0.4, 0.0, 0.3, 1.2}) {
            for (double pitch : {-1.2, -0.2, 0.0, 0.5, 1.4}) {
                for (double yaw : {-3.0, -0.7, 0.0, 0.9, 2.8}) {
                    const Vec3 a = CardanAnglesXYZ(FromCardanXYZ(roll, pitch, yaw));
                    worst = std::max({worst, std::abs(a[0] - roll), std::abs(a[1] - pitch), std::abs(a[2] - yaw)});
                }
            }
        }
        TEST_ASSERT(worst < 1e-12, "round trip with Chrono's Cardan XYZ (worst " + std::to_string(worst) + " rad)");
        const Vec3 identity = CardanAnglesXYZ(kIdentityQuat);
        TEST_ASSERT(identity[0] == 0.0 && identity[1] == 0.0 && identity[2] == 0.0, "identity: zero angles");
    }

    // ---- F3. Loads by source and body ----
    {
        VizFrame f = Frame(1.0, 2, 3);  // 2 bodies, 3 sources
        for (size_t s = 0; s < 3; ++s) {
            for (size_t b = 0; b < 2; ++b) {
                f.body_wrenches[s * 2 + b].force = {1.0 * (s + 1), 10.0 * (b + 1), 0.0};
                f.body_wrenches[s * 2 + b].moment = {0.0, 0.0, 100.0 * (s + 1) * (b + 1)};
            }
        }
        const Wrench w = BodyWrench(f, 2, 1);
        TEST_ASSERT(w.force[0] == 3.0 && w.force[1] == 20.0 && w.moment[2] == 600.0, "source 2 on body 1");
        const Wrench total = TotalWrench(f, 0);
        TEST_ASSERT(total.force[0] == 6.0 && total.force[1] == 30.0 && total.moment[2] == 600.0,
                    "total on body 0 sums the 3 sources");
        const Wrench none = BodyWrench(Frame(0.0, 2, 0), 0, 0);
        TEST_ASSERT(Magnitude(none.force) == 0.0, "no force data: zero load");
        TEST_NEAR(Magnitude({3.0, 4.0, 12.0}), 13.0, 1e-15, "magnitude");
    }

    // ---- F4. Interpolation and consistency ----
    {
        VizFrame a = Frame(0.0, 1, 1), b = Frame(1.0, 1, 1);
        a.body_wrenches[0].force = {0.0, 0.0, 100.0};
        b.body_wrenches[0].force = {0.0, 0.0, 300.0};
        b.body_wrenches[0].moment = {10.0, 0.0, 0.0};
        const VizFrame m = InterpolateFrames(a, b, 0.25);
        TEST_NEAR(m.body_wrenches[0].force[2], 150.0, 1e-12, "force interpolated linearly");
        TEST_NEAR(m.body_wrenches[0].moment[0], 2.5, 1e-12, "moment interpolated linearly");
        VizFrame no_loads = Frame(1.0, 1, 0);
        TEST_ASSERT(InterpolateFrames(a, no_loads, 0.75).body_wrenches.empty(),
                    "different load layout: the nearer frame unchanged");

        SceneDescription scene;
        scene.bodies.resize(1);
        scene.force_sources = {"Excitation"};
        TEST_ASSERT(IsConsistent(scene, a), "loads sized sources x bodies: consistent");
        TEST_ASSERT(IsConsistent(scene, no_loads), "no loads: consistent");
        VizFrame wrong = Frame(0.0, 1, 2);
        TEST_ASSERT(!IsConsistent(scene, wrong), "wrong load count: inconsistent");
        TEST_ASSERT(EstimateFrameBytes(wrong) > EstimateFrameBytes(no_loads), "loads counted in the frame size");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
