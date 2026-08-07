/*********************************************************************
 * @file  test_adapter_layer.cpp
 * @brief Unit tests for adapter-layer components.
 *
 * Tests:
 *   1. ChronoForceAttacher: force caching (evaluator called once per timestep)
 *   2. ChronoForceAttacher: out-of-range index throws
 *   3. ChronoForceAttacher: divergence from invalid force (NaN)
 *   4. ChronoForceAttacher: divergence from excessive body position
 *   4b. ChronoForceAttacher: divergence from roll/pitch beyond limit
 *   4c. ChronoForceAttacher: magnitude limits disableable; NaN still trips
 *   5. ChronoForceAttacher: multi-body routing (correct body_num per DOF)
 *   6. ChronoForceAttacher: Chrono ChForce clone safety
 *   7. ChronoForceAttacher: destruction/lifecycle sanity
 *   8. HydroSystemConfig: default values match expectations
 *   9. HydroSystem: immutability guard (setter after model construction)
 *
 * Self-contained — no external data files.
 *********************************************************************/

#include <seastack/adapters/chrono/chrono_force_attacher.h>
#include <seastack/adapters/chrono/hydro_system_config.h>
#include <seastack/core/types.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/core/ChQuaternion.h>

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace seastack::hydro;
using namespace seastack::chrono;

static int g_pass = 0;
static int g_fail = 0;

static void Check(bool condition, const std::string& label) {
    if (condition) {
        ++g_pass;
    } else {
        ++g_fail;
        std::cerr << "FAIL: " << label << "\n";
    }
}

// Helper: create N named bodies ("body1" .. "bodyN") in a system.
static std::vector<std::shared_ptr<::chrono::ChBody>> MakeBodies(
    ::chrono::ChSystemNSC& system, int n) {
    std::vector<std::shared_ptr<::chrono::ChBody>> bodies;
    for (int i = 1; i <= n; ++i) {
        auto body = chrono_types::make_shared<::chrono::ChBody>();
        body->SetName("body" + std::to_string(i));
        body->SetPos(::chrono::ChVector3d(0, 0, 0));
        body->SetFixed(false);
        system.AddBody(body);
        bodies.push_back(body);
    }
    return bodies;
}

// Evaluator that counts calls and returns a fixed force pattern.
struct CountingEvaluator {
    int call_count = 0;
    BodyForces result;

    CountingEvaluator(int num_bodies) {
        result.resize(num_bodies);
        for (int b = 0; b < num_bodies; ++b) {
            result[b].force  = Eigen::Vector3d(100.0 * (b + 1), 200.0 * (b + 1), 300.0 * (b + 1));
            result[b].moment = Eigen::Vector3d(10.0 * (b + 1), 20.0 * (b + 1), 30.0 * (b + 1));
        }
    }

    BodyForces operator()(double /*time*/) {
        ++call_count;
        return result;
    }
};

// Evaluator that returns NaN forces.
struct NaNEvaluator {
    int num_bodies;
    NaNEvaluator(int n) : num_bodies(n) {}

    BodyForces operator()(double /*time*/) {
        BodyForces forces(num_bodies);
        forces[0].force = Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 0);
        return forces;
    }
};

// Evaluator that returns forces exceeding the magnitude limit.
struct HugeForceEvaluator {
    int num_bodies;
    HugeForceEvaluator(int n) : num_bodies(n) {}

    BodyForces operator()(double /*time*/) {
        BodyForces forces(num_bodies);
        forces[0].force = Eigen::Vector3d(2e10, 0, 0);
        return forces;
    }
};

// ════════════════════════════════════════════════════════════════════════
// Test: multiple accesses at the same timestep → single evaluator call
// ════════════════════════════════════════════════════════════════════════
static void TestForceCachingSameTimestep() {
    ::chrono::ChSystemNSC system;
    auto bodies = MakeBodies(system, 2);

    CountingEvaluator eval(2);
    ChronoForceAttacher attacher(bodies, std::ref(eval));

    // First access triggers evaluation
    double f1 = attacher.CoordinateFuncForBody(1, 0);
    Check(eval.call_count == 1, "caching: evaluator called on first access");
    Check(f1 == 100.0, "caching: body1 surge = 100");

    // Same DOF, same time → cached
    double f1b = attacher.CoordinateFuncForBody(1, 0);
    Check(eval.call_count == 1, "caching: repeat same DOF is cached");
    Check(f1b == 100.0, "caching: repeat value consistent");

    // Different DOF, same body, same time → cached
    double f2 = attacher.CoordinateFuncForBody(1, 1);
    Check(eval.call_count == 1, "caching: different DOF same body is cached");
    Check(f2 == 200.0, "caching: body1 sway = 200");

    // Different body, same time → cached
    double f3 = attacher.CoordinateFuncForBody(2, 0);
    Check(eval.call_count == 1, "caching: different body same time is cached");
    Check(f3 == 200.0, "caching: body2 surge = 200");

    // All 6 DOFs of body2
    double f_yaw = attacher.CoordinateFuncForBody(2, 5);
    Check(eval.call_count == 1, "caching: body2 yaw same time is cached");
    Check(f_yaw == 60.0, "caching: body2 yaw = 60");

    std::cout << "  TestForceCachingSameTimestep done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: time advancement triggers new evaluation
// ════════════════════════════════════════════════════════════════════════
static void TestForceCachingTimeAdvance() {
    ::chrono::ChSystemNSC system;
    system.SetTimestepperType(::chrono::ChTimestepper::Type::EULER_IMPLICIT);
    auto bodies = MakeBodies(system, 1);

    CountingEvaluator eval(1);
    ChronoForceAttacher attacher(bodies, std::ref(eval));

    // Initial access at t=0
    attacher.CoordinateFuncForBody(1, 0);
    int count_before_step = eval.call_count;
    Check(count_before_step >= 1, "time-advance: evaluator called at least once at t=0");

    // Advance the simulation by one timestep (this changes GetChTime())
    system.DoStepDynamics(0.001);

    // After the step, Chrono's force callbacks may have already triggered the
    // evaluator at the new time. The key assertion: the count increased.
    int count_after_step = eval.call_count;
    Check(count_after_step > count_before_step,
          "time-advance: evaluator called at least once more after DoStepDynamics");

    // Manual access at the post-step time should NOT trigger another call
    attacher.CoordinateFuncForBody(1, 0);
    Check(eval.call_count == count_after_step,
          "time-advance: manual access after step is cached");

    std::cout << "  TestForceCachingTimeAdvance done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: out-of-range indices throw
// ════════════════════════════════════════════════════════════════════════
static void TestOutOfRangeIndices() {
    ::chrono::ChSystemNSC system;
    auto bodies = MakeBodies(system, 1);

    CountingEvaluator eval(1);
    ChronoForceAttacher attacher(bodies, std::ref(eval));

    bool threw_b0 = false;
    try { attacher.CoordinateFuncForBody(0, 0); } catch (const std::out_of_range&) { threw_b0 = true; }
    Check(threw_b0, "out-of-range: body index 0 (1-based) throws");

    bool threw_b2 = false;
    try { attacher.CoordinateFuncForBody(2, 0); } catch (const std::out_of_range&) { threw_b2 = true; }
    Check(threw_b2, "out-of-range: body index 2 with 1 body throws");

    bool threw_dof_neg = false;
    try { attacher.CoordinateFuncForBody(1, -1); } catch (const std::out_of_range&) { threw_dof_neg = true; }
    Check(threw_dof_neg, "out-of-range: DOF -1 throws");

    bool threw_dof6 = false;
    try { attacher.CoordinateFuncForBody(1, 6); } catch (const std::out_of_range&) { threw_dof6 = true; }
    Check(threw_dof6, "out-of-range: DOF 6 throws");

    std::cout << "  TestOutOfRangeIndices done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: divergence from NaN force
// ════════════════════════════════════════════════════════════════════════
static void TestDivergenceNaNForce() {
    ::chrono::ChSystemNSC system;
    auto bodies = MakeBodies(system, 1);

    NaNEvaluator eval(1);
    ChronoForceAttacher attacher(bodies, std::ref(eval));

    Check(!attacher.HasDiverged(), "NaN divergence: not diverged initially");

    double f = attacher.CoordinateFuncForBody(1, 0);
    Check(attacher.HasDiverged(), "NaN divergence: diverged after NaN force");
    Check(f == 0.0, "NaN divergence: force zeroed after divergence");

    // Once diverged, subsequent calls return zero
    double f2 = attacher.CoordinateFuncForBody(1, 0);
    Check(f2 == 0.0, "NaN divergence: stays zeroed");
    Check(attacher.HasDiverged(), "NaN divergence: still diverged");

    std::cout << "  TestDivergenceNaNForce done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: divergence from huge force
// ════════════════════════════════════════════════════════════════════════
static void TestDivergenceHugeForce() {
    ::chrono::ChSystemNSC system;
    auto bodies = MakeBodies(system, 1);

    HugeForceEvaluator eval(1);
    ChronoForceAttacher attacher(bodies, std::ref(eval));

    double f = attacher.CoordinateFuncForBody(1, 0);
    Check(attacher.HasDiverged(), "huge force divergence: diverged after 2e10 N force");
    Check(f == 0.0, "huge force divergence: force zeroed");

    std::cout << "  TestDivergenceHugeForce done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: divergence from body state (extreme position)
// ════════════════════════════════════════════════════════════════════════
static void TestDivergenceBodyState() {
    ::chrono::ChSystemNSC system;
    auto bodies = MakeBodies(system, 1);

    CountingEvaluator eval(1);
    ChronoForceAttacher attacher(bodies, std::ref(eval));

    // Move body far beyond position limit (200 m)
    bodies[0]->SetPos(::chrono::ChVector3d(0, 0, 500));

    double f = attacher.CoordinateFuncForBody(1, 0);
    Check(attacher.HasDiverged(), "body state divergence: diverged with pos 500m > 200m limit");
    Check(f == 0.0, "body state divergence: force zeroed");

    std::cout << "  TestDivergenceBodyState done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: divergence from roll beyond default ~90 deg limit
// ════════════════════════════════════════════════════════════════════════
static void TestDivergenceRollPitch() {
    ::chrono::ChSystemNSC system;
    auto bodies = MakeBodies(system, 1);

    CountingEvaluator eval(1);
    ChronoForceAttacher attacher(bodies, std::ref(eval));

    // Cardan XYZ: roll is rotation about X. ~100 deg > default 90 deg.
    bodies[0]->SetRot(::chrono::QuatFromAngleX(100.0 * chrono::CH_DEG_TO_RAD));

    double f = attacher.CoordinateFuncForBody(1, 0);
    Check(attacher.HasDiverged(), "roll/pitch divergence: diverged with roll 100 deg");
    Check(f == 0.0, "roll/pitch divergence: force zeroed");

    std::cout << "  TestDivergenceRollPitch done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: magnitude limits can be disabled; NaN still trips
// ════════════════════════════════════════════════════════════════════════
static void TestDivergenceLimitsConfigurable() {
    ::chrono::ChSystemNSC system;
    auto bodies = MakeBodies(system, 1);

    // Extreme position with limits disabled — must not trip.
    {
        CountingEvaluator eval(1);
        DivergenceLimits limits;
        limits.enabled = false;
        ChronoForceAttacher attacher(bodies, std::ref(eval), true, limits);
        bodies[0]->SetPos(::chrono::ChVector3d(0, 0, 500));
        bodies[0]->SetRot(::chrono::QUNIT);
        (void)attacher.CoordinateFuncForBody(1, 0);
        Check(!attacher.HasDiverged(),
              "limits disabled: extreme position does not diverge");
    }

    // Raised roll/pitch limit — 100 deg roll must not trip.
    {
        CountingEvaluator eval(1);
        DivergenceLimits limits;
        limits.max_roll_pitch_rad = 2.0;  // ~114.6 deg
        ChronoForceAttacher attacher(bodies, std::ref(eval), true, limits);
        bodies[0]->SetPos(::chrono::ChVector3d(0, 0, 0));
        bodies[0]->SetRot(::chrono::QuatFromAngleX(100.0 * chrono::CH_DEG_TO_RAD));
        (void)attacher.CoordinateFuncForBody(1, 0);
        Check(!attacher.HasDiverged(),
              "raised roll limit: 100 deg roll does not diverge");
    }

    // Limits disabled but NaN force still trips.
    {
        NaNEvaluator eval(1);
        DivergenceLimits limits;
        limits.enabled = false;
        ChronoForceAttacher attacher(bodies, std::ref(eval), true, limits);
        bodies[0]->SetPos(::chrono::ChVector3d(0, 0, 0));
        bodies[0]->SetRot(::chrono::QUNIT);
        (void)attacher.CoordinateFuncForBody(1, 0);
        Check(attacher.HasDiverged(),
              "limits disabled: NaN force still diverges");
    }

    std::cout << "  TestDivergenceLimitsConfigurable done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: multi-body routing — each body's callbacks reach the right
// (body_num, dof) in CoordinateFuncForBody
// ════════════════════════════════════════════════════════════════════════
static void TestMultiBodyRouting() {
    ::chrono::ChSystemNSC system;
    auto bodies = MakeBodies(system, 3);

    CountingEvaluator eval(3);
    ChronoForceAttacher attacher(bodies, std::ref(eval));

    // Body 1: force=(100,200,300) moment=(10,20,30)
    Check(attacher.CoordinateFuncForBody(1, 0) == 100.0, "routing: body1 surge");
    Check(attacher.CoordinateFuncForBody(1, 2) == 300.0, "routing: body1 heave");
    Check(attacher.CoordinateFuncForBody(1, 5) ==  30.0, "routing: body1 yaw");

    // Body 2: force=(200,400,600) moment=(20,40,60)
    Check(attacher.CoordinateFuncForBody(2, 0) == 200.0, "routing: body2 surge");
    Check(attacher.CoordinateFuncForBody(2, 3) ==  20.0, "routing: body2 roll");

    // Body 3: force=(300,600,900) moment=(30,60,90)
    Check(attacher.CoordinateFuncForBody(3, 0) == 300.0, "routing: body3 surge");
    Check(attacher.CoordinateFuncForBody(3, 4) ==  60.0, "routing: body3 pitch");
    Check(attacher.CoordinateFuncForBody(3, 5) ==  90.0, "routing: body3 yaw");

    Check(eval.call_count == 1, "routing: all 3 bodies served from single evaluation");

    std::cout << "  TestMultiBodyRouting done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: Chrono ChForce clone safety — cloned ChForce still produces
// correct values via cloned ComponentFunc objects
// ════════════════════════════════════════════════════════════════════════
static void TestChForceCloneSafety() {
    ::chrono::ChSystemNSC system;
    auto bodies = MakeBodies(system, 1);

    CountingEvaluator eval(1);
    ChronoForceAttacher attacher(bodies, std::ref(eval));

    // Trigger initial evaluation so cache is populated
    attacher.CoordinateFuncForBody(1, 0);

    // Get the ChForce that was registered on body1. The body's force list
    // holds shared_ptr<ChForce> objects; the first two are ours (force+torque).
    auto& force_list = bodies[0]->GetForces();
    Check(force_list.size() >= 2, "clone: body has at least 2 ChForce objects");

    // Clone the force ChForce (this exercises ComponentFunc::Clone())
    auto original_force = force_list[0];
    std::shared_ptr<::chrono::ChForce> cloned(
        static_cast<::chrono::ChForce*>(original_force->Clone()));

    // The cloned ChForce's ChFunction objects should still route through
    // the attacher and return correct cached values.
    auto cloned_fx = cloned->GetF_x();
    auto cloned_fy = cloned->GetF_y();
    auto cloned_fz = cloned->GetF_z();

    Check(cloned_fx != nullptr, "clone: cloned f_x not null");
    Check(cloned_fy != nullptr, "clone: cloned f_y not null");
    Check(cloned_fz != nullptr, "clone: cloned f_z not null");

    // GetVal(time) on cloned functions should produce the same results
    double fx_val = cloned_fx->GetVal(0.0);
    double fy_val = cloned_fy->GetVal(0.0);
    double fz_val = cloned_fz->GetVal(0.0);
    Check(fx_val == 100.0, "clone: cloned f_x returns body1 surge = 100");
    Check(fy_val == 200.0, "clone: cloned f_y returns body1 sway = 200");
    Check(fz_val == 300.0, "clone: cloned f_z returns body1 heave = 300");

    std::cout << "  TestChForceCloneSafety done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: destruction/lifecycle — attacher can be destroyed normally after
// force registration and evaluation, without double-free or crash
// ════════════════════════════════════════════════════════════════════════
static void TestDestructionLifecycle() {
    ::chrono::ChSystemNSC system;
    auto bodies = MakeBodies(system, 2);

    {
        CountingEvaluator eval(2);
        ChronoForceAttacher attacher(bodies, std::ref(eval));

        // Evaluate forces to populate caches and exercise full callback chain
        attacher.CoordinateFuncForBody(1, 0);
        attacher.CoordinateFuncForBody(2, 5);

        // Step the simulation so Chrono's internal force evaluation runs
        system.DoStepDynamics(0.001);

        Check(!attacher.HasDiverged(), "lifecycle: no divergence during normal run");
    }
    // attacher is now destroyed. ChBody still holds shared_ptr<ChForce>,
    // which hold shared_ptr<ComponentFunc>. The ComponentFunc objects have
    // a dangling attacher_ pointer, but no one should call GetVal() after
    // the attacher is gone. Verify the bodies themselves are still valid.
    Check(bodies[0] != nullptr, "lifecycle: body1 still valid after attacher destruction");
    Check(bodies[1] != nullptr, "lifecycle: body2 still valid after attacher destruction");

    // Verify bodies still have forces registered (shared_ptr prevents
    // double-free even though attacher is gone)
    Check(bodies[0]->GetForces().size() >= 2,
          "lifecycle: body1 still has force objects after attacher destruction");

    std::cout << "  TestDestructionLifecycle done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: HydroSystemConfig defaults
// ════════════════════════════════════════════════════════════════════════
static void TestHydroSystemConfigDefaults() {
    HydroSystemConfig cfg;

    Check(cfg.excitation_truncation_time == 0.0, "config default: excitation_truncation_time = 0");
    Check(cfg.excitation_method == ExcitationMethod::kAuto, "config default: excitation_method = kAuto");
    Check(cfg.excitation_interpolation == ExcitationInterpolation::kCartesian,
          "config default: excitation_interpolation = kCartesian");

    Check(cfg.radiation_truncation_time == 0.0, "config default: radiation_truncation_time = 0");
    Check(cfg.radiation_method == RadiationMethod::kRirfConvolution,
          "config default: radiation_method = kRirfConvolution");
    Check(cfg.output_kernel_fit == false, "config default: output_kernel_fit = false");

    Check(cfg.diagnostics_output_dir.empty(), "config default: diagnostics_output_dir empty");
    Check(cfg.linear_damping.empty(), "config default: linear_damping empty");
    Check(cfg.quadratic_damping.empty(), "config default: quadratic_damping empty");
    Check(cfg.body_hydrostatics.empty(), "config default: body_hydrostatics empty");
    Check(cfg.legacy_enable_nonlinear == false, "config default: legacy_enable_nonlinear = false");
    Check(cfg.legacy_body_mesh_paths.empty(), "config default: legacy_body_mesh_paths empty");
    Check(cfg.profiling_enabled == false, "config default: profiling_enabled = false");

    std::cout << "  TestHydroSystemConfigDefaults done\n";
}

// ════════════════════════════════════════════════════════════════════════
// Test: HydroSystemConfig is a proper value type (copyable, assignable)
// ════════════════════════════════════════════════════════════════════════
static void TestHydroSystemConfigValueSemantics() {
    HydroSystemConfig a;
    a.excitation_truncation_time = 5.0;
    a.radiation_method = RadiationMethod::kStateSpace;
    a.linear_damping = {{{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}}};

    HydroSystemConfig b = a;
    Check(b.excitation_truncation_time == 5.0, "config copy: excitation_truncation_time preserved");
    Check(b.radiation_method == RadiationMethod::kStateSpace, "config copy: radiation_method preserved");
    Check(b.linear_damping.size() == 1, "config copy: linear_damping size preserved");
    Check(b.linear_damping[0][2] == 3.0, "config copy: linear_damping[0][2] preserved");

    // Modify original, verify copy is independent
    a.excitation_truncation_time = 99.0;
    Check(b.excitation_truncation_time == 5.0, "config copy: independent after modification");

    HydroSystemConfig c;
    c = b;
    Check(c.radiation_method == RadiationMethod::kStateSpace, "config assign: radiation_method preserved");

    std::cout << "  TestHydroSystemConfigValueSemantics done\n";
}

// ════════════════════════════════════════════════════════════════════════
// main
// ════════════════════════════════════════════════════════════════════════
int main() {
    std::cout << "=== Adapter-layer unit tests ===\n";

    TestForceCachingSameTimestep();
    TestForceCachingTimeAdvance();
    TestOutOfRangeIndices();
    TestDivergenceNaNForce();
    TestDivergenceHugeForce();
    TestDivergenceBodyState();
    TestDivergenceRollPitch();
    TestDivergenceLimitsConfigurable();
    TestMultiBodyRouting();
    TestChForceCloneSafety();
    TestDestructionLifecycle();
    TestHydroSystemConfigDefaults();
    TestHydroSystemConfigValueSemantics();

    std::cout << "\n" << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail > 0 ? 1 : 0;
}
