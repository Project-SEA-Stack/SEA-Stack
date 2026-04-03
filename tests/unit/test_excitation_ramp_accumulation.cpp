/*********************************************************************
 * @file  test_excitation_ramp_accumulation.cpp
 * @brief Regression test: excitation ramp must not zero pre-accumulated forces.
 *
 * Validates that ExcitationComponent::Compute() accumulates into
 * inout_forces rather than overwriting it, especially during the
 * cosine ramp period (t < ramp_duration).
 *
 * Background: a bug in ComputeFromTFData() applied the ramp multiplier
 * to the entire inout_forces vector instead of only the excitation
 * contribution, zeroing buoyancy and other pre-accumulated forces
 * at t=0 (ramp=0).
 *
 * Self-contained — no external data files or Chrono required.
 *********************************************************************/

#include <seastack/hydro/force_components/excitation_component.h>
#include <seastack/core/system_state.h>
#include <seastack/core/types.h>

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace seastack::hydro;

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

static void CheckNear(double actual, double expected, double tol,
                       const std::string& label) {
    if (std::abs(actual - expected) <= tol) {
        ++g_pass;
    } else {
        ++g_fail;
        std::cerr << "FAIL: " << label
                  << " (expected " << expected << ", got " << actual
                  << ", tol=" << tol << ")\n";
    }
}

static ExcitationComponent MakeTwoBodyExcitationWithRamp(double ramp_duration) {
    constexpr int n_bodies = 2;
    constexpr int n_components = 3;

    ExcitationComponent::WaveComponentData wave_data;
    wave_data.amplitudes = Eigen::VectorXd::Constant(n_components, 1.0);
    wave_data.omegas     = Eigen::VectorXd::LinSpaced(n_components, 0.5, 1.5);
    wave_data.phases     = Eigen::VectorXd::Zero(n_components);

    std::vector<Eigen::MatrixXd> exc_re(n_bodies);
    std::vector<Eigen::MatrixXd> exc_im(n_bodies);
    for (int b = 0; b < n_bodies; ++b) {
        exc_re[b] = Eigen::MatrixXd::Constant(kDofPerBody, n_components, 100.0);
        exc_im[b] = Eigen::MatrixXd::Zero(kDofPerBody, n_components);
    }

    return ExcitationComponent(std::move(wave_data),
                               std::move(exc_re), std::move(exc_im),
                               n_bodies, ramp_duration);
}

/// At t=0 with a ramp, the excitation contribution should be zero,
/// but pre-existing forces in inout_forces must be preserved.
static void TestRampPreservesPreAccumulatedForces() {
    constexpr double ramp_duration = 60.0;
    auto exc = MakeTwoBodyExcitationWithRamp(ramp_duration);

    constexpr double buoyancy_body1 = 9.35e6;
    constexpr double buoyancy_body2 = 6.68e5;

    BodyForces forces(2);
    forces[0].force  = Eigen::Vector3d(0, 0, buoyancy_body1);
    forces[0].moment = Eigen::Vector3d(0, 0, 0);
    forces[1].force  = Eigen::Vector3d(0, 0, buoyancy_body2);
    forces[1].moment = Eigen::Vector3d(0, 0, 0);

    SystemState state;
    state.bodies.resize(2);

    exc.Compute(state, 0.0, forces);

    CheckNear(forces[0].force.z(), buoyancy_body1, 1.0,
              "body1 buoyancy preserved at t=0 (ramp=0)");
    CheckNear(forces[1].force.z(), buoyancy_body2, 1.0,
              "body2 buoyancy preserved at t=0 (ramp=0)");
}

/// At t > 0 during the ramp period, excitation should add a small
/// contribution without destroying the pre-existing buoyancy.
static void TestRampAccumulatesDuringRamp() {
    constexpr double ramp_duration = 60.0;
    auto exc = MakeTwoBodyExcitationWithRamp(ramp_duration);

    constexpr double buoyancy = 1.0e6;

    BodyForces forces(2);
    forces[0].force = Eigen::Vector3d(0, 0, buoyancy);
    forces[1].force = Eigen::Vector3d(0, 0, buoyancy);

    SystemState state;
    state.bodies.resize(2);

    exc.Compute(state, 1.0, forces);

    Check(forces[0].force.z() > buoyancy * 0.99,
          "body1 buoyancy mostly preserved at t=1 (early ramp)");
    Check(forces[1].force.z() > buoyancy * 0.99,
          "body2 buoyancy mostly preserved at t=1 (early ramp)");
}

/// With no ramp (ramp_duration=0), excitation adds full contribution
/// without overwriting pre-existing forces.
static void TestNoRampAccumulates() {
    auto exc = MakeTwoBodyExcitationWithRamp(0.0);

    constexpr double buoyancy = 5.0e5;

    BodyForces forces(2);
    forces[0].force = Eigen::Vector3d(0, 0, buoyancy);
    forces[1].force = Eigen::Vector3d(0, 0, buoyancy);

    SystemState state;
    state.bodies.resize(2);

    exc.Compute(state, 0.0, forces);

    Check(forces[0].force.z() > buoyancy - 1.0,
          "body1 buoyancy preserved with no ramp");
    Check(forces[1].force.z() > buoyancy - 1.0,
          "body2 buoyancy preserved with no ramp");
}

int main() {
    TestRampPreservesPreAccumulatedForces();
    TestRampAccumulatesDuringRamp();
    TestNoRampAccumulates();

    std::cout << "\n========================================\n"
              << "Results: " << g_pass << " passed, " << g_fail << " failed\n"
              << "========================================\n";
    return (g_fail == 0) ? 0 : 1;
}
