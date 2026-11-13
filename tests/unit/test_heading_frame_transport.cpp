/*********************************************************************
 * @file  test_heading_frame_transport.cpp
 * @brief Linear BEM forces follow the body's heading (yaw).
 *
 * BEM coefficients are defined in hull axes at zero yaw. For a body
 * yawed by psi, the linear components evaluate them in the heading frame
 * (world rotated by psi about +z) and rotate the result to world. With
 * psi = 90 deg the heading-frame y axis is world -x, so a pitch tilt or
 * pitch rate must produce a moment about world x scaled by the PITCH
 * coefficient, not the roll coefficient.
 *
 * Covers:
 *   1. Hydrostatics (Wigley H5, C55 >> C44): pitch tilt at yaw 90 deg.
 *   2. Constant-coefficient radiation: pitch rate and surge velocity at
 *      yaw 90 deg.
 *   3. Frequency-domain excitation: force rotated with the heading.
 *   4. Zero yaw: results identical to the untransformed world-frame formula.
 *********************************************************************/

#include <seastack/hydro/force_components/excitation_component.h>
#include <seastack/hydro/force_components/hydrostatics_component.h>
#include <seastack/hydro/force_components/radiation_cc_component.h>
#include <seastack/hydro/hydro_setup_helpers.h>
#include <seastack/hydro_io/h5_reader.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#ifndef SEASTACK_TEST_WIGLEY_H5_FILE
#error "SEASTACK_TEST_WIGLEY_H5_FILE must be defined"
#endif

namespace {

using seastack::hydro::BodyForces;
using seastack::hydro::HydroData;
using seastack::hydro::SystemState;

constexpr double kHalfPi = 1.5707963267948966;
constexpr double kGravity = 9.81;  // [m/s^2]

int g_failures = 0;

void Check(bool condition, const std::string& what) {
    std::cout << (condition ? "  PASS: " : "  FAIL: ") << what << "\n";
    if (!condition) ++g_failures;
}

bool Close(double actual, double expected, double tol) {
    return std::abs(actual - expected) <= tol;
}

// ── 1. Hydrostatics ─────────────────────────────────────────────────

/// Restoring moment [N m] for a body at its BEM CG with the given roll/pitch/yaw [rad].
Eigen::Vector3d HydrostaticMoment(const HydroData& data, const Eigen::Vector3d& rpy) {
    seastack::hydro::HydrostaticsComponent hs(
        data, 1, seastack::hydro::ComputeEquilibrium(data, 1),
        seastack::hydro::ComputeCbMinusCg(data, 1), Eigen::Vector3d(0.0, 0.0, -kGravity));
    SystemState state;
    state.bodies.resize(1);
    state.bodies[0].position = data.GetCGVector(0);
    state.bodies[0].orientation_rpy = rpy;
    BodyForces forces(1);
    hs.Compute(state, 0.0, forces);
    return forces[0].moment;
}

void TestHydrostatics() {
    std::cout << "\n[1] Hydrostatics: pitch tilt at yaw 90 deg (Wigley)\n";
    const HydroData data = seastack::hydro_io::H5FileInfo(SEASTACK_TEST_WIGLEY_H5_FILE, 1).ReadH5Data();
    const Eigen::MatrixXd c = data.GetLinMatrix(0);
    const double rho_g = data.GetRhoVal() * kGravity;
    const double c44 = rho_g * c(3, 3);
    const double c55 = rho_g * c(4, 4);
    Check(c55 > 5.0 * c44, "test hull has C55 >> C44");

    const double theta = 0.02;  // pitch [rad]
    const Eigen::Vector3d m0 = HydrostaticMoment(data, Eigen::Vector3d(0.0, theta, 0.0));
    Check(Close(m0.y(), -c55 * theta, 1e-9 * c55), "yaw 0: moment about world y = -C55 * theta");

    const Eigen::Vector3d m90 = HydrostaticMoment(data, Eigen::Vector3d(0.0, theta, kHalfPi));
    // Heading y = world -x, so M_heading.y = -C55*theta maps to M_world.x = +C55*theta.
    Check(Close(m90.x(), c55 * theta, 1e-6 * c55 * theta), "yaw 90: moment about world x = +C55 * theta");
    // Hydrostatics do not depend on heading: the yaw-90 moment is the yaw-0
    // moment rotated by 90 deg (this includes the small C45 coupling).
    const Eigen::Vector3d m0_rotated(-m0.y(), m0.x(), m0.z());
    Check((m90 - m0_rotated).norm() < 1e-6 * c55 * theta, "yaw 90: moment = yaw-0 moment rotated by 90 deg");
}

// ── 2. Constant-coefficient radiation ───────────────────────────────

void TestRadiation() {
    std::cout << "\n[2] Radiation (constant coefficients): yaw 90 deg\n";
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(6, 6);
    b.diagonal() << 10.0, 20.0, 30.0, 1.0, 40.0, 50.0;
    seastack::hydro::RadiationConstantCoefficientComponent comp(1, b);

    const double q = 0.1;  // pitch rate about heading y [rad/s]
    const double u = 2.0;  // surge velocity along heading x [m/s]
    SystemState state;
    state.bodies.resize(1);
    state.bodies[0].orientation_rpy = Eigen::Vector3d(0.0, 0.0, kHalfPi);
    state.bodies[0].angular_velocity = Eigen::Vector3d(-q, 0.0, 0.0);  // heading y = world -x
    state.bodies[0].linear_velocity = Eigen::Vector3d(0.0, u, 0.0);    // heading x = world +y
    BodyForces forces(1);
    comp.Compute(state, 0.0, forces);

    Check(Close(forces[0].moment.x(), b(4, 4) * q, 1e-12), "pitch rate damped by B55, about world x");
    Check(Close(forces[0].force.y(), -b(0, 0) * u, 1e-12), "surge velocity damped by B11, along world y");
    Check(forces[0].force.cwiseAbs().maxCoeff() == std::abs(forces[0].force.y()) &&
              forces[0].moment.cwiseAbs().maxCoeff() == std::abs(forces[0].moment.x()),
          "no leakage into other world components");

    SystemState zero_yaw;
    zero_yaw.bodies.resize(1);
    zero_yaw.bodies[0].angular_velocity = Eigen::Vector3d(0.01, 0.02, 0.03);
    zero_yaw.bodies[0].linear_velocity = Eigen::Vector3d(0.4, 0.5, 0.6);
    BodyForces f0(1);
    comp.Compute(zero_yaw, 0.0, f0);
    Eigen::VectorXd v(6);
    v << 0.4, 0.5, 0.6, 0.01, 0.02, 0.03;
    const Eigen::VectorXd expected = -b * v;
    Check((f0[0].force - expected.head<3>()).norm() < 1e-12 &&
              (f0[0].moment - expected.tail<3>()).norm() < 1e-12,
          "yaw 0: F = -B v unchanged");
}

// ── 3. Frequency-domain excitation ──────────────────────────────────

void TestExcitation() {
    std::cout << "\n[3] Excitation (frequency domain): yaw 90 deg\n";
    seastack::hydro::ExcitationComponent::WaveComponentData wave;
    wave.amplitudes = Eigen::VectorXd::Constant(1, 1.0);
    wave.omegas = Eigen::VectorXd::Constant(1, 0.6);
    wave.phases = Eigen::VectorXd::Zero(1);
    Eigen::MatrixXd re(6, 1);
    re << 1.0, 0.0, 3.0, 0.0, 5.0, 0.0;  // surge, heave, pitch per metre of amplitude
    const Eigen::MatrixXd im = Eigen::MatrixXd::Zero(6, 1);
    seastack::hydro::ExcitationComponent exc(wave, {re}, {im}, 1, 0.0);

    SystemState state;
    state.bodies.resize(1);
    state.bodies[0].orientation_rpy = Eigen::Vector3d(0.0, 0.0, kHalfPi);
    BodyForces forces(1);
    exc.Compute(state, 0.0, forces);
    Check(Close(forces[0].force.y(), 1.0, 1e-12) && Close(forces[0].force.x(), 0.0, 1e-12),
          "surge excitation along heading x = world +y");
    Check(Close(forces[0].force.z(), 3.0, 1e-12), "heave excitation unchanged");
    Check(Close(forces[0].moment.x(), -5.0, 1e-12) && Close(forces[0].moment.y(), 0.0, 1e-12),
          "pitch excitation about heading y = world -x");
}

}  // namespace

int main() {
    std::cout << "=== test_heading_frame_transport ===\n";
    try {
        TestHydrostatics();
        TestRadiation();
        TestExcitation();
    } catch (const std::exception& e) {
        std::cout << "  FAIL: threw: " << e.what() << "\n";
        ++g_failures;
    }
    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED") << " (" << g_failures
              << " failure(s))\n";
    return g_failures == 0 ? 0 : 1;
}
