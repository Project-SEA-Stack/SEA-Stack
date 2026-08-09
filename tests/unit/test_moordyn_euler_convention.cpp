/*********************************************************************
 * @file  test_moordyn_euler_convention.cpp
 * @brief Verify the SEA-Stack -> MoorDyn orientation conversion.
 *
 * Two properties are checked, because the RM3/MoorDyn verification case is
 * sensitive to both:
 *
 *   1. Exactness  — MoorDyn must rebuild the attitude SEA-Stack holds, at any
 *                   tilt (not just small angles).
 *   2. Continuity — the angle triple must stay on MoorDyn's canonical branch,
 *                   because MoorDyn extrapolates the pose linearly in Euler
 *                   space within each coupling step.
 *********************************************************************/

#include <seastack/mooring/moordyn_euler.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>

#include "test_macros.h"

namespace {

TestResults test_results;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

using seastack::mooring::ToMoorDynEulerXYZ;

/// SEA-Stack convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
Eigen::Matrix3d SeaStackRotation(const Eigen::Vector3d& rpy) {
    return (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

/// MoorDyn convention (Euler2Quat in MoorDyn Misc.hpp): R = Rx(a)*Ry(b)*Rz(c).
Eigen::Matrix3d MoorDynRotation(const Eigen::Vector3d& abc) {
    return (Eigen::AngleAxisd(abc[0], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(abc[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(abc[2], Eigen::Vector3d::UnitZ()))
        .toRotationMatrix();
}

/// Angle of the relative rotation between two attitudes [deg].
double AttitudeErrorDeg(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
    return Eigen::AngleAxisd(Eigen::Matrix3d(a.transpose() * b)).angle() / kDeg;
}

void TestRoundTripAtAllTilts() {
    std::cout << "\n-- MoorDyn rebuilds the SEA-Stack attitude exactly --\n";

    const double cases[][3] = {
        {0.0, 0.0, 0.0},      // at rest
        {1.5, 2.0, 0.8},      // RM3-scale motion
        {-1.5, 2.0, -0.8},    // negative roll (branch-flip trap)
        {5.0, 3.0, 2.0},   {20.0, 15.0, 10.0}, {-20.0, 15.0, -10.0},
        {60.0, 40.0, 30.0},   // heavily heeled
        {95.0, 20.0, 10.0},   // past capsize (WITT tip-over)
        {10.0, 89.0, 5.0},    // approaching the gimbal attitude
    };

    for (const auto& c : cases) {
        const Eigen::Vector3d rpy(c[0] * kDeg, c[1] * kDeg, c[2] * kDeg);
        const double err = AttitudeErrorDeg(SeaStackRotation(rpy),
                                            MoorDynRotation(ToMoorDynEulerXYZ(rpy)));
        TEST_NEAR(err, 0.0, 1e-9,
                  "attitude round-trip at roll/pitch/yaw = " + std::to_string(c[0]) +
                      "/" + std::to_string(c[1]) + "/" + std::to_string(c[2]) + " deg");
    }
}

void TestCanonicalBranch() {
    std::cout << "\n-- returned triple is on MoorDyn's canonical branch --\n";

    // MoorDyn's Quat2Euler always returns a middle angle in [-pi/2, pi/2].
    for (int i = 0; i <= 40; ++i) {
        for (int j = 0; j <= 8; ++j) {
            const Eigen::Vector3d rpy((-180.0 + i * 9.0) * kDeg,
                                      (-80.0 + j * 20.0) * kDeg, 37.0 * kDeg);
            const double middle = ToMoorDynEulerXYZ(rpy)[1];
            TEST_ASSERT(std::abs(middle) <= kPi / 2 + 1e-12,
                        "middle angle outside canonical [-90, 90] deg range");
        }
    }
}

void TestNoBranchJumpsThroughZeroRoll() {
    std::cout << "\n-- no branch jumps along an RM3-like trajectory --\n";

    // Small roll/pitch/yaw oscillating through zero, sampled at the coupling
    // step. A non-canonical decomposition jumps by ~pi here.
    Eigen::Vector3d previous = Eigen::Vector3d::Zero();
    double max_jump = 0.0;
    for (int n = 0; n <= 2000; ++n) {
        const double t = n * 0.01;
        const Eigen::Vector3d rpy(1.5 * std::sin(2 * kPi * t / 0.7) * kDeg,
                                  2.0 * std::sin(2 * kPi * t / 0.9 + 0.4) * kDeg,
                                  0.8 * std::sin(2 * kPi * t / 1.3 + 1.0) * kDeg);
        const Eigen::Vector3d angles = ToMoorDynEulerXYZ(rpy);
        if (n > 0) {
            max_jump = std::max(max_jump, (angles - previous).cwiseAbs().maxCoeff());
        }
        previous = angles;
    }
    std::cout << "   max step-to-step change: " << max_jump << " rad\n";
    TEST_ASSERT(max_jump < 0.01,
                "angle triple must vary smoothly across coupling steps");
}

void TestSmallAngleAgreement() {
    std::cout << "\n-- reduces to the identity map at small angles --\n";

    // At small tilt the two conventions agree to first order; this is why the
    // unconverted triple happened to match WEC-Sim for RM3.
    const Eigen::Vector3d rpy(0.5 * kDeg, 0.4 * kDeg, 0.3 * kDeg);
    const Eigen::Vector3d angles = ToMoorDynEulerXYZ(rpy);
    for (int i = 0; i < 3; ++i) {
        TEST_NEAR(angles[i], rpy[i], 1e-4, "small-angle agreement, component " +
                                               std::to_string(i));
    }
}

}  // namespace

int main() {
    std::cout << "========================================\n";
    std::cout << "MoorDyn Euler convention tests\n";
    std::cout << "========================================\n";

    TestRoundTripAtAllTilts();
    TestCanonicalBranch();
    TestNoBranchJumpsThroughZeroRoll();
    TestSmallAngleAgreement();

    test_results.Summary();
    return test_results.failed == 0 ? 0 : 1;
}
