/*********************************************************************
 * @file  moordyn_euler.h
 * @brief Convert SEA-Stack body orientations into MoorDyn's Euler convention.
 *
 * ROLE: MoorDyn's coupled-body interface takes an orientation as three Euler
 * angles. SEA-Stack and MoorDyn compose those angles in different orders, so
 * the triple must be converted rather than passed through.
 *********************************************************************/

#ifndef SEASTACK_MOORING_MOORDYN_EULER_H
#define SEASTACK_MOORING_MOORDYN_EULER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace seastack::mooring {

/**
 * @brief Convert a SEA-Stack body orientation into MoorDyn's Euler triple.
 *
 * Both codes name their angles "roll, pitch, yaw" but compose them in opposite
 * orders, so the same numbers describe different attitudes:
 *
 *   SEA-Stack `BodyState::orientation_rpy` (Chrono `GetCardanAnglesXYZ`)
 *       R = Rz(yaw) * Ry(pitch) * Rx(roll)
 *   MoorDyn coupled-body input (`Euler2Quat`, MoorDyn `Misc.hpp`)
 *       R = Rx(a) * Ry(b) * Rz(c)
 *
 * The two agree only to first order in the angles. Passing the triple through
 * unconverted misplaces the body-attached fairleads by an amount that grows
 * with tilt: ~0.3 deg of attitude error at 5 deg of tilt, ~7 deg at 20 deg, and
 * over 50 deg once the body approaches capsize.
 *
 * The returned triple is the *canonical* one, i.e. the same branch MoorDyn's
 * own `Quat2Euler` would produce, with the middle angle in [-pi/2, pi/2]. The
 * branch matters as well as the attitude: within a coupling step MoorDyn
 * extrapolates the body pose linearly in Euler space as
 * `r_ves + rd_ves * t` (see `Body::updateFairlead`). A branch that jumps by pi
 * between steps still rebuilds the correct attitude at t = 0, but corrupts that
 * extrapolation and injects spurious fairlead-tension spikes. Eigen's
 * `Matrix::eulerAngles(0, 1, 2)` is exact but not canonical — it flips branch
 * whenever roll changes sign — so it must not be used here.
 *
 * @param rpy Orientation as [roll, pitch, yaw] in the SEA-Stack convention [rad].
 * @return Angles [a, b, c] such that MoorDyn's Rx(a)*Ry(b)*Rz(c) reproduces the
 *         input orientation [rad].
 */
inline Eigen::Vector3d ToMoorDynEulerXYZ(const Eigen::Vector3d& rpy) {
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
            .toRotationMatrix();

    // For R = Rx(a)*Ry(b)*Rz(c):  R(0,2) = sin(b),  R(0,0) = cos(b)cos(c),
    // R(0,1) = -cos(b)sin(c),  R(1,2) = -sin(a)cos(b),  R(2,2) = cos(a)cos(b).
    Eigen::Vector3d angles;
    angles[1] = std::asin(std::clamp(rotation(0, 2), -1.0, 1.0));

    const double cos_b = std::hypot(rotation(0, 0), rotation(0, 1));
    constexpr double kGimbalTolerance = 1e-9;
    if (cos_b > kGimbalTolerance) {
        angles[0] = std::atan2(-rotation(1, 2), rotation(2, 2));
        angles[2] = std::atan2(-rotation(0, 1), rotation(0, 0));
    } else {
        // b = +/-90 deg: only (a - c) or (a + c) is observable. Assign the whole
        // rotation to a, matching MoorDyn's own degenerate-case choice.
        angles[0] = std::atan2(rotation(2, 1), rotation(1, 1));
        angles[2] = 0.0;
    }
    return angles;
}

}  // namespace seastack::mooring

#endif  // SEASTACK_MOORING_MOORDYN_EULER_H
