/*********************************************************************
 * @file  system_state.h
 * @brief Chrono-free state structs for hydrodynamic computation.
 *
 * MAIN TYPES:
 *   - BodyState: Position, orientation, velocities for one body
 *   - SystemState: Vector of BodyState for all bodies
 *
 * ROLE: Input to HydroSystem and force components. Decouples core
 * hydrodynamics from Chrono's ChBody representation.
 *********************************************************************/

#ifndef SEASTACK_CORE_SYSTEM_STATE_H
#define SEASTACK_CORE_SYSTEM_STATE_H

#include <Eigen/Dense>
#include <vector>
#include <seastack/core/types.h>

namespace seastack::hydro {

/**
 * @brief State of a single body (pose and velocities).
 * 
 * Contains position, orientation (as RPY angles), linear velocity,
 * and angular velocity for one body in the system.
 */
struct BodyState {
    /// Position of the body CG in the world (inertial) frame [m], 3-vector (x, y, z).
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    /// Orientation as roll-pitch-yaw Euler angles in the world frame [rad]. Convention:
    /// intrinsic ZYX rotation (yaw about Z, then pitch about Y', then roll about X'').
    /// Order in the vector is [roll, pitch, yaw].
    Eigen::Vector3d orientation_rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();  // Linear velocity in world frame (m/s)
    /// Angular velocity in the world (inertial) frame [rad/s], 3-vector.
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

/**
 * @brief State of all bodies in the hydrodynamic system.
 * 
 * Contains a vector of BodyState, one per body.
 */
struct SystemState {
    std::vector<BodyState> bodies;
};

}  // namespace seastack::hydro

#endif  // SEASTACK_CORE_SYSTEM_STATE_H

