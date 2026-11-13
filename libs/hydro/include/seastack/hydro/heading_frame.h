/*********************************************************************
 * @file  heading_frame.h
 * @brief Heading frame used by the linear (BEM) force components.
 *
 * BEM coefficients are defined in the hull's own axes at zero yaw. The
 * heading frame is the world frame rotated about +z by the body's yaw, so
 * x points along the hull's heading and z stays vertical. With Cardan XYZ
 * angles, R = Rz(yaw) * Ry(pitch) * Rx(roll), so roll and pitch are the
 * hull's tilts about the heading-frame x and y axes. The linear
 * components express body velocities and displacements in this frame,
 * apply the unchanged BEM coefficients, and rotate the resulting force and
 * moment back to the world frame. Coefficients are not re-interpolated for
 * the yawed heading.
 *********************************************************************/

#pragma once

#include <seastack/core/system_state.h>

#include <Eigen/Geometry>

namespace seastack::hydro {

/// Rotation from the body's heading frame to the world frame [-].
/// Yaw is BodyState::orientation_rpy[2] (Cardan XYZ angle about z), in radians.
inline Eigen::Matrix3d HeadingToWorld(const BodyState& body) {
    return Eigen::AngleAxisd(body.orientation_rpy[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

}  // namespace seastack::hydro
