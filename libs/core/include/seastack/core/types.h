/*********************************************************************
 * @file  hydro_types.h
 * @brief Core type aliases for hydrodynamic forces.
 *
 * MAIN TYPES:
 *   - GeneralizedForce: 6-DOF force vector for one body (force + moment)
 *   - BodyForces: Vector of GeneralizedForce for all bodies
 *
 * ROLE: Canonical definitions used throughout the hydrodynamics core.
 *********************************************************************/

#ifndef SEASTACK_CORE_TYPES_H
#define SEASTACK_CORE_TYPES_H

#include <Eigen/Dense>
#include <vector>

namespace seastack::hydro {

/// Degrees of freedom per rigid body: surge, sway, heave, roll, pitch, yaw.
constexpr int kDofPerBody = 6;

/**
 * @brief Generalized force vector for one body (6 DOF).
 * 
 * Separates translational forces [Fx, Fy, Fz] from rotational moments [Mx, My, Mz]
 * for surge, sway, heave, roll, pitch, yaw degrees of freedom.
 * 
 * This struct provides type safety (fixed-size) and semantic clarity compared to
 * a flat 6-element vector. Forces and moments are different physical quantities
 * (N vs N.m) and are now explicitly separated.
 */
struct GeneralizedForce {
    Eigen::Vector3d force  = Eigen::Vector3d::Zero();  // [Fx, Fy, Fz] in N
    Eigen::Vector3d moment = Eigen::Vector3d::Zero();   // [Mx, My, Mz] in N.m

    void setZero() { force.setZero(); moment.setZero(); }

    // Flatten to 6-DOF vector [Fx,Fy,Fz,Mx,My,Mz] for external interfaces
    Eigen::Matrix<double, 6, 1> ToVector6d() const {
        Eigen::Matrix<double, 6, 1> v;
        v.head<3>() = force;
        v.tail<3>() = moment;
        return v;
    }

    GeneralizedForce& operator+=(const GeneralizedForce& rhs) {
        force  += rhs.force;
        moment += rhs.moment;
        return *this;
    }
    GeneralizedForce& operator-=(const GeneralizedForce& rhs) {
        force  -= rhs.force;
        moment -= rhs.moment;
        return *this;
    }
    friend GeneralizedForce operator+(GeneralizedForce lhs, const GeneralizedForce& rhs) {
        lhs += rhs;
        return lhs;
    }
    friend GeneralizedForce operator-(GeneralizedForce lhs, const GeneralizedForce& rhs) {
        lhs -= rhs;
        return lhs;
    }
    friend GeneralizedForce operator*(double s, GeneralizedForce gf) {
        gf.force  *= s;
        gf.moment *= s;
        return gf;
    }
    friend GeneralizedForce operator*(GeneralizedForce gf, double s) {
        return s * gf;
    }
};

/**
 * @brief Forces for all bodies in the system.
 * 
 * Vector of GeneralizedForce, one per body.
 */
using BodyForces = std::vector<GeneralizedForce>;

}  // namespace seastack::hydro

#endif  // SEASTACK_CORE_TYPES_H

