/*********************************************************************
 * @file  radiation_cc_component.h
 * @brief Radiation damping force component (constant frequency-domain
 *        coefficients at the regular-wave frequency).
 *
 * Used with RadiationMethod::kConstantCoefficients for monochromatic regular
 * waves. Applies the damping part of
 *     F_rad = -A(w) * xddot - B(w) * xdot
 * with the full coupled 6N x 6N radiation damping matrix B(w) at the matched
 * BEM grid frequency. The added-mass part -A(w) * xddot is NOT applied here:
 * it enters the Chrono system mass matrix through the implicit added-mass
 * load, which then holds A(w) instead of A_inf (see HydroSystem).
 *
 * Velocity convention (same as the RIRF and state-space components): per
 * body, [linear velocity of the body reference point; angular velocity],
 * world frame, in DOF order surge, sway, heave, roll, pitch, yaw.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_FORCE_COMPONENTS_RADIATION_CC_COMPONENT_H
#define SEASTACK_HYDRO_FORCE_COMPONENTS_RADIATION_CC_COMPONENT_H

#include <seastack/core/force_component.h>
#include <seastack/core/system_state.h>

#include <Eigen/Dense>

namespace seastack::hydro {

/**
 * @brief Radiation damping force -B(w) * xdot with constant coefficients.
 */
class RadiationConstantCoefficientComponent : public IHydroForceComponent {
  public:
    /**
     * @param num_bodies Number of hydrodynamic bodies N.
     * @param radiation_damping Full coupled radiation damping B(w) [N.s/m,
     *        N.s, N.m.s/rad], 6N x 6N, dimensional (see
     *        HydroData::GetRadiationDampingAtFrequencyIndex()).
     */
    RadiationConstantCoefficientComponent(int num_bodies,
                                          const Eigen::MatrixXd& radiation_damping);

    HydroComponentType Type() const override { return HydroComponentType::kRadiation; }

    void Compute(const SystemState& state, double time, BodyForces& inout_forces) override;

    const Eigen::MatrixXd& radiation_damping() const { return radiation_damping_; }

  private:
    int num_bodies_;
    Eigen::MatrixXd radiation_damping_;  ///< B(w), 6N x 6N
    Eigen::VectorXd velocity_buffer_;    ///< Stacked body velocities, 6N
    Eigen::VectorXd force_buffer_;       ///< B(w) * velocity, 6N
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_FORCE_COMPONENTS_RADIATION_CC_COMPONENT_H
