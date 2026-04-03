/*********************************************************************
 * @file  hydraulic_motor.h
 * @brief Fixed-displacement hydraulic motor model.
 *
 * Unit convention: displacement is stored in m^3/rad (NOT m^3/rev).
 * This means the flow and torque equations have no 2*pi factors:
 *   Q = D_m * omega * eta_vol
 *   T = D_m * delta_P * eta_mech
 *
 * If a datasheet gives displacement in m^3/rev, divide by 2*pi
 * before constructing HydraulicMotorParams.
 *
 * Stateless — no internal state.
 *
 * Solver-agnostic: C++ stdlib only (no Eigen, no Chrono).
 *********************************************************************/
#ifndef SEASTACK_PTO_HYDRAULIC_MOTOR_H
#define SEASTACK_PTO_HYDRAULIC_MOTOR_H

namespace seastack {
namespace pto {

struct HydraulicMotorParams {
    double displacement;     // D_m [m^3/rad]
    double mech_efficiency;  // eta_mech [-]
    double vol_efficiency;   // eta_vol  [-]
};

class HydraulicMotor {
  public:
    explicit HydraulicMotor(const HydraulicMotorParams& params);

    /// Volumetric flow consumed by the motor: Q = D_m * omega * eta_vol
    double ComputeFlow(double angular_velocity) const;

    /// Mechanical torque produced: T = D_m * delta_P * eta_mech
    double ComputeTorque(double pressure_diff) const;

    double displacement() const { return params_.displacement; }

  private:
    HydraulicMotorParams params_;
};

}  // namespace pto
}  // namespace seastack

#endif  // SEASTACK_PTO_HYDRAULIC_MOTOR_H
