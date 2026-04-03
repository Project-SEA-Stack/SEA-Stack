/*********************************************************************
 * @file  rectified_hydraulic_pto.h
 * @brief Rectified hydraulic PTO assembly with PI speed control.
 *
 * Composes HydraulicCylinder, two HydraulicAccumulators (HP/LP),
 * HydraulicMotor, and an optional IController into a single IPTOModel.
 *
 * Physics model (v1 — smoothed ideal rectification):
 *   - Check-valve bridge uses smooth regularization near v=0:
 *       force direction: smooth_sign(v) = v / sqrt(v^2 + eps^2)
 *       rectified flow:  smooth_abs(v)  = v * smooth_sign(v)
 *     Both derive from the same velocity_smoothing parameter (eps).
 *   - No cylinder chamber pressure states.
 *   - Accumulator charge/discharge via polytropic gas model.
 *   - Fixed-displacement motor driven by HP-LP differential.
 *   - Generator modelled as inertia + viscous damping + controller torque.
 *   - Internal state advanced via forward Euler sub-stepping.
 *
 * Solver-agnostic: C++ stdlib only (no Eigen, no Chrono).
 *********************************************************************/
#ifndef SEASTACK_PTO_HYDRAULIC_RECTIFIED_HYDRAULIC_PTO_H
#define SEASTACK_PTO_HYDRAULIC_RECTIFIED_HYDRAULIC_PTO_H

#include <seastack/pto/pto_model.h>
#include <seastack/pto/hydraulic/hydraulic_cylinder.h>
#include <seastack/pto/hydraulic/hydraulic_accumulator.h>
#include <seastack/pto/hydraulic/hydraulic_motor.h>
#include <seastack/control/controller.h>

#include <memory>

namespace seastack {
namespace pto {

struct RectifiedHydraulicPTOParams {
    HydraulicCylinderParams cylinder;
    AccumulatorParams       hp_accumulator;
    AccumulatorParams       lp_accumulator;
    HydraulicMotorParams    motor;
    double generator_inertia;       // J [kg*m^2]
    double generator_damping;       // B [N*m*s/rad]
    int    num_substeps = 10;
    double velocity_smoothing = 0.01;  // eps [m/s] — smoothing band for
                                       // check-valve bridge regularization.
                                       // Force and flow transition smoothly
                                       // within |v| < ~eps instead of jumping.
};

struct PTODiagnostics {
    double hp_pressure;         // [Pa]
    double lp_pressure;         // [Pa]
    double hp_oil_volume;       // [m^3]
    double lp_oil_volume;       // [m^3]
    double motor_speed;         // [rad/s]
    double motor_torque;        // [N*m]
    double generator_torque;    // [N*m]
    double cylinder_force;      // [N]
    double piston_velocity;     // [m/s] (last input)
    double flow_rate;           // [m^3/s] (rectified flow to HP)
    double mechanical_power;    // [W] (F_pto * v_piston)
    double electrical_power;    // [W] (T_gen * omega)
    double controller_error;    // [rad/s] (omega - setpoint, 0 if no controller)
};

class RectifiedHydraulicPTO : public IPTOModel {
  public:
    /// @param params            PTO configuration.
    /// @param speed_controller  Optional controller for generator torque.
    ///                          If null, generator torque is zero.
    RectifiedHydraulicPTO(
        const RectifiedHydraulicPTOParams& params,
        std::shared_ptr<seastack::control::IController> speed_controller = nullptr);

    double ComputeForce(double displacement,
                        double velocity,
                        double time) override;

    PTODiagnostics GetDiagnostics() const;

  private:
    void AdvanceState(double piston_velocity, double dt, double time);

    HydraulicCylinder     cylinder_;
    HydraulicAccumulator  hp_accumulator_;
    HydraulicAccumulator  lp_accumulator_;
    HydraulicMotor        motor_;
    std::shared_ptr<seastack::control::IController> controller_;

    double motor_speed_       = 0.0;
    double generator_torque_  = 0.0;
    double prev_time_         = -1.0;
    double cached_force_      = 0.0;
    double last_piston_vel_   = 0.0;
    double last_flow_rate_    = 0.0;
    double last_motor_torque_ = 0.0;
    int    num_substeps_;

    double generator_inertia_;
    double generator_damping_;
    double velocity_smoothing_;
    double hp_precharge_pressure_;
    bool   pressure_warning_emitted_ = false;
};

}  // namespace pto
}  // namespace seastack

#endif  // SEASTACK_PTO_HYDRAULIC_RECTIFIED_HYDRAULIC_PTO_H
