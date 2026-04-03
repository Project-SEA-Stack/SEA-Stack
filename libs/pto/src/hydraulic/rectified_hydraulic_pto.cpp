#include <seastack/pto/hydraulic/rectified_hydraulic_pto.h>

#include <seastack/infra/logging.h>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPressureWarningFactor = 3.0;
constexpr double kPaToMPa = 1e6;

}  // namespace

namespace seastack {
namespace pto {

RectifiedHydraulicPTO::RectifiedHydraulicPTO(
    const RectifiedHydraulicPTOParams& params,
    std::shared_ptr<seastack::control::IController> speed_controller)
    : cylinder_(params.cylinder),
      hp_accumulator_(params.hp_accumulator),
      lp_accumulator_(params.lp_accumulator),
      motor_(params.motor),
      controller_(std::move(speed_controller)),
      num_substeps_(params.num_substeps),
      generator_inertia_(params.generator_inertia),
      generator_damping_(params.generator_damping),
      velocity_smoothing_(params.velocity_smoothing),
      hp_precharge_pressure_(params.hp_accumulator.precharge_pressure) {}

double RectifiedHydraulicPTO::ComputeForce(double /*displacement*/,
                                           double velocity,
                                           double time) {
    // Time-step caching: only advance state on genuinely new time steps.
    // Repeated or backward calls (e.g. from HHT sub-evaluations) return
    // the cached force without modifying internal state.
    if (time <= prev_time_ && prev_time_ >= 0.0) {
        return cached_force_;
    }

    double dt = (prev_time_ >= 0.0) ? (time - prev_time_) : 0.0;

    if (dt > 0.0) {
        // Evaluate controller once per outer step (deliberate v1 decision).
        // Generator torque is held constant across all sub-steps.
        if (controller_) {
            generator_torque_ = controller_->Compute(motor_speed_, time);
        }

        AdvanceState(velocity, dt, time);
    }

    // Cylinder force with smoothed ideal rectified bridge.
    // smooth_sign(v) = v / sqrt(v^2 + eps^2) replaces the discontinuous
    // sign(v), eliminating MN-scale force jumps at velocity zero-crossings.
    double p_hp = hp_accumulator_.Pressure();
    double p_lp = lp_accumulator_.Pressure();
    double dp   = p_hp - p_lp;

    double eps2 = velocity_smoothing_ * velocity_smoothing_;
    double smooth_sign_v = velocity / std::sqrt(velocity * velocity + eps2);
    double force = -smooth_sign_v * dp * cylinder_.piston_area();

    prev_time_    = time;
    cached_force_ = force;
    last_piston_vel_ = velocity;

    return force;
}

void RectifiedHydraulicPTO::AdvanceState(double piston_velocity,
                                         double dt,
                                         double /*time*/) {
    double dt_sub = dt / num_substeps_;

    double eps2 = velocity_smoothing_ * velocity_smoothing_;
    double smooth_sign_v = piston_velocity
        / std::sqrt(piston_velocity * piston_velocity + eps2);

    for (int i = 0; i < num_substeps_; ++i) {
        // 1. Cylinder flow (signed)
        double q_cyl = cylinder_.ComputeFlow(piston_velocity);

        // 2. Smoothed rectification: smooth_abs(v) = v * smooth_sign(v)
        //    = v^2 / sqrt(v^2 + eps^2).  Nonneg, zero at v=0, approaches
        //    |A_p * v| for |v| >> eps.
        double q_rect = q_cyl * smooth_sign_v;
        last_flow_rate_ = q_rect;

        // 3. Motor flow consumption
        double q_motor = motor_.ComputeFlow(motor_speed_);

        // 4. Accumulator volume updates
        double dv_hp = (q_rect - q_motor) * dt_sub;
        double dv_lp = (q_motor - q_rect) * dt_sub;

        hp_accumulator_.SetOilVolume(hp_accumulator_.OilVolume() + dv_hp);
        lp_accumulator_.SetOilVolume(lp_accumulator_.OilVolume() + dv_lp);

        // 5. Pressures and motor torque
        double p_hp = hp_accumulator_.Pressure();
        double p_lp = lp_accumulator_.Pressure();
        double t_motor = motor_.ComputeTorque(p_hp - p_lp);
        last_motor_torque_ = t_motor;

        // 6. Generator / motor shaft dynamics
        //    J * d_omega/dt = T_motor - T_gen - B * omega
        double d_omega = (t_motor - generator_torque_
                          - generator_damping_ * motor_speed_)
                         / generator_inertia_;
        motor_speed_ += d_omega * dt_sub;

        // Prevent negative motor speed (motor cannot run in reverse
        // in this simple model).
        if (motor_speed_ < 0.0) motor_speed_ = 0.0;
    }

    if (!pressure_warning_emitted_) {
        double hp_p = hp_accumulator_.Pressure();
        if (hp_p > kPressureWarningFactor * hp_precharge_pressure_) {
            LOG_WARNING("[RectifiedHydraulicPTO] WARNING: HP pressure "
                        << hp_p / kPaToMPa << " MPa exceeds 3x precharge ("
                        << hp_precharge_pressure_ / kPaToMPa << " MPa). "
                        << "Consider increasing motor displacement, "
                        << "accumulator volume, or controller output limits.");
            pressure_warning_emitted_ = true;
        }
    }
}

PTODiagnostics RectifiedHydraulicPTO::GetDiagnostics() const {
    PTODiagnostics d{};
    d.hp_pressure       = hp_accumulator_.Pressure();
    d.lp_pressure       = lp_accumulator_.Pressure();
    d.hp_oil_volume     = hp_accumulator_.OilVolume();
    d.lp_oil_volume     = lp_accumulator_.OilVolume();
    d.motor_speed       = motor_speed_;
    d.motor_torque      = last_motor_torque_;
    d.generator_torque  = generator_torque_;
    d.cylinder_force    = cached_force_;
    d.piston_velocity   = last_piston_vel_;
    d.flow_rate         = last_flow_rate_;
    d.mechanical_power  = cached_force_ * last_piston_vel_;
    d.electrical_power  = generator_torque_ * motor_speed_;
    // controller_error cannot be computed accurately from the IController
    // interface (no setpoint accessor).  Callers with access to the concrete
    // controller (e.g. PIController::last_error()) should use that instead.
    d.controller_error  = 0.0;
    return d;
}

}  // namespace pto
}  // namespace seastack
