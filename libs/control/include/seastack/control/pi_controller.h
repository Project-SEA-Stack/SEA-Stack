/*********************************************************************
 * @file  pi_controller.h
 * @brief PI controller with output clamping and conditional-integration
 *        anti-windup.
 *
 * Header-only implementation.  libs/control remains an INTERFACE library.
 *
 * Anti-windup strategy:
 *   1. Compute raw output:  u_raw = kp * error + ki * integral
 *   2. Clamp to limits:     u = clamp(u_raw, output_min, output_max)
 *   3. Conditional integration: skip accumulation when the output is
 *      saturated AND the error would worsen it.  Specifically, skip
 *      when (u_raw != u) AND sign(error) == sign(u_raw - u).
 *
 * Units:
 *   measurement — typically angular velocity [rad/s]
 *   output      — typically torque [N·m]
 *
 * Solver-agnostic: C++ stdlib only (no Eigen, no Chrono).
 *********************************************************************/
#ifndef SEASTACK_CONTROL_PI_CONTROLLER_H
#define SEASTACK_CONTROL_PI_CONTROLLER_H

#include <seastack/control/controller.h>

#include <algorithm>
#include <cmath>

namespace seastack {
namespace control {

struct PIControllerParams {
    double kp;
    double ki;
    double setpoint;
    double output_min;
    double output_max;
};

class PIController : public IController {
  public:
    explicit PIController(const PIControllerParams& params)
        : params_(params) {}

    double Compute(double measurement, double time) override {
        double error = measurement - params_.setpoint;
        last_error_ = error;

        if (prev_time_ >= 0.0) {
            double dt = time - prev_time_;
            if (dt > 0.0) {
                // Conditional integration: compute what the raw output
                // would be with the candidate new integral.
                double candidate_integral = integral_ + error * dt;
                double u_raw = params_.kp * error + params_.ki * candidate_integral;
                double u_clamped = std::clamp(u_raw, params_.output_min,
                                              params_.output_max);

                bool saturated = (u_raw != u_clamped);
                bool error_worsens = (error > 0.0 && u_raw > u_clamped) ||
                                     (error < 0.0 && u_raw < u_clamped);

                if (!saturated || !error_worsens) {
                    integral_ = candidate_integral;
                }
            }
        }
        prev_time_ = time;

        double u_raw = params_.kp * error + params_.ki * integral_;
        return std::clamp(u_raw, params_.output_min, params_.output_max);
    }

    void Reset() override {
        integral_ = 0.0;
        prev_time_ = -1.0;
        last_error_ = 0.0;
    }

    double integral() const { return integral_; }
    double last_error() const { return last_error_; }

  private:
    PIControllerParams params_;
    double integral_   = 0.0;
    double prev_time_  = -1.0;
    double last_error_ = 0.0;
};

}  // namespace control
}  // namespace seastack

#endif  // SEASTACK_CONTROL_PI_CONTROLLER_H
