/*********************************************************************
 * @file  controller.h
 * @brief Abstract controller interface for WEC control.
 *
 * Solver-agnostic: depends only on SEAStack::Core (no Chrono, no HDF5).
 *********************************************************************/
#ifndef SEASTACK_CONTROL_CONTROLLER_H
#define SEASTACK_CONTROL_CONTROLLER_H

namespace seastack {
namespace control {

/// Abstract interface for a feedback controller.
///
/// Takes a measurement (e.g., displacement, velocity, or force) and
/// computes a control command (e.g., PTO damping setpoint, force command).
///
/// Units are application-dependent; document them in each concrete class.
class IController {
  public:
    virtual ~IController() = default;

    /// Compute control output from current measurement and time.
    /// @param measurement  Sensor reading (units depend on implementation)
    /// @param time         Current simulation time [s]
    /// @return             Control command (units depend on implementation)
    virtual double Compute(double measurement, double time) = 0;

    /// Reset internal state (e.g., integrator windup).
    virtual void Reset() {}
};

}  // namespace control
}  // namespace seastack

#endif  // SEASTACK_CONTROL_CONTROLLER_H
