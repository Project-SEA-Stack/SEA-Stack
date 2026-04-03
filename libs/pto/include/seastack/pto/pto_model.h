/*********************************************************************
 * @file  pto_model.h
 * @brief Abstract PTO (Power Take-Off) model interface.
 *
 * Solver-agnostic: depends only on SEAStack::Core (no Chrono, no HDF5).
 * A Chrono adapter would wrap this into ChLinkTSDA::RegisterForceFunctor().
 *********************************************************************/
#ifndef SEASTACK_PTO_PTO_MODEL_H
#define SEASTACK_PTO_PTO_MODEL_H

namespace seastack {
namespace pto {

/// Abstract interface for a 1-DOF PTO device.
///
/// Models the resistive/reactive force produced by a PTO as a function of
/// the relative displacement and velocity between two bodies (or a body
/// and ground), plus the current simulation time.
///
/// Units convention (SI):
///   displacement [m], velocity [m/s], time [s], force [N]
///   (or [rad], [rad/s], [N·m] for rotational PTOs)
class IPTOModel {
  public:
    virtual ~IPTOModel() = default;

    /// Compute PTO force/torque at the current state.
    /// @param displacement  Relative displacement (extension positive) [m or rad]
    /// @param velocity      Relative velocity [m/s or rad/s]
    /// @param time          Current simulation time [s]
    /// @return              Resistive force [N] or torque [N·m] (sign: opposes motion)
    virtual double ComputeForce(double displacement,
                                double velocity,
                                double time) = 0;
};

}  // namespace pto
}  // namespace seastack

#endif  // SEASTACK_PTO_PTO_MODEL_H
