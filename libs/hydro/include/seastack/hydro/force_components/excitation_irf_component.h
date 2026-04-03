/*********************************************************************
 * @file  excitation_irf_component.h
 * @brief Wave excitation force via IRF convolution with elevation history.
 *
 * Computes excitation forces using the time-domain integral:
 *
 *   F_exc(t) = integral_0^T_max  K_exc(tau) * eta(t - tau) dtau
 *
 * where K_exc is the excitation impulse response function from the H5
 * file and eta is the free-surface elevation (queried from the wave
 * object at the origin).
 *
 * This component reproduces the force computation of the removed
 * IrregularWaves class and is intended for long-crested seas where
 * backward compatibility with the IRF-calibrated regression tests is
 * required.  For directional / multi-heading seas, use the
 * frequency-domain ExcitationComponent instead.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_FORCE_COMPONENTS_EXCITATION_IRF_COMPONENT_H
#define SEASTACK_HYDRO_FORCE_COMPONENTS_EXCITATION_IRF_COMPONENT_H

#include <seastack/core/force_component.h>
#include <seastack/core/system_state.h>
#include <seastack/hydro/waves/wave_base.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace seastack::hydro {

class HydroData;

class ExcitationIrfComponent : public IHydroForceComponent {
  public:
    /**
     * @param data        HydroData containing excitation IRF kernels (per-body).
     * @param waves       Wave object used to query eta(t) at each time step.
     * @param num_bodies  Number of hydrodynamic bodies.
     * @param truncation_time  Maximum IRF lag [s].  0 = use full kernel.
     * @param ramp_duration    Ramp duration [s].  0 = no ramp.
     *                         Applied per-sample inside the convolution:
     *                         eta(t-tau) *= RampFactor(t-tau).
     */
    ExcitationIrfComponent(const HydroData& data,
                           std::shared_ptr<WaveBase> waves,
                           int num_bodies,
                           double truncation_time = 0.0,
                           double ramp_duration = 0.0);

    HydroComponentType Type() const override { return HydroComponentType::kExcitation; }

    void Compute(const SystemState& state,
                 double time,
                 BodyForces& inout_forces) override;

  private:
    const HydroData& data_;
    std::shared_ptr<WaveBase> waves_;
    int num_bodies_;
    double ramp_duration_;

    // Per-body excitation IRF data (copied once, optionally truncated).
    struct BodyIrf {
        Eigen::VectorXd time_vec;    // tau grid [s], length N
        Eigen::VectorXd width_vec;   // trapezoidal quadrature weights
        Eigen::MatrixXd kernel;      // K_exc(dof, step), 6 x N
    };
    std::vector<BodyIrf> body_irfs_;

    /// Linear ramp: 0 at t<=0, 1 at t>=ramp_duration_.
    double RampFactor(double t) const;

    // Fast-path data for LDWF waves (precomputed from component list).
    bool has_fast_path_ = false;
    Eigen::VectorXd comp_amplitudes_;   // A_i, length N_comp
    Eigen::VectorXd comp_omegas_;       // omega_i
    Eigen::VectorXd comp_phases_;       // phi_i
    Eigen::MatrixXd irf_cos_wt_;        // cos(omega_i * tau_j), shape (N_irf, N_comp)
    Eigen::MatrixXd irf_sin_wt_;        // sin(omega_i * tau_j), shape (N_irf, N_comp)

    // Reusable buffer for eta_at_tau to avoid per-timestep allocation
    Eigen::VectorXd eta_at_tau_buffer_;

    void TryBuildFastPath();
    void ComputeFastPath(double time, Eigen::VectorXd& eta_at_tau) const;
    void ComputeSlowPath(double time, Eigen::VectorXd& eta_at_tau) const;
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_FORCE_COMPONENTS_EXCITATION_IRF_COMPONENT_H
