/*********************************************************************
 * @file  excitation_component.h
 * @brief Wave excitation force component (frequency-domain).
 *
 * Computes excitation forces via frequency-domain superposition of the
 * excitation transfer function H(omega, beta) from H5 data and the
 * wave component amplitudes/phases/frequencies.
 *
 * This component owns the interpolated excitation transfer function data,
 * separating excitation force responsibility from the wave kinematic model.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_FORCE_COMPONENTS_EXCITATION_COMPONENT_H
#define SEASTACK_HYDRO_FORCE_COMPONENTS_EXCITATION_COMPONENT_H

#include <seastack/core/force_component.h>
#include <seastack/core/system_state.h>

#include <Eigen/Dense>
#include <vector>

namespace seastack::hydro {

/**
 * @brief Wave excitation force component (frequency-domain superposition).
 *
 * Owns interpolated excitation transfer function data and wave component
 * parameters. Computes forces as:
 *   F(t) = sum_i Re[H(omega_i, beta_i)* * A_i * exp(i*(phi_i - omega_i*t))]
 */
class ExcitationComponent : public IHydroForceComponent {
  public:
    /// Wave component data needed for force computation.
    struct WaveComponentData {
        Eigen::VectorXd amplitudes;   ///< A_i [m]
        Eigen::VectorXd omegas;       ///< omega_i [rad/s]
        Eigen::VectorXd phases;       ///< phi_i [rad]
    };

    /**
     * @brief Construct with excitation TF data from InterpolateExcitationTransfer.
     *
     * @param wave_data Wave component parameters (amplitudes, frequencies, phases)
     * @param excitation_re Per-body Re(H*) matrices, shape (6 x n_components)
     * @param excitation_im Per-body Im(H*) matrices, shape (6 x n_components)
     * @param num_bodies Number of bodies
     * @param ramp_duration Cosine ramp duration [s], 0 = no ramp
     */
    ExcitationComponent(WaveComponentData wave_data,
                        std::vector<Eigen::MatrixXd> excitation_re,
                        std::vector<Eigen::MatrixXd> excitation_im,
                        int num_bodies,
                        double ramp_duration = 0.0);

    HydroComponentType Type() const override { return HydroComponentType::kExcitation; }

    void Compute(const SystemState& state,
                 double time,
                 BodyForces& inout_forces) override;

  private:
    WaveComponentData wave_data_;
    std::vector<Eigen::MatrixXd> excitation_re_;
    std::vector<Eigen::MatrixXd> excitation_im_;
    int num_bodies_;
    double ramp_duration_ = 0.0;

    void ComputeFromTFData(double time, BodyForces& inout_forces);
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_FORCE_COMPONENTS_EXCITATION_COMPONENT_H
