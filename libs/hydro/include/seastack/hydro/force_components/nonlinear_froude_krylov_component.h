/*********************************************************************
 * @file  nonlinear_froude_krylov_component.h
 * @brief Nonlinear Froude-Krylov force component: static and incident-wave
 *        pressure on the instantaneous wetted surface, minus the mesh linear
 *        Froude-Krylov force already contained in the BEM excitation.
 *
 * Theory, conventions and pass criteria: docs/theory/NONLINEAR_FROUDE_KRYLOV.md.
 *
 * Physics (per body, world axes, moment about the body CG):
 *   F = F_p(S_w(t)) - F_FK,lin(t)
 *   F_p     = -int_{S_w} p n dS,  p = -rho g z + p_d,  S_w = hull below eta(x, y, t)
 *   F_FK,lin = r(t) sum_i a_i (A_i cos(theta_i) + B_i sin(theta_i)),  theta_i = phi_i - w_i t
 * F_p contains the static buoyancy, so this component replaces
 * NonlinearHydrostaticsComponent for its bodies. Radiation and BEM
 * excitation (FK + diffraction) are applied by their own components.
 *
 * Pressure models:
 *   kAiryWheeler  p_d = rho g r(t) * WaveBase::GetDynamicPressureHead (Wheeler stretched)
 *   kHydrostatic  p_d = rho g eta (long-wave limit: p = rho g (eta - z))
 *
 * Assumptions:
 *   - Closed mesh, outward normals, vertices in the body CG frame (the builder
 *     shifts the BEM-frame OBJ by -HydroData::cg).
 *   - BEM frame = world frame: A_i, B_i are computed with the body at its BEM
 *     CG position, like the BEM excitation. A one-time warning is logged if a
 *     body starts more than 1% of the shortest wavelength away from it.
 *   - The ramp r(t) matches the excitation component: cosine for
 *     frequency-domain excitation, linear for excitation IRF convolution.
 *   - Linear incident wave, undisturbed by the body.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_FORCE_COMPONENTS_NONLINEAR_FROUDE_KRYLOV_COMPONENT_H
#define SEASTACK_HYDRO_FORCE_COMPONENTS_NONLINEAR_FROUDE_KRYLOV_COMPONENT_H

#include <seastack/core/force_component.h>
#include <seastack/core/system_state.h>
#include <seastack/hydro/config/hydro_config.h>
#include <seastack/hydro/geometry/wetted_surface_pressure.h>
#include <seastack/hydro/waves/wave_base.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace seastack::hydro {

class NonlinearFroudeKrylovComponent : public IHydroForceComponent {
  public:
    /// Shape of the start-up ramp r(t); must match the excitation component.
    enum class RampShape {
        kCosine,  ///< 0.5 (1 - cos(pi t / T_r)), frequency-domain excitation
        kLinear   ///< t / T_r, excitation IRF convolution
    };

    /// One active body.
    struct BodySetup {
        int body_index = 0;                      ///< System body index
        geometry::TriangleMesh mesh;             ///< Closed hull, body CG frame [m]
        FroudeKrylovPressureModel pressure_model = FroudeKrylovPressureModel::kAiryWheeler;
        Eigen::Vector3d bem_cg = Eigen::Vector3d::Zero();  ///< CG at the BEM position, world frame [m]
    };

    /**
     * @param bodies         Active bodies (at least one).
     * @param wave           Incident wave; nullptr or NoWave means still water.
     *                       Waves must expose discrete components (GetWaveComponents).
     * @param rho            Water density [kg/m^3].
     * @param gravity        Gravity vector [m/s^2], e.g. (0, 0, -9.81).
     * @param ramp_duration  Ramp duration T_r [s]; 0 = no ramp.
     * @param ramp_shape     Ramp shape, matching the excitation component.
     * @throws std::invalid_argument / std::runtime_error on invalid input or open meshes.
     */
    NonlinearFroudeKrylovComponent(std::vector<BodySetup> bodies,
                                   std::shared_ptr<WaveBase> wave,
                                   double rho,
                                   const Eigen::Vector3d& gravity,
                                   double ramp_duration,
                                   RampShape ramp_shape);

    HydroComponentType Type() const override { return HydroComponentType::kNonlinearFroudeKrylov; }

    void Compute(const SystemState& state, double time, BodyForces& inout_forces) override;

    /// Ramp factor r(t) applied to eta, p_d and the linear FK.
    double RampFactor(double time) const;

    /// Diagnostics from the last Compute(), indexed by system body index
    /// (zero for bodies this component does not handle).
    double GetLastWettedArea(int body_index) const;
    /// Pressure resultant F_p, M_p (6-vector: force, moment about the CG).
    Eigen::Matrix<double, 6, 1> GetLastPressureForce(int body_index) const;
    /// Subtracted mesh linear Froude-Krylov force (6-vector).
    Eigen::Matrix<double, 6, 1> GetLastLinearFroudeKrylovForce(int body_index) const;

  private:
    struct BodyData {
        int body_index;
        FroudeKrylovPressureModel pressure_model;
        Eigen::Vector3d bem_cg;
        geometry::WettedSurfacePressureIntegrator integrator;
        Eigen::MatrixXd linear_fk_cos;  ///< 6 x n_components: A_i (per unit amplitude)
        Eigen::MatrixXd linear_fk_sin;  ///< 6 x n_components: B_i (per unit amplitude)
        bool warned_emerged = false;
        bool checked_position = false;
    };

    void ComputeLinearFroudeKrylovCoefficients(BodyData& body) const;
    void CheckInitialPosition(BodyData& body, const Eigen::Vector3d& position);

    std::vector<BodyData> bodies_;
    std::shared_ptr<WaveBase> wave_;  ///< nullptr in still water
    double rho_g_;
    double ramp_duration_;
    RampShape ramp_shape_;

    // Wave components (copied once; empty in still water).
    Eigen::VectorXd amplitudes_, omegas_, phases_, kx_, ky_;
    double shortest_wavelength_ = 0.0; ///< [m]; 0 in still water

    // Per-vertex work buffers (reused between calls).
    Eigen::Matrix3Xd vertex_buffer_;  ///< World-frame vertices [m]
    Eigen::VectorXd height_buffer_, pressure_buffer_, zero_buffer_;
    Eigen::VectorXd eta_buffer_, head_buffer_;  ///< Ramped elevation, pressure head [m]

    // System-indexed diagnostics.
    std::vector<double> last_wetted_area_;
    std::vector<Eigen::Matrix<double, 6, 1>> last_pressure_force_;
    std::vector<Eigen::Matrix<double, 6, 1>> last_linear_fk_;
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_FORCE_COMPONENTS_NONLINEAR_FROUDE_KRYLOV_COMPONENT_H
