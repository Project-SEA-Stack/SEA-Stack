/*********************************************************************
 * @file  nonlinear_hydrostatics_component.h
 * @brief Nonlinear hydrostatic force component (instantaneous submerged volume).
 *
 * Computes buoyancy force and moment from the instantaneous submerged
 * volume and centre of buoyancy, replacing the linear stiffness-based
 * approach entirely. Uses ISubmergedVolumeCalculator strategy per body.
 *
 * Physics:
 *   F_b = rho * |g| * V_sub * e_z
 *   M_b = (r_CB - r_ref) x F_b
 *
 * where V_sub and r_CB are computed from the instantaneous submerged
 * hull geometry using the divergence theorem (signed tetrahedron volume
 * formulation). See Faltinsen (1990) Ch. 3 for hydrostatic restoring
 * theory.
 *
 * v1 Assumptions:
 *   - Flat waterplane at z = waterplane_z (no wave-surface intersection)
 *   - Closed (watertight) body mesh in OBJ format with CCW outward normals
 *   - Mesh vertices are in the body-fixed CG frame (HydroModelBuilder translates
 *     from BEM frame to CG frame using HydroData::cg before constructing calculators)
 *   - Water density and gravity are uniform constants
 *   - Mutually exclusive with linear HydrostaticsComponent
 *
 * v1 Known Limitations:
 *   - Open/half-hull meshes (e.g. BEM wetted-surface meshes) are not
 *     supported. TODO: auto-cap at waterplane.
 *   - Flat waterplane only. Planned: WaveAwareMeshSubmergedVolume.
 *   - No nonlinear Froude-Krylov (deliberate scope exclusion).
 *   - No mesh coarsening for high-resolution meshes.
 *
 * Extensibility:
 *   Future geometry methods (AnalyticalSubmergedVolume, WaveAwareMesh)
 *   implement ISubmergedVolumeCalculator and plug into this component
 *   without modification.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_FORCE_COMPONENTS_NONLINEAR_HYDROSTATICS_COMPONENT_H
#define SEASTACK_HYDRO_FORCE_COMPONENTS_NONLINEAR_HYDROSTATICS_COMPONENT_H

#include <seastack/core/force_component.h>
#include <seastack/core/system_state.h>
#include <seastack/hydro/geometry/submerged_volume.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace seastack::hydro {

/**
 * @brief Nonlinear hydrostatic force component.
 *
 * Owns one ISubmergedVolumeCalculator per body. At each timestep, computes
 * the instantaneous submerged volume and centre of buoyancy, then applies
 * the resulting buoyancy force and moment.
 *
 * Coordinate frame: calculators receive geometry in the body CG frame; pose in
 * SystemState is the CG in world coordinates (consistent with Chrono and linear
 * hydrostatics equilibrium from HydroData::cg).
 */
class NonlinearHydrostaticsComponent : public IHydroForceComponent {
public:
    /**
     * @brief Constructor.
     *
     * @param calculators  One ISubmergedVolumeCalculator per active body (ownership transferred).
     *                     The i-th calculator corresponds to active_bodies[i].
     * @param rho          Water density [kg/m^3]
     * @param gravity      Gravity vector (e.g. [0, 0, -9.81])
     * @param waterplane_z Z-coordinate of the flat waterplane [m] (default: 0)
     * @param active_bodies Body indices this component operates on. If empty, the component
     *                      operates on bodies 0..calculators.size()-1 (backward compatible).
     */
    NonlinearHydrostaticsComponent(
        std::vector<std::unique_ptr<geometry::ISubmergedVolumeCalculator>> calculators,
        double rho,
        Eigen::Vector3d gravity,
        double waterplane_z = 0.0,
        std::vector<int> active_bodies = {});

    HydroComponentType Type() const override {
        return HydroComponentType::kNonlinearHydrostatics;
    }

    void Compute(const SystemState& state, double time,
                 BodyForces& inout_forces) override;

    /// Retrieve the most recently computed submerged volume for a body.
    double GetLastSubmergedVolume(int body_index) const;

    /// Retrieve the most recently computed centre of buoyancy for a body.
    Eigen::Vector3d GetLastCentreOfBuoyancy(int body_index) const;

private:
    std::vector<std::unique_ptr<geometry::ISubmergedVolumeCalculator>> calculators_;
    double rho_;
    Eigen::Vector3d gravity_;
    double waterplane_z_;
    std::vector<int> active_bodies_;  ///< Body indices this component operates on

    // System-sized diagnostic arrays (sparse: zeros for non-active bodies).
    // Indexed by system body index for consistent diagnostics access.
    std::vector<double> last_volume_;
    std::vector<Eigen::Vector3d> last_cob_;

};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_FORCE_COMPONENTS_NONLINEAR_HYDROSTATICS_COMPONENT_H
