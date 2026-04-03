/*********************************************************************
 * @file  nonlinear_hydrostatics_component.cpp
 * @brief Nonlinear hydrostatic force component implementation.
 *
 * Per-body per-timestep algorithm:
 *   1. Build Eigen::Affine3d from BodyState (position + RPY -> rotation + translation)
 *   2. result = calculator->Compute(body_to_world, waterplane_z)
 *   3. F_b = rho * |g| * result.volume * e_z  (upward buoyancy)
 *   4. M_b = (result.centroid - body_position) x F_b
 *   5. Accumulate into inout_forces
 *
 * Moment reference: moments are accumulated about the body position
 * (origin of the body frame in world coordinates), consistent with the
 * linear HydrostaticsComponent which applies moments about the CG via
 * cb_minus_cg, and with ChronoForceAttacher which applies forces/torques at
 * the body reference frame.
 *********************************************************************/

#include <seastack/hydro/force_components/nonlinear_hydrostatics_component.h>

#include <cmath>
#include <stdexcept>

namespace seastack::hydro {

NonlinearHydrostaticsComponent::NonlinearHydrostaticsComponent(
    std::vector<std::unique_ptr<geometry::ISubmergedVolumeCalculator>> calculators,
    double rho,
    Eigen::Vector3d gravity,
    double waterplane_z,
    std::vector<int> active_bodies)
    : calculators_(std::move(calculators)),
      rho_(rho),
      gravity_(std::move(gravity)),
      waterplane_z_(waterplane_z),
      active_bodies_(std::move(active_bodies)) {
    if (calculators_.empty()) {
        throw std::invalid_argument(
            "NonlinearHydrostaticsComponent: at least one calculator required");
    }
    // If no active body mask, assume calculators map 1:1 to bodies 0..N-1
    if (active_bodies_.empty()) {
        active_bodies_.resize(calculators_.size());
        for (int i = 0; i < static_cast<int>(calculators_.size()); ++i)
            active_bodies_[i] = i;
    }
    if (calculators_.size() != active_bodies_.size()) {
        throw std::invalid_argument(
            "NonlinearHydrostaticsComponent: calculators count (" +
            std::to_string(calculators_.size()) +
            ") must match active_bodies count (" +
            std::to_string(active_bodies_.size()) + ")");
    }
}

void NonlinearHydrostaticsComponent::Compute(
    const SystemState& state, double /*time*/, BodyForces& inout_forces) {
    const int num_system_bodies = static_cast<int>(state.bodies.size());
    const int num_forces = static_cast<int>(inout_forces.size());

    for (int i = 0; i < static_cast<int>(active_bodies_.size()); ++i) {
        if (active_bodies_[i] >= num_system_bodies ||
            active_bodies_[i] >= num_forces) {
            throw std::invalid_argument(
                "NonlinearHydrostaticsComponent::Compute: active_bodies[" +
                std::to_string(i) + "] = " +
                std::to_string(active_bodies_[i]) +
                " is out of range (state.bodies.size()=" +
                std::to_string(num_system_bodies) + ", inout_forces.size()=" +
                std::to_string(num_forces) + ")");
        }
    }

    // Resize diagnostic arrays to system size (sparse: zeros for non-active bodies)
    if (static_cast<int>(last_volume_.size()) != num_system_bodies) {
        last_volume_.assign(num_system_bodies, 0.0);
        last_cob_.assign(num_system_bodies, Eigen::Vector3d::Zero());
    }

    const double g_mag = gravity_.norm();
    const Eigen::Vector3d e_z(0.0, 0.0, 1.0);

    for (int i = 0; i < static_cast<int>(active_bodies_.size()); ++i) {
        const int b = active_bodies_[i];
        const auto& body = state.bodies[b];

        // Build rigid transform from BodyState (position + RPY -> Affine3d)
        const Eigen::Vector3d& pos = body.position;
        const Eigen::Vector3d& rpy = body.orientation_rpy;

        Eigen::Affine3d body_to_world = Eigen::Affine3d::Identity();
        body_to_world.translate(pos);
        body_to_world.rotate(
            Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));

        auto result = calculators_[i]->Compute(body_to_world, waterplane_z_);

        // Cache for diagnostics (system-indexed)
        last_volume_[b] = result.volume;
        last_cob_[b] = result.centroid;

        if (result.volume <= 0.0) continue;

        const Eigen::Vector3d F_b = rho_ * g_mag * result.volume * e_z;
        const Eigen::Vector3d r_arm = result.centroid - pos;
        const Eigen::Vector3d M_b = r_arm.cross(F_b);

        inout_forces[b].force  += F_b;
        inout_forces[b].moment += M_b;
    }
}

double NonlinearHydrostaticsComponent::GetLastSubmergedVolume(int body_index) const {
    if (body_index < 0 || body_index >= static_cast<int>(last_volume_.size())) {
        return 0.0;  // Not yet computed or out of range
    }
    return last_volume_[body_index];
}

Eigen::Vector3d NonlinearHydrostaticsComponent::GetLastCentreOfBuoyancy(int body_index) const {
    if (body_index < 0 || body_index >= static_cast<int>(last_cob_.size())) {
        return Eigen::Vector3d::Zero();
    }
    return last_cob_[body_index];
}

}  // namespace seastack::hydro
