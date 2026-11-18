/*********************************************************************
 * @file  structure_subsystem.h
 * @brief Scenario subsystem that adds a Chrono::FEA Euler-beam structure,
 *        its mates to existing model bodies, and optional rigid
 *        attachments, from declarative YAML (structure_config.h).
 *
 * Used for flexible members such as crane jibs (root/tip mates), linkspan
 * bridges (girders + cross-beams + deck planks) and connectors.
 *
 * Ordering: Attach() runs after Chrono Populate (so the bodies the mates
 * reference already exist) and before hydro attach.  Chrono::FEA is part of
 * the core Chrono library, so no extra CMake component or build gate is
 * required.
 *
 * Scope: beams carry their own inertia, elasticity and (optionally) gravity.
 * No hydrodynamic load is applied to the beams themselves; wave loads reach
 * the structure only through the bodies it is mated to.
 *********************************************************************/

#ifndef SEASTACK_APP_STRUCTURE_SUBSYSTEM_H
#define SEASTACK_APP_STRUCTURE_SUBSYSTEM_H

#include "scenario_subsystem.h"
#include "structure_config.h"

#include <array>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace chrono {
class ChBody;
namespace fea {
class ChMesh;
class ChNodeFEAxyzrot;
class ChElementBeamEuler;
}  // namespace fea
class ChLinkMateGeneric;
}  // namespace chrono

namespace seastack::app {

/// Result of the one-off static load-path check at the configured settle time.
struct StructureStaticCheckResult {
    bool valid = false;          ///< True once the check has been performed
    double weight_kN = 0.0;      ///< Weight of beams (when gravity is on) + rigid attachments
    double reactions_kN = 0.0;   ///< Mate reactions on the support bodies, along gravity
    double error_percent = 0.0;  ///< (reactions - weight) / weight * 100
    /// Why the check could not be performed, when valid is false.  Reported by
    /// the caller so a check that never ran fails instead of silently passing.
    std::string invalid_reason = "the simulation never reached the settle time";
};

/// Builds an Euler-beam frame, its mates and its rigid attachments from a
/// StructureConfig, and optionally reports a one-off static load-path check
/// once the model has settled: structure weight vs the summed mate reactions
/// along gravity (plus midspan sag when a cross_beams ladder is present).
class StructureSubsystem : public IScenarioSubsystem {
  public:
    using NodePtr = std::shared_ptr<::chrono::fea::ChNodeFEAxyzrot>;

    /// @param config        Parsed structure scenario
    /// @param settle_time_s Simulated time at which the static load-path check
    ///        is sampled [s].  Comes from the `structure_static_load_path`
    ///        check's `time_s`; infinity (the default) means no check.
    explicit StructureSubsystem(StructureConfig config,
                                double settle_time_s = std::numeric_limits<double>::infinity());
    ~StructureSubsystem() override;

    /// Resolve a relative diagnostics CSV path against the run output directory.
    void SetDiagnosticsOutputDirectory(const std::string& dir) {
        diagnostics_output_directory_ = dir;
    }

    void Attach(::chrono::ChSystem& system) override;
    void OnAfterStep(double time, double dt) override;
    void OnRecord(seastack::chrono::SimulationExporter& exporter) override;

    /// Static load-path check result; valid once the settle time is reached.
    const StructureStaticCheckResult& GetStaticCheckResult() const { return static_check_result_; }

    /// Static per-beam section geometry, captured during Attach().
    struct BeamSectionInfo {
        std::string name;
        std::string section_name;
        double width = 0.0;
        double depth = 0.0;
    };

    /// One deformed sample of an explicit beam in world coordinates.
    struct StructureFrameSample {
        std::string beam_name;
        std::vector<std::array<double, 3>> node_positions;         ///< world [m]
        std::vector<std::array<double, 4>> node_orientations_wxyz;  ///< world
        /// Fx Fy Fz [N], Mx My Mz [N*m] per element at the element mid-point
        /// (eta = 0), in the element's local frame, via
        /// ChElementBeamEuler::EvaluateSectionForceTorque.
        std::vector<std::array<double, 6>> element_forces;
    };

    /// Sample the current deformed state of one explicit beam.  Throws if the
    /// beam name is unknown.
    StructureFrameSample SampleBeam(const std::string& beam_name) const;

    /// Reaction force and torque that a mate applies to its support body, in
    /// world axes [N, N*m].  Throws if the mate name is unknown.
    std::array<double, 6> GetMateReactionWorld(const std::string& mate_name) const;

  private:
    /// Resolve a node reference of the form `beam.first|last|mid|<index>`.
    NodePtr ResolveNode(const std::string& ref) const;

    StructureConfig config_;
    double settle_time_s_;
    std::string diagnostics_output_directory_;

    std::shared_ptr<::chrono::fea::ChMesh> mesh_;
    std::unordered_map<std::string, std::vector<NodePtr>> beam_nodes_;
    std::unordered_map<std::string,
                       std::vector<std::shared_ptr<::chrono::fea::ChElementBeamEuler>>>
        beam_elements_;
    /// Undeformed node positions per beam (world, captured just after build).
    std::unordered_map<std::string, std::vector<std::array<double, 3>>> undeformed_positions_;
    std::vector<BeamSectionInfo> beam_infos_;
    std::unordered_map<std::string, std::vector<NodePtr>> node_sets_;
    std::vector<std::shared_ptr<::chrono::ChLinkMateGeneric>> mates_;
    /// Set true once WriteStructureStatic has run inside OnRecord.
    bool structure_static_written_ = false;

    // For the settled static check: rail A of a cross_beams ladder, if any,
    // so midspan sag can be reported.
    std::vector<NodePtr> sag_rail_;
    /// Gravitational acceleration of the system [m/s^2], world axes.
    std::array<double, 3> gravity_{0.0, 0.0, -9.81};
    double total_structure_weight_N_ = 0.0;
    bool static_check_reported_ = false;
    StructureStaticCheckResult static_check_result_;

    // Optional high-rate structural diagnostics CSV.
    std::unique_ptr<std::ofstream> diagnostics_csv_;
    int diagnostics_step_ = 0;
};

}  // namespace seastack::app

#endif  // SEASTACK_APP_STRUCTURE_SUBSYSTEM_H
