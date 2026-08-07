/*********************************************************************
 * @file  structure_subsystem.h
 * @brief Scenario subsystem that adds a Chrono::FEA Euler-beam structure
 *        (girders + cross-beams), its end mates to existing model bodies,
 *        and rigid deck planks, from declarative YAML.
 *
 * Builds an aluminium ladder frame spanning the gap between two hulls on
 * four pinned bearings, carrying the thin planks the vehicle's tires roll on.
 *
 * Ordering: Attach() runs after Chrono Populate (so the bodies the mates
 * reference, e.g. body1/body2 from the model YAML, already exist) and before
 * hydro attach.  Chrono::FEA is part of the core Chrono library, so no extra
 * CMake component or build gate is required.
 *********************************************************************/

#ifndef SEASTACK_APP_STRUCTURE_SUBSYSTEM_H
#define SEASTACK_APP_STRUCTURE_SUBSYSTEM_H

#include "scenario_subsystem.h"
#include "structure_config.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace chrono {
class ChBody;
namespace fea {
class ChMesh;
class ChNodeFEAxyzrot;
}  // namespace fea
class ChLinkMateGeneric;
}  // namespace chrono

namespace seastack::app {

/// Result of the one-off static load-path check at the configured settle time.
struct BridgeStaticCheckResult {
    bool valid = false;          ///< True once the check has been performed
    double weight_kN = 0.0;      ///< Total structure weight
    double reactions_kN = 0.0;   ///< Sum of bearing reactions
    double error_percent = 0.0;  ///< (reactions - weight) / weight * 100
    /// Why the check could not be performed, when valid is false.  Reported by
    /// the caller so a check that never ran fails instead of silently passing.
    std::string invalid_reason = "the simulation never reached the settle time";
};

/// Builds an Euler-beam frame, its mates and its rigid attachments from a
/// StructureConfig, and reports a one-off static load-path check once the
/// model has settled (bridge weight vs summed bearing reactions, and midspan
/// sag relative to the bearing chord).
class StructureSubsystem : public IScenarioSubsystem {
  public:
    /// @param config      Parsed structure scenario
    /// @param settle_time_s Simulated time at which the static load-path check is
    ///        sampled [s].  Comes from the `bridge_static_load_path` check's
    ///        `time_s` so the check and the sample share one source of truth.
    StructureSubsystem(StructureConfig config, double settle_time_s);
    ~StructureSubsystem() override;

    void Attach(::chrono::ChSystem& system) override;
    void OnAfterStep(double time, double dt) override;

    /// Static load-path check result; valid once the settle time is reached.
    const BridgeStaticCheckResult& GetStaticCheckResult() const { return static_check_result_; }

  private:
    using NodePtr = std::shared_ptr<::chrono::fea::ChNodeFEAxyzrot>;

    /// Resolve a node reference of the form `beam.first|last|mid|<index>`.
    NodePtr ResolveNode(const std::string& ref) const;

    StructureConfig config_;
    double settle_time_s_;

    std::shared_ptr<::chrono::fea::ChMesh> mesh_;
    std::unordered_map<std::string, std::vector<NodePtr>> beam_nodes_;
    std::unordered_map<std::string, std::vector<NodePtr>> node_sets_;
    std::vector<std::shared_ptr<::chrono::ChLinkMateGeneric>> bearings_;

    // For the settled static check: the two rail (girder) node chains, if a
    // cross_beams block named them, so sag can be measured on rail A.
    std::vector<NodePtr> sag_rail_;
    double total_structure_weight_N_ = 0.0;
    bool static_check_reported_ = false;
    BridgeStaticCheckResult static_check_result_;
};

}  // namespace seastack::app

#endif  // SEASTACK_APP_STRUCTURE_SUBSYSTEM_H
