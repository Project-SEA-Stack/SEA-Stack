/*********************************************************************
 * @file  scenario_subsystem.h
 * @brief Optional physics extensions for RunSingleCase.
 *
 * A scenario subsystem owns bodies, links, or per-step work that is not
 * expressible in the Chrono model YAML alone (for example a Chrono::FEA
 * structure).  Ordering inside RunSingleCase:
 *   Attach()          -- after Chrono Populate, before hydro attach, so the
 *                        bodies the subsystem references already exist and
 *                        anything it adds is in the system before stepping
 *   OnBeforeStep()    -- after the previous DoStepDynamics, before the next
 *   OnAfterStep()     -- immediately after DoStepDynamics
 *   OnRecord()        -- after the exporter's RecordStep, when HDF5 is on
 *
 * When no subsystems are configured, RunSingleCase behaviour is unchanged.
 *********************************************************************/

#ifndef SEASTACK_APP_SCENARIO_SUBSYSTEM_H
#define SEASTACK_APP_SCENARIO_SUBSYSTEM_H

#include <seastack/adapters/chrono/simulation_export.h>

namespace chrono {
class ChSystem;
}

namespace seastack::app {

class IScenarioSubsystem {
  public:
    virtual ~IScenarioSubsystem() = default;

    /// Add bodies / meshes / constraints to the shared Chrono system.
    virtual void Attach(::chrono::ChSystem& /*sys*/) {}

    /// Per-step work before DoStepDynamics.
    virtual void OnBeforeStep(double /*time*/, double /*dt*/) {}

    /// Per-step work after DoStepDynamics: anything that must observe the new
    /// multibody state (post-step measurements, diagnostics).
    virtual void OnAfterStep(double /*time*/, double /*dt*/) {}

    /// Called once per step after the exporter's RecordStep, when an HDF5
    /// exporter exists.  Subsystems that own extra channels append them here.
    /// Guard per-frame appends on SimulationExporter::LastStepWasRecorded() so
    /// they stay in sync with the decimated body/time datasets.
    virtual void OnRecord(seastack::chrono::SimulationExporter& /*exporter*/) {}
};

}  // namespace seastack::app

#endif  // SEASTACK_APP_SCENARIO_SUBSYSTEM_H
