/*********************************************************************
 * @file  scenario_subsystem.h
 * @brief Optional physics/UI extensions for RunSingleCase.
 *
 * A scenario subsystem owns bodies, links, or per-step work that is not
 * expressible in the Chrono model YAML alone (vehicle, terrain, FEA
 * structure).  Ordering:
 *   Attach()          -- after Chrono Populate, before hydro attach and UI Init
 *                        (so bodies added here, e.g. a vehicle chassis, exist
 *                         when MoorDyn resolves its coupled body names)
 *   PreInitUI()       -- after CreateUI, before UI::Init (e.g. AttachVehicle)
 *   OnBeforeStep()    -- after the previous DoStepDynamics, before the next
 *   OnAfterStep()     -- immediately after DoStepDynamics
 *
 * When no subsystems are configured, RunSingleCase behaviour is unchanged.
 *********************************************************************/

#ifndef SEASTACK_APP_SCENARIO_SUBSYSTEM_H
#define SEASTACK_APP_SCENARIO_SUBSYSTEM_H

#include <gui/guihelper.h>

namespace chrono {
class ChSystem;
}

namespace seastack::app {

class IScenarioSubsystem {
  public:
    virtual ~IScenarioSubsystem() = default;

    /// Preferred UI when this subsystem is active.  The runner picks the
    /// strongest preference among all subsystems (Tracked > Wheeled > Standard).
    virtual seastack::viz::UiKind PreferredUiKind() const {
        return seastack::viz::UiKind::Standard;
    }

    /// Add bodies / meshes / constraints to the shared Chrono system.
    virtual void Attach(::chrono::ChSystem& /*sys*/) {}

    /// Called after CreateUI and before UI::Init.  Vehicle subsystems use this
    /// to AttachVehicle on a VehicleGUI before the visual system starts.
    virtual void PreInitUI(seastack::viz::UI& /*ui*/) {}

    /// Per-step work before DoStepDynamics.  For Chrono::Vehicle this is both
    /// Synchronize and Advance: a vehicle that does not own its ChSystem
    /// advances only its internal subsystem state here, and the runner then
    /// integrates the multibody state.  That is Chrono's own ordering (see
    /// demo_VEH_TwoCars.cpp).
    virtual void OnBeforeStep(double /*time*/, double /*dt*/) {}

    /// Per-step work after DoStepDynamics: anything that must observe the new
    /// multibody state (chase-camera dynamics, post-step measurements).
    virtual void OnAfterStep(double /*time*/, double /*dt*/) {}
};

}  // namespace seastack::app

#endif  // SEASTACK_APP_SCENARIO_SUBSYSTEM_H
