/*********************************************************************
 * @file  vehicle_subsystem.h
 * @brief Scenario subsystems that add a Chrono::Vehicle and rigid terrain to
 *        a RunSingleCase simulation from declarative YAML.
 *
 * Two subsystems:
 *   TerrainSubsystem  -- one RigidTerrain with height-map patches.
 *   VehicleSubsystem  -- a wheeled or tracked vehicle, its powertrain, tires,
 *                        driver, and per-step Synchronize/Advance.
 *
 * Ordering contract (enforced by pushing terrain before vehicle in the
 * subsystem vector): TerrainSubsystem::Attach runs first so the vehicle can
 * query the terrain height at its spawn point, and TerrainSubsystem::OnBeforeStep
 * synchronizes the terrain before the vehicle reads it.
 *
 * Built only when SEASTACK_ENABLE_VEHICLE is on.
 *********************************************************************/

#ifndef SEASTACK_APP_VEHICLE_SUBSYSTEM_H
#define SEASTACK_APP_VEHICLE_SUBSYSTEM_H

#include <seastack/config.h>

#if defined(SEASTACK_HAVE_VEHICLE)

#include "scenario_subsystem.h"
#include "vehicle_config.h"

#include <memory>
#include <string>

namespace chrono {
class ChBody;
namespace vehicle {
class ChTerrain;
class RigidTerrain;
class FlatTerrain;
class ChVehicle;
class TrackedVehicle;
class WheeledVehicle;
class ChDriver;
}  // namespace vehicle
}  // namespace chrono

namespace seastack::viz {
class VehicleGUI;
}

namespace seastack::app {

/// A RigidTerrain built from one or more height-map patches.  Owns the terrain
/// object for the life of the run and synchronizes it each step.
class TerrainSubsystem : public IScenarioSubsystem {
  public:
    explicit TerrainSubsystem(TerrainConfig config);
    ~TerrainSubsystem() override;

    void Attach(::chrono::ChSystem& system) override;
    void OnBeforeStep(double time, double dt) override;

    /// Terrain surface height under (x, y) [m]; valid only after Attach().
    double GetHeight(double x, double y) const;

    /// Non-owning access for the vehicle's Synchronize (wheeled needs a
    /// ChTerrain&).  Null before Attach().
    ::chrono::vehicle::RigidTerrain* GetRigidTerrain() const { return terrain_.get(); }

  private:
    TerrainConfig config_;
    std::unique_ptr<::chrono::vehicle::RigidTerrain> terrain_;
};

/// A single wheeled or tracked vehicle with its driver.  When `terrain` is
/// non-null the vehicle drives on that RigidTerrain; otherwise a FlatTerrain
/// placeholder is used (wheeled vehicles need a ChTerrain reference even when
/// contact is deck-local).
class VehicleSubsystem : public IScenarioSubsystem {
  public:
    VehicleSubsystem(VehicleConfig config, TerrainSubsystem* terrain);
    ~VehicleSubsystem() override;

    seastack::viz::UiKind PreferredUiKind() const override;

    void Attach(::chrono::ChSystem& system) override;
    void PreInitUI(seastack::viz::UI& ui) override;
    void OnBeforeStep(double time, double dt) override;
    void OnAfterStep(double time, double dt) override;

    /// Chassis body (for MoorDyn coupling).  Null before Attach().
    std::shared_ptr<::chrono::ChBody> GetChassisBody() const;
  private:
    ::chrono::vehicle::ChTerrain& TerrainRef();

    VehicleConfig config_;
    TerrainSubsystem* terrain_ = nullptr;  ///< non-owning; may be null

    std::unique_ptr<::chrono::vehicle::TrackedVehicle> tracked_;
    std::unique_ptr<::chrono::vehicle::WheeledVehicle> wheeled_;
    std::unique_ptr<::chrono::vehicle::ChDriver> driver_;
    std::unique_ptr<::chrono::vehicle::FlatTerrain> flat_terrain_;

    seastack::viz::VehicleGUI* vgui_ = nullptr;  ///< set in PreInitUI (GUI only)
    bool headless_driver_locked_ = false;        ///< interactive driver, headless guard
};

}  // namespace seastack::app

#endif  // SEASTACK_HAVE_VEHICLE

#endif  // SEASTACK_APP_VEHICLE_SUBSYSTEM_H
