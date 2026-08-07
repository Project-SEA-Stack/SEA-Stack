/*********************************************************************
 * @file  vehicle_subsystem.cpp
 * @brief Implementation of TerrainSubsystem and VehicleSubsystem.
 *
 * Builds RigidTerrain patches and a wheeled or tracked Chrono::Vehicle from
 * declarative YAML, then Synchronize/Advance them each step.
 *********************************************************************/

#include "vehicle_subsystem.h"

#if defined(SEASTACK_HAVE_VEHICLE)

#include <seastack/infra/logging.h>

#include <chrono/collision/ChCollisionSystem.h>
#include <chrono/core/ChBezierCurve.h>
#include <chrono/core/ChCoordsys.h>
#include <chrono/core/ChRotation.h>
#include <chrono/core/ChVector3.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChContactMaterialSMC.h>
#include <chrono/physics/ChSystem.h>
#include <chrono/utils/ChBodyGeometry.h>

#include <chrono_vehicle/ChSubsysDefs.h>
#include <chrono_vehicle/ChVehicleDataPath.h>
#include <chrono_vehicle/ChDriver.h>
#include <chrono_vehicle/driver/ChInteractiveDriver.h>
#include <chrono_vehicle/driver/ChPathFollowerDriver.h>
#include <chrono_vehicle/ChPowertrainAssembly.h>
#include <chrono_vehicle/terrain/FlatTerrain.h>
#include <chrono_vehicle/terrain/RigidTerrain.h>
#include <chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h>
#include <chrono_vehicle/utils/ChUtilsJSON.h>
#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>

#if defined(SEASTACK_HAVE_VSG)
#include "gui/guihelper_vehicle.h"
#endif

#include <filesystem>
#include <stdexcept>
#include <vector>

namespace seastack::app {

namespace veh = ::chrono::vehicle;
using ::chrono::ChColor;
using ::chrono::ChCoordsys;
using ::chrono::ChVector3d;

namespace {

/// YAML visualization string -> Chrono VisualizationType (defaults to MESH).
::chrono::VisualizationType MapVisualization(const std::string& s) {
    if (s == "NONE" || s == "none") {
        return ::chrono::VisualizationType::NONE;
    }
    if (s == "PRIMITIVES" || s == "primitives") {
        return ::chrono::VisualizationType::PRIMITIVES;
    }
    if (s == "COLLISION" || s == "collision") {
        return ::chrono::VisualizationType::COLLISION;
    }
    return ::chrono::VisualizationType::MESH;
}

/// Ground-texture tiling: the texture image repeats this many times across the
/// patch in each direction, so one tile spans size/kTextureTiles metres.
constexpr float kTextureTiles = 20.0f;

/// Fallback terrain when a vehicle case declares no patches.  A flat plane far
/// below any hull draught, so it never contacts anything; the friction value is
/// only used if something does reach it.
constexpr double kFallbackTerrainHeight_m = -50.0;
constexpr float kFallbackTerrainFriction = 0.9f;

}  // namespace

// ===========================================================================
// TerrainSubsystem
// ===========================================================================

TerrainSubsystem::TerrainSubsystem(TerrainConfig config) : config_(std::move(config)) {}

TerrainSubsystem::~TerrainSubsystem() = default;

void TerrainSubsystem::Attach(::chrono::ChSystem& system) {
    terrain_ = std::make_unique<veh::RigidTerrain>(&system);

    for (const auto& patch_cfg : config_.patches) {
        // SMC contact material for the patch (matches ChSystemSMC contact).
        auto material = ::chrono_types::make_shared<::chrono::ChContactMaterialSMC>();
        material->SetFriction(static_cast<float>(patch_cfg.material.friction));
        material->SetRestitution(static_cast<float>(patch_cfg.material.restitution));
        material->SetYoungModulus(static_cast<float>(patch_cfg.material.young_modulus));
        material->SetPoissonRatio(static_cast<float>(patch_cfg.material.poisson_ratio));
        material->SetKn(static_cast<float>(patch_cfg.material.normal_stiffness));
        material->SetGn(static_cast<float>(patch_cfg.material.normal_damping));
        material->SetKt(static_cast<float>(patch_cfg.material.tangential_stiffness));
        material->SetGt(static_cast<float>(patch_cfg.material.tangential_damping));

        const std::string heightmap = ResolveVehicleDataFile(patch_cfg.heightmap);

        // connected_mesh=false makes the height map a single BVH triangle-mesh
        // shape, a large win against many track shoes.
        auto patch = terrain_->AddPatch(
            material,
            ChCoordsys<>(ChVector3d(patch_cfg.center[0], patch_cfg.center[1], patch_cfg.center[2]),
                         ::chrono::QUNIT),
            heightmap, patch_cfg.size[0], patch_cfg.size[1],
            /*hMin=*/patch_cfg.height_range[0], /*hMax=*/patch_cfg.height_range[1],
            patch_cfg.connected_mesh, patch_cfg.sweep_sphere_radius);

        patch->SetColor(ChColor(static_cast<float>(patch_cfg.color[0]),
                                static_cast<float>(patch_cfg.color[1]),
                                static_cast<float>(patch_cfg.color[2])));
        if (patch_cfg.has_texture) {
            patch->SetTexture(ResolveVehicleDataFile(patch_cfg.texture), kTextureTiles,
                              kTextureTiles);
        }
    }

    terrain_->Initialize();

    // GetHeight() ray-casts against the collision system, so the terrain's
    // collision model has to be bound before the vehicle queries its spawn
    // height.
    if (const auto& collision = system.GetCollisionSystem()) {
        collision->BindAll();
    }
}

void TerrainSubsystem::OnBeforeStep(double time, double dt) {
    if (terrain_) {
        terrain_->Synchronize(time);
        terrain_->Advance(dt);
    }
}

double TerrainSubsystem::GetHeight(double x, double y) const {
    if (!terrain_) {
        return 0.0;
    }
    return terrain_->GetHeight(ChVector3d(x, y, 0.0));
}

// ===========================================================================
// VehicleSubsystem
// ===========================================================================

VehicleSubsystem::VehicleSubsystem(VehicleConfig config, TerrainSubsystem* terrain)
    : config_(std::move(config)), terrain_(terrain) {}

VehicleSubsystem::~VehicleSubsystem() = default;

seastack::viz::UiKind VehicleSubsystem::PreferredUiKind() const {
    return config_.kind == VehicleKind::Tracked ? seastack::viz::UiKind::TrackedVehicle
                                                : seastack::viz::UiKind::WheeledVehicle;
}

std::shared_ptr<::chrono::ChBody> VehicleSubsystem::GetChassisBody() const {
    if (tracked_) {
        return tracked_->GetChassisBody();
    }
    if (wheeled_) {
        return wheeled_->GetChassisBody();
    }
    return nullptr;
}

::chrono::vehicle::ChTerrain& VehicleSubsystem::TerrainRef() {
    if (terrain_ && terrain_->GetRigidTerrain()) {
        return *terrain_->GetRigidTerrain();
    }
    return *flat_terrain_;
}

void VehicleSubsystem::Attach(::chrono::ChSystem& system) {
    const std::string vehicle_file = ResolveVehicleDataFile(config_.vehicle_json);

    // --- Spawn pose -------------------------------------------------------
    // Third component of initial_position is either a world z or "terrain",
    // in which case the chassis reference sits ride_height above the terrain
    // surface under (x, y).
    double spawn_z = config_.initial_z;
    if (config_.spawn_on_terrain) {
        double surface = 0.0;
        if (terrain_) {
            surface = terrain_->GetHeight(config_.initial_xy[0], config_.initial_xy[1]);
        } else {
            seastack::infra::cli::LogWarning(
                "vehicle: initial_position z is 'terrain' but no terrain block is present; "
                "spawning ride_height above z = 0.");
        }
        spawn_z = surface + config_.ride_height;
    }
    const ChCoordsys<> pose(ChVector3d(config_.initial_xy[0], config_.initial_xy[1], spawn_z),
                            ::chrono::QuatFromAngleZ(config_.initial_yaw));

    // --- Vehicle + powertrain --------------------------------------------
    veh::ChVehicle* vehicle = nullptr;
    if (config_.kind == VehicleKind::Tracked) {
        tracked_ = std::make_unique<veh::TrackedVehicle>(&system, vehicle_file);
        tracked_->Initialize(pose);
        const auto& viz = config_.visualization;
        tracked_->SetChassisVisualizationType(MapVisualization(viz.chassis));
        tracked_->SetSprocketVisualizationType(MapVisualization(viz.sprocket));
        tracked_->SetIdlerVisualizationType(MapVisualization(viz.suspension));
        tracked_->SetIdlerWheelVisualizationType(MapVisualization(viz.wheel));
        tracked_->SetSuspensionVisualizationType(MapVisualization(viz.suspension));
        tracked_->SetRoadWheelVisualizationType(MapVisualization(viz.wheel));
        tracked_->SetRollerVisualizationType(MapVisualization(viz.suspension));
        tracked_->SetTrackShoeVisualizationType(MapVisualization(viz.track_shoe));
        vehicle = tracked_.get();
    } else {
        wheeled_ = std::make_unique<veh::WheeledVehicle>(&system, vehicle_file);
        wheeled_->Initialize(pose);
        wheeled_->GetChassis()->SetFixed(false);
        const auto& viz = config_.visualization;
        wheeled_->SetChassisVisualizationType(MapVisualization(viz.chassis));
        wheeled_->SetSuspensionVisualizationType(MapVisualization(viz.suspension));
        wheeled_->SetSteeringVisualizationType(MapVisualization(viz.suspension));
        wheeled_->SetWheelVisualizationType(MapVisualization(viz.wheel));
        vehicle = wheeled_.get();
    }

    {
        auto engine = veh::ReadEngineJSON(ResolveVehicleDataFile(config_.engine_json));
        auto transmission =
            veh::ReadTransmissionJSON(ResolveVehicleDataFile(config_.transmission_json));
        if (!engine || !transmission) {
            throw std::runtime_error(
                "vehicle: could not read powertrain JSON specs (engine_json='" +
                config_.engine_json + "', transmission_json='" + config_.transmission_json +
                "'). Check the paths are relative to Chrono's vehicle data root.");
        }
        vehicle->InitializePowertrain(
            ::chrono_types::make_shared<veh::ChPowertrainAssembly>(engine, transmission));
    }

    // --- Tires (wheeled only) --------------------------------------------
    if (wheeled_) {
        if (config_.tire_json.empty()) {
            throw std::runtime_error("vehicle: a WHEELED vehicle requires 'tire_json'.");
        }
        const auto tire_viz = MapVisualization(config_.visualization.tire);
        for (unsigned int i = 0; i < wheeled_->GetNumberAxles(); ++i) {
            for (auto& wheel : wheeled_->GetAxle(i)->GetWheels()) {
                auto tire = veh::ReadTireJSON(ResolveVehicleDataFile(config_.tire_json));
                wheeled_->InitializeTire(tire, wheel, tire_viz);
            }
        }
        // Placeholder terrain for the Synchronize API when the vehicle drives on
        // deck contact rather than a RigidTerrain (wheeled still needs ChTerrain).
        if (!terrain_) {
            flat_terrain_ = std::make_unique<veh::FlatTerrain>(kFallbackTerrainHeight_m,
                                                               kFallbackTerrainFriction);
        }
    }

    // Name the chassis so MoorDyn coupling can find it by name.
    if (auto chassis = GetChassisBody()) {
        chassis->SetName(config_.chassis_name);
    }

    // --- Driver -----------------------------------------------------------
    if (config_.driver.kind == DriverKind::PathFollower) {
        if (config_.driver.path_xy.size() < 2) {
            throw std::runtime_error(
                "vehicle: a PATH_FOLLOWER driver needs at least two path waypoints.");
        }
        std::vector<ChVector3d> points;
        points.reserve(config_.driver.path_xy.size());
        for (const auto& xy : config_.driver.path_xy) {
            points.emplace_back(xy.first, xy.second, spawn_z);
        }
        auto path = ::chrono_types::make_shared<::chrono::ChBezierCurve>(points);

        auto follower = std::make_unique<veh::ChPathFollowerDriver>(
            *vehicle, path, "yaml_path", config_.driver.target_speed);
        follower->GetSteeringController().SetLookAheadDistance(config_.driver.steering.lookahead);
        follower->GetSteeringController().SetGains(config_.driver.steering.kp,
                                                   config_.driver.steering.ki,
                                                   config_.driver.steering.kd);
        follower->GetSpeedController().SetGains(config_.driver.speed.kp, config_.driver.speed.ki,
                                                config_.driver.speed.kd);
        follower->Initialize();
        driver_ = std::move(follower);
    } else {
        auto interactive = std::make_unique<veh::ChInteractiveDriver>(*vehicle);
        interactive->SetSteeringDelta(config_.driver.interactive_deltas.steering);
        interactive->SetThrottleDelta(config_.driver.interactive_deltas.throttle);
        interactive->SetBrakingDelta(config_.driver.interactive_deltas.braking);
        interactive->Initialize();
        driver_ = std::move(interactive);
    }
}

void VehicleSubsystem::PreInitUI(seastack::viz::UI& ui) {
#if defined(SEASTACK_HAVE_VSG)
    auto* vgui = dynamic_cast<seastack::viz::VehicleGUI*>(&ui);
    if (vgui) {
        vgui_ = vgui;
        vgui->AttachVehicle(
            tracked_ ? static_cast<veh::ChVehicle*>(tracked_.get())
                     : static_cast<veh::ChVehicle*>(wheeled_.get()),
            driver_.get(),
            ChVector3d(config_.chase_camera.point[0], config_.chase_camera.point[1],
                       config_.chase_camera.point[2]),
            config_.chase_camera.distance, config_.chase_camera.height);
    }
#else
    (void)ui;
#endif
}

void VehicleSubsystem::OnBeforeStep(double time, double dt) {
    if (!driver_) {
        return;
    }

    // Interactive driver with no GUI (headless smoke test): lock the pedals to
    // the brake once so the vehicle stays put.
    if (config_.driver.kind == DriverKind::Interactive && vgui_ == nullptr &&
        !headless_driver_locked_) {
        if (auto* idrv = dynamic_cast<veh::ChInteractiveDriver*>(driver_.get())) {
            idrv->SetInputMode(veh::ChInteractiveDriver::InputMode::LOCK);
            idrv->SetBraking(1.0);
        }
        headless_driver_locked_ = true;
    }

    veh::DriverInputs inputs = driver_->GetInputs();

    // Park gates for the path-follower survey line: hold on the brakes before
    // start_time, and again once the chassis has passed stop_at_x (so a finite
    // path does not make the tracker turn back on itself).
    const bool before_start = time < config_.driver.start_time;
    bool past_stop = false;
    if (config_.driver.has_stop_at_x) {
        if (auto chassis = GetChassisBody()) {
            past_stop = chassis->GetPos().x() >= config_.driver.stop_at_x;
        }
    }
    if (before_start || past_stop) {
        inputs.m_throttle = 0.0;
        inputs.m_steering = 0.0;
        inputs.m_braking = 1.0;
    }

    driver_->Synchronize(time);
    if (tracked_) {
        tracked_->Synchronize(time, inputs);
    } else if (wheeled_) {
        wheeled_->Synchronize(time, inputs, TerrainRef());
    }
#if defined(SEASTACK_HAVE_VSG)
    if (vgui_) {
        vgui_->SynchronizeVehicleVis(time, inputs);
    }
#endif

    // Advance the driver and the vehicle's internal subsystem state (driveline,
    // powertrain, tire or track internals) BEFORE the system step, then let the
    // runner call DoStepDynamics. This is Chrono's own ordering for a vehicle
    // that does not own its ChSystem (see demo_VEH_TwoCars.cpp: Synchronize all,
    // Advance all, then sys.DoStepDynamics).
    //
    // Measured on rov/crawler_drive (45 s, dt = 5e-4 s), moving these two calls
    // into OnAfterStep changed peak lateral path error by 9% (0.506 -> 0.460 m)
    // and peak fairlead tension by 2% (1.296 -> 1.266 kN), because the driver's
    // closed loop then lags the multibody state by one step. Crawler travel was
    // unaffected (81.07 -> 80.95 m). Keeping Chrono's ordering.
    driver_->Advance(dt);
    if (tracked_) {
        tracked_->Advance(dt);
    } else if (wheeled_) {
        wheeled_->Advance(dt);
    }
}

void VehicleSubsystem::OnAfterStep(double /*time*/, double dt) {
#if defined(SEASTACK_HAVE_VSG)
    if (vgui_) {
        vgui_->AdvanceVehicleVis(dt);
    }
#else
    (void)dt;
#endif
}

}  // namespace seastack::app

#endif  // SEASTACK_HAVE_VEHICLE
