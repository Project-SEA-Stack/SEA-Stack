// VLFP + HMMWV: drive a Chrono::Vehicle HMMWV across the six hinged pontoons of
// the floating platform while it responds to regular waves.
//
// Requires SEASTACK_ENABLE_VEHICLE and SEASTACK_ENABLE_VSG (and a Chrono install
// built with CH_ENABLE_MODULE_VEHICLE=ON).
//
// Controls (in the GUI window):
//   W/S  accelerate / decelerate (combined throttle+brake)
//   A/D  steer left / right, C center steering, R release pedals
//   mouse drag / scroll  set a fixed viewpoint, then drive with WASD
//   2     shallow rear-right third-person (car + platform + waves)
//   3     cinematic track that moves with the car (locks framing on entry)
//   1/4   Chrono chase / inside; 5 back to Free mouse view
//
// Physics layout (one ChSystemSMC):
//   - 6 pontoons with hinges + surge spring (vlfp_pontoons.h), each with a box
//     collision shape on its full extent so the vehicle's rigid tires contact
//     the deck directly. Deck motion (heave/pitch in waves) is therefore felt
//     by the vehicle automatically.
//   - SEA-Stack HydroSystem applies BEM excitation/radiation/hydrostatics to
//     the pontoons; the vehicle load path is tire contact -> pontoon body.
//   - HMMWV built from Chrono's JSON specs with RIGID tires (contact-based; the
//     FlatTerrain object below is a placeholder required by the Synchronize API
//     and is never contacted).
//
// Numerics: tire/deck contact needs a much smaller step (1e-3 s) than the
// hydro-only demos (1e-2 s). Two choices make this run near realtime (measured
// on a 30 s headless run; HHT + RIRF convolution was ~6x slower than realtime):
//   1. EULER_IMPLICIT_LINEARIZED + SPARSE_LU -- one linearized solve per step,
//      the standard Chrono::Vehicle setup, robust for SMC contact. HHT remains
//      available via kIntegrator for accuracy comparisons.
//   2. State-space radiation instead of direct RIRF convolution -- at dt = 1e-3
//      the convolution history dominates the step cost; the state-space fit is
//      O(1) per step and reproduces the coupled 6-body radiation kernels.
// A further option (not implemented here) is to swap rigid tires for TMeasy
// tires plus a custom ChTerrain that reports deck height/normal from the
// pontoon poses -- no contact solve at all, smoother and faster, but the deck
// is then only approximated under each wheel.

#include <gui/guihelper.h>
#include <gui/guihelper_vehicle.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/infra/logging.h>

#include <chrono/collision/ChCollisionSystem.h>
#include <chrono/core/ChRealtimeStep.h>
#include <chrono/physics/ChContactMaterial.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChIterativeSolverLS.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include <chrono/timestepper/ChTimestepperImplicit.h>

#include <chrono_vehicle/ChVehicleDataPath.h>
#include <chrono_vehicle/driver/ChInteractiveDriver.h>
#include <chrono_vehicle/terrain/FlatTerrain.h>
#include <chrono_vehicle/utils/ChUtilsJSON.h>
#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>

#include <array>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <vector>

#include "vlfp_pontoons.h"

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

namespace {

// --- Simulation parameters (edit here) ---------------------------------------
constexpr double kStepSize = 1e-3;             // s (tire contact scale)
constexpr double kSimulationDuration = 600.0;  // s (GUI window close also ends the run)
constexpr double kHeadlessDuration = 30.0;     // s (--no_gui smoke test: no driver input)
constexpr double kRenderFps = 50.0;

// Regular wave. Raised for a more dynamic crossing; the YAML baseline uses
// H = 1.0 m. Drop toward 0.5 m if the deck becomes too lively to drive.
constexpr double kWaveHeight = 3.0;     // m (crest-to-trough)
constexpr double kWavePeriod = 10.0;    // s
constexpr double kWaterDepth = 1000.0;  // m
constexpr double kRampDuration = 15.0;  // s

// Same viscous damping corrections as the baseline VLFP demo.
constexpr std::array<double, 6> kLinearDamping = {
    2.0e4, 8.0e4, 8.0e4, 5.0e5, 2.0e5, 2.0e5};
constexpr std::array<double, 6> kQuadraticDamping = {
    1.0e4, 4.0e4, 4.0e4, 2.5e5, 1.0e5, 1.0e5};

// Vehicle spawn: pontoon 1, port side (+Y when facing +X), near the deck edge.
// Pontoons span y in [0, 58] m; leave ~4 m clear of the edge for the tire track.
// Chassis reference ~0.6 m above the deck top; the vehicle settles onto its
// tires during the first steps.
constexpr double kVehicleInitY = vlfp::kPlatformY + 0.5 * vlfp::kPontoonWidthY - 4.0;  // ~54 m
const ChVector3d kVehicleInitLoc(vlfp::PontoonCenterX(0), kVehicleInitY,
                                 vlfp::kDeckTopZ + 0.6);
constexpr double kVehicleInitYaw = 0.0;  // rad, 0 = +X

// HMMWV from Chrono's vehicle data directory (JSON specification files).
const std::string kVehicleJSON = "hmmwv/vehicle/HMMWV_Vehicle.json";
const std::string kEngineJSON = "hmmwv/powertrain/HMMWV_EngineShafts.json";
const std::string kTransmissionJSON = "hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json";
const std::string kTireJSON = "hmmwv/tire/HMMWV_RigidTire.json";

// --- Integrator selection ----------------------------------------------------
enum class Integrator { kHHT, kEulerImplicitLinearized };
constexpr Integrator kIntegrator = Integrator::kEulerImplicitLinearized;

void ApplySolverSettings(ChSystemSMC& system) {
    switch (kIntegrator) {
        case Integrator::kHHT: {
            // Plan A: same family as the hydro-only demos. Step control lets HHT
            // sub-step through stiff tire-contact transients.
            system.SetTimestepperType(ChTimestepper::Type::HHT);
            system.SetSolverType(ChSolver::Type::SPARSE_LU);
            auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
            if (!hht) {
                return;
            }
            hht->SetAlpha(-0.2);
            hht->SetJacobianUpdateMethod(
                ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION);
            hht->SetRelTolerance(1e-4);
            hht->SetMaxIters(50);
            hht->SetAbsTolerances(1e-4, 1e2);
            hht->SetStepControl(true);
            hht->SetMinStepSize(1e-6);
            break;
        }
        case Integrator::kEulerImplicitLinearized: {
            // Plan B: single linearized solve per step -- robust for SMC tire
            // contact and much faster than HHT here. The direct sparse solver
            // beats iterative solvers on this system (a Barzilai-Borwein solver
            // at 150 iterations was ~2.4x slower than HHT in testing).
            system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
            system.SetSolverType(ChSolver::Type::SPARSE_LU);
            break;
        }
    }
}

SeaStateDefinition MakeVehicleDemoSea() {
    SeaStateDefinition def;
    def.type = "regular";
    def.depth = kWaterDepth;
    def.amplitude = 0.5 * kWaveHeight;
    def.omega = 2.0 * CH_PI / kWavePeriod;
    def.direction_deg = 0.0;
    return def;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    bool profiling_on = true;
    bool save_data_on = true;
    bool plot_on = false;
    bool visualization_on = true;
    std::string data_dir;
    if (!seastack::chrono::GetCLIArguments(argc, argv, "VLFP + HMMWV drive-on demo", save_data_on,
                                           profiling_on, plot_on, visualization_on, data_dir)) {
        return 1;
    }
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) {
        return 1;
    }

    const std::filesystem::path data_path(seastack::chrono::GetDataDir());

    // Chrono::Vehicle JSON specs live under <chrono_data>/vehicle/.
    const std::string chrono_data_dir = seastack::chrono::GetChronoDataDir();
    if (chrono_data_dir.empty()) {
        seastack::infra::cli::LogError(
            "Chrono data directory not resolved; HMMWV JSON specs unavailable.");
        return 1;
    }
    vehicle::SetVehicleDataPath(chrono_data_dir + "vehicle/");

    ChSystemSMC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    ApplySolverSettings(system);

    // Deck contact material (SMC). Match the HMMWV rigid-tire JSON: both the
    // physical properties and the explicit stiffness/damping coefficients Chrono
    // uses when UseMaterialProperties is false (the ChSystemSMC default).
    auto deck_material = chrono_types::make_shared<ChContactMaterialSMC>();
    deck_material->SetFriction(0.9f);
    deck_material->SetRestitution(0.01f);
    deck_material->SetYoungModulus(2e7f);
    deck_material->SetPoissonRatio(0.3f);
    deck_material->SetKn(2e5f);
    deck_material->SetGn(40.0f);
    deck_material->SetKt(2e5f);
    deck_material->SetGt(20.0f);

    auto platform = vlfp::AddVlfpPlatform(system, data_path, /*enable_deck_collision=*/true,
                                          deck_material);

    // --- Vehicle ---------------------------------------------------------------
    vehicle::WheeledVehicle hmmwv(&system, vehicle::GetVehicleDataFile(kVehicleJSON));
    hmmwv.Initialize(ChCoordsys<>(kVehicleInitLoc, QuatFromAngleZ(kVehicleInitYaw)));
    hmmwv.GetChassis()->SetFixed(false);
    hmmwv.SetChassisVisualizationType(VisualizationType::MESH);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::MESH);

    auto engine = vehicle::ReadEngineJSON(vehicle::GetVehicleDataFile(kEngineJSON));
    auto transmission =
        vehicle::ReadTransmissionJSON(vehicle::GetVehicleDataFile(kTransmissionJSON));
    auto powertrain = chrono_types::make_shared<vehicle::ChPowertrainAssembly>(engine, transmission);
    hmmwv.InitializePowertrain(powertrain);

    for (unsigned int i = 0; i < hmmwv.GetNumberAxles(); i++) {
        for (auto& wheel : hmmwv.GetAxle(i)->GetWheels()) {
            auto tire = vehicle::ReadTireJSON(vehicle::GetVehicleDataFile(kTireJSON));
            hmmwv.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Placeholder terrain: rigid tires get their forces from actual deck contact,
    // not from terrain queries. Kept far below the platform.
    vehicle::FlatTerrain terrain(-50.0, 0.9f);

    vehicle::ChInteractiveDriver driver(hmmwv);
    driver.SetSteeringDelta(0.02);
    driver.SetThrottleDelta(0.02);
    driver.SetBrakingDelta(0.06);
    driver.Initialize();

    if (!visualization_on) {
        // Headless smoke test: no keyboard input, so park the vehicle. The hinged
        // platform settles with a ~1 deg deck slope on the end pontoons and a
        // free-rolling vehicle would slide off the -X edge within ~20 s.
        driver.SetInputMode(vehicle::ChInteractiveDriver::InputMode::LOCK);
        driver.SetBraking(1.0);
    }

    // --- Hydrodynamics ----------------------------------------------------------
    auto sea_state = MakeVehicleDemoSea();
    auto components = ComponentSampler::Build(sea_state);
    auto waves =
        std::make_shared<LinearDirectionalWaveField>(std::move(components), sea_state.depth);
    waves->SetRampDuration(kRampDuration);

    HydroSystem hydro_forces(platform.hydro_bodies, platform.h5file, waves);
    // State-space radiation instead of direct RIRF convolution: at the vehicle
    // timestep (1e-3 s vs 1e-2 s in the hydro-only demos) the convolution
    // history makes each step ~10x more expensive; the state-space fit keeps
    // radiation cost O(1) per step so the demo can run near realtime.
    hydro_forces.SetRadiationMethod(seastack::hydro::RadiationMethod::kStateSpace);
    {
        // Faster fit for interactive use (6 bodies x 36 DOFs of cross-coupling).
        seastack::hydro::StateSpaceOptions ss_opts;
        ss_opts.max_hankel_size = 100;
        ss_opts.max_order = 8;
        ss_opts.r2_threshold = 0.90;
        hydro_forces.SetStateSpaceOptions(ss_opts);
    }
    hydro_forces.SetProfilingEnabled(profiling_on);
    hydro_forces.SetLinearDamping(
        std::vector<std::array<double, 6>>(vlfp::kNumPontoons, kLinearDamping));
    hydro_forces.SetQuadraticDamping(
        std::vector<std::array<double, 6>>(vlfp::kNumPontoons, kQuadraticDamping));

    // Force the lazy hydro model + state-space kernel fit NOW, before the VSG
    // window opens. Otherwise the first Play click blocks the UI thread for the
    // full multi-body fit and the window looks frozen / "crashed".
    std::cout << "Fitting state-space radiation kernels (one-time, before GUI)...\n";
    const auto fit_start = std::chrono::high_resolution_clock::now();
    (void)hydro_forces.CoordinateFuncForBody(1, 0);
    const auto fit_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::high_resolution_clock::now() - fit_start)
                            .count();
    std::cout << "  State-space fit done in " << fit_ms / 1000.0 << " s\n";

    std::string out_dir = seastack::chrono::GetDemoOutDir();
    if (profiling_on || save_data_on) {
        out_dir = out_dir + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directory(std::filesystem::path(out_dir));
    }

    // --- GUI --------------------------------------------------------------------
    // VehicleGUI adds WASD driving, the chase camera, and the vehicle ImGui panel
    // on top of the standard SEA-Stack window (water surface, overlay).
    std::shared_ptr<seastack::viz::UI> pui;
    std::shared_ptr<seastack::viz::VehicleGUI> vgui;
    if (visualization_on) {
        vgui = std::make_shared<seastack::viz::VehicleGUI>();
        vgui->AttachVehicle(&hmmwv, &driver, ChVector3d(0.0, 0.0, 1.75),
                            /*chase_distance=*/8.0, /*chase_height=*/0.5);
        pui = vgui;
    } else {
        pui = seastack::viz::CreateUI(false);
    }
    seastack::viz::UI& ui = *pui;

    auto wall_start = std::chrono::high_resolution_clock::now();

    ui.Init(&system, "VLFP + HMMWV - drive across the floating platform (WASD)");
    // Wide establishing shot while paused; chase camera takes over after Start.
    ui.SetCamera(-40.0, 29.0, 20.0, 87.0, 29.0, 0.0);
    ui.SetWaveModel(waves);

    std::vector<double> time_vec;
    std::vector<double> veh_x;
    std::vector<double> veh_speed;
    std::array<std::vector<double>, vlfp::kNumPontoons> heave;

    ChRealtimeStepTimer realtime_timer;
    int render_frame = 0;
    constexpr double kOutputDt = 0.05;  // s (decimated logging)
    double next_output_time = 0.0;
    const double duration = visualization_on ? kSimulationDuration : kHeadlessDuration;

    while (system.GetChTime() <= duration) {
        const double time = system.GetChTime();

        // Render (and pump window events) at kRenderFps while running; every
        // iteration while paused so the Start button and window stay responsive.
        if (!ui.simulationStarted || time >= render_frame / kRenderFps) {
            if (!ui.IsRunning(kStepSize)) {
                break;
            }
            if (ui.simulationStarted) {
                ++render_frame;
            }
        }
        if (!ui.simulationStarted) {
            continue;
        }

        // Synchronize all modules at the current time.
        const vehicle::DriverInputs driver_inputs = driver.GetInputs();
        driver.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        if (vgui) {
            vgui->SynchronizeVehicleVis(time, driver_inputs);
        }

        // Advance vehicle subsystems (tires, powertrain; it does NOT step the
        // shared system since the vehicle does not own it), then the full
        // multibody + hydro system, then the chase camera.
        driver.Advance(kStepSize);
        hmmwv.Advance(kStepSize);
        terrain.Advance(kStepSize);
        try {
            system.DoStepDynamics(kStepSize);
        } catch (const std::exception& e) {
            seastack::infra::cli::LogError(std::string("DoStepDynamics failed at t=") +
                                           std::to_string(time) + " s: " + e.what());
            return 3;
        }
        if (vgui) {
            vgui->AdvanceVehicleVis(kStepSize);
        }

        if (hydro_forces.HasDiverged()) {
            seastack::infra::cli::LogError(std::string("HydroSystem reported divergence at t=") +
                                           std::to_string(time) + " s");
            return 2;
        }

        if (time >= next_output_time) {
            next_output_time += kOutputDt;
            time_vec.push_back(time);
            veh_x.push_back(hmmwv.GetPos().x());
            veh_speed.push_back(hmmwv.GetSpeed());
            for (int i = 0; i < vlfp::kNumPontoons; ++i) {
                heave[i].push_back(platform.pontoons[i]->GetPos().z());
            }
        }

        // Soft real-time pacing so keyboard driving feels natural (GUI runs only).
        if (visualization_on) {
            realtime_timer.Spin(kStepSize);
        }
    }

    auto wall_end = std::chrono::high_resolution_clock::now();
    const unsigned wall_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start).count();

    if (profiling_on) {
        const auto stats = hydro_forces.GetProfileStats();
        std::cout << "\n--- Profiling ---\n";
        std::cout << "  Wall time: " << wall_ms / 1000.0 << " s\n";
        std::cout << "  Sim time:  " << system.GetChTime() << " s\n";
        std::cout << "  Hydro: hydrostatics " << stats.hydrostatics_seconds << " s, radiation "
                  << stats.radiation_seconds << " s, excitation " << stats.excitation_seconds
                  << " s\n";
    }

    if (save_data_on && !time_vec.empty()) {
        const std::string output_path = out_dir + "/vlfp_vehicle.txt";
        std::ofstream out(output_path);
        if (!out.is_open()) {
            seastack::infra::cli::LogError("Could not open " + output_path);
        } else {
            out << std::left << std::setw(12) << "Time(s)" << std::setw(16) << "VehX(m)"
                << std::setw(16) << "VehSpeed(m/s)";
            for (int i = 0; i < vlfp::kNumPontoons; ++i) {
                out << std::setw(16) << ("Body" + std::to_string(i + 1) + "Z(m)");
            }
            out << "\n";
            for (size_t n = 0; n < time_vec.size(); ++n) {
                out << std::left << std::fixed << std::setprecision(6) << std::setw(12)
                    << time_vec[n] << std::setw(16) << veh_x[n] << std::setw(16) << veh_speed[n];
                for (int i = 0; i < vlfp::kNumPontoons; ++i) {
                    out << std::setw(16) << heave[i][n];
                }
                out << "\n";
            }
            std::cout << "Results saved to: " << output_path << "\n";
        }
    }

    return 0;
}
