// Trimaran: three hulls in irregular waves, no cross-arms.
// Read top-to-bottom: parameters, Chrono HHT + linear solver, hulls, waves, HydroSystem, loop.

#include <gui/guihelper.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/infra/logging.h>

#include <chrono/core/ChRealtimeStep.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include <chrono/timestepper/ChTimestepperImplicit.h>

#include <array>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

#include "trimaran_hulls.h"
#include "trimaran_sea_state.h"

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

namespace {

// --- Simulation parameters (edit here) ---------------------------------------
constexpr double kTimestep = 0.005;           // s
constexpr double kSimulationDuration = 200.0;  // s

// Per-body quadratic damping [surge..yaw]: translational N·s^2/m^2, rotational N·m·s^2/rad^2.
constexpr std::array<double, 6> kCenterQuadraticDamping = {
    1.2e4, 1.8e4, 1.0e4, 2.0e5, 2.0e5, 6.0e4};
constexpr std::array<double, 6> kOutriggerQuadraticDamping = {
    6.0e3, 1.0e4, 6.0e3, 1.2e5, 1.2e5, 4.0e4};

void ApplyHydroSolverSettings(ChSystemSMC& system) {
    // HHT: alpha in (-1/3, 0] adds high-frequency numerical damping. SPARSE_LU is typical for
    // hydro-only KKT. Internal step control helps stiff radiation/added-mass coupling.
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    system.SetSolverType(ChSolver::Type::SPARSE_LU);

    auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    if (!hht) {
        return;
    }
    hht->SetAlpha(-0.2);
    hht->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION);
    hht->SetRelTolerance(1e-6);
    hht->SetMaxIters(150);
    hht->SetAbsTolerances(1e-6, 1e-4);
    hht->SetStepControl(true);
    hht->SetStepIncreaseFactor(1.35);
    hht->SetStepDecreaseFactor(0.6);
    hht->SetRequiredSuccessfulSteps(10);
    hht->SetMaxItersSuccess(3);
    hht->SetMinStepSize(1e-12);
}

}  // namespace

int main(int argc, char* argv[]) {
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    bool profiling_on = true;
    bool save_data_on = true;
    bool plot_on = false;
    bool visualization_on = true;
    std::string data_dir;
    if (!seastack::chrono::GetCLIArguments(argc, argv, "Trimaran hydro-only demo", save_data_on,
                                           profiling_on, plot_on, visualization_on, data_dir)) {
        return 1;
    }
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) {
        return 1;
    }

    const std::filesystem::path data_path(seastack::chrono::GetDataDir());

    ChSystemSMC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));

    ApplyHydroSolverSettings(system);

    auto hulls = trimaran::AddTrimaranHulls(system, data_path);

    auto sea_state = trimaran::MakeTrimaranDemoIrregularSea();
    auto components = ComponentSampler::Build(sea_state);
    auto waves = std::make_shared<LinearDirectionalWaveField>(std::move(components), sea_state.depth);
    waves->SetRampDuration(60.0);

    HydroSystem hydro_forces(hulls.hydro_bodies, hulls.h5file);
    hydro_forces.SetExcitationMethod(seastack::hydro::ExcitationMethod::kFrequencyDomain);
    hydro_forces.AddWaves(waves);
    hydro_forces.EnableRirfSmoothing();
    hydro_forces.SetQuadraticDamping(std::vector<std::array<double, 6>>{
        kCenterQuadraticDamping, kOutriggerQuadraticDamping, kOutriggerQuadraticDamping});

    std::string out_dir = seastack::chrono::GetDemoOutDir();
    if (profiling_on || save_data_on) {
        out_dir = out_dir + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directory(std::filesystem::path(out_dir));
    }

    auto pui = seastack::viz::CreateUI(visualization_on);
    seastack::viz::UI& ui = *pui;

    auto wall_start = std::chrono::high_resolution_clock::now();

    ui.Init(&system, "Trimaran - hydro only (3 bodies)");
    ui.SetCamera(0, -60, 20, 0, 0, 0);
    ui.SetWaveModel(waves);

    std::vector<double> time_vec;
    std::vector<double> center_heave;
    std::vector<double> port_heave;
    std::vector<double> stbd_heave;

    while (system.GetChTime() <= kSimulationDuration) {
        if (!ui.IsRunning(kTimestep)) {
            break;
        }

        if (ui.simulationStarted) {
            try {
                system.DoStepDynamics(kTimestep);
            } catch (const std::exception& e) {
                seastack::infra::cli::LogError(std::string("DoStepDynamics failed at t=") +
                                               std::to_string(system.GetChTime()) + " s: " + e.what());
                return 3;
            }

            if (hydro_forces.HasDiverged()) {
                seastack::infra::cli::LogError(
                    std::string("HydroSystem reported divergence at t=") +
                    std::to_string(system.GetChTime()) + " s");
                return 2;
            }

            const auto finite = [](const ChVector3d& v) {
                return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
            };
            for (const auto& b : hulls.hydro_bodies) {
                const auto& p = b->GetPos();
                const auto& vel = b->GetLinVel();
                const auto w = b->GetAngVelLocal();
                if (!finite(p) || !finite(vel) || !finite(w)) {
                    seastack::infra::cli::LogError(
                        "Non-finite hull state at t=" + std::to_string(system.GetChTime()) + " s");
                    return 2;
                }
            }

            time_vec.push_back(system.GetChTime());
            center_heave.push_back(hulls.center->GetPos().z());
            port_heave.push_back(hulls.port->GetPos().z());
            stbd_heave.push_back(hulls.stbd->GetPos().z());
        }
    }

    auto wall_end = std::chrono::high_resolution_clock::now();
    const unsigned wall_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start).count();

    if (profiling_on) {
        std::cout << "\n--- Profiling ---\n";
        std::cout << "  Wall time: " << wall_ms / 1000.0 << " s\n";
        std::cout << "  Sim time:  " << kSimulationDuration << " s\n";
    }

    if (save_data_on && !time_vec.empty()) {
        const std::string output_path = out_dir + "/trimaran_hydro.txt";
        std::ofstream out(output_path);
        if (!out.is_open()) {
            seastack::infra::cli::LogError("Could not open " + output_path);
        } else {
            out << std::left << std::setw(12) << "Time(s)" << std::setw(16) << "CenterZ(m)"
                << std::setw(16) << "PortZ(m)" << std::setw(16) << "StbdZ(m)" << "\n";
            for (size_t i = 0; i < time_vec.size(); ++i) {
                out << std::left << std::fixed << std::setprecision(6) << std::setw(12)
                    << time_vec[i] << std::setw(16) << center_heave[i] << std::setw(16)
                    << port_heave[i] << std::setw(16) << stbd_heave[i] << "\n";
            }
            std::cout << "Results saved to: " << output_path << "\n";
        }
    }

    return 0;
}
