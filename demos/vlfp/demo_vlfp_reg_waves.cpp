// VLFP: six hinged pontoons in regular waves (H = 1 m, T = 10 s, heading +X).
// C++ twin of the YAML case `data/demos/run_seastack/vlfp/regular_waves/`.
// Read top-to-bottom: parameters, Chrono HHT + linear solver, platform, waves,
// HydroSystem, loop.

#include <gui/guihelper.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/infra/logging.h>

#include <chrono/physics/ChSystemSMC.h>
#include <chrono/timestepper/ChAssemblyAnalysis.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include <chrono/timestepper/ChTimestepperImplicit.h>

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
constexpr double kTimestep = 0.01;             // s
constexpr double kSimulationDuration = 200.0;  // s

// Regular wave (matches the YAML case).
constexpr double kWaveHeight = 1.0;      // m (crest-to-trough)
constexpr double kWavePeriod = 10.0;     // s
constexpr double kWaterDepth = 1000.0;   // m
constexpr double kRampDuration = 40.0;   // s

// Per-body viscous damping corrections [surge..yaw]:
// linear N·s/m (N·m·s/rad), quadratic N·s^2/m^2 (N·m·s^2/rad^2).
constexpr std::array<double, 6> kLinearDamping = {
    2.0e4, 8.0e4, 8.0e4, 5.0e5, 2.0e5, 2.0e5};
constexpr std::array<double, 6> kQuadraticDamping = {
    1.0e4, 4.0e4, 4.0e4, 2.5e5, 1.0e5, 1.0e5};

void ApplyHydroSolverSettings(ChSystemSMC& system) {
    // HHT + SPARSE_LU: same combination as the YAML case. GMRES can fail to
    // converge tightly on this 6-body + coupled-RIRF system.
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    system.SetSolverType(ChSolver::Type::SPARSE_LU);

    auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    if (!hht) {
        return;
    }
    hht->SetAlpha(-0.2);
    hht->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION);
    hht->SetRelTolerance(1e-4);
    hht->SetMaxIters(50);
    hht->SetAbsTolerances(1e-4, 1e2);
    hht->SetStepControl(false);
}

SeaStateDefinition MakeVlfpRegularSea() {
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
    if (!seastack::chrono::GetCLIArguments(argc, argv, "VLFP regular waves demo", save_data_on,
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

    auto platform = vlfp::AddVlfpPlatform(system, data_path);

    auto sea_state = MakeVlfpRegularSea();
    auto components = ComponentSampler::Build(sea_state);
    auto waves =
        std::make_shared<LinearDirectionalWaveField>(std::move(components), sea_state.depth);
    waves->SetRampDuration(kRampDuration);

    HydroSystem hydro_forces(platform.hydro_bodies, platform.h5file, waves);
    hydro_forces.EnableRirfSmoothing();
    hydro_forces.SetLinearDamping(
        std::vector<std::array<double, 6>>(vlfp::kNumPontoons, kLinearDamping));
    hydro_forces.SetQuadraticDamping(
        std::vector<std::array<double, 6>>(vlfp::kNumPontoons, kQuadraticDamping));

    std::string out_dir = seastack::chrono::GetDemoOutDir();
    if (profiling_on || save_data_on) {
        out_dir = out_dir + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directory(std::filesystem::path(out_dir));
    }

    auto pui = seastack::viz::CreateUI(visualization_on);
    seastack::viz::UI& ui = *pui;

    auto wall_start = std::chrono::high_resolution_clock::now();

    ui.Init(&system, "VLFP - six hinged pontoons, regular waves");
    ui.SetCamera(-60.0, 29.0, 25.0, 87.0, 29.0, 0.0);
    ui.SetWaveModel(waves);

    const AssemblyAnalysis::ExitFlag assembly_exit =
        system.DoAssembly(AssemblyAnalysis::POSITION | AssemblyAnalysis::VELOCITY);
    if (assembly_exit == AssemblyAnalysis::ExitFlag::NOT_CONVERGED) {
        seastack::infra::cli::LogError(
            "DoAssembly(POSITION|VELOCITY) did not converge; initial multibody state is "
            "inconsistent.");
        return 4;
    }

    std::vector<double> time_vec;
    std::array<std::vector<double>, vlfp::kNumPontoons> heave;

    while (system.GetChTime() <= kSimulationDuration) {
        if (!ui.IsRunning(kTimestep)) {
            break;
        }

        if (ui.simulationStarted) {
            try {
                system.DoStepDynamics(kTimestep);
            } catch (const std::exception& e) {
                seastack::infra::cli::LogError(std::string("DoStepDynamics failed at t=") +
                                               std::to_string(system.GetChTime()) +
                                               " s: " + e.what());
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
            for (const auto& b : platform.hydro_bodies) {
                if (!finite(b->GetPos()) || !finite(b->GetLinVel()) ||
                    !finite(b->GetAngVelLocal())) {
                    seastack::infra::cli::LogError(
                        "Non-finite pontoon state at t=" + std::to_string(system.GetChTime()) +
                        " s");
                    return 2;
                }
            }

            time_vec.push_back(system.GetChTime());
            for (int i = 0; i < vlfp::kNumPontoons; ++i) {
                heave[i].push_back(platform.pontoons[i]->GetPos().z());
            }
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
        const std::string output_path = out_dir + "/vlfp_reg_waves.txt";
        std::ofstream out(output_path);
        if (!out.is_open()) {
            seastack::infra::cli::LogError("Could not open " + output_path);
        } else {
            out << std::left << std::setw(12) << "Time(s)";
            for (int i = 0; i < vlfp::kNumPontoons; ++i) {
                out << std::setw(16) << ("Body" + std::to_string(i + 1) + "Z(m)");
            }
            out << "\n";
            for (size_t n = 0; n < time_vec.size(); ++n) {
                out << std::left << std::fixed << std::setprecision(6) << std::setw(12)
                    << time_vec[n];
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
