// 5SA with directional cos2s spreading + MoorDyn mooring.
// Demonstrates directional wave field, 5-body articulated structure,
// universal joints, and mooring coupling.
// Articulation (universal joints, TSDA dashpots, spring visuals): see five_sa_model_setup.h.

#include <gui/guihelper.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/infra/logging.h>

#ifdef SEASTACK_HAVE_MOORDYN
#include <seastack/mooring/moordyn_config.h>
#endif

#include <chrono/core/ChRealtimeStep.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

#include "five_sa_model_setup.h"

using namespace chrono;
using namespace seastack::hydro;

int main(int argc, char* argv[]) {
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    bool profilingOn = true, saveDataOn = true, plotOn = false, visualizationOn = true;
    std::string data_dir;
    if (!seastack::chrono::GetCLIArguments(argc, argv, "5SA spreading demo",
                                           saveDataOn, profilingOn, plotOn, visualizationOn, data_dir))
        return 1;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.02;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    double simulationDuration = 200.0;

    auto five_sa = SetupFiveSaModel(system, DATADIR, "5sa_directional.h5");

    // Directional sea state: JONSWAP with cos2s spreading
    SeaStateDefinition sea_state;
    sea_state.type = "irregular";
    sea_state.depth = 50.0;
    SeaStatePartition partition;
    partition.spectrum.type = "jonswap";
    partition.spectrum.Hs = 3.0;
    partition.spectrum.Tp = 10.0;
    partition.spectrum.gamma = 3.3;
    partition.spreading.type = "cos2s";
    partition.spreading.mean_direction_deg = 0.0;
    partition.spreading.s = 12.0;
    sea_state.partitions.push_back(partition);
    sea_state.n_omega = 64;
    sea_state.n_theta = 21;
    sea_state.seed = 42;

    auto components = ComponentSampler::Build(sea_state);
    auto waves = std::make_shared<LinearDirectionalWaveField>(std::move(components), sea_state.depth);
    waves->SetRampDuration(60.0);

    HydroSystem hydro_forces(five_sa.bodies, five_sa.h5file);

#ifdef SEASTACK_HAVE_MOORDYN
    {
        seastack::mooring::MoorDynConfig md_cfg;
        md_cfg.enabled = true;
        md_cfg.input_file = five_sa.moordyn_input;
        md_cfg.coupled_body_indices = {0, 2};  // body1 and body3
        hydro_forces.SetMoorDynConfig(md_cfg);
    }
#endif

    hydro_forces.SetExcitationMethod(seastack::hydro::ExcitationMethod::kFrequencyDomain);
    ApplyFiveSaHydroLinearDamping(hydro_forces);
    hydro_forces.AddWaves(waves);

    std::shared_ptr<seastack::viz::UI> pui = seastack::viz::CreateUI(visualizationOn);
    seastack::viz::UI& ui = *pui;

    ui.Init(&system, "5SA - Directional Spreading");
    ui.SetCamera(90, -80, 10, 90, 0, -2);
    ui.SetWaveModel(waves);

#ifdef SEASTACK_HAVE_MOORDYN
    ui.SetMooringLineProvider([&hydro_forces]() {
        return hydro_forces.GetMooringLineStates();
    });
#endif

    std::vector<double> time_vector;
    std::vector<double> body3_heave;

    while (system.GetChTime() <= simulationDuration) {
        if (!ui.IsRunning(timestep)) break;
        if (ui.simulationStarted) {
            system.DoStepDynamics(timestep);
            time_vector.push_back(system.GetChTime());
            body3_heave.push_back(five_sa.bodies[2]->GetPos().z());
        }
    }

    std::string out_dir = seastack::chrono::GetDemoOutDir();
    if (saveDataOn) {
        out_dir = out_dir + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directory(std::filesystem::path(out_dir));
        const std::string output_path = out_dir + "/5sa_spreading.txt";
        std::ofstream outputFile(output_path);
        if (!outputFile.is_open()) {
            seastack::infra::cli::LogError("Could not open " + output_path);
        } else {
            outputFile << std::left << std::setw(10) << "Time (s)"
                       << std::right << std::setw(16) << "Body3 Heave (m)" << std::endl;
            for (size_t i = 0; i < time_vector.size(); ++i)
                outputFile << std::left << std::setw(12) << std::setprecision(6) << std::fixed << time_vector[i]
                           << std::right << std::setw(16) << std::setprecision(6) << std::fixed << body3_heave[i]
                           << std::endl;
        }
    }

    return 0;
}
