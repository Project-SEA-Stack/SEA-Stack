// F3OF in irregular waves with TSDA station-keeping mooring.
// Demonstrates multi-body PTO coupling with spectral wave excitation.

#include <gui/guihelper.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/infra/logging.h>

#include <chrono/core/ChRealtimeStep.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemSMC.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

int main(int argc, char* argv[]) {
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    bool profilingOn = true, saveDataOn = true, plotOn = false, visualizationOn = true;
    std::string data_dir;
    if (!seastack::chrono::GetCLIArguments(argc, argv, "F3OF irregular waves demo",
                                           saveDataOn, profilingOn, plotOn, visualizationOn, data_dir))
        return 1;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto base_mesh = (DATADIR / "demos" / "f3of" / "geometry" / "base.obj").lexically_normal().generic_string();
    auto flap_mesh = (DATADIR / "demos" / "f3of" / "geometry" / "flap.obj").lexically_normal().generic_string();
    auto h5file    = (DATADIR / "demos" / "f3of" / "hydroData" / "f3of.h5").lexically_normal().generic_string();

    ChSystemSMC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.02;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    double simulationDuration = 200.0;

    std::shared_ptr<seastack::viz::UI> pui = seastack::viz::CreateUI(visualizationOn);
    seastack::viz::UI& ui = *pui;

    std::vector<double> time_vector;
    std::vector<double> base_surge;
    std::vector<double> fore_pitch;
    std::vector<double> aft_pitch;

    // Base body
    auto base = chrono_types::make_shared<ChBodyEasyMesh>(base_mesh, 0, false, true, false);
    system.Add(base);
    base->SetName("body1");
    base->SetPos(ChVector3d(0.0, 0.0, -9.0));
    base->SetMass(1089825.0);
    base->SetInertiaXX(ChVector3d(100000000.0, 76300000.0, 100000000.0));

    // Fore flap
    auto flapFore = chrono_types::make_shared<ChBodyEasyMesh>(flap_mesh, 0, false, true, false);
    system.Add(flapFore);
    flapFore->SetName("body2");
    flapFore->SetPos(ChVector3d(-12.5, 0.0, -5.5));
    flapFore->SetMass(179250.0);
    flapFore->SetInertiaXX(ChVector3d(100000000.0, 1300000.0, 100000000.0));

    // Aft flap
    auto flapAft = chrono_types::make_shared<ChBodyEasyMesh>(flap_mesh, 0, false, true, false);
    system.Add(flapAft);
    flapAft->SetName("body3");
    flapAft->SetPos(ChVector3d(12.5, 0.0, -5.5));
    flapAft->SetMass(179250.0);
    flapAft->SetInertiaXX(ChVector3d(100000000.0, 1300000.0, 100000000.0));

    // PTO revolute joints (unlocked)
    ChQuaternion<> revoluteRot = QuatFromAngleX(CH_PI / 2.0);
    auto revoluteFore = chrono_types::make_shared<ChLinkLockRevolute>();
    revoluteFore->Initialize(base, flapFore, ChFramed(ChVector3d(-12.5, 0.0, -9.0), revoluteRot));
    system.AddLink(revoluteFore);

    auto revoluteAft = chrono_types::make_shared<ChLinkLockRevolute>();
    revoluteAft->Initialize(base, flapAft, ChFramed(ChVector3d(12.5, 0.0, -9.0), revoluteRot));
    system.AddLink(revoluteAft);

    // Ground body for station-keeping
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -9.0));
    ground->SetFixed(true);
    ground->EnableCollision(false);

    // TSDA station-keeping spring (horizontal, surge direction)
    auto mooring_spring = chrono_types::make_shared<ChLinkTSDA>();
    mooring_spring->Initialize(base, ground, false, ChVector3d(0.0, 0.0, -9.0), ChVector3d(0.0, 0.0, -9.0));
    mooring_spring->SetSpringCoefficient(100000.0);
    mooring_spring->SetDampingCoefficient(10000.0);
    system.AddLink(mooring_spring);

    // Irregular wave
    SeaStateDefinition sea_state;
    sea_state.type = "irregular";
    SeaStatePartition partition;
    partition.spectrum.type = "jonswap";
    partition.spectrum.Hs = 2.0;
    partition.spectrum.Tp = 8.0;
    partition.spectrum.gamma = 3.3;
    sea_state.partitions.push_back(partition);
    sea_state.n_omega = 200;
    sea_state.seed = 42;
    auto components = ComponentSampler::Build(sea_state);
    auto waves = std::make_shared<LinearDirectionalWaveField>(std::move(components), sea_state.depth);
    waves->SetRampDuration(60.0);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(base);
    bodies.push_back(flapFore);
    bodies.push_back(flapAft);

    HydroSystem hydro_forces(bodies, h5file);
    hydro_forces.AddWaves(waves);

    ui.Init(&system, "F3OF - Irregular Waves");
    ui.SetCamera(0, -50, -10, 0, 0, -10);
    ui.SetWaveModel(waves);

    while (system.GetChTime() <= simulationDuration) {
        if (!ui.IsRunning(timestep)) break;
        if (ui.simulationStarted) {
            system.DoStepDynamics(timestep);
            time_vector.push_back(system.GetChTime());
            base_surge.push_back(base->GetPos().x());
            fore_pitch.push_back(flapFore->GetRot().GetCardanAnglesXYZ().y());
            aft_pitch.push_back(flapAft->GetRot().GetCardanAnglesXYZ().y());
        }
    }

    std::string out_dir = seastack::chrono::GetDemoOutDir();
    if (saveDataOn) {
        out_dir = out_dir + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directory(std::filesystem::path(out_dir));
        const std::string output_path = out_dir + "/f3of_irreg_waves.txt";
        std::ofstream outputFile(output_path);
        if (!outputFile.is_open()) {
            seastack::infra::cli::LogError("Could not open " + output_path);
        } else {
            outputFile << std::left << std::setw(10) << "Time (s)"
                       << std::right << std::setw(16) << "Base Surge (m)"
                       << std::right << std::setw(16) << "Fore Pitch (rad)"
                       << std::right << std::setw(16) << "Aft Pitch (rad)" << std::endl;
            for (size_t i = 0; i < time_vector.size(); ++i)
                outputFile << std::left << std::setw(12) << std::setprecision(6) << std::fixed << time_vector[i]
                           << std::right << std::setw(16) << std::setprecision(6) << std::fixed << base_surge[i]
                           << std::right << std::setw(16) << std::setprecision(6) << std::fixed << fore_pitch[i]
                           << std::right << std::setw(16) << std::setprecision(6) << std::fixed << aft_pitch[i]
                           << std::endl;
        }
    }

    return 0;
}
