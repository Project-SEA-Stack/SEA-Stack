// OSWEC in irregular waves: hinged flap + fixed base.
// Demonstrates 2-body constraint coupling with spectral wave excitation.

#include <gui/guihelper.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/infra/logging.h>

#include <chrono/core/ChRealtimeStep.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

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
    if (!seastack::chrono::GetCLIArguments(argc, argv, "OSWEC irregular waves demo",
                                           saveDataOn, profilingOn, plotOn, visualizationOn, data_dir))
        return 1;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto flap_mesh = (DATADIR / "demos" / "oswec" / "geometry" / "flap.obj").lexically_normal().generic_string();
    auto base_mesh = (DATADIR / "demos" / "oswec" / "geometry" / "base.obj").lexically_normal().generic_string();
    auto h5file    = (DATADIR / "demos" / "oswec" / "hydroData" / "oswec.h5").lexically_normal().generic_string();

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.03;
    system.SetSolverType(ChSolver::Type::GMRES);
    double simulationDuration = 200.0;

    std::shared_ptr<seastack::viz::UI> pui = seastack::viz::CreateUI(visualizationOn);
    seastack::viz::UI& ui = *pui;

    std::vector<double> time_vector;
    std::vector<double> flap_pitch;

    // Flap body
    auto flap_body = chrono_types::make_shared<ChBodyEasyMesh>(flap_mesh, 1000, false, true, false);
    system.Add(flap_body);
    flap_body->SetName("body1");
    flap_body->SetPos(ChVector3d(0.0, 0.0, -3.9));
    flap_body->SetMass(127000.0);
    flap_body->SetInertiaXX(ChVector3d(1.85e6, 1.85e6, 1.85e6));

    // Base body
    auto base_body = chrono_types::make_shared<ChBodyEasyMesh>(base_mesh, 1000, false, true, false);
    system.Add(base_body);
    base_body->SetName("body2");
    base_body->SetPos(ChVector3d(0, 0, -10.15));
    base_body->SetMass(1e9);
    base_body->SetInertiaXX(ChVector3d(1e6, 1e6, 1e6));

    // Fix base to ground
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -10.15));
    ground->SetFixed(true);
    ground->EnableCollision(false);
    auto anchor = chrono_types::make_shared<ChLinkMateGeneric>();
    anchor->Initialize(base_body, ground, false, base_body->GetVisualModelFrame(),
                       base_body->GetVisualModelFrame());
    system.Add(anchor);
    anchor->SetConstrainedCoords(true, true, true, true, true, true);

    // Revolute joint between base and flap
    ChQuaternion<> revoluteRot = QuatFromAngleX(CH_PI / 2.0);
    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(base_body, flap_body, ChFramed(ChVector3d(0.0, 0.0, -8.9), revoluteRot));
    system.AddLink(revolute);

    // Irregular wave via SeaStateDefinition
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
    bodies.push_back(flap_body);
    bodies.push_back(base_body);

    HydroSystem hydro_forces(bodies, h5file);
    hydro_forces.AddWaves(waves);

    ui.Init(&system, "OSWEC - Irregular Waves");
    ui.SetCamera(0, -50, -10, 0, 0, -10);
    ui.SetWaveModel(waves);

    while (system.GetChTime() <= simulationDuration) {
        if (!ui.IsRunning(timestep)) break;
        if (ui.simulationStarted) {
            system.DoStepDynamics(timestep);
            time_vector.push_back(system.GetChTime());
            flap_pitch.push_back(flap_body->GetRot().GetCardanAnglesXYZ().y());
        }
    }

    std::string out_dir = seastack::chrono::GetDemoOutDir();
    if (profilingOn || saveDataOn) {
        out_dir = out_dir + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directory(std::filesystem::path(out_dir));
    }

    if (saveDataOn) {
        const std::string output_path = out_dir + "/oswec_irreg_waves.txt";
        std::ofstream outputFile(output_path);
        if (!outputFile.is_open()) {
            seastack::infra::cli::LogError("Could not open " + output_path);
        } else {
            outputFile << std::left << std::setw(10) << "Time (s)"
                       << std::right << std::setw(12) << "Pitch (rad)" << std::endl;
            for (size_t i = 0; i < time_vector.size(); ++i)
                outputFile << std::left << std::setw(12) << std::setprecision(6) << std::fixed << time_vector[i]
                           << std::right << std::setw(12) << std::setprecision(6) << std::fixed << flap_pitch[i]
                           << std::endl;
        }
    }

    return 0;
}
