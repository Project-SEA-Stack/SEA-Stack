// Sphere decay with state-space radiation approximation.
// Demonstrates O(1)-per-step radiation damping via fitted transfer functions.

#include <gui/guihelper.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
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
    if (!seastack::chrono::GetCLIArguments(argc, argv, "Sphere decay (state-space) demo",
                                           saveDataOn, profilingOn, plotOn, visualizationOn, data_dir))
        return 1;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto body1_mesh = (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();
    auto h5file     = (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.015;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    double simulationDuration = 40.0;

    std::shared_ptr<seastack::viz::UI> pui = seastack::viz::CreateUI(visualizationOn);
    seastack::viz::UI& ui = *pui;

    std::vector<double> time_vector;
    std::vector<double> heave_position;

    auto sphereBody = chrono_types::make_shared<ChBodyEasyMesh>(body1_mesh, 1000, false, true, false);
    sphereBody->SetName("body1");
    sphereBody->SetPos(ChVector3d(0, 0, -1));
    sphereBody->SetMass(261.8e3);
    system.Add(sphereBody);

    auto no_wave = std::make_shared<NoWave>();

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(sphereBody);

    HydroSystem hydro_forces(bodies, h5file);
    hydro_forces.AddWaves(no_wave);

    hydro_forces.SetRadiationMethod(seastack::hydro::RadiationMethod::kStateSpace);
    StateSpaceOptions ss_opts;
    ss_opts.max_order = 10;
    ss_opts.r2_threshold = 0.99;
    hydro_forces.SetStateSpaceOptions(ss_opts);

    ui.Init(&system, "Sphere - Decay (State-Space)");

    while (system.GetChTime() <= simulationDuration) {
        if (!ui.IsRunning(timestep)) break;
        if (ui.simulationStarted) {
            system.DoStepDynamics(timestep);
            time_vector.push_back(system.GetChTime());
            heave_position.push_back(sphereBody->GetPos().z());
        }
    }

    std::string out_dir = seastack::chrono::GetDemoOutDir();
    if (profilingOn || saveDataOn) {
        out_dir = out_dir + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directory(std::filesystem::path(out_dir));
    }

    if (saveDataOn) {
        const std::string output_path = out_dir + "/decay_ss.txt";
        std::ofstream outputFile(output_path);
        if (!outputFile.is_open()) {
            seastack::infra::cli::LogError("Could not open " + output_path);
        } else {
            outputFile << std::left << std::setw(10) << "Time (s)"
                       << std::right << std::setw(12) << "Heave (m)" << std::endl;
            for (size_t i = 0; i < time_vector.size(); ++i)
                outputFile << std::left << std::setw(12) << std::setprecision(6) << std::fixed << time_vector[i]
                           << std::right << std::setw(12) << std::setprecision(6) << std::fixed << heave_position[i]
                           << std::endl;
        }
    }

    return 0;
}
