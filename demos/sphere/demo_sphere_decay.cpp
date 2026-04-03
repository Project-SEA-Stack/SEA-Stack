#include <gui/guihelper.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/infra/logging.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <chrono>      // std::chrono::high_resolution_clock::now
#include <filesystem>  // c++17 only
#include <fstream>
#include <iomanip>     // std::setprecision
#include <vector>      // std::vector<double>

// Use the namespaces of Chrono
using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

int main(int argc, char* argv[]) {
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    // Parse CLI arguments and initialize environment
    bool profilingOn     = true;
    bool saveDataOn      = true;
    bool plotOn          = false;  // time series in decay.txt; plots via matplotlib in test tooling
    bool visualizationOn = true;
    std::string data_dir;
    if (!seastack::chrono::GetCLIArguments(argc, argv, "Sphere decay demo", saveDataOn, profilingOn, plotOn, visualizationOn,
                                 data_dir))
        return 1;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    // Get model file names
    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto body1_meshfile =
        (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();
    auto h5fname = (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();

    // system/solver settings
    ChSystemNSC system;

    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));

    double timestep = 0.015;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    double simulationDuration = 40.0;

    // Create user interface
    std::shared_ptr<seastack::viz::UI> pui = seastack::viz::CreateUI(visualizationOn);

    seastack::viz::UI& ui = *pui.get();

    // Output timeseries
    std::vector<double> time_vector;
    std::vector<double> heave_position;

    // set up body from a mesh
    std::cout << "Attempting to open mesh file: " << body1_meshfile << std::endl;
    std::shared_ptr<ChBody> sphereBody = chrono_types::make_shared<ChBodyEasyMesh>(  //
        body1_meshfile,                                                              // file name
        1000,                                                                        // density
        false,  // do not evaluate mass automatically
        true,   // create visualization asset
        false   // do not collide
    );

    // define the body's initial conditions
    sphereBody->SetName("body1");  // must set body name correctly! (must match .h5 file)
    sphereBody->SetPos(ChVector3d(0, 0, -1));
    sphereBody->SetMass(261.8e3);

    // Create a visualization material
    auto cadet_blue = chrono_types::make_shared<ChVisualMaterial>();
    cadet_blue->SetDiffuseColor(ChColor(0.3f, 0.1f, 0.1f));
    sphereBody->GetVisualShape(0)->SetMaterial(0, cadet_blue);

    system.Add(sphereBody);

    auto default_dont_add_waves = std::make_shared<NoWave>();

    // attach hydrodynamic forces to body
    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(sphereBody);

    HydroSystem hydro_forces(bodies, h5fname);
    hydro_forces.AddWaves(default_dont_add_waves);

    // for profiling
    auto start = std::chrono::high_resolution_clock::now();

    // main simulation loop
    ui.Init(&system, "Sphere - Decay Test");

    while (system.GetChTime() <= simulationDuration) {
        if (ui.IsRunning(timestep) == false) break;

        if (ui.simulationStarted) {
            system.DoStepDynamics(timestep);

            // append data to output vector
            time_vector.push_back(system.GetChTime());
            heave_position.push_back(sphereBody->GetPos().z());
        }
    }

    // for profiling
    auto end          = std::chrono::high_resolution_clock::now();
    unsigned duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::string out_dir = seastack::chrono::GetDemoOutDir();
    if (profilingOn || saveDataOn) {
        out_dir = out_dir + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directory(std::filesystem::path(out_dir));
    }

    if (profilingOn) {
        const std::string profiling_path = out_dir + "/decay_duration.txt";
        std::ofstream profilingFile(profiling_path);
        if (!profilingFile.is_open()) {
            seastack::infra::cli::LogError("Could not open " + profiling_path);
        } else {
            profilingFile << duration << " ms\n";
            profilingFile.close();
        }
    }

    if (saveDataOn) {
        const std::string output_path = out_dir + "/decay.txt";
        std::ofstream outputFile(output_path);
        if (!outputFile.is_open()) {
            seastack::infra::cli::LogError("Could not open " + output_path);
        } else {
            outputFile << std::left << std::setw(10) << "Time (s)" << std::right << std::setw(12)
                       << "Heave (m)"
                       << std::endl;
            for (size_t i = 0; i < time_vector.size(); ++i)
                outputFile << std::left << std::setw(12) << std::setprecision(6) << std::fixed << time_vector[i]
                           << std::right << std::setw(12) << std::setprecision(6) << std::fixed
                           << heave_position[i]
                           << std::endl;
            outputFile.close();
        }
    }

    return 0;
}
