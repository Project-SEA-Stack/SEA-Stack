#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

int main(int argc, char* argv[]) {
    std::cout << "=== SPHERE DECAY MULTICODE VERIFICATION ===" << std::endl;
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto body1_meshfname =
        (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();
    auto h5fname =
        (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();

    if (!std::filesystem::exists(body1_meshfname)) {
        std::cerr << "ERROR: sphere mesh not found at " << body1_meshfname << std::endl;
        return 1;
    }
    if (!std::filesystem::exists(h5fname)) {
        std::cerr << "ERROR: sphere.h5 not found at " << h5fname << std::endl;
        return 1;
    }

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));

    double timestep = 0.015;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    double simulationDuration = seastack::chrono::GetSimDuration(40.0, 100.0);

    std::vector<double> time_vector;
    std::vector<double> heave_position;

    std::shared_ptr<ChBody> sphereBody = chrono_types::make_shared<ChBodyEasyMesh>(
        body1_meshfname, 1000, false, true, false);

    sphereBody->SetName("body1");
    sphereBody->SetPos(ChVector3d(0, 0, -1));
    sphereBody->SetMass(261.8e3);
    system.Add(sphereBody);

    auto no_waves = std::make_shared<NoWave>();

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(sphereBody);

    HydroSystem hydro_forces(bodies, h5fname);
    hydro_forces.AddWaves(no_waves);

    auto start = std::chrono::high_resolution_clock::now();

    while (system.GetChTime() <= simulationDuration) {
        system.DoStepDynamics(timestep);
        time_vector.push_back(system.GetChTime());
        heave_position.push_back(sphereBody->GetPos().z());
    }

    auto end = std::chrono::high_resolution_clock::now();
    unsigned duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Simulation completed in " << duration << " ms" << std::endl;

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(std::filesystem::path(out_dir));

    std::ofstream outputFile(out_dir + "/" + RESULTS_FILE_NAME + ".txt");
    if (outputFile.is_open()) {
        outputFile << std::left << std::setw(10) << "Time (s)"
                   << std::right << std::setw(12) << "Heave (m)" << std::endl;
        for (size_t i = 0; i < time_vector.size(); ++i)
            outputFile << std::left << std::setw(12) << std::setprecision(6) << std::fixed << time_vector[i]
                       << std::right << std::setw(12) << std::setprecision(6) << std::fixed << heave_position[i]
                       << std::endl;
        outputFile.close();
    } else {
        std::cerr << "Error: Could not open output file for writing." << std::endl;
        return 1;
    }

    return 0;
}
