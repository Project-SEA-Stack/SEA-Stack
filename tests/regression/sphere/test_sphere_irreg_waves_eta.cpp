#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

int main(int argc, char* argv[]) {
    std::cout << "=== SPHERE IRREGULAR WAVES ETA TEST STARTING ===" << std::endl;
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    try {
        // Initialize environment
        std::string data_dir;
        if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

        std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
        std::cout << "DEBUG: Data directory: " << DATADIR << std::endl;

        auto body1_meshfile =
            (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();
        auto h5fname = (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();

        auto eta_file_path =
            (DATADIR / "demos" / "sphere" / "eta" / "eta.txt").lexically_normal();
        if (!std::filesystem::exists(eta_file_path)) {
            std::cerr << "ERROR: ETA file not found at " << eta_file_path << std::endl;
            return 1;
        }

        // Check if files exist
        if (!std::filesystem::exists(body1_meshfile)) {
            std::cerr << "ERROR: Mesh file does not exist: " << body1_meshfile << std::endl;
            return 1;
        }
        if (!std::filesystem::exists(h5fname)) {
            std::cerr << "ERROR: H5 file does not exist: " << h5fname << std::endl;
            return 1;
        }

        // system/solver settings
        ChSystemNSC system;
        system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
        double timestep = 0.015;
        system.SetSolverType(ChSolver::Type::SPARSE_QR);
        double simulationDuration = seastack::chrono::GetSimDuration(600.0, 1200.0);

        // Setup Ground
        auto ground = chrono_types::make_shared<ChBody>();
        system.AddBody(ground);
        ground->SetPos(ChVector3d(0, 0, -5));
        ground->SetTag(-1);
        ground->SetFixed(true);
        ground->EnableCollision(false);

        // Output timeseries
        std::vector<double> time_vector;
        std::vector<double> heave_position;

        // set up body from a mesh
        std::cout << "Attempting to open mesh file: " << body1_meshfile << std::endl;
        std::shared_ptr<ChBody> sphereBody = chrono_types::make_shared<ChBodyEasyMesh>(
            body1_meshfile,
            1000,   // density
            false,  // do not evaluate mass automatically
            true,   // create visualization asset
            false   // do not collide
        );

        // define the body's initial conditions
        system.Add(sphereBody);
        sphereBody->SetName("body1");  // must set body name correctly! (must match .h5 file)
        sphereBody->SetPos(ChVector3d(0, 0, -2));
        sphereBody->SetMass(261.8e3);

        std::cout << "Body created from the mesh file: " << body1_meshfile << std::endl;

        // add prismatic joint between sphere and ground (limit to heave motion only)
        auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        prismatic->Initialize(sphereBody, ground, false, ChFramed(ChVector3d(0, 0, -2)),
                              ChFramed(ChVector3d(0, 0, -5)));
        system.AddLink(prismatic);

        // create the spring between body_1 and ground
        double spring_coef  = 0.0;
        double damping_coef = 0.0;
        auto spring_1       = chrono_types::make_shared<ChLinkTSDA>();
        spring_1->Initialize(sphereBody, ground, false, ChVector3d(0, 0, -2), ChVector3d(0, 0, -5));
        spring_1->SetSpringCoefficient(spring_coef);
        spring_1->SetDampingCoefficient(damping_coef);
        system.AddLink(spring_1);

        std::vector<std::shared_ptr<ChBody>> bodies;
        bodies.push_back(sphereBody);

        // Use ETA file for irregular waves
        std::shared_ptr<seastack::hydro::LinearDirectionalWaveField> my_hydro_inputs;

        try {
            auto components = seastack::hydro::ComponentSampler::BuildFromEtaFile(
                eta_file_path.generic_string(), 0.0, 9.81, 1000, 0.001, 1.0);
            my_hydro_inputs = std::make_shared<seastack::hydro::LinearDirectionalWaveField>(
                std::move(components), 0.0);
        } catch (const std::exception& e) {
            std::cerr << "Caught exception: " << e.what() << '\n';
            return 1;
        } catch (...) {
            std::cerr << "Caught unknown exception.\n";
            return 1;
        }

        if (!my_hydro_inputs) {
            std::cerr << "ERROR: Failed to create wave field." << std::endl;
            return 1;
        }

        HydroSystem hydro_forces(bodies, h5fname);
        hydro_forces.AddWaves(my_hydro_inputs);

        // for profiling
        auto start = std::chrono::high_resolution_clock::now();

        // main simulation loop
        while (system.GetChTime() <= simulationDuration) {
            system.DoStepDynamics(timestep);

            // append data to output vector
            time_vector.push_back(system.GetChTime());
            heave_position.push_back(sphereBody->GetPos().z());
        }

        // for profiling
        auto end          = std::chrono::high_resolution_clock::now();
        unsigned duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Simulation completed in " << duration << " ms" << std::endl;

        // Save results
        std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directories(std::filesystem::path(out_dir));

        std::ofstream outputFile(out_dir + "/" + RESULTS_FILE_NAME + ".txt");
        if (outputFile.is_open()) {
            outputFile.precision(10);
            outputFile.width(12);
            outputFile << std::left << std::setw(10) << "Time (s)" << std::right << std::setw(12) << "Heave (m)" << "\n";
            outputFile << std::left << std::setw(10) << "----------" << std::right << std::setw(12) << "----------" << "\n";

            for (size_t i = 0; i < time_vector.size(); i++) {
                outputFile << std::left << std::setw(10) << std::fixed << std::setprecision(3) << time_vector[i]
                           << std::right << std::setw(12) << std::fixed << std::setprecision(6) << heave_position[i]
                           << "\n";
            }
            outputFile.close();
        } else {
            std::cout << "Error: Could not open output file for writing." << std::endl;
            return 1;
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FATAL ERROR: Unhandled exception in main: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "FATAL ERROR: Unknown unhandled exception in main" << std::endl;
        return 1;
    }
}
