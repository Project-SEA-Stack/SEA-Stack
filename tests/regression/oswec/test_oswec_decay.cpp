#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <chrono>   // std::chrono::high_resolution_clock::now
#include <iomanip>  // std::setprecision
#include <vector>   // std::vector<double>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

#include "oswec_test_utils.h"

int main(int argc, char* argv[]) {
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    // Initialize environment
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    // Get model file names
    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto body1_meshfile = (DATADIR / "demos" / "oswec" / "geometry" / "flap.obj").lexically_normal().generic_string();
    auto body2_meshfile = (DATADIR / "demos" / "oswec" / "geometry" / "base.obj").lexically_normal().generic_string();
    auto h5fname        = (DATADIR / "demos" / "oswec" / "hydroData" / "oswec.h5").lexically_normal().generic_string();

    // system/solver settings
    ChSystemNSC system;

    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.03;
    system.SetSolverType(ChSolver::Type::GMRES);
    double simulationDuration = seastack::chrono::GetSimDuration(400.0, 800.0);

    // Output timeseries
    std::vector<double> time_vector;
    std::vector<double> flap_rot;

    std::array<double, 3> origin_to_hinge = {0, 0, -8.9};
    std::array<double, 3> hinge_to_cg     = {0, 0, 5};
    std::array<double, 3> axis            = {0, 1, 0};
    double angle_in_degrees               = 10;

    std::array<double, 3> rotated_hinge_to_cg = rotate_vector_3d(hinge_to_cg, axis, angle_in_degrees);
    std::array<double, 3> new_cg = add_vectors(origin_to_hinge, rotated_hinge_to_cg);

    // set up body from a mesh
    std::cout << "Attempting to open mesh file: " << body1_meshfile << std::endl;
    std::shared_ptr<ChBody> flap_body = chrono_types::make_shared<ChBodyEasyMesh>(
        body1_meshfile,
        1000,   // density
        false,  // do not evaluate mass automatically
        true,   // create visualization asset
        false   // collisions
    );

    // define the float's initial conditions
    system.Add(flap_body);
    flap_body->SetName("body1");
    auto ang_rad = CH_PI / 18.0;
    flap_body->SetPos(ChVector3d(new_cg[0], new_cg[1], new_cg[2]));
    flap_body->SetRot(QuatFromAngleY(ang_rad));
    flap_body->SetMass(127000.0);
    flap_body->SetInertiaXX(ChVector3d(1.85e6, 1.85e6, 1.85e6));

    // set up body from a mesh
    std::cout << "Attempting to open mesh file: " << body2_meshfile << std::endl;
    std::shared_ptr<ChBody> base_body = chrono_types::make_shared<ChBodyEasyMesh>(
        body2_meshfile,
        1000,   // density
        false,  // do not evaluate mass automatically
        true,   // create visualization asset
        false   // collisions
    );

    // define the plate's initial conditions
    system.Add(base_body);
    base_body->SetName("body2");
    base_body->SetPos(ChVector3d(0, 0, -10.15));
    base_body->SetMass(999);
    base_body->SetInertiaXX(ChVector3d(1, 1, 1));

    // create ground
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -10.15));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    // fix base to ground with special constraint
    auto anchor = chrono_types::make_shared<ChLinkMateGeneric>();
    anchor->Initialize(base_body, ground, false, base_body->GetVisualModelFrame(), base_body->GetVisualModelFrame());
    system.Add(anchor);
    anchor->SetConstrainedCoords(true, true, true, true, true, true);

    // define base-fore flap joint
    ChQuaternion<> revoluteRot = QuatFromAngleX(CH_PI / 2.0);
    auto revolute              = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(base_body, flap_body, ChFramed(ChVector3d(0.0, 0.0, -8.9), revoluteRot));
    system.AddLink(revolute);

    auto default_dont_add_waves = std::make_shared<NoWave>();

    // set up hydro forces
    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(flap_body);
    bodies.push_back(base_body);
    HydroSystem hydro_forces(bodies, h5fname, default_dont_add_waves);

    // for profiling
    auto start = std::chrono::high_resolution_clock::now();

    // main simulation loop
    while (system.GetChTime() <= simulationDuration) {
        system.DoStepDynamics(timestep);

        // append data to output vector
        time_vector.push_back(system.GetChTime());
        flap_rot.push_back(flap_body->GetRot().GetCardanAnglesXYZ().y());
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
        outputFile << std::left << std::setw(10) << "Time (s)" << std::right << std::setw(16)
                   << "Flap Rotation y (radians)" << std::right << std::setw(16) << "Flap Rotation y (degrees)"
                   << std::endl;
        for (size_t i = 0; i < time_vector.size(); ++i)
            outputFile << std::left << std::setw(10) << std::setprecision(2) << std::fixed << time_vector[i]
                       << std::right << std::setw(16) << std::setprecision(4) << std::fixed << flap_rot[i]
                       << std::right << std::setw(16) << std::setprecision(4) << std::fixed
                       << flap_rot[i] * 360.0 / (2.0 * CH_PI) << std::endl;
        outputFile.close();
    } else {
        std::cout << "Error: Could not open output file for writing." << std::endl;
        return 1;
    }

    return 0;
}
