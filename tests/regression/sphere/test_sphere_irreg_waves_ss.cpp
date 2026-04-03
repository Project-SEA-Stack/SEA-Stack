#include <chrono>
#include <filesystem>
#include <iomanip>
#include <vector>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <seastack/core/math_constants.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

int main(int argc, char* argv[]) {
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto body1_meshfile =
        (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();
    auto h5fname = (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.015;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    double simulationDuration = seastack::chrono::GetSimDuration(600.0, 1200.0);

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -5));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    std::vector<double> time_vector;
    std::vector<double> heave_position;

    std::cout << "Attempting to open mesh file: " << body1_meshfile << std::endl;
    std::shared_ptr<ChBody> sphereBody = chrono_types::make_shared<ChBodyEasyMesh>(
        body1_meshfile,
        1000,
        false,
        true,
        false
    );

    system.Add(sphereBody);
    sphereBody->SetName("body1");
    sphereBody->SetPos(ChVector3d(0, 0, -2));
    sphereBody->SetMass(261.8e3);

    std::cout << "Body created from the mesh file: " << body1_meshfile << std::endl;

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(sphereBody, ground, false, ChFramed(ChVector3d(0, 0, -2)), ChFramed(ChVector3d(0, 0, -5)));
    system.AddLink(prismatic);

    double spring_coef  = 0.0;
    double damping_coef = 0.0;
    auto spring_1       = chrono_types::make_shared<ChLinkTSDA>();
    spring_1->Initialize(sphereBody, ground, false, ChVector3d(0, 0, -2), ChVector3d(0, 0, -5));
    spring_1->SetSpringCoefficient(spring_coef);
    spring_1->SetDampingCoefficient(damping_coef);
    system.AddLink(spring_1);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(sphereBody);

    SeaStateDefinition sea_state;
    sea_state.type = "irregular";
    SeaStatePartition partition;
    partition.spectrum.type = "jonswap";
    partition.spectrum.Hs = 2.0;
    partition.spectrum.Tp = 12.0;
    partition.spectrum.gamma = 1.0;
    sea_state.partitions.push_back(partition);
    sea_state.omega_min = 2.0 * M_PI * 0.001;
    sea_state.omega_max = 2.0 * M_PI * 1.0;
    sea_state.n_omega = 1000;
    sea_state.seed = 1;

    auto components = ComponentSampler::Build(sea_state);
    auto my_hydro_inputs = std::make_shared<LinearDirectionalWaveField>(
        std::move(components), sea_state.depth);
    my_hydro_inputs->SetRampDuration(60.0);

    HydroSystem hydro_forces(bodies, h5fname);
    hydro_forces.AddWaves(my_hydro_inputs);

    hydro_forces.SetRadiationMethod(seastack::hydro::RadiationMethod::kStateSpace);
    seastack::hydro::StateSpaceOptions ss_opts;
    ss_opts.max_order = 10;
    ss_opts.r2_threshold = 0.99;
    hydro_forces.SetStateSpaceOptions(ss_opts);

    auto start = std::chrono::high_resolution_clock::now();

    while (system.GetChTime() <= simulationDuration) {
        system.DoStepDynamics(timestep);
        time_vector.push_back(system.GetChTime());
        heave_position.push_back(sphereBody->GetPos().z());
    }

    auto end          = std::chrono::high_resolution_clock::now();
    unsigned duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Simulation completed in " << duration << " ms" << std::endl;

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
}
