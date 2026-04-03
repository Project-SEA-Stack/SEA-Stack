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

    auto body1_meshfile = (DATADIR / "demos" / "oswec" / "geometry" / "flap.obj").lexically_normal().generic_string();
    auto body2_meshfile = (DATADIR / "demos" / "oswec" / "geometry" / "base.obj").lexically_normal().generic_string();
    auto h5fname        = (DATADIR / "demos" / "oswec" / "hydroData" / "oswec.h5").lexically_normal().generic_string();

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.015;
    system.SetSolverType(ChSolver::Type::GMRES);
    double simulationDuration = seastack::chrono::GetSimDuration(200.0, 400.0);

    std::vector<double> time_vector;
    std::vector<double> flap_rot;

    std::cout << "Attempting to open mesh file: " << body1_meshfile << std::endl;
    std::shared_ptr<ChBody> flap_body = chrono_types::make_shared<ChBodyEasyMesh>(
        body1_meshfile,
        1000,
        false,
        true,
        false
    );

    system.Add(flap_body);
    flap_body->SetName("body1");
    flap_body->SetPos(ChVector3d(0, 0, -3.9));
    flap_body->SetMass(127000.0);
    flap_body->SetInertiaXX(ChVector3d(1.85e6, 1.85e6, 1.85e6));

    std::cout << "Attempting to open mesh file: " << body2_meshfile << std::endl;
    std::shared_ptr<ChBody> base_body = chrono_types::make_shared<ChBodyEasyMesh>(
        body2_meshfile,
        1000,
        false,
        true,
        false
    );

    system.Add(base_body);
    base_body->SetName("body2");
    base_body->SetPos(ChVector3d(0, 0, -10.15));
    base_body->SetMass(999);
    base_body->SetInertiaXX(ChVector3d(1, 1, 1));

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -10.15));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto anchor = chrono_types::make_shared<ChLinkMateGeneric>();
    anchor->Initialize(base_body, ground, false, base_body->GetVisualModelFrame(), base_body->GetVisualModelFrame());
    system.Add(anchor);
    anchor->SetConstrainedCoords(true, true, true, true, true, true);

    ChQuaternion<> revoluteRot = QuatFromAngleX(CH_PI / 2.0);
    auto revolute              = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(base_body, flap_body, ChFramed(ChVector3d(0.0, 0.0, -8.9), revoluteRot));
    system.AddLink(revolute);

    // PTO damper between flap and base
    auto pto = chrono_types::make_shared<ChLinkRSDA>();
    pto->Initialize(flap_body, base_body, ChFramed(ChVector3d(0.0, 0.0, -8.9), revoluteRot));
    pto->SetSpringCoefficient(0.0);
    pto->SetDampingCoefficient(12.0e6);
    system.AddLink(pto);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(flap_body);
    bodies.push_back(base_body);

    SeaStateDefinition sea_state;
    sea_state.type = "irregular";
    SeaStatePartition partition;
    partition.spectrum.type = "jonswap";
    partition.spectrum.Hs = 2.5;
    partition.spectrum.Tp = 8.0;
    partition.spectrum.gamma = 1.0;
    sea_state.partitions.push_back(partition);
    sea_state.omega_min = 2.0 * M_PI * 0.001;
    sea_state.omega_max = 2.0 * M_PI * 1.0;
    sea_state.n_omega = 1000;
    sea_state.seed = 1;

    auto components = ComponentSampler::Build(sea_state);
    auto my_hydro_inputs = std::make_shared<LinearDirectionalWaveField>(
        std::move(components), sea_state.depth);
    my_hydro_inputs->SetRampDuration(30.0);

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
        flap_rot.push_back(flap_body->GetRot().GetCardanAnglesXYZ().y());
    }

    auto end          = std::chrono::high_resolution_clock::now();
    unsigned duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Simulation completed in " << duration << " ms" << std::endl;

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(std::filesystem::path(out_dir));

    std::ofstream outputFile(out_dir + "/" + RESULTS_FILE_NAME + ".txt");
    if (outputFile.is_open()) {
        outputFile << std::left << std::setw(10) << "Time (s)" << std::right << std::setw(16)
                   << "Flap Rotation y (radians)" << std::right << std::setw(16) << "Flap Rotation y (degrees)"
                   << std::endl;
        for (size_t i = 0; i < time_vector.size(); ++i)
            outputFile << std::left << std::setw(10) << std::setprecision(3) << std::fixed << time_vector[i]
                       << std::right << std::setw(16) << std::setprecision(6) << std::fixed << flap_rot[i]
                       << std::right << std::setw(16) << std::setprecision(6) << std::fixed
                       << flap_rot[i] * 360.0 / (2.0 * CH_PI) << std::endl;
        outputFile.close();
    } else {
        std::cout << "Error: Could not open output file for writing." << std::endl;
        return 1;
    }

    return 0;
}
