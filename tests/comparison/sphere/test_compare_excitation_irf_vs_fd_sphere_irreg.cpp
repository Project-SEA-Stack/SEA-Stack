/**
 * Internal method comparison: kPolar vs kCartesian excitation interpolation
 * on the sphere irregular wave case.
 *
 * Runs the same sea state twice with different interpolation methods and
 * writes both time series for post-hoc comparison.
 */

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/excitation_types.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

struct RunResult {
    std::vector<double> time;
    std::vector<double> heave;
};

static RunResult run_sim(const std::string& mesh, const std::string& h5,
                         ExcitationInterpolation interp, const std::string& label) {
    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.015;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    double duration = seastack::chrono::GetSimDuration(300.0, 600.0);

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -5));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto body = chrono_types::make_shared<ChBodyEasyMesh>(mesh, 1000, false, true, false);
    system.Add(body);
    body->SetName("body1");
    body->SetPos(ChVector3d(0, 0, -2));
    body->SetMass(261.8e3);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(body, ground, false, ChFramed(ChVector3d(0, 0, -2)),
                          ChFramed(ChVector3d(0, 0, -5)));
    system.AddLink(prismatic);

    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body, ground, false, ChVector3d(0, 0, -2), ChVector3d(0, 0, -5));
    spring->SetSpringCoefficient(0.0);
    spring->SetDampingCoefficient(0.0);
    system.AddLink(spring);

    SeaStateDefinition sea_state;
    sea_state.type = "irregular";
    SeaStatePartition part;
    part.spectrum.type = "jonswap";
    part.spectrum.Hs = 2.0;
    part.spectrum.Tp = 12.0;
    part.spectrum.gamma = 1.0;
    sea_state.partitions.push_back(part);
    sea_state.omega_min = 2.0 * M_PI * 0.001;
    sea_state.omega_max = 2.0 * M_PI * 1.0;
    sea_state.n_omega = 1000;
    sea_state.seed = 1;

    auto components = ComponentSampler::Build(sea_state);
    auto wave_field = std::make_shared<LinearDirectionalWaveField>(
        std::move(components), sea_state.depth);
    wave_field->SetRampDuration(60.0);

    std::vector<std::shared_ptr<ChBody>> bodies = {body};
    HydroSystem hydro(bodies, h5);
    hydro.SetExcitationInterpolation(interp);
    hydro.AddWaves(wave_field);

    RunResult result;
    auto start = std::chrono::high_resolution_clock::now();
    while (system.GetChTime() <= duration) {
        system.DoStepDynamics(timestep);
        result.time.push_back(system.GetChTime());
        result.heave.push_back(body->GetPos().z());
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "  " << label << " completed in " << ms << " ms\n";

    return result;
}

static void write_results(const RunResult& r, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "Error: cannot write " << path << "\n";
        return;
    }
    f << std::left << std::setw(14) << "# Time(s)" << std::right << std::setw(14) << "Heave(m)\n";
    for (size_t i = 0; i < r.time.size(); ++i)
        f << std::left << std::setw(14) << std::fixed << std::setprecision(6) << r.time[i]
          << std::right << std::setw(14) << std::fixed << std::setprecision(6) << r.heave[i] << "\n";
}

int main(int argc, char* argv[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    auto mesh = (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();
    auto h5   = (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);

    std::cout << "Running kPolar (legacy) interpolation...\n";
    auto polar = run_sim(mesh, h5, ExcitationInterpolation::kPolar, "kPolar");
    write_results(polar, out_dir + "/polar.txt");

    std::cout << "Running kCartesian (improved) interpolation...\n";
    auto cart = run_sim(mesh, h5, ExcitationInterpolation::kCartesian, "kCartesian");
    write_results(cart, out_dir + "/cartesian.txt");

    return 0;
}
