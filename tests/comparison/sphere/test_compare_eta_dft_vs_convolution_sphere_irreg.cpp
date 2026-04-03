/**
 * Internal method comparison: DFT-based eta-import vs direct eta convolution
 * on the sphere irregular wave case.
 *
 * 1. Generates an eta time series from a JONSWAP spectrum.
 * 2. Runs two simulations with the same eta:
 *    A) EtaTableWaveField — direct interpolation (convolution pathway)
 *    B) BuildFromEtaFile + LinearDirectionalWaveField (DFT pathway)
 * 3. Writes both heave responses and both eta traces for post-hoc comparison.
 */

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/eta_table_wave_field.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

static constexpr double kHs       = 2.0;
static constexpr double kTp       = 12.0;
static constexpr double kGamma    = 1.0;
static constexpr int    kNOmega   = 1000;
static constexpr int    kSeed     = 1;
static constexpr double kDt       = 0.015;
static constexpr double kRamp     = 60.0;
static constexpr double kDepth    = 0.0;

struct RunResult {
    std::vector<double> time;
    std::vector<double> heave;
    std::vector<double> eta;
};

static std::string generate_eta_file(const std::string& out_dir,
                                     double duration) {
    SeaStateDefinition def;
    def.type = "irregular";
    SeaStatePartition part;
    part.spectrum.type  = "jonswap";
    part.spectrum.Hs    = kHs;
    part.spectrum.Tp    = kTp;
    part.spectrum.gamma = kGamma;
    def.partitions.push_back(part);
    def.omega_min = 2.0 * M_PI * 0.001;
    def.omega_max = 2.0 * M_PI * 1.0;
    def.n_omega   = kNOmega;
    def.seed      = kSeed;

    auto components = ComponentSampler::Build(def);
    auto wave = std::make_shared<LinearDirectionalWaveField>(
        std::move(components), kDepth);

    std::string eta_path = out_dir + "/eta_input.txt";
    std::ofstream f(eta_path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot write " + eta_path);
    }

    const Eigen::Vector3d origin(0, 0, 0);
    int n_steps = static_cast<int>(std::ceil(duration / kDt)) + 2;
    f << std::fixed << std::setprecision(8);
    for (int i = 0; i < n_steps; ++i) {
        double t = i * kDt;
        double eta = wave->GetElevation(origin, t);
        f << t << ":" << eta << "\n";
    }
    f.close();
    std::cout << "  Generated eta file: " << eta_path
              << " (" << n_steps << " samples)\n";
    return eta_path;
}

static RunResult run_convolution(const std::string& eta_path,
                                 const std::string& mesh,
                                 const std::string& h5,
                                 double duration) {
    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    system.SetSolverType(ChSolver::Type::SPARSE_QR);

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -5));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto body = chrono_types::make_shared<ChBodyEasyMesh>(
        mesh, 1000, false, true, false);
    system.Add(body);
    body->SetName("body1");
    body->SetPos(ChVector3d(0, 0, -2));
    body->SetMass(261.8e3);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(body, ground, false,
                          ChFramed(ChVector3d(0, 0, -2)),
                          ChFramed(ChVector3d(0, 0, -5)));
    system.AddLink(prismatic);

    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body, ground, false,
                       ChVector3d(0, 0, -2), ChVector3d(0, 0, -5));
    spring->SetSpringCoefficient(0.0);
    spring->SetDampingCoefficient(0.0);
    system.AddLink(spring);

    auto waves = std::make_shared<EtaTableWaveField>(eta_path, kDepth);
    waves->SetRampDuration(kRamp);

    std::vector<std::shared_ptr<ChBody>> bodies = {body};
    HydroSystem hydro(bodies, h5);
    hydro.AddWaves(waves);

    const Eigen::Vector3d origin(0, 0, 0);
    RunResult result;
    auto start = std::chrono::high_resolution_clock::now();
    while (system.GetChTime() <= duration) {
        system.DoStepDynamics(kDt);
        result.time.push_back(system.GetChTime());
        result.heave.push_back(body->GetPos().z());
        result.eta.push_back(waves->GetElevation(origin, system.GetChTime()));
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end - start).count();
    std::cout << "  Convolution pathway completed in " << ms << " ms\n";
    return result;
}

static RunResult run_dft(const std::string& eta_path,
                         const std::string& mesh,
                         const std::string& h5,
                         double duration) {
    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    system.SetSolverType(ChSolver::Type::SPARSE_QR);

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -5));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto body = chrono_types::make_shared<ChBodyEasyMesh>(
        mesh, 1000, false, true, false);
    system.Add(body);
    body->SetName("body1");
    body->SetPos(ChVector3d(0, 0, -2));
    body->SetMass(261.8e3);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(body, ground, false,
                          ChFramed(ChVector3d(0, 0, -2)),
                          ChFramed(ChVector3d(0, 0, -5)));
    system.AddLink(prismatic);

    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body, ground, false,
                       ChVector3d(0, 0, -2), ChVector3d(0, 0, -5));
    spring->SetSpringCoefficient(0.0);
    spring->SetDampingCoefficient(0.0);
    system.AddLink(spring);

    auto components = ComponentSampler::BuildFromEtaFile(
        eta_path, kDepth, 9.81, kNOmega, 0.001, 1.0);
    auto waves = std::make_shared<LinearDirectionalWaveField>(
        std::move(components), kDepth);
    waves->SetRampDuration(kRamp);

    std::vector<std::shared_ptr<ChBody>> bodies = {body};
    HydroSystem hydro(bodies, h5);
    hydro.AddWaves(waves);

    const Eigen::Vector3d origin(0, 0, 0);
    RunResult result;
    auto start = std::chrono::high_resolution_clock::now();
    while (system.GetChTime() <= duration) {
        system.DoStepDynamics(kDt);
        result.time.push_back(system.GetChTime());
        result.heave.push_back(body->GetPos().z());
        result.eta.push_back(waves->GetElevation(origin, system.GetChTime()));
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end - start).count();
    std::cout << "  DFT pathway completed in " << ms << " ms\n";
    return result;
}

static void write_heave(const RunResult& r, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "Error: cannot write " << path << "\n";
        return;
    }
    f << std::left << std::setw(14) << "# Time(s)"
      << std::right << std::setw(14) << "Heave(m)\n";
    for (size_t i = 0; i < r.time.size(); ++i) {
        f << std::left  << std::setw(14) << std::fixed << std::setprecision(6)
          << r.time[i]
          << std::right << std::setw(14) << std::fixed << std::setprecision(6)
          << r.heave[i] << "\n";
    }
}

static void write_eta(const RunResult& r, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "Error: cannot write " << path << "\n";
        return;
    }
    f << std::left << std::setw(14) << "# Time(s)"
      << std::right << std::setw(14) << "Elevation(m)\n";
    for (size_t i = 0; i < r.time.size(); ++i) {
        f << std::left  << std::setw(14) << std::fixed << std::setprecision(6)
          << r.time[i]
          << std::right << std::setw(14) << std::fixed << std::setprecision(6)
          << r.eta[i] << "\n";
    }
}

int main(int argc, char* argv[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    auto mesh = (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj")
                    .lexically_normal().generic_string();
    auto h5 = (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5")
                  .lexically_normal().generic_string();

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/"
                          + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);

    double duration = seastack::chrono::GetSimDuration(300.0, 600.0);

    std::cout << "Generating reference eta from JONSWAP spectrum...\n";
    std::string eta_path = generate_eta_file(out_dir, duration);

    {
        double eta_last_time =
            (static_cast<int>(std::ceil(duration / kDt)) + 1) * kDt;
        double sim_last_query = duration + kDt;
        if (eta_last_time < sim_last_query - 1e-12) {
            std::cerr << "FATAL: eta table last time (" << eta_last_time
                      << " s) does not cover the simulation's last query time ("
                      << sim_last_query << " s). Extend generate_eta_file.\n";
            return 1;
        }
    }

    std::cout << "Running convolution pathway (EtaTableWaveField)...\n";
    auto conv = run_convolution(eta_path, mesh, h5, duration);
    write_heave(conv, out_dir + "/convolution.txt");
    write_eta(conv, out_dir + "/convolution_eta.txt");

    std::cout << "Running DFT pathway (BuildFromEtaFile + LDWF)...\n";
    auto dft = run_dft(eta_path, mesh, h5, duration);
    write_heave(dft, out_dir + "/dft.txt");
    write_eta(dft, out_dir + "/dft_eta.txt");

    return 0;
}
