/**
 * Benchmark: sphere in irregular waves with RIRF convolution radiation
 * and IRF excitation. Core single-body irregular-wave case.
 */

#include <bench_utils.h>

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/excitation_types.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <filesystem>
#include <string>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

static const int kNumTrials = 3;
static const double kTimestep = 0.015;
static const double kSimDuration = 600.0;

int main(int argc, char* argv[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    auto mesh = (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();
    auto h5   = (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();

    auto meta = seastack::bench::CollectMetadata();
    auto cli = seastack::bench::ParseBenchmarkArgs(argc, argv);
    int num_trials = seastack::bench::ResolveTrialCount(kNumTrials, cli);
    bool warmup = seastack::bench::ResolveWarmup(true, cli);

    seastack::bench::BenchmarkSettings settings;
    settings.timestep = kTimestep;
    settings.sim_duration = kSimDuration;
    settings.num_steps = static_cast<int>(kSimDuration / kTimestep);
    settings.num_bodies = 1;
    settings.radiation_method = "rirf_convolution";
    settings.excitation_method = "irf_convolution";
    settings.wave_type = "irregular_jonswap";
    settings.num_trials = num_trials;
    settings.warmup = warmup;

    std::string started_at = seastack::bench::GetISOTimestamp();

    auto trial_fn = [&]() -> seastack::bench::TrialResult {
        seastack::bench::Timer timer;
        seastack::bench::TrialResult result;

        timer.Start();

        ChSystemNSC system;
        system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
        system.SetSolverType(ChSolver::Type::SPARSE_QR);

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
        hydro.AddWaves(wave_field);
        hydro.SetProfilingEnabled(true);

        result.setup_wall_s = timer.StopSeconds();

        timer.Start();
        while (system.GetChTime() <= kSimDuration) {
            system.DoStepDynamics(kTimestep);
        }
        result.sim_wall_s = timer.StopSeconds();
        result.total_wall_s = result.setup_wall_s + result.sim_wall_s;
        result.components = seastack::bench::FromProfileStats(hydro.GetProfileStats());
        return result;
    };

    auto trials = seastack::bench::RunTrials(num_trials, warmup, trial_fn);
    std::string finished_at = seastack::bench::GetISOTimestamp();

    seastack::bench::PrintTrialSummary("sphere_irreg_conv", settings, trials);

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);
    seastack::bench::WriteBenchmarkJSON(
        out_dir + "/" + RESULTS_FILE_NAME + ".json",
        "sphere_irreg_conv", started_at, finished_at, meta, settings, trials);

    return 0;
}
