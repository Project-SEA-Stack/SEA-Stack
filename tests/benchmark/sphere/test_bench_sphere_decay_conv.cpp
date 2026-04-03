/**
 * Benchmark: sphere heave decay with RIRF convolution radiation.
 * Isolates radiation convolution cost with no wave excitation.
 */

#include <bench_utils.h>

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <filesystem>
#include <string>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

static const int kNumTrials = 3;
static const double kTimestep = 0.015;
static const double kSimDuration = 100.0;

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
    settings.excitation_method = "none";
    settings.wave_type = "none";
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

        auto body = chrono_types::make_shared<ChBodyEasyMesh>(mesh, 1000, false, true, false);
        system.Add(body);
        body->SetName("body1");
        body->SetPos(ChVector3d(0, 0, -1));
        body->SetMass(261.8e3);

        auto no_waves = std::make_shared<NoWave>();
        std::vector<std::shared_ptr<ChBody>> bodies = {body};
        HydroSystem hydro(bodies, h5);
        hydro.AddWaves(no_waves);
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

    seastack::bench::PrintTrialSummary("sphere_decay_conv", settings, trials);

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);
    seastack::bench::WriteBenchmarkJSON(
        out_dir + "/" + RESULTS_FILE_NAME + ".json",
        "sphere_decay_conv", started_at, finished_at, meta, settings, trials);

    return 0;
}
