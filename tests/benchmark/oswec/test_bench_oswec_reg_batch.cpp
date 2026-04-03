/**
 * Benchmark: OSWEC regular-wave batch (16 conditions).
 *
 * Execution modes:
 *   1. Per-condition (--condition N): runs condition N with proper trial
 *      methodology (warmup + multiple trials) and writes a per-condition JSON.
 *   2. Full batch (default): runs all 16 conditions sequentially in a single
 *      process for aggregate throughput measurement.
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
#include <iostream>
#include <string>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

static const int kNumTrials = 3;
static const double kTimestep = 0.03;
static const double kSimDuration = 1000.0;

static const std::vector<double> kPeriods = {
    4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0,
    18.5, 19.0, 19.25, 19.5, 20.0, 21.0, 22.0, 24.0};

seastack::bench::TrialResult RunCondition(
    int condition_index,
    const std::string& flap_mesh,
    const std::string& base_mesh,
    const std::string& h5) {

    seastack::bench::Timer timer;
    seastack::bench::TrialResult result;

    timer.Start();

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    system.SetSolverType(ChSolver::Type::GMRES);

    auto flap_body = chrono_types::make_shared<ChBodyEasyMesh>(
        flap_mesh, 1000, false, true, false);
    system.Add(flap_body);
    flap_body->SetName("body1");
    flap_body->SetPos(ChVector3d(0.0, 0.0, -3.9));
    flap_body->SetMass(127000.0);
    flap_body->SetInertiaXX(ChVector3d(1.85e6, 1.85e6, 1.85e6));

    auto base_body = chrono_types::make_shared<ChBodyEasyMesh>(
        base_mesh, 1000, false, true, false);
    system.Add(base_body);
    base_body->SetName("body2");
    base_body->SetPos(ChVector3d(0, 0, -10.15));
    base_body->SetMass(1e9);
    base_body->SetInertiaXX(ChVector3d(1e6, 1e6, 1e6));

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -10.15));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto anchor = chrono_types::make_shared<ChLinkMateGeneric>();
    anchor->Initialize(base_body, ground, false,
                       base_body->GetVisualModelFrame(),
                       base_body->GetVisualModelFrame());
    system.Add(anchor);
    anchor->SetConstrainedCoords(true, true, true, true, true, true);

    ChQuaternion<> revoluteRot = QuatFromAngleX(CH_PI / 2.0);
    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(base_body, flap_body,
                         ChFramed(ChVector3d(0.0, 0.0, -8.9), revoluteRot));
    system.AddLink(revolute);

    SeaStateDefinition sea_state;
    sea_state.type = "regular";
    sea_state.amplitude = 0.01;
    sea_state.omega = 2.0 * M_PI / kPeriods[condition_index];
    auto components = ComponentSampler::Build(sea_state);
    auto waves = std::make_shared<LinearDirectionalWaveField>(
        std::move(components), sea_state.depth);

    std::vector<std::shared_ptr<ChBody>> bodies = {flap_body, base_body};
    // HDF5 is not thread-safe, but with process-level parallelism each
    // condition runs in its own process, so no critical section needed
    HydroSystem hydro(bodies, h5);
    hydro.SetExcitationInterpolation(ExcitationInterpolation::kPolar);
    hydro.AddWaves(waves);
    hydro.SetProfilingEnabled(true);

    result.setup_wall_s = timer.StopSeconds();

    timer.Start();
    while (system.GetChTime() < kSimDuration - kTimestep / 2.0) {
        system.DoStepDynamics(kTimestep);
    }
    result.sim_wall_s = timer.StopSeconds();
    result.total_wall_s = result.setup_wall_s + result.sim_wall_s;
    result.components = seastack::bench::FromProfileStats(hydro.GetProfileStats());
    return result;
}

int main(int argc, char* argv[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    auto flap_mesh = (DATADIR / "demos" / "oswec" / "geometry" / "flap.obj").lexically_normal().generic_string();
    auto base_mesh = (DATADIR / "demos" / "oswec" / "geometry" / "base.obj").lexically_normal().generic_string();
    auto h5 = (DATADIR / "demos" / "oswec" / "hydroData" / "oswec.h5").lexically_normal().generic_string();

    int num_conditions = static_cast<int>(kPeriods.size());
    int num_steps = static_cast<int>(kSimDuration / kTimestep);

    auto meta = seastack::bench::CollectMetadata();
    auto cli = seastack::bench::ParseBenchmarkArgs(argc, argv);

    // ── Per-condition mode ──────────────────────────────────────────────────
    if (cli.condition > 0) {
        int c = cli.condition - 1;
        if (c >= num_conditions) {
            std::cerr << "Error: --condition must be 1-" << num_conditions << std::endl;
            return 1;
        }

        int num_trials = seastack::bench::ResolveTrialCount(kNumTrials, cli);
        bool warmup = seastack::bench::ResolveWarmup(true, cli);

        seastack::bench::BenchmarkSettings settings;
        settings.timestep = kTimestep;
        settings.sim_duration = kSimDuration;
        settings.num_steps = num_steps;
        settings.num_bodies = 2;
        settings.radiation_method = "rirf_convolution";
        settings.excitation_method = "frequency_domain";
        std::ostringstream wt;
        wt << "regular_T" << kPeriods[c];
        settings.wave_type = wt.str();
        settings.num_trials = num_trials;
        settings.warmup = warmup;

        std::string case_id = "oswec_reg_condition_" + std::to_string(cli.condition);
        std::cout << "=== " << case_id << " (T=" << kPeriods[c] << "s) ===" << std::endl;

        std::string started_at = seastack::bench::GetISOTimestamp();

        auto trial_fn = [&]() -> seastack::bench::TrialResult {
            return RunCondition(c, flap_mesh, base_mesh, h5);
        };

        auto trials = seastack::bench::RunTrials(num_trials, warmup, trial_fn);
        std::string finished_at = seastack::bench::GetISOTimestamp();

        seastack::bench::PrintTrialSummary(case_id, settings, trials);

        std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directories(out_dir);
        std::string filename = "results_bench_oswec_reg_condition_" +
                               std::to_string(cli.condition) + ".json";
        seastack::bench::WriteBenchmarkJSON(
            out_dir + "/" + filename,
            case_id, started_at, finished_at, meta, settings, trials);

        return 0;
    }

    // ── Full batch mode (default) ───────────────────────────────────────────
    std::string started_at = seastack::bench::GetISOTimestamp();
    std::vector<seastack::bench::TrialResult> condition_results;
    condition_results.resize(static_cast<size_t>(num_conditions));

    seastack::bench::BenchmarkSettings settings;
    settings.timestep = kTimestep;
    settings.sim_duration = kSimDuration;
    settings.num_steps = num_steps;
    settings.num_bodies = 2;
    settings.radiation_method = "rirf_convolution";
    settings.excitation_method = "frequency_domain";
    settings.wave_type = "regular_batch_16";
    settings.num_trials = num_conditions;
    settings.warmup = false;

    // Serial batch mode
    for (int c = 0; c < num_conditions; ++c) {
        std::cout << "  Condition " << (c + 1) << "/" << num_conditions
                  << " (T=" << kPeriods[c] << "s)..." << std::flush;

        auto result = RunCondition(c, flap_mesh, base_mesh, h5);

        std::cout << " sim=" << std::fixed << std::setprecision(2)
                  << result.sim_wall_s << "s" << std::endl;
        condition_results[static_cast<size_t>(c)] = result;
    }

    std::string finished_at = seastack::bench::GetISOTimestamp();

    seastack::bench::PrintTrialSummary("oswec_reg_batch", settings, condition_results);

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);
    seastack::bench::WriteBenchmarkJSON(
        out_dir + "/" + RESULTS_FILE_NAME + ".json",
        "oswec_reg_batch", started_at, finished_at, meta, settings, condition_results);

    return 0;
}
