/**
 * Benchmark: 5SA with directional cos2s spreading + MoorDyn mooring.
 * Exercises directional wave field (n_omega x n_theta), freq-domain excitation.
 * Requires SEASTACK_ENABLE_MOORING.
 *
 * Model assembly matches demos/5sa (see five_sa_model_setup.h).
 */

#include <bench_utils.h>

#include <seastack/adapters/chrono/helper.h>
#include <seastack/hydro/excitation_types.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#ifdef SEASTACK_HAVE_MOORDYN
#include <seastack/mooring/moordyn_config.h>
#endif

#include <chrono/physics/ChSystemNSC.h>

#include <filesystem>
#include <string>

#include "five_sa_model_setup.h"

using namespace chrono;
using namespace seastack::hydro;

static const int kNumTrials = 3;
static const double kTimestep = 0.02;
static const double kSimDuration = 200.0;

int main(int argc, char* argv[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto meta = seastack::bench::CollectMetadata();
    auto cli = seastack::bench::ParseBenchmarkArgs(argc, argv);
    int num_trials = seastack::bench::ResolveTrialCount(kNumTrials, cli);
    bool warmup = seastack::bench::ResolveWarmup(true, cli);

    seastack::bench::BenchmarkSettings settings;
    settings.timestep = kTimestep;
    settings.sim_duration = kSimDuration;
    settings.num_steps = static_cast<int>(kSimDuration / kTimestep);
    settings.num_bodies = 5;
    settings.radiation_method = "rirf_convolution";
    settings.excitation_method = "frequency_domain";
    settings.wave_type = "irregular_jonswap_cos2s";
    settings.moordyn = true;
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

        auto five_sa = SetupFiveSaModel(system, DATADIR, "5sa_directional.h5");

        SeaStateDefinition sea_state;
        sea_state.type = "irregular";
        sea_state.depth = 50.0;
        SeaStatePartition partition;
        partition.spectrum.type = "jonswap";
        partition.spectrum.Hs = 3.0;
        partition.spectrum.Tp = 10.0;
        partition.spectrum.gamma = 3.3;
        partition.spreading.type = "cos2s";
        partition.spreading.mean_direction_deg = 0.0;
        partition.spreading.s = 12.0;
        sea_state.partitions.push_back(partition);
        sea_state.n_omega = 64;
        sea_state.n_theta = 21;
        sea_state.seed = 42;

        auto components = ComponentSampler::Build(sea_state);
        auto waves = std::make_shared<LinearDirectionalWaveField>(
            std::move(components), sea_state.depth);
        waves->SetRampDuration(60.0);

        seastack::chrono::HydroSystem hydro(five_sa.bodies, five_sa.h5file);
        hydro.SetExcitationMethod(ExcitationMethod::kFrequencyDomain);

#ifdef SEASTACK_HAVE_MOORDYN
        {
            seastack::mooring::MoorDynConfig md_cfg;
            md_cfg.enabled = true;
            md_cfg.input_file = five_sa.moordyn_input;
            md_cfg.coupled_body_indices = {0, 2};
            hydro.SetMoorDynConfig(md_cfg);
        }
#endif
        ApplyFiveSaHydroLinearDamping(hydro);
        hydro.AddWaves(waves);
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

    seastack::bench::PrintTrialSummary("5sa_spreading", settings, trials);

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);
    seastack::bench::WriteBenchmarkJSON(
        out_dir + "/" + RESULTS_FILE_NAME + ".json",
        "5sa_spreading", started_at, finished_at, meta, settings, trials);

    return 0;
}
