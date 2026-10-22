/**
 * Benchmark: IEA sphere in irregular waves (JONSWAP, Hs = 4 m, Tp = 10 s,
 * 512 components, the YAML default) with nonlinear hydrostatics and nonlinear
 * Froude-Krylov (Airy + Wheeler), RIRF radiation and optimum-at-8 s PTO.
 *
 * Cost of nonlinear Froude-Krylov scales with vertices x wave components, so
 * this case is far more expensive per step than bench_sphere_reg_nlfk. The
 * duration is kept short. The wetted-surface pressure integration is reported
 * under hydrostatics_s in the component breakdown.
 */

#include <bench_utils.h>

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystemNSC.h>

#include <filesystem>
#include <string>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

static const int kNumTrials = 2;
static const double kTimestep = 0.01;        // [s]
static const double kSimDuration = 10.0;     // [s]
static const double kHs = 4.0;               // [m]
static const double kTp = 10.0;              // [s]
static const int kNumComponents = 512;       // kDefaultWaveSpectralNOmega
static const double kRampDuration = 2.0;     // [s]
static const double kPtoDamping = 633980.0;  // optimum at T = 8 s [N s/m]

int main(int argc, char* argv[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    auto mesh = (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();
    auto h5   = (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();

    auto meta = seastack::bench::CollectMetadata();
    auto cli = seastack::bench::ParseBenchmarkArgs(argc, argv);
    int num_trials = seastack::bench::ResolveTrialCount(kNumTrials, cli);
    bool warmup = seastack::bench::ResolveWarmup(false, cli);

    seastack::bench::BenchmarkSettings settings;
    settings.timestep = kTimestep;
    settings.sim_duration = kSimDuration;
    settings.num_steps = static_cast<int>(kSimDuration / kTimestep);
    settings.num_bodies = 1;
    settings.radiation_method = "rirf_convolution";
    settings.excitation_method = "frequency_domain";
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
        ground->SetPos(ChVector3d(0, 0, -10));
        ground->SetFixed(true);
        system.AddBody(ground);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetName("body1");
        body->SetPos(ChVector3d(0, 0, -2));
        body->SetMass(261.8e3);
        system.AddBody(body);

        auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        prismatic->Initialize(body, ground, false, ChFramed(ChVector3d(0, 0, -2)), ChFramed(ChVector3d(0, 0, -10)));
        system.AddLink(prismatic);

        auto pto = chrono_types::make_shared<ChLinkTSDA>();
        pto->Initialize(body, ground, false, ChVector3d(0, 0, -2), ChVector3d(0, 0, -10));
        pto->SetSpringCoefficient(0.0);
        pto->SetDampingCoefficient(kPtoDamping);
        system.AddLink(pto);

        SeaStateDefinition sea_state;
        sea_state.type = "irregular";
        SeaStatePartition part;
        part.spectrum.type = "jonswap";
        part.spectrum.Hs = kHs;
        part.spectrum.Tp = kTp;
        sea_state.partitions.push_back(part);
        sea_state.n_omega = kNumComponents;
        sea_state.seed = 1;
        sea_state.depth = 200.0;  // sphere.h5
        auto wave = std::make_shared<LinearDirectionalWaveField>(ComponentSampler::Build(sea_state), sea_state.depth);
        wave->SetRampDuration(kRampDuration);

        HydroModelBuilder::BodyHydrostaticsConfig cfg;
        cfg.model = HydrostaticsModel::kNonlinear;
        cfg.mesh_file = mesh;
        cfg.froude_krylov = FroudeKrylovModel::kNonlinear;

        std::vector<std::shared_ptr<ChBody>> bodies = {body};
        HydroSystem hydro(bodies, h5);
        hydro.SetBodyHydrostatics({cfg});
        hydro.AddWaves(wave);
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

    seastack::bench::PrintTrialSummary("sphere_irreg_nlfk", settings, trials);

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);
    seastack::bench::WriteBenchmarkJSON(
        out_dir + "/" + RESULTS_FILE_NAME + ".json",
        "sphere_irreg_nlfk", started_at, finished_at, meta, settings, trials);

    return 0;
}
