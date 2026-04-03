/**
 * Benchmark: RM3 with MoorDyn mooring in irregular waves (eta import).
 * 2-body, RIRF convolution + IRF excitation via EtaTableWaveField.
 * Requires SEASTACK_ENABLE_MOORING.
 */

#include <bench_utils.h>

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/eta_table_wave_field.h>

#ifdef SEASTACK_HAVE_MOORDYN
#include <seastack/mooring/moordyn_config.h>
#endif

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/timestepper/ChTimestepperImplicit.h>

#include <filesystem>
#include <string>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

static const int kNumTrials = 3;
static const double kTimestep = 0.01;
static const double kSimDuration = 300.0;

int main(int argc, char* argv[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    auto float_mesh = (DATADIR / "demos" / "rm3" / "geometry" / "float_cog.obj").lexically_normal().generic_string();
    auto plate_mesh = (DATADIR / "demos" / "rm3" / "geometry" / "plate_cog.obj").lexically_normal().generic_string();
    auto h5 = (DATADIR / "demos" / "rm3" / "hydroData" / "rm3.h5").lexically_normal().generic_string();
    auto eta_file = (DATADIR / "verification" / "rm3_mooring" / "inputs" / "eta_rm3_mooring.txt").lexically_normal().generic_string();
    auto moordyn_file = (DATADIR / "demos" / "rm3" / "mooring" / "lines_rm3.txt").lexically_normal().generic_string();

    auto meta = seastack::bench::CollectMetadata();
    auto cli = seastack::bench::ParseBenchmarkArgs(argc, argv);
    int num_trials = seastack::bench::ResolveTrialCount(kNumTrials, cli);
    bool warmup = seastack::bench::ResolveWarmup(true, cli);

    seastack::bench::BenchmarkSettings settings;
    settings.timestep = kTimestep;
    settings.sim_duration = kSimDuration;
    settings.num_steps = static_cast<int>(kSimDuration / kTimestep);
    settings.num_bodies = 2;
    settings.radiation_method = "rirf_convolution";
    settings.excitation_method = "irf_convolution_eta_import";
    settings.wave_type = "irregular_eta_file";
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
        system.SetTimestepperType(ChTimestepper::Type::HHT);
        system.SetSolverType(ChSolver::Type::GMRES);
        if (auto integrator = std::dynamic_pointer_cast<ChTimestepperImplicit>(system.GetTimestepper())) {
            integrator->SetStepControl(false);
            integrator->SetMaxIters(50);
        }

        auto float_body = chrono_types::make_shared<ChBodyEasyMesh>(float_mesh, 0, false, true, false);
        system.Add(float_body);
        float_body->SetName("body1");
        float_body->SetPos(ChVector3d(0, 0, -0.72));
        float_body->SetMass(725834);
        float_body->SetInertiaXX(ChVector3d(20907301.0, 21306090.66, 37085481.11));

        auto plate_body = chrono_types::make_shared<ChBodyEasyMesh>(plate_mesh, 0, false, true, false);
        system.Add(plate_body);
        plate_body->SetName("body2");
        plate_body->SetPos(ChVector3d(0, 0, -21.5));
        plate_body->SetMass(886691);
        plate_body->SetInertiaXX(ChVector3d(94419614.57, 94407091.24, 28542224.82));

        auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        prismatic->Initialize(float_body, plate_body, false,
                              ChFramed(ChVector3d(0, 0, -0.72)),
                              ChFramed(ChVector3d(0, 0, -21.5)));
        system.AddLink(prismatic);

        auto pto = chrono_types::make_shared<ChLinkTSDA>();
        pto->Initialize(float_body, plate_body, false,
                        ChVector3d(0, 0, -0.72), ChVector3d(0, 0, -21.5));
        pto->SetDampingCoefficient(1200000.0);
        pto->SetSpringCoefficient(0.0);
        system.AddLink(pto);

        auto waves = std::make_shared<EtaTableWaveField>(eta_file, 70.0);
        waves->SetRampDuration(40.0);

        std::vector<std::shared_ptr<ChBody>> bodies = {float_body, plate_body};
        HydroSystem hydro(bodies, h5);
        hydro.SetExcitationTruncationTime(20.0);

#ifdef SEASTACK_HAVE_MOORDYN
        {
            seastack::mooring::MoorDynConfig md_cfg;
            md_cfg.enabled = true;
            md_cfg.input_file = moordyn_file;
            md_cfg.coupled_body_indices = {1};
            hydro.SetMoorDynConfig(md_cfg);
        }
#endif
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

    seastack::bench::PrintTrialSummary("rm3_mooring_irreg", settings, trials);

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);
    seastack::bench::WriteBenchmarkJSON(
        out_dir + "/" + RESULTS_FILE_NAME + ".json",
        "rm3_mooring_irreg", started_at, finished_at, meta, settings, trials);

    return 0;
}
