/**
 * Benchmark: F3OF flap decay (Decay C3) with 3-body RIRF convolution.
 * Largest body count; measures cross-coupling convolution cost.
 */

#include <bench_utils.h>

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemSMC.h>

#include <filesystem>
#include <string>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

static const int kNumTrials = 3;
static const double kTimestep = 0.02;
static const double kSimDuration = 240.0;

int main(int argc, char* argv[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    auto base_mesh = (DATADIR / "demos" / "f3of" / "geometry" / "base.obj").lexically_normal().generic_string();
    auto flap_mesh = (DATADIR / "demos" / "f3of" / "geometry" / "flap.obj").lexically_normal().generic_string();
    auto h5 = (DATADIR / "demos" / "f3of" / "hydroData" / "f3of.h5").lexically_normal().generic_string();

    auto meta = seastack::bench::CollectMetadata();
    auto cli = seastack::bench::ParseBenchmarkArgs(argc, argv);
    int num_trials = seastack::bench::ResolveTrialCount(kNumTrials, cli);
    bool warmup = seastack::bench::ResolveWarmup(true, cli);

    seastack::bench::BenchmarkSettings settings;
    settings.timestep = kTimestep;
    settings.sim_duration = kSimDuration;
    settings.num_steps = static_cast<int>(kSimDuration / kTimestep);
    settings.num_bodies = 3;
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

        ChSystemSMC system;
        system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
        system.SetSolverType(ChSolver::Type::SPARSE_QR);

        auto base = chrono_types::make_shared<ChBodyEasyMesh>(base_mesh, 0, false, true, false);
        system.Add(base);
        base->SetName("body1");
        base->SetMass(1089825.0);
        base->SetInertiaXX(ChVector3d(100000000.0, 76300000.0, 100000000.0));

        auto flapFore = chrono_types::make_shared<ChBodyEasyMesh>(flap_mesh, 0, false, true, false);
        system.Add(flapFore);
        flapFore->SetName("body2");
        flapFore->SetMass(179250.0);
        flapFore->SetInertiaXX(ChVector3d(100000000.0, 1300000.0, 100000000.0));

        auto flapAft = chrono_types::make_shared<ChBodyEasyMesh>(flap_mesh, 0, false, true, false);
        system.Add(flapAft);
        flapAft->SetName("body3");
        flapAft->SetMass(179250.0);
        flapAft->SetInertiaXX(ChVector3d(100000000.0, 1300000.0, 100000000.0));

        // Decay C3: base fixed, fore flap 10 deg initial rotation
        base->SetPos(ChVector3d(0.0, 0.0, -9.0));
        double fore_ang = CH_PI / 18.0;
        flapFore->SetRot(QuatFromAngleY(fore_ang));
        flapFore->SetPos(ChVector3d(-12.5 + 3.5 * std::cos(CH_PI / 2.0 - fore_ang), 0.0,
                                    -9.0 + 3.5 * std::sin(CH_PI / 2.0 - fore_ang)));
        flapAft->SetRot(QuatFromAngleY(0.0));
        flapAft->SetPos(ChVector3d(12.5 + 3.5 * std::cos(CH_PI / 2.0), 0.0,
                                   -9.0 + 3.5 * std::sin(CH_PI / 2.0 - fore_ang)));

        ChQuaternion<> revoluteRot = QuatFromAngleX(CH_PI / 2.0);
        auto revoluteFore = chrono_types::make_shared<ChLinkLockRevolute>();
        revoluteFore->Initialize(base, flapFore,
                                 ChFramed(ChVector3d(-12.5, 0.0, -9.0), revoluteRot));
        system.AddLink(revoluteFore);

        auto revoluteAft = chrono_types::make_shared<ChLinkLockRevolute>();
        revoluteAft->Initialize(base, flapAft,
                                ChFramed(ChVector3d(12.5, 0.0, -9.0), revoluteRot));
        system.AddLink(revoluteAft);

        auto ground = chrono_types::make_shared<ChBody>();
        system.AddBody(ground);
        ground->SetPos(ChVector3d(0, 0, -12.0));
        ground->SetTag(-1);
        ground->SetFixed(true);
        ground->EnableCollision(false);

        auto anchor = chrono_types::make_shared<ChLinkMateGeneric>();
        anchor->Initialize(base, ground, false,
                           base->GetVisualModelFrame(),
                           base->GetVisualModelFrame());
        system.Add(anchor);
        anchor->SetConstrainedCoords(true, true, true, true, true, true);

        auto no_waves = std::make_shared<NoWave>();
        std::vector<std::shared_ptr<ChBody>> bodies = {base, flapFore, flapAft};
        HydroSystem hydro(bodies, h5, no_waves);
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

    seastack::bench::PrintTrialSummary("f3of_decay_c3", settings, trials);

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);
    seastack::bench::WriteBenchmarkJSON(
        out_dir + "/" + RESULTS_FILE_NAME + ".json",
        "f3of_decay_c3", started_at, finished_at, meta, settings, trials);

    return 0;
}
