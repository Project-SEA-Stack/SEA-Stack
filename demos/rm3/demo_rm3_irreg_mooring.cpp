// RM3 in irregular waves with MoorDyn mooring.
// Demonstrates wave-body-mooring coupling for a 2-body point absorber.

#include <gui/guihelper.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/infra/logging.h>

#ifdef SEASTACK_HAVE_MOORDYN
#include <seastack/mooring/moordyn_config.h>
#endif

#include <chrono/core/ChRealtimeStep.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

int main(int argc, char* argv[]) {
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    bool profilingOn = true, saveDataOn = true, plotOn = false, visualizationOn = true;
    std::string data_dir;
    if (!seastack::chrono::GetCLIArguments(argc, argv, "RM3 irregular waves + mooring demo",
                                           saveDataOn, profilingOn, plotOn, visualizationOn, data_dir))
        return 1;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto float_mesh = (DATADIR / "demos" / "rm3" / "geometry" / "float_cog.obj").lexically_normal().generic_string();
    auto plate_mesh = (DATADIR / "demos" / "rm3" / "geometry" / "plate_cog.obj").lexically_normal().generic_string();
    auto h5file     = (DATADIR / "demos" / "rm3" / "hydroData" / "rm3.h5").lexically_normal().generic_string();

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.01;
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    system.SetSolverType(ChSolver::Type::SPARSE_LU);
    double simulationDuration = 300.0;

    std::shared_ptr<seastack::viz::UI> pui = seastack::viz::CreateUI(visualizationOn);
    seastack::viz::UI& ui = *pui;

    std::vector<double> time_vector;
    std::vector<double> float_heave;

    // Float body
    auto float_body = chrono_types::make_shared<ChBodyEasyMesh>(float_mesh, 0, false, true, false);
    system.Add(float_body);
    float_body->SetName("body1");
    float_body->SetPos(ChVector3d(0, 0, -0.72));
    float_body->SetMass(725834);
    float_body->SetInertiaXX(ChVector3d(20907301.0, 21306090.66, 37085481.11));

    // Plate body
    auto plate_body = chrono_types::make_shared<ChBodyEasyMesh>(plate_mesh, 0, false, true, false);
    system.Add(plate_body);
    plate_body->SetName("body2");
    plate_body->SetPos(ChVector3d(0, 0, -21.29));
    plate_body->SetMass(886691);
    plate_body->SetInertiaXX(ChVector3d(94419614.57, 94407091.24, 28542224.82));

    // Prismatic joint (vertical heave)
    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(float_body, plate_body, false,
                          ChFramed(ChVector3d(0, 0, -0.72)), ChFramed(ChVector3d(0, 0, -21.29)));
    system.AddLink(prismatic);

    // PTO damper
    auto pto = chrono_types::make_shared<ChLinkTSDA>();
    pto->Initialize(float_body, plate_body, false, ChVector3d(0, 0, -0.72), ChVector3d(0, 0, -21.29));
    pto->SetDampingCoefficient(1200000.0);
    system.AddLink(pto);

    // Irregular wave
    SeaStateDefinition sea_state;
    sea_state.type = "irregular";
    SeaStatePartition partition;
    partition.spectrum.type = "jonswap";
    partition.spectrum.Hs = 2.5;
    partition.spectrum.Tp = 8.0;
    partition.spectrum.gamma = 3.3;
    sea_state.partitions.push_back(partition);
    sea_state.n_omega = 200;
    sea_state.seed = 42;
    auto components = ComponentSampler::Build(sea_state);
    auto waves = std::make_shared<LinearDirectionalWaveField>(std::move(components), sea_state.depth);
    waves->SetRampDuration(40.0);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(float_body);
    bodies.push_back(plate_body);

    HydroSystem hydro_forces(bodies, h5file);

#ifdef SEASTACK_HAVE_MOORDYN
    {
        auto moordyn_input =
            (DATADIR / "demos" / "rm3" / "mooring" / "lines_rm3.txt").lexically_normal().generic_string();
        seastack::mooring::MoorDynConfig md_cfg;
        md_cfg.enabled = true;
        md_cfg.input_file = moordyn_input;
        md_cfg.coupled_body_indices = {1};
        hydro_forces.SetMoorDynConfig(md_cfg);
    }
#endif

    hydro_forces.AddWaves(waves);

    ui.Init(&system, "RM3 - Irregular Waves + Mooring");
    ui.SetCamera(0, -50, -10, 0, 0, -10);
    ui.SetWaveModel(waves);

#ifdef SEASTACK_HAVE_MOORDYN
    ui.SetMooringLineProvider([&hydro_forces]() {
        return hydro_forces.GetMooringLineStates();
    });
#endif

    while (system.GetChTime() <= simulationDuration) {
        if (!ui.IsRunning(timestep)) break;
        if (ui.simulationStarted) {
            system.DoStepDynamics(timestep);
            time_vector.push_back(system.GetChTime());
            float_heave.push_back(float_body->GetPos().z());
        }
    }

    std::string out_dir = seastack::chrono::GetDemoOutDir();
    if (saveDataOn) {
        out_dir = out_dir + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directory(std::filesystem::path(out_dir));
        const std::string output_path = out_dir + "/rm3_irreg_mooring.txt";
        std::ofstream outputFile(output_path);
        if (!outputFile.is_open()) {
            seastack::infra::cli::LogError("Could not open " + output_path);
        } else {
            outputFile << std::left << std::setw(10) << "Time (s)"
                       << std::right << std::setw(16) << "Float Heave (m)" << std::endl;
            for (size_t i = 0; i < time_vector.size(); ++i)
                outputFile << std::left << std::setw(12) << std::setprecision(6) << std::fixed << time_vector[i]
                           << std::right << std::setw(16) << std::setprecision(8) << std::fixed << float_heave[i]
                           << std::endl;
        }
    }

    return 0;
}
