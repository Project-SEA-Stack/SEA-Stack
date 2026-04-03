// RM3 with rectified hydraulic PTO and PI speed controller in irregular waves.
// Demonstrates the hydraulic PTO assembly integrated with Chrono via
// the existing PTOForceFunctor adapter.  Writes a CSV file with full
// PTO diagnostics at every time step.

#include <gui/guihelper.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/adapters/chrono/pto_chrono_adapter.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/infra/logging.h>

#include <seastack/pto/hydraulic/rectified_hydraulic_pto.h>
#include <seastack/control/pi_controller.h>

#include <chrono/core/ChRealtimeStep.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

int main(int argc, char* argv[]) {
    std::cout << "=== RM3 HYDRAULIC PTO DEMO ===\n";
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    bool profilingOn = true, saveDataOn = true, plotOn = false, visualizationOn = true;
    std::string data_dir;
    if (!seastack::chrono::GetCLIArguments(argc, argv,
            "RM3 rectified hydraulic PTO demo",
            saveDataOn, profilingOn, plotOn, visualizationOn, data_dir))
        return 1;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto float_mesh = (DATADIR / "demos" / "rm3" / "geometry" / "float_cog.obj")
                          .lexically_normal().generic_string();
    auto plate_mesh = (DATADIR / "demos" / "rm3" / "geometry" / "plate_cog.obj")
                          .lexically_normal().generic_string();
    auto h5file = (DATADIR / "demos" / "rm3" / "hydroData" / "rm3.h5")
                      .lexically_normal().generic_string();

    // ── Chrono system ──────────────────────────────────────────────

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.01;
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    system.SetSolverType(ChSolver::Type::SPARSE_LU);
    double simulationDuration = 120.0;

    std::shared_ptr<seastack::viz::UI> pui = seastack::viz::CreateUI(visualizationOn);
    seastack::viz::UI& ui = *pui;

    // ── Bodies ─────────────────────────────────────────────────────

    auto float_body = chrono_types::make_shared<ChBodyEasyMesh>(
        float_mesh, 0, false, true, false);
    system.Add(float_body);
    float_body->SetName("body1");
    float_body->SetPos(ChVector3d(0, 0, -0.72));
    float_body->SetMass(725834);
    float_body->SetInertiaXX(ChVector3d(20907301.0, 21306090.66, 37085481.11));

    auto plate_body = chrono_types::make_shared<ChBodyEasyMesh>(
        plate_mesh, 0, false, true, false);
    system.Add(plate_body);
    plate_body->SetName("body2");
    plate_body->SetPos(ChVector3d(0, 0, -21.29));
    plate_body->SetMass(886691);
    plate_body->SetInertiaXX(ChVector3d(94419614.57, 94407091.24, 28542224.82));

    // Prismatic joint (heave)
    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(float_body, plate_body, false,
                          ChFramed(ChVector3d(0, 0, -0.72)),
                          ChFramed(ChVector3d(0, 0, -21.29)));
    system.AddLink(prismatic);

    // ── Hydraulic PTO ──────────────────────────────────────────────

    // Parameters tuned for ideal-rectification v1 model.
    // Larger accumulators and motor displacement compensate for the
    // simplified flow path (no chamber-pressure gating).
    seastack::pto::RectifiedHydraulicPTOParams pto_params{};
    pto_params.cylinder.piston_area = 0.0378;  // m^2

    pto_params.hp_accumulator.total_volume       = 0.05;     // m^3
    pto_params.hp_accumulator.precharge_pressure = 16.0e6;   // Pa
    pto_params.hp_accumulator.gamma              = 1.4;

    pto_params.lp_accumulator.total_volume       = 0.05;     // m^3
    pto_params.lp_accumulator.precharge_pressure = 8.0e6;    // Pa
    pto_params.lp_accumulator.gamma              = 1.4;

    pto_params.motor.displacement    = 6.0e-5;   // m^3/rad
    pto_params.motor.mech_efficiency = 0.85;
    pto_params.motor.vol_efficiency  = 0.90;

    pto_params.generator_inertia = 2.0;   // kg*m^2
    pto_params.generator_damping = 3.0;   // N*m*s/rad
    pto_params.num_substeps      = 20;

    // PI speed controller: target ~1000 RPM ≈ 104.72 rad/s
    seastack::control::PIControllerParams ctrl_params{};
    ctrl_params.kp         = 5.0;
    ctrl_params.ki         = 8.0;
    ctrl_params.setpoint   = 104.72;  // rad/s
    ctrl_params.output_min = 0.0;
    ctrl_params.output_max = 1000.0;  // N*m

    auto controller = std::make_shared<seastack::control::PIController>(ctrl_params);
    auto hydraulic_pto = std::make_shared<seastack::pto::RectifiedHydraulicPTO>(
        pto_params, controller);

    // Wire into Chrono via PTOForceFunctor + ChLinkTSDA
    auto tsda = chrono_types::make_shared<ChLinkTSDA>();
    tsda->Initialize(float_body, plate_body, false,
                     ChVector3d(0, 0, -0.72), ChVector3d(0, 0, -21.29));
    tsda->RegisterForceFunctor(
        std::make_shared<seastack::chrono::PTOForceFunctor>(hydraulic_pto));
    system.AddLink(tsda);

    // ── Waves ──────────────────────────────────────────────────────

    SeaStateDefinition sea_state;
    sea_state.type = "irregular";
    SeaStatePartition partition;
    partition.spectrum.type  = "jonswap";
    partition.spectrum.Hs    = 2.5;
    partition.spectrum.Tp    = 8.0;
    partition.spectrum.gamma = 3.3;
    sea_state.partitions.push_back(partition);
    sea_state.n_omega = 200;
    sea_state.seed    = 42;
    auto components = ComponentSampler::Build(sea_state);
    auto waves = std::make_shared<LinearDirectionalWaveField>(
        std::move(components), sea_state.depth);
    waves->SetRampDuration(40.0);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(float_body);
    bodies.push_back(plate_body);

    HydroSystem hydro_forces(bodies, h5file);
    hydro_forces.AddWaves(waves);

    // ── Data storage ───────────────────────────────────────────────

    struct Record {
        double time;
        double float_heave;
        double plate_heave;
        double rel_disp;
        double rel_vel;
        double ctrl_error;
        seastack::pto::PTODiagnostics diag;
    };
    std::vector<Record> records;
    records.reserve(static_cast<size_t>(simulationDuration / timestep) + 100);

    // ── Simulation loop ────────────────────────────────────────────

    auto wall_start = std::chrono::high_resolution_clock::now();

    ui.Init(&system, "RM3 - Hydraulic PTO");
    ui.SetCamera(0, -50, -10, 0, 0, -10);
    ui.SetWaveModel(waves);

    while (system.GetChTime() <= simulationDuration) {
        if (!ui.IsRunning(timestep)) break;
        if (ui.simulationStarted) {
            system.DoStepDynamics(timestep);

            Record rec{};
            rec.time = system.GetChTime();
            rec.float_heave = float_body->GetPos().z();
            rec.plate_heave = plate_body->GetPos().z();
            rec.rel_disp = tsda->GetLength() - tsda->GetRestLength();
            rec.rel_vel  = tsda->GetVelocity();
            rec.ctrl_error = controller->last_error();
            rec.diag = hydraulic_pto->GetDiagnostics();
            records.push_back(rec);
        }
    }

    auto wall_end = std::chrono::high_resolution_clock::now();
    auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                       wall_end - wall_start).count();

    // ── Console summary ────────────────────────────────────────────

    std::cout << "\n=== SIMULATION COMPLETE ===\n";
    std::cout << "Wall time: " << wall_ms << " ms ("
              << records.size() << " steps)\n";

    if (!records.empty()) {
        const auto& last = records.back().diag;
        std::cout << "\nFinal state:\n";
        std::cout << "  HP pressure:       " << last.hp_pressure / 1e6 << " MPa\n";
        std::cout << "  LP pressure:       " << last.lp_pressure / 1e6 << " MPa\n";
        std::cout << "  Motor speed:       " << last.motor_speed << " rad/s\n";
        std::cout << "  Generator torque:  " << last.generator_torque << " Nm\n";
        std::cout << "  Cylinder force:    " << last.cylinder_force / 1e3 << " kN\n";

        double total_mech = 0.0, total_elec = 0.0;
        for (size_t i = 1; i < records.size(); ++i) {
            double dt_rec = records[i].time - records[i - 1].time;
            total_mech += (-records[i].diag.cylinder_force * records[i].diag.piston_velocity) * dt_rec;
            total_elec += records[i].diag.electrical_power * dt_rec;
        }
        double dur = records.back().time - records.front().time;
        std::cout << "\nEnergy totals over " << dur << " s:\n";
        std::cout << "  Mech absorbed:     " << total_mech / 1e3 << " kJ\n";
        std::cout << "  Electrical:        " << total_elec / 1e3 << " kJ\n";
        if (dur > 0) {
            std::cout << "  Mean mech power:   " << (total_mech / dur) / 1e3 << " kW\n";
            std::cout << "  Mean elec power:   " << (total_elec / dur) / 1e3 << " kW\n";
        }
    }

    // ── CSV output ─────────────────────────────────────────────────

    if (saveDataOn && !records.empty()) {
        std::string out_dir = seastack::chrono::GetDemoOutDir();
        out_dir += "/" + std::string(RESULTS_DIR_NAME);
        std::filesystem::create_directories(std::filesystem::path(out_dir));

        std::string csv_path = out_dir + "/rm3_hydraulic_pto.csv";
        std::ofstream csv(csv_path);
        if (!csv.is_open()) {
            std::cerr << "Error: Could not open " << csv_path << "\n";
            return 1;
        }

        csv << "time,float_heave,plate_heave,rel_disp,rel_vel,"
               "F_pto,P_hp,P_lp,V_hp_oil,V_lp_oil,"
               "motor_speed,T_motor,T_gen,flow_rate,"
               "mech_power,elec_power,ctrl_error\n";

        csv << std::fixed;
        for (const auto& r : records) {
            csv << std::setprecision(6) << r.time << ","
                << std::setprecision(8) << r.float_heave << ","
                << r.plate_heave << ","
                << r.rel_disp << ","
                << r.rel_vel << ","
                << r.diag.cylinder_force << ","
                << r.diag.hp_pressure << ","
                << r.diag.lp_pressure << ","
                << r.diag.hp_oil_volume << ","
                << r.diag.lp_oil_volume << ","
                << r.diag.motor_speed << ","
                << r.diag.motor_torque << ","
                << r.diag.generator_torque << ","
                << r.diag.flow_rate << ","
                << r.diag.mechanical_power << ","
                << r.diag.electrical_power << ","
                << r.ctrl_error << "\n";
        }
        csv.close();
        std::cout << "\nCSV saved to " << csv_path << "\n";
    }

    return 0;
}
