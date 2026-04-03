/**
 * Internal method comparison: LinearPTO vs RectifiedHydraulicPTO on RM3
 * in irregular waves.
 *
 * Runs the same RM3 two-body system under JONSWAP Hs=2.5 m, Tp=8 s with:
 *   A) LinearPTO (c = 1,200,000 N*s/m, k = 0)
 *   B) RectifiedHydraulicPTO with PI speed controller
 *
 * Both runs use the same wave realisation, bodies, and integrator settings.
 * Results are written to text/CSV files for the Python analysis script.
 *
 * Pass/fail: the executable passes if both simulations complete and files
 * are written.  Metric interpretation is left to the Python script and
 * human review.
 */

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/adapters/chrono/pto_chrono_adapter.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/pto/linear_pto.h>
#include <seastack/pto/hydraulic/rectified_hydraulic_pto.h>
#include <seastack/control/pi_controller.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/timestepper/ChTimestepperImplicit.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

// ── Common signal record ───────────────────────────────────────────

struct CommonRecord {
    double time;
    double rel_disp;
    double rel_vel;
    double pto_force;
    double pto_power;  // F * v (positive = absorbed)
};

// ── Hydraulic-only record ──────────────────────────────────────────

struct HydraulicRecord {
    double hp_pressure;
    double lp_pressure;
    double motor_speed;
    double gen_torque;
    double elec_power;
};

// ── Build shared wave field ────────────────────────────────────────

static std::shared_ptr<LinearDirectionalWaveField> build_waves() {
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
    return waves;
}

// ── Run A: LinearPTO ───────────────────────────────────────────────

struct RunResultLinear {
    std::vector<CommonRecord> records;
};

static RunResultLinear run_linear_pto(
    const std::string& float_mesh, const std::string& plate_mesh,
    const std::string& h5file, double duration) {

    std::cout << "  Running LinearPTO...\n";

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.01;
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    system.SetSolverType(ChSolver::Type::SPARSE_LU);

    if (auto integ = std::dynamic_pointer_cast<ChTimestepperImplicit>(system.GetTimestepper())) {
        integ->SetStepControl(false);
        integ->SetMaxIters(50);
    }

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

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(float_body, plate_body, false,
                          ChFramed(ChVector3d(0, 0, -0.72)),
                          ChFramed(ChVector3d(0, 0, -21.29)));
    system.AddLink(prismatic);

    auto pto_model = std::make_shared<seastack::pto::LinearPTO>(0.0, 1200000.0);
    auto tsda = chrono_types::make_shared<ChLinkTSDA>();
    tsda->Initialize(float_body, plate_body, false,
                     ChVector3d(0, 0, -0.72), ChVector3d(0, 0, -21.29));
    tsda->RegisterForceFunctor(
        std::make_shared<seastack::chrono::PTOForceFunctor>(pto_model));
    system.AddLink(tsda);

    auto waves = build_waves();
    std::vector<std::shared_ptr<ChBody>> bodies = {float_body, plate_body};
    HydroSystem hydro(bodies, h5file);
    hydro.AddWaves(waves);

    RunResultLinear result;
    auto t0 = std::chrono::high_resolution_clock::now();

    while (system.GetChTime() <= duration) {
        system.DoStepDynamics(timestep);

        CommonRecord rec{};
        rec.time      = system.GetChTime();
        rec.rel_disp  = tsda->GetLength() - tsda->GetRestLength();
        rec.rel_vel   = tsda->GetVelocity();
        rec.pto_force = tsda->GetForce();
        rec.pto_power = -rec.pto_force * rec.rel_vel;
        result.records.push_back(rec);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << "    Linear completed in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
              << " ms (" << result.records.size() << " steps)\n";

    return result;
}

// ── Run B: RectifiedHydraulicPTO ───────────────────────────────────

struct RunResultHydraulic {
    std::vector<CommonRecord> common;
    std::vector<HydraulicRecord> hydraulic;
};

static RunResultHydraulic run_hydraulic_pto(
    const std::string& float_mesh, const std::string& plate_mesh,
    const std::string& h5file, double duration) {

    std::cout << "  Running RectifiedHydraulicPTO...\n";

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.01;
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    system.SetSolverType(ChSolver::Type::SPARSE_LU);

    if (auto integ = std::dynamic_pointer_cast<ChTimestepperImplicit>(system.GetTimestepper())) {
        integ->SetStepControl(false);
        integ->SetMaxIters(50);
    }

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

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(float_body, plate_body, false,
                          ChFramed(ChVector3d(0, 0, -0.72)),
                          ChFramed(ChVector3d(0, 0, -21.29)));
    system.AddLink(prismatic);

    // Hydraulic PTO parameters — tuned for ideal-rectification v1 model.
    // The ideal model sends ALL cylinder flow to HP (no chamber-pressure
    // gating), so accumulators and motor must be sized larger than PTO-Sim
    // defaults to maintain a reasonable pressure differential.
    seastack::pto::RectifiedHydraulicPTOParams pto_params{};
    pto_params.cylinder.piston_area = 0.0378;

    pto_params.hp_accumulator.total_volume       = 0.05;
    pto_params.hp_accumulator.precharge_pressure = 16.0e6;
    pto_params.hp_accumulator.gamma              = 1.4;

    pto_params.lp_accumulator.total_volume       = 0.05;
    pto_params.lp_accumulator.precharge_pressure = 8.0e6;
    pto_params.lp_accumulator.gamma              = 1.4;

    pto_params.motor.displacement    = 6.0e-5;   // 3x larger to handle higher flow
    pto_params.motor.mech_efficiency = 0.85;
    pto_params.motor.vol_efficiency  = 0.90;

    pto_params.generator_inertia = 2.0;
    pto_params.generator_damping = 3.0;
    pto_params.num_substeps      = 20;

    seastack::control::PIControllerParams ctrl_params{};
    ctrl_params.kp         = 5.0;
    ctrl_params.ki         = 8.0;
    ctrl_params.setpoint   = 104.72;
    ctrl_params.output_min = 0.0;
    ctrl_params.output_max = 1000.0;

    auto controller = std::make_shared<seastack::control::PIController>(ctrl_params);
    auto pto_model = std::make_shared<seastack::pto::RectifiedHydraulicPTO>(
        pto_params, controller);

    auto tsda = chrono_types::make_shared<ChLinkTSDA>();
    tsda->Initialize(float_body, plate_body, false,
                     ChVector3d(0, 0, -0.72), ChVector3d(0, 0, -21.29));
    tsda->RegisterForceFunctor(
        std::make_shared<seastack::chrono::PTOForceFunctor>(pto_model));
    system.AddLink(tsda);

    auto waves = build_waves();
    std::vector<std::shared_ptr<ChBody>> bodies = {float_body, plate_body};
    HydroSystem hydro(bodies, h5file);
    hydro.AddWaves(waves);

    RunResultHydraulic result;
    auto t0 = std::chrono::high_resolution_clock::now();

    while (system.GetChTime() <= duration) {
        system.DoStepDynamics(timestep);

        CommonRecord cr{};
        cr.time      = system.GetChTime();
        cr.rel_disp  = tsda->GetLength() - tsda->GetRestLength();
        cr.rel_vel   = tsda->GetVelocity();
        cr.pto_force = tsda->GetForce();
        cr.pto_power = -cr.pto_force * cr.rel_vel;
        result.common.push_back(cr);

        auto diag = pto_model->GetDiagnostics();
        HydraulicRecord hr{};
        hr.hp_pressure = diag.hp_pressure;
        hr.lp_pressure = diag.lp_pressure;
        hr.motor_speed = diag.motor_speed;
        hr.gen_torque  = diag.generator_torque;
        hr.elec_power  = diag.electrical_power;
        result.hydraulic.push_back(hr);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << "    Hydraulic completed in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
              << " ms (" << result.common.size() << " steps)\n";

    return result;
}

// ── File I/O ───────────────────────────────────────────────────────

static void write_common_csv(const std::vector<CommonRecord>& records,
                             const std::string& path) {
    std::ofstream f(path);
    f << "time,rel_disp,rel_vel,pto_force,pto_power\n";
    f << std::fixed;
    for (const auto& r : records) {
        f << std::setprecision(6) << r.time << ","
          << std::setprecision(8) << r.rel_disp << ","
          << r.rel_vel << ","
          << r.pto_force << ","
          << r.pto_power << "\n";
    }
}

static void write_hydraulic_csv(const std::vector<CommonRecord>& common,
                                const std::vector<HydraulicRecord>& hydraulic,
                                const std::string& path) {
    std::ofstream f(path);
    f << "time,rel_disp,rel_vel,pto_force,pto_power,"
         "hp_pressure,lp_pressure,motor_speed,gen_torque,elec_power\n";
    f << std::fixed;
    size_t n = std::min(common.size(), hydraulic.size());
    for (size_t i = 0; i < n; ++i) {
        const auto& c = common[i];
        const auto& h = hydraulic[i];
        f << std::setprecision(6) << c.time << ","
          << std::setprecision(8) << c.rel_disp << ","
          << c.rel_vel << ","
          << c.pto_force << ","
          << c.pto_power << ","
          << h.hp_pressure << ","
          << h.lp_pressure << ","
          << h.motor_speed << ","
          << h.gen_torque << ","
          << h.elec_power << "\n";
    }
}

// ── Main ───────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::cout << "=== RM3 COMPARISON: LinearPTO vs RectifiedHydraulicPTO ===\n\n";

    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto float_mesh = (DATADIR / "demos" / "rm3" / "geometry" / "float_cog.obj")
                          .lexically_normal().generic_string();
    auto plate_mesh = (DATADIR / "demos" / "rm3" / "geometry" / "plate_cog.obj")
                          .lexically_normal().generic_string();
    auto h5file = (DATADIR / "demos" / "rm3" / "hydroData" / "rm3.h5")
                      .lexically_normal().generic_string();

    for (const auto& [label, path] : std::vector<std::pair<std::string, std::string>>{
             {"float mesh", float_mesh}, {"plate mesh", plate_mesh}, {"h5 data", h5file}}) {
        if (!std::filesystem::exists(path)) {
            std::cerr << "ERROR: " << label << " not found: " << path << "\n";
            return 1;
        }
    }

    double duration = seastack::chrono::GetSimDuration(60.0, 120.0);

    // Run A
    auto linear_result = run_linear_pto(float_mesh, plate_mesh, h5file, duration);

    // Run B
    auto hydraulic_result = run_hydraulic_pto(float_mesh, plate_mesh, h5file, duration);

    // Write output files
    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);

    write_common_csv(linear_result.records, out_dir + "/linear.csv");
    write_hydraulic_csv(hydraulic_result.common, hydraulic_result.hydraulic,
                        out_dir + "/hydraulic.csv");

    std::cout << "\nResults written to " << out_dir << "\n";
    std::cout << "  linear.csv\n";
    std::cout << "  hydraulic.csv\n";

    return 0;
}
