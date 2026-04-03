// Trimaran: three hulls + rigid cross-arms (revolute at center deck, welded tip to outrigger).
// Adds linear + quadratic hydrodynamic damping vs hydro-only demo.

#include <gui/guihelper.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/infra/logging.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/timestepper/ChAssemblyAnalysis.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include <chrono/timestepper/ChTimestepperImplicit.h>

#include <array>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include "trimaran_hulls.h"
#include "trimaran_sea_state.h"

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroCouplingOptions;
using seastack::chrono::HydroSystem;

namespace {

constexpr double kTimestep = 0.02;             // s
constexpr double kSimulationDuration = 200.0;  // s

constexpr std::array<double, 6> kCenterQuadraticDamping = {
    1.2e4, 1.8e4, 1.0e4, 2.0e5, 2.0e5, 6.0e4};
constexpr std::array<double, 6> kOutriggerQuadraticDamping = {
    6.0e3, 1.0e4, 6.0e3, 1.2e5, 1.2e5, 4.0e4};
constexpr std::array<double, 6> kCenterLinearDamping = {0.0, 0.0, 1.0e5, 1.0e6, 1.0e6, 0.0};
constexpr std::array<double, 6> kOutriggerLinearDamping = {0.0, 0.0, 5.0e4, 2.0e5, 2.0e5, 0.0};

void ApplyRigidArmSolverSettings(ChSystemSMC& system) {
    // Mechanism + hydro: LU with HHT step-size control off avoids some stiff KKT / sub-step issues.
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    system.SetSolverType(ChSolver::Type::SPARSE_LU);

    auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    if (!hht) {
        return;
    }
    hht->SetAlpha(-0.2);
    hht->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION);
    hht->SetRelTolerance(1e-6);
    hht->SetMaxIters(180);
    hht->SetAbsTolerances(1e-6, 1e-4);
    hht->SetStepControl(false);
}

// Revolute axis: world +X (longitudinal); Chrono default revolute is Z, rotated by pi/2 about Y.
void AddRigidCrossArms(ChSystemSMC& system,
                       const std::shared_ptr<ChBody>& center_body,
                       const std::shared_ptr<ChBody>& port_body,
                       const std::shared_ptr<ChBody>& stbd_body) {
    using trimaran::kCenterFreeboard;
    using trimaran::kCenterHalfBeam;
    using trimaran::kOutriggerFreeboard;
    using trimaran::kOutriggerHalfBeam;
    using trimaran::kOutriggerY;

    const double arm_z_center = kCenterFreeboard;
    const double arm_z_outrigger = kOutriggerFreeboard;
    const ChQuaternion<> rev_rot = QuatFromAngleY(CH_PI / 2.0);

    constexpr double arm_cross = 0.35;   // m
    constexpr double arm_density = 600.0;  // kg/m^3 (visual / inertia for arm box only)

    struct ArmGeom {
        ChVector3d pt_root;
        ChVector3d pt_tip;
        std::shared_ptr<ChBody> outrigger;
    };
    // -Y side -> starboard hull; +Y -> port (+X forward, z up).
    const ArmGeom arms[2] = {
        {ChVector3d(0, -kCenterHalfBeam, arm_z_center),
         ChVector3d(0, -(kOutriggerY - kOutriggerHalfBeam), arm_z_outrigger), stbd_body},
        {ChVector3d(0, +kCenterHalfBeam, arm_z_center),
         ChVector3d(0, +(kOutriggerY - kOutriggerHalfBeam), arm_z_outrigger), port_body},
    };

    for (int i = 0; i < 2; ++i) {
        const ChVector3d& pt_root = arms[i].pt_root;
        const ChVector3d& pt_tip = arms[i].pt_tip;
        const auto& outr = arms[i].outrigger;

        const ChVector3d arm_vec = pt_tip - pt_root;
        const double L = arm_vec.Length();
        ChMatrix33<> mrot;
        mrot.SetFromAxisX(arm_vec, ChVector3d(0, 0, 1));
        const ChQuaternion<> qa = mrot.GetQuaternion();
        const ChVector3d mid = 0.5 * (pt_root + pt_tip);

        auto arm = chrono_types::make_shared<ChBodyEasyBox>(L, arm_cross, arm_cross, arm_density,
                                                            true, false);
        arm->SetPos(mid);
        arm->SetRot(qa);
        arm->EnableCollision(false);
        system.Add(arm);

        auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
        revolute->Initialize(center_body, arm, ChFramed(pt_root, rev_rot));
        system.AddLink(revolute);

        auto tip_weld = chrono_types::make_shared<ChLinkMateGeneric>();
        tip_weld->Initialize(arm, outr, false, ChFramed(pt_tip, qa), ChFramed(pt_tip, qa));
        tip_weld->SetConstrainedCoords(true, true, true, true, true, true);
        system.AddLink(tip_weld);
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    bool profiling_on = true;
    bool save_data_on = true;
    bool plot_on = false;
    bool visualization_on = true;
    std::string data_dir;
    if (!seastack::chrono::GetCLIArguments(argc, argv, "Trimaran rigid-arm demo", save_data_on,
                                           profiling_on, plot_on, visualization_on, data_dir)) {
        return 1;
    }
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) {
        return 1;
    }

    const std::filesystem::path data_path(seastack::chrono::GetDataDir());

    ChSystemSMC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));

    ApplyRigidArmSolverSettings(system);

    auto hulls = trimaran::AddTrimaranHulls(system, data_path);
    AddRigidCrossArms(system, hulls.center, hulls.port, hulls.stbd);

    auto sea_state = trimaran::MakeTrimaranDemoIrregularSea();
    auto components = ComponentSampler::Build(sea_state);
    auto waves = std::make_shared<LinearDirectionalWaveField>(std::move(components), sea_state.depth);
    waves->SetRampDuration(60.0);

    const HydroCouplingOptions default_coupling;
    auto hydro_forces = std::make_unique<HydroSystem>(hulls.hydro_bodies, hulls.h5file, waves,
                                                      default_coupling);
    hydro_forces->EnableRirfSmoothing();
    hydro_forces->SetQuadraticDamping(std::vector<std::array<double, 6>>{
        kCenterQuadraticDamping, kOutriggerQuadraticDamping, kOutriggerQuadraticDamping});
    hydro_forces->SetLinearDamping(std::vector<std::array<double, 6>>{
        kCenterLinearDamping, kOutriggerLinearDamping, kOutriggerLinearDamping});

    std::string out_dir = seastack::chrono::GetDemoOutDir();
    if (profiling_on || save_data_on) {
        out_dir = out_dir + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directory(std::filesystem::path(out_dir));
    }

    auto pui = seastack::viz::CreateUI(visualization_on);
    seastack::viz::UI& ui = *pui;

    auto wall_start = std::chrono::high_resolution_clock::now();

    ui.Init(&system, "Trimaran - rigid cross-arms");
    ui.SetCamera(0, -60, 20, 0, 0, 0);
    ui.SetWaveModel(waves);

    const AssemblyAnalysis::ExitFlag assembly_exit =
        system.DoAssembly(AssemblyAnalysis::POSITION | AssemblyAnalysis::VELOCITY);
    if (assembly_exit == AssemblyAnalysis::ExitFlag::NOT_CONVERGED) {
        seastack::infra::cli::LogError(
            "DoAssembly(POSITION|VELOCITY) did not converge; initial multibody state is inconsistent.");
        return 4;
    }

    std::vector<double> time_vec;
    std::vector<double> center_heave;
    std::vector<double> port_heave;
    std::vector<double> stbd_heave;

    int step_num = 0;

    while (system.GetChTime() <= kSimulationDuration) {
        if (!ui.IsRunning(kTimestep)) {
            break;
        }

        if (ui.simulationStarted) {
            try {
                system.DoStepDynamics(kTimestep);
            } catch (const std::exception& e) {
                seastack::infra::cli::LogError(std::string("DoStepDynamics exception at t=") +
                                               std::to_string(system.GetChTime()) + " s (step " +
                                               std::to_string(step_num) + "): " + e.what());
                return 3;
            }
            ++step_num;

            const double t = system.GetChTime();

            if (hydro_forces->HasDiverged()) {
                seastack::infra::cli::LogError(std::string("HydroSystem divergence at t=") +
                                               std::to_string(t) + " s (step " +
                                               std::to_string(step_num) + ")");
                return 2;
            }

            const auto finite = [](const ChVector3d& v) {
                return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
            };
            for (const auto& b : hulls.hydro_bodies) {
                const auto& p = b->GetPos();
                const auto& vel = b->GetLinVel();
                const auto w = b->GetAngVelLocal();
                if (!finite(p) || !finite(vel) || !finite(w)) {
                    seastack::infra::cli::LogError(
                        "Non-finite hull state at t=" + std::to_string(t) + " s (step " +
                        std::to_string(step_num) + ")");
                    return 2;
                }
            }

            time_vec.push_back(t);
            center_heave.push_back(hulls.center->GetPos().z());
            port_heave.push_back(hulls.port->GetPos().z());
            stbd_heave.push_back(hulls.stbd->GetPos().z());
        }
    }

    auto wall_end = std::chrono::high_resolution_clock::now();
    const unsigned wall_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start).count();

    if (profiling_on) {
        std::cout << "\n--- Profiling ---\n";
        std::cout << "  Wall time: " << wall_ms / 1000.0 << " s\n";
        std::cout << "  Sim time:  " << kSimulationDuration << " s\n";
    }

    if (save_data_on && !time_vec.empty()) {
        const std::string output_path = out_dir + "/trimaran_rigid.txt";
        std::ofstream out(output_path);
        if (!out.is_open()) {
            seastack::infra::cli::LogError("Could not open " + output_path);
        } else {
            out << std::left << std::setw(12) << "Time(s)" << std::setw(16) << "CenterZ(m)"
                << std::setw(16) << "PortZ(m)" << std::setw(16) << "StbdZ(m)" << "\n";
            for (size_t i = 0; i < time_vec.size(); ++i) {
                out << std::left << std::fixed << std::setprecision(6) << std::setw(12)
                    << time_vec[i] << std::setw(16) << center_heave[i] << std::setw(16)
                    << port_heave[i] << std::setw(16) << stbd_heave[i] << "\n";
            }
            std::cout << "Results saved to: " << output_path << "\n";
        }
    }

    return 0;
}
