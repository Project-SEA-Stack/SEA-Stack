// Trimaran: same mechanism topology as demo_trimaran_rigid.cpp, but each arm is an Euler beam
// instead of a rigid box. Revolute at center deck -> lumped root body -> FEA beam -> tip weld
// to outrigger. No PTO / RSDA (add later once this baseline is stable).

#include <gui/guihelper.h>
#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/infra/logging.h>

#include <chrono/assets/ChVisualShapeFEA.h>
#include <chrono/core/ChRealtimeStep.h>
#include <chrono/fea/ChBeamSectionEuler.h>
#include <chrono/fea/ChBuilderBeam.h>
#include <chrono/fea/ChMesh.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkMate.h>
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
#include <vector>

#include "trimaran_hulls.h"
#include "trimaran_sea_state.h"

using namespace chrono;
using namespace chrono::fea;
using namespace seastack::hydro;
using seastack::chrono::HydroCouplingOptions;
using seastack::chrono::HydroSystem;

namespace {

// Match rigid demo outer dt and hydro damping so differences isolate the arm model.
constexpr double kTimestep = 0.02;             // s
constexpr double kSimulationDuration = 200.0;  // s

constexpr std::array<double, 6> kCenterQuadraticDamping = {
    1.2e4, 1.8e4, 1.0e4, 2.0e5, 2.0e5, 6.0e4};
constexpr std::array<double, 6> kOutriggerQuadraticDamping = {
    6.0e3, 1.0e4, 6.0e3, 1.2e5, 1.2e5, 4.0e4};
constexpr std::array<double, 6> kCenterLinearDamping = {0.0, 0.0, 1.0e5, 1.0e6, 1.0e6, 0.0};
constexpr std::array<double, 6> kOutriggerLinearDamping = {0.0, 0.0, 5.0e4, 2.0e5, 2.0e5, 0.0};

// Same HHT + SPARSE_LU policy as demo_trimaran_rigid.cpp.
void ApplyFeArmSolverSettings(ChSystemSMC& system) {
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

// Beam: CHS geometry (OD, wall in m). Density kept at steel-like value for demo mass; E/G are a stiff
// elastomer (not steel ~210 GPa) so the arm is much stiffer than the old 2.1 MPa stand-in but the
// hydro+FEA implicit solve remains tractable at kTimestep.
constexpr double kBeamOuterDiameter = 1.0;
constexpr double kBeamWallThickness = 0.030;
// ~200 MPa: hard rubber / highly filled elastomer — order ~100x softer than steel.
constexpr double kYoungModulusPa = 2.0e8;
// G ~ E/3 (nearly incompressible rubber, nu ~ 0.5).
constexpr double kShearModulusPa = 2.0e8 / 3.0;
constexpr double kBeamDensityKgM3 = 7850.0;
constexpr int kNumBeamElements = 6;

// Lumped hub at deck (same role as rigid arm body COM, but at the root point).
constexpr double kRootBodyMassKg = 50000.0;
const ChVector3d kRootBodyInertia(5000.0, 5000.0, 5000.0);

struct TrimaranFeModel {
    std::vector<std::shared_ptr<ChBody>> hydro_bodies;
    std::string h5file;
    std::shared_ptr<ChBody> center;
    std::shared_ptr<ChBody> port;
    std::shared_ptr<ChBody> stbd;
};

TrimaranFeModel BuildFeArmsLikeRigid(ChSystemSMC& system, const std::filesystem::path& data_path) {
    using trimaran::AddTrimaranHulls;
    using trimaran::kCenterFreeboard;
    using trimaran::kCenterHalfBeam;
    using trimaran::kOutriggerFreeboard;
    using trimaran::kOutriggerHalfBeam;
    using trimaran::kOutriggerY;

    TrimaranFeModel model;

    auto hulls = AddTrimaranHulls(system, data_path);
    model.hydro_bodies = hulls.hydro_bodies;
    model.h5file = hulls.h5file;
    model.center = hulls.center;
    model.port = hulls.port;
    model.stbd = hulls.stbd;

    const double arm_z_center = kCenterFreeboard;
    const double arm_z_outrigger = kOutriggerFreeboard;
    const ChQuaternion<> rev_rot = QuatFromAngleY(CH_PI / 2.0);

    struct ArmGeom {
        ChVector3d pt_root;
        ChVector3d pt_tip;
        std::shared_ptr<ChBody> outrigger;
    };
    const ArmGeom arms[2] = {
        {ChVector3d(0, -kCenterHalfBeam, arm_z_center),
         ChVector3d(0, -(kOutriggerY - kOutriggerHalfBeam), arm_z_outrigger), hulls.stbd},
        {ChVector3d(0, +kCenterHalfBeam, arm_z_center),
         ChVector3d(0, +(kOutriggerY - kOutriggerHalfBeam), arm_z_outrigger), hulls.port},
    };

    const double OD = kBeamOuterDiameter;
    const double wall = kBeamWallThickness;
    const double ID = OD - 2.0 * wall;
    const double r_o = OD / 2.0;
    const double r_i = ID / 2.0;
    const double A = CH_PI * (r_o * r_o - r_i * r_i);
    const double I = (CH_PI / 4.0) * (std::pow(r_o, 4) - std::pow(r_i, 4));
    const double J = 2.0 * I;

    auto beam_section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();
    beam_section->SetYoungModulus(kYoungModulusPa);
    beam_section->SetShearModulus(kShearModulusPa);
    beam_section->SetDensity(kBeamDensityKgM3);
    beam_section->SetArea(A);
    beam_section->SetIyy(I);
    beam_section->SetIzz(I);
    beam_section->SetJ(J);
    beam_section->SetDrawCircularRadius(OD / 2.0);

    auto fea_mesh = chrono_types::make_shared<ChMesh>();
    ChBuilderBeamEuler builder;

    for (int i = 0; i < 2; ++i) {
        const ChVector3d& pt_root = arms[i].pt_root;
        const ChVector3d& pt_tip = arms[i].pt_tip;
        const auto& outr = arms[i].outrigger;

        const ChVector3d arm_vec = pt_tip - pt_root;
        ChMatrix33<> mrot;
        mrot.SetFromAxisX(arm_vec, ChVector3d(0, 0, 1));
        const ChQuaternion<> qa = mrot.GetQuaternion();

        builder.BuildBeam(fea_mesh, beam_section, kNumBeamElements, pt_root, pt_tip,
                          ChVector3d(0, 0, 1));

        auto& nodes = builder.GetLastBeamNodes();
        auto root_node = nodes.front();
        auto tip_node = nodes.back();

        auto root_body = chrono_types::make_shared<ChBody>();
        root_body->SetPos(pt_root);
        root_body->SetMass(kRootBodyMassKg);
        root_body->SetInertiaXX(kRootBodyInertia);
        root_body->EnableCollision(false);
        system.AddBody(root_body);

        auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
        revolute->Initialize(hulls.center, root_body, ChFramed(pt_root, rev_rot));
        system.AddLink(revolute);

        auto root_clamp = chrono_types::make_shared<ChLinkMateGeneric>();
        root_clamp->Initialize(root_body, root_node, false, root_node->Frame(), root_node->Frame());
        root_clamp->SetConstrainedCoords(true, true, true, true, true, true);
        system.Add(root_clamp);

        // Same absolute frames as rigid demo tip weld: arm direction qa at pt_tip.
        auto tip_weld = chrono_types::make_shared<ChLinkMateGeneric>();
        tip_weld->Initialize(tip_node, outr, false, ChFramed(pt_tip, qa), ChFramed(pt_tip, qa));
        tip_weld->SetConstrainedCoords(true, true, true, true, true, true);
        system.Add(tip_weld);
    }

    fea_mesh->SetAutomaticGravity(false);

    auto vis_beam = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_beam->SetColormapRange(-500, 500);
    vis_beam->SetSmoothFaces(true);
    vis_beam->SetWireframe(false);
    fea_mesh->AddVisualShapeFEA(vis_beam);

    auto vis_nodes = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_nodes->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    vis_nodes->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_nodes->SetSymbolsThickness(0.2);
    vis_nodes->SetSymbolsScale(0.5);
    fea_mesh->AddVisualShapeFEA(vis_nodes);

    system.Add(fea_mesh);

    return model;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    bool profiling_on = true;
    bool save_data_on = true;
    bool plot_on = false;
    bool visualization_on = true;
    std::string data_dir;
    if (!seastack::chrono::GetCLIArguments(argc, argv, "Trimaran FEA arms (rigid topology)", save_data_on,
                                           profiling_on, plot_on, visualization_on, data_dir)) {
        return 1;
    }
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) {
        return 1;
    }

    const std::filesystem::path data_path(seastack::chrono::GetDataDir());

    ChSystemSMC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));

    ApplyFeArmSolverSettings(system);

    auto model = BuildFeArmsLikeRigid(system, data_path);

    auto sea_state = trimaran::MakeTrimaranDemoIrregularSea();
    auto components = ComponentSampler::Build(sea_state);
    auto waves = std::make_shared<LinearDirectionalWaveField>(std::move(components), sea_state.depth);
    waves->SetRampDuration(60.0);

    const HydroCouplingOptions default_coupling;
    auto hydro_forces = std::make_unique<HydroSystem>(model.hydro_bodies, model.h5file, waves,
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

    ui.Init(&system, "Trimaran - FEA cross-arms (rigid topology)");
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
            for (const auto& b : model.hydro_bodies) {
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
            center_heave.push_back(model.center->GetPos().z());
            port_heave.push_back(model.port->GetPos().z());
            stbd_heave.push_back(model.stbd->GetPos().z());
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
        const std::string output_path = out_dir + "/trimaran_fea.txt";
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
