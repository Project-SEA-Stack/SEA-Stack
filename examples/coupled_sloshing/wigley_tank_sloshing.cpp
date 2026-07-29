// =============================================================================
// SEA-Stack — Coupled Wigley hull (potential-flow) + SPH deck-sloshing tank
//                                                                    [POC]
//
// Two disjoint fluid domains act on ONE hull rigid body, coupled through
// rigid-body dynamics:
//   - EXTERIOR ocean: linear potential flow (BEM/BEMIO) applied exactly as in
//     the existing wigley/spreading demo through SEA-Stack's HydroSystem
//     (per-DOF ChForce callbacks + ChLoadHydrodynamics infinite-frequency
//     added mass).
//   - INTERIOR tank: Chrono::SPH fluid. The tank walls are BCE_RIGID markers on
//     the hull body (thin box walls via AddRigidBody), so they move with the
//     ship and the sloshing reaction feeds back into the hull force
//     accumulator.
//
// Because BEM covers the wetted exterior and SPH covers only the interior tank,
// there is no fluid double-counting. Both force paths sum inside
// ChBody::UpdateForces when ChFsiSystemSPH::DoStepDynamics advances the shared
// multibody system.
//
// Frames / units (SI): hull reference frame origin at the waterline, midship;
//   x fore(-)/aft(+), y to port, z up; still-water free surface at z = 0; deck
//   at z = +FREEBOARD (3.5 m). Gravity acts in -z. The Chrono MBS body is a
//   ChBodyAuxRef, so FSI wall geometry expressed in the reference frame lands on
//   the deck at local z = 3.5.
//
// Flotation: a deck tank adds topside weight the BEM hydrostatics do not know
// about. To keep the hull floating at the linearization point (waterline at
// z=0) we reduce the hull STRUCTURAL mass by the tank fluid mass, so
//   hull_structure_mass + tank_fluid_mass == BEM equilibrium displacement.
// This is convention-agnostic: total weight carried by buoyancy is unchanged.
// (Hull inertia is left unchanged — the ~2% mass shift is a documented
// approximation for this demonstrator.)
//
// This is a demonstrator of a coupled method, not a validated sloshing solver.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <seastack/adapters/chrono/setup_from_yaml.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/adapters/chrono/simulation_export.h>

#include <chrono_parsers/yaml/ChParserMbsYAML.h>
#include <chrono/physics/ChSystem.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyAuxRef.h>
#include <chrono/utils/ChBodyGeometry.h>

#include <chrono_fsi/sph/ChFsiProblemSPH.h>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
using namespace ::chrono;
using namespace ::chrono::fsi;
using namespace ::chrono::fsi::sph;

using seastack::chrono::HydroSystem;
using seastack::chrono::SetupHydroFromYAMLFile;
using seastack::chrono::SimulationExporter;

// ---------------------------------------------------------------------------
// Tank / solver parameters (top-of-file so they are easy to tweak, per plan).
// ---------------------------------------------------------------------------
namespace params {
// Tank interior dimensions (m) and still-water fill depth (m).
double tank_Lx = 4.0;   // along ship x (fore-aft)
double tank_Ly = 4.0;   // along ship y (transverse)
double tank_H = 2.0;    // wall height (open top)
double fill_depth = 1.0;

// Deck placement (hull reference-frame local coordinates, m).
double deck_x = 0.0;    // midship
double deck_y = 0.0;    // centreline
double deck_z = 3.5;    // freeboard (deck) height above waterline

// SPH resolution and coupling.
double spacing = 0.10;  // SPH initial spacing (m)
double dt_cfd = 1e-4;   // SPH (CFD) sub-step (s)
double dt_mbd = 1e-3;   // multibody sub-step == FSI coupling step (s)

// Tank fluid.
double fluid_density = 1000.0;   // fresh ballast water (kg/m^3)
double fluid_viscosity = 1e-3;

// BEM equilibrium displacement of the wigley hull (kg); see model YAML mass.
double bem_equilibrium_mass = 956667.0;
}  // namespace params

// ---------------------------------------------------------------------------
// Minimal CLI configuration.
// ---------------------------------------------------------------------------
struct Config {
    std::string case_dir = "data/demos/run_seastack/wigley/spreading";
    std::string hydro_override;      // optional: use this hydro YAML instead of the case's
    std::string out_dir = "coupled_sloshing_out";
    double duration = 12.0;          // seconds
    double roll_kick = 0.0;          // initial hull roll rate (rad/s) — two-way disturbance
    bool rebalance_mass = true;      // reduce hull mass by tank fluid mass
    bool no_waves = false;           // zero out wave excitation (still-water two-way test)
};

// Thin subclass to expose the protected script directory (matches single_run.cpp).
namespace {
class HullParser : public parsers::ChParserMbsYAML {
  public:
    HullParser() : ChParserMbsYAML() {}
    void SetScriptDir(const std::string& dir) { m_script_directory = dir; }
};
}  // namespace

// Build the hull multibody system from the wigley model + simulation YAML.
// Mirrors the essential logic of single_run.cpp::InitializeChronoSystem.
static std::shared_ptr<ChSystem> BuildHullSystem(const std::string& model_file,
                                                 const std::string& sim_file) {
    HullParser parser;
    parser.SetScriptDir(fs::path(model_file).parent_path().generic_string());

    auto sim_yaml = YAML::LoadFile(sim_file);
    parser.LoadSimData(sim_yaml);
    auto sim_node = sim_yaml["simulation"];
    if (sim_node && sim_node["contact_method"]) {
        if (sim_node["time_step"] && sim_node["integrator"] && !sim_node["integrator"]["time_step"])
            sim_node["integrator"]["time_step"] = sim_node["time_step"];
        parser.LoadSolverData(sim_node);
    }

    auto system = parser.CreateSystem();

    auto model_yaml = YAML::LoadFile(model_file);
    parser.LoadModelData(model_yaml);
    parser.Populate(*system);
    return system;
}

static std::shared_ptr<ChBody> FindBody(ChSystem& sys, const std::string& name) {
    for (const auto& b : sys.GetBodies())
        if (b && b->GetName() == name)
            return b;
    return nullptr;
}

// Assemble the open-top tank as thin box walls (floor + 4 sides), expressed in
// the hull reference frame. Returns geometry ready for AddRigidBody.
static std::shared_ptr<utils::ChBodyGeometry> BuildTankGeometry(double cx, double cy, double floor_z,
                                                                double Lx, double Ly, double H,
                                                                double tw) {
    auto g = chrono_types::make_shared<utils::ChBodyGeometry>();
    g->materials.push_back(ChContactMaterialData());
    // Floor (just below the fluid).
    g->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(
        ChVector3d(cx, cy, floor_z - tw / 2), QUNIT, ChVector3d(Lx + 2 * tw, Ly + 2 * tw, tw)));
    // +X / -X walls.
    g->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(
        ChVector3d(cx + Lx / 2 + tw / 2, cy, floor_z + H / 2), QUNIT, ChVector3d(tw, Ly, H)));
    g->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(
        ChVector3d(cx - Lx / 2 - tw / 2, cy, floor_z + H / 2), QUNIT, ChVector3d(tw, Ly, H)));
    // +Y / -Y walls.
    g->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(
        ChVector3d(cx, cy + Ly / 2 + tw / 2, floor_z + H / 2), QUNIT, ChVector3d(Lx + 2 * tw, tw, H)));
    g->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(
        ChVector3d(cx, cy - Ly / 2 - tw / 2, floor_z + H / 2), QUNIT, ChVector3d(Lx + 2 * tw, tw, H)));
    return g;
}

static Config ParseArgs(int argc, char** argv) {
    Config c;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (a == "--case") c.case_dir = next();
        else if (a == "--hydro") c.hydro_override = next();
        else if (a == "--out") c.out_dir = next();
        else if (a == "--duration") c.duration = std::stod(next());
        else if (a == "--roll-kick") c.roll_kick = std::stod(next());
        else if (a == "--no-rebalance") c.rebalance_mass = false;
        else if (a == "--no-waves") c.no_waves = true;
        else if (a[0] != '-') c.case_dir = a;  // positional case dir
    }
    return c;
}

int main(int argc, char** argv) {
    Config cfg = ParseArgs(argc, argv);

    // Resolve the wigley case files (auto-detect the *.model.yaml stem).
    fs::path case_dir(cfg.case_dir);
    if (!fs::exists(case_dir)) {
        std::cerr << "Case directory not found: " << case_dir << "\n"
                  << "Usage: wigley_tank_sloshing [--case DIR] [--hydro FILE] [--out DIR]\n"
                  << "       [--duration S] [--roll-kick RAD/S] [--no-waves] [--no-rebalance]\n";
        return 1;
    }
    std::string model_file, sim_file, hydro_file;
    for (const auto& e : fs::directory_iterator(case_dir)) {
        const std::string p = e.path().generic_string();
        if (p.size() > 11 && p.substr(p.size() - 11) == ".model.yaml") model_file = p;
        else if (p.size() > 16 && p.substr(p.size() - 16) == ".simulation.yaml") sim_file = p;
        else if (p.size() > 11 && p.substr(p.size() - 11) == ".hydro.yaml") hydro_file = p;
    }
    if (!cfg.hydro_override.empty()) hydro_file = cfg.hydro_override;
    if (model_file.empty() || sim_file.empty() || hydro_file.empty()) {
        std::cerr << "Could not find model/simulation/hydro YAML in " << case_dir << "\n";
        return 1;
    }
    std::cout << "[coupled] model : " << model_file << "\n";
    std::cout << "[coupled] sim   : " << sim_file << "\n";
    std::cout << "[coupled] hydro : " << hydro_file << "\n";

    // -----------------------------------------------------------------------
    // 1. Build the hull multibody system.
    // -----------------------------------------------------------------------
    auto system = BuildHullSystem(model_file, sim_file);
    if (!system) {
        std::cerr << "Failed to build hull system.\n";
        return 1;
    }
    auto hull = FindBody(*system, "body1");
    if (!hull) {
        std::cerr << "Hull body 'body1' not found.\n";
        return 1;
    }
    system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // -----------------------------------------------------------------------
    // 2. Attach exterior potential-flow hydro (kept alive for the whole run).
    // -----------------------------------------------------------------------
    std::vector<std::shared_ptr<ChBody>> bodies(system->GetBodies().begin(), system->GetBodies().end());
    std::unique_ptr<HydroSystem> hydro;
    try {
        hydro = SetupHydroFromYAMLFile(hydro_file, bodies, params::dt_mbd, cfg.duration, 0.0);
    } catch (const std::exception& e) {
        std::cerr << "Hydro setup failed: " << e.what() << "\n";
        return 1;
    }
    if (cfg.no_waves && hydro && hydro->GetWave()) {
        // Still-water two-way test: keep hydrostatics + radiation, drop excitation
        // by suppressing the wave field amplitude.
        hydro->SetExcitationTruncationTime(1e-9);
    }

    // -----------------------------------------------------------------------
    // 3. Build the SPH deck tank on the hull.
    // -----------------------------------------------------------------------
    ChFsiProblemCartesian fsi(params::spacing, system.get());
    fsi.SetVerbose(true);
    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    fsi.SetStepSizeCFD(params::dt_cfd);
    fsi.SetStepsizeMBD(params::dt_mbd);

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = params::fluid_density;
    fluid_props.viscosity = params::fluid_viscosity;
    fsi.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = params::spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 8.0;
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.artificial_viscosity = 0.1;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;
    sph_params.num_proximity_search_steps = 1;
    fsi.SetSPHParameters(sph_params);

    // Tank walls (BCE_RIGID) on the hull, expressed in the reference frame.
    const double tw = sph_params.num_bce_layers * params::spacing;
    auto tank_geom = BuildTankGeometry(params::deck_x, params::deck_y, params::deck_z,
                                       params::tank_Lx, params::tank_Ly, params::tank_H, tw);
    fsi.AddRigidBody(hull, tank_geom, /*check_embedded=*/true, /*use_grid_bce=*/true);

    // Interior fluid: partial fill, no fixed walls. Reference position is the
    // centre of the bottom face, in world coordinates at t=0 (hull at identity).
    fsi.RegisterParticlePropertiesCallback(
        chrono_types::make_shared<DepthPressurePropertiesCallback>(params::fill_depth));
    fsi.Construct(ChVector3d(params::tank_Lx, params::tank_Ly, params::fill_depth),
                  ChVector3d(params::deck_x, params::deck_y, params::deck_z), BoxSide::NONE);

    // Generous computational domain around the deck so no particle is deleted.
    const double mx = 0.5 * std::max(params::tank_Lx, params::tank_Ly) + 2.0;
    fsi.SetComputationalDomain(
        ChAABB(ChVector3d(params::deck_x - mx, params::deck_y - mx, params::deck_z - 1.0),
               ChVector3d(params::deck_x + mx, params::deck_y + mx, params::deck_z + params::tank_H + 2.0)),
        BC_NONE);

    fsi.Initialize();

    const size_t n_sph = fsi.GetNumSPHParticles();
    const double particle_mass = params::fluid_density * std::pow(params::spacing, 3);
    const double tank_fluid_mass = n_sph * particle_mass;

    // -----------------------------------------------------------------------
    // 4. Flotation: rebalance hull structural mass by the tank fluid mass.
    // -----------------------------------------------------------------------
    if (cfg.rebalance_mass) {
        const double new_mass = std::max(1.0, params::bem_equilibrium_mass - tank_fluid_mass);
        const double scale = new_mass / hull->GetMass();
        // Scale inertia consistently with the (small) mass change.
        if (auto aux = std::dynamic_pointer_cast<ChBodyAuxRef>(hull)) {
            ChVector3d I = aux->GetInertiaXX();
            aux->SetMass(new_mass);
            aux->SetInertiaXX(I * scale);
        } else {
            hull->SetMass(new_mass);
        }
    }

    std::cout << "[coupled] SPH particles      : " << n_sph << "\n";
    std::cout << "[coupled] tank fluid mass    : " << tank_fluid_mass << " kg\n";
    std::cout << "[coupled] hull structural mass: " << hull->GetMass() << " kg\n";
    std::cout << "[coupled] BCE markers on hull: " << fsi.GetNumBCE(hull) << "\n";

    // Optional two-way disturbance: initial hull roll rate about x.
    if (cfg.roll_kick != 0.0) {
        hull->SetAngVelParent(ChVector3d(cfg.roll_kick, 0, 0));
    }

    // -----------------------------------------------------------------------
    // 5. HDF5 export of hull states (same layout as other cases).
    // -----------------------------------------------------------------------
    std::error_code ec;
    fs::create_directories(cfg.out_dir, ec);
    std::unique_ptr<SimulationExporter> exporter;
    try {
        SimulationExporter::Options opts;
        opts.output_path = (fs::path(cfg.out_dir) / "results.coupled_sloshing.h5").generic_string();
        opts.input_model_file = model_file;
        opts.input_simulation_file = sim_file;
        opts.input_hydro_file = hydro_file;
        opts.output_directory = cfg.out_dir;
        opts.scenario_type = cfg.no_waves ? "still" : "irregular";
        opts.log_final_output_path = true;
        exporter = std::make_unique<SimulationExporter>(opts);
        exporter->WriteSimulationInfo(system.get(), "10.0.0", "wigley_tank_sloshing",
                                      params::dt_mbd, cfg.duration);
        exporter->WriteModel(system.get());
        const int est_steps = static_cast<int>(cfg.duration / params::dt_mbd);
        exporter->BeginResults(system.get(), est_steps);
    } catch (const std::exception& e) {
        std::cerr << "[coupled] HDF5 export disabled: " << e.what() << "\n";
        exporter.reset();
    }

    // -----------------------------------------------------------------------
    // 6. Coupled time loop: fsi.DoStepDynamics advances the shared MBS + SPH.
    // -----------------------------------------------------------------------
    const auto ref0 = std::dynamic_pointer_cast<ChBodyAuxRef>(hull);
    auto ref_pos = [&]() { return ref0 ? ref0->GetFrameRefToAbs().GetPos() : hull->GetPos(); };

    double time = 0.0;
    int step = 0;
    double log_at = 0.0;
    bool diverged = false;
    std::cout << "\n[coupled] running " << cfg.duration << " s at coupling dt=" << params::dt_mbd
              << " s (CFD dt=" << params::dt_cfd << " s)\n" << std::endl;

    while (time < cfg.duration) {
        fsi.DoStepDynamics(params::dt_mbd);
        time += params::dt_mbd;
        ++step;

        if (exporter) exporter->RecordStep(system.get());

        const ChVector3d p = ref_pos();
        const ChVector3d rpy = hull->GetRot().GetCardanAnglesXYZ();
        const ChVector3d Ffsi = fsi.GetFsiBodyForce(hull);
        const ChVector3d Tfsi = fsi.GetFsiBodyTorque(hull);
        if (!std::isfinite(p.z()) || !std::isfinite(Ffsi.Length()) || std::abs(p.z()) > 50.0) {
            std::cerr << "[coupled] DIVERGED at t=" << time << " (heave=" << p.z() << ")\n";
            diverged = true;
            break;
        }
        if (time >= log_at) {
            std::cout << "t=" << time
                      << "  heave=" << p.z() << " m"
                      << "  roll=" << rpy.x() << " rad"
                      << "  |F_tank|=" << Ffsi.Length() << " N"
                      << "  T_tank_roll=" << Tfsi.x() << " N*m"
                      << "  n_sph=" << fsi.GetNumSPHParticles() << std::endl;
            log_at += 0.5;
        }
    }

    if (exporter) {
        exporter->SetRunMetadata("", "", 0.0, step, params::dt_mbd, time);
        exporter->Finalize();
    }

    std::cout << "\n[coupled] " << (diverged ? "FAILED (diverged)" : "COMPLETED")
              << "  steps=" << step << "  t_final=" << time << " s\n";
    std::cout << "[coupled] particle retention: " << fsi.GetNumSPHParticles() << " / " << n_sph << "\n";
    return diverged ? 1 : 0;
}
