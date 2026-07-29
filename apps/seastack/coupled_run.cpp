/*********************************************************************
 * @file  coupled_run.cpp
 * @brief Coupled seakeeping (BEM) + SPH tank-sloshing runner implementation.
 *
 * Builds the hull multibody system from the model+simulation YAML (as the
 * potential-flow path does), attaches exterior linear hydrodynamics through
 * SEA-Stack's HydroSystem, mounts a deck tank whose walls are BCE_RIGID markers
 * on the hull and whose interior fluid is Chrono::SPH, then advances the shared
 * system with ChFsiSystemSPH::DoStepDynamics so both force paths sum inside the
 * hull's force accumulator (two-way coupling).
 *
 * Frames / units (SI): hull reference frame origin at the still-water line,
 * midship; x fore, y port, z up; deck at local z = tank.deck_z. Gravity -z.
 *
 * Flotation: a deck tank adds topside weight the BEM hydrostatics do not model.
 * By default the hull STRUCTURAL mass is reduced by the tank fluid mass so the
 * ship floats at the linearization point (total weight carried by buoyancy is
 * unchanged). Hull inertia is left unchanged (documented small approximation).
 *********************************************************************/

#include "coupled_run.h"

#include <seastack/config.h>
#include <seastack/infra/logging.h>
#include <seastack/adapters/chrono/setup_from_yaml.h>
#include <seastack/adapters/chrono/hydro_system.h>

#include "guihelper.h"

#include "chrono_parsers/yaml/ChParserMbsYAML.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/utils/ChBodyGeometry.h"
#include "chrono/ChVersion.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#if defined(SEASTACK_HAVE_VSG)
#include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualMaterial.h"
#endif

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace seastack::app {

namespace {

// Best-effort scalar probe of a YAML file (used for end_time / time_step).
bool TryFindYamlDouble(const std::string& yaml_path, const std::string& key, double& out_value) {
    std::ifstream in(yaml_path);
    if (!in.is_open()) return false;
    auto trim = [](std::string& s) {
        s.erase(0, s.find_first_not_of(" \t\r\n"));
        size_t p = s.find_last_not_of(" \t\r\n");
        if (p == std::string::npos) s.clear(); else s.erase(p + 1);
    };
    std::string line;
    while (std::getline(in, line)) {
        trim(line);
        if (line.empty() || line[0] == '#') continue;
        size_t pos = line.find(':');
        if (pos == std::string::npos) continue;
        std::string k = line.substr(0, pos), v = line.substr(pos + 1);
        trim(k); trim(v);
        if (k == key) {
            try { out_value = std::stod(v); return true; } catch (...) { return false; }
        }
    }
    return false;
}

std::string ReadEntireFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return {};
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Thin subclass exposing the protected script directory (matches single_run.cpp).
class HullParser : public ::chrono::parsers::ChParserMbsYAML {
  public:
    HullParser() : ChParserMbsYAML() {}
    void SetScriptDir(const std::string& dir) { m_script_directory = dir; }
};

// Build the hull multibody system from model + simulation YAML.
std::shared_ptr<::chrono::ChSystem> BuildHullSystem(const std::string& model_file,
                                                    const std::string& sim_file) {
    HullParser parser;
    const std::filesystem::path model_dir = std::filesystem::path(model_file).parent_path();
    parser.SetScriptDir(model_dir.generic_string());

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

    // Chrono MBS YAML does not reliably apply visualization.color to model_file
    // meshes (and the Wigley OBJ has no MTL). Mirror single_run: tint triangle
    // meshes from the model YAML so industrial yellow is present before the
    // translucent glass tank is attached (which would otherwise skip paint).
    try {
        auto model_node = model_yaml["model"];
        if (model_node && model_node["bodies"]) {
            std::unordered_map<std::string, YAML::Node> vis_by_name;
            for (const auto& entry : model_node["bodies"]) {
                if (!entry["name"] || !entry["visualization"]) continue;
                vis_by_name.emplace(entry["name"].as<std::string>(), entry["visualization"]);
            }
            auto resolve_mesh_path = [&](const std::string& path_str) -> std::string {
                std::filesystem::path p(path_str);
                if (p.is_relative()) p = model_dir / p;
                std::error_code ec;
                auto c = std::filesystem::weakly_canonical(p, ec);
                return (!ec ? c : p.lexically_normal()).generic_string();
            };
            for (const auto& body : system->GetBodies()) {
                if (!body) continue;
                auto vit = vis_by_name.find(body->GetName());
                if (vit == vis_by_name.end() || !vit->second) continue;
                const YAML::Node& vis_node = vit->second;
                if (!vis_node["color"] || !vis_node["color"].IsSequence() ||
                    vis_node["color"].size() < 3) {
                    continue;
                }
                const float r = vis_node["color"][0].as<float>();
                const float g = vis_node["color"][1].as<float>();
                const float b = vis_node["color"][2].as<float>();
                auto vis = body->GetVisualModel();
                if (!vis) continue;
                for (unsigned si = 0; si < vis->GetNumShapes(); ++si) {
                    auto shape = body->GetVisualShape(si);
                    auto existing_tri =
                        std::dynamic_pointer_cast<::chrono::ChVisualShapeTriangleMesh>(shape);
                    if (existing_tri && existing_tri->GetMesh()) {
                        auto tri_shape = ::chrono_types::make_shared<::chrono::ChVisualShapeTriangleMesh>(
                            existing_tri->GetMesh(), false);
                        tri_shape->SetColor(::chrono::ChColor(r, g, b));
                        body->GetVisualModel()->Clear();
                        body->AddVisualShape(tri_shape);
                        break;
                    }
                    auto mf = std::dynamic_pointer_cast<::chrono::ChVisualShapeModelFile>(shape);
                    if (!mf) continue;
                    auto trimesh = ::chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(
                        resolve_mesh_path(mf->GetFilename()), true, false);
                    if (!trimesh) continue;
                    auto tri_shape = ::chrono_types::make_shared<::chrono::ChVisualShapeTriangleMesh>(
                        trimesh, false);
                    tri_shape->SetColor(::chrono::ChColor(r, g, b));
                    body->GetVisualModel()->Clear();
                    body->AddVisualShape(tri_shape);
                    break;
                }
            }
        }
    } catch (...) {
        // Non-fatal: GUI paint pass can still yellow the hull.
    }

    return system;
}

std::shared_ptr<::chrono::ChBody> FindBody(::chrono::ChSystem& sys, const std::string& name) {
    for (const auto& b : sys.GetBodies())
        if (b && b->GetName() == name) return b;
    return nullptr;
}

// Open-top tank as thin box walls (floor + 4 sides), expressed in the hull
// reference frame; ready for ChFsiProblem::AddRigidBody.
std::shared_ptr<::chrono::utils::ChBodyGeometry> BuildTankGeometry(double cx, double cy, double floor_z,
                                                                   double Lx, double Ly, double H,
                                                                   double tw) {
    using ::chrono::ChVector3d;
    using ::chrono::QUNIT;
    using ::chrono::ChContactMaterialData;
    namespace u = ::chrono::utils;
    auto g = ::chrono_types::make_shared<u::ChBodyGeometry>();
    g->materials.push_back(ChContactMaterialData());
    g->coll_boxes.push_back(u::ChBodyGeometry::BoxShape(
        ChVector3d(cx, cy, floor_z - tw / 2), QUNIT, ChVector3d(Lx + 2 * tw, Ly + 2 * tw, tw)));
    g->coll_boxes.push_back(u::ChBodyGeometry::BoxShape(
        ChVector3d(cx + Lx / 2 + tw / 2, cy, floor_z + H / 2), QUNIT, ChVector3d(tw, Ly, H)));
    g->coll_boxes.push_back(u::ChBodyGeometry::BoxShape(
        ChVector3d(cx - Lx / 2 - tw / 2, cy, floor_z + H / 2), QUNIT, ChVector3d(tw, Ly, H)));
    g->coll_boxes.push_back(u::ChBodyGeometry::BoxShape(
        ChVector3d(cx, cy + Ly / 2 + tw / 2, floor_z + H / 2), QUNIT, ChVector3d(Lx + 2 * tw, tw, H)));
    g->coll_boxes.push_back(u::ChBodyGeometry::BoxShape(
        ChVector3d(cx, cy - Ly / 2 - tw / 2, floor_z + H / 2), QUNIT, ChVector3d(Lx + 2 * tw, tw, H)));
    return g;
}

}  // namespace

CoupledRunResult RunCoupledCase(const CoupledRunConfig& config) {
    // '::chrono' (global) — an unqualified 'using namespace chrono' inside
    // seastack::app would resolve to seastack::chrono (the adapter namespace).
    using namespace ::chrono;
    using namespace ::chrono::fsi;
    using namespace ::chrono::fsi::sph;
    using seastack::chrono::HydroSystem;
    using seastack::chrono::SetupHydroFromYAMLFile;
    using seastack::chrono::SimulationExporter;

    CoupledRunResult result;
    const auto& tank = config.tank;

    try {
        const bool want_render = !config.nogui;
#if !defined(SEASTACK_HAVE_VSG)
        if (want_render) {
            seastack::infra::cli::LogWarning(
                "[coupled] GUI requested but SEA-Stack was built without VSG; running headless.");
        }
#endif
        if (!want_render) {
            seastack::infra::debug::LogDebug("[coupled] headless coupled seakeeping + SPH tank run");
        }

        // -----------------------------------------------------------------
        // 1. Hull multibody system.
        // -----------------------------------------------------------------
        auto system = BuildHullSystem(config.model_file, config.simulation_file);
        if (!system) {
            result.error_message = "Failed to build hull system from model/simulation YAML.";
            seastack::infra::cli::LogError(result.error_message);
            return result;
        }
        auto hull = FindBody(*system, tank.hull_body);
        if (!hull) {
            result.error_message = "Hull body '" + tank.hull_body + "' not found in the model.";
            seastack::infra::cli::LogError(result.error_message);
            return result;
        }

        const ChVector3d gravity = system->GetGravitationalAcceleration();

        // Time bounds from the simulation YAML.
        double end_time = 0.0;
        TryFindYamlDouble(config.simulation_file, "end_time", end_time);
        if (end_time <= 0.0) {
            result.error_message =
                "Coupled (headless) run requires a positive simulation.end_time in the "
                "simulation YAML.";
            seastack::infra::cli::LogError(result.error_message);
            return result;
        }
        const double dt = (tank.mbd_step > 0.0) ? tank.mbd_step : 1e-3;

        // -----------------------------------------------------------------
        // 2. Exterior potential-flow hydro (kept alive for the whole run).
        // -----------------------------------------------------------------
        std::vector<std::shared_ptr<ChBody>> bodies(system->GetBodies().begin(),
                                                    system->GetBodies().end());
        std::unique_ptr<HydroSystem> hydro;
        try {
            hydro = SetupHydroFromYAMLFile(config.hydro_file, bodies, dt, end_time, 0.0);
        } catch (const std::exception& e) {
            result.error_message = std::string("Exterior hydro setup failed: ") + e.what();
            seastack::infra::cli::LogError(result.error_message);
            return result;
        }

        // -----------------------------------------------------------------
        // 3. SPH deck tank on the hull.
        // -----------------------------------------------------------------
        ChFsiProblemCartesian fsi(tank.spacing, system.get());
        fsi.SetVerbose(config.debug_mode);
        fsi.SetGravitationalAcceleration(gravity);
        fsi.SetStepSizeCFD(tank.cfd_step > 0.0 ? tank.cfd_step : 1e-4);
        fsi.SetStepsizeMBD(dt);

        ChFsiFluidSystemSPH::FluidProperties fluid_props;
        fluid_props.density = tank.fluid_density;
        fluid_props.viscosity = tank.fluid_viscosity;
        fsi.SetCfdSPH(fluid_props);

        ChFsiFluidSystemSPH::SPHParameters sph_params;
        sph_params.integration_scheme = IntegrationScheme::RK2;
        // Wall BCE layers: >= 3 is needed for full kernel support at the tank
        // walls, otherwise fluid particles stick to / leak through boundaries.
        sph_params.num_bce_layers = std::max(1, tank.num_bce_layers);
        sph_params.initial_spacing = tank.spacing;
        sph_params.d0_multiplier = 1.2;  // slightly larger support radius for wall containment
        sph_params.max_velocity = tank.max_velocity;
        sph_params.shifting_method = ShiftingMethod::XSPH;
        sph_params.shifting_xsph_eps = 0.5;
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
        sph_params.boundary_method = BoundaryMethod::ADAMI;
        // Artificial viscosity trades mobility for robustness: lower = more
        // realistic splashy sloshing, higher = more damped/stable. Tunable per
        // case from the sph_file.
        sph_params.artificial_viscosity = tank.artificial_viscosity;
        sph_params.eos_type = EosType::TAIT;
        sph_params.use_delta_sph = true;
        sph_params.delta_sph_coefficient = 0.1;
        sph_params.num_proximity_search_steps = 1;
        fsi.SetSPHParameters(sph_params);

        const double tw = sph_params.num_bce_layers * tank.spacing;
        auto tank_geom = BuildTankGeometry(tank.deck_x, tank.deck_y, tank.deck_z,
                                           tank.length, tank.width, tank.height, tw);
        fsi.AddRigidBody(hull, tank_geom, /*check_embedded=*/true, /*use_grid_bce=*/true);

        fsi.RegisterParticlePropertiesCallback(
            chrono_types::make_shared<DepthPressurePropertiesCallback>(tank.fill_depth));

        // Fit the fluid grid strictly INSIDE the tank walls. Chrono centres the
        // fluid box on 'pos' but its real span is round(size/spacing) intervals;
        // for a size that is not a whole multiple of the spacing this overshoots
        // asymmetrically (e.g. 1.5 m at spacing 0.08 -> 1.52 m, poking into the
        // +x/+y walls). The jammed edge particles then get shoved back in and set
        // the whole body sloshing at t=0. Snap each horizontal extent down to a
        // whole number of spacings that leaves >= ~1 spacing clearance from the
        // walls, keeping the grid symmetric and fully contained so the fluid can
        // start at rest.
        auto fit_inside = [&](double interior) {
            int n = static_cast<int>(std::floor((interior - tank.spacing) / tank.spacing));
            n = std::max(1, n);
            return n * tank.spacing;
        };
        const double fluid_lx = fit_inside(tank.length);
        const double fluid_ly = fit_inside(tank.width);
        fsi.Construct(ChVector3d(fluid_lx, fluid_ly, tank.fill_depth),
                      ChVector3d(tank.deck_x, tank.deck_y, tank.deck_z), BoxSide::NONE);

        const double mx = 0.5 * std::max(tank.length, tank.width) + 2.0;
        fsi.SetComputationalDomain(
            ChAABB(ChVector3d(tank.deck_x - mx, tank.deck_y - mx, tank.deck_z - 1.0),
                   ChVector3d(tank.deck_x + mx, tank.deck_y + mx, tank.deck_z + tank.height + 2.0)),
            BC_NONE);

        fsi.Initialize();

        const size_t n_sph = fsi.GetNumSPHParticles();
        const double particle_mass = tank.fluid_density * std::pow(tank.spacing, 3);
        const double tank_fluid_mass = n_sph * particle_mass;

        // -----------------------------------------------------------------
        // 4. Flotation: rebalance hull structural mass by the tank fluid mass.
        // -----------------------------------------------------------------
        if (tank.rebalance_mass) {
            const double bem_mass = hull->GetMass();  // model-YAML equilibrium displacement
            const double new_mass = std::max(1.0, bem_mass - tank_fluid_mass);
            const double scale = new_mass / bem_mass;
            hull->SetMass(new_mass);
            if (auto aux = std::dynamic_pointer_cast<ChBodyAuxRef>(hull)) {
                aux->SetInertiaXX(aux->GetInertiaXX() * scale);
            }
        }

        seastack::infra::cli::LogInfo(
            "[coupled] hull=" + tank.hull_body +
            "  SPH particles=" + std::to_string(n_sph) +
            "  tank fluid=" + std::to_string(static_cast<long long>(tank_fluid_mass)) + " kg" +
            "  hull mass=" + std::to_string(static_cast<long long>(hull->GetMass())) + " kg" +
            "  BCE=" + std::to_string(fsi.GetNumBCE(hull)));
        seastack::infra::cli::LogInfo(
            "[coupled] duration=" + std::to_string(end_time) + " s  coupling dt=" +
            std::to_string(dt) + " s  CFD dt=" + std::to_string(tank.cfd_step) + " s");

        // Excitation. Normally the hull is driven by the exterior wave field
        // (see the hydro YAML). For a still-water roll-decay test, `roll_rate`
        // releases the hull from an initial roll angular velocity about ship-x;
        // it then oscillates and decays while the tank fluid sloshes. (A true
        // angle-release would tilt the tank relative to the world-aligned fluid
        // grid and misalign them, so we release from a velocity instead.)
        if (std::abs(tank.roll_rate) > 0.0) {
            hull->SetAngVelParent(ChVector3d(tank.roll_rate, 0, 0));
            seastack::infra::cli::LogInfo(
                "[coupled] initial roll rate " +
                seastack::infra::FormatNumber(tank.roll_rate, 3) +
                " rad/s (still-water decay excitation)");
        }

        // -----------------------------------------------------------------
        // 5. HDF5 exporter (hull states; same layout as other cases).
        // -----------------------------------------------------------------
        std::unique_ptr<SimulationExporter> exporter;
        const bool persist = !config.output_directory.empty();
        if (persist) {
            std::error_code ec;
            std::filesystem::create_directories(config.output_directory, ec);
            try {
                std::filesystem::path output_h5 =
                    std::filesystem::path(config.output_directory) / "results.coupled.h5";
                SimulationExporter::Options opts;
                opts.output_path = output_h5.generic_string();
                opts.input_model_file = config.model_file;
                opts.input_simulation_file = config.simulation_file;
                opts.input_hydro_file = config.hydro_file;
                opts.output_directory = config.output_directory;
                opts.scenario_type = "coupled";
                opts.export_config = config.export_config;
                if (config.cli_output_level == "compact")
                    opts.export_config.level = seastack::chrono::ExportLevel::kCompact;
                else if (config.cli_output_level == "detailed")
                    opts.export_config.level = seastack::chrono::ExportLevel::kDetailed;
                else if (config.cli_output_level == "standard")
                    opts.export_config.level = seastack::chrono::ExportLevel::kStandard;
                opts.log_final_output_path = false;
                {
                    std::string model_text = ReadEntireFile(config.model_file);
                    if (!model_text.empty()) opts.model_yaml = std::move(model_text);
                }
                exporter = std::make_unique<SimulationExporter>(opts);
                exporter->WriteSimulationInfo(system.get(), std::string(CHRONO_VERSION),
                                              "wigley_tank_sloshing", dt, end_time);
                exporter->WriteModel(system.get());
                exporter->BeginResults(system.get(), static_cast<int>(end_time / dt));
                result.primary_artifact_path = output_h5.generic_string();
            } catch (const std::exception& e) {
                seastack::infra::cli::LogWarning(std::string("[coupled] HDF5 exporter disabled: ") + e.what());
                exporter.reset();
                result.primary_artifact_path.clear();
            }
        } else {
            result.artifact_note =
                "No persistent HDF5 (add output_directory to the setup YAML to save results).";
        }

        // -----------------------------------------------------------------
        // 6. Optional SEA-Stack GUI (skybox / free surface / ImGui) + SPH plugin.
        // -----------------------------------------------------------------
        std::shared_ptr<seastack::viz::UI> ui;
        bool gui_ok = false;
#if defined(SEASTACK_HAVE_VSG)
        if (want_render) {
            try {
                auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.0);
                auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(fsi.GetFsiSystemSPH().get());
                visFSI->EnableFluidMarkers(true);
                visFSI->EnableBoundaryMarkers(false);
                // Hide the opaque BCE wall markers; instead draw a translucent
                // tank box (below) so the sloshing fluid stays clearly visible.
                visFSI->EnableRigidBodyMarkers(false);
                visFSI->SetSPHColorCallback(col_callback);

                // Translucent tank so the fluid is visibly contained without the
                // BCE markers obscuring it. Attached to the hull in its reference
                // frame (same frame as the tank walls), so it rolls with the ship.
                {
                    auto tank_box = chrono_types::make_shared<ChVisualShapeBox>(
                        tank.length, tank.width, tank.height);
                    auto glass = chrono_types::make_shared<ChVisualMaterial>();
                    glass->SetDiffuseColor(ChColor(0.55f, 0.8f, 1.0f));
                    glass->SetOpacity(0.42f);  // visible container without hiding the fluid
                    tank_box->AddMaterial(glass);
                    hull->AddVisualShape(
                        tank_box,
                        ChFrame<>(ChVector3d(tank.deck_x, tank.deck_y,
                                             tank.deck_z + 0.5 * tank.height),
                                  QUNIT));
                }

                ui = seastack::viz::CreateUI(true);
                ui->AttachVisualPlugin(visFSI);
                ui->Init(system.get(), "SEA-Stack — Wigley + SPH deck tank");
                // Frame the whole (small) hull plus the deck tank from an
                // aft-port, slightly elevated vantage. Standoff is sized for the
                // ~10 m demo hull so both ship and tank are visible together.
                ui->SetCamera(tank.deck_x + 9.0, tank.deck_y - 15.0, tank.deck_z + 7.0,
                              tank.deck_x, tank.deck_y, tank.deck_z);
                // Free-surface grid: prefer simulation YAML water_grid_width /
                // water_grid_length when present; otherwise a compact default
                // sized for the ~10 m hull (not the old 50 m 120x80 patch).
                double grid_w = 30.0, grid_l = 30.0;
                TryFindYamlDouble(config.simulation_file, "water_grid_width", grid_w);
                TryFindYamlDouble(config.simulation_file, "water_grid_length", grid_l);
                ui->SetWaterGridExtent(grid_w, grid_l, 0.0, 0.0);
                if (hydro && hydro->GetWave()) {
                    ui->SetWaveModel(hydro->GetWave());
                }
                gui_ok = true;
                seastack::infra::cli::LogInfo(
                    "[coupled] SEA-Stack GUI ready — press Start in the viewer to run");
            } catch (const std::exception& e) {
                seastack::infra::cli::LogWarning(
                    std::string("[coupled] GUI init failed, falling back to headless: ") + e.what());
                ui.reset();
            }
        }
#else
        (void)want_render;
#endif
        if (!gui_ok) {
            ui = seastack::viz::CreateUI(false);
            ui->Init(system.get(), "SEA-Stack coupled (headless)");
        }

        // -----------------------------------------------------------------
        // 7. Coupled time loop.
        // -----------------------------------------------------------------
        const auto ref_body = std::dynamic_pointer_cast<ChBodyAuxRef>(hull);
        const auto wall_start = std::chrono::steady_clock::now();
        double time = 0.0;
        long step_count = 0;
        double next_progress = 0.0;

        while (true) {
            if (!ui->IsRunning(dt))
                break;
            if (time >= end_time)
                break;

            // Mirror single_run: only advance while the viewer Start button is on.
            if (!ui->simulationStarted)
                continue;

            fsi.DoStepDynamics(dt);
            time += dt;
            ++step_count;

            if (exporter) exporter->RecordStep(system.get());

            const ChVector3d p = ref_body ? ref_body->GetFrameRefToAbs().GetPos() : hull->GetPos();
            const ChVector3d rpy = hull->GetRot().GetCardanAnglesXYZ();
            const ChVector3d Ffsi = fsi.GetFsiBodyForce(hull);
            if (!std::isfinite(p.z()) || !std::isfinite(Ffsi.Length()) || std::abs(p.z()) > 50.0) {
                result.diverged = true;
                seastack::infra::cli::LogError(
                    "[coupled] Simulation diverged at t=" + std::to_string(time) + " s.");
                break;
            }
            if (!gui_ok && time >= next_progress) {
                double pct = 100.0 * time / end_time;
                seastack::infra::cli::LogInfo(
                    "[coupled] t=" + seastack::infra::FormatNumber(time, 2) + " s (" +
                    std::to_string(static_cast<int>(pct)) + "%)  heave=" +
                    seastack::infra::FormatNumber(p.z(), 4) + " m  roll=" +
                    seastack::infra::FormatNumber(rpy.x(), 4) + " rad  |F_tank|=" +
                    seastack::infra::FormatNumber(Ffsi.Length(), 0) + " N  n_sph=" +
                    std::to_string(fsi.GetNumSPHParticles()));
                next_progress += std::max(end_time / 20.0, dt);
            }
        }

        const auto wall_end = std::chrono::steady_clock::now();
        result.wall_time_s = std::chrono::duration<double>(wall_end - wall_start).count();
        result.sim_time_final = time;

        if (exporter) {
            try {
                exporter->SetRunMetadata(std::string(""), std::string(""), result.wall_time_s,
                                         static_cast<int>(step_count), dt, time);
                exporter->Finalize();
            } catch (const std::exception& e) {
                seastack::infra::cli::LogWarning(std::string("[coupled] HDF5 finalize failed: ") + e.what());
                result.primary_artifact_path.clear();
            }
        }

        result.exit_code = result.diverged ? 2 : 0;
        return result;

    } catch (const std::exception& e) {
        result.error_message = std::string("Coupled run failed: ") + e.what();
        seastack::infra::cli::LogError(result.error_message);
        return result;
    } catch (...) {
        result.error_message = "Coupled run failed: unknown fatal error.";
        seastack::infra::cli::LogError(result.error_message);
        return result;
    }
}

}  // namespace seastack::app
