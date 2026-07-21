/*********************************************************************
 * @file  single_run.cpp
 * @brief Implementation of RunSingleCase() — single sea-state runner.
 *
 * Executes one SEA-Stack simulation from a fully resolved config
 * (model, simulation, hydro data, output settings) and returns
 * structured results (wall time, sim time, divergence, PTO metrics).
 *
 * The simulation logic here was extracted from RunFromYAML (the CLI
 * runner).  Helper functions TryFindYamlDouble, HCParser, and
 * InitializeChronoSystem are file-local copies that both translation
 * units may share during the transition period.
 *********************************************************************/

#include "single_run.h"

#include <seastack/config.h>
#include <seastack/version.h>
#include <seastack/infra/logging.h>
#include <seastack/adapters/chrono/setup_from_yaml.h>
#include <seastack/hydro/config/yaml_parser.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/adapters/chrono/simulation_export.h>
#include <seastack/hydro/waves/wave_base.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#ifdef SEASTACK_HAVE_EXTERNAL
#include "external_pto_yaml.h"
#endif

#include <chrono_parsers/yaml/ChParserMbsYAML.h>
#include <chrono/physics/ChSystem.h>
#include <chrono/physics/ChBody.h>
#include <chrono/core/ChRealtimeStep.h>
#include <chrono/core/ChDataPath.h>
#include <chrono/assets/ChColor.h>
#include <chrono/assets/ChVisualShapeModelFile.h>
#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/core/ChFrame.h>
#include <chrono/core/ChMatrix33.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/timestepper/ChAssemblyAnalysis.h>
#include <yaml-cpp/yaml.h>

#include "gui/guihelper.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <cmath>

#ifdef SEASTACK_HAVE_HYDRO_IO
#include <H5Cpp.h>
#endif

#ifdef _WIN32
#include <windows.h>
#endif

using seastack::hydro::YAMLHydroData;
using seastack::hydro::WaveMode;
using seastack::hydro::WaveBase;
using seastack::hydro::LinearDirectionalWaveField;
using seastack::hydro::ReadHydroYAML;
using seastack::chrono::HydroSystem;
using seastack::chrono::SetupHydroFromYAML;

namespace seastack::app {

// ---------------------------------------------------------------------------
// File-local helpers (moved from run_from_yaml.cpp)
// ---------------------------------------------------------------------------

static bool TryFindYamlDouble(const std::string& yaml_path, const std::string& key, double& out_value) {
    std::ifstream in(yaml_path);
    if (!in.is_open()) {
        return false;
    }
    auto ltrim = [](std::string& s) { s.erase(0, s.find_first_not_of(" \t\r\n")); };
    auto rtrim = [](std::string& s) { size_t p = s.find_last_not_of(" \t\r\n"); if (p == std::string::npos) s.clear(); else s.erase(p + 1); };
    std::string line;
    while (std::getline(in, line)) {
        ltrim(line); rtrim(line);
        if (line.empty() || line[0] == '#') continue;
        size_t pos = line.find(':');
        if (pos == std::string::npos) continue;
        std::string k = line.substr(0, pos);
        std::string v = line.substr(pos + 1);
        ltrim(k); rtrim(k); ltrim(v); rtrim(v);
        if (k == key) {
            try {
                out_value = std::stod(v);
                return true;
            } catch (...) {
                return false;
            }
        }
    }
    return false;
}

/// Full file read for SimulationExporter::Options::model_yaml (PTO damping recovery, /inputs provenance).
static std::string ReadEntireFileUtf8(const std::string& path) {
    std::ifstream in(path, std::ios::in | std::ios::binary);
    if (!in.is_open()) {
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Thin subclass of ChParserMbsYAML that exposes the protected
// m_script_directory member so relative mesh paths resolve correctly.
namespace {
class HCParser : public ::chrono::parsers::ChParserMbsYAML {
  public:
    HCParser() : ChParserMbsYAML() {}
    void SetScriptDir(const std::string& dir) { m_file_handler.SetReferenceDirectory(dir); }
};

static std::shared_ptr<::chrono::ChBody> FindBodyByName(::chrono::ChSystem& system,
                                                        const std::string& name) {
    for (const auto& b : system.GetBodies()) {
        if (b && b->GetName() == name) {
            return b;
        }
    }
    return nullptr;
}

// YAML MBS LOCK / REVOLUTE frames for trimaran rigid do not match demo_trimaran_rigid.cpp
// (revolute needs QuatFromAngleY(pi/2); tip mate needs arm quaternion on both frames).
// When all four expected links exist, replace them with ChLinkLockRevolute + ChLinkMateGeneric.
static void RetargetTrimaranRigidArmJointsIfPresent(::chrono::ChSystem& system) {
    using ::chrono::ChFramed;
    using ::chrono::ChLinkLockRevolute;
    using ::chrono::ChLinkMateGeneric;
    using ::chrono::ChMatrix33;
    using ::chrono::ChQuaternion;
    using ::chrono::ChVector3d;
    using ::chrono::QuatFromAngleY;

    auto center = FindBodyByName(system, "body1");
    auto stbd_hull = FindBodyByName(system, "body2");
    auto port_hull = FindBodyByName(system, "body3");
    auto arm_stbd = FindBodyByName(system, "arm_stbd");
    auto arm_port = FindBodyByName(system, "arm_port");
    if (!center || !stbd_hull || !port_hull || !arm_stbd || !arm_port) {
        return;
    }

    const std::unordered_set<std::string> kExpected = {
        "center_arm_stbd_revolute", "center_arm_port_revolute", "arm_stbd_tip_lock",
        "arm_port_tip_lock"};
    std::vector<std::shared_ptr<::chrono::ChLinkBase>> to_remove;
    to_remove.reserve(kExpected.size());
    for (const auto& lk : system.GetLinks()) {
        if (lk && kExpected.count(lk->GetName())) {
            to_remove.push_back(lk);
        }
    }
    if (to_remove.size() != kExpected.size()) {
        return;
    }

    for (const auto& lk : to_remove) {
        system.RemoveLink(lk);
    }

    // Same geometry as demos/trimaran/trimaran_hulls.h + demo_trimaran_rigid.cpp AddRigidCrossArms.
    constexpr double kCenterHalfBeam = 6.0;
    constexpr double kOutriggerY = 15.0;
    constexpr double kOutriggerHalfBeam = 2.0;
    constexpr double kCenterFreeboard = 3.5;
    constexpr double kOutriggerFreeboard = 1.5;

    const double pi = 4.0 * std::atan(1.0);
    const ChQuaternion rev_rot = QuatFromAngleY(pi / 2.0);

    struct ArmSpec {
        const char* revolute_name;
        const char* weld_name;
        std::shared_ptr<::chrono::ChBody> arm;
        std::shared_ptr<::chrono::ChBody> outrigger;
        ChVector3d pt_root;
        ChVector3d pt_tip;
    };
    const ArmSpec arms[2] = {
        {"center_arm_stbd_revolute", "arm_stbd_tip_lock", arm_stbd, stbd_hull,
         ChVector3d(0, -kCenterHalfBeam, kCenterFreeboard),
         ChVector3d(0, -(kOutriggerY - kOutriggerHalfBeam), kOutriggerFreeboard)},
        {"center_arm_port_revolute", "arm_port_tip_lock", arm_port, port_hull,
         ChVector3d(0, kCenterHalfBeam, kCenterFreeboard),
         ChVector3d(0, (kOutriggerY - kOutriggerHalfBeam), kOutriggerFreeboard)},
    };

    for (const ArmSpec& a : arms) {
        const ChVector3d arm_vec = a.pt_tip - a.pt_root;
        ChMatrix33<> mrot;
        mrot.SetFromAxisX(arm_vec, ChVector3d(0, 0, 1));
        const ChQuaternion qa = mrot.GetQuaternion();

        auto revolute = ::chrono_types::make_shared<ChLinkLockRevolute>();
        revolute->SetName(a.revolute_name);
        revolute->Initialize(center, a.arm, ChFramed(a.pt_root, rev_rot));
        system.AddLink(revolute);

        auto tip_weld = ::chrono_types::make_shared<ChLinkMateGeneric>();
        tip_weld->SetName(a.weld_name);
        tip_weld->Initialize(a.arm, a.outrigger, false, ChFramed(a.pt_tip, qa),
                             ChFramed(a.pt_tip, qa));
        tip_weld->SetConstrainedCoords(true, true, true, true, true, true);
        system.AddLink(tip_weld);
    }

    seastack::infra::debug::LogDebug(
        "Retargeted trimaran rigid arm joints to match demo_trimaran_rigid.cpp frames.");
}

}  // namespace

static std::shared_ptr<::chrono::ChSystem> InitializeChronoSystem(
        const std::string& model_file, const std::string& sim_file) {
    seastack::infra::debug::LogDebug("Initializing Chrono system from YAML inputs...");

    try {
        seastack::infra::debug::LogDebug("Creating Chrono YAML parser");
        HCParser parser;

        std::filesystem::path model_dir = std::filesystem::path(model_file).parent_path();
        parser.SetScriptDir(model_dir.generic_string());
        seastack::infra::debug::LogDebug(std::string("Script directory set to: ") + model_dir.generic_string());

        seastack::infra::debug::LogDebug(std::string("Loading simulation file: ") + sim_file);
        auto sim_yaml = YAML::LoadFile(sim_file);
        parser.LoadSimData(sim_yaml);

        auto sim_node = sim_yaml["simulation"];
        if (sim_node && sim_node["contact_method"]) {
            if (sim_node["time_step"] && sim_node["integrator"] && !sim_node["integrator"]["time_step"])
                sim_node["integrator"]["time_step"] = sim_node["time_step"];
            parser.LoadSolverData(sim_node);
        }

        seastack::infra::debug::LogDebug("Creating system");
        auto system = parser.CreateSystem();

        seastack::infra::debug::LogDebug(std::string("Loading model file: ") + model_file);
        auto model_yaml = YAML::LoadFile(model_file);
        parser.LoadModelData(model_yaml);

        seastack::infra::debug::LogDebug("Analyzing mesh files referenced in YAML model");
        seastack::infra::debug::LogDebug(std::string("Model directory: ") + model_dir.generic_string());

        seastack::infra::debug::LogDebug("Populating system");
        parser.Populate(*system);
        seastack::infra::debug::LogDebug("System populated successfully");

        RetargetTrimaranRigidArmJointsIfPresent(*system);

        // Apply per-body visualization from model YAML (match Chrono bodies by name).
        // Chrono 10 MBS YAML loads model_file via ChBodyGeometry as ChVisualShapeTriangleMesh at identity
        // frame; it does not read shape_location. Older paths used ChVisualShapeModelFile. Apply YAML
        // shape_location and/or diffuse color here so hull meshes align with CoG like the C++ demos.
        try {
            auto model_node = model_yaml["model"];
            if (model_node && model_node["bodies"]) {
                std::unordered_map<std::string, YAML::Node> vis_by_name;
                for (const auto& entry : model_node["bodies"]) {
                    if (!entry["name"]) {
                        continue;
                    }
                    const std::string nm = entry["name"].as<std::string>();
                    vis_by_name.emplace(nm, entry["visualization"]);
                }

                auto resolve_mesh_path = [&](const std::string& path_str) -> std::string {
                    std::filesystem::path p(path_str);
                    if (p.is_relative()) {
                        p = model_dir / p;
                    }
                    std::error_code ec;
                    std::filesystem::path c = std::filesystem::weakly_canonical(p, ec);
                    if (!ec) {
                        return c.generic_string();
                    }
                    return p.lexically_normal().generic_string();
                };

                for (const auto& body : system->GetBodies()) {
                    if (!body) {
                        continue;
                    }
                    auto vit = vis_by_name.find(body->GetName());
                    if (vit == vis_by_name.end()) {
                        continue;
                    }
                    const YAML::Node& vis_node = vit->second;
                    if (!vis_node) {
                        continue;
                    }
                    const bool has_color =
                        static_cast<bool>(vis_node["color"]) && vis_node["color"].IsSequence() &&
                        vis_node["color"].size() >= 3;
                    const bool has_shape_location =
                        static_cast<bool>(vis_node["shape_location"]) &&
                        vis_node["shape_location"].IsSequence() &&
                        vis_node["shape_location"].size() >= 3;
                    if (!has_color && !has_shape_location) {
                        continue;
                    }
                    float r = 1.f;
                    float g = 1.f;
                    float b = 1.f;
                    if (has_color) {
                        auto c = vis_node["color"];
                        r = c[0].as<float>();
                        g = c[1].as<float>();
                        b = c[2].as<float>();
                    }
                    ::chrono::ChVector3d shape_pos(0, 0, 0);
                    if (has_shape_location) {
                        auto sl = vis_node["shape_location"];
                        shape_pos.x() = sl[0].as<double>();
                        shape_pos.y() = sl[1].as<double>();
                        shape_pos.z() = sl[2].as<double>();
                    }
                    auto vis = body->GetVisualModel();
                    if (!vis) {
                        continue;
                    }
                    for (unsigned si = 0; si < vis->GetNumShapes(); ++si) {
                        auto shape = body->GetVisualShape(si);
                        auto existing_tri =
                            std::dynamic_pointer_cast<::chrono::ChVisualShapeTriangleMesh>(shape);
                        if (existing_tri && existing_tri->GetMesh()) {
                            auto mesh = existing_tri->GetMesh();
                            auto tri_shape =
                                ::chrono_types::make_shared<::chrono::ChVisualShapeTriangleMesh>(mesh,
                                                                                                 false);
                            if (has_color) {
                                tri_shape->SetColor(::chrono::ChColor(r, g, b));
                            } else {
                                for (unsigned mi = 0; mi < existing_tri->GetNumMaterials(); ++mi) {
                                    tri_shape->SetMaterial(mi, existing_tri->GetMaterial(mi));
                                }
                            }
                            body->GetVisualModel()->Clear();
                            body->AddVisualShape(tri_shape, ::chrono::ChFrame<>(shape_pos));
                            break;
                        }
                        auto mf = std::dynamic_pointer_cast<::chrono::ChVisualShapeModelFile>(shape);
                        if (!mf) {
                            continue;
                        }
                        std::string obj_path = resolve_mesh_path(mf->GetFilename());
                        auto trimesh =
                            ::chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(obj_path, true, false);
                        if (!trimesh) {
                            seastack::infra::debug::LogDebug(
                                std::string("CreateFromWavefrontFile failed for body ") + body->GetName() +
                                ": " + obj_path);
                            continue;
                        }
                        auto tri_shape =
                            ::chrono_types::make_shared<::chrono::ChVisualShapeTriangleMesh>(trimesh, false);
                        if (has_color) {
                            tri_shape->SetColor(::chrono::ChColor(r, g, b));
                        }
                        body->GetVisualModel()->Clear();
                        body->AddVisualShape(tri_shape, ::chrono::ChFrame<>(shape_pos));
                        break;
                    }
                }
            }
        } catch (const std::exception& e) {
            seastack::infra::debug::LogDebug(std::string("Body visualization YAML application failed: ") +
                                             e.what());
        } catch (...) {
            seastack::infra::debug::LogDebug("Body visualization YAML application failed (unknown error)");
        }

        return system;
    } catch (const std::exception& e) {
        seastack::infra::cli::LogError(std::string("Failed to initialize Chrono system: ") + e.what());
        return nullptr;
    }
}

// ==========================================================================
// RunSingleCase — core single-case simulation entry point
// ==========================================================================

SingleRunResult RunSingleCase(const SingleRunConfig& config) {
    SingleRunResult result;
    try {
        std::string persisted_hdf5_path;
        std::string artifact_note_text;
        auto wall_start = std::chrono::steady_clock::now();
        bool nogui = config.nogui;

        const bool show_system_banner = !config.concise_cli || config.debug_mode;
        const auto startup_line = [&](const std::string& msg) {
            seastack::infra::debug::LogDebug(msg);
        };

        // Profiling accumulators
        std::chrono::steady_clock::time_point t;
        double prof_setup_seconds = 0.0;
        double prof_loop_seconds = 0.0;
        double prof_export_seconds = 0.0;
        double prof_other_seconds = 0.0;

        // -----------------------------------------------------------------
        // 1. Initialize Chrono System
        // -----------------------------------------------------------------
        startup_line("[startup] Initializing solver...");
        auto system = InitializeChronoSystem(config.model_file, config.simulation_file);
        if (!system) {
            seastack::infra::cli::LogError("[solver] Chrono system initialization failed. "
                "This is fatal — the physics engine could not be created from the model/simulation YAML files.");
            seastack::infra::cli::LogError("  - Verify model and simulation YAML files are valid.");
            seastack::infra::cli::LogError("  - Run with --debug for detailed initialization logs.");
            result.error_message = "Chrono system initialization failed.";
            return result;
        }
        startup_line("[startup] Solver initialized");

        if (config.debug_mode && !system->GetLinks().empty()) {
            const auto asm_exit =
                system->DoAssembly(::chrono::AssemblyAnalysis::POSITION |
                                   ::chrono::AssemblyAnalysis::VELOCITY);
            if (asm_exit == ::chrono::AssemblyAnalysis::ExitFlag::NOT_CONVERGED) {
                seastack::infra::cli::LogWarning(
                    "DoAssembly(POSITION|VELOCITY) did not converge. "
                    "Joint frames or initial coordinates may be inconsistent.");
            }
        }

        // -----------------------------------------------------------------
        // 2. Setup hydrodynamic forces and display wave info
        // -----------------------------------------------------------------
        std::unique_ptr<HydroSystem> hydro_forces;
        YAMLHydroData hydro_data;
        double loop_dt = system->GetStep();
        {
            double yaml_dt = 0.0;
            if (TryFindYamlDouble(config.simulation_file, "time_step", yaml_dt) && yaml_dt > 0.0) {
                loop_dt = yaml_dt;
            }
        }

#ifdef SEASTACK_HAVE_EXTERNAL
        seastack::app::ExternalPtoAttachment external_pto_attachment;
        if (config.has_external_pto) {
            try {
                external_pto_attachment = seastack::app::ExternalPtoAttachment::Attach(
                    *system, config.external_pto, loop_dt);
            } catch (const std::exception& e) {
                seastack::infra::cli::LogError(
                    std::string("external_pto attach failed: ") + e.what());
                result.error_message = e.what();
                return result;
            }
        }
#endif

        std::string hydro_file_path;  // non-empty only when source is a file
        bool hydro_data_ready = false;

        if (auto* path_ptr = std::get_if<std::string>(&config.hydro_source)) {
            hydro_file_path = *path_ptr;
            if (!hydro_file_path.empty()) {
                // Source is a file path — parse it
                if (std::filesystem::exists(hydro_file_path)) {
                    hydro_data = ReadHydroYAML(hydro_file_path);
                    hydro_data_ready = true;
                } else {
                    seastack::infra::cli::LogWarning("Hydro file not found: " + hydro_file_path);
                    seastack::infra::cli::ShowSummaryLine("🌊", "Type", "None (file not found)", seastack::infra::LogColor::Yellow);
                }
            }
        } else if (auto* data_ptr = std::get_if<YAMLHydroData>(&config.hydro_source)) {
            hydro_data = *data_ptr;
            hydro_data_ready = true;
        }

        if (hydro_data_ready) {
            seastack::infra::debug::LogDebug(std::string("Parsed ") + std::to_string(hydro_data.bodies.size()) + " body(ies)");

            try {
                seastack::infra::debug::LogDebug("Finding Chrono bodies in system...");
                std::vector<std::shared_ptr<::chrono::ChBody>> bodies;
                for (auto& body : system->GetBodies()) {
                    bodies.push_back(body);
                }
                seastack::infra::debug::LogDebug(std::string("Found ") + std::to_string(bodies.size()) + " Chrono body(ies)");

                seastack::infra::debug::LogDebug("Initializing HydroSystem...");
                double sim_duration_hint = 0.0;
                TryFindYamlDouble(config.simulation_file, "end_time", sim_duration_hint);
                hydro_forces = SetupHydroFromYAML(hydro_data, bodies, loop_dt, sim_duration_hint, 0.0);
                seastack::infra::debug::LogDebug("Hydrodynamic forces initialized successfully");

                // Diagnostics CSV output directory (precedence: explicit diagnostics dir,
                // HDF5 output dir, parent of hydro_yaml_path, parent of hydro file path).
                try {
                    std::filesystem::path diag_dir;
                    if (!config.diagnostics_output_directory.empty()) {
                        diag_dir = std::filesystem::path(config.diagnostics_output_directory);
                    } else if (!config.output_directory.empty()) {
                        diag_dir = std::filesystem::path(config.output_directory);
                    } else if (!config.hydro_yaml_path.empty()) {
                        diag_dir = std::filesystem::path(config.hydro_yaml_path).parent_path();
                    } else if (!hydro_file_path.empty()) {
                        diag_dir = std::filesystem::path(hydro_file_path).parent_path();
                    }
                    if (hydro_forces && !diag_dir.empty()) {
                        hydro_forces->SetDiagnosticsOutputDirectory(diag_dir.string());
                    }
                } catch (...) {}

                if (hydro_forces && config.profile_mode) {
                    hydro_forces->SetProfilingEnabled(true);
                }

                // Wave CLI widgets are verbose; show only with --debug
                if (config.debug_mode) {
                    if (!hydro_data.waves.partitions.empty() ||
                        (hydro_data.waves.spreading.type != "none" && !hydro_data.waves.spreading.type.empty())) {
                        std::vector<seastack::infra::cli::WavePartitionSummary> summaries;
                        if (!hydro_data.waves.partitions.empty()) {
                            for (const auto& p : hydro_data.waves.partitions) {
                                summaries.push_back({p.spectrum_type, p.Hs, p.Tp,
                                                     p.mean_direction_deg,
                                                     p.spreading.type, p.spreading.s});
                            }
                        } else {
                            summaries.push_back({hydro_data.waves.spectrum,
                                                 hydro_data.waves.height,
                                                 hydro_data.waves.period,
                                                 hydro_data.waves.direction,
                                                 hydro_data.waves.spreading.type,
                                                 hydro_data.waves.spreading.s});
                        }
                        int n_omega = hydro_data.waves.discretization.n_omega > 0
                                    ? hydro_data.waves.discretization.n_omega
                                    : (hydro_data.waves.nfrequencies > 0
                                           ? hydro_data.waves.nfrequencies
                                           : seastack::hydro::kDefaultWaveSpectralNOmega);
                        int n_theta = hydro_data.waves.discretization.n_theta > 0
                                    ? hydro_data.waves.discretization.n_theta : 1;
                        int n_comp = 0;
                        if (hydro_forces && hydro_forces->GetWave()) {
                            const auto* wc = hydro_forces->GetWave()->GetWaveComponents();
                            if (wc) n_comp = static_cast<int>(wc->size());
                        }
                        seastack::infra::cli::ShowDirectionalWaveModel(
                            hydro_data.waves.type, summaries, n_comp, n_omega, n_theta);
                    } else {
                        seastack::infra::cli::ShowWaveModel(hydro_data.waves.type,
                                               hydro_data.waves.height,
                                               hydro_data.waves.period,
                                               hydro_data.waves.direction,
                                               hydro_data.waves.phase);
                    }
                }

            }
#if defined(SEASTACK_HAVE_HYDRO_IO)
            catch (const H5::Exception& e) {
                std::string detail = e.getFuncName() + std::string(" — ") + e.getDetailMsg();
                seastack::infra::cli::LogError(
                    std::string("Failed to setup hydrodynamic forces (HDF5): ") + detail);
                seastack::infra::cli::CollectWarning("Continuing without hydrodynamic forces...");
                seastack::infra::cli::ShowSummaryLine("🌊", "Type", "None (setup failed)", seastack::infra::LogColor::Yellow);
            }
#endif
            catch (const std::exception& e) {
                seastack::infra::cli::LogError(std::string("Failed to setup hydrodynamic forces: ") + e.what());
                seastack::infra::cli::CollectWarning("Continuing without hydrodynamic forces...");
                seastack::infra::cli::ShowSummaryLine("🌊", "Type", "None (setup failed)", seastack::infra::LogColor::Yellow);
            } catch (...) {
                seastack::infra::cli::LogError(
                    "Failed to setup hydrodynamic forces: unexpected exception during hydro setup "
                    "(possible cross-library exception handling issue).");
                seastack::infra::cli::CollectWarning("Continuing without hydrodynamic forces...");
                seastack::infra::cli::ShowSummaryLine("🌊", "Type", "None (setup failed)", seastack::infra::LogColor::Yellow);
            }
        } else if (hydro_file_path.empty()) {
            // No hydro source provided at all
            seastack::infra::debug::LogDebug("No hydro data provided, running without hydrodynamic forces");
            if (config.debug_mode) {
                seastack::infra::cli::ShowSummaryLine("🌊", "Type", "None (still water)", seastack::infra::LogColor::White);
            }
        }

        // -----------------------------------------------------------------
        // 3. Setup visualization
        // -----------------------------------------------------------------
        startup_line("[startup] Visualization init...");

        std::shared_ptr<seastack::viz::UI> pui;
        bool gui_init_ok = false;
        try {
            pui = seastack::viz::CreateUI(!nogui);
        } catch (const std::exception& e) {
            seastack::infra::cli::LogWarning(std::string("[viz] CreateUI failed: ") + e.what());
        } catch (...) {
            seastack::infra::cli::LogWarning("[viz] CreateUI failed (unknown error)");
        }

        if (pui && !nogui) {
            try {
                pui->Init(system.get(), "SEA-Stack YAML");
                gui_init_ok = true;
            } catch (const std::exception& e) {
                seastack::infra::cli::LogWarning(std::string("[viz] GUI initialization failed: ") + e.what());
                pui.reset();
            } catch (...) {
                seastack::infra::cli::LogWarning("[viz] GUI initialization failed (unknown error)");
                pui.reset();
            }
        }

        if (!pui) {
            if (!nogui) {
                seastack::infra::cli::LogWarning(
                    "[viz] Visualization unavailable — falling back to headless mode. "
                    "The simulation will continue without a GUI window.");
                seastack::infra::cli::LogWarning(
                    "[viz] To suppress this warning, run with --nogui.");
                nogui = true;
            }
            pui = seastack::viz::CreateUI(true);
            pui->Init(system.get(), "SEA-Stack");
        }

        seastack::viz::UI& ui = *pui;

        if (gui_init_ok) {
            try {
                ui.SetCamera(0, -50, -10, 0, 0, -10);
            } catch (const std::exception& e) {
                seastack::infra::cli::LogWarning(std::string("Camera setup failed: ") + e.what());
            }

            try {
                if (hydro_forces) {
                    auto wave_ptr = hydro_forces->GetWave();
                    if (wave_ptr) {
                        ui.SetWaveModel(wave_ptr);
                    }
                }
            } catch (const std::exception& e) {
                seastack::infra::cli::LogWarning(std::string("Wave visualization setup failed: ") + e.what());
            }

#ifdef SEASTACK_HAVE_MOORDYN
            if (hydro_forces && hydro_data.moordyn_enabled && !hydro_data.moordyn_input_file.empty()) {
                ui.SetMooringLineProvider([&hydro_forces]() {
                    return hydro_forces->GetMooringLineStates();
                });
                ui.SetMooringVisualizationRadii(hydro_data.moordyn_visualization_line_radius,
                                                 hydro_data.moordyn_visualization_endpoint_radius,
                                                 hydro_data.moordyn_visualization_node_marker_radius);
            }
#endif
        }

        if (gui_init_ok) {
            startup_line("[startup] Visualization: GUI ready");
        } else {
            startup_line("[startup] Visualization: headless mode");
        }

        // -----------------------------------------------------------------
        // 4. System Readiness Summary (verbose layout only with --debug)
        // -----------------------------------------------------------------
        if (config.debug_mode) {
            seastack::infra::cli::ShowSectionSeparator();

            seastack::infra::cli::LogSuccess("✅ Chrono system initialized — ready to begin simulation loop");
            seastack::infra::cli::ShowEmptyLine();

            std::vector<std::string> system_info_lines;
            system_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("🔗", "Bodies", std::to_string(system->GetBodies().size())));
            system_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("⚙️", "Constraints", std::to_string(system->GetLinks().size())));
            system_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("⏱️", "Time Step", seastack::infra::FormatNumber(loop_dt, 4) + " s"));

            int num_bodies = system->GetBodies().size();
            int approx_dof = num_bodies * 6;
            system_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("🎯", "Est. Degrees of Freedom", std::to_string(approx_dof)));

            seastack::infra::cli::ShowSectionBox("System Configuration", system_info_lines);
        }

        if (config.debug_mode) {
            seastack::infra::cli::ShowEmptyLine();
            std::vector<std::string> solver_info_lines;

            auto solver = system->GetSolver();
            if (solver) {
                solver_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("🔧", "Solver Type", "ChSolver (default)"));
                solver_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("🎯", "Max Iterations", "150 (default)"));
                solver_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("📐", "Tolerance", "1e-10 (default)"));
                try {
                    solver_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("🔍", "Solver State", "Active"));
                } catch (...) {
                    solver_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("🔍", "Solver State", "Unknown"));
                }
            } else {
                solver_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("⚠️", "Solver", "No solver detected"));
            }

            solver_info_lines.push_back(seastack::infra::cli::CreateAlignedLine("📊", "System DOF", "Computing..."));

            seastack::infra::cli::ShowSectionBox("Solver Configuration", solver_info_lines);
        }

        // -----------------------------------------------------------------
        // 5. HDF5 exporter (always created for PTO metrics; uses a temp
        //    directory when per-cell output is not requested)
        // -----------------------------------------------------------------
        std::unique_ptr<seastack::chrono::SimulationExporter> exporter;
        bool export_detailed_hydro = false;
        std::filesystem::path resolved_output_dir;
        bool using_temp_exporter = false;
        {
            try {
                if (!config.output_directory.empty()) {
                    resolved_output_dir = std::filesystem::path(config.output_directory);
                } else {
                    resolved_output_dir = std::filesystem::temp_directory_path() /
                        ("seastack_metrics_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
                    using_temp_exporter = true;
                }
                std::error_code ec;
                std::filesystem::create_directories(resolved_output_dir, ec);
                if (!using_temp_exporter) {
                    startup_line(std::string("[startup] Output directory: ") + resolved_output_dir.string());
                }

                std::string wave_type = hydro_data.waves.type.empty() ? std::string("still") : hydro_data.waves.type;
                std::filesystem::path output_h5 = resolved_output_dir / (std::string("results.") + wave_type + ".h5");

                seastack::chrono::SimulationExporter::Options exp_opts;
                exp_opts.output_path = output_h5.generic_string();
                exp_opts.input_model_file = config.model_file;
                exp_opts.input_simulation_file = config.simulation_file;
                if (!hydro_file_path.empty()) {
                    exp_opts.input_hydro_file = hydro_file_path;
                }
                exp_opts.output_directory = resolved_output_dir.generic_string();
                exp_opts.scenario_type = wave_type;
                exp_opts.export_config = config.export_config;
                exp_opts.log_final_output_path = false;

                if (!using_temp_exporter) {
                    persisted_hdf5_path = output_h5.generic_string();
                } else {
                    artifact_note_text =
                        "No persistent HDF5 (add output directory in setup to save results).";
                }

                // CLI --output-level overrides YAML
                if (!config.cli_output_level.empty()) {
                    if (config.cli_output_level == "compact")       exp_opts.export_config.level = seastack::chrono::ExportLevel::kCompact;
                    else if (config.cli_output_level == "detailed") exp_opts.export_config.level = seastack::chrono::ExportLevel::kDetailed;
                    else                                            exp_opts.export_config.level = seastack::chrono::ExportLevel::kStandard;
                }
                {
                    std::string model_text = ReadEntireFileUtf8(config.model_file);
                    if (!model_text.empty()) {
                        exp_opts.model_yaml = std::move(model_text);
                    } else if (!config.model_file.empty()) {
                        seastack::infra::cli::LogWarning(
                            std::string("[startup] Could not read model file for exporter (damping recovery / provenance): ")
                            + config.model_file);
                    }
                }
                exporter = std::make_unique<seastack::chrono::SimulationExporter>(exp_opts);

                if (exp_opts.export_config.level == seastack::chrono::ExportLevel::kDetailed && hydro_forces) {
                    hydro_forces->SetPerComponentCaptureEnabled(true);
                    export_detailed_hydro = true;
                }

                double duration_hint = 0.0; TryFindYamlDouble(config.simulation_file, "end_time", duration_hint);
                exporter->WriteSimulationInfo(system.get(), std::string(""), std::filesystem::path(config.model_file).filename().generic_string(), loop_dt, duration_hint);
                exporter->WriteModel(system.get());
                {
                    int est_steps = 0;
                    if (duration_hint > 0.0 && loop_dt > 0.0) {
                        int dec = std::max(1, exp_opts.export_config.decimation);
                        est_steps = static_cast<int>(duration_hint / (loop_dt * dec));
                    }
                    exporter->BeginResults(system.get(), est_steps);
                }

                if (hydro_forces) {
                    auto wave_ptr = hydro_forces->GetWave();
                    if (wave_ptr && wave_ptr->GetWaveMode() == WaveMode::kIrregular) {
                        try {
                            auto ldwf = std::dynamic_pointer_cast<LinearDirectionalWaveField>(wave_ptr);
                            if (ldwf) {
                                std::vector<double> f = ldwf->GetFrequenciesHz();
                                std::vector<double> S = ldwf->GetSpectralDensityEstimate();
                                auto [tvec, eta] = ldwf->ComputeElevationTimeSeries(0.0, duration_hint, loop_dt);
                                exporter->WriteIrregularInputs(f, S, tvec, eta);
                            }
                        }
#if defined(SEASTACK_HAVE_HYDRO_IO)
                        catch (const H5::Exception& e) {
                            seastack::infra::debug::LogDebug(
                                std::string("Skipping spectrum HDF5 export (HDF5): ") + e.getDetailMsg());
                        }
#endif
                        catch (const std::exception& e) {
                            seastack::infra::debug::LogDebug(std::string("Skipping spectrum HDF5 export: ") + e.what());
                        }
                    }
                }

            }
#if defined(SEASTACK_HAVE_HYDRO_IO)
            catch (const H5::Exception& e) {
                std::string detail = e.getFuncName() + std::string(" — ") + e.getDetailMsg();
                seastack::infra::cli::LogWarning(std::string("HDF5 exporter disabled (HDF5): ") + detail);
                exporter.reset();
            }
#endif
            catch (const std::exception& e) {
                seastack::infra::cli::LogWarning(std::string("HDF5 exporter disabled: ") + e.what());
                exporter.reset();
            } catch (...) {
                seastack::infra::cli::LogWarning(
                    "HDF5 exporter disabled: unexpected exception during HDF5 export "
                    "(possible cross-library exception handling issue).");
                exporter.reset();
            }
        }

        // -----------------------------------------------------------------
        // 6. Run simulation
        // -----------------------------------------------------------------
        startup_line("[startup] Entering simulation loop");
        bool first_step = true;
        int step_count = 0;
        double initial_time = system->GetChTime();
        double previous_time = initial_time;
        bool diverged = false;

        std::shared_ptr<::chrono::ChBody> first_body = nullptr;
        if (!system->GetBodies().empty()) {
            first_body = system->GetBodies()[0];
        }

        double yaml_end_time = 0.0;
        TryFindYamlDouble(config.simulation_file, "end_time", yaml_end_time);

        // Diagnostic dump: compare system state to compiled regression tests.
        if (config.debug_mode) {
            std::ostringstream diag;
            diag << "[pre-step diag] Timestepper type id = "
                 << static_cast<int>(system->GetTimestepper()->GetType())
                 << ", Solver type id = " << static_cast<int>(system->GetSolver()->GetType()) << "\n";
            for (const auto& b : system->GetBodies()) {
                auto p = b->GetPos();
                diag << "  body '" << b->GetName()
                     << "' fixed=" << b->IsFixed()
                     << " mass=" << b->GetMass()
                     << " pos=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                     << " offset_w=" << b->GetOffset_w() << "\n";
            }
            seastack::infra::debug::LogDebug(diag.str());
        }

        if (nogui) {
            const double end_time_bound = (yaml_end_time > 0.0) ? yaml_end_time : 40.0;
            const double remaining_time = std::max(0.0, end_time_bound - initial_time);
            const size_t total_steps_est = static_cast<size_t>(std::max(1.0, std::ceil(remaining_time / std::max(1e-12, loop_dt))));
            size_t last_progress_step = 0;

            seastack::infra::cli::ShowProgress(0, total_steps_est,
                std::string("t=") + seastack::infra::FormatNumber(initial_time, 2) + " / " +
                seastack::infra::FormatNumber(end_time_bound, 2));

            while (system->GetChTime() < end_time_bound) {
                double current_time = system->GetChTime();
                try {
                    if (config.profile_mode) { t = std::chrono::steady_clock::now(); }
                    system->DoStepDynamics(loop_dt);
                    if (config.profile_mode) { prof_loop_seconds += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t).count(); }
                    step_count++;
                    if (exporter) {
                        if (config.profile_mode) { t = std::chrono::steady_clock::now(); }
                        exporter->RecordStep(system.get());
                        if (export_detailed_hydro) {
                            exporter->RecordHydroForces(hydro_forces->GetLastComponentForces());
                        }
                        if (config.profile_mode) { prof_export_seconds += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t).count(); }
                    }
                    if (step_count == 1 || step_count - static_cast<int>(last_progress_step) >= 25) {
                        const size_t current_steps = static_cast<size_t>(std::min<double>(total_steps_est, std::ceil((system->GetChTime() - initial_time) / std::max(1e-12, loop_dt))));
                        std::string msg = std::string("t=") + seastack::infra::FormatNumber(system->GetChTime(), 2) + " / " +
                                          seastack::infra::FormatNumber(end_time_bound, 2);
                        seastack::infra::cli::ShowProgress(current_steps, total_steps_est, msg);
                        last_progress_step = static_cast<size_t>(step_count);
                    }
                    previous_time = current_time;

                    if (hydro_forces && hydro_forces->HasDiverged()) {
                        seastack::infra::cli::StopProgress();
                        seastack::infra::cli::LogError("Simulation diverged — terminating. See above for details.");
                        diverged = true;
                        break;
                    }
                } catch (const std::exception& e) {
                    seastack::infra::cli::StopProgress();
                    seastack::infra::cli::LogError(std::string("🔥 Exception during DoStepDynamics at step ") + std::to_string(step_count) + ": " + e.what());
                    seastack::infra::cli::LogError(std::string("Simulation time: ") + seastack::infra::FormatNumber(current_time, 6) + " s");
                    seastack::infra::cli::LogError(std::string("Step size: ") + seastack::infra::FormatNumber(loop_dt, 6) + " s");
                    break;
                } catch (...) {
                    seastack::infra::cli::StopProgress();
                    seastack::infra::cli::LogError(std::string("🔥 Unknown exception during DoStepDynamics at step ") + std::to_string(step_count));
                    seastack::infra::cli::LogError(std::string("Simulation time: ") + seastack::infra::FormatNumber(current_time, 6) + " s");
                    break;
                }
            }
            // Finalize progress line
            if (system->GetChTime() >= end_time_bound - 1e-9) {
                const std::string fin_msg =
                    std::string("t=") + seastack::infra::FormatNumber(system->GetChTime(), 2) + " / " +
                    seastack::infra::FormatNumber(end_time_bound, 2);
                seastack::infra::cli::ShowProgress(total_steps_est, total_steps_est, fin_msg);
            } else {
                seastack::infra::cli::StopProgress();
            }
        } else {
            // GUI-driven loop
            startup_line("Tip: if the GUI becomes unresponsive, try --nogui for headless mode.");
            while (ui.IsRunning(loop_dt)) {
                if (yaml_end_time > 0.0 && system->GetChTime() >= yaml_end_time) {
                    startup_line(std::string("Reached configured end_time: ") + seastack::infra::FormatNumber(yaml_end_time, 3) + " s. Stopping.");
                    break;
                }
                if (ui.simulationStarted) {
                double current_time = system->GetChTime();

                if (config.trace_mode) {
                    std::string step_info = "⏱️ t = " + seastack::infra::FormatNumber(current_time, 3) + " s";
                    if (first_body) {
                        auto pos = first_body->GetPos();
                        auto vel = first_body->GetPosDt();
                        step_info += " | Body0: pos=(" + seastack::infra::FormatNumber(pos.x(), 2) + "," +
                                   seastack::infra::FormatNumber(pos.y(), 2) + "," + seastack::infra::FormatNumber(pos.z(), 2) + ")";
                        step_info += " vel=(" + seastack::infra::FormatNumber(vel.x(), 2) + "," +
                                   seastack::infra::FormatNumber(vel.y(), 2) + "," + seastack::infra::FormatNumber(vel.z(), 2) + ")";
                    }
                    seastack::infra::debug::LogDebug(step_info);
                } else if (config.debug_mode && step_count % 25 == 0) {
                    seastack::infra::debug::LogDebug(std::string("⏱️ t = ") + seastack::infra::FormatNumber(current_time, 3) + " s (step " + std::to_string(step_count) + ")");
                } else if (step_count % 50 == 0) {
                    seastack::infra::debug::LogDebug(std::string("⏱️ t = ") + seastack::infra::FormatNumber(current_time, 3) + " s (step " + std::to_string(step_count) + ")");
                }

                try {
                    if (config.profile_mode) { t = std::chrono::steady_clock::now(); }
                    system->DoStepDynamics(loop_dt);
                    if (config.profile_mode) { prof_loop_seconds += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t).count(); }
                    step_count++;
                    if (exporter) {
                        if (config.profile_mode) { t = std::chrono::steady_clock::now(); }
                        exporter->RecordStep(system.get());
                        if (export_detailed_hydro) {
                            exporter->RecordHydroForces(hydro_forces->GetLastComponentForces());
                        }
                        if (config.profile_mode) { prof_export_seconds += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t).count(); }
                    }

                    if (first_step) {
                        double new_time = system->GetChTime();
                        if (std::abs(new_time - current_time) < 1e-12) {
                            seastack::infra::cli::LogWarning("⚠️ Simulation did not progress — check constraints, initial state, or instability");
                            seastack::infra::cli::LogWarning(std::string("Time before step: ") + seastack::infra::FormatNumber(current_time, 6) + " s");
                            seastack::infra::cli::LogWarning(std::string("Time after step:  ") + seastack::infra::FormatNumber(new_time, 6) + " s");
                            seastack::infra::cli::LogWarning(std::string("Time difference:  ") + seastack::infra::FormatNumber(new_time - current_time, 10) + " s");
                            if (config.debug_mode) {
                                seastack::infra::debug::LogDebug("🔍 Checking system state for stall...");
                                seastack::infra::debug::LogDebug(std::string("Bodies count: ") + std::to_string(system->GetBodies().size()));
                                seastack::infra::debug::LogDebug(std::string("Constraints count: ") + std::to_string(system->GetLinks().size()));
                                if (first_body) {
                                    auto pos = first_body->GetPos();
                                    auto vel = first_body->GetPosDt();
                                    seastack::infra::debug::LogDebug(std::string("First body position: (") +
                                              seastack::infra::FormatNumber(pos.x(), 6) + ", " +
                                              seastack::infra::FormatNumber(pos.y(), 6) + ", " +
                                              seastack::infra::FormatNumber(pos.z(), 6) + ")");
                                    seastack::infra::debug::LogDebug(std::string("First body velocity: (") +
                                              seastack::infra::FormatNumber(vel.x(), 6) + ", " +
                                              seastack::infra::FormatNumber(vel.y(), 6) + ", " +
                                              seastack::infra::FormatNumber(vel.z(), 6) + ")");
                                }
                            }
                        } else {
                            if (config.debug_mode) {
                                seastack::infra::debug::LogDebug(std::string("✅ Simulation progressing normally (Δt = ") +
                                          seastack::infra::FormatNumber(new_time - current_time, 6) + " s)");
                            }
                        }

                        first_step = false;
                    }

                    if (config.debug_mode && step_count % 25 == 0) {
                        if (config.trace_mode) {
                            seastack::infra::debug::LogDebug(std::string("🔍 Step ") + std::to_string(step_count) + " solver info: [convergence data not available]");
                        }
                    }

                    previous_time = current_time;

                    if (hydro_forces && hydro_forces->HasDiverged()) {
                        seastack::infra::cli::LogError("Simulation diverged — terminating. See above for details.");
                        diverged = true;
                        break;
                    }

                } catch (const std::exception& e) {
                    seastack::infra::cli::LogError(std::string("🔥 Exception during DoStepDynamics at step ") + std::to_string(step_count) + ": " + e.what());
                    seastack::infra::cli::LogError(std::string("Simulation time: ") + seastack::infra::FormatNumber(current_time, 6) + " s");
                    seastack::infra::cli::LogError(std::string("Step size: ") + seastack::infra::FormatNumber(loop_dt, 6) + " s");

                    if (config.debug_mode && first_body) {
                        auto pos = first_body->GetPos();
                        auto vel = first_body->GetPosDt();
                        seastack::infra::cli::LogError("First body state at failure:");
                        seastack::infra::cli::LogError(std::string("  Position: (") + seastack::infra::FormatNumber(pos.x(), 6) + ", " +
                                    seastack::infra::FormatNumber(pos.y(), 6) + ", " + seastack::infra::FormatNumber(pos.z(), 6) + ")");
                        seastack::infra::cli::LogError(std::string("  Velocity: (") + seastack::infra::FormatNumber(vel.x(), 6) + ", " +
                                    seastack::infra::FormatNumber(vel.y(), 6) + ", " + seastack::infra::FormatNumber(vel.z(), 6) + ")");
                    }

                    seastack::infra::cli::LogWarning("This may indicate numerical instability, constraint conflicts, or configuration issues");
                    break;
                } catch (...) {
                    seastack::infra::cli::LogError(std::string("🔥 Unknown exception during DoStepDynamics at step ") + std::to_string(step_count));
                    seastack::infra::cli::LogError(std::string("Simulation time: ") + seastack::infra::FormatNumber(current_time, 6) + " s");
                    seastack::infra::cli::LogError("This indicates a serious system error");
                    break;
                }
            }
            }
        }

        auto wall_end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start);
        const double wall_s = std::chrono::duration_cast<std::chrono::duration<double>>(wall_end - wall_start).count();

        // Finalize HDF5 output with runtime metadata
        if (exporter) {
            exporter->SetRunMetadata(std::string(""), std::string(""), wall_s, step_count, loop_dt, system->GetChTime());
            exporter->Finalize();
        }

        result.primary_artifact_path = persisted_hdf5_path;
        result.artifact_note = artifact_note_text;

        // Optional profiling summary
        if (config.profile_mode) {
            double wall_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(wall_end - wall_start).count();
            prof_other_seconds = std::max(0.0, wall_seconds - (prof_setup_seconds + prof_loop_seconds + prof_export_seconds));

            std::vector<std::string> prof_lines;
            auto pct = [&](double s){ return seastack::infra::FormatNumber(100.0 * (s / std::max(1e-12, wall_seconds)), 1) + "%"; };

            prof_lines.push_back(seastack::infra::cli::CreateAlignedLine("📦", "Setup", seastack::infra::FormatNumber(prof_setup_seconds, 3) + " s (" + pct(prof_setup_seconds) + ")"));
            prof_lines.push_back(seastack::infra::cli::CreateAlignedLine("⚙️", "Dynamics Loop", seastack::infra::FormatNumber(prof_loop_seconds, 3) + " s (" + pct(prof_loop_seconds) + ")"));

            if (hydro_forces) {
                auto hp = hydro_forces->GetProfileStats();
                double hydro_total = hp.hydrostatics_seconds + hp.radiation_seconds + hp.excitation_seconds;
                double chrono_solver = std::max(0.0, prof_loop_seconds - hydro_total);

                auto loop_pct = [&](double s){ return seastack::infra::FormatNumber(100.0 * (s / std::max(1e-12, prof_loop_seconds)), 1) + "%"; };
                prof_lines.push_back(seastack::infra::cli::CreateAlignedLine("   🔧", "Chrono Solver", seastack::infra::FormatNumber(chrono_solver, 4) + " s  (" + loop_pct(chrono_solver) + ")"));
                prof_lines.push_back(seastack::infra::cli::CreateAlignedLine("   ⚓", "Hydrostatics", seastack::infra::FormatNumber(hp.hydrostatics_seconds, 4) + " s  (" + loop_pct(hp.hydrostatics_seconds) + ")  [" + std::to_string(hp.hydrostatics_calls) + " calls]"));
                prof_lines.push_back(seastack::infra::cli::CreateAlignedLine("   💧", "Radiation Damping", seastack::infra::FormatNumber(hp.radiation_seconds, 4) + " s  (" + loop_pct(hp.radiation_seconds) + ")  [" + std::to_string(hp.radiation_calls) + " calls]"));
                prof_lines.push_back(seastack::infra::cli::CreateAlignedLine("   🌊", "Wave Forces", seastack::infra::FormatNumber(hp.excitation_seconds, 4) + " s  (" + loop_pct(hp.excitation_seconds) + ")  [" + std::to_string(hp.excitation_calls) + " calls]"));
            }

            if (exporter) {
                prof_lines.push_back(seastack::infra::cli::CreateAlignedLine("💾", "Export", seastack::infra::FormatNumber(prof_export_seconds, 3) + " s (" + pct(prof_export_seconds) + ")"));
            }
            prof_lines.push_back(seastack::infra::cli::CreateAlignedLine("━━━", "━━━━━━━━━━━━━━━━━━━━━━", "━━━━━━━━━━━━━━━━━━━━"));
            prof_lines.push_back(seastack::infra::cli::CreateAlignedLine("📈", "Total Runtime", seastack::infra::FormatNumber(wall_seconds, 3) + " s (100%)"));
            seastack::infra::cli::ShowSectionBox("🔬 Performance Profiling", prof_lines);
        }

        seastack::infra::cli::DisplayWarnings();

        if (show_system_banner) {
            const double sim_elapsed = system->GetChTime() - initial_time;
            const double rtf = (wall_s > 1e-9) ? (sim_elapsed / wall_s) : -1.0;
            seastack::infra::cli::ShowSimulationResults(system->GetChTime(), step_count, wall_s, rtf,
                                                        result.primary_artifact_path, result.artifact_note,
                                                        config.cli_log_file_path);
        }

        // Populate result
        result.exit_code = diverged ? 2 : 0;
        result.wall_time_s = wall_s;
        result.sim_time_final = system->GetChTime();
        result.diverged = diverged;

        // Extract per-PTO metrics from the exporter's running integrals
        if (exporter) {
            double sim_dur = system->GetChTime() - initial_time;
            auto pto_summaries = exporter->GetPtoSummary(sim_dur);
            for (const auto& ps : pto_summaries) {
                PtoMetrics pm;
                pm.name                   = ps.name;
                pm.final_absorbed_energy_J = ps.final_energy_J;
                pm.mean_absorbed_power_W   = ps.mean_power_W;
                result.pto_metrics.push_back(std::move(pm));
                result.total_energy_J     += ps.final_energy_J;
                result.total_mean_power_W += ps.mean_power_W;
            }
        }

        if (config.capture_pto_total_timeseries && exporter && result.exit_code == 0 && !diverged) {
            exporter->GetTotalPtoPowerSeries(result.pto_total_time_s, result.pto_total_power_W);
        }

        // Clean up temp exporter directory (metrics already extracted above)
        if (using_temp_exporter && !resolved_output_dir.empty()) {
            exporter.reset();
            std::error_code ec;
            std::filesystem::remove_all(resolved_output_dir, ec);
        }

        return result;

    } catch (const std::exception& e) {
        seastack::infra::cli::LogError(std::string("Unhandled exception: ") + e.what());
        result.exit_code = 1;
        result.error_message = std::string("Unhandled exception: ") + e.what();
        return result;
    } catch (...) {
        seastack::infra::cli::LogError("Unknown fatal error during simulation.");
        result.exit_code = 1;
        result.error_message = "Unknown fatal error during simulation.";
        return result;
    }
}

}  // namespace seastack::app
