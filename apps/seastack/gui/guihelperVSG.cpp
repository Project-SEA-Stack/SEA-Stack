// =============================================================================
// SEA-Stack VSG GUI Implementation
// =============================================================================
// Main implementation of GUIImplVSG - the VSG visualization backend.
// This file coordinates the various VSG subsystems (materials, lighting,
// water surface, GUI overlay) to provide the complete visualization.
// =============================================================================
#include "guihelper_impl.h"

#include "vsg_config.h"
#include "vsg_gui_component.h"
#include "vsg_lighting.h"
#include "vsg_materials.h"
#include "vsg_mooring_lines.h"
#include "vsg_radiation_surface.h"
#include "vsg_water_surface.h"

using namespace seastack::hydro;

#include <iostream>
#include <memory>
#include <stdexcept>

#include <chrono/core/ChTypes.h>
#include <chrono/assets/ChVisualMaterial.h>
#include <chrono/assets/ChVisualModel.h>
#include <chrono/physics/ChBody.h>

namespace seastack::viz {

using namespace vsg_config;

namespace {
// True if any visual shape on this body carries a translucent material (opacity
// < 1). Such bodies (e.g. a see-through hull set via the YAML visualization
// `opacity` field) are left untouched by the opaque scene-paint pass below.
bool BodyHasTranslucentMaterial(const ::chrono::ChBody& body) {
    auto model = body.GetVisualModel();
    if (!model) {
        return false;
    }
    for (unsigned int i = 0; i < model->GetNumShapes(); ++i) {
        auto shape = model->GetShape(i);
        if (!shape) {
            continue;
        }
        for (unsigned int m = 0; m < shape->GetNumMaterials(); ++m) {
            auto mat = shape->GetMaterial(m);
            if (mat && mat->GetOpacity() < 0.999f) {
                return true;
            }
        }
    }
    return false;
}
}  // namespace

// =============================================================================
// GUIImplVSG Implementation
// =============================================================================

GUIImplVSG::GUIImplVSG()
    : GUIImplVSG(::chrono_types::make_shared<::chrono::vsg3d::ChVisualSystemVSG>()) {}

GUIImplVSG::GUIImplVSG(std::shared_ptr<::chrono::vsg3d::ChVisualSystemVSG> vis)
    : pVis(std::move(vis)),
      animated_water_(std::make_unique<AnimatedWaterSurface>()),
      viewer_settings_(std::make_unique<ViewerSettings>()) {}

GUIImplVSG::~GUIImplVSG() = default;

// ─────────────────────────────────────────────────────────────────────────────
// Init sequence (must be single-threaded, called after Chrono system is fully
// populated with bodies, joints, and visual assets):
//   1. AttachSystem  — registers body list with the viewer
//   2. Window / camera / light setup — all pre-Initialize configuration
//   3. EnableSkyTexture — loads cubemap from CHRONO_DATA_DIR (can fail if
//      the data path is missing; non-fatal but produces blank sky)
//   4. Apply materials — iterates system bodies (must exist already)
//   5. AddGuiComponent — registers ImGui overlay
//   6. pVis->Initialize() — creates Vulkan device, swap chain, compiles
//      shaders, builds scene graph.
//   7. AddFillLight — requires scene graph from step 6
// ─────────────────────────────────────────────────────────────────────────────
void GUIImplVSG::Init(UI& ui, ::chrono::ChSystem* system, const char* title) {
    system_ = system;

    std::cerr << "[VSG] Stage 1: AttachSystem" << std::endl;
    pVis->AttachSystem(system);

    pVis->SetWindowTitle(title);
    pVis->SetWindowSize(1280, 720);
    pVis->SetWindowPosition(100, 100);

    const ::chrono::ChVector3d eye(0.0, -kCameraDistance, kCameraHeight);
    const ::chrono::ChVector3d target(0.0, 0.0, -10.0);
    pVis->AddCamera(eye, target);
    pVis->SetCameraVertical(::chrono::CameraVerticalDir::Z);
    pVis->SetCameraAngleDeg(40.0);

    pVis->SetLightIntensity(kKeyIntensity);
    pVis->SetLightDirection(kKeyAzimuth, kKeyElevation);

    std::cerr << "[VSG] Stage 2: EnableSkyTexture" << std::endl;
    try {
        pVis->EnableSkyTexture(::chrono::SkyMode::BOX);
    } catch (const std::exception& e) {
        std::cerr << "[VSG] WARNING: Skybox texture load failed: " << e.what()
                  << " (continuing without skybox)" << std::endl;
    } catch (...) {
        std::cerr << "[VSG] WARNING: Skybox texture load failed (unknown error)" << std::endl;
    }

    if (system) {
        int body_index = 0;
        for (auto& body : system->GetBodies()) {
            if (!body || !ShouldApplyScenePaint(*body)) {
                continue;
            }
            // Preserve a YAML-specified translucent hull (opacity < 1): the opaque
            // painted-metal material would otherwise overwrite its transparency.
            if (BodyHasTranslucentMaterial(*body)) {
                continue;
            }
            const auto material = MakePaintedMetalVariant(body_index);
            ApplyMaterialToAllVisualShapes(*body, material);
            ++body_index;

            if (kEnableWireframe) {
                auto model = body->GetVisualModel();
                if (model) {
                    model->EnableWireframe(true);
                }
            }
        }
    }

    if (!mooring_viz_)
        mooring_viz_ = std::make_unique<MooringLinesViz>();

    pVis->AddGuiComponent(std::make_shared<SeastackGuiComponent>(
        pVis.get(), ui.simulationStarted, viewer_settings_.get(),
        mooring_viz_.get()));

    std::cerr << "[VSG] Stage 3: pVis->Initialize() (Vulkan device + scene compile)" << std::endl;
    pVis->Initialize();

    // Validate that Initialize() produced a usable scene graph.
    auto scene = pVis->GetVSGScene();
    if (!scene) {
        std::cerr << "[VSG] FATAL: pVis->Initialize() completed but GetVSGScene() is null"
                  << std::endl;
        throw std::runtime_error(
            "VSG initialization failed: scene graph is null after Initialize(). "
            "This may be caused by GPU driver issues or missing Vulkan support. "
            "Try running with --nogui.");
    }

    std::cerr << "[VSG] Stage 4: AddFillLight + scene ready" << std::endl;
    AddFillLight(pVis.get());
}

void GUIImplVSG::SetCamera(double x, double y, double z, double dirx, double diry, double dirz) {
    pVis->SetCameraPosition({x, y, z});
    pVis->SetCameraTarget({dirx, diry, dirz});
}

bool GUIImplVSG::ShouldApplyScenePaint(const ::chrono::ChBody& body) const {
    const std::string& name = body.GetName();
    return name != "water_surface" && name != "animated_water_surface";
}

void GUIImplVSG::SetWaveModel(std::shared_ptr<seastack::hydro::WaveBase> wave) {
    wave_model_ = wave;
    // Water surface will be created/updated on first render via EnsureWaterSurface().
}

void GUIImplVSG::SetWaterGridExtent(double width, double length, double center_x, double center_y) {
    if (!viewer_settings_) {
        return;
    }

    viewer_settings_->grid_width = width;
    viewer_settings_->grid_length = length;
    viewer_settings_->grid_center_x = center_x;
    viewer_settings_->grid_center_y = center_y;
    viewer_settings_->grid_extent_changed = true;

    std::cout << "[WaterSurface] Grid extent: " << width << " x " << length << " m"
              << ", center: (" << center_x << ", " << center_y << ")" << std::endl;
}

void GUIImplVSG::SetMooringLineProvider(MooringVizProvider provider) {
    mooring_provider_ = std::move(provider);
}

void GUIImplVSG::SetMooringVisualizationRadii(double line_radius_m,
                                              double endpoint_radius_m,
                                              double node_marker_radius_m) {
    mooring_viz_line_radius_request_ = line_radius_m;
    mooring_viz_endpoint_radius_request_ = endpoint_radius_m;
    mooring_viz_node_marker_radius_request_ = node_marker_radius_m;
    if (mooring_viz_) {
        mooring_viz_->SetVisualizationRadii(line_radius_m, endpoint_radius_m, node_marker_radius_m);
    }
}

void GUIImplVSG::EnsureWaterSurface() {
    // Require both system and visual system to be valid.
    if (!system_ || !pVis) {
        if constexpr (kDebugWaveSurface) {
            static bool printed_once = false;
            if (!printed_once) {
                std::cout << "[WaterSurface] EnsureWaterSurface early return: system_="
                          << (system_ ? "ok" : "null") << " pVis=" << (pVis ? "ok" : "null") << std::endl;
                printed_once = true;
            }
        }
        return;
    }

    // If already initialized for this visual system, nothing to do.
    if (animated_water_->IsInitializedFor(pVis.get())) {
        return;
    }

    if constexpr (kDebugWaveSurface) {
        bool has_waves = wave_model_ && wave_model_->GetWaveMode() != WaveMode::kNoWave;
        std::cout << "[WaterSurface] EnsureWaterSurface: has_waves=" << has_waves << std::endl;
    }

    // Auto-compute grid extent from body bounding box when no manual extent
    // has been set via SetWaterGridExtent(). This ensures the free surface
    // covers the full model regardless of whether the simulation was set up
    // via YAML or the C++ API.
    if (viewer_settings_ &&
        viewer_settings_->grid_width <= 0.0 && viewer_settings_->grid_length <= 0.0) {
        double xmin = +1e20, xmax = -1e20;
        double ymin = +1e20, ymax = -1e20;
        for (const auto& body : system_->GetBodies()) {
            auto pos = body->GetPos();
            xmin = std::min(xmin, pos.x());
            xmax = std::max(xmax, pos.x());
            ymin = std::min(ymin, pos.y());
            ymax = std::max(ymax, pos.y());
        }
        if (xmax > xmin) {
            double span_x = xmax - xmin;
            double span_y = ymax - ymin;
            double width  = std::max(kWaterGridSize, span_x * 1.5);
            double length = std::max(kWaterGridSize, span_y * 1.5);
            viewer_settings_->grid_width    = width;
            viewer_settings_->grid_length   = length;
            viewer_settings_->grid_center_x = (xmin + xmax) / 2.0;
            viewer_settings_->grid_center_y = (ymin + ymax) / 2.0;
            std::cout << "[WaterSurface] Auto-sized grid: "
                      << width << " x " << length << " m, center: ("
                      << viewer_settings_->grid_center_x << ", "
                      << viewer_settings_->grid_center_y << ")" << std::endl;
        }
    }

    // Always create animated water surface (supports waves + radiation viz).
    // Static plane fallback removed - animated water handles all cases.
    int resolution = (viewer_settings_) ? viewer_settings_->grid_resolution : 0;
    animated_water_->Initialize(pVis.get(), resolution, viewer_settings_.get());

    if (animated_water_->IsInitialized()) {
        std::cout << "[WaterSurface] wave_ptr=" << (wave_model_ ? "ok" : "null");
        if (wave_model_) {
            std::cout << " mode=" << static_cast<int>(wave_model_->GetWaveMode());
        }
        std::cout << " " << animated_water_->GetStatusString() << std::endl;

        // Initial update at current system time.
        double t = system_->GetChTime();
        animated_water_->Update(wave_model_, t);
    } else {
        if constexpr (kDebugWaveSurface) {
            std::cout << "[WaterSurface] Initialize() failed!" << std::endl;
        }
    }
}

bool GUIImplVSG::IsRunning(double timestep) {
    if (pVis->Run() == false) {
        return false;
    }

    // Reserve a scene-graph group for mooring geometry before the water
    // surface is created, so opaque mooring lines are rendered first and
    // remain visible through the transparent free surface.
    if (mooring_provider_ && !mooring_scene_group_) {
        if (auto scene = pVis->GetVSGScene()) {
            mooring_scene_group_ = vsg::Group::create();
            scene->addChild(mooring_scene_group_);
        }
    }

    EnsureWaterSurface();

    // Mooring line visualisation (lazy-initialised on first valid data).
    if (mooring_provider_) {
        auto line_data = mooring_provider_();
        if (!line_data.empty()) {
            if (!mooring_viz_)
                mooring_viz_ = std::make_unique<MooringLinesViz>();
            if (!mooring_viz_->IsInitializedFor(pVis.get())) {
                mooring_viz_->SetVisualizationRadii(mooring_viz_line_radius_request_,
                                                   mooring_viz_endpoint_radius_request_,
                                                   mooring_viz_node_marker_radius_request_);
                mooring_viz_->Initialize(pVis.get(), line_data, mooring_scene_group_);
            }

            const bool color_on  = viewer_settings_ && viewer_settings_->show_mooring_colors;
            const bool range_lock = viewer_settings_ && viewer_settings_->mooring_range_locked;
            mooring_viz_->Update(line_data, color_on, range_lock);
        }
    }

    // Handle viewer settings changes.
    if (viewer_settings_ && animated_water_) {
        // Handle visibility toggle.
        animated_water_->SetVisible(viewer_settings_->show_water);

        // Handle resolution or extent change (requires mesh rebuild).
        if ((viewer_settings_->resolution_changed || viewer_settings_->grid_extent_changed) &&
            animated_water_->IsInitialized()) {
            animated_water_->Reinitialize(viewer_settings_->grid_resolution, viewer_settings_.get());
            viewer_settings_->resolution_changed = false;
            viewer_settings_->grid_extent_changed = false;
        }
    }

    // Update animated water surface if wave model is set (or forced static).
    // VSG vertex buffers are marked dirty() in Update() for GPU re-upload.
    if (animated_water_ && animated_water_->IsInitialized()) {
        double t = system_ ? system_->GetChTime() : 0.0;

        // Update radiation source body state if radiation viz is enabled.
        // Chooses the first non-water body in the system as the source.
        // NOTE: This is visualization-only and does NOT affect physics.
        if (viewer_settings_ && viewer_settings_->show_radiation_viz && system_) {
            UpdateRadiationSourceBody(t);
        }

        // Update() handles null wave_model_ gracefully (keeps surface flat).
        // Pass viewer_settings_ for scale multiplier and throttle.
        animated_water_->Update(wave_model_, t, viewer_settings_.get());
    }

    pVis->BeginScene();
    pVis->Render();
    pVis->EndScene();

    return true;
}

void GUIImplVSG::UpdateRadiationSourceBody(double t) {
    if (!animated_water_ || !system_) {
        return;
    }

    // Update global params.
    if (viewer_settings_) {
        RadiationSurfaceViz::Params rad_params;
        rad_params.visual_scale = static_cast<double>(viewer_settings_->radiation_visual_scale);
        rad_params.wave_period = 8.0;  // Default; should match body oscillation frequency

        // Get wave properties from wave model if available.
        if (wave_model_) {
            // Water depth and gravity (available in all wave models).
            if (wave_model_->GetWaterDepth() > 0.0) {
                rad_params.water_depth = wave_model_->GetWaterDepth();
            }
            rad_params.gravity = wave_model_->GetGravity();

            double T_char = wave_model_->GetCharacteristicPeriod();
            if (T_char > 0.0) {
                rad_params.wave_period = T_char;
            }
        }

        animated_water_->GetRadiationViz().SetParams(rad_params);
    }

    // Log source bodies once.
    static bool logged_sources = false;

    // Iterate over ALL moving bodies and update their radiation state.
    for (auto& body : system_->GetBodies()) {
        if (!body) {
            continue;
        }

        const std::string& name = body->GetName();

        // Skip water surfaces and ground bodies.
        if (name == "water_surface" || name == "animated_water_surface" ||
            name == "ground" || name == "floor" || name.empty()) {
            continue;
        }

        // Skip fixed bodies.
        if (body->IsFixed()) {
            continue;
        }

        // Get body motion state.
        const ::chrono::ChVector3d pos = body->GetPos();
        const ::chrono::ChVector3d vel = body->GetPosDt();
        const ::chrono::ChVector3d ang_vel = body->GetAngVelLocal();

        // Estimate body radius from AABB (rough approximation).
        double radius = 5.0;  // Default
        auto aabb = body->GetTotalAABB();
        if (aabb.max.x() > aabb.min.x()) {
            double dx = aabb.max.x() - aabb.min.x();
            double dy = aabb.max.y() - aabb.min.y();
            radius = std::max(dx, dy) / 2.0;
            radius = std::max(radius, 1.0);  // Minimum 1m
        }

        // Update radiation viz for this body.
        animated_water_->GetRadiationViz().SetSourceState(name, pos, vel, ang_vel, radius, t);

        if (!logged_sources) {
            std::cout << "[RadiationViz] Source: " << name << " (r=" << radius << "m)" << std::endl;
        }
    }

    logged_sources = true;
}

}  // namespace seastack::viz
