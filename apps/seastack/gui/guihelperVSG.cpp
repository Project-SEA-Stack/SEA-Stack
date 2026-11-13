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
#include "vsg_theme.h"
#include "vsg_water_surface.h"

using namespace seastack::hydro;

#include <iostream>
#include <memory>
#include <stdexcept>

#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/core/ChTypes.h>

#include <seastack/viz_core/mesh_normals.h>

#include <array>
#include <vector>

namespace seastack::viz {

using namespace vsg_config;

namespace {

bool IsWaterSurfaceBody(const ::chrono::ChBody& body) {
    return body.GetName() == "water_surface" || body.GetName() == "animated_water_surface";
}

/// World-frame bounds of all body visual shapes [m] (water surfaces excluded).
/// Bodies without visual shapes contribute their reference-frame origin.
Bounds3 ComputeModelBounds(const ::chrono::ChSystem& system) {
    Bounds3 bounds;
    for (const auto& body : system.GetBodies()) {
        if (!body || IsWaterSurfaceBody(*body)) {
            continue;
        }
        const ::chrono::ChVector3d p = body->GetPos();
        bounds.Extend(Vec3{p.x(), p.y(), p.z()});
        if (auto model = body->GetVisualModel()) {
            const ::chrono::ChAABB local = model->GetBoundingBox();
            if (!local.IsInverted()) {
                const ::chrono::ChAABB world = local.Transform(body->GetVisualModelFrame());
                bounds.Extend(Vec3{world.min.x(), world.min.y(), world.min.z()});
                bounds.Extend(Vec3{world.max.x(), world.max.y(), world.max.z()});
            }
        }
    }
    return bounds;
}

/// Crease angle [deg] for display normals: smoother below, sharp edge above.
constexpr double kCreaseAngleDeg = 40.0;

/// Give triangle-mesh visual shapes crease-angle smooth normals when the mesh
/// has none (e.g. coarse BEM panel meshes, which otherwise shade faceted).
/// Display only: vertex positions are untouched. Returns meshes smoothed.
int SmoothBodyMeshNormals(::chrono::ChBody& body) {
    auto model = body.GetVisualModel();
    if (!model) {
        return 0;
    }
    int smoothed = 0;
    for (const auto& instance : model->GetShapeInstances()) {
        auto tri = std::dynamic_pointer_cast<::chrono::ChVisualShapeTriangleMesh>(instance.shape);
        if (!tri || !tri->GetMesh()) {
            continue;
        }
        auto mesh = tri->GetMesh();
        const auto& faces = mesh->GetIndicesVertexes();
        if (faces.empty() || mesh->GetIndicesNormals().size() == faces.size()) {
            continue;  // keep authored normals
        }
        std::vector<std::array<double, 3>> verts;
        verts.reserve(mesh->GetCoordsVertices().size());
        for (const auto& v : mesh->GetCoordsVertices()) {
            verts.push_back({v.x(), v.y(), v.z()});
        }
        std::vector<std::array<int, 3>> tris;
        tris.reserve(faces.size());
        for (const auto& f : faces) {
            tris.push_back({f.x(), f.y(), f.z()});
        }
        const CornerNormals normals = ComputeCreaseNormals(verts, tris, kCreaseAngleDeg);
        auto& out_n = mesh->GetCoordsNormals();
        auto& out_ni = mesh->GetIndicesNormals();
        out_n.clear();
        out_ni.clear();
        for (const auto& n : normals.normals) {
            out_n.emplace_back(n[0], n[1], n[2]);
        }
        for (const auto& fi : normals.face_normal_indices) {
            out_ni.emplace_back(fi[0], fi[1], fi[2]);
        }
        ++smoothed;
    }
    return smoothed;
}

}  // namespace

// =============================================================================
// GUIImplVSG Implementation
// =============================================================================

GUIImplVSG::GUIImplVSG()
    : pVis(::chrono_types::make_shared<SeastackVisualSystemVSG>()),
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
//   7. AddStudioLights, CaptureSkyNode — require scene graph from step 6
// ─────────────────────────────────────────────────────────────────────────────
void GUIImplVSG::Init(UI& ui, ::chrono::ChSystem* system, const char* title) {
    system_ = system;

    std::cerr << "[VSG] Stage 1: AttachSystem" << std::endl;
    pVis->AttachSystem(system);

    pVis->SetWindowTitle(title);
    pVis->SetWindowSize(kWindowWidth, kWindowHeight);
    pVis->SetWindowPosition(100, 100);

    // Default camera: isometric view framing the whole model, so lab-scale and
    // full-scale models both fill the view. SetCamera() overrides this.
    model_bounds_ = system ? ComputeModelBounds(*system) : Bounds3{};
    if (model_bounds_.IsValid()) {
        const Vec3 s = model_bounds_.Size();
        std::cout << "[Camera] Model bounds: " << s[0] << " x " << s[1] << " x " << s[2] << " m" << std::endl;
    }
    const CameraPose pose = FrameBounds(model_bounds_, CameraView::kIsometric, kCameraFovDeg,
                                        static_cast<double>(kWindowWidth) / kWindowHeight);
    pVis->AddCamera(::chrono::ChVector3d(pose.eye[0], pose.eye[1], pose.eye[2]),
                    ::chrono::ChVector3d(pose.target[0], pose.target[1], pose.target[2]));
    pVis->SetCameraVertical(::chrono::CameraVerticalDir::Z);
    pVis->SetCameraAngleDeg(kCameraFovDeg);
    // Chrono's panel duplicates the playback HUD and its wall clock includes
    // pauses; hidden by default, re-enabled from Display > Chrono debug panel.
    pVis->SetBaseGuiVisibility(false);

    pVis->SetLightIntensity(kKeyIntensity);
    pVis->SetLightDirection(kKeyAzimuth, kKeyElevation);
    // Shadows stay off: Chrono VSG's shadow pipeline crashes when nodes (our
    // water surface and mooring lines) are added after Initialize().
    pVis->EnableShadows(false);
    if (viewer_settings_) {
        viewer_settings_->theme = options_.theme;
        viewer_settings_->theme_changed = true;
    }

    // The skybox is always loaded; the theme shows or hides it at runtime.
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
            if (body && !IsWaterSurfaceBody(*body)) {
                SmoothBodyMeshNormals(*body);
                const auto material = MakeBodyPaletteMaterial(body_index);
                ApplyMaterialToAllVisualShapes(*body, material, options_.use_model_colors);
                ++body_index;

                if (kEnableWireframe) {
                    auto model = body->GetVisualModel();
                    if (model) {
                        model->EnableWireframe(true);
                    }
                }
            }
        }
    }

    if (!mooring_viz_)
        mooring_viz_ = std::make_unique<MooringLinesViz>();

    pVis->AddGuiComponent(std::make_shared<SeastackGuiComponent>(
        pVis.get(), ui.simulationStarted, ui.playbackSpeed, ui.runStatus,
        viewer_settings_.get(), mooring_viz_.get()));

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

    std::cerr << "[VSG] Stage 4: studio lights + scene ready" << std::endl;
    AddStudioLights(pVis.get());
    pVis->CaptureSkyNode();
    UpdateClipPlanes();
}

void GUIImplVSG::SetCamera(double x, double y, double z, double dirx, double diry, double dirz) {
    // (dirx, diry, dirz) is the target point [m], not a direction.
    CameraPose pose;
    pose.eye = {x, y, z};
    pose.target = {dirx, diry, dirz};
    const double dx = x - dirx, dy = y - diry;
    if (dx * dx + dy * dy < 1e-12) {
        pose.up = {0.0, 1.0, 0.0};  // looking straight down or up
    }
    pVis->SetCameraPose(pose);
    UpdateClipPlanes();
}

void GUIImplVSG::UpdateClipPlanes() {
    const auto eye = pVis->GetCameraPosition();
    const auto target = pVis->GetCameraTarget();
    const double view_distance =
        std::max((eye - target).Length(),
                 FramingDistance(model_bounds_.Radius(), kCameraFovDeg, pVis->GetViewportAspect()));
    double water_extent = 0.0;
    if (viewer_settings_) {
        water_extent = std::max(viewer_settings_->grid_width, viewer_settings_->grid_length);
    }
    pVis->SetClipPlanes(ComputeClipPlanes(view_distance, water_extent));
}

void GUIImplVSG::HandleCameraRequests() {
    if (!viewer_settings_) {
        return;
    }
    constexpr double kViewTransition = 0.5;  // [s] animated move between views
    const double aspect = pVis->GetViewportAspect();
    if (viewer_settings_->camera_view_requested) {
        viewer_settings_->camera_view_requested = false;
        viewer_settings_->camera_fit_requested = false;
        pVis->SetCameraPose(FrameBounds(model_bounds_, viewer_settings_->camera_view, kCameraFovDeg, aspect),
                            kViewTransition);
    } else if (viewer_settings_->camera_fit_requested) {
        viewer_settings_->camera_fit_requested = false;
        pVis->SetCameraPose(FrameBounds(model_bounds_, pVis->GetViewDirection(), kCameraFovDeg, aspect),
                            kViewTransition);
    }
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

    // Auto-size the water patch from the model bounds and the wavelength when
    // no manual extent has been set via SetWaterGridExtent(), so lab-scale and
    // full-scale models both get a sensible patch.
    if (viewer_settings_ &&
        viewer_settings_->grid_width <= 0.0 && viewer_settings_->grid_length <= 0.0 &&
        model_bounds_.IsValid()) {
        double wavelength = 0.0;
        if (wave_model_ && wave_model_->GetWaveMode() != WaveMode::kNoWave) {
            wavelength = DeepWaterWavelength(wave_model_->GetCharacteristicPeriod(), wave_model_->GetGravity());
        }
        const WaterExtent w = ComputeWaterExtent(model_bounds_, wavelength);
        viewer_settings_->grid_width    = w.width;
        viewer_settings_->grid_length   = w.length;
        viewer_settings_->grid_center_x = w.center_x;
        viewer_settings_->grid_center_y = w.center_y;
        std::cout << "[WaterSurface] Auto-sized grid: "
                  << w.width << " x " << w.length << " m, center: ("
                  << w.center_x << ", " << w.center_y << ")" << std::endl;
        UpdateClipPlanes();
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
                // Model-scaled defaults; explicit YAML radii (>= 0) override them.
                const MooringVizRadii defaults = DefaultMooringVizRadii(ModelLengthScale(model_bounds_));
                mooring_viz_->SetVisualizationRadii(defaults.line, defaults.endpoint, defaults.node_marker);
                mooring_viz_->SetVisualizationRadii(mooring_viz_line_radius_request_,
                                                   mooring_viz_endpoint_radius_request_,
                                                   mooring_viz_node_marker_radius_request_);
                mooring_viz_->Initialize(pVis.get(), line_data, mooring_scene_group_);
                if (options_.has_mooring_tension_range) {
                    auto& range = mooring_viz_->TensionRange();
                    if (range.SetFixedRange(options_.mooring_tension_min, options_.mooring_tension_max)) {
                        range.SetMode(ColorRangeMode::kFixed);
                    }
                }
            }

            const bool color_on = viewer_settings_ && viewer_settings_->show_mooring_colors;
            mooring_viz_->Update(line_data, color_on);
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

    HandleCameraRequests();

    if (viewer_settings_ && viewer_settings_->theme_changed) {
        pVis->ApplySceneTheme(GetSceneTheme(viewer_settings_->theme), kFillLightName, kRimLightName);
        viewer_settings_->theme_changed = false;
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
        const ::chrono::ChVector3d ang_vel = body->GetAngVelParent();  // world frame

        // Estimate body radius from AABB (rough approximation).
        double radius = 5.0;  // Default
        auto aabb = body->GetTotalAABB();
        if (aabb.max.x() > aabb.min.x()) {
            double dx = aabb.max.x() - aabb.min.x();
            double dy = aabb.max.y() - aabb.min.y();
            radius = std::max(dx, dy) / 2.0;
            // Guard against degenerate AABBs only; a larger floor would
            // over-amplify lab-scale bodies.
            radius = std::max(radius, 0.05);  // m
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
