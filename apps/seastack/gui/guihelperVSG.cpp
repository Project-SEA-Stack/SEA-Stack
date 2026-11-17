// =============================================================================
// SEA-Stack VSG GUI Implementation
// =============================================================================
// Main implementation of GUIImplVSG - the VSG visualization backend.
// This file coordinates the various VSG subsystems (materials, lighting,
// water surface, GUI overlay) to provide the complete visualization.
// =============================================================================
#include "guihelper_impl.h"

#include "radiated_wave_overlay.h"
#include "vsg_config.h"
#include "vsg_gui_component.h"
#include "vsg_lighting.h"
#include "vsg_materials.h"
#include "vsg_mooring_lines.h"
#include "vsg_theme.h"
#include "vsg_water_surface.h"

using namespace seastack::hydro;

#include <iostream>
#include <memory>
#include <stdexcept>

#include <chrono/assets/ChVisualMaterial.h>
#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/core/ChTypes.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
#include <chrono_vsg/utils/ChShapeBuilderVSG.h>

#include <seastack/viz_core/mesh_normals.h>

#include <array>
#include <vector>

namespace seastack::viz {

using namespace vsg_config;

namespace {

using seastack::chrono::IsWaterSurfaceBody;

/// Overlay nodes built with Chrono's shape builder, in Chrono's scene.
class ChronoOverlayBuilder : public OverlayBuilder {
  public:
    explicit ChronoOverlayBuilder(::chrono::vsg3d::ChVisualSystemVSG* vis) : vis_(vis) {}

    vsg::ref_ptr<vsg::Group> OverlayScene() override { return vis_->GetVSGScene(); }

    vsg::ref_ptr<vsg::Node> CreateOverlayMesh(const TriangleMesh& mesh, const OverlayMaterial& material) override {
        auto shape_builder = vis_->GetVSGShapeBuilder();
        if (!shape_builder) {
            return {};
        }
        std::vector<::chrono::ChVisualMaterialSharedPtr> materials = {ToChronoMaterial(material)};
        return shape_builder->CreateTrimeshPbrMatShape(ToChronoMesh(mesh), vsg::MatrixTransform::create(), materials,
                                                       /*double_faced=*/false, /*wireframe=*/false);
    }

    vsg::ref_ptr<vsg::Node> CreateIndexedOverlayMesh(const TriangleMesh& mesh, const Rgba& color) override {
        auto shape_builder = vis_->GetVSGShapeBuilder();
        if (!shape_builder) {
            return {};
        }
        return shape_builder->CreateTrimeshColAvgShape(ToChronoMesh(mesh), vsg::MatrixTransform::create(),
                                                       ::chrono::ChColor(color[0], color[1], color[2]),
                                                       /*double_faced=*/false, /*wireframe=*/false);
    }

    vsg::ref_ptr<vsg::Node> CreateSphereOverlay(const OverlayMaterial& material,
                                                vsg::ref_ptr<vsg::MatrixTransform> transform) override {
        auto shape_builder = vis_->GetVSGShapeBuilder();
        if (!shape_builder) {
            return {};
        }
        using ST = ::chrono::vsg3d::ShapeBuilder::ShapeType;
        return shape_builder->CreatePbrShape(ST::SPHERE, ToChronoMaterial(material), transform,
                                             /*double_faced=*/false, /*wireframe=*/false);
    }

  private:
    static std::shared_ptr<::chrono::ChTriangleMeshConnected> ToChronoMesh(const TriangleMesh& mesh) {
        auto ch_mesh = ::chrono_types::make_shared<::chrono::ChTriangleMeshConnected>();
        for (const Vec3& v : mesh.vertices) {
            ch_mesh->m_vertices.emplace_back(v[0], v[1], v[2]);
        }
        for (const auto& f : mesh.faces) {
            ch_mesh->m_face_v_indices.emplace_back(f[0], f[1], f[2]);
        }
        return ch_mesh;
    }

    static std::shared_ptr<::chrono::ChVisualMaterial> ToChronoMaterial(const OverlayMaterial& material) {
        auto ch_material = ::chrono_types::make_shared<::chrono::ChVisualMaterial>();
        ch_material->SetDiffuseColor(::chrono::ChColor(material.color[0], material.color[1], material.color[2]));
        ch_material->SetOpacity(material.color[3]);
        ch_material->SetRoughness(material.roughness);
        ch_material->SetMetallic(material.metallic);
        return ch_material;
    }

    ::chrono::vsg3d::ChVisualSystemVSG* vis_;
};

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
      viewer_settings_(std::make_unique<ViewerSettings>()),
      radiated_waves_(std::make_unique<RadiatedWaveOverlay>()) {}

GUIImplVSG::~GUIImplVSG() = default;

void GUIImplVSG::SetHydroForces(seastack::chrono::HydroSystem* hydro) { radiated_waves_->SetHydro(hydro); }

void GUIImplVSG::AfterStep(double sim_time) { radiated_waves_->RecordStep(sim_time); }

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

    // Built after the materials so the scene description records what is drawn.
    extractor_ = std::make_unique<seastack::chrono::VizExtractor>(system, mooring_provider_);

    // Default camera: isometric view framing the whole model, so lab-scale and
    // full-scale models both fill the view. SetCamera() overrides this.
    model_bounds_ = extractor_->Scene().model_bounds;
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
    pVis->HideLogo();  // SEA-Stack owns the chrome; Chrono's watermark is noise

    pVis->SetLightIntensity(kKeyIntensity);
    pVis->SetLightDirection(kKeyAzimuth, kKeyElevation);
    // Shadows stay off: Chrono VSG's shadow pipeline crashes when nodes (our
    // water surface and mooring lines) are added after Initialize().
    pVis->EnableShadows(false);
    if (viewer_settings_) {
        viewer_settings_->theme = options_.theme;
        viewer_settings_->theme_from_model = options_.theme_from_model;
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
    overlay_builder_ = std::make_unique<ChronoOverlayBuilder>(pVis.get());
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
    if (extractor_) {
        extractor_->SetMooringProvider(mooring_provider_);
    }
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
    // Require both system and overlay builder to be valid.
    if (!system_ || !overlay_builder_) {
        if constexpr (kDebugWaveSurface) {
            static bool printed_once = false;
            if (!printed_once) {
                std::cout << "[WaterSurface] EnsureWaterSurface early return: system_="
                          << (system_ ? "ok" : "null") << " overlay_builder_="
                          << (overlay_builder_ ? "ok" : "null") << std::endl;
                printed_once = true;
            }
        }
        return;
    }

    // If already initialized for this viewer, nothing to do.
    if (animated_water_->IsInitializedFor(overlay_builder_.get())) {
        return;
    }

    if constexpr (kDebugWaveSurface) {
        bool has_waves = wave_model_ && wave_model_->GetWaveMode() != WaveMode::kNoWave;
        std::cout << "[WaterSurface] EnsureWaterSurface: has_waves=" << has_waves << std::endl;
    }

    // Auto-size the water patch from the model bounds and the wavelength when
    // no manual extent has been set via SetWaterGridExtent(), so lab-scale and
    // full-scale models both get a sensible patch.
    if (AutoSizeWaterGrid(viewer_settings_.get(), model_bounds_, wave_model_.get())) {
        UpdateClipPlanes();
    }

    // Always create animated water surface (supports waves + radiation viz).
    // Static plane fallback removed - animated water handles all cases.
    ChooseWaterResolution(wave_model_.get(), viewer_settings_.get());
    int resolution = (viewer_settings_) ? viewer_settings_->grid_resolution : 0;
    animated_water_->Initialize(overlay_builder_.get(), resolution, viewer_settings_.get());

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

    // One snapshot per rendered frame; everything below draws from it.
    const VizFrame frame = extractor_ ? extractor_->Capture(0) : VizFrame{};

    // Mooring line visualisation (lazy-initialised on first valid data).
    if (extractor_ && mooring_viz_) {
        MooringVizOptions mooring_options;
        mooring_options.line_radius_m = mooring_viz_line_radius_request_;
        mooring_options.endpoint_radius_m = mooring_viz_endpoint_radius_request_;
        mooring_options.node_marker_radius_m = mooring_viz_node_marker_radius_request_;
        mooring_options.has_tension_range = options_.has_mooring_tension_range;
        mooring_options.tension_min = options_.mooring_tension_min;
        mooring_options.tension_max = options_.mooring_tension_max;
        const bool color_on = viewer_settings_ && viewer_settings_->show_mooring_colors;
        DrawMooringLines(mooring_viz_.get(), overlay_builder_.get(), mooring_scene_group_, extractor_->Scene(),
                         frame, model_bounds_, mooring_options, color_on);
    }

    ChooseWaterResolution(wave_model_.get(), viewer_settings_.get());
    ApplyWaterSettings(animated_water_.get(), viewer_settings_.get());
    if (animated_water_) {
        UpdateWaterReadouts(*animated_water_, wave_model_.get(), viewer_settings_.get());
    }

    // Update animated water surface if wave model is set (or forced static).
    // VSG vertex buffers are marked dirty() in Update() for GPU re-upload.
    if (animated_water_ && animated_water_->IsInitialized()) {
        const double t = frame.time;

        if (viewer_settings_ && extractor_) {
            animated_water_->SetRadiationField(radiated_waves_->Update(
                viewer_settings_.get(), extractor_->Scene(), t,
                animated_water_->NeedsUpdate(t, viewer_settings_.get())));
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

}  // namespace seastack::viz
