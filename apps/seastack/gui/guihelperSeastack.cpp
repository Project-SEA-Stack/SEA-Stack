// =============================================================================
// SEA-Stack GUI with SEA-Stack's own viewer (--viewer=seastack)
// =============================================================================
#include "guihelper_impl.h"

#include "seastack_viewer.h"
#include "vsg_config.h"
#include "vsg_gui_component.h"
#include "vsg_materials.h"
#include "vsg_theme.h"
#include "vsg_water_surface.h"

#include <chrono/core/ChDataPath.h>

#include <iostream>
#include <stdexcept>

namespace seastack::viz {

using namespace vsg_config;

GUIImplSeastack::GUIImplSeastack()
    : water_(std::make_unique<AnimatedWaterSurface>()),
      viewer_(std::make_unique<SeastackViewer>()),
      viewer_settings_(std::make_unique<ViewerSettings>()) {}

GUIImplSeastack::~GUIImplSeastack() = default;

void GUIImplSeastack::Init(UI& ui, ::chrono::ChSystem* system, const char* title) {
    // Same palette as the Chrono viewer, applied to the Chrono assets so the
    // extracted scene carries it.
    if (system) {
        int body_index = 0;
        for (auto& body : system->GetBodies()) {
            if (body && !seastack::chrono::IsWaterSurfaceBody(*body)) {
                ApplyMaterialToAllVisualShapes(*body, MakeBodyPaletteMaterial(body_index), options_.use_model_colors);
                ++body_index;
            }
        }
    }
    extractor_ = std::make_unique<seastack::chrono::VizExtractor>(system, mooring_provider_);
    model_bounds_ = extractor_->Scene().model_bounds;

    viewer_settings_->theme = options_.theme;
    viewer_settings_->theme_changed = true;
    gui_ = std::make_unique<SeastackGuiComponent>(nullptr, ui.simulationStarted, ui.playbackSpeed, ui.runStatus,
                                                  viewer_settings_.get(), nullptr);

    SeastackViewer::Options options;
    options.title = title ? title : "SEA-Stack";
    options.width = kWindowWidth;
    options.height = kWindowHeight;
    options.fov_deg = kCameraFovDeg;
    options.font_path = ::chrono::GetChronoDataFile("vsg/fonts/Ubuntu_Mono/UbuntuMono-Regular.ttf");
    SeastackGuiComponent* gui = gui_.get();
    if (!viewer_->Initialize(extractor_->Scene(), options, [gui]() { gui->Draw(); })) {
        throw std::runtime_error("SEA-Stack viewer initialization failed. Try --viewer=chrono or --nogui.");
    }
    std::cout << "[SeastackViewer] " << extractor_->Scene().bodies.size()
              << " bodies; mooring lines are not drawn by this viewer yet." << std::endl;

    viewer_->SetCameraPose(
        FrameBounds(model_bounds_, CameraView::kIsometric, kCameraFovDeg, viewer_->GetViewportAspect()));
    UpdateClipPlanes();
}

void GUIImplSeastack::SetCamera(double x, double y, double z, double dirx, double diry, double dirz) {
    // (dirx, diry, dirz) is the target point [m], not a direction.
    CameraPose pose;
    pose.eye = {x, y, z};
    pose.target = {dirx, diry, dirz};
    const double dx = x - dirx, dy = y - diry;
    if (dx * dx + dy * dy < 1e-12) {
        pose.up = {0.0, 1.0, 0.0};  // looking straight down or up
    }
    viewer_->SetCameraPose(pose);
    UpdateClipPlanes();
}

void GUIImplSeastack::SetWaterGridExtent(double width, double length, double center_x, double center_y) {
    viewer_settings_->grid_width = width;
    viewer_settings_->grid_length = length;
    viewer_settings_->grid_center_x = center_x;
    viewer_settings_->grid_center_y = center_y;
    viewer_settings_->grid_extent_changed = true;
    std::cout << "[WaterSurface] Grid extent: " << width << " x " << length << " m"
              << ", center: (" << center_x << ", " << center_y << ")" << std::endl;
}

void GUIImplSeastack::UpdateWater(const VizFrame& frame) {
    if (!water_tried_) {
        // One attempt: a failure is logged by the viewer and not retried every frame.
        water_tried_ = true;
        AutoSizeWaterGrid(viewer_settings_.get(), model_bounds_, wave_model_.get());
        water_->Initialize(viewer_.get(), viewer_settings_->grid_resolution, viewer_settings_.get());
    }
    if (!water_->IsInitialized()) {
        return;
    }
    ApplyWaterSettings(water_.get(), viewer_settings_.get());
    if (viewer_settings_->show_radiation_viz) {
        UpdateRadiationSources(water_.get(), *viewer_settings_, wave_model_.get(), extractor_->Scene(), frame);
    }
    water_->Update(wave_model_, frame.time, viewer_settings_.get());
}

void GUIImplSeastack::SetMooringLineProvider(MooringVizProvider provider) {
    mooring_provider_ = std::move(provider);
    if (extractor_) {
        extractor_->SetMooringProvider(mooring_provider_);
    }
}

void GUIImplSeastack::UpdateClipPlanes() {
    const double view_distance =
        std::max(viewer_->GetViewDistance(),
                 FramingDistance(model_bounds_.Radius(), kCameraFovDeg, viewer_->GetViewportAspect()));
    const double water_extent = std::max(viewer_settings_->grid_width, viewer_settings_->grid_length);
    viewer_->SetClipPlanes(ComputeClipPlanes(view_distance, water_extent));
}

void GUIImplSeastack::HandleCameraRequests() {
    constexpr double kViewTransition = 0.5;  // [s] animated move between views
    const double aspect = viewer_->GetViewportAspect();
    if (viewer_settings_->camera_view_requested) {
        viewer_settings_->camera_view_requested = false;
        viewer_settings_->camera_fit_requested = false;
        viewer_->SetCameraPose(FrameBounds(model_bounds_, viewer_settings_->camera_view, kCameraFovDeg, aspect),
                               kViewTransition);
    } else if (viewer_settings_->camera_fit_requested) {
        viewer_settings_->camera_fit_requested = false;
        viewer_->SetCameraPose(FrameBounds(model_bounds_, viewer_->GetViewDirection(), kCameraFovDeg, aspect),
                               kViewTransition);
    }
}

bool GUIImplSeastack::IsRunning(double /*timestep*/) {
    if (!extractor_) {
        return false;
    }
    const VizFrame frame = extractor_->Capture(0);
    viewer_->SetFrame(frame);
    UpdateWater(frame);
    HandleCameraRequests();
    UpdateClipPlanes();  // the trackball zoom changes the view distance
    if (viewer_settings_->theme_changed) {
        viewer_->ApplySceneTheme(GetSceneTheme(viewer_settings_->theme));
        viewer_settings_->theme_changed = false;
    }
    return viewer_->RenderFrame();
}

}  // namespace seastack::viz
