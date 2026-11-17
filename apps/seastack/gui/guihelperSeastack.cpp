// =============================================================================
// SEA-Stack GUI with SEA-Stack's own viewer (--viewer=seastack)
// =============================================================================
// Suppress MSVC warning about IMGUI_API macro redefinition in vsgImGui headers.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4005)  // macro redefinition
#endif

#include "guihelper_impl.h"

#include "radiated_wave_overlay.h"
#include "seastack_viewer.h"
#include "vsg_analysis_overlays.h"
#include "vsg_config.h"
#include "vsg_gui_component.h"
#include "vsg_materials.h"
#include "vsg_mooring_lines.h"
#include "vsg_theme.h"
#include "vsg_water_surface.h"

#include <vsgImGui/imgui.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <chrono/core/ChDataPath.h>

#include <seastack/adapters/chrono/runtime_controls.h>
#include <seastack/viz_core/frame_series.h>
#include <seastack/viz_core/scene_picking.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace seastack::viz {

using namespace vsg_config;

GUIImplSeastack::GUIImplSeastack()
    : water_(std::make_unique<AnimatedWaterSurface>()),
      radiated_waves_(std::make_unique<RadiatedWaveOverlay>()),
      viewer_(std::make_unique<SeastackViewer>()),
      viewer_settings_(std::make_unique<ViewerSettings>()),
      analysis_(std::make_unique<AnalysisOverlays>()) {}

GUIImplSeastack::~GUIImplSeastack() = default;

void GUIImplSeastack::Init(UI& ui, ::chrono::ChSystem* system, const char* title) {
    ui_ = &ui;
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
    InitViewer(ui, title);
}

void GUIImplSeastack::InitReplay(UI& ui, std::unique_ptr<seastack::chrono::ReplayRecording> recording,
                                 const char* title) {
    if (!recording || recording->frames.empty()) {
        throw std::runtime_error("Replay: the recording has no frames.");
    }
    replay_ = std::move(recording);
    if (!replay_->viewer_colors) {
        // Headless runs record the colours as authored; paint them as the GUI would have.
        for (size_t b = 0; b < replay_->scene.bodies.size(); ++b) {
            for (VisualShape& shape : replay_->scene.bodies[b].shapes) {
                ApplyBodyPalette(static_cast<int>(b), options_.use_model_colors, &shape);
            }
        }
    }
    model_bounds_ = replay_->scene.model_bounds;
    wave_model_ = replay_->wave;

    history_frame_bytes_ = EstimateFrameBytes(replay_->frames.front());
    history_ = std::make_unique<FrameHistory>(replay_->frames.size());
    for (VizFrame& frame : replay_->frames) {
        history_->Push(std::move(frame));
    }
    replay_->frames.clear();
    replay_->frames.shrink_to_fit();

    TimelineState& tl = viewer_settings_->timeline;
    tl.available = true;
    tl.replay = true;
    tl.playback = true;
    tl.playing = false;
    tl.playback_time = replay_->input_frame.time;
    ui.simulationStarted = false;
    ui.runStatus.finished = true;
    ui.runStatus.end_time = history_->Newest().time;
    InitViewer(ui, title);
}

const SceneDescription& GUIImplSeastack::Scene() const {
    return replay_ ? replay_->scene : extractor_->Scene();
}

void GUIImplSeastack::InitViewer(UI& ui, const char* title) {
    ui_ = &ui;
    viewer_settings_->theme = options_.theme;
    viewer_settings_->theme_from_model = options_.theme_from_model;
    viewer_settings_->theme_changed = true;
    mooring_viz_ = std::make_unique<MooringLinesViz>();
    gui_ = std::make_unique<SeastackGuiComponent>(nullptr, ui.simulationStarted, ui.playbackSpeed, ui.runStatus,
                                                  viewer_settings_.get(), mooring_viz_.get());

    SeastackViewer::Options options;
    options.title = title ? title : "SEA-Stack";
    options.width = kWindowWidth;
    options.height = kWindowHeight;
    options.fov_deg = kCameraFovDeg;
    options.font_path = ::chrono::GetChronoDataFile("vsg/fonts/open-sans/OpenSans-Regular.ttf");
    options.skybox_path = ::chrono::GetChronoDataFile("vsg/textures/vsg_skybox.ktx");  // as the Chrono viewer
    auto draw_gui = [this]() {
        gui_->Draw();
        HandlePicking();
        DrawBodyLabels();
        const ModelState& model = viewer_settings_->model;
        if (history_ && !viewer_settings_->capturing_frame) {
            DrawSelection();
            analysis_->DrawSceneOverlays(Scene(), *history_, last_frame_, *viewer_, gui_->SceneRect(),
                                         model_bounds_.Radius(), viewer_->UiScale(), model,
                                         &viewer_settings_->analysis);
        }
        if (history_) {
            analysis_->DrawPlots(Scene(), *history_, last_frame_, gui_->SceneRect(), viewer_->UiScale(),
                                 model.selected, &viewer_settings_->analysis);
        }
    };
    if (!viewer_->Initialize(Scene(), options, draw_gui)) {
        throw std::runtime_error("SEA-Stack viewer initialization failed. Try --viewer=chrono or --nogui.");
    }
    gui_->SetUiScale(viewer_->UiScale());
    mooring_group_ = vsg::Group::create();
    lines_switch_ = vsg::Switch::create();
    lines_switch_->addChild(true, mooring_group_);
    viewer_->OverlayScene()->addChild(lines_switch_);  // before the water is added

    ModelState& model = viewer_settings_->model;
    for (const BodyDescription& body : Scene().bodies) {
        model.body_names.push_back(body.name);
        model.body_fixed.push_back(body.fixed ? 1 : 0);
    }
    const size_t n_bodies = Scene().bodies.size();
    model.body_visible.assign(n_bodies, 1);
    model.body_label.assign(n_bodies, 0);
    model.body_axes.assign(n_bodies, 0);
    model.body_forces.assign(n_bodies, 0);
    model.body_trail.assign(n_bodies, 0);

    viewer_->SetCameraPose(
        FrameBounds(model_bounds_, CameraView::kIsometric, kCameraFovDeg, viewer_->GetViewportAspect()));
    reframe_pending_ = true;
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
    reframe_pending_ = false;  // keep the camera the model asked for
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
        ChooseWaterResolution(wave_model_.get(), viewer_settings_.get());
        water_->Initialize(viewer_.get(), viewer_settings_->grid_resolution, viewer_settings_.get());
    }
    if (!water_->IsInitialized()) {
        return;
    }
    ChooseWaterResolution(wave_model_.get(), viewer_settings_.get());
    ApplyWaterSettings(water_.get(), viewer_settings_.get());
    UpdateWaterReadouts(*water_, wave_model_.get(), viewer_settings_.get());
    water_->SetRadiationField(radiated_waves_->Update(viewer_settings_.get(), Scene(), frame.time,
                                                      water_->NeedsUpdate(frame.time, viewer_settings_.get())));
    water_->Update(wave_model_, frame.time, viewer_settings_.get());
}

void GUIImplSeastack::SetMooringVisualizationRadii(double line_radius_m, double endpoint_radius_m,
                                                   double node_marker_radius_m) {
    mooring_line_radius_m_ = line_radius_m;
    mooring_endpoint_radius_m_ = endpoint_radius_m;
    mooring_node_marker_radius_m_ = node_marker_radius_m;
}

void GUIImplSeastack::UpdateMooring(const VizFrame& frame) {
    MooringVizOptions mooring_options;
    mooring_options.line_radius_m = mooring_line_radius_m_;
    mooring_options.endpoint_radius_m = mooring_endpoint_radius_m_;
    mooring_options.node_marker_radius_m = mooring_node_marker_radius_m_;
    mooring_options.has_tension_range = options_.has_mooring_tension_range;
    mooring_options.tension_min = options_.mooring_tension_min;
    mooring_options.tension_max = options_.mooring_tension_max;
    DrawMooringLines(mooring_viz_.get(), viewer_.get(), mooring_group_, Scene(), frame, model_bounds_,
                     mooring_options, viewer_settings_->show_mooring_colors);
    if (!tsda_segment_viz_) {
        tsda_segment_viz_ = std::make_unique<MooringLinesViz>();
        tsda_barrel_viz_ = std::make_unique<MooringLinesViz>();
        tsda_rod_viz_ = std::make_unique<MooringLinesViz>();
        tsda_spring_viz_ = std::make_unique<MooringLinesViz>();
    }
    // SEGMENT = thin cable; SPRING = hydraulic ram (barrel + rod + spring).
    TsdaGlyphRenderers glyphs;
    glyphs.segments = tsda_segment_viz_.get();
    glyphs.barrels = tsda_barrel_viz_.get();
    glyphs.rods = tsda_rod_viz_.get();
    glyphs.springs = tsda_spring_viz_.get();
    DrawTsdaCables(glyphs, viewer_.get(), mooring_group_, Scene(), frame);
}

void GUIImplSeastack::SetMooringLineProvider(MooringVizProvider provider) {
    mooring_provider_ = std::move(provider);
    if (extractor_) {
        extractor_->SetMooringProvider(mooring_provider_);
    }
}

void GUIImplSeastack::SetHydroForces(seastack::chrono::HydroSystem* hydro) {
    if (extractor_) {
        extractor_->SetForceProvider(seastack::chrono::MakeHydroForceProvider(hydro));
    }
    radiated_waves_->SetHydro(hydro);
}

void GUIImplSeastack::AfterStep(double sim_time) { radiated_waves_->RecordStep(sim_time); }

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
    ModelState& model = viewer_settings_->model;
    if (model.frame_selected_requested) {
        model.frame_selected_requested = false;
        if (model.selected >= 0) {
            const Bounds3 bounds = BodyWorldBounds(static_cast<size_t>(model.selected), last_frame_);
            if (bounds.IsValid()) {
                viewer_->SetCameraPose(FrameBounds(bounds, viewer_->GetViewDirection(), kCameraFovDeg, aspect),
                                       kViewTransition);
            }
        }
    }
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

void GUIImplSeastack::UpdateSceneViewport() {
    // The layout comes from the previous frame's Draw(), so it lags by one frame.
    const ScreenRect rect = gui_->SceneRect();
    if (rect.w < 1.0f || rect.h < 1.0f) {
        return;  // no frame drawn yet
    }
    viewer_->SetSceneViewport(static_cast<int>(std::lround(rect.x)), static_cast<int>(std::lround(rect.y)),
                              static_cast<int>(std::lround(rect.w)), static_cast<int>(std::lround(rect.h)));
    if (reframe_pending_) {
        reframe_pending_ = false;
        viewer_->SetCameraPose(FrameBounds(model_bounds_, viewer_->GetViewDirection(), kCameraFovDeg,
                                           viewer_->GetViewportAspect()));
    }
}

void GUIImplSeastack::DrawBodyLabels() {
    const std::vector<char>& labelled = viewer_settings_->model.body_label;
    if (std::find(labelled.begin(), labelled.end(), 1) == labelled.end() || (!extractor_ && !replay_)) {
        return;
    }
    const SceneDescription& scene = Scene();
    if (last_frame_.bodies.size() != scene.bodies.size()) {
        return;
    }
    // Background list: labels sit over the 3D view but under the panels.
    const ScreenRect rect = gui_->SceneRect();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->PushClipRect(ImVec2(rect.x, rect.y), ImVec2(rect.x + rect.w, rect.y + rect.h), true);
    const ImU32 text_color = IM_COL32(255, 255, 255, 255);
    const ImU32 shadow_color = IM_COL32(0, 0, 0, 200);
    const float marker_r = 3.0f * viewer_->UiScale();
    const std::vector<char>& visible = viewer_settings_->model.body_visible;
    for (size_t i = 0; i < scene.bodies.size(); ++i) {
        const std::string& name = scene.bodies[i].name;
        float x = 0.0f;
        float y = 0.0f;
        if ((i < visible.size() && !visible[i]) || i >= labelled.size() || !labelled[i]) {
            continue;
        }
        if (name.empty() || !viewer_->ProjectToWindow(last_frame_.bodies[i].com_position, &x, &y)) {
            continue;
        }
        const ImVec2 size = ImGui::CalcTextSize(name.c_str());
        const ImVec2 pos(x - 0.5f * size.x, y - size.y - 2.0f * marker_r);
        dl->AddCircleFilled(ImVec2(x, y), marker_r, text_color);
        dl->AddText(ImVec2(pos.x + 1.0f, pos.y + 1.0f), shadow_color, name.c_str());
        dl->AddText(pos, text_color, name.c_str());
    }
    dl->PopClipRect();
}

Bounds3 GUIImplSeastack::BodyWorldBounds(size_t i, const VizFrame& frame) const {
    Bounds3 world;
    if (i >= frame.bodies.size() || i >= Scene().bodies.size()) {
        return world;
    }
    const BodyState& state = frame.bodies[i];
    world.Extend(state.com_position);
    const Bounds3& b = Scene().bodies[i].local_bounds;
    if (b.IsValid()) {
        for (int c = 0; c < 8; ++c) {
            world.Extend(Transform(state.pose, {(c & 1) ? b.max[0] : b.min[0], (c & 2) ? b.max[1] : b.min[1],
                                                (c & 4) ? b.max[2] : b.min[2]}));
        }
    }
    return world;
}

void GUIImplSeastack::HandlePicking() {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse || viewer_settings_->capturing_frame) {
        return;
    }
    const ScreenRect rect = gui_->SceneRect();
    const ImVec2 p = io.MousePos;
    if (p.x < rect.x || p.y < rect.y || p.x > rect.x + rect.w || p.y > rect.y + rect.h) {
        return;
    }
    // A click, not the end of a camera drag.
    const float click_px = 4.0f * viewer_->UiScale();
    const bool double_click = ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
    const bool click = ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
                       io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < click_px * click_px;
    if (!click && !double_click) {
        return;
    }
    Ray ray;
    if (!viewer_->WindowRay(p.x, p.y, &ray)) {
        return;
    }
    ModelState& model = viewer_settings_->model;
    const int hit = PickBody(Scene(), last_frame_, ray, model.body_visible);
    if (double_click) {
        if (hit >= 0) {
            model.selected = hit;
            model.frame_selected_requested = true;
        }
    } else {
        model.selected = hit;  // empty space clears the selection
    }
}

void GUIImplSeastack::DrawSelection() {
    const ModelState& model = viewer_settings_->model;
    const SceneDescription& scene = Scene();
    const int i = model.selected;
    if (i < 0 || i >= static_cast<int>(scene.bodies.size()) || i >= static_cast<int>(last_frame_.bodies.size()) ||
        (i < static_cast<int>(model.body_visible.size()) && !model.body_visible[i])) {
        return;
    }
    const Bounds3& b = scene.bodies[i].local_bounds;
    if (!b.IsValid()) {
        return;
    }
    // Box around the body's shapes, in its own frame.
    std::array<ImVec2, 8> corners;
    for (int c = 0; c < 8; ++c) {
        const Vec3 world = Transform(last_frame_.bodies[i].pose, {(c & 1) ? b.max[0] : b.min[0],
                                                                  (c & 2) ? b.max[1] : b.min[1],
                                                                  (c & 4) ? b.max[2] : b.min[2]});
        if (!viewer_->ProjectToWindow(world, &corners[c].x, &corners[c].y)) {
            return;
        }
    }
    const ScreenRect rect = gui_->SceneRect();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->PushClipRect(ImVec2(rect.x, rect.y), ImVec2(rect.x + rect.w, rect.y + rect.h), true);
    const ImU32 color = IM_COL32(255, 200, 40, 230);
    const float thickness = 1.5f * viewer_->UiScale();
    for (int c = 0; c < 8; ++c) {
        for (int bit : {1, 2, 4}) {
            if (!(c & bit)) {
                dl->AddLine(corners[c], corners[c | bit], color, thickness);
            }
        }
    }
    const std::string& name = scene.bodies[i].name;
    float x = 0.0f, y = 0.0f;
    const bool labelled = i < static_cast<int>(model.body_label.size()) && model.body_label[i];
    if (!labelled && !name.empty() &&
        viewer_->ProjectToWindow(last_frame_.bodies[i].com_position, &x, &y)) {
        const ImVec2 size = ImGui::CalcTextSize(name.c_str());
        dl->AddText(ImVec2(x - 0.5f * size.x + 1.0f, y - size.y + 1.0f), IM_COL32(0, 0, 0, 200), name.c_str());
        dl->AddText(ImVec2(x - 0.5f * size.x, y - size.y), color, name.c_str());
    }
    dl->PopClipRect();
}

void GUIImplSeastack::SyncModel(const VizFrame& frame) {
    ModelState& model = viewer_settings_->model;
    const SceneDescription& scene = Scene();
    model.has_lines = !scene.mooring_lines.empty() || !scene.cable_lines.empty();
    for (size_t i = 0; i < model.body_visible.size(); ++i) {
        viewer_->SetBodyVisible(i, model.body_visible[i] != 0);
    }
    for (size_t i = 0; i < model.body_axes.size(); ++i) {
        viewer_->SetBodyFrameVisible(i, model.body_axes[i] != 0);
    }
    lines_switch_->setAllChildren(model.lines_visible);
    model.selected_wrenches.clear();
    if (model.selected >= 0 && model.selected < static_cast<int>(frame.bodies.size())) {
        const size_t b = static_cast<size_t>(model.selected);
        const BodyState& s = frame.bodies[b];
        model.selected_com = s.com_position;
        model.selected_velocity = s.linear_velocity;
        model.selected_rotation = CardanAnglesXYZ(s.pose.rotation);
        for (double& a : model.selected_rotation) {
            a *= 180.0 / 3.14159265358979323846;
        }
        const size_t n_sources = scene.force_sources.size();
        if (n_sources > 0 && frame.body_wrenches.size() == n_sources * frame.bodies.size()) {
            for (size_t src = 0; src < n_sources; ++src) {
                model.selected_wrenches.push_back(BodyWrench(frame, src, b));
            }
            model.selected_wrenches.push_back(TotalWrench(frame, b));
        }
    }
}

void GUIImplSeastack::HandleScreenshot() {
    // Report the result of last frame's capture, then queue a new request.
    if (!viewer_->LastScreenshotPath().empty() && viewer_->LastScreenshotPath() != reported_screenshot_) {
        reported_screenshot_ = viewer_->LastScreenshotPath();
        viewer_settings_->screenshot_status =
            viewer_->LastScreenshotOk()
                ? "Saved " + std::filesystem::path(reported_screenshot_).filename().string()
                : std::string("Screenshot failed (see console)");
        viewer_settings_->screenshot_status_time = -1.0;
    }
    viewer_settings_->capturing_frame = viewer_settings_->screenshot_requested;
    if (viewer_settings_->screenshot_requested) {
        viewer_settings_->screenshot_requested = false;
        viewer_->RequestScreenshot(NextScreenshotPath(options_.screenshot_directory));
    }
}

void GUIImplSeastack::SetRuntimeControls(seastack::chrono::RuntimeControls* controls,
                                         const std::string& schedule_path, double time_step) {
    runtime_controls_ = controls;
    schedule_path_ = schedule_path;
    time_step_ = time_step;
}

void GUIImplSeastack::SyncParameters() {
    ParametersPanelState& panel = viewer_settings_->parameters;
    panel.available = (runtime_controls_ != nullptr) && !replay_;
    if (!panel.available) {
        return;
    }
    seastack::chrono::RuntimeControls& controls = *runtime_controls_;

    // Requests from the last GUI frame.
    for (const auto& [id, value] : panel.requests) {
        std::string error;
        if (!controls.Request(id, value, &error)) {
            panel.status = "Not applied: " + error;
        } else {
            panel.status.clear();
        }
    }
    panel.requests.clear();
    if (std::string refusal = controls.TakeRefusal(); !refusal.empty()) {
        panel.status = refusal;
    }
    if (panel.export_requested) {
        panel.export_requested = false;
        try {
            seastack::chrono::WriteRuntimeSchedule(schedule_path_, controls.events(), time_step_,
                                                   "Exported from the GUI during the run");
            panel.status = "Wrote " + std::filesystem::path(schedule_path_).filename().string();
        } catch (const std::exception& e) {
            panel.status = std::string("Export failed: ") + e.what();
        }
    }

    panel.schedule_path = schedule_path_;
    panel.run_finished = ui_->runStatus.finished;
    panel.sim_time = ui_->runStatus.sim_time;
    panel.rows.clear();
    for (const auto& p : controls.parameters()) {
        ParameterRow row;
        row.id = p.id;
        row.element = p.element;
        row.link_name = p.link_name;
        row.field = p.field;
        if (p.element == "tsda" || p.element == "rsda") {
            row.link = (p.element == "tsda" ? "TSDA " : "RSDA ") + p.link_name;
        } else {
            row.link = p.element == "waves" ? "Waves" : p.element;
        }
        row.effective_time = p.effective_time;
        row.label = p.label;
        row.units = p.units;
        row.source = p.source;
        row.owner = p.owner;
        row.value = p.value;
        row.default_value = p.default_value;
        row.min_value = p.min_value;
        row.pending = p.pending;
        row.pending_value = p.pending_value;
        panel.rows.push_back(std::move(row));
    }
    // One line per change; a group edit (same step, field and value on several
    // links) is shown as one line.
    const auto& events = controls.events();
    if (events.size() == shown_event_count_) {
        return;
    }
    shown_event_count_ = events.size();
    panel.event_lines.clear();
    for (size_t i = 0; i < events.size();) {
        const auto& e = events[i];
        const seastack::chrono::RuntimeParameter* p = controls.Find(e.command.parameter);
        size_t n = 1;
        bool same_previous = true;
        while (p != nullptr && i + n < events.size()) {
            const auto& next = events[i + n];
            const seastack::chrono::RuntimeParameter* q = controls.Find(next.command.parameter);
            if (q == nullptr || next.command.step != e.command.step || next.command.value != e.command.value ||
                q->element != p->element || q->field != p->field) {
                break;
            }
            same_previous = same_previous && (next.previous_value == e.previous_value);
            ++n;
        }
        std::string name = e.command.parameter;
        if (p != nullptr && p->link_name.empty()) {
            name = (p->element == "waves" ? "Waves" : p->element) + " " + p->label;
        } else if (p != nullptr) {
            name = (n == 1 ? p->link_name : std::to_string(n) + (p->element == "tsda" ? " TSDAs" : " RSDAs")) +
                   " " + p->label;
        }
        char previous[32] = "mixed";
        if (same_previous) {
            std::snprintf(previous, sizeof(previous), "%.6g", e.previous_value);
        }
        char line[256];
        std::snprintf(line, sizeof(line), "t = %.2f s  %s: %s -> %.6g %s", e.command.time, name.c_str(), previous,
                      e.command.value, p ? p->units.c_str() : "");
        panel.event_lines.emplace_back(line);
        i += n;
    }
}

void GUIImplSeastack::RecordFrame(const VizFrame& live) {
    // Memory budget for the timeline; at about 3 kB per frame (10 bodies,
    // 4 lines x 40 nodes) this keeps roughly 85k frames.
    constexpr size_t kHistoryBudgetBytes = size_t{256} << 20;
    constexpr size_t kMaxHistoryFrames = 200000;
    const size_t bytes = EstimateFrameBytes(live);
    // Mooring data can appear after the first frames; size the buffer again
    // (dropping the few frames recorded so far) if frames grew noticeably.
    if (!history_ || bytes > history_frame_bytes_ + history_frame_bytes_ / 4) {
        history_frame_bytes_ = bytes;
        history_ = std::make_unique<FrameHistory>(
            FrameCapacityForBudget(bytes, kHistoryBudgetBytes, kMaxHistoryFrames));
    }
    // Same time = nothing new (paused, or slow motion between steps); an
    // earlier time restarts the history (see FrameHistory::Push).
    if (history_->Empty() || live.time != history_->Newest().time) {
        history_->Push(live);
    }
}

VizFrame GUIImplSeastack::SelectFrame(VizFrame live) {
    constexpr double kSameTime = 1e-9;  // [s]
    TimelineState& tl = viewer_settings_->timeline;
    const FrameHistory& h = *history_;
    tl.available = true;
    // A replay starts at the input pose, before the first recorded step.
    tl.start_time = replay_ ? std::min(replay_->input_frame.time, h.Oldest().time) : h.Oldest().time;
    tl.end_time = h.Newest().time;
    tl.frame_count = h.Size();
    tl.memory_mb = static_cast<double>(h.Size() * history_frame_bytes_) / (1024.0 * 1024.0);
    tl.interpolated = false;
    tl.smooth_slow_motion_active = false;
    tl.showing_input_pose = false;

    const auto now = std::chrono::steady_clock::now();
    const double wall_dt =
        has_last_select_wall_ ? std::chrono::duration<double>(now - last_select_wall_).count() : 0.0;
    last_select_wall_ = now;
    has_last_select_wall_ = true;

    if (tl.playback) {
        if (tl.playing) {
            constexpr double kMaxPlaybackSpeed = 20.0;  // "Max" speed for recorded frames [-]
            const double speed = ui_->playbackSpeed > 0.0 ? ui_->playbackSpeed : kMaxPlaybackSpeed;
            tl.playback_time += speed * wall_dt;
        }
        if (tl.step_request != 0) {
            // Previous / next recorded frame strictly before / after the shown time.
            size_t i = h.NearestIndex(tl.playback_time);
            const bool before_i = h.At(i).time >= tl.playback_time - kSameTime;
            if (tl.step_request < 0 && before_i && i == 0 && tl.start_time < h.At(0).time) {
                tl.playback_time = tl.start_time;  // back from the first sample to the input pose
            } else {
                if (tl.step_request < 0 && before_i && i > 0) {
                    --i;
                } else if (tl.step_request > 0 && h.At(i).time <= tl.playback_time + kSameTime &&
                           i + 1 < h.Size()) {
                    ++i;
                }
                tl.playback_time = h.At(i).time;
            }
            tl.step_request = 0;
        }
        if (tl.playback_time >= tl.end_time && tl.playing) {
            tl.playing = false;
            if (!tl.replay) {
                // Played up to the live state: carry on with the simulation.
                tl.playback = false;
                ui_->simulationStarted = !ui_->runStatus.finished;
            }
        }
        tl.playback_time = std::clamp(tl.playback_time, tl.start_time, tl.end_time);
        ui_->animatingWithoutStepping = tl.playback && tl.playing;
        if (tl.playback) {
            if (replay_ && tl.playback_time < h.Oldest().time - kSameTime) {
                // Results start after the first step (SIM-160): show the input
                // pose as it is, labelled, rather than interpolating into it.
                tl.showing_input_pose = true;
                return replay_->input_frame;
            }
            VizFrame shown;
            tl.interpolated = h.Sample(tl.playback_time, &shown);
            last_live_display_time_ = live.time;
            return shown;
        }
    }

    ui_->animatingWithoutStepping = false;
    // Live. In slow motion, draw the time between the last two steps that the
    // run loop asks for, never moving backwards (e.g. just after resuming).
    const double display_time = ui_->runStatus.display_time;
    if (last_live_display_time_ > live.time) {
        last_live_display_time_ = live.time;  // history restarted
    }
    if (tl.smooth_slow_motion && std::isfinite(display_time) && h.Size() >= 2) {
        const double t = std::clamp(display_time, std::max(last_live_display_time_, h.At(h.Size() - 2).time),
                                    h.Newest().time);
        last_live_display_time_ = t;
        VizFrame shown;
        h.Sample(t, &shown);
        tl.smooth_slow_motion_active = true;
        return shown;
    }
    last_live_display_time_ = live.time;
    return live;
}

bool GUIImplSeastack::IsRunning(double /*timestep*/) {
    VizFrame live;  // a replay has no live state; it is always in playback
    if (!replay_) {
        if (!extractor_) {
            return false;
        }
        live = extractor_->Capture(0);
        RecordFrame(live);
    }
    last_frame_ = SelectFrame(std::move(live));
    if (replay_) {
        ui_->runStatus.sim_time = viewer_settings_->timeline.playback_time;
    }
    const VizFrame& frame = last_frame_;
    viewer_->SetFrame(frame);
    SyncModel(frame);
    HandleScreenshot();
    SyncParameters();
    UpdateMooring(frame);
    UpdateWater(frame);
    UpdateSceneViewport();
    HandleCameraRequests();
    UpdateClipPlanes();  // the trackball zoom changes the view distance
    if (viewer_settings_->theme_changed) {
        viewer_->ApplySceneTheme(GetSceneTheme(viewer_settings_->theme));
        viewer_settings_->theme_changed = false;
    }
    return viewer_->RenderFrame();
}

}  // namespace seastack::viz
