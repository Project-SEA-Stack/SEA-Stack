// =============================================================================
// SEA-Stack Viewer - Implementation
// =============================================================================

// Suppress MSVC warning about IMGUI_API macro redefinition in vsgImGui headers.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4005)  // macro redefinition
#endif

#include "seastack_viewer.h"

#include "vsg_config.h"
#include "vsg_skybox.h"

#include <seastack/viz_core/mesh_normals.h>
#include <seastack/viz_core/primitive_meshes.h>

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/imgui_internal.h>  // FindHoveredWindowEx

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <algorithm>
#include <cmath>
#include <iostream>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace seastack::viz {

using namespace vsg_config;

namespace {

/// Crease angle [deg] for meshes without authored normals (as the Chrono viewer).
constexpr double kCreaseAngleDeg = 40.0;

/// Trackball without VSG's Space = "home view" binding; Space is play / pause.
class SeastackTrackball : public vsg::Inherit<vsg::Trackball, SeastackTrackball> {
  public:
    explicit SeastackTrackball(vsg::ref_ptr<vsg::Camera> camera) : Inherit(camera) {}
    void apply(vsg::KeyPressEvent& key_press) override {
        if (key_press.keyBase == vsg::KEY_Space) {
            return;
        }
        Inherit::apply(key_press);
    }
};

/// Mouse forwarding to ImGui. vsgImGui's handler decides from the previous
/// frame's hover state, so when the cursor jumps between panel and scene
/// within one frame a press can go to the wrong side, and a release can be
/// withheld from ImGui, which then thinks the button is held and ignores
/// every later click. Here ImGui always sees every mouse event, as its own
/// backends do; a press belongs to ImGui if it lands on a panel (last
/// frame's window rectangles) or a popup is open, and the camera gets the
/// rest. Keyboard handling is vsgImGui's.
class SeastackImGuiEvents : public vsg::Inherit<vsgImGui::SendEventsToImGui, SeastackImGuiEvents> {
  public:
    void apply(vsg::ButtonPressEvent& press) override {
        ImGuiIO& io = ImGui::GetIO();
        const ImVec2 pos(static_cast<float>(press.x), static_cast<float>(press.y));
        io.AddMousePosEvent(pos.x, pos.y);
        io.AddMouseButtonEvent(ToImGuiButton(press.button), true);
        ImGuiWindow* hovered = nullptr;
        ImGuiWindow* hovered_under_moving = nullptr;
        ImGui::FindHoveredWindowEx(pos, false, &hovered, &hovered_under_moving);
        imgui_owns_drag_ = hovered != nullptr || ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId);
        press.handled = imgui_owns_drag_;
    }
    void apply(vsg::ButtonReleaseEvent& release) override {
        ImGuiIO& io = ImGui::GetIO();
        io.AddMousePosEvent(static_cast<float>(release.x), static_cast<float>(release.y));
        io.AddMouseButtonEvent(ToImGuiButton(release.button), false);
        release.handled = imgui_owns_drag_;
        imgui_owns_drag_ = false;
    }
    void apply(vsg::MoveEvent& move) override {
        ImGuiIO& io = ImGui::GetIO();
        io.AddMousePosEvent(static_cast<float>(move.x), static_cast<float>(move.y));
        // A drag that started over the scene stays with the camera.
        const bool button_down = move.mask != vsg::BUTTON_MASK_OFF;
        move.handled = button_down ? imgui_owns_drag_ : io.WantCaptureMouse;
    }
    void apply(vsg::ScrollWheelEvent& wheel) override {
        ImGuiIO& io = ImGui::GetIO();
        io.AddMouseWheelEvent(wheel.delta.x, wheel.delta.y);
        wheel.handled = io.WantCaptureMouse;
    }

  private:
    /// VSG buttons 1 / 2 / 3 = left / middle / right; ImGui 0 / 2 / 1.
    static int ToImGuiButton(uint32_t button) {
        return button == 1 ? 0 : button == 3 ? 1 : 2;
    }

    bool imgui_owns_drag_ = false;  ///< Current press went to ImGui (not the camera)
};

/// Pipeline states for SEA-Stack meshes: both faces drawn (BEM panel meshes
/// may be open or wound inwards), optional alpha blending.
struct SetPipelineStates : public vsg::Visitor {
    bool blend = false;
    bool depth_test = true;  ///< false: draw over everything drawn before
    void apply(vsg::Object& object) override { object.traverse(*this); }
    void apply(vsg::RasterizationState& state) override { state.cullMode = VK_CULL_MODE_NONE; }
    void apply(vsg::DepthStencilState& state) override {
        state.depthTestEnable = depth_test ? VK_TRUE : VK_FALSE;
        state.depthWriteEnable = depth_test ? VK_TRUE : VK_FALSE;
    }
    void apply(vsg::ColorBlendState& state) override {
        if (!blend) {
            return;
        }
        for (auto& att : state.attachments) {
            att.blendEnable = VK_TRUE;
            att.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
            att.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
            att.colorBlendOp = VK_BLEND_OP_ADD;
            att.srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
            att.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
            att.alphaBlendOp = VK_BLEND_OP_ADD;
        }
    }
};

/// Parent <- local transform of `pose`. Filled element by element: the 16-scalar
/// dmat4 constructor (used by vsg::translate / rotate) is also exported by
/// Chrono_vsg.dll, and instantiating it here breaks the link (LNK2005).
vsg::dmat4 ToMatrix(const Pose& pose) {
    const double w = pose.rotation[0], x = pose.rotation[1], y = pose.rotation[2], z = pose.rotation[3];
    const double r[3][3] = {
        {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)},
        {2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)},
        {2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)},
    };
    vsg::dmat4 m;  // identity; m[column][row]
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            m[col][row] = r[row][col];
        }
        m[3][row] = pose.position[static_cast<size_t>(row)];
    }
    return m;
}

/// Tell Windows this process draws at full resolution (otherwise it renders
/// at 96 DPI and stretches the window, blurring text and geometry) and return
/// the display scale [-], e.g. 1.5 at 150 %. Process-wide; must run before
/// the first window is created. Returns 1 on other platforms.
float EnableDpiAwareness() {
#ifdef _WIN32
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    return static_cast<float>(GetDpiForSystem()) / 96.0f;
#else
    return 1.0f;
#endif
}

/// Axis length [m] for a body's frame triad: scaled to the body, but kept
/// between 3 % and 15 % of the model radius so tiny and huge bodies both read.
double TriadLength(const Bounds3& body_bounds, const Bounds3& model_bounds) {
    const double model_r = model_bounds.IsValid() ? std::max(model_bounds.Radius(), 0.1) : 10.0;
    const double body_r = body_bounds.IsValid() ? body_bounds.Radius() : 0.0;
    return std::clamp(0.6 * body_r, 0.03 * model_r, 0.15 * model_r);
}

float SrgbToLinear(float c) {
    return (c <= 0.04045f) ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
}

/// Directional light shining from (azimuth, elevation) [rad] in the Z-up world
/// frame; VSG directions point from the light into the scene.
vsg::ref_ptr<vsg::DirectionalLight> MakeDirectionalLight(const char* name, double azimuth, double elevation,
                                                         float intensity, const vsg::vec3& color) {
    const double se = std::sin(elevation);
    const double ce = std::cos(elevation);
    auto light = vsg::DirectionalLight::create();
    light->name = name;
    light->color = color;
    light->intensity = intensity;
    light->direction.set(static_cast<float>(-ce * std::cos(azimuth)), static_cast<float>(-ce * std::sin(azimuth)),
                         static_cast<float>(-se));
    return light;
}

}  // namespace

bool SeastackViewer::Initialize(const SceneDescription& scene, const Options& options, GuiCallback draw_gui) {
    ui_scale_ = EnableDpiAwareness();
    auto traits = vsg::WindowTraits::create();
    traits->windowTitle = options.title;
    traits->width = static_cast<uint32_t>(options.width * ui_scale_);
    traits->height = static_cast<uint32_t>(options.height * ui_scale_);
    traits->x = 100;
    traits->y = 100;
    traits->samples = VK_SAMPLE_COUNT_4_BIT;                                 // 4x MSAA
    traits->swapchainPreferences.presentMode = VK_PRESENT_MODE_FIFO_KHR;     // vsync

    window_ = vsg::Window::create(traits);
    if (!window_) {
        std::cerr << "[SeastackViewer] Could not create window." << std::endl;
        return false;
    }
    viewer_ = vsg::Viewer::create();
    viewer_->addWindow(window_);

    // ---- Scene: sky, lights and bodies ----
    scene_root_ = vsg::Group::create();
    sky_switch_ = vsg::Switch::create();
    if (!options.skybox_path.empty()) {
        if (auto sky = CreateSkybox(options.skybox_path)) {
            sky_switch_->addChild(false, sky);  // shown by ApplySceneTheme
        }
    }
    scene_root_->addChild(sky_switch_);
    ambient_light_ = vsg::AmbientLight::create();
    ambient_light_->name = "ambient";
    ambient_light_->color.set(1.0f, 1.0f, 1.0f);
    ambient_light_->intensity = 0.2f;
    scene_root_->addChild(ambient_light_);
    scene_root_->addChild(
        MakeDirectionalLight("key light", kKeyAzimuth, kKeyElevation, kKeyIntensity, vsg::vec3(1.0f, 1.0f, 1.0f)));
    fill_light_ = MakeDirectionalLight(kFillLightName, kFillAzimuth, kFillElevation, kFillIntensity,
                                       vsg::vec3(0.85f, 0.90f, 1.00f));
    rim_light_ = MakeDirectionalLight(kRimLightName, kRimAzimuth, kRimElevation, kRimIntensity,
                                      vsg::vec3(1.00f, 0.95f, 0.88f));
    scene_root_->addChild(fill_light_);
    scene_root_->addChild(rim_light_);

    pbr_shader_set_ = vsg::createPhysicsBasedRenderingShaderSet();
    shared_objects_ = vsg::SharedObjects::create();
    num_bodies_ = scene.bodies.size();
    body_transforms_.clear();
    for (const BodyDescription& body : scene.bodies) {
        auto transform = vsg::MatrixTransform::create();
        for (const VisualShape& shape : body.shapes) {
            const TriangleMesh mesh = MakeShapeMesh(shape);
            OverlayMaterial material;
            material.color = shape.color;
            material.roughness = shape.roughness;
            material.metallic = shape.metallic;
            auto node = mesh.faces.empty() ? vsg::ref_ptr<vsg::Node>()
                                           : CreatePbrNode(UnsharedArrays(mesh), material, /*dynamic=*/false);
            if (!node) {
                continue;
            }
            auto local = vsg::MatrixTransform::create(ToMatrix(shape.local));
            local->addChild(node);
            transform->addChild(local);
        }
        scene_root_->addChild(transform);
        body_transforms_.push_back(transform);
    }
    overlay_root_ = vsg::Group::create();
    scene_root_->addChild(overlay_root_);
    // Frame triads last: drawn without a depth test, so they show through hulls and water.
    frame_switch_ = vsg::Switch::create();
    frame_transforms_.clear();
    for (const BodyDescription& body : scene.bodies) {
        auto transform = vsg::MatrixTransform::create();
        transform->addChild(CreateAxisTriad(TriadLength(body.local_bounds, scene.model_bounds)));
        frame_switch_->addChild(false, transform);
        frame_transforms_.push_back(transform);
    }
    scene_root_->addChild(frame_switch_);

    // ---- Camera ----
    look_at_ = vsg::LookAt::create(vsg::dvec3(10.0, -10.0, 10.0), vsg::dvec3(0.0, 0.0, 0.0), vsg::dvec3(0.0, 0.0, 1.0));
    const VkExtent2D extent = window_->extent2D();
    auto perspective = vsg::Perspective::create(options.fov_deg,
                                                static_cast<double>(extent.width) / static_cast<double>(extent.height),
                                                0.05, 500.0);
    camera_ = vsg::Camera::create(perspective, look_at_, vsg::ViewportState::create(extent));

    // ---- Render graph and ImGui overlay ----
    render_graph_ = vsg::createRenderGraphForView(window_, camera_, scene_root_, VK_SUBPASS_CONTENTS_INLINE,
                                                  false /* no headlight */);
    auto command_graph = vsg::CommandGraph::create(window_, render_graph_);

    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;  // no imgui.ini next to the case
    if (!options.font_path.empty()) {
        if (!ImGui::GetIO().Fonts->AddFontFromFileTTF(options.font_path.c_str(), options.font_size * ui_scale_)) {
            std::cerr << "[SeastackViewer] Could not load font " << options.font_path << std::endl;
        }
    }
    render_graph_->addChild(vsgImGui::RenderImGui::create(window_, vsgImGui::RenderImGui::LegacyFunction([draw_gui]() {
                                                              if (draw_gui) {
                                                                  draw_gui();
                                                              }
                                                              return true;
                                                          })));

    // ImGui gets events first, so dragging a slider does not move the camera.
    viewer_->addEventHandler(SeastackImGuiEvents::create());
    viewer_->addEventHandler(vsg::CloseHandler::create(viewer_));
    trackball_ = SeastackTrackball::create(camera_);
    viewer_->addEventHandler(trackball_);

    viewer_->assignRecordAndSubmitTaskAndPresentation({command_graph});
    viewer_->compile();
    return true;
}

vsg::ref_ptr<vsg::Node> SeastackViewer::CompileNode(vsg::ref_ptr<vsg::Node> node) {
    if (!node || !viewer_ || !viewer_->compileManager) {
        return node;
    }
    auto result = viewer_->compileManager->compile(node);
    if (!result) {
        std::cerr << "[SeastackViewer] Could not compile overlay node." << std::endl;
        return {};
    }
    vsg::updateViewer(*viewer_, result);
    return node;
}

vsg::ref_ptr<vsg::Node> SeastackViewer::CreateOverlayMesh(const TriangleMesh& mesh, const OverlayMaterial& material) {
    if (mesh.faces.empty()) {
        return {};
    }
    return CompileNode(CreatePbrNode(UnsharedArrays(mesh), material, /*dynamic=*/true));
}

vsg::ref_ptr<vsg::Node> SeastackViewer::CreateIndexedOverlayMesh(const TriangleMesh& mesh, const Rgba& color) {
    if (mesh.faces.empty()) {
        return {};
    }
    // White material so the per-vertex colours show unchanged.
    return CompileNode(CreatePbrNode(SharedArrays(mesh, color), OverlayMaterial{}, /*dynamic=*/true));
}

vsg::ref_ptr<vsg::Node> SeastackViewer::CreateSphereOverlay(const OverlayMaterial& material,
                                                            vsg::ref_ptr<vsg::MatrixTransform> transform) {
    if (!transform) {
        return {};
    }
    auto sphere = CreatePbrNode(UnsharedArrays(MakeSphereMesh(1.0, 24, 12)), material, /*dynamic=*/false);
    if (!sphere) {
        return {};
    }
    transform->addChild(sphere);
    return CompileNode(transform);
}

SeastackViewer::MeshArrays SeastackViewer::UnsharedArrays(const TriangleMesh& mesh) {
    const size_t num_faces = mesh.faces.size();

    std::vector<Vec3> corner_normals = mesh.corner_normals;
    if (corner_normals.size() != 3 * num_faces) {
        const CornerNormals smooth = ComputeCreaseNormals(mesh.vertices, mesh.faces, kCreaseAngleDeg);
        corner_normals.resize(3 * num_faces);
        for (size_t f = 0; f < num_faces; ++f) {
            for (int k = 0; k < 3; ++k) {
                corner_normals[3 * f + k] = smooth.normals[smooth.face_normal_indices[f][k]];
            }
        }
    }

    const uint32_t n = static_cast<uint32_t>(3 * num_faces);
    MeshArrays a;
    a.vertices = vsg::vec3Array::create(n);
    a.normals = vsg::vec3Array::create(n);
    a.texcoords = vsg::vec2Array::create(n, vsg::vec2(0.0f, 0.0f));
    a.colors = vsg::vec4Array::create(n, vsg::vec4(1.0f, 1.0f, 1.0f, 1.0f));
    a.indices = vsg::uintArray::create(n);
    for (size_t f = 0; f < num_faces; ++f) {
        for (int k = 0; k < 3; ++k) {
            const size_t c = 3 * f + k;
            const Vec3& p = mesh.vertices[static_cast<size_t>(mesh.faces[f][k])];
            const Vec3& nn = corner_normals[c];
            (*a.vertices)[c] = vsg::vec3(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
            (*a.normals)[c] = vsg::vec3(static_cast<float>(nn[0]), static_cast<float>(nn[1]), static_cast<float>(nn[2]));
            (*a.indices)[c] = static_cast<uint32_t>(c);
        }
    }
    return a;
}

SeastackViewer::MeshArrays SeastackViewer::SharedArrays(const TriangleMesh& mesh, const Rgba& color) {
    const uint32_t nv = static_cast<uint32_t>(mesh.vertices.size());
    MeshArrays a;
    a.vertices = vsg::vec3Array::create(nv);
    a.normals = vsg::vec3Array::create(nv, vsg::vec3(0.0f, 0.0f, 0.0f));
    a.texcoords = vsg::vec2Array::create(nv, vsg::vec2(0.0f, 0.0f));
    a.colors = vsg::vec4Array::create(nv, vsg::vec4(color[0], color[1], color[2], 1.0f));
    a.indices = vsg::uintArray::create(static_cast<uint32_t>(3 * mesh.faces.size()));
    for (uint32_t i = 0; i < nv; ++i) {
        const Vec3& p = mesh.vertices[i];
        (*a.vertices)[i] = vsg::vec3(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
    }
    // Area-weighted vertex normals (unnormalised face cross products summed).
    for (size_t f = 0; f < mesh.faces.size(); ++f) {
        const auto& face = mesh.faces[f];
        const vsg::vec3& p0 = (*a.vertices)[static_cast<uint32_t>(face[0])];
        const vsg::vec3& p1 = (*a.vertices)[static_cast<uint32_t>(face[1])];
        const vsg::vec3& p2 = (*a.vertices)[static_cast<uint32_t>(face[2])];
        const vsg::vec3 face_normal = vsg::cross(p1 - p0, p2 - p0);
        for (int k = 0; k < 3; ++k) {
            (*a.normals)[static_cast<uint32_t>(face[k])] += face_normal;
            (*a.indices)[static_cast<uint32_t>(3 * f + k)] = static_cast<uint32_t>(face[k]);
        }
    }
    for (auto& nn : *a.normals) {
        const float len = vsg::length(nn);
        nn = (len > 1e-12f) ? nn / len : vsg::vec3(0.0f, 0.0f, 1.0f);
    }
    return a;
}

vsg::ref_ptr<vsg::Node> SeastackViewer::CreatePbrNode(const MeshArrays& arrays, const OverlayMaterial& shape,
                                                      bool dynamic, bool on_top) {
    if (!arrays.indices || arrays.indices->size() == 0) {
        return {};
    }
    if (dynamic) {
        for (vsg::Data* data : {static_cast<vsg::Data*>(arrays.vertices.get()),
                                static_cast<vsg::Data*>(arrays.normals.get()),
                                static_cast<vsg::Data*>(arrays.colors.get())}) {
            data->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;
        }
    }

    // Material colours are linear (see Rgba); vertex colours multiply them.
    auto material = vsg::PbrMaterialValue::create();
    material->value().baseColorFactor.set(shape.color[0], shape.color[1], shape.color[2], shape.color[3]);
    material->value().diffuseFactor.set(shape.color[0], shape.color[1], shape.color[2], shape.color[3]);
    material->value().roughnessFactor = shape.roughness;
    material->value().metallicFactor = shape.metallic;

    auto config = vsg::GraphicsPipelineConfigurator::create(pbr_shader_set_);
    config->shaderHints->defines.insert("VSG_TWO_SIDED_LIGHTING");
    SetPipelineStates states;
    states.blend = shape.color[3] < 1.0f;
    states.depth_test = !on_top;
    config->accept(states);
    if (!config->assignDescriptor("material", material) && !config->assignDescriptor("PbrMaterial", material)) {
        std::cerr << "[SeastackViewer] PBR shader set has no material descriptor." << std::endl;
        return {};
    }
    vsg::DataList data_list;
    config->assignArray(data_list, "vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, arrays.vertices);
    config->assignArray(data_list, "vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, arrays.normals);
    config->assignArray(data_list, "vsg_TexCoord0", VK_VERTEX_INPUT_RATE_VERTEX, arrays.texcoords);
    config->assignArray(data_list, "vsg_Color", VK_VERTEX_INPUT_RATE_VERTEX, arrays.colors);
    config->init();

    auto draw = vsg::VertexIndexDraw::create();
    draw->assignArrays(data_list);
    draw->assignIndices(arrays.indices);
    draw->indexCount = static_cast<uint32_t>(arrays.indices->size());
    draw->instanceCount = 1;

    auto state_group = vsg::StateGroup::create();
    config->copyTo(state_group, shared_objects_);
    state_group->addChild(draw);
    return state_group;
}

vsg::ref_ptr<vsg::Node> SeastackViewer::CreateAxisTriad(double length) {
    struct Axis {
        Quat rotation;  ///< Turns the z-aligned cylinder onto this axis
        Rgba color;     ///< Linear
    };
    constexpr double kHalfSqrt2 = 0.70710678118654752;
    const Axis axes[3] = {
        {{kHalfSqrt2, 0.0, kHalfSqrt2, 0.0}, {0.85f, 0.10f, 0.08f, 1.0f}},   // +90 deg about y: z -> x
        {{kHalfSqrt2, -kHalfSqrt2, 0.0, 0.0}, {0.12f, 0.70f, 0.12f, 1.0f}},  // -90 deg about x: z -> y
        {kIdentityQuat, {0.10f, 0.30f, 0.95f, 1.0f}},
    };
    const double shaft_r = 0.02 * length;
    const double tip_r = 0.05 * length;
    auto group = vsg::Group::create();
    for (const Axis& axis : axes) {
        OverlayMaterial material;
        material.color = axis.color;
        material.roughness = 0.4f;
        Pose shaft_pose;
        shaft_pose.rotation = axis.rotation;
        shaft_pose.position = Rotate(axis.rotation, {0.0, 0.0, 0.5 * length});
        auto shaft = vsg::MatrixTransform::create(ToMatrix(shaft_pose));
        shaft->addChild(CreatePbrNode(UnsharedArrays(MakeCylinderMesh(shaft_r, length, 16)), material, false, true));
        group->addChild(shaft);
        Pose tip_pose;
        tip_pose.position = Rotate(axis.rotation, {0.0, 0.0, length});
        auto tip = vsg::MatrixTransform::create(ToMatrix(tip_pose));
        tip->addChild(CreatePbrNode(UnsharedArrays(MakeSphereMesh(tip_r, 16, 8)), material, false, true));
        group->addChild(tip);
    }
    OverlayMaterial origin_material;
    origin_material.color = {0.9f, 0.9f, 0.9f, 1.0f};
    group->addChild(CreatePbrNode(UnsharedArrays(MakeSphereMesh(1.5 * shaft_r, 16, 8)), origin_material, false, true));
    return group;
}

void SeastackViewer::SetShowBodyFrames(bool show) {
    if (show == show_body_frames_) {
        return;
    }
    show_body_frames_ = show;
    if (frame_switch_) {
        frame_switch_->setAllChildren(show);
    }
}

bool SeastackViewer::ProjectToWindow(const Vec3& world, float* x_px, float* y_px) const {
    if (!camera_ || !x_px || !y_px) {
        return false;
    }
    // Matrix-vector products only (see ToMatrix on the dmat4 link issue).
    const vsg::dvec4 eye_space = camera_->viewMatrix->transform() * vsg::dvec4(world[0], world[1], world[2], 1.0);
    const vsg::dvec4 clip = camera_->projectionMatrix->transform() * eye_space;
    if (clip.w <= 1e-9) {
        return false;
    }
    const VkViewport vp = camera_->getViewport();
    *x_px = vp.x + static_cast<float>(0.5 * (clip.x / clip.w + 1.0)) * vp.width;
    *y_px = vp.y + static_cast<float>(0.5 * (clip.y / clip.w + 1.0)) * vp.height;
    return true;
}

void SeastackViewer::SetFrame(const VizFrame& frame) {
    if (frame.bodies.size() != num_bodies_) {
        return;
    }
    for (size_t i = 0; i < num_bodies_; ++i) {
        body_transforms_[i]->matrix = ToMatrix(frame.bodies[i].pose);
        frame_transforms_[i]->matrix = body_transforms_[i]->matrix;
    }
}

bool SeastackViewer::RenderFrame() {
    if (!viewer_) {
        return false;
    }
    // vsg::Exception does not derive from std::exception; report it here so
    // the run ends with a readable message instead of "unknown error".
    try {
        if (!viewer_->advanceToNextFrame()) {
            return false;
        }
        viewer_->handleEvents();
        viewer_->update();
        viewer_->recordAndSubmit();
        viewer_->present();
    } catch (const vsg::Exception& e) {
        std::cerr << "[SeastackViewer] VSG error: " << e.message << " (VkResult " << e.result << ")" << std::endl;
        return false;
    }
    return viewer_->active();
}

void SeastackViewer::SetCameraPose(const CameraPose& pose, double duration_s) {
    if (!look_at_) {
        return;
    }
    const vsg::dvec3 eye(pose.eye[0], pose.eye[1], pose.eye[2]);
    const vsg::dvec3 target(pose.target[0], pose.target[1], pose.target[2]);
    const vsg::dvec3 up(pose.up[0], pose.up[1], pose.up[2]);
    if (duration_s > 0.0 && trackball_) {
        trackball_->setViewpoint(vsg::LookAt::create(eye, target, up), duration_s);
        return;
    }
    look_at_->eye = eye;
    look_at_->center = target;
    look_at_->up = up;
}

Vec3 SeastackViewer::GetViewDirection() const {
    if (!look_at_) {
        return ViewDirection(CameraView::kIsometric);
    }
    const vsg::dvec3 d = vsg::normalize(look_at_->eye - look_at_->center);
    return {d.x, d.y, d.z};
}

double SeastackViewer::GetViewDistance() const {
    return look_at_ ? vsg::length(look_at_->eye - look_at_->center) : 0.0;
}

double SeastackViewer::GetViewportAspect() const {
    if (camera_) {
        const VkRect2D area = camera_->getRenderArea();
        if (area.extent.height > 0) {
            return static_cast<double>(area.extent.width) / static_cast<double>(area.extent.height);
        }
    }
    return 16.0 / 9.0;
}

void SeastackViewer::SetSceneViewport(int x, int y, int width, int height) {
    if (!camera_ || !camera_->viewportState || !window_) {
        return;
    }
    const VkExtent2D extent = window_->extent2D();
    const int window_w = static_cast<int>(extent.width);
    const int window_h = static_cast<int>(extent.height);
    if (window_w <= 0 || window_h <= 0) {
        return;  // minimised
    }
    x = std::clamp(x, 0, window_w - 1);
    y = std::clamp(y, 0, window_h - 1);
    width = std::clamp(width, 1, window_w - x);
    height = std::clamp(height, 1, window_h - y);
    // The camera viewport is applied as dynamic state each frame, and the
    // trackball maps the mouse through it, so no pipeline rebuild is needed.
    camera_->viewportState->set(x, y, static_cast<uint32_t>(width), static_cast<uint32_t>(height));
    if (auto* perspective = dynamic_cast<vsg::Perspective*>(camera_->projectionMatrix.get())) {
        perspective->aspectRatio = static_cast<double>(width) / static_cast<double>(height);
    }
}

void SeastackViewer::SetClipPlanes(const ClipPlanes& clip) {
    if (!camera_) {
        return;
    }
    if (auto* perspective = dynamic_cast<vsg::Perspective*>(camera_->projectionMatrix.get())) {
        perspective->nearDistance = clip.near_m;
        perspective->farDistance = clip.far_m;
    }
}

void SeastackViewer::ApplySceneTheme(const SceneTheme& theme) {
    if (render_graph_) {
        // Swapchain is sRGB, so clear values are linear.
        VkClearColorValue clear{};
        for (int i = 0; i < 3; ++i) {
            clear.float32[i] = SrgbToLinear(theme.background_srgb[static_cast<size_t>(i)]);
        }
        clear.float32[3] = 1.0f;
        render_graph_->setClearValues(clear);
    }
    if (sky_switch_) {
        sky_switch_->setAllChildren(theme.show_sky);
    }
    if (ambient_light_) {
        ambient_light_->intensity = theme.ambient_intensity;
    }
    if (fill_light_) {
        fill_light_->intensity = theme.fill_intensity;
    }
    if (rim_light_) {
        rim_light_->intensity = theme.rim_intensity;
    }
}

}  // namespace seastack::viz
