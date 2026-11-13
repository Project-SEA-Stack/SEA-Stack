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

#include <seastack/viz_core/mesh_normals.h>
#include <seastack/viz_core/primitive_meshes.h>

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <algorithm>
#include <cmath>
#include <iostream>

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

/// Draw both faces: BEM panel meshes may be open or wound inwards.
struct DisableBackFaceCulling : public vsg::Visitor {
    void apply(vsg::Object& object) override { object.traverse(*this); }
    void apply(vsg::RasterizationState& state) override { state.cullMode = VK_CULL_MODE_NONE; }
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
    auto traits = vsg::WindowTraits::create();
    traits->windowTitle = options.title;
    traits->width = static_cast<uint32_t>(options.width);
    traits->height = static_cast<uint32_t>(options.height);
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

    // ---- Scene: lights and bodies ----
    scene_root_ = vsg::Group::create();
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
            auto node = CreateMeshNode(mesh, shape);
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
        if (!ImGui::GetIO().Fonts->AddFontFromFileTTF(options.font_path.c_str(), options.font_size)) {
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
    viewer_->addEventHandler(vsgImGui::SendEventsToImGui::create());
    viewer_->addEventHandler(vsg::CloseHandler::create(viewer_));
    trackball_ = SeastackTrackball::create(camera_);
    viewer_->addEventHandler(trackball_);

    viewer_->assignRecordAndSubmitTaskAndPresentation({command_graph});
    viewer_->compile();
    return true;
}

vsg::ref_ptr<vsg::Node> SeastackViewer::CreateMeshNode(const TriangleMesh& mesh, const VisualShape& shape) {
    const size_t num_faces = mesh.faces.size();
    if (num_faces == 0) {
        return {};
    }

    // Per-corner normals: authored or primitive ones, else crease-angle smoothing.
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

    // Unshared corners: one vertex per face corner, so normals can be per corner.
    const uint32_t n = static_cast<uint32_t>(3 * num_faces);
    auto vertices = vsg::vec3Array::create(n);
    auto normals = vsg::vec3Array::create(n);
    auto texcoords = vsg::vec2Array::create(n, vsg::vec2(0.0f, 0.0f));
    auto colors = vsg::vec4Array::create(n, vsg::vec4(1.0f, 1.0f, 1.0f, 1.0f));
    auto indices = vsg::uintArray::create(n);
    for (size_t f = 0; f < num_faces; ++f) {
        for (int k = 0; k < 3; ++k) {
            const size_t c = 3 * f + k;
            const Vec3& p = mesh.vertices[static_cast<size_t>(mesh.faces[f][k])];
            const Vec3& nn = corner_normals[c];
            (*vertices)[c] = vsg::vec3(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
            (*normals)[c] = vsg::vec3(static_cast<float>(nn[0]), static_cast<float>(nn[1]), static_cast<float>(nn[2]));
            (*indices)[c] = static_cast<uint32_t>(c);
        }
    }

    // Shape colours are linear (see Rgba); vertex colour stays white.
    auto material = vsg::PbrMaterialValue::create();
    material->value().baseColorFactor.set(shape.color[0], shape.color[1], shape.color[2], shape.color[3]);
    material->value().diffuseFactor.set(shape.color[0], shape.color[1], shape.color[2], shape.color[3]);
    material->value().roughnessFactor = shape.roughness;
    material->value().metallicFactor = shape.metallic;

    auto config = vsg::GraphicsPipelineConfigurator::create(pbr_shader_set_);
    config->shaderHints->defines.insert("VSG_TWO_SIDED_LIGHTING");
    DisableBackFaceCulling disable_culling;
    config->accept(disable_culling);
    if (!config->assignDescriptor("material", material) && !config->assignDescriptor("PbrMaterial", material)) {
        std::cerr << "[SeastackViewer] PBR shader set has no material descriptor." << std::endl;
        return {};
    }
    vsg::DataList arrays;
    config->assignArray(arrays, "vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, vertices);
    config->assignArray(arrays, "vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, normals);
    config->assignArray(arrays, "vsg_TexCoord0", VK_VERTEX_INPUT_RATE_VERTEX, texcoords);
    config->assignArray(arrays, "vsg_Color", VK_VERTEX_INPUT_RATE_VERTEX, colors);
    config->init();

    auto draw = vsg::VertexIndexDraw::create();
    draw->assignArrays(arrays);
    draw->assignIndices(indices);
    draw->indexCount = n;
    draw->instanceCount = 1;

    auto state_group = vsg::StateGroup::create();
    config->copyTo(state_group, shared_objects_);
    state_group->addChild(draw);
    return state_group;
}

void SeastackViewer::SetFrame(const VizFrame& frame) {
    if (frame.bodies.size() != num_bodies_) {
        return;
    }
    for (size_t i = 0; i < num_bodies_; ++i) {
        body_transforms_[i]->matrix = ToMatrix(frame.bodies[i].pose);
    }
}

bool SeastackViewer::RenderFrame() {
    if (!viewer_ || !viewer_->advanceToNextFrame()) {
        return false;
    }
    viewer_->handleEvents();
    viewer_->update();
    viewer_->recordAndSubmit();
    viewer_->present();
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
    if (window_) {
        const auto extent = window_->extent2D();
        if (extent.height > 0) {
            return static_cast<double>(extent.width) / static_cast<double>(extent.height);
        }
    }
    return 16.0 / 9.0;
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
