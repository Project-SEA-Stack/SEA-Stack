// =============================================================================
// SEA-Stack Viewer
// =============================================================================
// SEA-Stack's own VSG viewer (--viewer=seastack). Draws bodies from a
// SceneDescription and poses them from a VizFrame each frame; it never reads
// Chrono. Owns the window (4x MSAA, vsync), camera, studio lights and the
// ImGui overlay.
// =============================================================================
#pragma once

#include "vsg_overlay_builder.h"
#include "vsg_theme.h"

#include <seastack/viz_core/scene_framing.h>
#include <seastack/viz_core/scene_model.h>

#include <vsg/all.h>

#include <functional>
#include <string>
#include <vector>

namespace seastack::viz {

class SeastackViewer : public OverlayBuilder {
  public:
    struct Options {
        std::string title = "SEA-Stack";
        int width = 1280;         ///< Window width [px at 100 % display scale]
        int height = 720;         ///< Window height [px at 100 % display scale]
        double fov_deg = 40.0;    ///< Vertical field of view [deg]
        std::string font_path;    ///< ImGui TrueType font; empty = ImGui default
        float font_size = 15.0f;  ///< ImGui font size [px at 100 % display scale]
        std::string skybox_path;  ///< KTX cubemap shown by themes with a sky; empty = none
    };

    /// ImGui drawing callback, called once per rendered frame.
    using GuiCallback = std::function<void()>;

    /// Create the window, scene and overlay. Returns false (and logs the
    /// reason) if Vulkan or the window could not be set up.
    bool Initialize(const SceneDescription& scene, const Options& options, GuiCallback draw_gui);

    /// Pose the bodies from `frame`. Ignored if it does not match the scene.
    void SetFrame(const VizFrame& frame);

    /// Handle events and draw one frame. Returns false once the window closes.
    bool RenderFrame();

    /// Move the camera to `pose`; with duration_s > 0 the move is animated.
    void SetCameraPose(const CameraPose& pose, double duration_s = 0.0);

    /// Current view direction as a unit vector from target towards eye.
    Vec3 GetViewDirection() const;

    /// Camera eye-to-target distance [m].
    double GetViewDistance() const;

    /// Viewport width / height [-].
    double GetViewportAspect() const;

    /// Render the 3D scene into this window rectangle [px, origin top-left],
    /// e.g. the area the GUI panels leave free. Clamped to the window.
    void SetSceneViewport(int x, int y, int width, int height);

    /// Display scale [-] (e.g. 1.5 at 150 %); window and font are scaled by it.
    float UiScale() const { return ui_scale_; }

    void SetClipPlanes(const ClipPlanes& clip);

    /// Clear colour, skybox visibility and light levels for `theme`.
    void ApplySceneTheme(const SceneTheme& theme);

    // OverlayBuilder: overlays are drawn after the bodies so translucent
    // water blends over them.
    vsg::ref_ptr<vsg::Group> OverlayScene() override { return overlay_root_; }
    vsg::ref_ptr<vsg::Node> CreateOverlayMesh(const TriangleMesh& mesh, const OverlayMaterial& material) override;
    vsg::ref_ptr<vsg::Node> CreateIndexedOverlayMesh(const TriangleMesh& mesh, const Rgba& color) override;
    vsg::ref_ptr<vsg::Node> CreateSphereOverlay(const OverlayMaterial& material,
                                                vsg::ref_ptr<vsg::MatrixTransform> transform) override;

  private:
    /// Vertex arrays in the OverlayBuilder layout, plus triangle indices.
    struct MeshArrays {
        vsg::ref_ptr<vsg::vec3Array> vertices;
        vsg::ref_ptr<vsg::vec3Array> normals;
        vsg::ref_ptr<vsg::vec2Array> texcoords;
        vsg::ref_ptr<vsg::vec4Array> colors;
        vsg::ref_ptr<vsg::uintArray> indices;
    };

    /// One vertex per face corner (3 f + k), so normals can be per corner;
    /// authored normals, else crease-angle smoothing. Colours white.
    static MeshArrays UnsharedArrays(const TriangleMesh& mesh);

    /// Vertex i = mesh vertex i, area-weighted vertex normals, colours `color`.
    static MeshArrays SharedArrays(const TriangleMesh& mesh, const Rgba& color);

    /// PBR node drawing `arrays`. Alpha < 1 enables blending; `dynamic`
    /// marks the vertex, normal and colour arrays for per-frame updates.
    vsg::ref_ptr<vsg::Node> CreatePbrNode(const MeshArrays& arrays, const OverlayMaterial& material, bool dynamic);

    /// Compile a node made after Viewer::compile(); returns null on failure.
    vsg::ref_ptr<vsg::Node> CompileNode(vsg::ref_ptr<vsg::Node> node);

    vsg::ref_ptr<vsg::Viewer> viewer_;
    vsg::ref_ptr<vsg::Window> window_;
    vsg::ref_ptr<vsg::Camera> camera_;
    vsg::ref_ptr<vsg::LookAt> look_at_;
    vsg::ref_ptr<vsg::Trackball> trackball_;
    vsg::ref_ptr<vsg::RenderGraph> render_graph_;
    vsg::ref_ptr<vsg::Group> scene_root_;
    vsg::ref_ptr<vsg::Group> overlay_root_;
    vsg::ref_ptr<vsg::Switch> sky_switch_;  ///< Holds the skybox, if it loaded
    vsg::ref_ptr<vsg::ShaderSet> pbr_shader_set_;
    vsg::ref_ptr<vsg::SharedObjects> shared_objects_;

    vsg::ref_ptr<vsg::AmbientLight> ambient_light_;
    vsg::ref_ptr<vsg::DirectionalLight> fill_light_;
    vsg::ref_ptr<vsg::DirectionalLight> rim_light_;

    /// One transform per scene body, in scene order (world <- body ref frame).
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> body_transforms_;
    size_t num_bodies_ = 0;
    float ui_scale_ = 1.0f;
};

}  // namespace seastack::viz
