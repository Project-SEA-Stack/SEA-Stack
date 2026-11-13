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
        int width = 1280;         ///< Window width [px]
        int height = 720;         ///< Window height [px]
        double fov_deg = 40.0;    ///< Vertical field of view [deg]
        std::string font_path;    ///< ImGui TrueType font; empty = ImGui default
        float font_size = 20.0f;  ///< ImGui font size [px]
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

    void SetClipPlanes(const ClipPlanes& clip);

    /// Clear colour and light levels for `theme`.
    void ApplySceneTheme(const SceneTheme& theme);

    // OverlayBuilder: overlays are drawn after the bodies so translucent
    // water blends over them.
    vsg::ref_ptr<vsg::Group> OverlayScene() override { return overlay_root_; }
    vsg::ref_ptr<vsg::Node> CreateOverlayMesh(const TriangleMesh& mesh, const OverlayMaterial& material) override;

  private:
    /// PBR node for one triangle mesh with unshared vertices (see
    /// OverlayBuilder for the array layout). Alpha < 1 enables blending;
    /// `dynamic` marks the arrays for per-frame updates.
    vsg::ref_ptr<vsg::Node> CreateMeshNode(const TriangleMesh& mesh, const OverlayMaterial& material,
                                           bool dynamic);

    vsg::ref_ptr<vsg::Viewer> viewer_;
    vsg::ref_ptr<vsg::Window> window_;
    vsg::ref_ptr<vsg::Camera> camera_;
    vsg::ref_ptr<vsg::LookAt> look_at_;
    vsg::ref_ptr<vsg::Trackball> trackball_;
    vsg::ref_ptr<vsg::RenderGraph> render_graph_;
    vsg::ref_ptr<vsg::Group> scene_root_;
    vsg::ref_ptr<vsg::Group> overlay_root_;
    vsg::ref_ptr<vsg::ShaderSet> pbr_shader_set_;
    vsg::ref_ptr<vsg::SharedObjects> shared_objects_;

    vsg::ref_ptr<vsg::AmbientLight> ambient_light_;
    vsg::ref_ptr<vsg::DirectionalLight> fill_light_;
    vsg::ref_ptr<vsg::DirectionalLight> rim_light_;

    /// One transform per scene body, in scene order (world <- body ref frame).
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> body_transforms_;
    size_t num_bodies_ = 0;
};

}  // namespace seastack::viz
