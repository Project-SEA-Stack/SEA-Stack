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
#include "vsg_water_shader.h"

#include <seastack/viz_core/scene_framing.h>
#include <seastack/viz_core/scene_model.h>
#include <seastack/viz_core/scene_picking.h>

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

    /// Scale [-] for the font, ImGui style and panel sizes. 1 at 96 DPI,
    /// 1.5 at 150 %, 2 when a Retina framebuffer is twice the window size.
    /// The window is not multiplied by the framebuffer part of this scale.
    float UiScale() const { return ui_scale_; }

    void SetClipPlanes(const ClipPlanes& clip);

    /// Show an RGB axis triad (x red, y green, z blue) at each body's
    /// reference frame, long enough to reach outside the body's shapes.
    void SetShowBodyFrames(bool show);

    /// Show or hide the axis triad of scene body i (drawn only while the body is visible).
    void SetBodyFrameVisible(size_t i, bool show);

    /// Show or hide scene body i (its shapes and frame triad).
    void SetBodyVisible(size_t i, bool visible);

    /// Map a world point [m] to window pixels (origin top-left) through the
    /// current camera. Returns false if the point is behind the camera.
    bool ProjectToWindow(const Vec3& world, float* x_px, float* y_px) const;

    /// Camera ray (world) through window pixel (x_px, y_px), origin top-left.
    /// Returns false if the pixel is outside the 3D view.
    bool WindowRay(float x_px, float y_px, Ray* ray) const;

    /// Clear colour, procedural sky and water colours, and light levels for `theme`.
    void ApplySceneTheme(const SceneTheme& theme);

    /// Soft shadows from the key light, cast by the bodies onto themselves
    /// and each other (not onto the water). Visual only.
    void SetShadows(bool on);

    // OverlayBuilder: overlays are drawn after the bodies so translucent
    // water blends over them.
    vsg::ref_ptr<vsg::Group> OverlayScene() override { return overlay_root_; }
    vsg::ref_ptr<vsg::Node> CreateOverlayMesh(const TriangleMesh& mesh, const OverlayMaterial& material) override;
    /// Water shader (Fresnel sky reflection, sun glint, soft edges); the PBR
    /// overlay if the shader cannot be compiled.
    vsg::ref_ptr<vsg::Node> CreateWaterMesh(const TriangleMesh& mesh, int nx, const OverlayMaterial& material) override;
    vsg::ref_ptr<vsg::Node> CreateGpuWaterGrid(const TriangleMesh& flat_grid, int nx) override;
    int SetGpuWaterWaves(const std::vector<DisplayWaveComponent>& components) override {
        return water_shader_.SetWaves(components);
    }
    void SetWaterElevationScale(float elevation_scale) override { water_shader_.SetElevationScale(elevation_scale); }
    bool SetWaterElevationStyle(bool on, float range_m, float contour_m, WaterPalette palette) override {
        water_shader_.SetElevationStyle(on, range_m, contour_m, palette);
        return true;
    }
    bool SetWaterGridLines(float cells_per_line) override {
        water_shader_.SetGridLines(cells_per_line);
        return true;
    }
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
    /// marks the vertex, normal and colour arrays for per-frame updates;
    /// `on_top` disables the depth test (draw it after the rest of the scene).
    vsg::ref_ptr<vsg::Node> CreatePbrNode(const MeshArrays& arrays, const OverlayMaterial& material, bool dynamic,
                                          bool on_top = false);

    /// x / y / z arrows (red / green / blue) of `length` [m] from the origin.
    vsg::ref_ptr<vsg::Node> CreateAxisTriad(double length);

    /// Compile a node made after Viewer::compile(); returns null on failure.
    vsg::ref_ptr<vsg::Node> CompileNode(vsg::ref_ptr<vsg::Node> node);

    vsg::ref_ptr<vsg::Viewer> viewer_;
    vsg::ref_ptr<vsg::Window> window_;
    vsg::ref_ptr<vsg::Camera> camera_;
    vsg::ref_ptr<vsg::LookAt> look_at_;
    vsg::ref_ptr<vsg::Trackball> trackball_;
    vsg::ref_ptr<vsg::RenderGraph> render_graph_;
    vsg::ref_ptr<vsg::View> view_;  ///< The scene view; its ViewDependentState renders the shadow maps
    vsg::ref_ptr<vsg::Group> scene_root_;
    vsg::ref_ptr<vsg::Group> overlay_root_;
    vsg::ref_ptr<vsg::Switch> sky_switch_;  ///< Holds the procedural sky backdrop
    vsg::ref_ptr<vsg::ShaderSet> pbr_shader_set_;
    vsg::ref_ptr<vsg::SharedObjects> shared_objects_;

    vsg::ref_ptr<vsg::AmbientLight> ambient_light_;
    vsg::ref_ptr<vsg::DirectionalLight> key_light_;
    vsg::ref_ptr<vsg::ShadowSettings> key_shadows_;  ///< Assigned to key_light_ while shadows are on
    vsg::dvec3 model_center_;     ///< Centre of the model bounds at Initialize() [m]
    double model_radius_ = 10.0;  ///< Radius of the model bounds at Initialize() [m]
    vsg::ref_ptr<vsg::DirectionalLight> fill_light_;
    vsg::ref_ptr<vsg::DirectionalLight> rim_light_;
    vsg::ref_ptr<vsg::DirectionalLight> sky_fill_light_;    ///< Straight down, sky colour; no shadow
    vsg::ref_ptr<vsg::DirectionalLight> sea_bounce_light_;  ///< Straight up, water colour; no shadow
    WaterShader water_shader_;  ///< Shared by every water mesh; eye updated each frame

    /// One transform per scene body, in scene order (world <- body ref frame).
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> body_transforms_;
    /// Child i = body_transforms_[i]; masks follow SetBodyVisible.
    vsg::ref_ptr<vsg::Switch> body_switch_;
    std::vector<char> body_visible_;
    /// Axis triads (hidden by default), drawn after everything else; one
    /// transform per body, posed like body_transforms_.
    vsg::ref_ptr<vsg::Switch> frame_switch_;
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> frame_transforms_;
    std::vector<char> body_axes_;  ///< Per body: triad requested (SetBodyFrameVisible)
    /// Triad i is drawn when body i is visible and its axes are requested.
    void UpdateFrameMask(size_t i);
    size_t num_bodies_ = 0;
    float ui_scale_ = 1.0f;
};

}  // namespace seastack::viz
