/// @file vsg_visual_system.h
/// @brief Chrono VSG visual system with SEA-Stack camera control.
#pragma once

#include "vsg_theme.h"

#include <seastack/viz_core/scene_framing.h>

#include <chrono_vsg/ChVisualSystemVSG.h>

#include <vsg/all.h>

#include <cmath>

namespace seastack::viz {

/// Thin extension of Chrono's VSG visual system that exposes the camera
/// operations SEA-Stack needs after Initialize(): setting a full pose
/// (eye, target and up, which resets any roll left by the trackball),
/// animated moves between views, and model-scaled clip planes.
///
/// Uses only protected members of ChVisualSystemVSG; no Chrono changes.
class SeastackVisualSystemVSG : public ::chrono::vsg3d::ChVisualSystemVSG {
  public:
    /// Move the camera to `pose`. With duration_s > 0 the trackball animates
    /// the move; otherwise it jumps. No effect before Initialize().
    void SetCameraPose(const CameraPose& pose, double duration_s = 0.0) {
        if (!m_lookAt) {
            return;
        }
        const vsg::dvec3 eye(pose.eye[0], pose.eye[1], pose.eye[2]);
        const vsg::dvec3 target(pose.target[0], pose.target[1], pose.target[2]);
        const vsg::dvec3 up(pose.up[0], pose.up[1], pose.up[2]);
        if (duration_s > 0.0) {
            if (auto trackball = FindTrackball()) {
                trackball->setViewpoint(vsg::LookAt::create(eye, target, up), duration_s);
                return;
            }
        }
        m_lookAt->eye = eye;
        m_lookAt->center = target;
        m_lookAt->up = up;
    }

    /// Current view direction as a unit vector from target towards eye.
    Vec3 GetViewDirection() const {
        if (!m_lookAt) {
            return ViewDirection(CameraView::kIsometric);
        }
        const vsg::dvec3 d = vsg::normalize(m_lookAt->eye - m_lookAt->center);
        return {d.x, d.y, d.z};
    }

    /// Set the perspective near / far clip distances [m]. Chrono fixes these
    /// at 0.05 / 500 m, which clips lab-scale and very large models.
    void SetClipPlanes(const ClipPlanes& clip) {
        if (!m_vsg_camera) {
            return;
        }
        if (auto* perspective = dynamic_cast<vsg::Perspective*>(m_vsg_camera->projectionMatrix.get())) {
            perspective->nearDistance = clip.near_m;
            perspective->farDistance = clip.far_m;
        }
    }

    /// Remember the skybox node so themes can hide and restore it. Call once
    /// right after Initialize(); Chrono adds the skybox as the first scene child.
    void CaptureSkyNode() {
        if (!m_scene || m_scene->children.empty()) {
            return;
        }
        auto first = m_scene->children.front();
        if (first && !first.cast<vsg::Light>()) {
            sky_node_ = first;
            sky_placeholder_ = vsg::Group::create();
        }
    }

    /// Apply the runtime-switchable parts of a theme: clear colour, skybox
    /// visibility and light levels. No effect before Initialize().
    void ApplySceneTheme(const SceneTheme& theme, const char* fill_light_name, const char* rim_light_name) {
        if (m_renderGraph) {
            // Swapchain is sRGB, so clear values are linear.
            VkClearColorValue clear{};
            for (int i = 0; i < 3; ++i) {
                clear.float32[i] = SrgbToLinear(theme.background_srgb[static_cast<size_t>(i)]);
            }
            clear.float32[3] = 1.0f;
            m_renderGraph->setClearValues(clear);
        }
        if (m_scene && sky_node_) {
            m_scene->children.front() = theme.show_sky ? sky_node_ : sky_placeholder_;
        }
        if (m_scene) {
            for (auto& child : m_scene->children) {
                auto light = child.cast<vsg::Light>();
                if (!light) {
                    continue;
                }
                if (light.cast<vsg::AmbientLight>()) {
                    light->intensity = theme.ambient_intensity;
                } else if (light->name == fill_light_name) {
                    light->intensity = theme.fill_intensity;
                } else if (light->name == rim_light_name) {
                    light->intensity = theme.rim_intensity;
                }
            }
        }
    }

    /// Viewport width / height [-].
    double GetViewportAspect() const {
        if (m_window) {
            const auto extent = m_window->extent2D();
            if (extent.height > 0) {
                return static_cast<double>(extent.width) / static_cast<double>(extent.height);
            }
        }
        return 16.0 / 9.0;
    }

  private:
    static float SrgbToLinear(float c) {
        return (c <= 0.04045f) ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
    }

    vsg::ref_ptr<vsg::Node> sky_node_;
    vsg::ref_ptr<vsg::Node> sky_placeholder_;

    vsg::ref_ptr<vsg::Trackball> FindTrackball() const {
        if (!m_viewer) {
            return {};
        }
        for (const auto& handler : m_viewer->getEventHandlers()) {
            if (auto trackball = handler.cast<vsg::Trackball>()) {
                return trackball;
            }
        }
        return {};
    }
};

}  // namespace seastack::viz
