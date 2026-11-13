// =============================================================================
// SEA-Stack VSG Lighting Utilities - Implementation
// =============================================================================
#include "vsg_lighting.h"

#include "vsg_config.h"

#include <cmath>

#include <vsg/all.h>

namespace seastack::viz {

using namespace vsg_config;

namespace {

/// Directional light shining from (azimuth, elevation) [rad] in the Z-up world
/// frame; VSG directions point from the light into the scene.
vsg::ref_ptr<vsg::DirectionalLight> MakeDirectionalLight(const char* name, double azimuth, double elevation,
                                                         float intensity, const vsg::vec3& color) {
    const double se = std::sin(elevation);
    const double ce = std::cos(elevation);
    const double sa = std::sin(azimuth);
    const double ca = std::cos(azimuth);

    auto light = vsg::DirectionalLight::create();
    light->name = name;
    light->color = color;
    light->intensity = intensity;
    light->direction.set(static_cast<float>(-ce * ca), static_cast<float>(-ce * sa), static_cast<float>(-se));
    return light;
}

}  // namespace

void AddStudioLights(::chrono::vsg3d::ChVisualSystemVSG* vis) {
    if (!vis) {
        return;
    }
    auto scene = vis->GetVSGScene();
    if (!scene) {
        return;
    }

    // Key light is Chrono's "sun light". Fill (slightly cool) lifts the dark
    // side; rim (slightly warm, from behind and above) separates silhouettes
    // from the background. Intensities are set per theme at runtime.
    scene->addChild(MakeDirectionalLight(kFillLightName, kFillAzimuth, kFillElevation, kFillIntensity,
                                         vsg::vec3(0.85f, 0.90f, 1.00f)));
    scene->addChild(MakeDirectionalLight(kRimLightName, kRimAzimuth, kRimElevation, kRimIntensity,
                                         vsg::vec3(1.00f, 0.95f, 0.88f)));
}

}  // namespace seastack::viz
