// =============================================================================
// SEA-Stack VSG Skybox
// =============================================================================
// Cubemap backdrop for the SEA-Stack viewer, matching the Chrono viewer's
// skybox (both follow vsgExamples' vsgskybox). Drawn at the far plane, so it
// sits behind everything regardless of where it is in the scene graph.
// =============================================================================
#pragma once

#include <vsg/all.h>

#include <string>

namespace seastack::viz {

/// Skybox node textured with the KTX cubemap at `path`, oriented for the
/// Z-up world frame. Returns null (and logs the reason) if the file cannot
/// be read.
vsg::ref_ptr<vsg::Node> CreateSkybox(const std::string& path);

}  // namespace seastack::viz
