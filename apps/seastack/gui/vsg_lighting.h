// =============================================================================
// SEA-Stack VSG Lighting Utilities
// =============================================================================
// Functions for configuring the VSG scene lighting setup.
// Implements a 3-light studio configuration (key + fill + ambient).
// =============================================================================
#pragma once

#include <chrono_vsg/ChVisualSystemVSG.h>

namespace seastack::viz {

/// Adds the fill and rim lights of the studio rig to the VSG scene (the key
/// light is Chrono's own "sun light"). The fill comes from opposite the key
/// and lifts the dark side; the rim comes from behind and above and
/// separates silhouettes from the background. Lights are named
/// (vsg_config::kFillLightName, kRimLightName) so themes can adjust them.
///
/// @note Must be called AFTER ChVisualSystemVSG::Initialize() since the scene
///       is created during initialization.
///
/// @param vis Pointer to the initialized VSG visual system. Does nothing if null.
void AddStudioLights(::chrono::vsg3d::ChVisualSystemVSG* vis);

}  // namespace seastack::viz

