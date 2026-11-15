#pragma once

#include <seastack/core/mooring_viz_data.h>
#include <seastack/viz_core/scene_model.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace chrono {
class ChBody;
class ChLinkTSDA;
class ChSystem;
}  // namespace chrono

namespace seastack::chrono {

class HydroSystem;

/// Loads the solver applied in the last step, by source (solver output).
struct ForceSample {
    std::vector<std::string> sources;             ///< e.g. "Excitation"
    std::vector<const ::chrono::ChBody*> bodies;  ///< Bodies the loads act on
    /// wrenches[s * bodies.size() + b]: source s on bodies[b]; world frame,
    /// moment about the centre of mass.
    std::vector<seastack::viz::Wrench> wrenches;
};
using ForceProvider = std::function<ForceSample()>;

/// Per-component loads of `hydro` (one source per component type, e.g.
/// "Radiation", "Mooring"). Enables the per-component capture of `hydro`
/// (a copy of each contribution; results are unchanged). The sample is
/// empty until the first force evaluation. `hydro` must outlive the provider.
ForceProvider MakeHydroForceProvider(HydroSystem* hydro);

/// Builds the renderer-independent scene (seastack::viz::SceneDescription)
/// from a Chrono system and captures VizFrames from it. Read-only: never
/// changes the system, so capturing frames cannot affect results.
///
/// Bodies are captured in system order at construction, excluding water-surface
/// helper bodies. Body visual shapes supported: box, sphere, cylinder and
/// triangle mesh (mesh scale applied); other shapes are skipped.
///
/// TSDAs that carry a Chrono point-point visual (SEGMENT or SPRING) are
/// exported as cable_lines (hoist ropes / PTO springs), separate from MoorDyn
/// mooring_lines, so the SEA-Stack viewer can draw Chrono-style glyphs.
class VizExtractor {
  public:
    /// @param system  Chrono system (must outlive the extractor).
    /// @param mooring Optional mooring provider (see SetMooringProvider).
    explicit VizExtractor(::chrono::ChSystem* system, seastack::viz::MooringVizProvider mooring = {});

    /// Set or replace the mooring provider. The line and node counts of the
    /// first non-empty result define the MoorDyn topology.
    void SetMooringProvider(seastack::viz::MooringVizProvider mooring);

    /// Set the source of per-body loads (VizFrame::body_wrenches). The source
    /// names of the first non-empty sample define scene.force_sources; scene
    /// bodies the sample does not list get zero loads.
    void SetForceProvider(ForceProvider forces);

    const seastack::viz::SceneDescription& Scene() const { return scene_; }

    /// State of all described bodies, mooring lines and cables now. Non-const
    /// only because the mooring topology and force sources are filled in on
    /// first available data.
    /// @param step Physics step count to stamp on the frame.
    seastack::viz::VizFrame Capture(int64_t step);

  private:
    ::chrono::ChSystem* system_;
    seastack::viz::MooringVizProvider mooring_;
    ForceProvider forces_;
    std::vector<std::shared_ptr<::chrono::ChBody>> bodies_;
    /// TSDAs with a SEGMENT/SPRING visual; indices match scene_.cable_lines.
    std::vector<std::shared_ptr<::chrono::ChLinkTSDA>> tsda_cables_;
    seastack::viz::SceneDescription scene_;
};

/// True for helper bodies that carry the visual water surface, not a model body.
bool IsWaterSurfaceBody(const ::chrono::ChBody& body);

/// Visual content of `system` that VizExtractor cannot export, so the
/// SEA-Stack viewer would not show it: body shapes other than box, sphere,
/// cylinder and triangle mesh; FEA meshes; visual shapes on links other than
/// TSDA SEGMENT/SPRING glyphs; other physics items with visual shapes. Each
/// entry is a short description for the user; empty = everything is drawable.
std::vector<std::string> FindUndrawableItems(const ::chrono::ChSystem& system);

}  // namespace seastack::chrono
