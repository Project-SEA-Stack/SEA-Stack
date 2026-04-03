/*********************************************************************
 * @file  mooring_component.h
 * @brief Mooring force component backed by MoorDyn.
 *********************************************************************/

#ifndef SEASTACK_MOORING_MOORING_COMPONENT_H
#define SEASTACK_MOORING_MOORING_COMPONENT_H

#include <seastack/core/force_component.h>
#include <seastack/core/system_state.h>
#include <seastack/core/mooring_viz_data.h>
#include <memory>
#include <vector>

namespace seastack::mooring {

using seastack::hydro::BodyForces;
using seastack::hydro::HydroComponentType;
using seastack::hydro::IHydroForceComponent;
using seastack::hydro::SystemState;

class MoorDynWrapper;

/**
 * @brief IHydroForceComponent implementation that delegates to MoorDyn
 *        via MoorDynWrapper for mooring line force computation.
 *
 * Caches the most recent MoorDyn forces so that repeated evaluations
 * at the same time (e.g. HHT Newton iterations) return consistent
 * values without re-stepping MoorDyn.
 */
class MooringComponent : public IHydroForceComponent {
public:
    explicit MooringComponent(std::unique_ptr<MoorDynWrapper> wrapper);

    HydroComponentType Type() const override {
        return HydroComponentType::kMooring;
    }

    void Compute(const SystemState& state,
                 double time,
                 BodyForces& inout_forces) override;

    /// Return current node positions and tension for all mooring lines.
    std::vector<seastack::viz::MooringLineVizData> GetMooringLineStates() const;

private:
    void ApplyCachedForces(BodyForces& inout_forces) const;

    std::unique_ptr<MoorDynWrapper> wrapper_;
    double last_time_ = -1.0;
    BodyForces cached_forces_;
    bool has_cached_forces_ = false;
};

}  // namespace seastack::mooring

#endif  // SEASTACK_MOORING_MOORING_COMPONENT_H
