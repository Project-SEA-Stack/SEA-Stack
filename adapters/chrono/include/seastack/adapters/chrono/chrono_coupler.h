/*********************************************************************
 * @file  chrono_coupler.h
 * @brief ChronoHydroCoupler: bridges Chrono bodies and HydroForces.
 *
 * MAIN TYPES:
 *   - ChronoHydroCoupler: Adapter between Chrono and HydroForces
 *
 * ROLE: Extracts SystemState from Chrono bodies, invokes HydroForces
 * to compute forces. Used internally by HydroSystem façade.
 *********************************************************************/

#ifndef SEASTACK_ADAPTERS_CHRONO_COUPLER_H
#define SEASTACK_ADAPTERS_CHRONO_COUPLER_H

#include <seastack/core/system_state.h>
#include <seastack/hydro/hydro_forces.h>
#include <chrono/physics/ChBody.h>
#include <vector>
#include <memory>

namespace seastack::chrono {

// Forward declaration for internal utilities (defined in chrono_state_utils.h)
void BuildSystemStateFromChronoBodies(
    const std::vector<std::shared_ptr<::chrono::ChBody>>& bodies,
    seastack::hydro::SystemState& out_state);

/**
 * @brief ChronoHydroCoupler: bridges Chrono bodies and HydroForces.
 * 
 * Extracts state from Chrono bodies, evaluates forces via HydroForces,
 * and (in future) applies forces back to Chrono bodies. This is the
 * adapter between Chrono's physics engine and the Chrono-free HydroForces.
 */
class ChronoHydroCoupler {
public:
    /**
     * @brief Constructor.
     * 
     * @param hydro_forces Non-owning reference to HydroForces (caller must ensure lifetime)
     * @param bodies Vector of Chrono body pointers
     */
    ChronoHydroCoupler(hydro::HydroForces& hydro_forces,
                       std::vector<std::shared_ptr<::chrono::ChBody>> bodies);

    /**
     * @brief Evaluate hydrodynamic forces.
     * 
     * Extracts state from Chrono bodies, calls HydroForces to compute
     * forces, and returns the result.
     * 
     * @param time Current simulation time
     * @param per_component If non-null, filled with per-component force breakdown.
     * @return BodyForces Total forces (one GeneralizedForce per body, 6 DOF each)
     */
    hydro::BodyForces Evaluate(double time,
                               std::vector<hydro::ComponentForceRecord>* per_component = nullptr);

    /**
     * @brief Apply forces to Chrono bodies.
     * 
     * @deprecated Not implemented. Force application to Chrono bodies is
     * handled internally by the HydroSystem / ChronoForceAttacher callback path.
     * This method will be removed in a future release.
     * 
     * @param forces Forces to apply (currently ignored)
     */
    [[deprecated("Not implemented; force application uses the HydroSystem callback path")]]
    void ApplyForcesToChrono(const hydro::BodyForces& forces);

    /**
     * @brief Get profiling statistics from HydroForces.
     * 
     * Returns cumulative timing and call counts for each component type.
     * 
     * @return HydroForcesProfileStats Profiling statistics
     */
    hydro::HydroForcesProfileStats GetProfileStats() const { 
        return hydro_forces_.GetProfileStats(); 
    }

    /**
     * @brief Enable or disable profiling in HydroForces.
     * 
     * @param enabled True to enable profiling, false to disable
     */
    void SetProfilingEnabled(bool enabled) {
        hydro_forces_.SetProfilingEnabled(enabled);
    }

private:
    hydro::HydroForces& hydro_forces_;
    std::vector<std::shared_ptr<::chrono::ChBody>> bodies_;
};

}  // namespace seastack::chrono

#endif  // SEASTACK_ADAPTERS_CHRONO_COUPLER_H
