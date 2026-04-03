/*********************************************************************
 * @file  chrono_hydro_coupler.cpp
 * @brief Implementation of ChronoHydroCoupler.
 *********************************************************************/

#include <seastack/adapters/chrono/chrono_coupler.h>
#include <seastack/infra/debug_trace.h>
#include "chrono_state_utils.h"

#include <stdexcept>
#include <string>

namespace seastack::chrono {

ChronoHydroCoupler::ChronoHydroCoupler(
    hydro::HydroForces& hydro_forces,
    std::vector<std::shared_ptr<::chrono::ChBody>> bodies)
    : hydro_forces_(hydro_forces),
      bodies_(std::move(bodies)) {
    if (bodies_.empty()) {
        throw std::invalid_argument(
            "ChronoHydroCoupler: bodies vector must not be empty");
    }
}

hydro::BodyForces ChronoHydroCoupler::Evaluate(
        double time,
        std::vector<hydro::ComponentForceRecord>* per_component) {
    hydro::SystemState system_state;
    SEASTACK_TRACE_ONCE(
        std::string("ChronoHydroCoupler::Evaluate entering BuildSystemStateFromChronoBodies at t=") +
        std::to_string(time));
    BuildSystemStateFromChronoBodies(bodies_, system_state);
    SEASTACK_TRACE_ONCE(
        std::string("ChronoHydroCoupler::Evaluate returned from BuildSystemStateFromChronoBodies at t=") +
        std::to_string(time));

    SEASTACK_TRACE_ONCE(
        std::string("ChronoHydroCoupler::Evaluate entering HydroForces::Evaluate at t=") +
        std::to_string(time));
    hydro::BodyForces forces = hydro_forces_.Evaluate(system_state, time, per_component);
    SEASTACK_TRACE_ONCE(
        std::string("ChronoHydroCoupler::Evaluate returned from HydroForces::Evaluate at t=") +
        std::to_string(time));
    return forces;
}

void ChronoHydroCoupler::ApplyForcesToChrono(const hydro::BodyForces& forces) {
    (void)forces;
    // Force application is handled by the HydroSystem / ChronoForceAttacher callback path.
}

}  // namespace seastack::chrono
