/*********************************************************************
 * @file  chrono_state_utils.cpp
 * @brief Implementation of Chrono-SystemState conversion utilities.
 *
 * Part of the Chrono coupling layer (src/hydro/chrono/).
 *********************************************************************/

#include "chrono_state_utils.h"

namespace seastack::chrono {

void BuildSystemStateFromChronoBodies(
    const std::vector<std::shared_ptr<::chrono::ChBody>>& bodies,
    seastack::hydro::SystemState& out_state) {
    out_state.bodies.clear();
    out_state.bodies.reserve(bodies.size());
    
    for (const auto& body : bodies) {
        seastack::hydro::BodyState body_state;
        
        // Use Chrono's .eigen() methods for zero-cost conversion to Eigen types
        body_state.position         = body->GetPos().eigen();
        body_state.orientation_rpy  = body->GetRot().GetCardanAnglesXYZ().eigen();
        body_state.linear_velocity  = body->GetPosDt().eigen();
        body_state.angular_velocity = body->GetAngVelParent().eigen();
        
        out_state.bodies.push_back(body_state);
    }
}

}  // namespace seastack::chrono

