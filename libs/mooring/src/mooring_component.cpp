/*********************************************************************
 * @file  mooring_component.cpp
 * @brief Implementation of MooringComponent.
 *********************************************************************/

#include <seastack/mooring/mooring_component.h>
#include <seastack/mooring/moordyn_wrapper.h>

#include <seastack/infra/debug_trace.h>

#include <cmath>

namespace seastack::mooring {

MooringComponent::MooringComponent(std::unique_ptr<MoorDynWrapper> wrapper)
    : wrapper_(std::move(wrapper)) {
    if (!wrapper_) {
        throw std::invalid_argument(
            "MooringComponent requires a non-null MoorDynWrapper");
    }
}

void MooringComponent::Compute(const SystemState& state,
                               double time,
                               BodyForces& inout_forces) {
    SEASTACK_TRACE_ONCE(
        "MooringComponent::Compute entered at t=" + std::to_string(time));

    if (!wrapper_ || !wrapper_->IsInitialized()) {
        return;
    }

    // Tolerance for detecting "same time" (HHT Newton re-evaluations).
    constexpr double kTimeTol = 1.0e-12;
    constexpr double kInitialDt = 1.0e-6;
    const bool same_time = has_cached_forces_ &&
                           std::abs(time - last_time_) < kTimeTol;

    if (same_time) {
        ApplyCachedForces(inout_forces);
        return;
    }

    double dt = 0.0;
    if (last_time_ < 0.0) {
        dt = kInitialDt;
    } else {
        dt = time - last_time_;
    }
    last_time_ = time;

    if (dt <= 0.0) {
        return;
    }

    SEASTACK_TRACE_ONCE(
        "MooringComponent::Compute assigning cached_forces_ for " +
        std::to_string(inout_forces.size()) + " bodies");

    // Step MoorDyn into a temporary buffer so we can cache the pure
    // mooring contribution separately from the accumulated inout_forces.
    // Avoid vector::assign with a temporary Eigen vector here; in the RM3
    // post-init path on MSVC that pattern was triggering heap corruption.
    cached_forces_.resize(inout_forces.size());
    for (auto& f : cached_forces_) {
        f.setZero();
    }

    SEASTACK_TRACE_ONCE(
        "MooringComponent::Compute finished cached_forces_ assignment");

    SEASTACK_TRACE_ONCE(
        "MooringComponent::Compute entering first wrapper_->Step at t=" +
        std::to_string(time) + ", dt=" + std::to_string(dt));
    wrapper_->Step(state, time, dt, cached_forces_);

    SEASTACK_TRACE_ONCE(
        "MooringComponent::Compute returned from first wrapper_->Step at t=" +
        std::to_string(time));
    has_cached_forces_ = true;

    ApplyCachedForces(inout_forces);
}

std::vector<seastack::viz::MooringLineVizData> MooringComponent::GetMooringLineStates() const {
    if (!wrapper_ || !wrapper_->IsInitialized()) {
        return {};
    }
    return wrapper_->GetLineStates();
}

void MooringComponent::ApplyCachedForces(BodyForces& inout_forces) const {
    for (size_t i = 0; i < cached_forces_.size(); ++i) {
        inout_forces[i] += cached_forces_[i];
    }
}

}  // namespace seastack::mooring
