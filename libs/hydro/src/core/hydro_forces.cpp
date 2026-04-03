/*********************************************************************
 * @file  hydro_forces.cpp
 * @brief Implementation of HydroForces (Chrono-free force engine).
 *
 * HydroForces owns and orchestrates force components (hydrostatics,
 * radiation, excitation) to compute total hydrodynamic forces. It
 * operates on pure SystemState data with no Project Chrono dependencies,
 * enabling unit testing and potential use with other physics engines.
 *
 * Called by seastack::chrono::ChronoHydroCoupler, which extracts state from Chrono bodies.
 *********************************************************************/

#include <seastack/hydro/hydro_forces.h>

#include <seastack/infra/debug_trace.h>

#include <chrono>
#include <stdexcept>
#include <Eigen/Dense>

namespace {

const char* HydroComponentTypeName(seastack::hydro::HydroComponentType type) {
    switch (type) {
        case seastack::hydro::HydroComponentType::kHydrostatics:
            return "Hydrostatics";
        case seastack::hydro::HydroComponentType::kNonlinearHydrostatics:
            return "NonlinearHydrostatics";
        case seastack::hydro::HydroComponentType::kRadiation:
            return "Radiation";
        case seastack::hydro::HydroComponentType::kExcitation:
            return "Excitation";
        case seastack::hydro::HydroComponentType::kMooring:
            return "Mooring";
        case seastack::hydro::HydroComponentType::kDamping:
            return "Damping";
    }
    return "Unknown";
}

}  // namespace

namespace seastack::hydro {

HydroForces::HydroForces(int num_bodies,
                         std::vector<std::unique_ptr<IHydroForceComponent>> components)
    : num_bodies_(num_bodies),
      components_(std::move(components)),
      profile_stats_(),
      profiling_enabled_(false),
      forces_buffer_(num_bodies_) {
    // Require at least one body; otherwise hydrodynamic forces are meaningless.
    if (num_bodies_ <= 0) {
        throw std::invalid_argument(
            "HydroForces: num_bodies must be > 0 (got " + std::to_string(num_bodies_) + ")");
    }
}

BodyForces HydroForces::Evaluate(const SystemState& state, double time,
                                 std::vector<ComponentForceRecord>* per_component) {
    for (auto& force : forces_buffer_) {
        force.force.setZero();
        force.moment.setZero();
    }

    if (per_component) {
        per_component->clear();
        per_component->reserve(components_.size());
    }

    const bool trace_this_evaluation = !first_evaluation_traced_;
    first_evaluation_traced_ = true;
    int component_index = 0;
    for (const auto& component : components_) {
        if (trace_this_evaluation) {
            seastack::infra::EmitMoorDynTrace(
                "HydroForces::Evaluate entering component[" +
                std::to_string(component_index) + "]=" +
                std::string(HydroComponentTypeName(component->Type())) + " at t=" +
                std::to_string(time));
        }

        // Snapshot before Compute (only when capturing per-component)
        BodyForces snapshot;
        if (per_component) {
            snapshot = forces_buffer_;
        }

        if (profiling_enabled_) {
            auto t0 = std::chrono::steady_clock::now();
            component->Compute(state, time, forces_buffer_);
            auto t1 = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(t1 - t0).count();
            
            switch (component->Type()) {
                case HydroComponentType::kHydrostatics:
                case HydroComponentType::kNonlinearHydrostatics:
                    profile_stats_.hydrostatics_seconds += elapsed;
                    profile_stats_.hydrostatics_calls++;
                    break;
                case HydroComponentType::kRadiation:
                    profile_stats_.radiation_seconds += elapsed;
                    profile_stats_.radiation_calls++;
                    break;
                case HydroComponentType::kExcitation:
                    profile_stats_.excitation_seconds += elapsed;
                    profile_stats_.excitation_calls++;
                    break;
                case HydroComponentType::kMooring:
                    profile_stats_.mooring_seconds += elapsed;
                    profile_stats_.mooring_calls++;
                    break;
                case HydroComponentType::kDamping:
                    break;
            }
        } else {
            component->Compute(state, time, forces_buffer_);
        }

        // Delta snapshot: extract this component's contribution
        if (per_component) {
            ComponentForceRecord rec;
            rec.type = component->Type();
            rec.forces.resize(num_bodies_);
            for (int bi = 0; bi < num_bodies_; ++bi) {
                rec.forces[bi] = forces_buffer_[bi] - snapshot[bi];
            }
            per_component->push_back(std::move(rec));
        }

        if (trace_this_evaluation) {
            seastack::infra::EmitMoorDynTrace(
                "HydroForces::Evaluate returned from component[" +
                std::to_string(component_index) + "]=" +
                std::string(HydroComponentTypeName(component->Type())) + " at t=" +
                std::to_string(time));
        }
        ++component_index;
    }

    return forces_buffer_;
}

}  // namespace seastack::hydro

