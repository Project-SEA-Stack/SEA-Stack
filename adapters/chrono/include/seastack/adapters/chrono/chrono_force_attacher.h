/*********************************************************************
 * @file  chrono_force_attacher.h
 * @brief ChronoForceAttacher: Chrono force-callback plumbing and caching.
 *
 * MAIN TYPES:
 *   - ChronoForceAttacher: Registers per-DOF Chrono ChFunction callbacks
 *     on bodies, caches forces per timestep, and detects divergence.
 *
 * ROLE: Pure Chrono-adapter responsibility — registers force callbacks on
 * Chrono bodies and caches forces per timestep. Force computation is
 * delegated to the caller-supplied ForceEvaluator callback.
 *
 * LIFETIME CONTRACT: The ChronoForceAttacher instance must outlive all
 * Chrono force callback invocations. In practice, this is guaranteed
 * because the attacher is owned by HydroSystem, which owns the Chrono
 * body references. Destroying the attacher while Chrono still holds
 * force callbacks would produce dangling pointer dereferences.
 *
 * KEY ASSUMPTIONS: 6 DOF per body, bodies named "body1", "body2", etc.
 *********************************************************************/

#ifndef SEASTACK_ADAPTERS_CHRONO_FORCE_ATTACHER_H
#define SEASTACK_ADAPTERS_CHRONO_FORCE_ATTACHER_H

#include <seastack/core/types.h>
#include <seastack/core/system_state.h>
#include <seastack/hydro/hydro_forces.h>

#include <functional>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChForce.h>

namespace seastack::chrono {

class ChronoForceAttacher;

using seastack::hydro::kDofPerBody;

/// Alias for the solver-agnostic profiling stats from HydroForces.
using HydroProfileStats = seastack::hydro::HydroForcesProfileStats;

/**
 * @brief Numerical blow-up thresholds for ChronoForceAttacher.
 *
 * Magnitude checks (position, velocity, roll/pitch, force) run only when
 * `enabled` is true. Non-finite (NaN/Inf) state and force values always trip
 * divergence — that path is not user-disableable.
 *
 * A threshold of 0 or negative means "no limit" for that magnitude entry.
 * Defaults match the historical compile-time constants (90 deg roll/pitch).
 */
struct DivergenceLimits {
    bool enabled = true;
    double max_position_m = 200.0;
    double max_velocity_ms = 20.0;
    double max_ang_vel_rads = 5.0;
    double max_roll_pitch_rad = 1.5708;  ///< ~90 deg; YAML exposes degrees
    double max_force_magnitude = 1.0e10;
};

// ═══════════════════════════════════════════════════════════════════════════════
//
//  ChronoForceAttacher — Chrono callback plumbing and per-timestep caching
//
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Manages Chrono force callbacks, per-timestep caching, and divergence detection.
 *
 * Registers ChFunction-derived callback objects on Chrono bodies. Each
 * timestep, extracts body state, checks for divergence, delegates to the
 * ForceEvaluator callback for actual force computation, flattens the result,
 * and caches it.
 *
 * ## Body Indexing Convention
 *
 *   Chrono bodies must be named "body1", "body2", etc. The numeric suffix is
 *   parsed to determine body order. The callback interface uses 1-based body
 *   indexing to match Chrono's naming convention.
 *
 * ## Lifetime
 *
 *   This object must outlive all Chrono force evaluations that reference its
 *   callbacks. The internal ChFunction objects hold a non-owning pointer back
 *   to this attacher. This is acceptable because the attacher is owned by
 *   HydroSystem, which also owns the Chrono body references, but callers
 *   must not destroy the attacher while the simulation is still stepping.
 */
class ChronoForceAttacher {
  public:
    /// Callback signature for force computation. Takes simulation time,
    /// returns per-body generalized forces.
    using ForceEvaluator = std::function<seastack::hydro::BodyForces(double time)>;

    /**
     * @brief Construct and optionally register force callbacks on Chrono bodies.
     *
     * @param bodies Chrono bodies to attach forces to.
     * @param evaluator Callback invoked once per new timestep to compute forces.
     * @param attach_forces_to_bodies When false, no ChForce objects are added; `CoordinateFuncForBody`
     *        returns 0 without calling the evaluator (added-mass-only / KKT isolation experiments).
     * @param limits Divergence magnitude thresholds (defaults match historical behaviour).
     */
    ChronoForceAttacher(std::vector<std::shared_ptr<::chrono::ChBody>> bodies,
                        ForceEvaluator evaluator,
                        bool attach_forces_to_bodies = true,
                        DivergenceLimits limits = {});

    ~ChronoForceAttacher();

    ChronoForceAttacher(const ChronoForceAttacher&) = delete;
    ChronoForceAttacher& operator=(const ChronoForceAttacher&) = delete;

    /**
     * @brief Compute or retrieve force for a specific body and DOF.
     *
     * Called by Chrono's ChForce callbacks. Forces are computed once per
     * timestep via the ForceEvaluator and cached.
     *
     * @param b Body index (1-based: body1 -> b=1).
     * @param dof_index DOF index (0-based: 0=surge, ..., 5=yaw).
     * @return Force component [N or N.m].
     */
    double CoordinateFuncForBody(int b, int dof_index);

    bool HasDiverged() const { return diverged_; }

  private:
    struct BodyForceCallbacks;

    std::vector<std::shared_ptr<::chrono::ChBody>> bodies_;
    int num_bodies_;
    ForceEvaluator force_evaluator_;

    std::vector<BodyForceCallbacks> callbacks_;
    std::vector<double> total_force_;
    double prev_time_;

    const bool attach_forces_to_bodies_;

    bool diverged_ = false;
    bool divergence_logged_ = false;
    DivergenceLimits limits_;

    seastack::hydro::SystemState cached_state_;

    static int ParseBodyNumber(const std::string& body_name);
    void CheckBodyStateDivergence();
    void CheckForceValidity();
};

}  // namespace seastack::chrono

#endif  // SEASTACK_ADAPTERS_CHRONO_FORCE_ATTACHER_H
