/*********************************************************************
 * @file  force_component.h
 * @brief Interface for hydrodynamic force components.
 *
 * MAIN TYPES:
 *   - IHydroForceComponent: Abstract interface for force computation
 *   - HydroComponentType: Enum for profiling (Hydrostatics/Radiation/Excitation)
 *
 * ROLE: All force components implement IHydroForceComponent. HydroSystem
 * owns a collection of components and calls Compute() on each.
 *********************************************************************/

#ifndef SEASTACK_CORE_FORCE_COMPONENT_H
#define SEASTACK_CORE_FORCE_COMPONENT_H

#include <seastack/core/system_state.h>

namespace seastack::hydro {

/**
 * @brief Identifies the type of a hydrodynamic force component.
 * 
 * Used for profiling to categorize execution time by component type.
 */
enum class HydroComponentType {
    kHydrostatics,              ///< Hydrostatic restoring forces and buoyancy (linear, from BEM)
    kNonlinearHydrostatics,     ///< Nonlinear hydrostatic buoyancy (instantaneous submerged volume)
    kRadiation,                 ///< Radiation damping (RIRF convolution)
    kExcitation,                ///< Wave excitation forces
    kMooring,                   ///< Mooring forces (e.g. MoorDyn)
    kDamping,                   ///< User-specified per-DOF damping (linear + optional quadratic)
    kExternal                   ///< External force module (out-of-process / FMI bridge)
};

/**
 * @brief Interface for computing hydrodynamic forces.
 * 
 * All force components (hydrostatics, radiation, excitation) implement this
 * interface to provide a consistent way to compute forces.
 * 
 * @note Components may maintain internal state (e.g., velocity history for
 *       radiation damping), but Compute() should be side-effect-free
 *       outside the component instance.
 */
class IHydroForceComponent {
public:
    virtual ~IHydroForceComponent() = default;

    /**
     * @brief Get the type of this component.
     * 
     * Used for profiling to categorize execution time by component type.
     * 
     * @return Component type (Hydrostatics, Radiation, or Excitation)
     */
    virtual HydroComponentType Type() const = 0;

    /**
     * @brief Compute force contribution and add to inout_forces.
     * 
     * Computes the force contribution for the given system state and time,
     * adding the result to the provided forces vector.
     * 
     * `state` and `inout_forces` cover every coupled body and may be longer
     * than the component's own body count when auxiliary (mooring-only) bodies
     * are appended; see HydroForces::Evaluate for the index convention.  A
     * component must only touch the slots it owns.
     *
     * @param state Current system state (positions, velocities for all bodies)
     * @param time Current simulation time
     * @param inout_forces Force vector to add contribution to (one GeneralizedForce per body)
     */
    virtual void Compute(const SystemState& state,
                        double time,
                        BodyForces& inout_forces) = 0;
};

/// Per-component force snapshot produced by HydroForces::Evaluate when
/// per-component capture is enabled.
struct ComponentForceRecord {
    HydroComponentType type;
    BodyForces forces;  ///< Contribution of this single component (world frame)
};

}  // namespace seastack::hydro

#endif  // SEASTACK_CORE_FORCE_COMPONENT_H

