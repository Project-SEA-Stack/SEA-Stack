/*********************************************************************
 * @file  external_force_model.h
 * @brief Abstract interface for out-of-process (or other) external force models.
 *
 * Solver-agnostic: depends only on the C++ standard library.
 * Transport backends (IPC, FMI, …) implement this interface.
 * Higher-level bridges (ExternalPtoModel, ExternalForceComponent) call it.
 *
 * Units (SI) and sign conventions are application-defined for the generic
 * vector interface; the 1-DOF PTO bridge documents them explicitly.
 *
 * Protocol version and wire format: docs/extending/EXTERNAL_FORCE_MODULES.md
 *********************************************************************/
#ifndef SEASTACK_EXTERNAL_EXTERNAL_FORCE_MODEL_H
#define SEASTACK_EXTERNAL_EXTERNAL_FORCE_MODEL_H

#include <cstdint>
#include <string>
#include <vector>

namespace seastack {
namespace external {

/// Wire protocol version exchanged during the initialize handshake.
constexpr int kProtocolVersion = 1;

/// Arguments passed to IExternalForceModel::Initialize.
struct ExternalInit {
    /// Logical model kind (e.g. "pto", "body_force"). Informational.
    std::string kind;
    /// Expected number of scalar inputs per Evaluate call.
    int n_inputs = 0;
    /// Expected number of scalar outputs per Evaluate call.
    int n_outputs = 0;
    /// Nominal simulation time step [s] (may be 0 if unknown at init).
    double dt = 0.0;
    /// Opaque JSON object string for model-specific configuration.
    /// Empty string means "{}".
    std::string config_json;
};

/// Metadata returned by a successful Initialize.
struct ExternalMeta {
    std::string name;
    std::string version;
    /// Number of continuous/discrete states owned by the module (informational).
    int n_states = 0;
};

/**
 * @brief Abstract external force / control model.
 *
 * Lifecycle (v1):
 *   Initialize → (Evaluate | Reset | Commit | Rollback)* → Shutdown
 *
 * Evaluate is called once per *new* simulation time by bridges that apply
 * time-caching (matching RectifiedHydraulicPTO / ChronoForceAttacher).
 * Commit/Rollback are optional no-ops in v1 transports that freeze the force
 * within a time level; they exist so FMI and true co-simulation can slot in
 * without changing the interface.
 */
class IExternalForceModel {
  public:
    virtual ~IExternalForceModel() = default;

    /// Handshake: configure the module and return its metadata.
    /// @throws std::runtime_error on protocol or process failure.
    virtual ExternalMeta Initialize(const ExternalInit& init) = 0;

    /// Evaluate the model at the given simulation time.
    /// @param time  Simulation time [s]
    /// @param in    Input vector (size must match ExternalInit::n_inputs)
    /// @param out   Output vector resized/filled by the implementation
    ///              (size must match ExternalInit::n_outputs)
    /// @throws std::runtime_error on error replies, timeouts, or I/O failure
    virtual void Evaluate(double time,
                          const std::vector<double>& in,
                          std::vector<double>& out) = 0;

    /// Clear integrator / controller state (e.g. on decay restart).
    virtual void Reset() {}

    /// Accept the last Evaluate as the committed state for this time level.
    /// Default: no-op (v1 IPC commits by construction via time-caching).
    virtual void Commit() {}

    /// Discard tentative state from the last Evaluate (solver rollback).
    /// Default: no-op.
    virtual void Rollback() {}

    /// Tear down the module (close sockets, join process, free FMU).
    virtual void Shutdown() = 0;
};

}  // namespace external
}  // namespace seastack

#endif  // SEASTACK_EXTERNAL_EXTERNAL_FORCE_MODEL_H
