/*********************************************************************
 * @file  hydro_system.h
 * @brief HydroSystem façade: attaches hydrodynamic forces to Chrono bodies.
 *
 * MAIN TYPES:
 *   - HydroSystem: Primary hydrodynamics façade (adapter over HydroForces)
 *
 * ROLE: Thin facade composing HydroSystemConfig (settings accumulation),
 * ChronoForceAttacher (Chrono callback plumbing), and HydroModel/
 * ChronoHydroCoupler (force computation). Public API is backward-compatible.
 *
 * DECOMPOSED FROM: The original monolithic HydroSystem class.
 *   - HydroSystemConfig: Value type for all settings (hydro_system_config.h)
 *   - ChronoForceAttacher: Chrono callbacks + caching (chrono_force_attacher.h)
 *   - HydroSystem: This file — orchestration facade
 *********************************************************************/

#ifndef SEASTACK_ADAPTERS_CHRONO_HYDRO_SYSTEM_H
#define SEASTACK_ADAPTERS_CHRONO_HYDRO_SYSTEM_H

#include <seastack/config.h>

#include <seastack/adapters/chrono/hydro_system_config.h>

#include <seastack/core/types.h>
#include <seastack/hydro/hydro_data.h>
#include <seastack/hydro/waves/wave_base.h>
#include <seastack/core/system_state.h>
#include <seastack/hydro/hydro_forces.h>
#include <seastack/hydro/hydro_model_builder.h>
#include <seastack/hydro/excitation_types.h>
#include <seastack/hydro/radiation_types.h>

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <chrono/physics/ChBody.h>

#ifdef SEASTACK_HAVE_MOORDYN
#include <seastack/mooring/moordyn_config.h>
#include <seastack/core/mooring_viz_data.h>
#endif

namespace seastack::hydro {
class HydrostaticsComponent;
class ExcitationComponent;
class IHydroForceComponent;
}

namespace seastack::chrono {
class ChronoHydroCoupler;
class ChronoForceAttacher;
}

namespace seastack::chrono {

/// Selects which Chrono-side hydro couplings are active. `HydroSystem` always registers
/// `ChLoadHydrodynamics` with H5 infinite-frequency added mass when there is at least one body.
struct HydroCouplingOptions {
    /// When true (default), register per-body ChForce callbacks that evaluate SEA-Stack hydro forces.
    bool attach_hydro_force_callbacks = true;
};

/// Solver-agnostic profiling stats from HydroForces.
using HydroProfileStats = seastack::hydro::HydroForcesProfileStats;

/// Deliberate re-export so adapter consumers can write seastack::chrono::kDofPerBody.
/// Canonical definition lives in seastack::hydro (libs/core/types.h).
using seastack::hydro::kDofPerBody;

// ═══════════════════════════════════════════════════════════════════════════════
//
//  HydroSystem — Primary Hydrodynamics Façade / Adapter
//
// ═══════════════════════════════════════════════════════════════════════════════
/**
 * @brief Primary hydrodynamics façade for SEA-Stack applications.
 *
 * HydroSystem bridges three layers:
 *   1. **Chrono bodies** — The physical bodies in the Chrono simulation.
 *   2. **HydroForces (core)** — Chrono-free hydrodynamic force computation.
 *   3. **Chrono force callbacks** — ChForce functions that Chrono calls each timestep.
 *
 * Composed of:
 *   - HydroSystemConfig — accumulates settings (maps 1:1 to HydroModelBuilder)
 *   - ChronoForceAttacher — manages per-DOF force callbacks, caching, divergence
 *   - HydroModel + ChronoHydroCoupler — lazily constructed force engine
 *
 * @see HydroForces, ChronoForceAttacher, HydroSystemConfig
 */
class HydroSystem {
  public:
    HydroSystem() = delete;

    /**
     * @brief Main constructor.
     *
     * @param user_bodies List of pointers to bodies for the hydro forces.
     * @param h5_file_name Name of the h5 file where hydro data is stored.
     * @param waves WaveBase object. Defaults to NoWave if not provided.
     */
    HydroSystem(std::vector<std::shared_ptr<::chrono::ChBody>> user_bodies,
                std::string h5_file_name,
                std::shared_ptr<seastack::hydro::WaveBase> waves = std::make_shared<seastack::hydro::NoWave>(),
                const HydroCouplingOptions& coupling = {});

    ~HydroSystem();

    HydroSystem(const HydroSystem&) = delete;
    HydroSystem& operator=(const HydroSystem&) = delete;

    // ─────────────────────────────────────────────────────────────────────────
    // Wave management
    // ─────────────────────────────────────────────────────────────────────────

    void AddWaves(std::shared_ptr<seastack::hydro::WaveBase> waves);
    std::shared_ptr<seastack::hydro::WaveBase> GetWave() const { return user_waves_; }

    // ─────────────────────────────────────────────────────────────────────────
    // Configuration setters (delegate to config_)
    // ─────────────────────────────────────────────────────────────────────────

    void SetExcitationTruncationTime(double seconds);
    void SetExcitationMethod(seastack::hydro::ExcitationMethod method) {
        ThrowIfModelConstructed("SetExcitationMethod");
        config_.excitation_method = method;
    }
    void SetExcitationInterpolation(seastack::hydro::ExcitationInterpolation interp) {
        ThrowIfModelConstructed("SetExcitationInterpolation");
        config_.excitation_interpolation = interp;
    }

    void SetRadiationTruncationTime(double seconds) {
        ThrowIfModelConstructed("SetRadiationTruncationTime");
        config_.radiation_truncation_time = seconds;
    }
    void SetRadiationMethod(seastack::hydro::RadiationMethod method) {
        ThrowIfModelConstructed("SetRadiationMethod");
        config_.radiation_method = method;
    }
    void SetStateSpaceOptions(const seastack::hydro::StateSpaceOptions& opts) {
        ThrowIfModelConstructed("SetStateSpaceOptions");
        config_.state_space_opts = opts;
    }
    void SetRadiationKernelProcessing(const seastack::hydro::RadiationKernelProcessing& opts) {
        ThrowIfModelConstructed("SetRadiationKernelProcessing");
        config_.kernel_processing = opts;
    }
    void EnableRirfSmoothing() {
        ThrowIfModelConstructed("EnableRirfSmoothing");
        config_.kernel_processing = seastack::hydro::RadiationKernelProcessing::DefaultSmoothing();
    }
    void SetOutputKernelFit(bool enabled) {
        ThrowIfModelConstructed("SetOutputKernelFit");
        config_.output_kernel_fit = enabled;
    }

    void SetDiagnosticsOutputDirectory(const std::string& dir) {
        ThrowIfModelConstructed("SetDiagnosticsOutputDirectory");
        config_.diagnostics_output_dir = dir;
    }

    void SetLinearDamping(const std::vector<std::array<double, 6>>& per_body) {
        ThrowIfModelConstructed("SetLinearDamping");
        config_.linear_damping = per_body;
    }
    void SetQuadraticDamping(const std::vector<std::array<double, 6>>& per_body) {
        ThrowIfModelConstructed("SetQuadraticDamping");
        config_.quadratic_damping = per_body;
    }

    void SetBodyHydrostatics(
        std::vector<seastack::hydro::HydroModelBuilder::BodyHydrostaticsConfig> configs) {
        ThrowIfModelConstructed("SetBodyHydrostatics");
        config_.body_hydrostatics = std::move(configs);
    }
    void EnableNonlinearHydrostatics() {
        ThrowIfModelConstructed("EnableNonlinearHydrostatics");
        config_.legacy_enable_nonlinear = true;
    }
    void SetBodyMeshFiles(const std::vector<std::string>& mesh_paths) {
        ThrowIfModelConstructed("SetBodyMeshFiles");
        config_.legacy_body_mesh_paths = mesh_paths;
    }

    void SetProfilingEnabled(bool enabled);

#ifdef SEASTACK_HAVE_MOORDYN
    void SetMoorDynConfig(const seastack::mooring::MoorDynConfig& cfg) {
        ThrowIfModelConstructed("SetMoorDynConfig");
        config_.moordyn_config = cfg;
    }
    std::vector<seastack::viz::MooringLineVizData> GetMooringLineStates() const;
#endif

    // ─────────────────────────────────────────────────────────────────────────
    // Query methods (reach into model)
    // ─────────────────────────────────────────────────────────────────────────

    double GetLastSubmergedVolume(int body_index) const;
    Eigen::Vector3d GetLastCentreOfBuoyancy(int body_index) const;

    bool HasKernelFitDiagnostics() const;
    std::vector<seastack::hydro::KernelFitDiagnostics> GetKernelFitDiagnostics() const;

    // ─────────────────────────────────────────────────────────────────────────
    // Runtime status (delegate to attacher)
    // ─────────────────────────────────────────────────────────────────────────

    bool HasDiverged() const;

    /// Per-body, per-DOF force from the last cached evaluation.
    /// @param body 1-based body index.
    /// @param dof  DOF index [0..5]: surge, sway, heave, roll, pitch, yaw.
    double CoordinateFuncForBody(int body, int dof);

    HydroProfileStats GetProfileStats() const { return profile_stats_; }

    /// Enable per-component hydro force capture (for detailed export).
    void SetPerComponentCaptureEnabled(bool enabled) {
        per_component_capture_ = enabled;
    }

    /// Returns per-component force breakdown from the most recent Evaluate call.
    /// Empty unless SetPerComponentCaptureEnabled(true) was called.
    const std::vector<seastack::hydro::ComponentForceRecord>& GetLastComponentForces() const {
        return last_component_forces_;
    }

    /// Read-only access to the configuration.
    const HydroSystemConfig& config() const { return config_; }

  private:
    // Configuration accumulator
    HydroSystemConfig config_;
    bool model_constructed_ = false;

    // Chrono force-callback plumbing (owns per-body ChForce + ComponentFunc objects)
    std::unique_ptr<ChronoForceAttacher> force_attacher_;

    // Chrono bodies (retained for added-mass setup and model construction)
    std::vector<std::shared_ptr<::chrono::ChBody>> bodies_;
    int num_bodies_;

    // BEM data
    seastack::hydro::HydroData file_info_;

    // Wave model
    std::shared_ptr<seastack::hydro::WaveBase> user_waves_;

    // Lazily constructed force engine
    std::unique_ptr<seastack::hydro::HydroModel> hydro_model_;
    std::unique_ptr<seastack::chrono::ChronoHydroCoupler> chrono_coupler_;

    // Profiling data (updated after each force evaluation)
    HydroProfileStats profile_stats_;

    // Per-component force capture (detailed export mode)
    bool per_component_capture_ = false;
    std::vector<seastack::hydro::ComponentForceRecord> last_component_forces_;

    /// Throws std::logic_error if the model has already been constructed.
    void ThrowIfModelConstructed(const char* method_name) const;

    /// Lazily construct hydro_model_ and chrono_coupler_ via HydroModelBuilder.
    void EnsureHydroForcesAndCoupler();

    /// Force evaluation callback provided to ChronoForceAttacher.
    seastack::hydro::BodyForces EvaluateForces(double time);
};

}  // namespace seastack::chrono

#endif
