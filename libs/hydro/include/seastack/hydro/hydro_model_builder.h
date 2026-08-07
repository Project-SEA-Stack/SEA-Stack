/*********************************************************************
 * @file  hydro_model_builder.h
 * @brief HydroModelBuilder: intent-driven construction of HydroForces.
 *
 * MAIN TYPES:
 *   - HydroModel:        Self-contained model (owns data + wave + forces)
 *   - HydroModelBuilder: Fluent builder that validates and assembles a HydroModel
 *
 * The builder encapsulates the setup ceremony (equilibrium, cb_minus_cg,
 * RIRF widths, wave init, component creation) behind a declarative API.
 * It delegates to existing helpers and component constructors — no
 * substantial computation lives here.
 *
 * Example (standalone, Chrono-free):
 *
 *   auto data = seastack::hydro_io::H5FileInfo("sphere.h5", 1).ReadH5Data();
 *
 *   SeaStateDefinition sea_state;
 *   sea_state.type = "regular";
 *   sea_state.amplitude = 0.5;
 *   sea_state.omega = 2.0 * M_PI / 8.0;
 *
 *   auto model = HydroModelBuilder()
 *       .FromHydroData(std::move(data))
 *       .WithSeaState(sea_state)
 *       .EnableHydrostatics()
 *       .EnableRadiation()
 *       .EnableExcitation()
 *       .Build();
 *
 *   BodyForces forces = model.Evaluate(state, time);
 *
 * For the expert/component-level workflow, see HydroForces directly.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_HYDRO_MODEL_BUILDER_H
#define SEASTACK_HYDRO_HYDRO_MODEL_BUILDER_H

#include <seastack/core/force_component.h>
#include <seastack/core/system_state.h>
#include <seastack/hydro/config/hydro_config.h>
#include <seastack/hydro/hydro_data.h>
#include <seastack/hydro/hydro_forces.h>
#include <seastack/hydro/excitation_types.h>
#include <seastack/hydro/radiation_types.h>
#include <seastack/hydro/waves/wave_base.h>
#include <seastack/hydro/waves/wave_component.h>

#include <Eigen/Core>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace seastack::hydro {

class HydroModelBuilder;

// ═══════════════════════════════════════════════════════════════════════
// HydroModel — self-contained hydrodynamic model
// ═══════════════════════════════════════════════════════════════════════

/// Self-contained hydrodynamic model returned by HydroModelBuilder::Build().
///
/// Owns the HydroData, wave object, and HydroForces so that all internal
/// references remain valid for the model's lifetime.  Provides the same
/// Evaluate() interface as HydroForces.
class HydroModel {
  public:
    /// Evaluate total hydrodynamic forces for the given state and time.
    BodyForces Evaluate(const SystemState& state, double time);

    /// Access the underlying HydroForces engine.
    HydroForces& GetForces();
    const HydroForces& GetForces() const;

    /// Access the hydrodynamic coefficient data.
    const HydroData& GetData() const;

    /// Access the wave model (may be null if no wave was configured).
    std::shared_ptr<WaveBase> GetWave() const;

    int num_bodies() const;

    ~HydroModel();
    HydroModel(HydroModel&&) noexcept;
    HydroModel& operator=(HydroModel&&) noexcept;

    HydroModel(const HydroModel&) = delete;
    HydroModel& operator=(const HydroModel&) = delete;

  private:
    friend class HydroModelBuilder;
    HydroModel(std::unique_ptr<HydroData> data,
               std::shared_ptr<WaveBase> wave,
               std::unique_ptr<HydroForces> forces);

    std::unique_ptr<HydroData> data_;
    std::shared_ptr<WaveBase> wave_;
    std::unique_ptr<HydroForces> forces_;
};

// ═══════════════════════════════════════════════════════════════════════
// HydroModelBuilder — fluent builder
// ═══════════════════════════════════════════════════════════════════════

class HydroModelBuilder {
  public:
    HydroModelBuilder() = default;

    // ── Data source (exactly one call required before Build) ─────────

    /// Use pre-loaded hydrodynamic coefficient data.
    /// The data is moved into the builder.
    HydroModelBuilder& FromHydroData(HydroData data);

    // ── Wave configuration (at most one) ─────────────────────────────

    /// Define a sea state from a declarative definition.
    /// Handles regular, irregular, directional, bimodal, and eta-import
    /// cases through a single unified API (preferred over legacy methods).
    HydroModelBuilder& WithSeaState(SeaStateDefinition def);

    /// Use a pre-constructed, already-initialized wave object.
    HydroModelBuilder& WithWave(std::shared_ptr<WaveBase> wave);

    // ── Force component selection ────────────────────────────────────

    /// Per-body hydrostatics configuration for the builder.
    struct BodyHydrostaticsConfig {
        HydrostaticsModel model = HydrostaticsModel::kLinear;
        std::string mesh_file;  ///< OBJ path, required when model == kNonlinear
    };

    /// Set per-body hydrostatics configuration (preferred API).
    /// One entry per body, in body order.
    HydroModelBuilder& WithBodyHydrostatics(
        std::vector<BodyHydrostaticsConfig> configs);

    /// Enable hydrostatic restoring forces (linear, from BEM stiffness matrix).
    /// Convenience wrapper: fills all bodies with kLinear.
    HydroModelBuilder& EnableHydrostatics();

    /// Enable nonlinear hydrostatic buoyancy (instantaneous submerged volume).
    /// Convenience wrapper: fills all bodies with kNonlinear.
    /// Requires body mesh files via WithBodyMeshFiles().
    ///
    /// Frame convention: OBJ vertices are assumed in the BEM coordinate frame
    /// (same reference as HydroData coefficients). Build() translates each mesh
    /// by \c -cg from HydroData so stored geometry is in the body-fixed CG frame
    /// that matches \c SystemState::body.position from the dynamics adapter.
    HydroModelBuilder& EnableNonlinearHydrostatics();

    /// Provide per-body OBJ mesh file paths for nonlinear hydrostatics.
    /// One path per body, in body order. Required when nonlinear hydrostatics
    /// is enabled; ignored otherwise.
    ///
    /// Input meshes are in BEM frame; see EnableNonlinearHydrostatics() for the
    /// BEM-to-CG transformation applied at build time.
    HydroModelBuilder& WithBodyMeshFiles(const std::vector<std::string>& mesh_paths);

    /// Enable radiation damping forces.
    HydroModelBuilder& EnableRadiation();

    /// Enable wave excitation forces (requires a wave to be configured).
    HydroModelBuilder& EnableExcitation();

    // ── Advanced configuration (all optional) ────────────────────────

    /// Select the excitation force method.
    /// Default is kIrfConvolution for backward compatibility.
    /// kFrequencyDomain is required for directional / multi-heading seas.
    HydroModelBuilder& WithExcitationMethod(ExcitationMethod method);

    /// Select the excitation transfer function interpolation method.
    /// Default is kCartesian. Use kPolar for HydroChrono-compatible regression.
    HydroModelBuilder& WithExcitationInterpolation(ExcitationInterpolation interp);

    /// Select the radiation damping method (default: RIRF convolution).
    HydroModelBuilder& WithRadiationMethod(RadiationMethod method);

    /// Configure RIRF kernel processing (smoothing / tapering).
    /// Only applies when radiation method is kRirfConvolution.
    HydroModelBuilder& WithRadiationOptions(
        const RadiationKernelProcessing& opts);

    /// Configure state-space fitting parameters.
    /// Only applies when radiation method is kStateSpace.
    HydroModelBuilder& WithStateSpaceOptions(const StateSpaceOptions& opts);

    /// Truncate radiation RIRF to [0, T] seconds. 0 = use full RIRF.
    HydroModelBuilder& WithRadiationTruncationTime(double seconds);

    /// Truncate excitation IRF to [-T, T] seconds. 0 = use full IRF.
    /// Forwarded to the wave model during Build().
    HydroModelBuilder& WithExcitationTruncationTime(double seconds);

    /// Set the excitation ramp duration [s]. 0 = no ramp.
    /// Applied to builder-constructed waves (WithSeaState).
    HydroModelBuilder& WithRampDuration(double seconds);

    /// Add per-body linear damping: F_i = -B_lin_i * v_i.
    /// One entry per body, each with 6 DOF coefficients.
    /// Units: N·s/m (translational) or N·m·s/rad (rotational).
    HydroModelBuilder& WithLinearDamping(
        const std::vector<std::array<double, 6>>& per_body);

    /// Add per-body quadratic damping: F_i = -B_quad_i * v_i * |v_i|.
    /// One entry per body, each with 6 DOF coefficients.
    /// Units: N·s²/m² (translational) or N·m·s²/rad² (rotational).
    HydroModelBuilder& WithQuadraticDamping(
        const std::vector<std::array<double, 6>>& per_body);

    /// Override gravity vector (default: (0, 0, -g) from HydroData).
    HydroModelBuilder& WithGravity(const Eigen::Vector3d& gravity);

    /// Set directory for diagnostics output (kernel CSVs, etc.).
    HydroModelBuilder& WithDiagnosticsOutputDir(const std::string& dir);

    /// Append an externally-created force component.
    /// Use this for custom or adapter-specific components (e.g. mooring).
    HydroModelBuilder& AddComponent(
        std::unique_ptr<IHydroForceComponent> component);

    /// Reserve extra body slots in the force buffer / SystemState for auxiliary
    /// bodies that participate in mooring only (no BEM forces). These are
    /// appended after the hydrodynamic bodies. Used by the Chrono adapter to let
    /// MoorDyn couple non-hydro bodies (e.g. a vehicle chassis). Default 0.
    HydroModelBuilder& WithAuxiliaryBodyCount(int n);

    // ── Build ────────────────────────────────────────────────────────

    /// Validate configuration and build a self-contained HydroModel.
    ///
    /// @throws std::runtime_error on invalid or incomplete configuration:
    ///   - No HydroData provided
    ///   - No force components enabled
    ///   - Excitation enabled without a wave
    ///   - Radiation enabled but RIRF data is empty
    ///   - Body count mismatch between data and damping coefficients
    ///   - State-space radiation with RIRF-only kernel processing
    HydroModel Build();

  private:
    // Data
    std::unique_ptr<HydroData> data_;

    // Wave (at most one is set)
    std::optional<SeaStateDefinition> sea_state_def_;
    std::shared_ptr<WaveBase> wave_;

    // Component flags
    bool enable_radiation_    = false;
    bool enable_excitation_   = false;

    // Per-body hydrostatics configuration (filled by WithBodyHydrostatics()
    // or by the legacy convenience wrappers EnableHydrostatics / EnableNonlinearHydrostatics).
    // If empty at Build() time, auto-filled with kLinear for all bodies.
    std::vector<BodyHydrostaticsConfig> body_hydrostatics_;

    // Legacy convenience flags (used by old wrappers to defer filling body_hydrostatics_
    // until Build() when body count is known).
    bool legacy_enable_hydrostatics_ = false;
    bool legacy_enable_nonlinear_hydrostatics_ = false;
    std::vector<std::string> legacy_body_mesh_paths_;

    // Advanced settings
    ExcitationMethod excitation_method_ =
        ExcitationMethod::kAuto;
    ExcitationInterpolation excitation_interpolation_ =
        ExcitationInterpolation::kCartesian;
    RadiationMethod radiation_method_ =
        RadiationMethod::kRirfConvolution;
    RadiationKernelProcessing kernel_processing_;
    StateSpaceOptions state_space_opts_;
    double radiation_truncation_time_  = 0.0;
    double excitation_truncation_time_ = 0.0;
    double ramp_duration_              = 0.0;
    std::vector<std::array<double, 6>> linear_damping_;
    std::vector<std::array<double, 6>> quadratic_damping_;
    std::optional<Eigen::Vector3d> gravity_override_;
    std::string diagnostics_output_dir_;

    // Extra components added via AddComponent()
    std::vector<std::unique_ptr<IHydroForceComponent>> extra_components_;

    // Auxiliary (mooring-only) bodies appended after the hydrodynamic bodies.
    int auxiliary_body_count_ = 0;
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_HYDRO_MODEL_BUILDER_H
