/*********************************************************************
 * @file  hydro_model_builder.cpp
 * @brief Implements HydroModelBuilder and HydroModel.
 *
 * The builder is a thin orchestrator: it delegates to existing helpers
 * (ComputeEquilibrium, ComputeRirfWidthVector), ComponentSampler, and
 * component constructors.
 * Validation checks are centralized in Build().
 *
 * Nonlinear hydrostatics: OBJ meshes are converted from BEM frame to body CG
 * frame (subtract HydroData::cg per body) before volume calculators are built;
 * see docs/nonlinear_hydrostatics_coordinate_frames.md.
 *********************************************************************/

#include <seastack/hydro/hydro_model_builder.h>
#include <seastack/hydro/hydro_setup_helpers.h>
#include <seastack/hydro/excitation_transfer.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/force_components/hydrostatics_component.h>
#include <seastack/hydro/force_components/nonlinear_hydrostatics_component.h>
#include <seastack/hydro/force_components/radiation_component.h>
#include <seastack/hydro/force_components/radiation_ss_component.h>
#include <seastack/hydro/force_components/excitation_component.h>
#include <seastack/hydro/force_components/excitation_irf_component.h>
#include <seastack/hydro/force_components/damping_component.h>
#include <seastack/hydro/geometry/mesh_io.h>
#include <seastack/hydro/geometry/submerged_volume.h>
#include <seastack/infra/logging.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <sstream>

namespace seastack::hydro {

namespace {

/// True when the wave exposes multiple components with distinct propagation
/// headings (direction [rad]). Used by kAuto excitation selection: IRF
/// convolution is only valid for long-crested (single-heading) irregular seas.
bool WaveComponentsHaveMultipleDistinctHeadings(const WaveBase* wave) {
    if (!wave) {
        return false;
    }
    const auto* comps = wave->GetWaveComponents();
    if (!comps || comps->size() <= 1) {
        return false;
    }
    const double c0 = std::cos((*comps)[0].direction);
    const double s0 = std::sin((*comps)[0].direction);
    constexpr double tol = 1e-5;
    for (std::size_t i = 1; i < comps->size(); ++i) {
        const double c1 = std::cos((*comps)[i].direction);
        const double s1 = std::sin((*comps)[i].direction);
        if (std::abs(c1 - c0) > tol || std::abs(s1 - s0) > tol) {
            return true;
        }
    }
    return false;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// HydroModel implementation
// ═══════════════════════════════════════════════════════════════════════

HydroModel::HydroModel(std::unique_ptr<HydroData> data,
                       std::shared_ptr<WaveBase> wave,
                       std::unique_ptr<HydroForces> forces)
    : data_(std::move(data)),
      wave_(std::move(wave)),
      forces_(std::move(forces)) {}

HydroModel::~HydroModel() = default;
HydroModel::HydroModel(HydroModel&&) noexcept = default;
HydroModel& HydroModel::operator=(HydroModel&&) noexcept = default;

BodyForces HydroModel::Evaluate(const SystemState& state, double time) {
    return forces_->Evaluate(state, time);
}

HydroForces& HydroModel::GetForces() { return *forces_; }
const HydroForces& HydroModel::GetForces() const { return *forces_; }
const HydroData& HydroModel::GetData() const { return *data_; }
std::shared_ptr<WaveBase> HydroModel::GetWave() const { return wave_; }
int HydroModel::num_bodies() const { return forces_->num_bodies(); }

// ═══════════════════════════════════════════════════════════════════════
// HydroModelBuilder — fluent setters
// ═══════════════════════════════════════════════════════════════════════

HydroModelBuilder& HydroModelBuilder::FromHydroData(HydroData data) {
    data_ = std::make_unique<HydroData>(std::move(data));
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithSeaState(SeaStateDefinition def) {
    sea_state_def_ = std::move(def);
    wave_.reset();
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithWave(
    std::shared_ptr<WaveBase> wave) {
    wave_ = std::move(wave);
    sea_state_def_.reset();
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithBodyHydrostatics(
    std::vector<BodyHydrostaticsConfig> configs) {
    body_hydrostatics_ = std::move(configs);
    return *this;
}

HydroModelBuilder& HydroModelBuilder::EnableHydrostatics() {
    legacy_enable_hydrostatics_ = true;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::EnableNonlinearHydrostatics() {
    legacy_enable_nonlinear_hydrostatics_ = true;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithBodyMeshFiles(
    const std::vector<std::string>& mesh_paths) {
    legacy_body_mesh_paths_ = mesh_paths;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::EnableRadiation() {
    enable_radiation_ = true;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::EnableExcitation() {
    enable_excitation_ = true;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithExcitationMethod(
    ExcitationMethod method) {
    excitation_method_ = method;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithExcitationInterpolation(
    ExcitationInterpolation interp) {
    excitation_interpolation_ = interp;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithRadiationMethod(
    RadiationMethod method) {
    radiation_method_ = method;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithRadiationOptions(
    const RadiationKernelProcessing& opts) {
    kernel_processing_ = opts;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithStateSpaceOptions(
    const StateSpaceOptions& opts) {
    state_space_opts_ = opts;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithRadiationTruncationTime(
    double seconds) {
    radiation_truncation_time_ = seconds;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithExcitationTruncationTime(
    double seconds) {
    excitation_truncation_time_ = seconds;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithRampDuration(double seconds) {
    ramp_duration_ = seconds;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithLinearDamping(
    const std::vector<std::array<double, 6>>& per_body) {
    linear_damping_ = per_body;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithQuadraticDamping(
    const std::vector<std::array<double, 6>>& per_body) {
    quadratic_damping_ = per_body;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithGravity(
    const Eigen::Vector3d& gravity) {
    gravity_override_ = gravity;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::WithDiagnosticsOutputDir(
    const std::string& dir) {
    diagnostics_output_dir_ = dir;
    return *this;
}

HydroModelBuilder& HydroModelBuilder::AddComponent(
    std::unique_ptr<IHydroForceComponent> component) {
    extra_components_.push_back(std::move(component));
    return *this;
}

// ═══════════════════════════════════════════════════════════════════════
// Build — validation + assembly
// ═══════════════════════════════════════════════════════════════════════

HydroModel HydroModelBuilder::Build() {
    // ── 1. Validate data ────────────────────────────────────────────
    if (!data_) {
        throw std::runtime_error(
            "HydroModelBuilder::Build(): no hydrodynamic data provided. "
            "Call FromHydroData() before Build().");
    }

    const int num_bodies = data_->num_bodies();
    if (num_bodies <= 0) {
        throw std::runtime_error(
            "HydroModelBuilder::Build(): HydroData contains no bodies.");
    }

    // ── 2. Resolve per-body hydrostatics config ───────────────────
    // Legacy wrappers fill body_hydrostatics_ from convenience flags if
    // WithBodyHydrostatics() was not called directly.
    if (body_hydrostatics_.empty()) {
        if (legacy_enable_nonlinear_hydrostatics_) {
            body_hydrostatics_.resize(num_bodies);
            for (int b = 0; b < num_bodies; ++b) {
                body_hydrostatics_[b].model = HydrostaticsModel::kNonlinear;
                if (b < static_cast<int>(legacy_body_mesh_paths_.size())) {
                    body_hydrostatics_[b].mesh_file = legacy_body_mesh_paths_[b];
                }
            }
        } else if (legacy_enable_hydrostatics_) {
            body_hydrostatics_.resize(num_bodies);
            // All default to kLinear — no further action needed.
        } else {
            // Empty-vector fallback: default to linear for all bodies.
            body_hydrostatics_.resize(num_bodies);
        }
    }

    if (static_cast<int>(body_hydrostatics_.size()) != num_bodies) {
        std::ostringstream oss;
        oss << "HydroModelBuilder::Build(): body_hydrostatics has "
            << body_hydrostatics_.size() << " entries but HydroData has "
            << num_bodies << " bodies.";
        throw std::runtime_error(oss.str());
    }

    // body_hydrostatics_ is always populated at this point (auto-filled if empty)
    const bool has_any_hydrostatics = !body_hydrostatics_.empty();

    // ── 2a. Validate component selection ────────────────────────────
    const bool has_any_standard =
        has_any_hydrostatics || enable_radiation_ || enable_excitation_;
    if (!has_any_standard && extra_components_.empty()) {
        throw std::runtime_error(
            "HydroModelBuilder::Build(): no force components enabled. "
            "Call at least one of EnableHydrostatics(), EnableNonlinearHydrostatics(), "
            "WithBodyHydrostatics(), EnableRadiation(), EnableExcitation(), or AddComponent().");
    }

    // ── 2b. Canonical validation rules for per-body hydrostatics ────
    for (int b = 0; b < num_bodies; ++b) {
        const auto& hs = body_hydrostatics_[b];
        if (hs.model == HydrostaticsModel::kNonlinear && hs.mesh_file.empty()) {
            throw std::runtime_error(
                "HydroModelBuilder::Build(): body " + std::to_string(b) +
                " has hydrostatics model = nonlinear but no mesh_file is specified. "
                "Each nonlinear body requires an explicit OBJ mesh path.");
        }
        if (hs.model == HydrostaticsModel::kLinear && !hs.mesh_file.empty()) {
            LOG_WARNING("WARNING: body " << b
                        << " has hydrostatics model = linear but mesh_file is set ('"
                        << hs.mesh_file << "'). The mesh_file will be ignored.");
        }
    }

    // ── 3. Validate and prepare wave ────────────────────────────────
    bool has_wave_def = sea_state_def_.has_value();
    bool has_wave = wave_ != nullptr || has_wave_def;

    if (enable_excitation_ && !has_wave) {
        throw std::runtime_error(
            "HydroModelBuilder::Build(): excitation is enabled but no wave "
            "was configured. Call WithSeaState() or WithWave() before Build().");
    }

    std::shared_ptr<WaveBase> wave = wave_;
    if (sea_state_def_.has_value()) {
        // Work on a local copy to keep Build() idempotent.
        auto def = sea_state_def_.value();
        const bool use_file_water_depth = (def.depth == 0.0);
        if (def.depth == 0.0) {
            def.depth = data_->GetSimulationInfo().water_depth;
        }
        constexpr double kDefaultGravity = 9.81;
        // If gravity was not explicitly set (left at 0 or the standard default),
        // use the value from HydroData.
        if (def.g == 0.0 || def.g == kDefaultGravity) {
            def.g = data_->GetSimulationInfo().g;
        }
        auto components = ComponentSampler::Build(def);
        auto ldwf = std::make_shared<LinearDirectionalWaveField>(
            std::move(components), def.depth);
        ldwf->SetNumBodies(static_cast<unsigned int>(num_bodies));
        ldwf->ApplySimulationEnvironment(data_->GetSimulationInfo(),
                                         use_file_water_depth);
        if (ramp_duration_ > 0.0) {
            ldwf->SetRampDuration(ramp_duration_);
        }
        ldwf->Initialize();
        wave = ldwf;
    }

    // Forward excitation truncation to pre-built waves
    if (wave && excitation_truncation_time_ > 0.0 && !has_wave_def) {
        wave->SetExcitationTruncationTime(excitation_truncation_time_);
    }

    // ── 4. Validate radiation configuration ─────────────────────────
    if (enable_radiation_) {
        Eigen::VectorXd rirf_time = data_->GetRIRFTimeVector();
        if (rirf_time.size() == 0) {
            throw std::runtime_error(
                "HydroModelBuilder::Build(): radiation is enabled but "
                "RIRF data is empty in the provided HydroData.");
        }
        if (radiation_method_ == RadiationMethod::kStateSpace &&
            kernel_processing_.RequiresProcessing()) {
            throw std::runtime_error(
                "HydroModelBuilder::Build(): kernel processing "
                "(smoothing/tapering) is only supported with RIRF "
                "convolution, not state-space radiation. Remove "
                "WithRadiationOptions() or switch to RIRF convolution.");
        }
    }

    // ── 5. Validate damping coefficient sizes ─────────────────────────
    if (!linear_damping_.empty() &&
        static_cast<int>(linear_damping_.size()) != num_bodies) {
        std::ostringstream oss;
        oss << "HydroModelBuilder::Build(): linear damping has "
            << linear_damping_.size() << " entries but HydroData has "
            << num_bodies << " bodies.";
        throw std::runtime_error(oss.str());
    }
    if (!quadratic_damping_.empty() &&
        static_cast<int>(quadratic_damping_.size()) != num_bodies) {
        std::ostringstream oss;
        oss << "HydroModelBuilder::Build(): quadratic damping has "
            << quadratic_damping_.size() << " entries but HydroData has "
            << num_bodies << " bodies.";
        throw std::runtime_error(oss.str());
    }

    // ── 6. Compute derived quantities ───────────────────────────────
    auto equilibrium  = ComputeEquilibrium(*data_, num_bodies);
    auto cb_minus_cg  = ComputeCbMinusCg(*data_, num_bodies);

    Eigen::Vector3d gravity;
    if (gravity_override_.has_value()) {
        gravity = gravity_override_.value();
    } else {
        gravity = Eigen::Vector3d(0.0, 0.0, -data_->GetSimulationInfo().g);
    }

    // ── 7. Assemble force components ────────────────────────────────
    std::vector<std::unique_ptr<IHydroForceComponent>> components;

    // Partition bodies by hydrostatics model
    std::vector<int> linear_bodies, nonlinear_bodies;
    for (int b = 0; b < num_bodies; ++b) {
        if (body_hydrostatics_[b].model == HydrostaticsModel::kNonlinear)
            nonlinear_bodies.push_back(b);
        else
            linear_bodies.push_back(b);
    }

    if (!linear_bodies.empty()) {
        components.push_back(
            std::make_unique<HydrostaticsComponent>(
                *data_, num_bodies, equilibrium, cb_minus_cg, gravity,
                linear_bodies));
    }

    if (!nonlinear_bodies.empty()) {
        std::vector<std::unique_ptr<geometry::ISubmergedVolumeCalculator>> calculators;
        calculators.reserve(nonlinear_bodies.size());
        for (int b : nonlinear_bodies) {
            auto mesh = geometry::LoadObjMesh(body_hydrostatics_[b].mesh_file);

            // Required frame conversion for nonlinear hydrostatics:
            // Transform mesh from BEM coordinate frame (origin at BEM reference)
            // to body-fixed CG frame (used by dynamics).
            // Chrono body.position represents CG, so mesh vertices must be expressed
            // relative to CG. HydroData::cg is the CG position in BEM coordinates,
            // so subtracting it moves the BEM-origin mesh into the CG frame.
            {
                const Eigen::VectorXd cg_v = data_->GetCGVector(b);
                if (cg_v.size() < 3) {
                    throw std::runtime_error(
                        "HydroModelBuilder::Build(): CG vector must have at least 3 "
                        "components for nonlinear hydrostatics (body index " +
                        std::to_string(b) + ").");
                }
                const Eigen::Vector3d cg = cg_v.head<3>();
                mesh.vertices.rowwise() -= cg.transpose();
            }

            calculators.push_back(
                std::make_unique<geometry::MeshSubmergedVolume>(std::move(mesh)));
        }
        double rho = data_->GetRhoVal();
        components.push_back(
            std::make_unique<NonlinearHydrostaticsComponent>(
                std::move(calculators), rho, gravity, 0.0, nonlinear_bodies));
    }

    if (enable_radiation_) {
        if (radiation_method_ == RadiationMethod::kStateSpace) {
            components.push_back(
                std::make_unique<RadiationStateSpaceComponent>(
                    *data_, num_bodies, state_space_opts_));
        } else {
            Eigen::VectorXd rirf_time = data_->GetRIRFTimeVector();
            Eigen::VectorXd rirf_width = ComputeRirfWidthVector(rirf_time);

            // Apply truncation
            if (radiation_truncation_time_ > 0.0 && rirf_time.size() > 0) {
                Eigen::Index keep = rirf_time.size();
                for (Eigen::Index i = 0; i < rirf_time.size(); ++i) {
                    if (rirf_time[i] > radiation_truncation_time_) {
                        keep = i;
                        break;
                    }
                }
                if (keep < rirf_time.size()) {
                    rirf_time  = rirf_time.head(keep).eval();
                    rirf_width = rirf_width.head(keep).eval();
                }
            }

            const int rirf_steps = static_cast<int>(rirf_time.size());
            components.push_back(
                std::make_unique<RadiationComponent>(
                    *data_, num_bodies, rirf_steps, rirf_time, rirf_width,
                    kernel_processing_, diagnostics_output_dir_));
        }
    }

    if (enable_excitation_) {
        // Still water: no incident wave → wave excitation is identically zero; skip
        // ExcitationComponent / ExcitationIrfComponent (avoids FD/IRF setup).
        const bool skip_wave_excitation =
            wave && wave->GetWaveMode() == WaveMode::kNoWave;
        if (!skip_wave_excitation) {
        ExcitationMethod resolved = excitation_method_;
        if (resolved == ExcitationMethod::kAuto) {
            // Regular waves use frequency-domain. Long-crested irregular seas
            // use excitation IRF convolution (single incident heading).  Irregular
            // seas with multiple headings (directional spreading, bimodal with
            // different mean directions, etc.) require frequency-domain transfer
            // functions — IRF data in H5 is per-heading and cannot be convolved
            // with a single kernel against the total eta.
            if (wave && wave->GetWaveMode() == WaveMode::kIrregular) {
                if (WaveComponentsHaveMultipleDistinctHeadings(wave.get())) {
                    resolved = ExcitationMethod::kFrequencyDomain;
                } else {
                    resolved = ExcitationMethod::kIrfConvolution;
                }
            } else {
                resolved = ExcitationMethod::kFrequencyDomain;
            }
        }

        {
            const bool multi_heading = WaveComponentsHaveMultipleDistinctHeadings(wave.get());
            const auto* wc           = wave ? wave->GetWaveComponents() : nullptr;
            const std::size_t n_comp = wc ? wc->size() : 0;
            const char* method_str =
                (resolved == ExcitationMethod::kIrfConvolution) ? "irf_convolution" : "frequency_domain";
            const char* sel =
                (excitation_method_ == ExcitationMethod::kAuto) ? "auto" : "explicit";
            LOG_DEBUG("Hydro excitation: method=" << method_str << " selection=" << sel
                                                  << " multi_heading_wave=" << (multi_heading ? "true" : "false")
                                                  << " n_wave_components=" << n_comp);
        }

        if (resolved == ExcitationMethod::kIrfConvolution) {
            double exc_ramp = ramp_duration_;
            if (exc_ramp <= 0.0 && wave) {
                exc_ramp = wave->GetRampDuration();
            }
            components.push_back(
                std::make_unique<ExcitationIrfComponent>(
                    *data_, wave, num_bodies, excitation_truncation_time_,
                    exc_ramp));
        } else {
            // Frequency-domain: build ExcitationComponent with owned TF
            // data when the wave exposes discrete components (preferred path).
            const auto* wave_comps = wave ? wave->GetWaveComponents() : nullptr;
            if (wave_comps && !wave_comps->empty() &&
                !data_->GetRegularWaveInfos().empty() &&
                data_->GetRegularWaveInfos()[0].freq_list.size() > 0) {
                const auto& comps = *wave_comps;

                ExcitationTFData tf = InterpolateExcitationTransfer(
                    comps,
                    data_->GetRegularWaveInfos(),
                    data_->GetSimulationInfo().wave_directions,
                    excitation_interpolation_);

                ExcitationComponent::WaveComponentData wcd;
                const Eigen::Index nc = static_cast<Eigen::Index>(comps.size());
                wcd.amplitudes.resize(nc);
                wcd.omegas.resize(nc);
                wcd.phases.resize(nc);
                for (Eigen::Index i = 0; i < nc; ++i) {
                    wcd.amplitudes[i] = comps[i].amplitude;
                    wcd.omegas[i]     = comps[i].omega;
                    wcd.phases[i]     = comps[i].phase;
                }

                double ramp = ramp_duration_;
                if (ramp <= 0.0 && wave) ramp = wave->GetRampDuration();

                components.push_back(
                    std::make_unique<ExcitationComponent>(
                        std::move(wcd), std::move(tf.re), std::move(tf.im),
                        num_bodies, ramp));
            } else {
                std::ostringstream oss;
                oss << "HydroModelBuilder::Build(): frequency-domain excitation requires "
                    << "a wave with discrete components and H5 regular-wave data "
                    << "with a non-empty frequency grid (freq_list). ";
                if (!wave_comps || wave_comps->empty()) {
                    oss << "This wave model has no discrete component list (e.g. eta-table import). "
                        << "Frequency-domain excitation requires LinearDirectionalWaveField components "
                        << "and H5 regular-wave data; use ExcitationMethod::kIrfConvolution or kAuto for IRF.";
                } else {
                    oss << "Regular-wave excitation data in HydroData is missing or has "
                        << "no frequencies; check the BEM/HDF5 file.";
                }
                throw std::runtime_error(oss.str());
            }
        }
        }  // !skip_wave_excitation
    }

    // Optional body damping (linear + quadratic, skip if all coefficients are zero)
    {
        auto has_nonzero_coeffs = [](const std::vector<std::array<double, 6>>& v) {
            for (const auto& body_coeffs : v)
                for (double c : body_coeffs)
                    if (c != 0.0) return true;
            return false;
        };
        bool has_linear    = !linear_damping_.empty() && has_nonzero_coeffs(linear_damping_);
        bool has_quadratic = !quadratic_damping_.empty() && has_nonzero_coeffs(quadratic_damping_);
        if (has_linear || has_quadratic) {
            // Ensure linear coefficients vector exists (DampingComponent requires it).
            auto lin = has_linear
                ? linear_damping_
                : std::vector<std::array<double, 6>>(num_bodies, {0, 0, 0, 0, 0, 0});
            components.push_back(
                std::make_unique<DampingComponent>(lin, has_quadratic ? quadratic_damping_
                                                                     : std::vector<std::array<double, 6>>{}));
        }
    }

    // Append any externally-added components (e.g. mooring)
    for (auto& comp : extra_components_) {
        components.push_back(std::move(comp));
    }
    extra_components_.clear();

    // ── 8. Construct HydroForces and package into HydroModel ────────
    auto forces = std::make_unique<HydroForces>(
        num_bodies, std::move(components));

    return HydroModel(std::move(data_), std::move(wave), std::move(forces));
}

}  // namespace seastack::hydro
