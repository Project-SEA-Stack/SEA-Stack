/*********************************************************************
 * @file  hydro_system.cpp
 * @brief Implementation of the slimmed HydroSystem façade.
 *
 * HydroSystem composes HydroSystemConfig (settings) and
 * ChronoForceAttacher (Chrono callbacks). It owns the HydroModel and
 * ChronoHydroCoupler for force computation, lazily constructed via
 * HydroModelBuilder.
 *********************************************************************/

#include "seastack/adapters/chrono/hydro_system.h"

#include <seastack/adapters/chrono/chrono_force_attacher.h>
#include <seastack/adapters/chrono/chrono_coupler.h>
#include <seastack/core/system_state.h>
#include <seastack/hydro/force_components/nonlinear_froude_krylov_component.h>
#include <seastack/hydro/force_components/nonlinear_hydrostatics_component.h>
#include <seastack/hydro/force_components/radiation_ss_component.h>
#include <seastack/hydro/hydro_forces.h>
#include <seastack/hydro/hydro_model_builder.h>
#include <seastack/hydro/radiation/bem_frequency_match.h>
#include <seastack/hydro/waves/wave_base.h>
#include <seastack/core/math_constants.h>
#include <seastack/hydro_io/h5_reader.h>
#include <seastack/infra/debug_trace.h>
#include <seastack/infra/logging.h>

#include <cmath>
#include <iomanip>
#include <optional>
#include <stdexcept>
#include <string>

#include <chrono/physics/ChLoadHydrodynamics.h>
#include <chrono/physics/ChSystemNSC.h>

#ifdef SEASTACK_HAVE_MOORDYN
#include <seastack/mooring/mooring_component.h>
#include <seastack/mooring/moordyn_wrapper.h>
#endif
#include "chrono_state_utils.h"

namespace seastack::chrono {

using ::chrono::ChBody;
using seastack::hydro_io::H5FileInfo;
using seastack::hydro::WaveBase;

HydroSystem::~HydroSystem() = default;

void HydroSystem::ThrowIfModelConstructed(const char* method_name) const {
    if (model_constructed_) {
        throw std::logic_error(
            std::string("HydroSystem::") + method_name +
            "() called after the hydro model has been constructed. "
            "All configuration must be set before the first timestep.");
    }
}

bool HydroSystem::HasDiverged() const {
    return force_attacher_->HasDiverged();
}

double HydroSystem::CoordinateFuncForBody(int body, int dof) {
    return force_attacher_->CoordinateFuncForBody(body, dof);
}

HydroSystem::HydroSystem(std::vector<std::shared_ptr<ChBody>> user_bodies,
                         std::string h5_file_name,
                         std::shared_ptr<WaveBase> waves,
                         const HydroCouplingOptions& coupling)
    : bodies_(user_bodies),
      num_bodies_(static_cast<int>(bodies_.size())),
      file_info_(H5FileInfo(h5_file_name, num_bodies_).ReadH5Data()),
      hydro_model_(nullptr),
      chrono_coupler_(nullptr) {

    // Create force attacher with a callback to this->EvaluateForces.
    auto evaluator = [this](double time) -> seastack::hydro::BodyForces {
        return EvaluateForces(time);
    };
    force_attacher_ = std::make_unique<ChronoForceAttacher>(
        bodies_, std::move(evaluator), coupling.attach_hydro_force_callbacks,
        coupling.divergence);

    // Apply infinite-frequency added mass via Chrono's built-in ChLoadHydrodynamics.
    // Replaced by A(w) in UpdateAddedMassLoad() for constant-coefficient radiation.
    if (num_bodies_ > 0) {
        const auto& body_info = file_info_.GetBodyInfos();
        std::vector<Eigen::MatrixXd> blocks;
        for (int i = 0; i < num_bodies_; i++) {
            blocks.push_back(body_info[i].inf_added_mass);
        }
        ReplaceAddedMassLoad(blocks);
        added_mass_source_ = AddedMassSource::kInfiniteFrequency;
    }

    user_waves_ = waves;
    AddWaves(user_waves_);
}

void HydroSystem::ReplaceAddedMassLoad(const std::vector<Eigen::MatrixXd>& blocks) {
    auto* system = bodies_[0]->GetSystem();
    if (added_mass_load_) {
        system->Remove(added_mass_load_);
        added_mass_load_.reset();
    }
    ::chrono::ChBodyAddedMassBlocks body_blocks;
    for (int i = 0; i < num_bodies_; i++) {
        body_blocks.push_back({bodies_[i], blocks[i]});
    }
    auto hydro_load = chrono_types::make_shared<::chrono::ChLoadHydrodynamics>(body_blocks);
    hydro_load->SetVerbose(false);
    system->Add(hydro_load);
    added_mass_load_ = hydro_load;
}

void HydroSystem::UpdateAddedMassLoad() {
    if (num_bodies_ <= 0 || !added_mass_load_) {
        return;
    }

    // Resolve the BEM grid point for constant-coefficient radiation. Any
    // invalid configuration (non-regular wave, off-grid frequency, missing
    // A(w)/B(w)) keeps A_inf here; HydroModelBuilder::Build() reports it.
    std::optional<seastack::hydro::BemFrequencyMatch> match;
    if (config_.radiation_method == seastack::hydro::RadiationMethod::kConstantCoefficients &&
        user_waves_ && user_waves_->GetWaveMode() == seastack::hydro::WaveMode::kRegular &&
        file_info_.HasFrequencyDomainRadiation()) {
        const auto* comps = user_waves_->GetWaveComponents();
        if (comps && comps->size() == 1) {
            try {
                match = seastack::hydro::MatchBemFrequency(
                    file_info_.GetBemFrequencies(), (*comps)[0].omega,
                    config_.constant_coefficient_opts.frequency_tolerance);
            } catch (const std::exception& e) {
                LOG_DEBUG("Constant-coefficient radiation: keeping A_inf for now; " << e.what());
            }
        }
    }

    if (match) {
        if (added_mass_source_ == AddedMassSource::kRegularWaveFrequency &&
            added_mass_frequency_index_ == match->index) {
            return;
        }
        const Eigen::MatrixXd a_w = file_info_.GetAddedMassAtFrequencyIndex(match->index);
        std::vector<Eigen::MatrixXd> blocks;
        for (int i = 0; i < num_bodies_; i++) {
            blocks.push_back(a_w.middleRows(kDofPerBody * i, kDofPerBody));
        }
        ReplaceAddedMassLoad(blocks);
        added_mass_source_ = AddedMassSource::kRegularWaveFrequency;
        added_mass_frequency_index_ = match->index;

        const Eigen::MatrixXd b_w = file_info_.GetRadiationDampingAtFrequencyIndex(match->index);
        const auto& body_info = file_info_.GetBodyInfos();
        LOG_INFO("Constant-coefficient radiation: regular wave T = " << std::setprecision(10)
                 << kTwoPi / match->omega_requested << " s (w = " << match->omega_requested
                 << " rad/s) matched BEM grid point " << match->index << ": T = "
                 << kTwoPi / match->omega_matched << " s (w = " << match->omega_matched
                 << " rad/s), |dw| = " << std::setprecision(3)
                 << std::abs(match->omega_matched - match->omega_requested)
                 << " rad/s (relative " << match->relative_difference << ", tolerance "
                 << config_.constant_coefficient_opts.frequency_tolerance << ").");
        for (int i = 0; i < num_bodies_; i++) {
            const int r = kDofPerBody * i;
            LOG_INFO("  " << body_info[i].body_name << " diag A(w)   = "
                     << a_w.diagonal().segment(r, kDofPerBody).transpose());
            LOG_INFO("  " << body_info[i].body_name << " diag B(w)   = "
                     << b_w.diagonal().segment(r, kDofPerBody).transpose());
            if (body_info[i].inf_added_mass.cols() >= r + kDofPerBody) {
                LOG_INFO("  " << body_info[i].body_name << " diag A_inf = "
                         << body_info[i].inf_added_mass.block(0, r, kDofPerBody, kDofPerBody)
                                .diagonal().transpose()
                         << " (NOT applied)");
            }
        }
        LOG_INFO("Constant-coefficient radiation: full coupled A(w) ("
                 << a_w.rows() << "x" << a_w.cols()
                 << ") REPLACES A_inf in the Chrono added-mass load; B(w) is applied as "
                    "the radiation force.");
    } else if (added_mass_source_ != AddedMassSource::kInfiniteFrequency) {
        const auto& body_info = file_info_.GetBodyInfos();
        std::vector<Eigen::MatrixXd> blocks;
        for (int i = 0; i < num_bodies_; i++) {
            blocks.push_back(body_info[i].inf_added_mass);
        }
        ReplaceAddedMassLoad(blocks);
        added_mass_source_ = AddedMassSource::kInfiniteFrequency;
        added_mass_frequency_index_ = -1;
        LOG_DEBUG("Added-mass load restored to infinite-frequency added mass A_inf.");
    }
}

void HydroSystem::AddWaves(std::shared_ptr<WaveBase> waves) {
    ThrowIfModelConstructed("AddWaves");
    user_waves_ = waves;
    user_waves_->SetNumBodies(static_cast<unsigned int>(num_bodies_));

    if (config_.excitation_truncation_time > 0.0) {
        user_waves_->SetExcitationTruncationTime(config_.excitation_truncation_time);
    }

    // Sync gravity from H5; preserve wave-model water depth (YAML / sea-state may override file depth).
    const auto& sim = file_info_.GetSimulationInfo();
    user_waves_->UpdateEnvironment(sim.g, user_waves_->GetWaterDepth());

    user_waves_->Initialize();

    UpdateAddedMassLoad();
}

void HydroSystem::SetExcitationTruncationTime(double seconds) {
    ThrowIfModelConstructed("SetExcitationTruncationTime");
    config_.excitation_truncation_time = seconds;
    if (user_waves_) {
        user_waves_->SetExcitationTruncationTime(seconds);
    }
}

void HydroSystem::SetProfilingEnabled(bool enabled) {
    config_.profiling_enabled = enabled;
    if (chrono_coupler_) {
        chrono_coupler_->SetProfilingEnabled(enabled);
    }
}

// ────────────────────────────────────────────────────────────────────
// Lazy model construction
// ────────────────────────────────────────────────────────────────────

void HydroSystem::EnsureHydroForcesAndCoupler() {
    if (hydro_model_ && chrono_coupler_) {
        return;
    }

    const auto grav_ch = bodies_[0]->GetSystem()->GetGravitationalAcceleration();
    const Eigen::Vector3d gravity(grav_ch.x(), grav_ch.y(), grav_ch.z());

    seastack::hydro::HydroModelBuilder builder;
    builder.FromHydroData(file_info_)
           .WithWave(user_waves_)
           .WithGravity(gravity);

    if (!config_.body_hydrostatics.empty()) {
        builder.WithBodyHydrostatics(config_.body_hydrostatics);
    } else if (config_.legacy_enable_nonlinear) {
        builder.EnableNonlinearHydrostatics()
               .WithBodyMeshFiles(config_.legacy_body_mesh_paths);
    } else {
        builder.EnableHydrostatics();
    }

    builder.EnableRadiation()
           .EnableExcitation()
           .WithExcitationMethod(config_.excitation_method)
           .WithExcitationInterpolation(config_.excitation_interpolation)
           .WithRadiationMethod(config_.radiation_method)
           .WithRadiationOptions(config_.kernel_processing)
           .WithStateSpaceOptions(config_.state_space_opts)
           .WithConstantCoefficientOptions(config_.constant_coefficient_opts)
           .WithRadiationTruncationTime(config_.radiation_truncation_time)
           .WithExcitationTruncationTime(config_.excitation_truncation_time);

    if (!config_.linear_damping.empty()) {
        builder.WithLinearDamping(config_.linear_damping);
    }
    if (!config_.quadratic_damping.empty()) {
        builder.WithQuadraticDamping(config_.quadratic_damping);
    }

    if (!config_.diagnostics_output_dir.empty()) {
        builder.WithDiagnosticsOutputDir(config_.diagnostics_output_dir);
    }

#ifdef SEASTACK_HAVE_MOORDYN
    if (config_.moordyn_config.enabled) {
        seastack::hydro::SystemState init_state;
        seastack::chrono::BuildSystemStateFromChronoBodies(bodies_, init_state);

        auto wrapper = std::make_unique<seastack::mooring::MoorDynWrapper>(
            config_.moordyn_config.input_file, config_.moordyn_config.coupled_body_indices);
        wrapper->Initialize(init_state);
        builder.AddComponent(
            std::make_unique<seastack::mooring::MooringComponent>(std::move(wrapper)));
    }
#endif

    hydro_model_ = std::make_unique<seastack::hydro::HydroModel>(builder.Build());

    // The added-mass load must hold A(w) for constant-coefficient radiation and
    // A_inf otherwise -- never both, never the wrong one. Build() has already
    // validated the constant-coefficient configuration.
    if (num_bodies_ > 0) {
        const bool cc = config_.radiation_method ==
                        seastack::hydro::RadiationMethod::kConstantCoefficients;
        int expected_index = -1;
        if (cc) {
            expected_index = seastack::hydro::MatchBemFrequency(
                                 file_info_.GetBemFrequencies(),
                                 (*user_waves_->GetWaveComponents())[0].omega,
                                 config_.constant_coefficient_opts.frequency_tolerance)
                                 .index;
        }
        const bool consistent =
            cc ? (added_mass_source_ == AddedMassSource::kRegularWaveFrequency &&
                  added_mass_frequency_index_ == expected_index)
               : (added_mass_source_ == AddedMassSource::kInfiniteFrequency);
        if (!consistent) {
            hydro_model_.reset();
            throw std::logic_error(
                "HydroSystem: added-mass load does not match the radiation method "
                "(expected " + std::string(cc ? "A(w)" : "A_inf") +
                "). This is an internal error; please report it.");
        }
    }

    chrono_coupler_ = std::make_unique<seastack::chrono::ChronoHydroCoupler>(
        hydro_model_->GetForces(), bodies_);

    if (config_.profiling_enabled) {
        chrono_coupler_->SetProfilingEnabled(true);
    }

    model_constructed_ = true;
}

void HydroSystem::BuildModel() {
    EnsureHydroForcesAndCoupler();
}

seastack::hydro::BodyForces HydroSystem::EvaluateForces(double time) {
    EnsureHydroForcesAndCoupler();

    seastack::hydro::BodyForces body_forces = chrono_coupler_->Evaluate(
        time, per_component_capture_ ? &last_component_forces_ : nullptr);

    profile_stats_ = chrono_coupler_->GetProfileStats();

    return body_forces;
}

// ────────────────────────────────────────────────────────────────────
// Model query methods
// ────────────────────────────────────────────────────────────────────

double HydroSystem::GetLastSubmergedVolume(int body_index) const {
    if (!hydro_model_) return 0.0;
    for (const auto& comp : hydro_model_->GetForces().GetComponents()) {
        if (comp->Type() == seastack::hydro::HydroComponentType::kNonlinearHydrostatics) {
            auto* nl = dynamic_cast<const seastack::hydro::NonlinearHydrostaticsComponent*>(comp.get());
            if (nl) return nl->GetLastSubmergedVolume(body_index);
        }
    }
    return 0.0;
}

Eigen::Vector3d HydroSystem::GetLastCentreOfBuoyancy(int body_index) const {
    if (!hydro_model_) return Eigen::Vector3d::Zero();
    for (const auto& comp : hydro_model_->GetForces().GetComponents()) {
        if (comp->Type() == seastack::hydro::HydroComponentType::kNonlinearHydrostatics) {
            auto* nl = dynamic_cast<const seastack::hydro::NonlinearHydrostaticsComponent*>(comp.get());
            if (nl) return nl->GetLastCentreOfBuoyancy(body_index);
        }
    }
    return Eigen::Vector3d::Zero();
}

double HydroSystem::GetLastWettedArea(int body_index) const {
    if (!hydro_model_) return 0.0;
    for (const auto& comp : hydro_model_->GetForces().GetComponents()) {
        if (comp->Type() == seastack::hydro::HydroComponentType::kNonlinearFroudeKrylov) {
            auto* fk = dynamic_cast<const seastack::hydro::NonlinearFroudeKrylovComponent*>(comp.get());
            if (fk) return fk->GetLastWettedArea(body_index);
        }
    }
    return 0.0;
}

bool HydroSystem::HasKernelFitDiagnostics() const {
    if (config_.radiation_method != seastack::hydro::RadiationMethod::kStateSpace) {
        return false;
    }
    if (hydro_model_) {
        for (const auto& comp : hydro_model_->GetForces().GetComponents()) {
            if (comp->Type() == seastack::hydro::HydroComponentType::kRadiation) {
                auto* ss_comp = dynamic_cast<seastack::hydro::RadiationStateSpaceComponent*>(comp.get());
                if (ss_comp && ss_comp->HasDiagnostics()) {
                    return true;
                }
            }
        }
    }
    return false;
}

std::vector<seastack::hydro::KernelFitDiagnostics> HydroSystem::GetKernelFitDiagnostics() const {
    if (hydro_model_) {
        for (const auto& comp : hydro_model_->GetForces().GetComponents()) {
            if (comp->Type() == seastack::hydro::HydroComponentType::kRadiation) {
                auto* ss_comp = dynamic_cast<seastack::hydro::RadiationStateSpaceComponent*>(comp.get());
                if (ss_comp && ss_comp->HasDiagnostics()) {
                    return ss_comp->GetDiagnostics();
                }
            }
        }
    }
    return {};
}

#ifdef SEASTACK_HAVE_MOORDYN
std::vector<seastack::viz::MooringLineVizData> HydroSystem::GetMooringLineStates() const {
    if (!hydro_model_) return {};

    SEASTACK_TRACE_ONCE("HydroSystem::GetMooringLineStates first visualization/state access");

    for (const auto& comp : hydro_model_->GetForces().GetComponents()) {
        if (comp->Type() != seastack::hydro::HydroComponentType::kMooring) continue;
        auto* mooring = dynamic_cast<seastack::mooring::MooringComponent*>(comp.get());
        if (mooring) return mooring->GetMooringLineStates();
    }
    return {};
}
#endif

}  // namespace seastack::chrono
