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
#include <seastack/hydro/force_components/nonlinear_hydrostatics_component.h>
#include <seastack/hydro/force_components/radiation_ss_component.h>
#include <seastack/hydro/hydro_forces.h>
#include <seastack/hydro/hydro_model_builder.h>
#include <seastack/hydro/waves/wave_base.h>
#include <seastack/hydro_io/h5_reader.h>
#include <seastack/infra/debug_trace.h>
#include <seastack/infra/logging.h>

#include <cstddef>
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
        bodies_, std::move(evaluator), coupling.attach_hydro_force_callbacks);

    // Apply infinite-frequency added mass via Chrono's built-in ChLoadHydrodynamics.
    if (num_bodies_ > 0) {
        const auto& body_info = file_info_.GetBodyInfos();
        ::chrono::ChBodyAddedMassBlocks body_blocks;
        for (int i = 0; i < num_bodies_; i++) {
            body_blocks.push_back({bodies_[i], body_info[i].inf_added_mass});
        }
        auto hydro_load = chrono_types::make_shared<::chrono::ChLoadHydrodynamics>(body_blocks);
        hydro_load->SetVerbose(false);
        bodies_[0]->GetSystem()->Add(hydro_load);
    }

    user_waves_ = waves;
    AddWaves(user_waves_);
}

void HydroSystem::AddAuxiliaryCoupledBody(std::shared_ptr<ChBody> body) {
    ThrowIfModelConstructed("AddAuxiliaryCoupledBody");
    if (!body) {
        throw std::invalid_argument(
            "HydroSystem::AddAuxiliaryCoupledBody: body must not be null");
    }
    auxiliary_bodies_.push_back(std::move(body));
}

std::vector<std::shared_ptr<ChBody>> HydroSystem::AllCoupledBodies() const {
    std::vector<std::shared_ptr<ChBody>> all = bodies_;
    all.insert(all.end(), auxiliary_bodies_.begin(), auxiliary_bodies_.end());
    return all;
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

    // Auxiliary bodies (e.g. a vehicle chassis) are appended after the hydro
    // bodies in the SystemState and the force buffer, so MoorDyn can couple them
    // even though they carry no BEM forces.
    const std::vector<std::shared_ptr<ChBody>> all_bodies = AllCoupledBodies();
    if (!auxiliary_bodies_.empty()) {
        builder.WithAuxiliaryBodyCount(static_cast<int>(auxiliary_bodies_.size()));
    }

#ifdef SEASTACK_HAVE_MOORDYN
    if (config_.moordyn_config.enabled) {
        seastack::hydro::SystemState init_state;
        seastack::chrono::BuildSystemStateFromChronoBodies(all_bodies, init_state);

        auto wrapper = std::make_unique<seastack::mooring::MoorDynWrapper>(
            config_.moordyn_config.input_file, config_.moordyn_config.coupled_body_indices);
        wrapper->Initialize(init_state);
        builder.AddComponent(
            std::make_unique<seastack::mooring::MooringComponent>(std::move(wrapper)));
    }
#endif

    hydro_model_ = std::make_unique<seastack::hydro::HydroModel>(builder.Build());

    // The coupler builds the SystemState from all coupled bodies (hydro + aux).
    // Hydro force components only read/write the first num_bodies_ entries; the
    // mooring component may write into the auxiliary slots.
    chrono_coupler_ = std::make_unique<seastack::chrono::ChronoHydroCoupler>(
        hydro_model_->GetForces(), all_bodies);

    // One force accumulator per auxiliary body, refreshed each evaluation with
    // the mooring wrench MoorDyn returns for that body.
    auxiliary_accumulators_.clear();
    auxiliary_accumulators_.reserve(auxiliary_bodies_.size());
    for (const auto& body : auxiliary_bodies_) {
        auxiliary_accumulators_.push_back(body->AddAccumulator());
    }

    if (config_.profiling_enabled) {
        chrono_coupler_->SetProfilingEnabled(true);
    }

    model_constructed_ = true;
}

seastack::hydro::BodyForces HydroSystem::EvaluateForces(double time) {
    EnsureHydroForcesAndCoupler();

    seastack::hydro::BodyForces body_forces = chrono_coupler_->Evaluate(
        time, per_component_capture_ ? &last_component_forces_ : nullptr);

    // Auxiliary bodies do not have hydro ChForce callbacks, so their mooring
    // wrench is applied directly here through per-body accumulators.
    ApplyAuxiliaryMooringForces(body_forces);

    profile_stats_ = chrono_coupler_->GetProfileStats();

    return body_forces;
}

void HydroSystem::ApplyAuxiliaryMooringForces(
    const seastack::hydro::BodyForces& body_forces) {
    for (std::size_t i = 0; i < auxiliary_bodies_.size(); ++i) {
        const std::size_t idx = static_cast<std::size_t>(num_bodies_) + i;
        if (idx >= body_forces.size() || i >= auxiliary_accumulators_.size()) {
            break;
        }
        const auto& gf = body_forces[idx];
        const auto& body = auxiliary_bodies_[i];
        const unsigned int acc = auxiliary_accumulators_[i];

        // The wrench is expressed about the body centre of mass in world axes,
        // matching MoorDynWrapper's convention.
        body->EmptyAccumulator(acc);
        body->AccumulateForce(acc,
                              ::chrono::ChVector3d(gf.force[0], gf.force[1], gf.force[2]),
                              body->GetPos(), /*local=*/false);
        body->AccumulateTorque(acc,
                               ::chrono::ChVector3d(gf.moment[0], gf.moment[1], gf.moment[2]),
                               /*local=*/false);
    }
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
