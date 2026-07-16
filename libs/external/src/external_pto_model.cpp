/*********************************************************************
 * @file  external_pto_model.cpp
 * @brief IPTOModel bridge: pack kinematics, time-cache, call IExternalForceModel.
 *********************************************************************/

#include <seastack/external/external_pto_model.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace seastack {
namespace external {

/// Take ownership of the transport (usually IpcExternalForceModel).
ExternalPtoModel::ExternalPtoModel(std::unique_ptr<IExternalForceModel> backend)
    : backend_(std::move(backend)) {
    if (!backend_) {
        throw std::invalid_argument("ExternalPtoModel: backend must not be null");
    }
}

ExternalPtoModel::~ExternalPtoModel() {
    try {
        Shutdown();
    } catch (...) {
    }
}

/// Choose TSDA vs RSDA channel names for the initialize handshake.
void ExternalPtoModel::SetLinkKind(ExternalPtoLinkKind kind) {
    if (initialized_) {
        throw std::runtime_error(
            "ExternalPtoModel::SetLinkKind after Initialize");
    }
    link_kind_ = kind;
}

/// Rich = 17 kinematics channels; lean = displacement + velocity only.
void ExternalPtoModel::EnableRichState(bool enable) {
    if (initialized_) {
        throw std::runtime_error(
            "ExternalPtoModel::EnableRichState after Initialize");
    }
    rich_state_ = enable;
}

/// Build ExternalInit (kind, in_names, config) and handshake with the backend.
ExternalMeta ExternalPtoModel::Initialize(double dt,
                                          const std::string& config_json) {
    ExternalInit init;
    if (rich_state_) {
        if (link_kind_ == ExternalPtoLinkKind::Rsda) {
            init.kind = "pto_rsda";
            init.in_names = ExternalPtoChannelNamesRsda();
        } else {
            init.kind = "pto_tsda";
            init.in_names = ExternalPtoChannelNamesTsda();
        }
        init.n_inputs = static_cast<int>(init.in_names.size());
    } else {
        init.kind = "pto";
        init.in_names = ExternalPtoChannelNamesLean();
        init.n_inputs = 2;
    }
    init.n_outputs = 1;
    init.dt = dt;
    init.config_json = config_json.empty() ? "{}" : config_json;
    expected_n_inputs_ = init.n_inputs;
    last_dt_ = dt;
    const ExternalMeta meta = backend_->Initialize(init);
    initialized_ = true;
    prev_time_ = -1.0;
    cached_force_ = 0.0;
    evaluate_call_count_ = 0;
    prev_velocity_ = 0.0;
    have_prev_velocity_ = false;
    return meta;
}

/// Pack the universal 2-channel layout: [displacement, velocity].
std::vector<double> ExternalPtoModel::PackLean(double displacement,
                                               double velocity) const {
    return {displacement, velocity};
}

/// Pack the 17-channel rich layout (link extras + body kinematics).
std::vector<double> ExternalPtoModel::PackRich(
    const ExternalPtoState& state) const {
    std::vector<double> in(17);
    in[0] = state.displacement;
    in[1] = state.velocity;
    if (link_kind_ == ExternalPtoLinkKind::Rsda) {
        in[2] = state.angle;
        in[3] = state.rest_angle;
    } else {
        in[2] = state.length;
        in[3] = state.rest_length;
    }
    in[4] = state.rel_accel;
    for (int i = 0; i < 3; ++i) {
        in[5 + i] = state.body1_pos[i];
        in[8 + i] = state.body1_vel[i];
        in[11 + i] = state.body2_pos[i];
        in[14 + i] = state.body2_vel[i];
    }
    return in;
}

/// Call the backend unless `time` was already evaluated (HHT re-query cache).
double ExternalPtoModel::EvaluatePacked(double time,
                                        const std::vector<double>& in) {
    if (!initialized_) {
        throw std::runtime_error(
            "ExternalPtoModel::ComputeForce before Initialize");
    }
    // Same (or earlier) time → return last force; no IPC / no re-evaluate.
    if (time <= prev_time_ && prev_time_ >= 0.0) {
        return cached_force_;
    }
    const double dt = (prev_time_ >= 0.0) ? (time - prev_time_) : last_dt_;
    last_dt_ = (dt > 0.0) ? dt : last_dt_;

    std::vector<double> out;
    backend_->Evaluate(time, in, out);
    ++evaluate_call_count_;
    if (out.size() != 1) {
        throw std::runtime_error("ExternalPtoModel: expected one force output");
    }
    cached_force_ = out[0];
    prev_time_ = time;
    return cached_force_;
}

/// IPTOModel entry used by lean Chrono functors (disp/vel only).
double ExternalPtoModel::ComputeForce(double displacement,
                                      double velocity,
                                      double time) {
    if (rich_state_) {
        // Functors that only have disp/vel still go through the rich path so
        // channel counts match the handshake (other fields stay zero).
        ExternalPtoState s;
        s.time = time;
        s.displacement = displacement;
        s.velocity = velocity;
        return ComputeForce(s);
    }
    return EvaluatePacked(time, PackLean(displacement, velocity));
}

/// Full-state entry used by ExternalPtoForceFunctor / ExternalPtoTorqueFunctor.
double ExternalPtoModel::ComputeForce(const ExternalPtoState& state) {
    if (!initialized_) {
        throw std::runtime_error(
            "ExternalPtoModel::ComputeForce before Initialize");
    }
    // Cache hit: do not update FD acceleration state.
    if (state.time <= prev_time_ && prev_time_ >= 0.0) {
        return cached_force_;
    }

    ExternalPtoState packed = state;
    // Finite-difference relative acceleration across accepted steps.
    if (have_prev_velocity_ && prev_time_ >= 0.0) {
        const double dt = state.time - prev_time_;
        if (dt > 0.0) {
            packed.rel_accel = (state.velocity - prev_velocity_) / dt;
        } else {
            packed.rel_accel = 0.0;
        }
    } else {
        packed.rel_accel = 0.0;
    }

    std::vector<double> in =
        rich_state_ ? PackRich(packed)
                    : PackLean(packed.displacement, packed.velocity);
    // Capture kinematics before EvaluatePacked advances prev_time_.
    const double vel = state.velocity;
    const double force = EvaluatePacked(state.time, in);
    prev_velocity_ = vel;
    have_prev_velocity_ = true;
    return force;
}

/// Clear module state and drop the force cache.
void ExternalPtoModel::Reset() {
    if (backend_ && initialized_) {
        backend_->Reset();
    }
    prev_time_ = -1.0;
    cached_force_ = 0.0;
    prev_velocity_ = 0.0;
    have_prev_velocity_ = false;
}

/// Tear down the backend (closes IPC / joins the child process).
void ExternalPtoModel::Shutdown() {
    if (backend_ && initialized_) {
        backend_->Shutdown();
        initialized_ = false;
    }
}

}  // namespace external
}  // namespace seastack
