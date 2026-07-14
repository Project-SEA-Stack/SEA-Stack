#include <seastack/external/external_force_component.h>

#include <stdexcept>
#include <utility>

namespace seastack {
namespace external {
namespace {

constexpr int kStatePerBody = 12;   // pos3 + rpy3 + linvel3 + angvel3
constexpr int kForcePerBody = 6;    // force3 + moment3

}  // namespace

ExternalForceComponent::ExternalForceComponent(
    std::unique_ptr<IExternalForceModel> backend, int num_bodies)
    : backend_(std::move(backend)), num_bodies_(num_bodies) {
    if (!backend_) {
        throw std::invalid_argument(
            "ExternalForceComponent: backend must not be null");
    }
    if (num_bodies_ <= 0) {
        throw std::invalid_argument(
            "ExternalForceComponent: num_bodies must be > 0");
    }
    cached_forces_.assign(static_cast<size_t>(num_bodies_), {});
}

ExternalForceComponent::~ExternalForceComponent() {
    try {
        Shutdown();
    } catch (...) {
    }
}

ExternalMeta ExternalForceComponent::Initialize(
    double dt, const std::string& config_json) {
    ExternalInit init;
    init.kind = "body_force";
    init.n_inputs = num_bodies_ * kStatePerBody;
    init.n_outputs = num_bodies_ * kForcePerBody;
    init.dt = dt;
    init.config_json = config_json.empty() ? "{}" : config_json;
    last_dt_ = dt;
    const ExternalMeta meta = backend_->Initialize(init);
    initialized_ = true;
    prev_time_ = -1.0;
    evaluate_call_count_ = 0;
    for (auto& f : cached_forces_) {
        f.setZero();
    }
    return meta;
}

void ExternalForceComponent::Compute(
    const seastack::hydro::SystemState& state,
    double time,
    seastack::hydro::BodyForces& inout_forces) {
    if (!initialized_) {
        throw std::runtime_error(
            "ExternalForceComponent::Compute before Initialize");
    }
    if (static_cast<int>(state.bodies.size()) != num_bodies_) {
        throw std::runtime_error(
            "ExternalForceComponent: SystemState body count mismatch");
    }
    if (static_cast<int>(inout_forces.size()) < num_bodies_) {
        throw std::runtime_error(
            "ExternalForceComponent: inout_forces too small");
    }

    if (time <= prev_time_ && prev_time_ >= 0.0) {
        for (int i = 0; i < num_bodies_; ++i) {
            inout_forces[static_cast<size_t>(i)] +=
                cached_forces_[static_cast<size_t>(i)];
        }
        return;
    }

    const double dt = (prev_time_ >= 0.0) ? (time - prev_time_) : last_dt_;
    last_dt_ = (dt > 0.0) ? dt : last_dt_;

    std::vector<double> in(static_cast<size_t>(num_bodies_ * kStatePerBody));
    for (int b = 0; b < num_bodies_; ++b) {
        const auto& body = state.bodies[static_cast<size_t>(b)];
        const size_t off = static_cast<size_t>(b * kStatePerBody);
        in[off + 0] = body.position.x();
        in[off + 1] = body.position.y();
        in[off + 2] = body.position.z();
        in[off + 3] = body.orientation_rpy.x();
        in[off + 4] = body.orientation_rpy.y();
        in[off + 5] = body.orientation_rpy.z();
        in[off + 6] = body.linear_velocity.x();
        in[off + 7] = body.linear_velocity.y();
        in[off + 8] = body.linear_velocity.z();
        in[off + 9] = body.angular_velocity.x();
        in[off + 10] = body.angular_velocity.y();
        in[off + 11] = body.angular_velocity.z();
    }

    std::vector<double> out;
    backend_->Evaluate(time, in, out);
    ++evaluate_call_count_;

    if (static_cast<int>(out.size()) != num_bodies_ * kForcePerBody) {
        throw std::runtime_error(
            "ExternalForceComponent: unexpected output size");
    }

    for (int b = 0; b < num_bodies_; ++b) {
        const size_t off = static_cast<size_t>(b * kForcePerBody);
        auto& gf = cached_forces_[static_cast<size_t>(b)];
        gf.force.x() = out[off + 0];
        gf.force.y() = out[off + 1];
        gf.force.z() = out[off + 2];
        gf.moment.x() = out[off + 3];
        gf.moment.y() = out[off + 4];
        gf.moment.z() = out[off + 5];
        inout_forces[static_cast<size_t>(b)] += gf;
    }
    prev_time_ = time;
}

void ExternalForceComponent::Reset() {
    if (backend_ && initialized_) {
        backend_->Reset();
    }
    prev_time_ = -1.0;
    for (auto& f : cached_forces_) {
        f.setZero();
    }
}

void ExternalForceComponent::Shutdown() {
    if (backend_ && initialized_) {
        backend_->Shutdown();
        initialized_ = false;
    }
}

}  // namespace external
}  // namespace seastack
