#include <seastack/external/external_pto_model.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace seastack {
namespace external {

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

ExternalMeta ExternalPtoModel::Initialize(double dt,
                                          const std::string& config_json) {
    ExternalInit init;
    init.kind = "pto";
    init.n_inputs = 2;
    init.n_outputs = 1;
    init.dt = dt;
    init.config_json = config_json.empty() ? "{}" : config_json;
    last_dt_ = dt;
    const ExternalMeta meta = backend_->Initialize(init);
    initialized_ = true;
    prev_time_ = -1.0;
    cached_force_ = 0.0;
    evaluate_call_count_ = 0;
    return meta;
}

double ExternalPtoModel::ComputeForce(double displacement,
                                      double velocity,
                                      double time) {
    if (!initialized_) {
        throw std::runtime_error("ExternalPtoModel::ComputeForce before Initialize");
    }

    // Time-step caching: only round-trip on genuinely new time steps.
    // Repeated or backward calls (e.g. HHT sub-evaluations) return the
    // cached force without contacting the module (matches RectifiedHydraulicPTO).
    if (time <= prev_time_ && prev_time_ >= 0.0) {
        return cached_force_;
    }

    const double dt = (prev_time_ >= 0.0) ? (time - prev_time_) : last_dt_;
    last_dt_ = (dt > 0.0) ? dt : last_dt_;

    std::vector<double> in = {displacement, velocity};
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

void ExternalPtoModel::Reset() {
    if (backend_ && initialized_) {
        backend_->Reset();
    }
    prev_time_ = -1.0;
    cached_force_ = 0.0;
}

void ExternalPtoModel::Shutdown() {
    if (backend_ && initialized_) {
        backend_->Shutdown();
        initialized_ = false;
    }
}

}  // namespace external
}  // namespace seastack
