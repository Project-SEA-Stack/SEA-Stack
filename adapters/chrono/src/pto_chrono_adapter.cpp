#include <seastack/adapters/chrono/pto_chrono_adapter.h>

#include <stdexcept>

namespace seastack::chrono {

PTOForceFunctor::PTOForceFunctor(std::shared_ptr<seastack::pto::IPTOModel> model)
    : model_(std::move(model)) {
    if (!model_) {
        throw std::invalid_argument(
            "PTOForceFunctor requires a non-null IPTOModel");
    }
}

double PTOForceFunctor::evaluate(double time,
                                 double rest_length,
                                 double length,
                                 double vel,
                                 const ::chrono::ChLinkTSDA& /*link*/) {
    double displacement = length - rest_length;
    return model_->ComputeForce(displacement, vel, time);
}

}  // namespace seastack::chrono
