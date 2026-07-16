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

/// Chrono TSDA callback → lean IPTOModel (displacement, velocity).
double PTOForceFunctor::evaluate(double time,
                                 double rest_length,
                                 double length,
                                 double vel,
                                 const ::chrono::ChLinkTSDA& /*link*/) {
    double displacement = length - rest_length;
    return model_->ComputeForce(displacement, vel, time);
}

PTOTorqueFunctor::PTOTorqueFunctor(std::shared_ptr<seastack::pto::IPTOModel> model)
    : model_(std::move(model)) {
    if (!model_) {
        throw std::invalid_argument(
            "PTOTorqueFunctor requires a non-null IPTOModel");
    }
}

/// Chrono RSDA callback → lean IPTOModel (angle-rest as displacement, omega).
double PTOTorqueFunctor::evaluate(double time,
                                  double rest_angle,
                                  double angle,
                                  double vel,
                                  const ::chrono::ChLinkRSDA& /*link*/) {
    const double displacement = angle - rest_angle;
    return model_->ComputeForce(displacement, vel, time);
}

#ifdef SEASTACK_HAVE_EXTERNAL

namespace {

/// Copy world-frame body positions/velocities into ExternalPtoState.
void FillBodyKinematics(::chrono::ChBodyFrame* body1,
                        ::chrono::ChBodyFrame* body2,
                        seastack::external::ExternalPtoState& s) {
    if (body1) {
        const auto& p = body1->GetPos();
        const auto& v = body1->GetPosDt();
        s.body1_pos[0] = p.x();
        s.body1_pos[1] = p.y();
        s.body1_pos[2] = p.z();
        s.body1_vel[0] = v.x();
        s.body1_vel[1] = v.y();
        s.body1_vel[2] = v.z();
    }
    if (body2) {
        const auto& p = body2->GetPos();
        const auto& v = body2->GetPosDt();
        s.body2_pos[0] = p.x();
        s.body2_pos[1] = p.y();
        s.body2_pos[2] = p.z();
        s.body2_vel[0] = v.x();
        s.body2_vel[1] = v.y();
        s.body2_vel[2] = v.z();
    }
}

}  // namespace

ExternalPtoForceFunctor::ExternalPtoForceFunctor(
    std::shared_ptr<seastack::external::ExternalPtoModel> model)
    : model_(std::move(model)) {
    if (!model_) {
        throw std::invalid_argument(
            "ExternalPtoForceFunctor requires a non-null ExternalPtoModel");
    }
}

/// Chrono TSDA callback → pack rich ExternalPtoState → ExternalPtoModel.
double ExternalPtoForceFunctor::evaluate(double time,
                                         double rest_length,
                                         double length,
                                         double vel,
                                         const ::chrono::ChLinkTSDA& link) {
    seastack::external::ExternalPtoState s;
    s.time = time;
    s.rest_length = rest_length;
    s.length = length;
    s.displacement = length - rest_length;
    s.velocity = vel;
    FillBodyKinematics(link.GetBody1(), link.GetBody2(), s);
    return model_->ComputeForce(s);
}

ExternalPtoTorqueFunctor::ExternalPtoTorqueFunctor(
    std::shared_ptr<seastack::external::ExternalPtoModel> model)
    : model_(std::move(model)) {
    if (!model_) {
        throw std::invalid_argument(
            "ExternalPtoTorqueFunctor requires a non-null ExternalPtoModel");
    }
}

/// Chrono RSDA callback → pack rich ExternalPtoState → ExternalPtoModel.
double ExternalPtoTorqueFunctor::evaluate(double time,
                                          double rest_angle,
                                          double angle,
                                          double vel,
                                          const ::chrono::ChLinkRSDA& link) {
    seastack::external::ExternalPtoState s;
    s.time = time;
    s.rest_angle = rest_angle;
    s.angle = angle;
    s.displacement = angle - rest_angle;
    s.velocity = vel;
    FillBodyKinematics(link.GetBody1(), link.GetBody2(), s);
    return model_->ComputeForce(s);
}

#endif  // SEASTACK_HAVE_EXTERNAL

}  // namespace seastack::chrono
