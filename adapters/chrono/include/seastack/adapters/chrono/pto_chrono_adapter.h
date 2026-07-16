/*********************************************************************
 * @file  pto_chrono_adapter.h
 * @brief Adapter: plug a solver-agnostic IPTOModel into Chrono's TSDA/RSDA.
 *
 * Chrono calls evaluate() each force query; functors pack kinematics and
 * forward to IPTOModel / ExternalPtoModel (which may IPC to a user process).
 *
 * Wrappers:
 *   PTOForceFunctor      — ChLinkTSDA::ForceFunctor  → IPTOModel (disp, vel)
 *   PTOTorqueFunctor     — ChLinkRSDA::TorqueFunctor → IPTOModel
 *   ExternalPtoForceFunctor / ExternalPtoTorqueFunctor — rich ExternalPtoState
 *     packing when SEASTACK_HAVE_EXTERNAL is defined.
 *********************************************************************/
#ifndef SEASTACK_ADAPTERS_CHRONO_PTO_CHRONO_ADAPTER_H
#define SEASTACK_ADAPTERS_CHRONO_PTO_CHRONO_ADAPTER_H

#include <memory>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChLinkRSDA.h>
#include <seastack/config.h>
#include <seastack/pto/pto_model.h>

#ifdef SEASTACK_HAVE_EXTERNAL
#include <seastack/external/external_pto_model.h>
#endif

namespace seastack {
namespace chrono {

/// ChLinkTSDA::ForceFunctor that delegates to an IPTOModel.
///
/// Displacement is (length - rest_length); velocity sign follows Chrono
/// (positive = extending).
class PTOForceFunctor : public ::chrono::ChLinkTSDA::ForceFunctor {
  public:
    explicit PTOForceFunctor(std::shared_ptr<seastack::pto::IPTOModel> model);

    double evaluate(double time,
                    double rest_length,
                    double length,
                    double vel,
                    const ::chrono::ChLinkTSDA& link) override;

  private:
    std::shared_ptr<seastack::pto::IPTOModel> model_;
};

/// ChLinkRSDA::TorqueFunctor that delegates to an IPTOModel.
///
/// Displacement is (angle - rest_angle) [rad]; velocity is omega [rad/s];
/// return value is torque [N·m].
class PTOTorqueFunctor : public ::chrono::ChLinkRSDA::TorqueFunctor {
  public:
    explicit PTOTorqueFunctor(std::shared_ptr<seastack::pto::IPTOModel> model);

    double evaluate(double time,
                    double rest_angle,
                    double angle,
                    double vel,
                    const ::chrono::ChLinkRSDA& link) override;

  private:
    std::shared_ptr<seastack::pto::IPTOModel> model_;
};

#ifdef SEASTACK_HAVE_EXTERNAL

/// Rich TSDA functor: packs link + body kinematics into ExternalPtoState.
class ExternalPtoForceFunctor : public ::chrono::ChLinkTSDA::ForceFunctor {
  public:
    explicit ExternalPtoForceFunctor(
        std::shared_ptr<seastack::external::ExternalPtoModel> model);

    double evaluate(double time,
                    double rest_length,
                    double length,
                    double vel,
                    const ::chrono::ChLinkTSDA& link) override;

  private:
    std::shared_ptr<seastack::external::ExternalPtoModel> model_;
};

/// Rich RSDA functor: packs link + body kinematics into ExternalPtoState.
class ExternalPtoTorqueFunctor : public ::chrono::ChLinkRSDA::TorqueFunctor {
  public:
    explicit ExternalPtoTorqueFunctor(
        std::shared_ptr<seastack::external::ExternalPtoModel> model);

    double evaluate(double time,
                    double rest_angle,
                    double angle,
                    double vel,
                    const ::chrono::ChLinkRSDA& link) override;

  private:
    std::shared_ptr<seastack::external::ExternalPtoModel> model_;
};

#endif  // SEASTACK_HAVE_EXTERNAL

}  // namespace chrono
}  // namespace seastack

#endif  // SEASTACK_ADAPTERS_CHRONO_PTO_CHRONO_ADAPTER_H
