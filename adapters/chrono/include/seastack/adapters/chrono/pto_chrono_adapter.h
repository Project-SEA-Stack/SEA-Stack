/*********************************************************************
 * @file  pto_chrono_adapter.h
 * @brief Adapter: plug a solver-agnostic IPTOModel into Chrono's ChLinkTSDA.
 *
 * Wraps any seastack::pto::IPTOModel into a ChLinkTSDA::ForceFunctor
 * so that PTO logic defined outside Chrono can drive a translational
 * spring-damper link.
 *
 * Typical usage:
 * @code{.cpp}
 *   auto pto = std::make_shared<seastack::pto::LinearPTO>(500.0, 50.0);
 *   auto tsda = chrono_types::make_shared<ChLinkTSDA>();
 *   tsda->Initialize(body_a, body_b, false, pt_a, pt_b);
 *   tsda->RegisterForceFunctor(
 *       std::make_shared<seastack::chrono::PTOForceFunctor>(pto));
 *   system.AddLink(tsda);
 * @endcode
 *********************************************************************/
#ifndef SEASTACK_ADAPTERS_CHRONO_PTO_CHRONO_ADAPTER_H
#define SEASTACK_ADAPTERS_CHRONO_PTO_CHRONO_ADAPTER_H

#include <memory>
#include <chrono/physics/ChLinkTSDA.h>
#include <seastack/pto/pto_model.h>

namespace seastack::chrono {

/// ChLinkTSDA::ForceFunctor that delegates to an IPTOModel.
///
/// Displacement is computed as (current_length - rest_length), so a positive
/// value means the link is extended beyond rest.  Velocity sign follows
/// Chrono's convention (positive = extending).
class PTOForceFunctor : public ::chrono::ChLinkTSDA::ForceFunctor {
  public:
    /// @param model  Shared ownership of the PTO model.  Must not be null.
    explicit PTOForceFunctor(std::shared_ptr<seastack::pto::IPTOModel> model);

    double evaluate(double time,
                    double rest_length,
                    double length,
                    double vel,
                    const ::chrono::ChLinkTSDA& link) override;

  private:
    std::shared_ptr<seastack::pto::IPTOModel> model_;
};

}  // namespace seastack::chrono

#endif  // SEASTACK_ADAPTERS_CHRONO_PTO_CHRONO_ADAPTER_H
