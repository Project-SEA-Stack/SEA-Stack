/*********************************************************************
 * @file  external_force_component.h
 * @brief IHydroForceComponent bridge over IExternalForceModel (6-DOF / body).
 *
 * Packs SystemState into a flat input vector and unpacks BodyForces from
 * the module outputs. Applies the same time-caching rule as ExternalPtoModel.
 *********************************************************************/
#ifndef SEASTACK_EXTERNAL_EXTERNAL_FORCE_COMPONENT_H
#define SEASTACK_EXTERNAL_EXTERNAL_FORCE_COMPONENT_H

#include <seastack/core/force_component.h>
#include <seastack/external/external_force_model.h>

#include <memory>
#include <string>
#include <vector>

namespace seastack {
namespace external {

class ExternalForceComponent : public seastack::hydro::IHydroForceComponent {
  public:
    /// @param backend  Owned transport.
    /// @param num_bodies  Number of bodies expected in SystemState.
    ExternalForceComponent(std::unique_ptr<IExternalForceModel> backend,
                           int num_bodies);

    ~ExternalForceComponent() override;

    ExternalMeta Initialize(double dt, const std::string& config_json = "{}");

    seastack::hydro::HydroComponentType Type() const override {
        return seastack::hydro::HydroComponentType::kExternal;
    }

    void Compute(const seastack::hydro::SystemState& state,
                 double time,
                 seastack::hydro::BodyForces& inout_forces) override;

    void Reset();
    void Shutdown();

    int evaluate_call_count() const { return evaluate_call_count_; }

  private:
    std::unique_ptr<IExternalForceModel> backend_;
    int num_bodies_ = 0;
    bool initialized_ = false;
    double prev_time_ = -1.0;
    double last_dt_ = 0.0;
    seastack::hydro::BodyForces cached_forces_;
    int evaluate_call_count_ = 0;
};

}  // namespace external
}  // namespace seastack

#endif  // SEASTACK_EXTERNAL_EXTERNAL_FORCE_COMPONENT_H
