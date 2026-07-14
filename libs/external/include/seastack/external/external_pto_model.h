/*********************************************************************
 * @file  external_pto_model.h
 * @brief IPTOModel bridge over IExternalForceModel with time-caching.
 *
 * Inputs:  [displacement, velocity]
 * Output:  [force]
 * Caching matches RectifiedHydraulicPTO: one Evaluate round-trip per new
 * simulation time; repeated/earlier times return the cached force.
 *********************************************************************/
#ifndef SEASTACK_EXTERNAL_EXTERNAL_PTO_MODEL_H
#define SEASTACK_EXTERNAL_EXTERNAL_PTO_MODEL_H

#include <seastack/external/external_force_model.h>
#include <seastack/pto/pto_model.h>

#include <memory>
#include <string>

namespace seastack {
namespace external {

class ExternalPtoModel : public seastack::pto::IPTOModel {
  public:
    /// Takes ownership of the transport. Call Initialize() before ComputeForce.
    explicit ExternalPtoModel(std::unique_ptr<IExternalForceModel> backend);

    ~ExternalPtoModel() override;

    /// Handshake with the module. config_json is forwarded as-is (JSON object).
    ExternalMeta Initialize(double dt, const std::string& config_json = "{}");

    double ComputeForce(double displacement,
                        double velocity,
                        double time) override;

    void Reset();
    void Shutdown();

    int evaluate_call_count() const { return evaluate_call_count_; }

  private:
    std::unique_ptr<IExternalForceModel> backend_;
    bool initialized_ = false;
    double prev_time_ = -1.0;
    double cached_force_ = 0.0;
    double last_dt_ = 0.0;
    int evaluate_call_count_ = 0;
};

}  // namespace external
}  // namespace seastack

#endif  // SEASTACK_EXTERNAL_EXTERNAL_PTO_MODEL_H
