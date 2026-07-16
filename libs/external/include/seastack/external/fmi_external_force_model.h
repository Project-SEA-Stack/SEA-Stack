/*********************************************************************
 * @file  fmi_external_force_model.h
 * @brief Prototype FMI-import backend for IExternalForceModel.
 *
 * This is a skeleton that documents how an FMU would map onto the
 * external-force lifecycle (Initialize / Evaluate / Commit / Rollback /
 * Shutdown). A full implementation requires an FMI 2/3 import library
 * (e.g. fmi4c) and is intentionally not linked in v1.
 *
 * All methods throw std::runtime_error explaining the missing dependency.
 *********************************************************************/
#ifndef SEASTACK_EXTERNAL_FMI_EXTERNAL_FORCE_MODEL_H
#define SEASTACK_EXTERNAL_FMI_EXTERNAL_FORCE_MODEL_H

#include <seastack/external/external_force_model.h>

#include <stdexcept>
#include <string>

namespace seastack {
namespace external {

struct FmiExternalForceOptions {
    /// Path to an FMU archive (.fmu) or extracted directory.
    std::string fmu_path;
    /// Optional instance name passed to the FMI instantiate call.
    std::string instance_name = "seastack_external";
};

/**
 * @brief Placeholder FMI co-simulation import.
 *
 * Mapping (planned):
 *   Initialize → fmi2Instantiate / fmi2SetupExperiment / fmi2EnterInitializationMode
 *   Evaluate   → set reals (inputs) → fmi2DoStep(t, dt) → get reals (outputs)
 *   Commit     → (optional) fmi2GetFMUstate snapshot discard
 *   Rollback   → fmi2SetFMUstate to last commit
 *   Shutdown   → fmi2Terminate / fmi2FreeInstance
 */
class FmiExternalForceModel : public IExternalForceModel {
  public:
    explicit FmiExternalForceModel(FmiExternalForceOptions options)
        : options_(std::move(options)) {}

    ExternalMeta Initialize(const ExternalInit& /*init*/) override {
        throw std::runtime_error(
            "FmiExternalForceModel: FMI import is not enabled in this build. "
            "Use IpcExternalForceModel, or link an FMI library and complete "
            "the prototype in fmi_external_force_model.h. FMU path was: " +
            options_.fmu_path);
    }

    void Evaluate(double /*time*/,
                  const std::vector<double>& /*in*/,
                  std::vector<double>& /*out*/) override {
        throw std::runtime_error("FmiExternalForceModel: not implemented");
    }

    void Commit() override {
        throw std::runtime_error("FmiExternalForceModel: not implemented");
    }

    void Rollback() override {
        throw std::runtime_error("FmiExternalForceModel: not implemented");
    }

    void Shutdown() override {
        // No-op: nothing was allocated.
    }

  private:
    FmiExternalForceOptions options_;
};

}  // namespace external
}  // namespace seastack

#endif  // SEASTACK_EXTERNAL_FMI_EXTERNAL_FORCE_MODEL_H
