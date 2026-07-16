/*********************************************************************
 * @file  external_pto_yaml.h
 * @brief Parse optional external PTO attach YAML and register on Chrono TSDA/RSDA.
 *
 * Wiring used by run_seastack:
 *   setup `external_pto_file` -> spawn IPC child -> ExternalPtoModel ->
 *   Chrono force/torque functor on the named link.
 *********************************************************************/
#ifndef SEASTACK_APP_EXTERNAL_PTO_YAML_H
#define SEASTACK_APP_EXTERNAL_PTO_YAML_H

#include <seastack/config.h>

#ifdef SEASTACK_HAVE_EXTERNAL

#include <seastack/infra/config/yaml_discovery.h>

#include <chrono/physics/ChSystem.h>

#include <filesystem>
#include <memory>
#include <string>

namespace seastack::app {

/// Load external PTO attach settings into config (no-op if absent).
///
/// Setup may use either:
///   - `external_pto_file: path/to/*.external_pto.yaml` (preferred), or
///   - an inline `external_pto:` map (backward compatible).
/// Both at once is an error. The attach YAML must provide `link` and `command`.
void LoadExternalPtoFromSetupYaml(const std::filesystem::path& setup_path,
                                  seastack::infra::SetupConfig& config);

/// Keeps ExternalPtoModel + PTOForceFunctor alive for the simulation duration.
class ExternalPtoAttachment {
  public:
    ExternalPtoAttachment();
    ~ExternalPtoAttachment();

    ExternalPtoAttachment(const ExternalPtoAttachment&) = delete;
    ExternalPtoAttachment& operator=(const ExternalPtoAttachment&) = delete;
    ExternalPtoAttachment(ExternalPtoAttachment&&) noexcept;
    ExternalPtoAttachment& operator=(ExternalPtoAttachment&&) noexcept;

    /// Spawn the IPC child, handshake, and register a Chrono force/torque functor
    /// on the named TSDA or RSDA. Keeps the model alive for the run.
    static ExternalPtoAttachment Attach(
        ::chrono::ChSystem& system,
        const seastack::infra::SetupConfig::ExternalPtoConfig& cfg,
        double dt);

    explicit operator bool() const { return static_cast<bool>(impl_); }

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace seastack::app

#endif  // SEASTACK_HAVE_EXTERNAL

#endif  // SEASTACK_APP_EXTERNAL_PTO_YAML_H
