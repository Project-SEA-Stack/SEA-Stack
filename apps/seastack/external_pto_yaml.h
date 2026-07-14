/*********************************************************************
 * @file  external_pto_yaml.h
 * @brief Parse optional external_pto block and attach to Chrono TSDAs.
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

/// Read `external_pto:` from setup YAML into config (no-op if absent).
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

    /// Locate named ChLinkTSDA and register ExternalPtoModel force functor.
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
