#include "external_pto_yaml.h"

#ifdef SEASTACK_HAVE_EXTERNAL

#include <seastack/adapters/chrono/pto_chrono_adapter.h>
#include <seastack/external/external_pto_model.h>
#include <seastack/external/ipc_external_force_model.h>
#include <seastack/infra/logging.h>

#include <chrono/physics/ChLinkRSDA.h>
#include <chrono/physics/ChLinkTSDA.h>

#include <yaml-cpp/yaml.h>

#include <sstream>
#include <stdexcept>
#include <utility>

namespace seastack::app {
namespace {

std::string YamlNodeToJsonObject(const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        return "{}";
    }
    std::ostringstream oss;
    oss << '{';
    bool first = true;
    for (auto it = node.begin(); it != node.end(); ++it) {
        if (!first) {
            oss << ',';
        }
        first = false;
        const std::string key = it->first.as<std::string>();
        oss << '"' << key << '"';
        oss << ':';
        const YAML::Node& val = it->second;
        if (val.IsScalar()) {
            // Prefer numeric if parseable; otherwise quoted string / bool.
            try {
                const double d = val.as<double>();
                oss << d;
            } catch (...) {
                const std::string s = val.as<std::string>();
                if (s == "true" || s == "false" || s == "null") {
                    oss << s;
                } else {
                    oss << '"' << s << '"';
                }
            }
        } else if (val.IsSequence()) {
            oss << '[';
            for (size_t i = 0; i < val.size(); ++i) {
                if (i > 0) {
                    oss << ',';
                }
                oss << val[i].as<double>();
            }
            oss << ']';
        } else {
            oss << "null";
        }
    }
    oss << '}';
    return oss.str();
}

/// Fill ExternalPtoConfig from one YAML map (sidecar or inline `external_pto:`).
/// Resolves relative script paths against `setup_dir`.
void ParseExternalPtoAttachNode(const YAML::Node& ep,
                                seastack::infra::SetupConfig::ExternalPtoConfig& cfg,
                                const std::filesystem::path& setup_dir,
                                const char* source_label) {
    if (ep["link"]) {
        cfg.link_name = ep["link"].as<std::string>();
    }
    if (ep["timeout_ms"]) {
        cfg.timeout_ms = ep["timeout_ms"].as<int>();
    }
    if (ep["working_directory"]) {
        cfg.working_directory = ep["working_directory"].as<std::string>();
    }
    if (ep["rich_state"]) {
        cfg.rich_state = ep["rich_state"].as<bool>();
    }
    if (ep["command"]) {
        cfg.command.clear();
        if (ep["command"].IsSequence()) {
            for (const auto& item : ep["command"]) {
                cfg.command.push_back(item.as<std::string>());
            }
        } else {
            cfg.command.push_back(ep["command"].as<std::string>());
        }
    }
    if (ep["config"]) {
        cfg.config_json = YamlNodeToJsonObject(ep["config"]);
    }
    if (cfg.link_name.empty() || cfg.command.empty()) {
        throw std::runtime_error(
            std::string(source_label) +
            " requires 'link' and non-empty 'command'");
    }
    // Resolve relative command[1+] / working_directory against the setup dir,
    // as ABSOLUTE paths. The child process runs with its cwd set to
    // working_directory, so relative script paths would otherwise be resolved
    // against that cwd (double-nesting) rather than against the launcher.
    if (cfg.working_directory.empty()) {
        cfg.working_directory = setup_dir.string();
    } else {
        std::filesystem::path wd(cfg.working_directory);
        if (wd.is_relative()) {
            cfg.working_directory = (setup_dir / wd).string();
        }
    }
    for (size_t i = 1; i < cfg.command.size(); ++i) {
        std::filesystem::path p(cfg.command[i]);
        if (p.is_relative() && p.extension() == ".py") {
            cfg.command[i] = (setup_dir / p).string();
        }
    }
}

}  // namespace

/// Read setup YAML: prefer `external_pto_file`, else inline `external_pto:`.
void LoadExternalPtoFromSetupYaml(const std::filesystem::path& setup_path,
                                  seastack::infra::SetupConfig& config) {
    config.has_external_pto = false;
    if (!std::filesystem::exists(setup_path)) {
        return;
    }
    YAML::Node root = YAML::LoadFile(setup_path.string());
    const bool has_file = static_cast<bool>(root["external_pto_file"]);
    const bool has_inline = static_cast<bool>(root["external_pto"]);
    if (has_file && has_inline) {
        throw std::runtime_error(
            "setup YAML may not set both 'external_pto_file' and inline "
            "'external_pto:' — use one or the other");
    }
    if (!has_file && !has_inline) {
        return;
    }

    std::error_code abs_ec;
    auto setup_dir = std::filesystem::absolute(setup_path, abs_ec).parent_path();
    if (abs_ec) {
        setup_dir = setup_path.parent_path();
    }

    YAML::Node ep;
    const char* source_label = "external_pto";
    if (has_file) {
        std::filesystem::path pto_path(root["external_pto_file"].as<std::string>());
        if (pto_path.is_relative()) {
            pto_path = setup_dir / pto_path;
        }
        if (!std::filesystem::exists(pto_path)) {
            throw std::runtime_error(
                "external_pto_file not found: " + pto_path.string());
        }
        ep = YAML::LoadFile(pto_path.string());
        if (!ep || !ep.IsMap()) {
            throw std::runtime_error(
                "external_pto_file must be a YAML map: " + pto_path.string());
        }
        source_label = "external_pto_file";
    } else {
        ep = root["external_pto"];
    }

    ParseExternalPtoAttachNode(ep, config.external_pto, setup_dir, source_label);
    config.has_external_pto = true;
}

struct ExternalPtoAttachment::Impl {
    std::shared_ptr<seastack::external::ExternalPtoModel> model;
};

ExternalPtoAttachment::ExternalPtoAttachment() = default;

ExternalPtoAttachment::~ExternalPtoAttachment() {
    if (impl_ && impl_->model) {
        try {
            impl_->model->Shutdown();
        } catch (...) {
        }
    }
}

ExternalPtoAttachment::ExternalPtoAttachment(ExternalPtoAttachment&&) noexcept =
    default;
ExternalPtoAttachment& ExternalPtoAttachment::operator=(
    ExternalPtoAttachment&&) noexcept = default;

ExternalPtoAttachment ExternalPtoAttachment::Attach(
    ::chrono::ChSystem& system,
    const seastack::infra::SetupConfig::ExternalPtoConfig& cfg,
    double dt) {
    // Find named TSDA/RSDA -> spawn IPC child -> wrap in ExternalPtoModel ->
    // register Chrono force/torque functor (rich or lean).
    std::shared_ptr<::chrono::ChLinkTSDA> tsda;
    std::shared_ptr<::chrono::ChLinkRSDA> rsda;
    for (auto& link : system.GetLinks()) {
        if (link->GetName() != cfg.link_name) {
            continue;
        }
        if (auto* as_tsda = dynamic_cast<::chrono::ChLinkTSDA*>(link.get())) {
            tsda = std::dynamic_pointer_cast<::chrono::ChLinkTSDA>(link);
            (void)as_tsda;
        } else if (auto* as_rsda =
                       dynamic_cast<::chrono::ChLinkRSDA*>(link.get())) {
            rsda = std::dynamic_pointer_cast<::chrono::ChLinkRSDA>(link);
            (void)as_rsda;
        }
    }
    if (tsda && rsda) {
        throw std::runtime_error(
            "external_pto: link name '" + cfg.link_name +
            "' matched both a ChLinkTSDA and a ChLinkRSDA");
    }
    if (!tsda && !rsda) {
        throw std::runtime_error(
            "external_pto: no ChLinkTSDA or ChLinkRSDA named '" +
            cfg.link_name + "' found in model");
    }

    seastack::external::IpcExternalForceOptions opts;
    opts.command = cfg.command;
    opts.timeout_ms = cfg.timeout_ms;
    opts.working_directory = cfg.working_directory;

    auto ipc =
        std::make_unique<seastack::external::IpcExternalForceModel>(opts);
    auto model =
        std::make_shared<seastack::external::ExternalPtoModel>(std::move(ipc));

    if (rsda) {
        model->SetLinkKind(seastack::external::ExternalPtoLinkKind::Rsda);
    } else {
        model->SetLinkKind(seastack::external::ExternalPtoLinkKind::Tsda);
    }
    model->EnableRichState(cfg.rich_state);
    model->Initialize(dt, cfg.config_json);

    if (tsda) {
        tsda->SetSpringCoefficient(0.0);
        tsda->SetDampingCoefficient(0.0);
        if (cfg.rich_state) {
            tsda->RegisterForceFunctor(
                std::make_shared<seastack::chrono::ExternalPtoForceFunctor>(
                    model));
        } else {
            tsda->RegisterForceFunctor(
                std::make_shared<seastack::chrono::PTOForceFunctor>(model));
        }
        seastack::infra::cli::LogInfo(
            "Attached external PTO module to TSDA link '" + cfg.link_name +
            "'");
    } else {
        rsda->SetSpringCoefficient(0.0);
        rsda->SetDampingCoefficient(0.0);
        if (cfg.rich_state) {
            rsda->RegisterTorqueFunctor(
                std::make_shared<seastack::chrono::ExternalPtoTorqueFunctor>(
                    model));
        } else {
            rsda->RegisterTorqueFunctor(
                std::make_shared<seastack::chrono::PTOTorqueFunctor>(model));
        }
        seastack::infra::cli::LogInfo(
            "Attached external PTO module to RSDA link '" + cfg.link_name +
            "'");
    }

    ExternalPtoAttachment out;
    out.impl_ = std::make_unique<Impl>();
    out.impl_->model = std::move(model);
    return out;
}

}  // namespace seastack::app

#endif  // SEASTACK_HAVE_EXTERNAL
