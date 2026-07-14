#include "external_pto_yaml.h"

#ifdef SEASTACK_HAVE_EXTERNAL

#include <seastack/adapters/chrono/pto_chrono_adapter.h>
#include <seastack/external/external_pto_model.h>
#include <seastack/external/ipc_external_force_model.h>
#include <seastack/infra/logging.h>

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

}  // namespace

void LoadExternalPtoFromSetupYaml(const std::filesystem::path& setup_path,
                                  seastack::infra::SetupConfig& config) {
    config.has_external_pto = false;
    if (!std::filesystem::exists(setup_path)) {
        return;
    }
    YAML::Node root = YAML::LoadFile(setup_path.string());
    if (!root["external_pto"]) {
        return;
    }
    const YAML::Node ep = root["external_pto"];
    auto& cfg = config.external_pto;
    if (ep["link"]) {
        cfg.link_name = ep["link"].as<std::string>();
    }
    if (ep["timeout_ms"]) {
        cfg.timeout_ms = ep["timeout_ms"].as<int>();
    }
    if (ep["working_directory"]) {
        cfg.working_directory = ep["working_directory"].as<std::string>();
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
            "external_pto requires 'link' and non-empty 'command'");
    }
    // Resolve relative command[1+] / working_directory against the setup dir,
    // as ABSOLUTE paths. The child process runs with its cwd set to
    // working_directory, so relative script paths would otherwise be resolved
    // against that cwd (double-nesting) rather than against the launcher.
    std::error_code _abs_ec;
    auto setup_dir = std::filesystem::absolute(setup_path, _abs_ec).parent_path();
    if (_abs_ec) {
        setup_dir = setup_path.parent_path();
    }
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
    std::shared_ptr<::chrono::ChLinkTSDA> tsda;
    for (auto& link : system.GetLinks()) {
        auto* as_tsda = dynamic_cast<::chrono::ChLinkTSDA*>(link.get());
        if (as_tsda && link->GetName() == cfg.link_name) {
            tsda = std::dynamic_pointer_cast<::chrono::ChLinkTSDA>(link);
            break;
        }
    }
    if (!tsda) {
        throw std::runtime_error(
            "external_pto: ChLinkTSDA named '" + cfg.link_name +
            "' not found in model");
    }

    seastack::external::IpcExternalForceOptions opts;
    opts.command = cfg.command;
    opts.timeout_ms = cfg.timeout_ms;
    opts.working_directory = cfg.working_directory;

    auto ipc =
        std::make_unique<seastack::external::IpcExternalForceModel>(opts);
    auto model =
        std::make_shared<seastack::external::ExternalPtoModel>(std::move(ipc));
    model->Initialize(dt, cfg.config_json);

    // Clear Chrono built-in spring/damper so only the external force applies.
    tsda->SetSpringCoefficient(0.0);
    tsda->SetDampingCoefficient(0.0);
    tsda->RegisterForceFunctor(
        std::make_shared<seastack::chrono::PTOForceFunctor>(model));

    seastack::infra::cli::LogInfo(
        "Attached external PTO module to link '" + cfg.link_name + "'");

    ExternalPtoAttachment out;
    out.impl_ = std::make_unique<Impl>();
    out.impl_->model = std::move(model);
    return out;
}

}  // namespace seastack::app

#endif  // SEASTACK_HAVE_EXTERNAL
