#include "external_pto_yaml.h"

#ifdef SEASTACK_HAVE_EXTERNAL

#include <seastack/adapters/chrono/model_yaml_link_coeffs.h>
#include <seastack/adapters/chrono/pto_chrono_adapter.h>
#include <seastack/external/external_pto_model.h>
#include <seastack/external/ipc_external_force_model.h>
#include <seastack/infra/logging.h>

#include <chrono/physics/ChLinkRSDA.h>
#include <chrono/physics/ChLinkTSDA.h>

#include <yaml-cpp/yaml.h>

#include <fstream>
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
    if (ep["combine_native"]) {
        cfg.combine_native = ep["combine_native"].as<bool>();
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

/// Read setup YAML. Exactly one of the following may be present:
///   - `external_pto_file:`  single sidecar attach file
///   - `external_pto:`       single inline attach map
///   - `external_ptos:`      sequence of attaches (one child per entry); each
///                           item is an inline attach map, or `{ file: path }`
///                           pointing to a sidecar attach file.
/// All parsed attaches are stored in `config.external_ptos`; the legacy scalar
/// `config.external_pto` mirrors the first entry.
void LoadExternalPtoFromSetupYaml(const std::filesystem::path& setup_path,
                                  seastack::infra::SetupConfig& config) {
    config.has_external_pto = false;
    config.has_external_pto_file = false;
    config.external_pto_file.clear();
    config.external_ptos.clear();
    if (!std::filesystem::exists(setup_path)) {
        return;
    }
    YAML::Node root = YAML::LoadFile(setup_path.string());
    const bool has_file = static_cast<bool>(root["external_pto_file"]);
    const bool has_inline = static_cast<bool>(root["external_pto"]);
    const bool has_list = static_cast<bool>(root["external_ptos"]);
    const int n_forms = static_cast<int>(has_file) + static_cast<int>(has_inline) +
                        static_cast<int>(has_list);
    if (n_forms > 1) {
        throw std::runtime_error(
            "setup YAML may set only one of 'external_pto_file', inline "
            "'external_pto:', or 'external_ptos:' — not more than one");
    }
    if (n_forms == 0) {
        return;
    }

    std::error_code abs_ec;
    auto setup_dir = std::filesystem::absolute(setup_path, abs_ec).parent_path();
    if (abs_ec) {
        setup_dir = setup_path.parent_path();
    }

    // --- sequence form: one attach per entry -------------------------------
    if (has_list) {
        const YAML::Node seq = root["external_ptos"];
        if (!seq.IsSequence() || seq.size() == 0) {
            throw std::runtime_error(
                "external_ptos must be a non-empty sequence of attach maps");
        }
        for (const YAML::Node& item : seq) {
            YAML::Node node = item;
            const char* label = "external_ptos entry";
            // `{ file: path }` loads a sidecar attach file for this entry.
            if (item.IsMap() && item["file"]) {
                std::filesystem::path pto_path(item["file"].as<std::string>());
                if (pto_path.is_relative()) {
                    pto_path = setup_dir / pto_path;
                }
                if (!std::filesystem::exists(pto_path)) {
                    throw std::runtime_error(
                        "external_ptos file not found: " + pto_path.string());
                }
                node = YAML::LoadFile(pto_path.string());
                if (!node || !node.IsMap()) {
                    throw std::runtime_error(
                        "external_ptos file must be a YAML map: " +
                        pto_path.string());
                }
            }
            seastack::infra::SetupConfig::ExternalPtoConfig cfg_i;
            ParseExternalPtoAttachNode(node, cfg_i, setup_dir, label);
            config.external_ptos.push_back(std::move(cfg_i));
        }
        config.external_pto = config.external_ptos.front();
        config.has_external_pto = true;
        return;
    }

    // --- single form (backward compatible) ---------------------------------
    YAML::Node ep;
    const char* source_label = "external_pto";
    if (has_file) {
        config.external_pto_file = root["external_pto_file"].as<std::string>();
        config.has_external_pto_file = true;
        std::filesystem::path pto_path(config.external_pto_file);
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
    config.external_ptos.push_back(config.external_pto);
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
    double dt,
    const std::filesystem::path& model_yaml_path) {
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

    // Recover model YAML k/c/preload. Chrono embeds them in its YAML functor
    // (which we replace); link coefficients stay at zero so export power uses
    // -(F*v) on the total force without double-counting.
    seastack::chrono::LinkSpringDamperCoeffs yaml_coeffs;
    std::string model_yaml_text;
    if (!model_yaml_path.empty()) {
        std::ifstream in(model_yaml_path);
        if (in) {
            std::ostringstream ss;
            ss << in.rdbuf();
            model_yaml_text = ss.str();
            if (tsda) {
                yaml_coeffs =
                    seastack::chrono::LookupTsdaSpringDamperFromModelYaml(
                        model_yaml_text, cfg.link_name);
            } else {
                yaml_coeffs =
                    seastack::chrono::LookupRsdaSpringDamperFromModelYaml(
                        model_yaml_text, cfg.link_name);
            }
        }
    }

    const bool yaml_nonzero =
        (yaml_coeffs.has_spring && yaml_coeffs.spring_coefficient != 0.0) ||
        (yaml_coeffs.has_damping && yaml_coeffs.damping_coefficient != 0.0) ||
        (yaml_coeffs.has_preload && yaml_coeffs.preload != 0.0);

    seastack::chrono::NativeSpringDamper native;
    if (cfg.combine_native) {
        if (yaml_coeffs.has_spring) {
            native.k = yaml_coeffs.spring_coefficient;
        }
        if (yaml_coeffs.has_damping) {
            native.c = yaml_coeffs.damping_coefficient;
        }
        if (yaml_coeffs.has_preload) {
            native.preload = yaml_coeffs.preload;
        }
        seastack::infra::cli::LogInfo(
            "external_pto combine_native on '" + cfg.link_name +
            "': k=" + std::to_string(native.k) +
            ", c=" + std::to_string(native.c) +
            ", preload=" + std::to_string(native.preload));
    } else if (yaml_nonzero) {
        seastack::infra::cli::LogWarning(
            "external_pto on '" + cfg.link_name +
            "': model YAML spring_coefficient/damping_coefficient/preload "
            "are non-zero but ignored (Chrono functor is replaced). Set "
            "combine_native: true on the external PTO attach to apply them "
            "on top of the external force.");
    }

    if (tsda) {
        tsda->SetSpringCoefficient(0.0);
        tsda->SetDampingCoefficient(0.0);
        if (cfg.rich_state) {
            tsda->RegisterForceFunctor(
                std::make_shared<seastack::chrono::ExternalPtoForceFunctor>(
                    model, native));
        } else {
            tsda->RegisterForceFunctor(
                std::make_shared<seastack::chrono::PTOForceFunctor>(model,
                                                                   native));
        }
        std::string detail = "Attached external PTO to TSDA '" + cfg.link_name + "'";
        if (!cfg.command.empty()) {
            detail += " via " +
                      std::filesystem::path(cfg.command.back()).filename().string();
        }
        seastack::infra::cli::LogInfo(detail);
    } else {
        rsda->SetSpringCoefficient(0.0);
        rsda->SetDampingCoefficient(0.0);
        if (cfg.rich_state) {
            rsda->RegisterTorqueFunctor(
                std::make_shared<seastack::chrono::ExternalPtoTorqueFunctor>(
                    model, native));
        } else {
            rsda->RegisterTorqueFunctor(
                std::make_shared<seastack::chrono::PTOTorqueFunctor>(model,
                                                                    native));
        }
        std::string detail = "Attached external PTO to RSDA '" + cfg.link_name + "'";
        if (!cfg.command.empty()) {
            detail += " via " +
                      std::filesystem::path(cfg.command.back()).filename().string();
        }
        seastack::infra::cli::LogInfo(detail);
    }

    ExternalPtoAttachment out;
    out.impl_ = std::make_unique<Impl>();
    out.impl_->model = std::move(model);
    return out;
}

}  // namespace seastack::app

#endif  // SEASTACK_HAVE_EXTERNAL
