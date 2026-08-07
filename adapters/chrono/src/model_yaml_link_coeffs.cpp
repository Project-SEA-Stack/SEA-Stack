#include <seastack/adapters/chrono/model_yaml_link_coeffs.h>

#include <yaml-cpp/yaml.h>

namespace seastack::chrono {
namespace {

// Match SimulationExporter::Impl::SanitizeName for YAML `name:` lookup.
std::string SanitizeLinkNameForYaml(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (char c : in) {
        if (c == ' ')
            out.push_back('_');
        else if (c == '/' || c == '\\' || c == ':')
            ;
        else
            out.push_back(c);
    }
    if (out.empty()) {
        out = "unnamed";
    }
    return out;
}

LinkSpringDamperCoeffs LookupFromSequence(const YAML::Node& seq,
                                          const std::string& link_name) {
    LinkSpringDamperCoeffs out;
    if (!seq || !seq.IsSequence()) {
        return out;
    }
    const std::string sanitized = SanitizeLinkNameForYaml(link_name);
    for (const auto& e : seq) {
        if (!e["name"]) {
            continue;
        }
        const std::string raw = e["name"].as<std::string>();
        if (!YamlLinkNameMatches(sanitized, raw) && raw != link_name) {
            continue;
        }
        out.found = true;
        if (e["spring_coefficient"]) {
            out.spring_coefficient = e["spring_coefficient"].as<double>();
            out.has_spring = true;
        }
        if (e["damping_coefficient"]) {
            out.damping_coefficient = e["damping_coefficient"].as<double>();
            out.has_damping = true;
        }
        if (e["preload"]) {
            out.preload = e["preload"].as<double>();
            out.has_preload = true;
        }
        return out;
    }
    return out;
}

}  // namespace

bool YamlLinkNameMatches(const std::string& sanitized_link_name,
                         const std::string& yaml_name_raw) {
    const std::string k = SanitizeLinkNameForYaml(yaml_name_raw);
    if (sanitized_link_name == k) {
        return true;
    }
    if (sanitized_link_name.size() > k.size() + 1 &&
        sanitized_link_name[sanitized_link_name.size() - k.size() - 1] == '_' &&
        sanitized_link_name.compare(sanitized_link_name.size() - k.size(),
                                    k.size(), k) == 0) {
        return true;
    }
    return false;
}

LinkSpringDamperCoeffs LookupTsdaSpringDamperFromModelYaml(
    const std::string& model_yaml,
    const std::string& link_name) {
    if (model_yaml.empty()) {
        return {};
    }
    try {
        YAML::Node root = YAML::Load(model_yaml);
        return LookupFromSequence(root["model"]["tsdas"], link_name);
    } catch (...) {
        return {};
    }
}

LinkSpringDamperCoeffs LookupRsdaSpringDamperFromModelYaml(
    const std::string& model_yaml,
    const std::string& link_name) {
    if (model_yaml.empty()) {
        return {};
    }
    try {
        YAML::Node root = YAML::Load(model_yaml);
        return LookupFromSequence(root["model"]["rsdas"], link_name);
    } catch (...) {
        return {};
    }
}

bool LookupTsdaDampingFromModelYaml(const std::string& model_yaml,
                                    const std::string& sanitized_name,
                                    double& c_out) {
    const auto coeffs =
        LookupTsdaSpringDamperFromModelYaml(model_yaml, sanitized_name);
    if (!coeffs.found || !coeffs.has_damping) {
        return false;
    }
    c_out = coeffs.damping_coefficient;
    return true;
}

bool LookupRsdaDampingFromModelYaml(const std::string& model_yaml,
                                    const std::string& sanitized_name,
                                    double& c_out) {
    const auto coeffs =
        LookupRsdaSpringDamperFromModelYaml(model_yaml, sanitized_name);
    if (!coeffs.found || !coeffs.has_damping) {
        return false;
    }
    c_out = coeffs.damping_coefficient;
    return true;
}

}  // namespace seastack::chrono
