/*********************************************************************
 * @file  model_yaml_link_coeffs.h
 * @brief Look up TSDA/RSDA spring-damper coefficients from Chrono MBS model YAML.
 *
 * Chrono’s YAML parser embeds k/c in a force functor and does not copy them
 * onto ChLinkTSDA/RSDA. SEA-Stack recovers them for export metrics and for
 * optional external-PTO + native spring-damper combining.
 *********************************************************************/
#ifndef SEASTACK_ADAPTERS_CHRONO_MODEL_YAML_LINK_COEFFS_H
#define SEASTACK_ADAPTERS_CHRONO_MODEL_YAML_LINK_COEFFS_H

#include <string>

namespace seastack::chrono {

/// Coefficients from a named TSDA/RSDA entry in model YAML.
struct LinkSpringDamperCoeffs {
    double spring_coefficient = 0.0;
    double damping_coefficient = 0.0;
    double preload = 0.0;
    bool found = false;       ///< named link entry exists
    bool has_spring = false;
    bool has_damping = false;
    bool has_preload = false;
};

/// Match Chrono sanitized link name to a YAML `name` (exact or `_suffix`).
bool YamlLinkNameMatches(const std::string& sanitized_link_name,
                         const std::string& yaml_name_raw);

/// Read spring/damping/preload for a TSDA named in model YAML content.
LinkSpringDamperCoeffs LookupTsdaSpringDamperFromModelYaml(
    const std::string& model_yaml,
    const std::string& link_name);

/// Read spring/damping/preload for an RSDA named in model YAML content.
LinkSpringDamperCoeffs LookupRsdaSpringDamperFromModelYaml(
    const std::string& model_yaml,
    const std::string& link_name);

/// Convenience: damping only (used by SimulationExporter).
bool LookupTsdaDampingFromModelYaml(const std::string& model_yaml,
                                    const std::string& sanitized_name,
                                    double& c_out);

bool LookupRsdaDampingFromModelYaml(const std::string& model_yaml,
                                    const std::string& sanitized_name,
                                    double& c_out);

}  // namespace seastack::chrono

#endif  // SEASTACK_ADAPTERS_CHRONO_MODEL_YAML_LINK_COEFFS_H
