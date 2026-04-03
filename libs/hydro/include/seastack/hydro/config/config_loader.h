/*********************************************************************
 * @file  config_loader.h
 *
 * @brief Simple helper to load hydrodynamics config from YAML.
 *
 * This wrapper keeps the handwritten parser in one place and gives
 * callers a stable, easy-to-read entry point.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_CONFIG_CONFIG_LOADER_H
#define SEASTACK_HYDRO_CONFIG_CONFIG_LOADER_H

#include <seastack/hydro/config/hydro_config.h>
#include <string>

namespace seastack {
namespace hydro {

/**
 * @brief Load hydrodynamics configuration from a YAML file.
 *
 * This function keeps the existing parsing behaviour exactly the same.
 * It simply forwards to the legacy ReadHydroYAML helper.
 *
 * @param yaml_path Absolute or relative path to hydro.yaml.
 * @return Parsed YAMLHydroData structure.
 * @throws std::runtime_error on parsing errors (same as before).
 */
YAMLHydroData LoadHydroConfigFromYaml(const std::string& yaml_path);

}  // namespace hydro
}  // namespace seastack

#endif  // SEASTACK_HYDRO_CONFIG_CONFIG_LOADER_H

