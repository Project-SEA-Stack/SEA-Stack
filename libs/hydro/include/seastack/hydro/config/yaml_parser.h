/*********************************************************************
 * @file  yaml_parser.h
 *
 * @brief YAML parser for hydro.yaml files.
 *
 * This is an internal header. Library consumers should use the public
 * LoadHydroConfigFromYaml() function from config_loader.h instead.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_CONFIG_YAML_PARSER_H
#define SEASTACK_HYDRO_CONFIG_YAML_PARSER_H

#include <seastack/hydro/config/hydro_config.h>
#include <string>

namespace seastack::hydro {

/**
 * @brief Read and parse a hydro.yaml file into YAMLHydroData structure.
 *
 * @param hydro_file_path Path to the hydro.yaml file to parse.
 * @return YAMLHydroData containing the parsed configuration.
 * @throws std::runtime_error if the file cannot be read or parsed.
 */
YAMLHydroData ReadHydroYAML(const std::string& hydro_file_path);

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_CONFIG_YAML_PARSER_H

