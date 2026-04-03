/*********************************************************************
 * @file  config_loader.cpp
 *
 * @brief Implementation of the simple hydrodynamics config loader.
 *********************************************************************/

#include <seastack/hydro/config/config_loader.h>
#include <seastack/hydro/config/yaml_parser.h>

namespace seastack {
namespace hydro {

YAMLHydroData LoadHydroConfigFromYaml(const std::string& yaml_path) {
    return ReadHydroYAML(yaml_path);
}

}  // namespace hydro
}  // namespace seastack

