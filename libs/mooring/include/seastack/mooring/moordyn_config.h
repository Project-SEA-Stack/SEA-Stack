/*********************************************************************
 * @file  moordyn_config.h
 * @brief Configuration struct for MoorDyn mooring coupling.
 *********************************************************************/

#ifndef SEASTACK_MOORING_MOORDYN_CONFIG_H
#define SEASTACK_MOORING_MOORDYN_CONFIG_H

#include <string>
#include <vector>

namespace seastack::mooring {

struct MoorDynConfig {
    bool enabled = false;
    std::string input_file;
    /// 0-based indices into the body list that correspond to MoorDyn
    /// coupled bodies, in the order they appear in the MoorDyn input
    /// file's Body List.
    std::vector<int> coupled_body_indices;
};

}  // namespace seastack::mooring

#endif  // SEASTACK_MOORING_MOORDYN_CONFIG_H
