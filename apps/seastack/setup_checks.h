/*********************************************************************
 * @file  setup_checks.h
 * @brief Declarative validation checks from a setup YAML `checks:` block.
 *
 * Infra's ParseSetupFile is a line scanner that deliberately stays free of
 * yaml-cpp, so it can only read flat path roles.  The nested `checks:`
 * sequence is parsed here, in the app layer, which already links yaml-cpp,
 * and is evaluated here too so the schema and its meaning live together.
 *
 * A check that is requested but cannot be evaluated (missing subsystem, run
 * too short) FAILS.  A check that silently passes is worse than no check.
 *********************************************************************/

#ifndef SEASTACK_APP_SETUP_CHECKS_H
#define SEASTACK_APP_SETUP_CHECKS_H

#include "scenario_subsystem.h"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace seastack::app {

/// Check type that compares the summed structure mate reactions against the
/// structure's own weight once the model has settled.
inline constexpr const char* kStructureStaticLoadPathCheck = "structure_static_load_path";

/// One declarative check.  Supported types and their keys:
///   - structure_static_load_path (alias: bridge_static_load_path):
///       time_s [s], tolerance_percent [%]
struct CheckConfig {
    std::string type;                 ///< canonical type (aliases are resolved on load)
    double time_s = 2.0;              ///< s, simulated time at which to sample
    double tolerance_percent = 10.0;  ///< %
};

/// Parse the `checks:` sequence from a setup YAML.  Returns an empty vector
/// when the block is absent.  Throws std::runtime_error on a malformed block,
/// an unknown check type, an unknown key, or an out-of-range value.
std::vector<CheckConfig> LoadChecksFromSetupYaml(const std::filesystem::path& setup_path);

/// Return the first check of the given type, or nullptr when not requested.
const CheckConfig* FindCheck(const std::vector<CheckConfig>& checks, const std::string& type);

/// Evaluate the checks that can only be judged once the run has finished.
/// Currently that is `structure_static_load_path`, which needs a
/// StructureSubsystem to have sampled its mate reactions.
///
/// @param checks      Checks requested in the setup YAML
/// @param subsystems  Scenario subsystems attached to this run
/// @param final_time  Simulated time reached [s]
/// @return true when every requested post-run check passed
bool EvaluatePostRunChecks(const std::vector<CheckConfig>& checks,
                           const std::vector<std::shared_ptr<IScenarioSubsystem>>& subsystems,
                           double final_time);

}  // namespace seastack::app

#endif  // SEASTACK_APP_SETUP_CHECKS_H
