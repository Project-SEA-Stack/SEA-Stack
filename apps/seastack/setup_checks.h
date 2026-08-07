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

/// One declarative check. Parameters vary by `type`; each check reads only the
/// fields listed for it and ignores the rest.
///   - water_depth_consistency: tolerance_m
///   - bridge_static_load_path: time_s, tolerance_percent
struct CheckConfig {
    std::string type;
    double tolerance_m = 0.01;         ///< m
    double time_s = 2.0;               ///< s, simulated time at which to sample
    double tolerance_percent = 10.0;   ///< %
};

/// Parse the `checks:` sequence from a setup YAML.  Returns an empty vector
/// when the block is absent; logs a debug message and returns empty on a
/// malformed block.
std::vector<CheckConfig> LoadChecksFromSetupYaml(const std::filesystem::path& setup_path);

/// Return the first check of the given type, or nullptr when not requested.
const CheckConfig* FindCheck(const std::vector<CheckConfig>& checks, const std::string& type);

/// Evaluate the checks that can only be judged once the run has finished.
/// Currently that is `bridge_static_load_path`, which needs a StructureSubsystem
/// to have sampled its bearing reactions.
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
