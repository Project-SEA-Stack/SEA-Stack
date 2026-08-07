/*********************************************************************
 * @file  setup_checks.cpp
 * @brief Parsing and evaluation of declarative setup-YAML checks.
 *********************************************************************/

#include "setup_checks.h"

#include "structure_subsystem.h"

#include <seastack/infra/logging.h>

#include <yaml-cpp/yaml.h>

#include <cmath>

namespace seastack::app {

std::vector<CheckConfig> LoadChecksFromSetupYaml(const std::filesystem::path& setup_path) {
    std::vector<CheckConfig> checks;
    try {
        YAML::Node root = YAML::LoadFile(setup_path.string());
        if (!root["checks"] || !root["checks"].IsSequence()) {
            return checks;
        }
        for (const auto& check_node : root["checks"]) {
            CheckConfig check;
            if (check_node["type"]) {
                check.type = check_node["type"].as<std::string>();
            }
            if (check_node["tolerance_m"]) {
                check.tolerance_m = check_node["tolerance_m"].as<double>();
            }
            if (check_node["time_s"]) {
                check.time_s = check_node["time_s"].as<double>();
            }
            if (check_node["tolerance_percent"]) {
                check.tolerance_percent = check_node["tolerance_percent"].as<double>();
            }
            if (!check.type.empty()) {
                checks.push_back(std::move(check));
            }
        }
    } catch (const YAML::Exception& e) {
        seastack::infra::debug::LogDebug(std::string("Could not parse checks block: ") + e.what());
    } catch (const std::exception& e) {
        seastack::infra::debug::LogDebug(std::string("Could not load checks block: ") + e.what());
    }
    return checks;
}

const CheckConfig* FindCheck(const std::vector<CheckConfig>& checks, const std::string& type) {
    for (const auto& check : checks) {
        if (check.type == type) {
            return &check;
        }
    }
    return nullptr;
}

bool EvaluatePostRunChecks(const std::vector<CheckConfig>& checks,
                           const std::vector<std::shared_ptr<IScenarioSubsystem>>& subsystems,
                           double final_time) {
    bool all_passed = true;

    for (const auto& check : checks) {
        if (check.type != "bridge_static_load_path") {
            continue;  // water_depth_consistency runs at setup, not here
        }

        // A requested check must be evaluable. Each early exit below is a
        // failure, not a pass: a check that cannot fail is not a check.
        const StructureSubsystem* structure = nullptr;
        for (const auto& subsystem : subsystems) {
            if (auto* s = dynamic_cast<StructureSubsystem*>(subsystem.get())) {
                structure = s;
                break;
            }
        }
        if (structure == nullptr) {
            seastack::infra::cli::LogError(
                "bridge_static_load_path check FAILED: the case requests it but has no "
                "structure_file, so there is no FEA structure to measure.");
            all_passed = false;
            continue;
        }

        if (final_time < check.time_s) {
            seastack::infra::cli::LogError(
                "bridge_static_load_path check FAILED: the run ended at " +
                std::to_string(final_time) + " s, before the check time of " +
                std::to_string(check.time_s) + " s.");
            all_passed = false;
            continue;
        }

        const auto& result = structure->GetStaticCheckResult();
        if (!result.valid) {
            seastack::infra::cli::LogError(
                "bridge_static_load_path check FAILED: no measurement was taken because " +
                result.invalid_reason + ".");
            all_passed = false;
            continue;
        }

        if (std::abs(result.error_percent) > check.tolerance_percent) {
            seastack::infra::cli::LogError(
                "bridge_static_load_path check FAILED: bearing reactions " +
                std::to_string(result.reactions_kN) + " kN vs weight " +
                std::to_string(result.weight_kN) + " kN (" +
                std::to_string(result.error_percent) + "% error, tolerance " +
                std::to_string(check.tolerance_percent) + "%)");
            all_passed = false;
        } else {
            seastack::infra::cli::LogInfo(
                "bridge_static_load_path check PASSED: " +
                std::to_string(std::abs(result.error_percent)) + "% within " +
                std::to_string(check.tolerance_percent) + "% tolerance");
        }
    }

    return all_passed;
}

}  // namespace seastack::app
