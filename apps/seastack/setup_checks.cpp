/*********************************************************************
 * @file  setup_checks.cpp
 * @brief Parsing and evaluation of declarative setup-YAML checks.
 *********************************************************************/

#include "setup_checks.h"

#include "structure_subsystem.h"
#include "yaml_read_helpers.h"

#include <seastack/infra/logging.h>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

namespace seastack::app {

namespace {

/// Earlier name of structure_static_load_path, kept so existing cases still run.
constexpr const char* kBridgeStaticLoadPathAlias = "bridge_static_load_path";

}  // namespace

std::vector<CheckConfig> LoadChecksFromSetupYaml(const std::filesystem::path& setup_path) {
    namespace yr = yaml_read;
    const std::string file_where = "setup " + setup_path.filename().string();

    YAML::Node root;
    try {
        root = YAML::LoadFile(setup_path.string());
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(file_where + ": YAML syntax error: " + e.what());
    }

    std::vector<CheckConfig> checks;
    if (!root.IsMap() || !root["checks"] || root["checks"].IsNull()) {
        return checks;
    }
    yr::RequireSequence(root["checks"], file_where + ": checks");

    std::size_t index = 0;
    for (const auto& node : root["checks"]) {
        const std::string where = file_where + ": checks[" + std::to_string(index++) + "]";
        yr::RequireMap(node, where);
        yr::RequireKey(node, "type", where);

        CheckConfig check;
        yr::ReadString(node, "type", check.type, where);
        if (check.type == kBridgeStaticLoadPathAlias) {
            check.type = kStructureStaticLoadPathCheck;
        }
        if (check.type != kStructureStaticLoadPathCheck) {
            const std::vector<std::string> known = {kStructureStaticLoadPathCheck};
            throw std::runtime_error(where + ": unknown check type '" + check.type + "'." +
                                     yr::DidYouMean(check.type, known) +
                                     " Supported types: " + yr::JoinNames(known) + ".");
        }

        yr::CheckKeys(node, {"type", "time_s", "tolerance_percent"}, where);
        yr::ReadDouble(node, "time_s", check.time_s, where);
        yr::ReadDouble(node, "tolerance_percent", check.tolerance_percent, where);
        if (!(check.time_s > 0.0)) {
            throw std::runtime_error(where + ".time_s must be > 0 s.");
        }
        if (!(check.tolerance_percent > 0.0)) {
            throw std::runtime_error(where + ".tolerance_percent must be > 0.");
        }
        checks.push_back(std::move(check));
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
    const std::string name = kStructureStaticLoadPathCheck;

    for (const auto& check : checks) {
        if (check.type != kStructureStaticLoadPathCheck) {
            continue;
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
                name + " check FAILED: the case requests it but has no structure_file, so "
                       "there is no FEA structure to measure.");
            all_passed = false;
            continue;
        }

        if (final_time < check.time_s) {
            seastack::infra::cli::LogError(
                name + " check FAILED: the run ended at " + std::to_string(final_time) +
                " s, before the check time of " + std::to_string(check.time_s) + " s.");
            all_passed = false;
            continue;
        }

        const auto& result = structure->GetStaticCheckResult();
        if (!result.valid) {
            seastack::infra::cli::LogError(name + " check FAILED: no measurement was taken because " +
                                           result.invalid_reason + ".");
            all_passed = false;
            continue;
        }

        if (std::abs(result.error_percent) > check.tolerance_percent) {
            seastack::infra::cli::LogError(
                name + " check FAILED: vertical mate reactions " +
                std::to_string(result.reactions_kN) + " kN vs structure weight " +
                std::to_string(result.weight_kN) + " kN (" + std::to_string(result.error_percent) +
                "% error, tolerance " + std::to_string(check.tolerance_percent) + "%)");
            all_passed = false;
        } else {
            seastack::infra::cli::LogInfo(name + " check PASSED: " +
                                          std::to_string(std::abs(result.error_percent)) +
                                          "% within " + std::to_string(check.tolerance_percent) +
                                          "% tolerance");
        }
    }

    return all_passed;
}

}  // namespace seastack::app
