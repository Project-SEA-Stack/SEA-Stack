/*********************************************************************
 * @file  campaign_runner.h
 * @brief Power-matrix campaign runner for SEA-Stack.
 *
 * Parses a campaign YAML (kind: performance_matrix), enumerates cells
 * over the specified axes, applies a steepness filter, runs each cell
 * via RunSingleCase, and writes a sparse summary HDF5 (+ optional CSV).
 *********************************************************************/

#ifndef SEASTACK_APP_CAMPAIGN_RUNNER_H
#define SEASTACK_APP_CAMPAIGN_RUNNER_H

#include <string>

namespace seastack::app {

/// Run a power-matrix campaign from a campaign YAML file.
/// Returns 0 on success (even if individual cells fail), non-zero on
/// fatal parse/IO errors.
int RunCampaign(const std::string& campaign_yaml_path,
                bool debug_mode   = false,
                bool profile_mode = false,
                bool quiet_mode   = false);

}  // namespace seastack::app

#endif  // SEASTACK_APP_CAMPAIGN_RUNNER_H
