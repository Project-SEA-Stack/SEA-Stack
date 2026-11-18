/*********************************************************************
 * @file  replay_run.h
 * @brief Offline replay of a recorded run (run_seastack --replay).
 *********************************************************************/

#ifndef SEASTACK_APP_REPLAY_RUN_H
#define SEASTACK_APP_REPLAY_RUN_H

#include <string>

namespace seastack::app {

/// Show a results file in the SEA-Stack viewer at any playback speed, without
/// building or stepping a Chrono system. Visual only.
///
/// @param results_path  HDF5 results file written by run_seastack (schema 1.1 or later).
/// @return 0 when the window is closed normally, 1 if the file cannot be replayed.
int RunReplay(const std::string& results_path);

}  // namespace seastack::app

#endif  // SEASTACK_APP_REPLAY_RUN_H
