/*********************************************************************
 * @file  app_init.h
 * @brief Shared application initialization for all run_seastack modes.
 *
 * Centralizes Chrono data-path setup so that single-run, campaign,
 * and subprocess (--run-cell) modes all behave identically.
 *********************************************************************/

#ifndef SEASTACK_APP_INIT_H
#define SEASTACK_APP_INIT_H

namespace seastack::app {

/// Configure the Chrono data path and export CHRONO_DATA_DIR for child
/// processes.  Delegates to seastack::chrono::SetInitialEnvironment()
/// for the heavy lifting.  Safe to call more than once (idempotent).
void InitChronoEnvironment();

}  // namespace seastack::app

#endif  // SEASTACK_APP_INIT_H
