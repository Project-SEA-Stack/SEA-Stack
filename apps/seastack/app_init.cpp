/*********************************************************************
 * @file  app_init.cpp
 * @brief Shared application initialization for all run_seastack modes.
 *********************************************************************/

#include "app_init.h"

#include <seastack/adapters/chrono/helper.h>
#include <seastack/infra/logging.h>

#include <atomic>
#include <cstdlib>
#include <string>

namespace {
std::atomic<bool> g_chrono_env_initialized{false};
}

void seastack::app::InitChronoEnvironment() {
    if (g_chrono_env_initialized.exchange(true)) {
        return;
    }

    seastack::chrono::SetInitialEnvironment("");

    // Export the resolved Chrono data path so that child processes (--run-cell
    // subprocesses) inherit it without repeating the probe logic.
    std::string chrono_path = seastack::chrono::GetChronoDataDir();
    if (!chrono_path.empty()) {
#ifdef _WIN32
        _putenv_s("CHRONO_DATA_DIR", chrono_path.c_str());
#else
        setenv("CHRONO_DATA_DIR", chrono_path.c_str(), 1);
#endif
        seastack::infra::debug::LogDebug(
            std::string("Exported CHRONO_DATA_DIR=") + chrono_path);
    }
}
