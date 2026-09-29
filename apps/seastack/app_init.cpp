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

#ifdef __APPLE__
#include <filesystem>
#include <system_error>
#endif

namespace {
std::atomic<bool> g_chrono_env_initialized{false};

#ifdef __APPLE__
// The Vulkan loader only searches system/XDG locations for drivers when the executable
// is not inside a .app bundle. Point it at the MoltenVK manifest shipped in the release
// ZIP (<root>/share/vulkan/icd.d, next to <root>/bin). Build-tree runs have no such
// file and keep using the system driver; an explicit user override always wins.
void UseBundledVulkanDriver() {
    if (std::getenv("VK_DRIVER_FILES") != nullptr || std::getenv("VK_ICD_FILENAMES") != nullptr) {
        return;
    }
    const std::string exe_dir = seastack::infra::GetExecutableDirectory();
    if (exe_dir.empty()) {
        return;
    }
    const std::filesystem::path icd = std::filesystem::path(exe_dir).parent_path() / "share" /
                                      "vulkan" / "icd.d" / "MoltenVK_icd.json";
    std::error_code ec;
    if (!std::filesystem::is_regular_file(icd, ec)) {
        return;
    }
    setenv("VK_DRIVER_FILES", icd.c_str(), 1);
    seastack::infra::debug::LogDebug("Exported VK_DRIVER_FILES=" + icd.string());
}
#endif
}  // namespace

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

#ifdef __APPLE__
    UseBundledVulkanDriver();
#endif
}
