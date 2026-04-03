/**
 * @file doctor.cpp
 * @brief `--doctor` environment and package diagnostics for SEA-Stack.
 *
 * ## What --doctor checks
 *   1. Executable startup (trivial — confirms the process reached this code)
 *   2. Version / build info (SEASTACK_VERSION, build type, Chrono version)
 *   3. Package root detection (exe-relative data/chrono directory)
 *   4. Expected package paths (data/chrono/, demos/)
 *   5. Writable output directory (temp-file probe in cwd)
 *   6. Visualization availability (compile-time VSG flag)
 *   7. Chrono data sanity (data dir is non-empty)
 *
 * ## Diagnostics file
 *   Written to <cwd>/seastack_doctor_<YYYYMMDD_HHMMSS>.txt
 *
 * ## Intended use
 *   - First-run validation after installing a package
 *   - CI smoke test for packaged builds
 *   - User-support artifact ("please attach the diagnostics file")
 */

#include "doctor.h"

#include <seastack/config.h>
#include <seastack/version.h>
#include <seastack/infra/logging.h>

#include <chrono>
#include <ctime>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef SEASTACK_VERSION
#define SEASTACK_VERSION "unknown"
#endif
#ifndef SEASTACK_BUILD_TYPE
#define SEASTACK_BUILD_TYPE "unknown"
#endif
#ifndef CHRONO_VERSION
#define CHRONO_VERSION "unknown"
#endif

namespace seastack::app {

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

static std::string StatusTag(DoctorCheck::Status s) {
    switch (s) {
        case DoctorCheck::PASS: return "PASS";
        case DoctorCheck::WARN: return "WARN";
        case DoctorCheck::FAIL: return "FAIL";
    }
    return "????";
}

// Fills *out with local calendar time, or UTC on failure. Returns false if both
// fail (caller must not pass *out to std::put_time).
static bool FillCalendarTime(std::time_t t, std::tm* out) {
#ifdef _WIN32
    if (localtime_s(out, &t) == 0) return true;
    return gmtime_s(out, &t) == 0;
#else
    if (localtime_r(&t, out) != nullptr) return true;
    return gmtime_r(&t, out) != nullptr;
#endif
}

static std::string Now_YYYYMMDD_HHMMSS() {
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    if (!FillCalendarTime(tt, &tm)) {
        return "invalid-time";
    }
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return ss.str();
}

static std::string Now_ISO8601() {
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    if (!FillCalendarTime(tt, &tm)) {
        return "invalid-time";
    }
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
    return ss.str();
}

/// Resolve the package root as <exe_dir>/.. (lexically normalised).
static std::filesystem::path PackageRoot(const std::string& exe_dir) {
    if (exe_dir.empty()) return {};
    return (std::filesystem::path(exe_dir) / "..").lexically_normal();
}

// -----------------------------------------------------------------------
// Individual checks
// -----------------------------------------------------------------------

DoctorCheck CheckExecutableStartup() {
    return {"Executable startup", DoctorCheck::PASS, "Process running", ""};
}

DoctorCheck CheckVersionInfo() {
    std::string ver = SEASTACK_VERSION;
    std::string build = SEASTACK_BUILD_TYPE;
    std::string chrono = CHRONO_VERSION;

    std::string detail = std::string("SEA-Stack ") + ver +
                         " (" + build + "), Chrono " + chrono;

    if (ver == "unknown") {
        return {"Version/build info", DoctorCheck::WARN, detail,
                "Binary may not have been built with version info"};
    }
    return {"Version/build info", DoctorCheck::PASS, detail, ""};
}

DoctorCheck CheckPackageRoot(const std::string& exe_dir) {
    if (exe_dir.empty()) {
        return {"Package root", DoctorCheck::FAIL,
                "Could not determine executable directory",
                "Verify the executable path is accessible"};
    }

    auto pkg = PackageRoot(exe_dir);
    auto chrono_data = pkg / "data" / "chrono";
    if (std::filesystem::exists(chrono_data) &&
        std::filesystem::is_directory(chrono_data)) {
        return {"Package root", DoctorCheck::PASS,
                pkg.string(), ""};
    }

    // Fallback: build-tree path from SEASTACK_DATA_DIR_PATH
#ifdef SEASTACK_DATA_DIR_PATH
    std::filesystem::path build_data =
        std::filesystem::path(SEASTACK_DATA_DIR_PATH) / "chrono";
    if (std::filesystem::exists(build_data)) {
        return {"Package root", DoctorCheck::WARN,
                "Using build-tree fallback: " + build_data.string(),
                "Verify package layout or set CHRONO_DATA_DIR"};
    }
#endif

    return {"Package root", DoctorCheck::FAIL,
            "data/chrono not found under " + pkg.string(),
            "Verify the package was extracted completely"};
}

DoctorCheck CheckExpectedPaths(const std::string& exe_dir) {
    if (exe_dir.empty()) {
        return {"Expected paths", DoctorCheck::FAIL,
                "Cannot check — exe directory unknown", ""};
    }

    auto pkg = PackageRoot(exe_dir);
    struct Entry {
        std::string rel;
        bool required;
    };
    std::vector<Entry> entries = {
        {"data/chrono", true},
        {"demos", false},
    };

    std::vector<std::string> missing_required;
    std::vector<std::string> missing_optional;

    for (auto& e : entries) {
        auto p = pkg / e.rel;
        if (!std::filesystem::exists(p)) {
            if (e.required) missing_required.push_back(e.rel);
            else            missing_optional.push_back(e.rel);
        }
    }

    if (!missing_required.empty()) {
        std::string detail = "Missing required: ";
        for (size_t i = 0; i < missing_required.size(); ++i) {
            if (i) detail += ", ";
            detail += missing_required[i];
        }
        return {"Expected paths", DoctorCheck::FAIL, detail,
                "Verify the package was extracted completely"};
    }
    if (!missing_optional.empty()) {
        std::string detail = "Missing optional: ";
        for (size_t i = 0; i < missing_optional.size(); ++i) {
            if (i) detail += ", ";
            detail += missing_optional[i];
        }
        return {"Expected paths", DoctorCheck::WARN, detail,
                "Demo data may not be available"};
    }
    return {"Expected paths", DoctorCheck::PASS,
            "All expected paths present under " + pkg.string(), ""};
}

DoctorCheck CheckWritableDirectory(const std::string& dir) {
    if (dir.empty()) {
        return {"Writable output directory", DoctorCheck::FAIL,
                "Directory path is empty",
                "Run from a directory with write permissions"};
    }

    std::filesystem::path probe = std::filesystem::path(dir) /
                                  "seastack_doctor_probe.tmp";
    try {
        {
            std::ofstream f(probe, std::ios::out | std::ios::trunc);
            if (!f.is_open()) {
                return {"Writable output directory", DoctorCheck::FAIL,
                        "Cannot create file in " + dir,
                        "Check write permissions or run from a writable directory"};
            }
            f << "probe";
        }
        std::filesystem::remove(probe);
        return {"Writable output directory", DoctorCheck::PASS, dir, ""};
    } catch (const std::exception& e) {
        try { std::filesystem::remove(probe); } catch (...) {}
        return {"Writable output directory", DoctorCheck::FAIL,
                std::string("Write test failed: ") + e.what(),
                "Check write permissions or run from a writable directory"};
    }
}

DoctorCheck CheckVisualization() {
#ifdef SEASTACK_HAVE_VSG
    return {"Visualization", DoctorCheck::PASS,
            "VSG available (compile-time)", ""};
#else
    return {"Visualization", DoctorCheck::WARN,
            "VSG not compiled in — GUI visualization unavailable",
            "Use --nogui for headless simulation"};
#endif
}

DoctorCheck CheckChronoData(const std::string& exe_dir) {
    // Find the Chrono data dir using the same logic as the runner.
    std::filesystem::path chrono_data;
    if (!exe_dir.empty()) {
        auto candidate = (std::filesystem::path(exe_dir) / ".." / "data" / "chrono")
                             .lexically_normal();
        if (std::filesystem::exists(candidate) &&
            std::filesystem::is_directory(candidate)) {
            chrono_data = candidate;
        }
    }
#ifdef SEASTACK_DATA_DIR_PATH
    if (chrono_data.empty()) {
        auto fallback = std::filesystem::path(SEASTACK_DATA_DIR_PATH) / "chrono";
        if (std::filesystem::exists(fallback)) chrono_data = fallback;
    }
#endif

    if (chrono_data.empty()) {
        return {"Chrono data", DoctorCheck::FAIL,
                "Chrono data directory not found",
                "Verify data/chrono/ was included in the package"};
    }

    // Sanity: directory should contain at least one entry.
    try {
        auto it = std::filesystem::directory_iterator(chrono_data);
        if (it == std::filesystem::directory_iterator{}) {
            return {"Chrono data", DoctorCheck::FAIL,
                    "Directory is empty: " + chrono_data.string(),
                    "Verify data/chrono/ was included in the package"};
        }
    } catch (const std::exception& e) {
        return {"Chrono data", DoctorCheck::FAIL,
                std::string("Cannot read directory: ") + e.what(),
                "Check directory permissions"};
    }

    return {"Chrono data", DoctorCheck::PASS, chrono_data.string(), ""};
}

// -----------------------------------------------------------------------
// Diagnostics file
// -----------------------------------------------------------------------

bool WriteDiagnosticsFile(const std::string& path,
                          const std::vector<DoctorCheck>& checks) {
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f.is_open()) return false;

    f << "SEA-Stack Doctor Diagnostics\n";
    f << "============================\n";
    f << "Timestamp:       " << Now_ISO8601() << "\n";
    f << "Platform:        " << seastack::infra::GetPlatformSummary() << "\n";
    f << "SEA-Stack:       " << SEASTACK_VERSION << " (" << SEASTACK_BUILD_TYPE << ")\n";
    f << "Chrono:          " << CHRONO_VERSION << "\n";
    f << "Executable:      " << seastack::infra::GetExecutablePath() << "\n";
    f << "Exe directory:   " << seastack::infra::GetExecutableDirectory() << "\n";
    f << "Working dir:     " << std::filesystem::current_path().string() << "\n";
    f << "\nCheck Results\n";
    f << "-------------\n";

    for (auto& c : checks) {
        f << "[" << StatusTag(c.status) << "] " << c.name;
        if (!c.detail.empty()) f << ": " << c.detail;
        f << "\n";
        if (!c.action.empty()) {
            f << "       -> " << c.action << "\n";
        }
    }

    f << "\n";
    return true;
}

// -----------------------------------------------------------------------
// RunDoctor — orchestrator
// -----------------------------------------------------------------------

int RunDoctor(const std::string& /*exe_path*/) {
    std::string exe_dir = seastack::infra::GetExecutableDirectory();
    std::string cwd = std::filesystem::current_path().string();

    std::vector<DoctorCheck> checks;
    checks.push_back(CheckExecutableStartup());
    checks.push_back(CheckVersionInfo());
    checks.push_back(CheckPackageRoot(exe_dir));
    checks.push_back(CheckExpectedPaths(exe_dir));
    checks.push_back(CheckWritableDirectory(cwd));
    checks.push_back(CheckVisualization());
    checks.push_back(CheckChronoData(exe_dir));

    // --- Console report ---
    std::string platform;
#ifdef _WIN32
    platform = "Windows";
#elif defined(__APPLE__)
    platform = "macOS";
#else
    platform = "Linux";
#endif

    std::cout << "\nSEA-Stack Doctor  (v" << SEASTACK_VERSION
              << ", " << SEASTACK_BUILD_TYPE << ", " << platform << ")\n";
    std::cout << "=============================================\n";

    int pass_count = 0, warn_count = 0, fail_count = 0;
    for (auto& c : checks) {
        std::cout << "[" << StatusTag(c.status) << "] " << c.name;
        if (!c.detail.empty()) std::cout << ": " << c.detail;
        std::cout << "\n";
        if (!c.action.empty()) {
            std::cout << "       -> " << c.action << "\n";
        }
        switch (c.status) {
            case DoctorCheck::PASS: ++pass_count; break;
            case DoctorCheck::WARN: ++warn_count; break;
            case DoctorCheck::FAIL: ++fail_count; break;
        }
    }

    std::cout << "=============================================\n";
    std::cout << fail_count << " FAIL, " << warn_count << " WARN, "
              << pass_count << " PASS\n";

    // --- Diagnostics file ---
    std::string diag_filename = "seastack_doctor_" + Now_YYYYMMDD_HHMMSS() + ".txt";
    std::string diag_path = (std::filesystem::path(cwd) / diag_filename).string();

    bool wrote = WriteDiagnosticsFile(diag_path, checks);

    // --- Next steps ---
    if (fail_count > 0 || warn_count > 0) {
        std::cout << "\nNext steps:\n";
        if (fail_count > 0) {
            std::cout << "  - Fix FAILed checks above before running simulations.\n";
        }
        // Contextual suggestions based on actual results
        for (auto& c : checks) {
            if (c.status != DoctorCheck::PASS && !c.action.empty()) {
                std::cout << "  - " << c.action << "\n";
            }
        }
    } else {
        std::cout << "\nAll checks passed. Environment looks ready.\n";
    }

    if (wrote) {
        std::cout << "  - Diagnostics saved to: " << diag_path << "\n";
    } else {
        std::cerr << "  - WARNING: Could not write diagnostics file to " << diag_path << "\n";
    }
    std::cout << "\n";

    return (fail_count > 0) ? 1 : 0;
}

}  // namespace seastack::app
