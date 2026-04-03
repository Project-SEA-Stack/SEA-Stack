/**
 * @file doctor.h
 * @brief `--doctor` environment/package diagnostics for SEA-Stack.
 */
#pragma once

#include <string>
#include <vector>

namespace seastack::app {

struct DoctorCheck {
    std::string name;
    enum Status { PASS, WARN, FAIL } status;
    std::string detail;
    std::string action;
};

/// Run all doctor checks, print a console report, write a diagnostics file,
/// and return 0 (no FAILs) or 1 (at least one FAIL).
int RunDoctor(const std::string& exe_path);

// Individual checks exposed for unit testing.
DoctorCheck CheckExecutableStartup();
DoctorCheck CheckVersionInfo();
DoctorCheck CheckPackageRoot(const std::string& exe_dir);
DoctorCheck CheckExpectedPaths(const std::string& exe_dir);
DoctorCheck CheckWritableDirectory(const std::string& dir);
DoctorCheck CheckVisualization();
DoctorCheck CheckChronoData(const std::string& exe_dir);

/// Write diagnostics file to @p path. Returns true on success.
bool WriteDiagnosticsFile(const std::string& path,
                          const std::vector<DoctorCheck>& checks);

}  // namespace seastack::app
