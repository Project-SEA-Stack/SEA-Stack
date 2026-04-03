/**
 * @file test_doctor.cpp
 * @brief Unit tests for --doctor diagnostic checks.
 *
 * Tests exercise the individual check functions without requiring a full
 * simulation environment or Chrono libraries.
 */

#include "doctor.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static int tests_run = 0;
static int tests_passed = 0;

static void Check(bool cond, const std::string& name) {
    ++tests_run;
    if (cond) {
        ++tests_passed;
        std::cout << "[PASS] " << name << "\n";
    } else {
        std::cerr << "[FAIL] " << name << "\n";
    }
}

// ---------------------------------------------------------------------------
// Test: CheckWritableDirectory on a writable temp directory
// ---------------------------------------------------------------------------
static void TestWritableDirectoryPass() {
    fs::path tmp = fs::temp_directory_path() / "seastack_doctor_test";
    fs::create_directories(tmp);

    auto result = seastack::app::CheckWritableDirectory(tmp.string());
    Check(result.status == seastack::app::DoctorCheck::PASS,
          "CheckWritableDirectory PASS on writable temp dir");

    fs::remove_all(tmp);
}

// ---------------------------------------------------------------------------
// Test: CheckWritableDirectory on a non-existent / unwritable path
// ---------------------------------------------------------------------------
static void TestWritableDirectoryFail() {
    // Use a path that almost certainly does not exist and cannot be written.
    std::string bad_path;
#ifdef _WIN32
    bad_path = "Z:\\nonexistent_seastack_test_dir_12345";
#else
    bad_path = "/nonexistent_seastack_test_dir_12345";
#endif

    auto result = seastack::app::CheckWritableDirectory(bad_path);
    Check(result.status == seastack::app::DoctorCheck::FAIL,
          "CheckWritableDirectory FAIL on non-existent path");
}

// ---------------------------------------------------------------------------
// Test: CheckWritableDirectory with empty string
// ---------------------------------------------------------------------------
static void TestWritableDirectoryEmpty() {
    auto result = seastack::app::CheckWritableDirectory("");
    Check(result.status == seastack::app::DoctorCheck::FAIL,
          "CheckWritableDirectory FAIL on empty string");
}

// ---------------------------------------------------------------------------
// Test: CheckPackageRoot with empty exe_dir
// ---------------------------------------------------------------------------
static void TestPackageRootEmpty() {
    auto result = seastack::app::CheckPackageRoot("");
    Check(result.status == seastack::app::DoctorCheck::FAIL,
          "CheckPackageRoot FAIL on empty exe_dir");
}

// ---------------------------------------------------------------------------
// Test: CheckPackageRoot with a constructed directory containing data/chrono
// ---------------------------------------------------------------------------
static void TestPackageRootConstructed() {
    fs::path tmp = fs::temp_directory_path() / "seastack_doctor_test_pkg";
    // Create <tmp>/bin and <tmp>/data/chrono to simulate installed layout
    fs::path bin_dir = tmp / "bin";
    fs::path data_dir = tmp / "data" / "chrono";
    fs::create_directories(bin_dir);
    fs::create_directories(data_dir);

    // exe_dir simulates <pkg>/bin
    auto result = seastack::app::CheckPackageRoot(bin_dir.string());
    Check(result.status == seastack::app::DoctorCheck::PASS,
          "CheckPackageRoot PASS with constructed package layout");

    fs::remove_all(tmp);
}

// ---------------------------------------------------------------------------
// Test: CheckExecutableStartup always passes
// ---------------------------------------------------------------------------
static void TestExecutableStartup() {
    auto result = seastack::app::CheckExecutableStartup();
    Check(result.status == seastack::app::DoctorCheck::PASS,
          "CheckExecutableStartup always PASS");
}

// ---------------------------------------------------------------------------
// Test: CheckVersionInfo returns PASS (version is baked in at build time)
// ---------------------------------------------------------------------------
static void TestVersionInfo() {
    auto result = seastack::app::CheckVersionInfo();
    Check(result.status == seastack::app::DoctorCheck::PASS ||
          result.status == seastack::app::DoctorCheck::WARN,
          "CheckVersionInfo returns PASS or WARN");
    Check(!result.detail.empty(),
          "CheckVersionInfo detail is non-empty");
}

// ---------------------------------------------------------------------------
// Test: WriteDiagnosticsFile creates a file with expected content
// ---------------------------------------------------------------------------
static void TestDiagnosticsFile() {
    fs::path tmp = fs::temp_directory_path() / "seastack_doctor_test_diag";
    fs::create_directories(tmp);
    std::string diag_path = (tmp / "test_diag.txt").string();

    std::vector<seastack::app::DoctorCheck> checks;
    checks.push_back({"Test check", seastack::app::DoctorCheck::PASS, "detail", ""});
    checks.push_back({"Fail check", seastack::app::DoctorCheck::FAIL, "bad", "fix it"});

    bool wrote = seastack::app::WriteDiagnosticsFile(diag_path, checks);
    Check(wrote, "WriteDiagnosticsFile returns true");
    Check(fs::exists(diag_path), "Diagnostics file exists");

    // Verify file contains expected markers (close stream before remove_all on
    // Windows — deleting a tree while a file inside is open can corrupt CRT state).
    std::string content;
    {
        std::ifstream f(diag_path);
        content.assign(std::istreambuf_iterator<char>(f),
                       std::istreambuf_iterator<char>());
    }
    Check(content.find("SEA-Stack Doctor") != std::string::npos,
          "Diagnostics file contains header");
    Check(content.find("[PASS] Test check") != std::string::npos,
          "Diagnostics file contains PASS check");
    Check(content.find("[FAIL] Fail check") != std::string::npos,
          "Diagnostics file contains FAIL check");
    Check(content.find("fix it") != std::string::npos,
          "Diagnostics file contains action text");

    fs::remove_all(tmp);
}

// ---------------------------------------------------------------------------
// Test: CheckExpectedPaths with valid layout
// ---------------------------------------------------------------------------
static void TestExpectedPathsPass() {
    fs::path tmp = fs::temp_directory_path() / "seastack_doctor_test_paths";
    fs::path bin_dir = tmp / "bin";
    fs::create_directories(bin_dir);
    fs::create_directories(tmp / "data" / "chrono");
    fs::create_directories(tmp / "demos");

    auto result = seastack::app::CheckExpectedPaths(bin_dir.string());
    Check(result.status == seastack::app::DoctorCheck::PASS,
          "CheckExpectedPaths PASS with full layout");

    fs::remove_all(tmp);
}

// ---------------------------------------------------------------------------
// Test: CheckExpectedPaths with missing demos (optional -> WARN)
// ---------------------------------------------------------------------------
static void TestExpectedPathsMissingOptional() {
    fs::path tmp = fs::temp_directory_path() / "seastack_doctor_test_paths2";
    fs::path bin_dir = tmp / "bin";
    fs::create_directories(bin_dir);
    fs::create_directories(tmp / "data" / "chrono");
    // No demos/ directory

    auto result = seastack::app::CheckExpectedPaths(bin_dir.string());
    Check(result.status == seastack::app::DoctorCheck::WARN,
          "CheckExpectedPaths WARN when optional demos/ missing");

    fs::remove_all(tmp);
}

int main() {
    TestExecutableStartup();
    TestVersionInfo();
    TestWritableDirectoryPass();
    TestWritableDirectoryFail();
    TestWritableDirectoryEmpty();
    TestPackageRootEmpty();
    TestPackageRootConstructed();
    TestExpectedPathsPass();
    TestExpectedPathsMissingOptional();
    TestDiagnosticsFile();

    std::cout << "\n" << tests_passed << "/" << tests_run << " tests passed.\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
