/*********************************************************************
 * @file  test_yaml_parser.cpp
 * @brief Unit tests for the hand-rolled hydro YAML parser.
 *
 * Tests cover:
 *   1. Minimal valid regular-wave config
 *   2. Irregular wave with JONSWAP spectrum
 *   3. Body linear damping
 *   4. Radiation method selection (state_space, constant_coefficients)
 *   5. MoorDyn coupling section
 *   6. Empty file → error
 *   7. Missing wave section defaults
 *   8. Multi-body config
 *   9. Block-list forms (damping, moordyn.bodies, period.values), CRLF,
 *      loud errors for malformed lists, parse of every shipped hydro YAML
 *********************************************************************/

#include <seastack/hydro/config/yaml_parser.h>
#include <seastack/hydro/config/hydro_config.h>

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

using namespace seastack::hydro;

static constexpr double kTol = 1e-9;

/// Write a string to a temporary file (bytes written as-is) and return the path.
static std::string WriteTempYaml(const std::string& content, const std::string& suffix) {
    std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("test_yaml_" + suffix + ".yaml");
    std::ofstream f(path, std::ios::binary);
    f << content;
    f.close();
    return path.string();
}

/// Best effort: on Windows another process (e.g. a virus scanner) may briefly
/// hold a freshly written temp file open.
static void CleanupFile(const std::string& path) {
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

/// Like assert(), but also active in Release builds.
static void Check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error("check failed: " + message);
    }
}

/// Parse `yaml` and require a std::runtime_error whose message contains every
/// string in `expected`. Returns the message.
static std::string ExpectParseError(const std::string& yaml, const std::string& suffix,
                                    std::initializer_list<std::string> expected) {
    auto path = WriteTempYaml(yaml, suffix);
    std::string message;
    bool threw = false;
    try {
        ReadHydroYAML(path);
    } catch (const std::runtime_error& e) {
        threw = true;
        message = e.what();
    }
    CleanupFile(path);
    Check(threw, suffix + ": expected a parse error");
    for (const auto& s : expected) {
        Check(message.find(s) != std::string::npos,
              suffix + ": error message \"" + message + "\" does not contain \"" + s + "\"");
    }
    return message;
}

static std::string Filename(const std::string& path) {
    return std::filesystem::path(path).filename().string();
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: Minimal valid regular-wave config
// ─────────────────────────────────────────────────────────────────────────────
static bool test_minimal_regular() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: sphere.h5
  waves:
    type: regular
    height: 2.0
    period: 8.0
)";
    auto path = WriteTempYaml(yaml, "regular");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    assert(data.bodies.size() == 1);
    assert(data.bodies[0].name == "body1");
    assert(Filename(data.bodies[0].h5_file) == "sphere.h5");
    assert(data.waves.type == "regular");
    assert(std::abs(data.waves.height - 2.0) < kTol);
    assert(std::abs(data.waves.period - 8.0) < kTol);

    std::cout << "  PASSED: test_minimal_regular\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: Irregular wave with JONSWAP
// ─────────────────────────────────────────────────────────────────────────────
static bool test_irregular_jonswap() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: buoy.h5
  waves:
    type: irregular
    height: 3.5
    period: 10.0
    spectrum: jonswap
    gamma: 2.5
    nfrequencies: 128
    seed: 99
)";
    auto path = WriteTempYaml(yaml, "irreg");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    assert(data.waves.type == "irregular");
    assert(std::abs(data.waves.height - 3.5) < kTol);
    assert(std::abs(data.waves.period - 10.0) < kTol);
    assert(data.waves.spectrum == "jonswap");
    assert(std::abs(data.waves.gamma - 2.5) < kTol);
    assert(data.waves.nfrequencies == 128);
    assert(data.waves.seed == 99);

    std::cout << "  PASSED: test_irregular_jonswap\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: Body linear damping
// ─────────────────────────────────────────────────────────────────────────────
static bool test_body_linear_damping() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: hull.h5
      linear_damping: [100, 200, 300, 0, 0, 0]
  waves:
    type: no_wave
)";
    auto path = WriteTempYaml(yaml, "damping");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    assert(data.bodies.size() == 1);
    assert(std::abs(data.bodies[0].linear_damping[0] - 100.0) < kTol);
    assert(std::abs(data.bodies[0].linear_damping[1] - 200.0) < kTol);
    assert(std::abs(data.bodies[0].linear_damping[2] - 300.0) < kTol);
    assert(std::abs(data.bodies[0].linear_damping[3]) < kTol);

    std::cout << "  PASSED: test_body_linear_damping\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3b: Body quadratic damping
// ─────────────────────────────────────────────────────────────────────────────
static bool test_body_quadratic_damping() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: hull.h5
      linear_damping: [100, 200, 300, 0, 0, 0]
      quadratic_damping: [10, 20, 30, 40, 50, 60]
  waves:
    type: no_wave
)";
    auto path = WriteTempYaml(yaml, "qdamping");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    assert(data.bodies.size() == 1);
    assert(std::abs(data.bodies[0].linear_damping[0] - 100.0) < kTol);
    assert(std::abs(data.bodies[0].quadratic_damping[0] - 10.0) < kTol);
    assert(std::abs(data.bodies[0].quadratic_damping[1] - 20.0) < kTol);
    assert(std::abs(data.bodies[0].quadratic_damping[2] - 30.0) < kTol);
    assert(std::abs(data.bodies[0].quadratic_damping[3] - 40.0) < kTol);
    assert(std::abs(data.bodies[0].quadratic_damping[4] - 50.0) < kTol);
    assert(std::abs(data.bodies[0].quadratic_damping[5] - 60.0) < kTol);

    std::cout << "  PASSED: test_body_quadratic_damping\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: Radiation method = state_space
// ─────────────────────────────────────────────────────────────────────────────
static bool test_radiation_state_space() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: wec.h5
  waves:
    type: regular
    height: 1.0
    period: 6.0
  radiation:
    method: state_space
    state_space:
      max_order: 8
      r2_threshold: 0.99
)";
    auto path = WriteTempYaml(yaml, "ss");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    assert(data.radiation_method == "state_space");
    assert(data.ss_max_order == 8);
    assert(std::abs(data.ss_r2_threshold - 0.99) < kTol);

    std::cout << "  PASSED: test_radiation_state_space\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4b: Radiation method = constant_coefficients (+ frequency_tolerance)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_radiation_constant_coefficients() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: wec.h5
  waves:
    type: regular
    height: 1.0
    period: 6.0
  radiation:
    method: constant_coefficients
    frequency_tolerance: 2.5e-5
)";
    auto path = WriteTempYaml(yaml, "cc");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    if (data.radiation_method != "constant_coefficients") return false;
    if (std::abs(data.radiation_frequency_tolerance - 2.5e-5) > 1e-18) return false;

    // Omitted key keeps the default relative tolerance (1e-4).
    YAMLHydroData defaults;
    if (defaults.radiation_frequency_tolerance != 1e-4) return false;

    std::cout << "  PASSED: test_radiation_constant_coefficients\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5: MoorDyn coupling section
// ─────────────────────────────────────────────────────────────────────────────
static bool test_moordyn_section() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: platform.h5
  waves:
    type: regular
    height: 1.0
    period: 8.0
  moordyn:
    enabled: true
    input_file: lines.txt
    bodies: [body1]
)";
    auto path = WriteTempYaml(yaml, "moordyn");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    assert(data.moordyn_enabled == true);
    assert(Filename(data.moordyn_input_file) == "lines.txt");
    assert(data.moordyn_body_names.size() == 1);
    assert(data.moordyn_body_names[0] == "body1");

    std::cout << "  PASSED: test_moordyn_section\n";
    return true;
}

static bool test_moordyn_visualization_radii() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: platform.h5
  waves:
    type: regular
    height: 1.0
    period: 8.0
  moordyn:
    enabled: true
    input_file: lines.txt
    bodies: [body1]
    visualization_line_radius: 0.02
    visualization_endpoint_radius: 0.07
    visualization_node_marker_radius: 0.03
)";
    auto path = WriteTempYaml(yaml, "moordyn_viz");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    assert(data.moordyn_visualization_line_radius == 0.02);
    assert(data.moordyn_visualization_endpoint_radius == 0.07);
    assert(data.moordyn_visualization_node_marker_radius == 0.03);

    std::cout << "  PASSED: test_moordyn_visualization_radii\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 6: Empty file → error
// ─────────────────────────────────────────────────────────────────────────────
static bool test_empty_file_error() {
    auto path = WriteTempYaml("", "empty");
    bool threw = false;
    try {
        ReadHydroYAML(path);
    } catch (const std::exception&) {
        threw = true;
    }
    CleanupFile(path);

    assert(threw);
    std::cout << "  PASSED: test_empty_file_error\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 7: Missing wave section → no error (defaults to no_wave)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_missing_waves_defaults() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: data.h5
)";
    auto path = WriteTempYaml(yaml, "nowaves");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    assert(data.bodies.size() == 1);

    std::cout << "  PASSED: test_missing_waves_defaults\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 8: Multi-body config
// ─────────────────────────────────────────────────────────────────────────────
static bool test_multi_body() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: float.h5
    - name: body2
      h5_file: spar.h5
  waves:
    type: regular
    height: 1.5
    period: 7.0
)";
    auto path = WriteTempYaml(yaml, "multi");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    assert(data.bodies.size() == 2);
    assert(data.bodies[0].name == "body1");
    assert(data.bodies[1].name == "body2");
    assert(Filename(data.bodies[0].h5_file) == "float.h5");
    assert(Filename(data.bodies[1].h5_file) == "spar.h5");

    std::cout << "  PASSED: test_multi_body\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 9: Hull damping, inline vs block lists
// ─────────────────────────────────────────────────────────────────────────────
static const std::array<double, 6> kLinearB = {1.0e3, 2.5e3, 3.25e4, 4.0e5, 5.0e5, 6.0e2};
static const std::array<double, 6> kQuadB = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0};

static void CheckDamping(const YAMLHydroData& data, const std::string& label) {
    Check(data.bodies.size() == 1, label + ": one body");
    for (size_t i = 0; i < 6; ++i) {
        Check(data.bodies[0].linear_damping[i] == kLinearB[i], label + ": linear_damping[" + std::to_string(i) + "]");
        Check(data.bodies[0].quadratic_damping[i] == kQuadB[i],
              label + ": quadratic_damping[" + std::to_string(i) + "]");
    }
}

static const char* kDampingInlineYaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: hull.h5
      linear_damping: [1.0e3, 2.5e3, 3.25e4, 4.0e5, 5.0e5, 6.0e2]
      quadratic_damping: [10, 20, 30, 40, 50, 60]
  waves:
    type: no_wave
)";

// Block form with comments, blank lines, inline comments, quotes, and items
// at the key's own indentation (all valid YAML).
static const char* kDampingBlockYaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: hull.h5
      linear_damping:   # [surge, sway, heave, roll, pitch, yaw]
        - 1.0e3      # surge

        - 2.5e3
        # heave next
        - 3.25e4
        - 4.0e5
        - "5.0e5"
        - 6.0e2
      quadratic_damping:
      - 10
      - 20
      - 30
      - 40
      - 50
      - 60
  waves:
    type: no_wave
)";

static bool test_damping_inline_and_block_identical() {
    auto inline_path = WriteTempYaml(kDampingInlineYaml, "damp_inline");
    auto inline_data = ReadHydroYAML(inline_path);
    CleanupFile(inline_path);
    CheckDamping(inline_data, "inline");

    auto block_path = WriteTempYaml(kDampingBlockYaml, "damp_block");
    auto block_data = ReadHydroYAML(block_path);
    CleanupFile(block_path);
    CheckDamping(block_data, "block");

    Check(inline_data.bodies[0].linear_damping == block_data.bodies[0].linear_damping, "linear identical");
    Check(inline_data.bodies[0].quadratic_damping == block_data.bodies[0].quadratic_damping, "quadratic identical");
    Check(block_data.waves.type == "no_wave", "keys after the block list still parsed");

    std::cout << "  PASSED: test_damping_inline_and_block_identical\n";
    return true;
}

static std::string ToCrlf(const std::string& lf) {
    std::string out;
    for (char c : lf) {
        if (c == '\n') out += '\r';
        out += c;
    }
    return out;
}

static bool test_damping_crlf() {
    auto inline_path = WriteTempYaml(ToCrlf(kDampingInlineYaml), "damp_inline_crlf");
    auto inline_data = ReadHydroYAML(inline_path);
    CleanupFile(inline_path);
    CheckDamping(inline_data, "inline CRLF");

    auto block_path = WriteTempYaml(ToCrlf(kDampingBlockYaml), "damp_block_crlf");
    auto block_data = ReadHydroYAML(block_path);
    CleanupFile(block_path);
    CheckDamping(block_data, "block CRLF");

    std::cout << "  PASSED: test_damping_crlf\n";
    return true;
}

static bool test_damping_block_multi_body() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: float
      h5_file: rm3.h5
      linear_damping:
        - 0
        - 0
        - 1.5e5
        - 0
        - 0
        - 0
    - name: spar
      h5_file: rm3.h5
  waves:
    type: no_wave
)";
    auto path = WriteTempYaml(yaml, "damp_multi");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    Check(data.bodies.size() == 2, "two bodies");
    Check(data.bodies[0].linear_damping[2] == 1.5e5, "float heave damping");
    for (double c : data.bodies[1].linear_damping) {
        Check(c == 0.0, "spar has no damping");
    }
    std::cout << "  PASSED: test_damping_block_multi_body\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 10: Malformed damping lists throw with file, body, and line
// ─────────────────────────────────────────────────────────────────────────────
static bool test_damping_errors() {
    // Block list with 5 values (key on line 6).
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: hull.h5
      linear_damping:
        - 1
        - 2
        - 3
        - 4
        - 5
  waves:
    type: no_wave
)", "err_block_count", {"test_yaml_err_block_count.yaml:6:", "body 'body1': linear_damping", "has 5 values; expected 6",
                       "[surge, sway, heave, roll, pitch, yaw]"});

    // Inline list with 7 values.
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      linear_damping: [1, 2, 3, 4, 5, 6, 7]
)", "err_inline_count", {"test_yaml_err_inline_count.yaml:5:", "body 'body1': linear_damping", "has 7 values"});

    // Non-numeric block item (line 10).
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      h5_file: hull.h5
      linear_damping:
        - 1
        - 2
        - 3
        - 4e5x
        - 5
        - 6
)", "err_block_nan", {"test_yaml_err_block_nan.yaml:10:", "body 'body1': linear_damping entry 4", "\"4e5x\"",
                     "not a finite number"});

    // Non-numeric inline item.
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      quadratic_damping: [1, 2, abc, 4, 5, 6]
)", "err_inline_nan", {"test_yaml_err_inline_nan.yaml:5:", "body 'body1': quadratic_damping entry 3", "\"abc\""});

    // Empty inline entry.
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      linear_damping: [1, 2, , 4, 5, 6]
)", "err_inline_empty_entry", {":5:", "linear_damping entry 3 is empty"});

    // Empty key, no items, followed by another body key.
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      linear_damping:
      h5_file: hull.h5
)", "err_empty_key", {"test_yaml_err_empty_key.yaml:5:", "body 'body1': linear_damping has no value"});

    // Empty key at end of file.
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      quadratic_damping:
)", "err_empty_key_eof", {":5:", "body 'body1': quadratic_damping has no value"});

    // Unterminated inline list (multi-line flow lists are not supported).
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      linear_damping: [1, 2, 3,
                       4, 5, 6]
)", "err_unterminated", {"test_yaml_err_unterminated.yaml:5:", "body 'body1': linear_damping",
                        "unterminated inline list"});

    // Scalar instead of a list.
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      linear_damping: 1000
)", "err_scalar", {":5:", "must be an inline list"});

    // Quadratic block form with a wrong count, second body.
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
    - name: body2
      quadratic_damping:
        - 1
        - 2
        - 3
        - 4
        - 5
        - 6
        - 7
)", "err_quad_block_count", {":6:", "body 'body2': quadratic_damping has 7 values"});

    std::cout << "  PASSED: test_damping_errors\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 11: Other block lists and list guards
// ─────────────────────────────────────────────────────────────────────────────
static bool test_moordyn_bodies_block() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
    - name: body2
  moordyn:
    enabled: true
    input_file: lines.txt
    bodies:
      - body1
      - body2   # spar
    visualization_line_radius: 0.02
)";
    auto path = WriteTempYaml(yaml, "moordyn_block");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);

    Check(data.moordyn_body_names.size() == 2, "two moordyn bodies");
    Check(data.moordyn_body_names[0] == "body1" && data.moordyn_body_names[1] == "body2", "moordyn body names");
    Check(data.moordyn_visualization_line_radius == 0.02, "key after the block list still parsed");
    std::cout << "  PASSED: test_moordyn_bodies_block\n";
    return true;
}

static bool test_period_list_forms() {
    std::string block_yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
  waves:
    type: regular
    height: 1.0
    period:
      values:
        - 6.0
        - 7.5
        - 9.0
    direction: 30.0
)";
    auto path = WriteTempYaml(block_yaml, "period_block");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);
    Check(data.waves.period_values == std::vector<double>({6.0, 7.5, 9.0}), "period block values");
    Check(data.waves.period == 6.0, "period = first value");
    Check(data.waves.direction == 30.0, "waves key right after the period block is parsed");

    std::string inline_yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
  waves:
    type: regular
    height: 1.0
    period: [6.0, 7.5, 9.0]
)";
    path = WriteTempYaml(inline_yaml, "period_inline");
    data = ReadHydroYAML(path);
    CleanupFile(path);
    Check(data.waves.period_values == std::vector<double>({6.0, 7.5, 9.0}), "period inline values");

    std::string linspace_yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
  waves:
    type: regular
    height: 1.0
    period:
      linspace: { start: 6.0, stop: 9.0, num: 4 }
)";
    path = WriteTempYaml(linspace_yaml, "period_linspace");
    data = ReadHydroYAML(path);
    CleanupFile(path);
    Check(data.waves.period_values.size() == 4, "period linspace count");
    Check(std::abs(data.waves.period_values[1] - 7.0) < kTol && std::abs(data.waves.period_values[3] - 9.0) < kTol,
          "period linspace values");

    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
  waves:
    type: regular
    height: 1.0
    period:
      values: [6.0, seven, 9.0]
)", "period_bad_value", {":9:", "waves.period.values entry 2", "\"seven\""});

    std::cout << "  PASSED: test_period_list_forms\n";
    return true;
}

static bool test_unsupported_list_items_error() {
    // Block list under a key that does not accept one.
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      added_mass_scale:
        - 1.0
)", "err_unknown_list", {"test_yaml_err_unknown_list.yaml:6:", "unexpected YAML list item", "under 'added_mass_scale'"});

    // Body item that does not start with "- name".
    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - h5_file: hull.h5
      name: body1
)", "err_body_without_name", {":4:", "must start with '- name: <name>'"});

    std::cout << "  PASSED: test_unsupported_list_items_error\n";
    return true;
}

static bool test_hydrostatics_inline_and_model_check() {
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: body1
      hydrostatics: { model: nonlinear, mesh_file: hull.obj }
)";
    auto path = WriteTempYaml(yaml, "hs_inline");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);
    Check(data.bodies[0].hydrostatics.model == HydrostaticsModel::kNonlinear, "inline hydrostatics model");
    Check(Filename(data.bodies[0].hydrostatics.mesh_file) == "hull.obj", "inline hydrostatics mesh_file");

    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: body1
      hydrostatics:
        model: nonlinar
        mesh_file: hull.obj
)", "err_hs_model", {":6:", "hydrostatics.model must be 'linear' or 'nonlinear'", "\"nonlinar\""});

    std::cout << "  PASSED: test_hydrostatics_inline_and_model_check\n";
    return true;
}

// U10 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md): flat body keys for nonlinear
// hydrostatics and nonlinear Froude-Krylov, and their error cases.
static bool test_froude_krylov_keys() {
    // Canonical flat form: body-level mesh_file is copied to hydrostatics.mesh_file.
    std::string yaml = R"(
hydrodynamics:
  bodies:
    - name: float
      mesh_file: hull.obj
      hydrostatics: nonlinear
      froude_krylov: nonlinear
      froude_krylov_pressure: hydrostatic
    - name: spar
      mesh_file: spar.obj
      hydrostatics: linear
)";
    auto path = WriteTempYaml(yaml, "fk_flat");
    auto data = ReadHydroYAML(path);
    CleanupFile(path);
    Check(data.bodies[0].hydrostatics.model == HydrostaticsModel::kNonlinear, "flat hydrostatics: nonlinear");
    Check(Filename(data.bodies[0].hydrostatics.mesh_file) == "hull.obj", "body mesh_file used for nonlinear body");
    Check(data.bodies[0].froude_krylov == FroudeKrylovModel::kNonlinear, "froude_krylov: nonlinear");
    Check(data.bodies[0].froude_krylov_pressure == FroudeKrylovPressureModel::kHydrostatic,
          "froude_krylov_pressure: hydrostatic");
    Check(data.bodies[1].hydrostatics.model == HydrostaticsModel::kLinear, "flat hydrostatics: linear");
    Check(data.bodies[1].hydrostatics.mesh_file.empty(), "linear body keeps no hydrostatics mesh");
    Check(data.bodies[1].froude_krylov == FroudeKrylovModel::kLinear, "froude_krylov defaults to linear");
    Check(data.bodies[1].froude_krylov_pressure == FroudeKrylovPressureModel::kAiryWheeler,
          "froude_krylov_pressure defaults to airy_wheeler");

    // Block form with the same mesh in both places is accepted.
    yaml = R"(
hydrodynamics:
  bodies:
    - name: float
      mesh_file: hull.obj
      hydrostatics:
        model: nonlinear
        mesh_file: hull.obj
      froude_krylov: nonlinear
      froude_krylov_pressure: airy_wheeler
)";
    path = WriteTempYaml(yaml, "fk_block_same_mesh");
    data = ReadHydroYAML(path);
    CleanupFile(path);
    Check(Filename(data.bodies[0].hydrostatics.mesh_file) == "hull.obj", "matching mesh in both places accepted");

    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: float
      mesh_file: hull.obj
      hydrostatics:
        model: nonlinear
        mesh_file: other.obj
)", "err_fk_mesh_conflict", {"body 'float'", "differ", "body-level mesh_file"});

    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: float
      froude_krylov: nonlinaer
)", "err_fk_value", {":5:", "froude_krylov must be 'linear' or 'nonlinear'", "\"nonlinaer\""});

    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: float
      froude_krylov_pressure: wheeler
)", "err_fk_pressure_value", {":5:", "froude_krylov_pressure must be 'airy_wheeler' or 'hydrostatic'",
                               "\"wheeler\""});

    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: float
      mesh_file: hull.obj
      hydrostatics: nonlinear
      froude_krylov: nonlinear
      include_excitation: false
)", "err_fk_no_excitation", {"body 'float'", "include_excitation: true"});

    ExpectParseError(R"(
hydrodynamics:
  bodies:
    - name: float
      hydrostatics: nonlinar
)", "err_hs_scalar_value", {":5:", "hydrostatics.model must be 'linear' or 'nonlinear'"});

    std::cout << "  PASSED: test_froude_krylov_keys\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 12: Every shipped hydro YAML still parses
// ─────────────────────────────────────────────────────────────────────────────
static bool test_parse_all_shipped_yamls() {
#ifdef SEASTACK_TEST_DATA_DIR
    const std::filesystem::path data_dir(SEASTACK_TEST_DATA_DIR);
    Check(std::filesystem::is_directory(data_dir), "data dir exists: " + data_dir.string());
    int count = 0;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(data_dir)) {
        const std::string name = entry.path().filename().string();
        const std::string suffix = ".hydro.yaml";
        if (!entry.is_regular_file() || name.size() < suffix.size() ||
            name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {
            continue;
        }
        try {
            ReadHydroYAML(entry.path().string());
        } catch (const std::exception& e) {
            throw std::runtime_error("failed to parse " + entry.path().string() + ": " + e.what());
        }
        ++count;
    }
    Check(count > 0, "found shipped hydro YAMLs");
    std::cout << "  PASSED: test_parse_all_shipped_yamls (" << count << " files)\n";
#else
    std::cout << "  SKIPPED: test_parse_all_shipped_yamls (SEASTACK_TEST_DATA_DIR not defined)\n";
#endif
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "test_yaml_parser:\n";
    int passed = 0;
    int failed = 0;

    auto run = [&](bool (*fn)(), const char* name) {
        try {
            if (fn()) { ++passed; } else { ++failed; std::cerr << "  FAILED: " << name << "\n"; }
        } catch (const std::exception& e) {
            ++failed;
            std::cerr << "  FAILED: " << name << " (exception: " << e.what() << ")\n";
        }
    };

    run(test_minimal_regular,       "test_minimal_regular");
    run(test_irregular_jonswap,     "test_irregular_jonswap");
    run(test_body_linear_damping,    "test_body_linear_damping");
    run(test_body_quadratic_damping, "test_body_quadratic_damping");
    run(test_radiation_state_space,  "test_radiation_state_space");
    run(test_radiation_constant_coefficients, "test_radiation_constant_coefficients");
    run(test_moordyn_section,       "test_moordyn_section");
    run(test_moordyn_visualization_radii, "test_moordyn_visualization_radii");
    run(test_empty_file_error,      "test_empty_file_error");
    run(test_missing_waves_defaults,"test_missing_waves_defaults");
    run(test_multi_body,            "test_multi_body");
    run(test_damping_inline_and_block_identical, "test_damping_inline_and_block_identical");
    run(test_damping_crlf,          "test_damping_crlf");
    run(test_damping_block_multi_body, "test_damping_block_multi_body");
    run(test_damping_errors,        "test_damping_errors");
    run(test_moordyn_bodies_block,  "test_moordyn_bodies_block");
    run(test_period_list_forms,     "test_period_list_forms");
    run(test_unsupported_list_items_error, "test_unsupported_list_items_error");
    run(test_hydrostatics_inline_and_model_check, "test_hydrostatics_inline_and_model_check");
    run(test_froude_krylov_keys,    "test_froude_krylov_keys");
    run(test_parse_all_shipped_yamls, "test_parse_all_shipped_yamls");

    std::cout << "\nResults: " << passed << " passed, " << failed << " failed.\n";
    return (failed > 0) ? 1 : 0;
}
