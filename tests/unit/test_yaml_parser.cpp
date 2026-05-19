/*********************************************************************
 * @file  test_yaml_parser.cpp
 * @brief Unit tests for the hand-rolled hydro YAML parser.
 *
 * Tests cover:
 *   1. Minimal valid regular-wave config
 *   2. Irregular wave with JONSWAP spectrum
 *   3. Body linear damping
 *   4. Radiation method selection (state_space)
 *   5. MoorDyn coupling section
 *   6. Empty file → error
 *   7. Missing wave section defaults
 *   8. Multi-body config
 *********************************************************************/

#include <seastack/hydro/config/yaml_parser.h>
#include <seastack/hydro/config/hydro_config.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace seastack::hydro;

static constexpr double kTol = 1e-9;

/// Write a string to a temporary file and return the path.
static std::string WriteTempYaml(const std::string& content, const std::string& suffix) {
    std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("test_yaml_" + suffix + ".yaml");
    std::ofstream f(path);
    f << content;
    f.close();
    return path.string();
}

static void CleanupFile(const std::string& path) {
    std::filesystem::remove(path);
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
    run(test_moordyn_section,       "test_moordyn_section");
    run(test_moordyn_visualization_radii, "test_moordyn_visualization_radii");
    run(test_empty_file_error,      "test_empty_file_error");
    run(test_missing_waves_defaults,"test_missing_waves_defaults");
    run(test_multi_body,            "test_multi_body");

    std::cout << "\nResults: " << passed << " passed, " << failed << " failed.\n";
    return (failed > 0) ? 1 : 0;
}
