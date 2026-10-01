/*********************************************************************
 * @file  test_yaml_damping_regression.cpp
 * @brief Regression: block-style hull damping in hydro YAML reaches the
 *        simulation (it used to be dropped silently, leaving zero damping).
 *
 * TALOS-style setup: one body, linear_damping as a YAML block list,
 * regular waves, constant-coefficient radiation. The IEA sphere
 * (sphere.h5) is restricted to heave and driven at w = 1.0 rad/s, which is
 * a BEM grid point of sphere.h5 (w[19] = 1.000000048889152 rad/s).
 *
 * Checks:
 *   1. Block and inline YAML give the same HydroSystem damping config.
 *   2. Block and inline YAML give identical heave time series.
 *   3. The damped response differs from the undamped one (damping applied).
 *********************************************************************/

#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/adapters/chrono/setup_from_yaml.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChSystemNSC.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace {

// Heave hull damping [N/(m/s)]; for scale, B33(w = 1.0 rad/s) = 88539 N.s/m.
constexpr double kHeaveDamping = 2.0e5;
constexpr double kTimeStep = 0.01;        // [s]
constexpr double kDuration = 60.0;        // [s]
constexpr double kRampDuration = 10.0;    // [s]
constexpr double kSteadyStart = 40.0;     // [s] start of the amplitude window

void Check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error("check failed: " + message);
    }
    std::cout << "  ok: " << message << "\n";
}

std::string HydroYaml(const std::string& damping_lines) {
    return std::string(R"(hydrodynamics:
  bodies:
    - name: body1
      h5_file: )") + SEASTACK_TEST_SPHERE_H5_FILE + "\n" + damping_lines + R"(
  waves:
    type: regular
    height: 1.0
    period: 6.283185307   # w = 1.0 rad/s (BEM grid point of sphere.h5)
    direction: 0
    ramp_duration: 10.0

  radiation:
    method: constant_coefficients
)";
}

const std::string kBlockDamping = R"(      linear_damping:   # [surge, sway, heave, roll, pitch, yaw]
        - 0.0
        - 0.0
        - 2.0e5   # heave
        - 0.0
        - 0.0
        - 0.0
)";
const std::string kInlineDamping = "      linear_damping: [0.0, 0.0, 2.0e5, 0.0, 0.0, 0.0]\n";

struct RunResult {
    std::vector<std::array<double, 6>> linear_damping;  // HydroSystem config
    std::vector<double> heave;                          // body z [m] per step
};

/// Run the heave-only sphere for kDuration with the given hydro YAML.
RunResult Run(const std::string& yaml, const std::string& tag) {
    const auto path = std::filesystem::temp_directory_path() / ("test_yaml_damping_regression_" + tag + ".hydro.yaml");
    {
        std::ofstream f(path, std::ios::binary);
        f << yaml;
    }

    chrono::ChSystemNSC sys;
    sys.SetGravitationalAcceleration(chrono::ChVector3d(0, 0, -9.81));
    // The added-mass load adds a mass matrix block, which PSOR cannot handle.
    sys.SetSolverType(chrono::ChSolver::Type::GMRES);

    auto ground = chrono_types::make_shared<chrono::ChBody>();
    ground->SetName("ground");
    ground->SetFixed(true);
    sys.Add(ground);

    // IEA sphere (r = 5 m), floating at equilibrium with its centre at z = 0.
    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetName("body1");
    body->SetMass(261800.0);
    body->SetInertiaXX(chrono::ChVector3d(999.0, 999.0, 999.0));
    body->SetPos(chrono::ChVector3d(0, 0, 0));
    sys.Add(body);

    // Heave only (prismatic joint along world z).
    auto heave_joint = chrono_types::make_shared<chrono::ChLinkLockPrismatic>();
    heave_joint->Initialize(ground, body, chrono::ChFrame<>(chrono::ChVector3d(0, 0, 0), chrono::QUNIT));
    sys.Add(heave_joint);

    const std::vector<std::shared_ptr<chrono::ChBody>> hydro_bodies = {body};
    auto hydro =
        seastack::chrono::SetupHydroFromYAMLFile(path.string(), hydro_bodies, kTimeStep, kDuration, kRampDuration);

    RunResult result;
    result.linear_damping = hydro->config().linear_damping;
    const int n_steps = static_cast<int>(std::lround(kDuration / kTimeStep));
    result.heave.reserve(n_steps);
    for (int i = 0; i < n_steps; ++i) {
        sys.DoStepDynamics(kTimeStep);
        result.heave.push_back(body->GetPos().z());
    }

    std::error_code ec;
    std::filesystem::remove(path, ec);
    return result;
}

/// Half the peak-to-peak heave over [kSteadyStart, kDuration].
double SteadyAmplitude(const std::vector<double>& heave) {
    const size_t first = static_cast<size_t>(std::lround(kSteadyStart / kTimeStep));
    const auto [lo, hi] = std::minmax_element(heave.begin() + first, heave.end());
    return 0.5 * (*hi - *lo);
}

}  // namespace

int main() {
    std::cout << "test_yaml_damping_regression:\n";
    try {
        const RunResult block = Run(HydroYaml(kBlockDamping), "block");
        const RunResult inline_form = Run(HydroYaml(kInlineDamping), "inline");
        const RunResult none = Run(HydroYaml(""), "none");

        Check(block.linear_damping.size() == 1, "block: damping configured for one body");
        const std::array<double, 6> expected = {0.0, 0.0, kHeaveDamping, 0.0, 0.0, 0.0};
        Check(block.linear_damping[0] == expected, "block: HydroSystem linear damping = [0 0 2e5 0 0 0]");
        Check(inline_form.linear_damping == block.linear_damping, "inline and block damping config identical");
        Check(none.linear_damping.empty(), "no damping key: no damping configured");

        Check(block.heave.size() == inline_form.heave.size(), "same number of steps");
        double max_diff = 0.0;
        for (size_t i = 0; i < block.heave.size(); ++i) {
            max_diff = std::max(max_diff, std::abs(block.heave[i] - inline_form.heave[i]));
        }
        Check(max_diff == 0.0, "inline and block heave time series identical");

        const double amp_damped = SteadyAmplitude(block.heave);
        const double amp_undamped = SteadyAmplitude(none.heave);
        std::cout << "  heave amplitude: damped " << amp_damped << " m, undamped " << amp_undamped << " m\n";
        Check(std::isfinite(amp_damped) && amp_damped > 0.05, "damped run responds to the 0.5 m waves");
        Check(amp_damped < 0.97 * amp_undamped, "hull damping reduces the heave amplitude by more than 3%");
    } catch (const std::exception& e) {
        std::cerr << "  FAILED: " << e.what() << "\n";
        return 1;
    }
    std::cout << "PASSED\n";
    return 0;
}
