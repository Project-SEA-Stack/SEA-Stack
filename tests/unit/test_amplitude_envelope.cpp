/*********************************************************************
 * @file  test_amplitude_envelope.cpp
 * @brief Unit tests for the runtime wave amplitude envelope e(t) (live
 *        wave-height changes) and its use in frequency-domain excitation.
 *
 * Pass criteria:
 *   E1  e = 1 before any change; half-cosine ramp between the values;
 *       history is kept for past times (IRF convolution needs it).
 *   E2  invalid changes are rejected.
 *   E3  ExcitationComponent with an unchanged envelope is bit-identical to
 *       one without; with e constant, the force is e times the reference.
 *   E4  the visualization ramp includes e(t).
 *
 * Self-contained: no data files, no Chrono.
 *********************************************************************/

#include <seastack/hydro/force_components/excitation_component.h>
#include <seastack/hydro/waves/amplitude_envelope.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include "test_macros.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>

using seastack::hydro::AmplitudeEnvelope;
using seastack::hydro::BodyForces;
using seastack::hydro::ExcitationComponent;
using seastack::hydro::SystemState;
using seastack::hydro::LinearDirectionalWaveField;
using seastack::hydro::WaveComponent;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Excitation on one body from three components; fixed TF values (no BEM data).
std::unique_ptr<ExcitationComponent> MakeExcitation(double ramp_duration,
                                                    std::shared_ptr<const AmplitudeEnvelope> envelope) {
    ExcitationComponent::WaveComponentData wave;
    wave.amplitudes = Eigen::Vector3d(0.5, 0.3, 0.1);
    wave.omegas = Eigen::Vector3d(0.6, 0.8, 1.1);
    wave.phases = Eigen::Vector3d(0.2, 1.3, -0.7);
    Eigen::MatrixXd re(6, 3), im(6, 3);
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
            re(i, j) = 1.0e5 * (1.0 + i) * (1.0 + 0.3 * j);
            im(i, j) = -2.0e4 * (2.0 - i) * (1.0 - 0.2 * j);
        }
    }
    return std::make_unique<ExcitationComponent>(std::move(wave), std::vector<Eigen::MatrixXd>{re},
                                                 std::vector<Eigen::MatrixXd>{im}, 1, ramp_duration,
                                                 std::move(envelope));
}

/// Six force/moment components at time t.
std::vector<double> Force(ExcitationComponent& excitation, double t) {
    SystemState state;
    state.bodies.resize(1);
    BodyForces forces(1);
    forces[0].force.setZero();
    forces[0].moment.setZero();
    excitation.Compute(state, t, forces);
    return {forces[0].force.x(),  forces[0].force.y(),  forces[0].force.z(),
            forces[0].moment.x(), forces[0].moment.y(), forces[0].moment.z()};
}

bool BitEqual(const std::vector<double>& a, const std::vector<double>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

}  // namespace

int main() {
    TestResults test_results;

    // ---- E1. Values and history ----
    {
        AmplitudeEnvelope e;
        TEST_ASSERT(!e.HasChanges() && e.Value(-5.0) == 1.0 && e.Value(100.0) == 1.0, "e = 1 with no change");
        const double done = e.RampTo(10.0, 0.5, 4.0);
        TEST_ASSERT(done == 14.0, "RampTo returns the end of the transition");
        TEST_ASSERT(e.Value(9.99) == 1.0, "before the change: 1");
        TEST_ASSERT(e.Value(10.0) == 1.0, "at the start of the ramp: previous value");
        TEST_NEAR(e.Value(12.0), 0.75, 1e-15, "half-way: mean of the two values (half cosine)");
        TEST_NEAR(e.Value(11.0), 1.0 - 0.5 * 0.5 * (1.0 - std::cos(kPi * 0.25)), 1e-15, "quarter-way: cosine shape");
        TEST_ASSERT(e.Value(14.0) == 0.5 && e.Value(1e6) == 0.5, "after the ramp: target");
        TEST_ASSERT(e.Target() == 0.5, "target");

        // A change during a ramp starts from the value at that time.
        e.RampTo(12.0, 2.0, 2.0);
        TEST_NEAR(e.Value(12.0), 0.75, 1e-15, "second ramp starts at the current value");
        TEST_ASSERT(e.Value(14.0) == 2.0, "second ramp reaches its target");
        TEST_NEAR(e.Value(11.0), 1.0 - 0.5 * 0.5 * (1.0 - std::cos(kPi * 0.25)), 1e-15,
                  "earlier history unchanged (IRF looks back)");

        // Step change (duration 0), then a change at the same time replaces it.
        e.RampTo(20.0, 0.0, 0.0);
        TEST_ASSERT(e.Value(20.0) == 0.0 && e.Value(19.999) == 2.0, "step to calm");
        e.RampTo(20.0, 3.0, 0.0);
        TEST_ASSERT(e.Value(20.0) == 3.0 && e.Value(19.999) == 2.0, "same-time change replaces the previous one");
    }

    // ---- E2. Rejected changes ----
    {
        AmplitudeEnvelope e;
        auto throws = [&](double t, double v, double d) {
            try {
                e.RampTo(t, v, d);
            } catch (const std::invalid_argument&) {
                return true;
            }
            return false;
        };
        TEST_ASSERT(throws(0.0, -0.1, 1.0), "negative target rejected");
        TEST_ASSERT(throws(0.0, 1.0, -1.0), "negative duration rejected");
        TEST_ASSERT(throws(0.0, std::nan(""), 1.0), "NaN target rejected");
        TEST_ASSERT(!e.HasChanges(), "rejected changes leave e unchanged");
        e.RampTo(5.0, 0.5, 1.0);
        TEST_ASSERT(throws(4.0, 1.0, 1.0), "change before the last one rejected");
    }

    // ---- E3. Frequency-domain excitation ----
    {
        constexpr double kRamp = 5.0;
        auto reference = MakeExcitation(kRamp, nullptr);
        auto envelope = std::make_shared<AmplitudeEnvelope>();
        auto with_envelope = MakeExcitation(kRamp, envelope);
        bool identical = true;
        for (double t : {0.0, 0.37, 2.5, 4.99, 5.0, 7.3, 31.1}) {
            identical = identical && BitEqual(Force(*reference, t), Force(*with_envelope, t));
        }
        TEST_ASSERT(identical, "unchanged envelope: bit-identical to no envelope (ramp and after)");

        // Same value at step 0: e stays exactly 1, so still bit-identical.
        envelope->RampTo(0.0, 1.0, 16.0);
        identical = true;
        for (double t : {0.0, 0.37, 2.5, 8.0, 31.1}) {
            identical = identical && BitEqual(Force(*reference, t), Force(*with_envelope, t));
        }
        TEST_ASSERT(identical, "change to the same value: bit-identical");

        // Linear scaling: once the ramp is over, F = e F_reference.
        envelope->RampTo(1.0, 0.4, 2.0);
        double worst = 0.0;
        for (double t : {6.0, 9.1, 17.25, 40.0}) {
            const auto f = Force(*with_envelope, t);
            const auto f_ref = Force(*reference, t);
            for (size_t i = 0; i < f.size(); ++i) {
                worst = std::max(worst, std::abs(f[i] - 0.4 * f_ref[i]) / (std::abs(f_ref[i]) + 1.0));
            }
        }
        TEST_ASSERT(worst < 1e-14, "constant e = 0.4: force is 0.4 x reference (worst rel. error " +
                                       std::to_string(worst) + ")");
        // During the transition: F = r(t) e(t) F_unramped.
        const double t = 2.0;
        const double r = 0.5 * (1.0 - std::cos(kPi * t / kRamp));
        const auto f = Force(*with_envelope, t);
        auto unramped = MakeExcitation(0.0, nullptr);
        const auto f_un = Force(*unramped, t);
        TEST_NEAR(f[2], r * envelope->Value(t) * f_un[2], 1e-9 * std::abs(f_un[2]),
                  "during the transition: start-up ramp times e(t)");
    }

    // ---- E4. Visualization ramp includes e(t) ----
    {
        WaveComponent c;
        c.amplitude = 1.0;
        c.omega = 0.8;
        c.k = 0.8 * 0.8 / 9.81;
        c.direction = 0.0;
        c.phase = 0.0;
        LinearDirectionalWaveField wave({c}, 0.0);
        wave.Initialize();
        wave.SetRampDuration(10.0);
        const double before = wave.GetExcitationRampForVisualization(20.0);
        wave.GetAmplitudeEnvelope()->RampTo(20.0, 0.25, 0.0);
        TEST_ASSERT(before == 1.0, "visualization ramp 1 after start-up, no change");
        TEST_ASSERT(wave.GetExcitationRampForVisualization(25.0) == 0.25, "visualization ramp follows e");
        TEST_NEAR(wave.GetExcitationRampForVisualization(5.0), 0.5, 1e-15, "start-up ramp unchanged before the change");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
