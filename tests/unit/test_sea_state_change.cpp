/*********************************************************************
 * @file  test_sea_state_change.cpp
 * @brief Unit tests for live wave period and direction changes: the
 *        guardrails (CheckSeaStateChange), the crossfade in
 *        LinearDirectionalWaveField, and frequency-domain excitation on the
 *        crossfaded sea.
 *
 * Pass criteria:
 *   S1  CheckSeaStateChange allows covered changes and refuses, with a
 *       reason: constant-coefficient radiation + period change; energy
 *       outside the BEM frequencies or headings above the limit; IRF with one
 *       kernel for several headings + direction change; a different
 *       discretisation. A full-circle heading set covers the wrap.
 *   S2  Crossfade: unchanged (to round-off) before it; (1 - s) old + s new during it
 *       (half-cosine s); exactly the new sea after it, with the phase of the
 *       old one at the origin at the start. A direction change leaves eta at
 *       the origin unchanged. Elevation, pressure head, velocity and the
 *       visualization path agree. A second change during a crossfade and a
 *       different component count are rejected.
 *   S3  ExcitationComponent on the crossfaded wave: bit-identical before any
 *       change; after it, the force of an excitation built on the new sea;
 *       during it, the weighted sum.
 *
 * Self-contained: no data files, no Chrono.
 *********************************************************************/

#include <seastack/hydro/force_components/excitation_component.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/sea_state_change.h>

#include "test_macros.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using seastack::hydro::BodyForces;
using seastack::hydro::CheckSeaStateChange;
using seastack::hydro::ExcitationComponent;
using seastack::hydro::ExcitationTFData;
using seastack::hydro::LinearDirectionalWaveField;
using seastack::hydro::SeaStateChangeLimits;
using seastack::hydro::SystemState;
using seastack::hydro::WaveComponent;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.81;

WaveComponent Component(double amplitude, double omega, double direction, double phase) {
    WaveComponent c;
    c.amplitude = amplitude;
    c.omega = omega;
    c.k = omega * omega / kG;  // deep water
    c.direction = direction;
    c.phase = phase;
    return c;
}

std::vector<WaveComponent> Sea(double omega_scale, double direction) {
    return {Component(0.5, 0.6 * omega_scale, direction, 0.2), Component(0.3, 0.8 * omega_scale, direction, 1.3),
            Component(0.1, 1.1 * omega_scale, direction, 4.0)};
}

double HalfCosine(double t, double t0, double duration) {
    if (t <= t0) return 0.0;
    if (t >= t0 + duration) return 1.0;
    return 0.5 * (1.0 - std::cos(kPi * (t - t0) / duration));
}

/// Synthetic excitation TF of one body: smooth in frequency and direction.
ExcitationTFData Transfer(const std::vector<WaveComponent>& comps) {
    ExcitationTFData tf;
    tf.re.assign(1, Eigen::MatrixXd(6, static_cast<Eigen::Index>(comps.size())));
    tf.im.assign(1, Eigen::MatrixXd(6, static_cast<Eigen::Index>(comps.size())));
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(comps.size()); ++j) {
        const auto& c = comps[static_cast<std::size_t>(j)];
        for (int i = 0; i < 6; ++i) {
            tf.re[0](i, j) = 1.0e5 * (1.0 + i) * c.omega * std::cos(c.direction + 0.1 * i);
            tf.im[0](i, j) = -2.0e4 * (2.0 - i) * c.omega * c.omega * std::sin(c.direction + 0.3);
        }
    }
    return tf;
}

std::unique_ptr<ExcitationComponent> MakeExcitation(const std::vector<WaveComponent>& comps,
                                                    std::shared_ptr<const LinearDirectionalWaveField> wave) {
    ExcitationComponent::WaveComponentData data;
    const Eigen::Index n = static_cast<Eigen::Index>(comps.size());
    data.amplitudes.resize(n);
    data.omegas.resize(n);
    data.phases.resize(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        data.amplitudes[i] = comps[static_cast<std::size_t>(i)].amplitude;
        data.omegas[i] = comps[static_cast<std::size_t>(i)].omega;
        data.phases[i] = comps[static_cast<std::size_t>(i)].phase;
    }
    ExcitationTFData tf = Transfer(comps);
    auto envelope = wave ? wave->GetAmplitudeEnvelope() : nullptr;
    return std::make_unique<ExcitationComponent>(std::move(data), std::move(tf.re), std::move(tf.im), 1, 0.0,
                                                 envelope, wave, wave ? Transfer : nullptr);
}

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

double MaxRelDiff(const std::vector<double>& a, const std::vector<double>& b) {
    double scale = 0.0, diff = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        scale = std::max(scale, std::abs(b[i]));
        diff = std::max(diff, std::abs(a[i] - b[i]));
    }
    return diff / scale;
}

bool Contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

}  // namespace

int main() {
    TestResults test_results;

    // ---- S1. Guardrails ----
    {
        SeaStateChangeLimits limits;
        limits.bem_frequencies = Eigen::VectorXd::LinSpaced(50, 0.2, 2.0);
        limits.bem_headings = (Eigen::VectorXd(4) << 0.0, kPi / 6, kPi / 3, kPi / 2).finished();
        const auto current = Sea(1.0, 0.0);

        TEST_ASSERT(CheckSeaStateChange(current, current, limits).empty(), "no change: allowed");
        TEST_ASSERT(CheckSeaStateChange(current, Sea(1.3, 0.0), limits).empty(), "period change inside the band");
        TEST_ASSERT(CheckSeaStateChange(current, Sea(1.0, kPi / 4), limits).empty(), "direction between headings");

        const std::string band = CheckSeaStateChange(current, Sea(3.0, 0.0), limits);
        TEST_ASSERT(Contains(band, "outside the BEM frequency range"), "period change out of band refused: " + band);
        // 0.1^2 / (0.5^2 + 0.3^2 + 0.1^2) = 2.9% of the energy above the band.
        auto tail = Sea(1.0, 0.0);
        tail[2].omega = 2.5;
        TEST_ASSERT(!CheckSeaStateChange(current, tail, limits).empty(), "2.9% of the energy outside: refused");
        limits.max_energy_outside = 0.05;
        TEST_ASSERT(CheckSeaStateChange(current, tail, limits).empty(), "... allowed with a 5% limit");
        limits.max_energy_outside = 0.01;

        const std::string heading = CheckSeaStateChange(current, Sea(1.0, 2.0 * kPi / 3), limits);
        TEST_ASSERT(Contains(heading, "outside the BEM headings 0 to 90 deg"), "direction outside refused: " + heading);
        TEST_ASSERT(!CheckSeaStateChange(current, Sea(1.0, -kPi / 6), limits).empty(),
                    "direction across the open wrap (330 deg) refused");

        SeaStateChangeLimits full = limits;
        full.bem_headings = Eigen::VectorXd::LinSpaced(12, 0.0, 11.0 * kPi / 6);  // 0..330 deg
        TEST_ASSERT(CheckSeaStateChange(current, Sea(1.0, -kPi / 12), full).empty(),
                    "full circle: 345 deg is covered across the wrap");

        SeaStateChangeLimits one = limits;
        one.bem_headings = Eigen::VectorXd::Zero(1);
        const std::string single = CheckSeaStateChange(current, Sea(1.0, kPi / 6), one);
        TEST_ASSERT(Contains(single, "one BEM heading (0 deg)"), "one heading: direction change refused: " + single);
        TEST_ASSERT(CheckSeaStateChange(current, Sea(1.2, 0.0), one).empty(), "one heading: period change allowed");
        TEST_ASSERT(CheckSeaStateChange(current, Sea(1.0, 2.0 * kPi), one).empty(), "direction + 360 deg: no change");

        SeaStateChangeLimits cc = limits;
        cc.constant_coefficient_radiation = true;
        TEST_ASSERT(Contains(CheckSeaStateChange(current, Sea(1.2, 0.0), cc), "constant-coefficient"),
                    "constant-coefficient radiation: period change refused");
        TEST_ASSERT(CheckSeaStateChange(current, Sea(1.0, kPi / 6), cc).empty(),
                    "constant-coefficient radiation: direction change allowed");

        SeaStateChangeLimits irf = limits;
        irf.heading_kernels = false;
        TEST_ASSERT(Contains(CheckSeaStateChange(current, Sea(1.0, kPi / 6), irf), "one excitation kernel"),
                    "IRF with one kernel: direction change refused");
        TEST_ASSERT(CheckSeaStateChange(current, Sea(1.2, 0.0), irf).empty(), "IRF with one kernel: period allowed");

        auto fewer = current;
        fewer.pop_back();
        TEST_ASSERT(Contains(CheckSeaStateChange(current, fewer, limits), "different number"),
                    "different discretisation refused");
    }

    // ---- S2. Crossfade in the wave field ----
    {
        constexpr double t0 = 10.0, duration = 16.0;
        const Eigen::Vector3d origin(0.0, 0.0, 0.0);
        const Eigen::Vector3d point(30.0, 20.0, -2.0);
        const Eigen::Vector3d surface(30.0, 20.0, 0.0);

        for (const bool turn : {true, false}) {
            const std::string what = turn ? "direction: " : "period: ";
            LinearDirectionalWaveField old_sea(Sea(1.0, 0.0), 0.0);
            auto field = std::make_shared<LinearDirectionalWaveField>(Sea(1.0, 0.0), 0.0);
            const double before = field->GetElevation(point, 4.0);
            const double end = field->CrossfadeTo(turn ? Sea(1.0, kPi / 6) : Sea(1.25, 0.0), t0, duration);
            TEST_ASSERT(end == t0 + duration, what + "returns the end of the crossfade");
            // Not bitwise: the sum now runs over both sets (new ones at weight 0).
            TEST_NEAR(field->GetElevation(point, 4.0), before, 1e-14, what + "unchanged before the change");

            // The new sea as its own field, with the phases CrossfadeTo chose.
            const auto all = field->GetComponents();
            const std::vector<WaveComponent> new_comps(all.begin() + 3, all.end());
            LinearDirectionalWaveField new_sea(new_comps, 0.0);
            bool phase_kept = true;
            for (std::size_t i = 0; i < 3; ++i) {
                const double old_theta = all[i].phase - all[i].omega * t0;
                const double new_theta = new_comps[i].phase - new_comps[i].omega * t0;
                phase_kept = phase_kept && std::abs(std::remainder(new_theta - old_theta, 2.0 * kPi)) < 1e-12;
            }
            TEST_ASSERT(phase_kept, what + "each new component has the old phase at the origin at t0");

            double worst = 0.0;
            for (double t : {t0, 12.0, 18.0, 25.9, t0 + duration, 40.0, 77.7}) {
                const double s = HalfCosine(t, t0, duration);
                for (const auto& p : {origin, point}) {
                    const double expected = (1.0 - s) * old_sea.GetElevation(p, t) + s * new_sea.GetElevation(p, t);
                    worst = std::max(worst, std::abs(field->GetElevation(p, t) - expected));
                }
            }
            TEST_ASSERT(worst < 1e-12, what + "eta = (1 - s) old + s new (worst " + std::to_string(worst) + " m)");
            TEST_ASSERT(std::abs(field->GetElevation(point, 50.0) - new_sea.GetElevation(point, 50.0)) < 1e-12,
                        what + "after the crossfade: the new sea");
            if (turn) {
                double drift = 0.0;
                for (double t : {11.0, 18.0, 33.0, 60.0}) {
                    drift = std::max(drift, std::abs(field->GetElevation(origin, t) - old_sea.GetElevation(origin, t)));
                }
                TEST_ASSERT(drift < 1e-12, "direction: eta at the origin is unchanged");
            }

            const double t = 18.0;
            const double s = HalfCosine(t, t0, duration);
            const Eigen::Vector3d v = field->GetVelocity(point, t, 0.0);
            const Eigen::Vector3d v_expected =
                (1.0 - s) * old_sea.GetVelocity(point, t, 0.0) + s * new_sea.GetVelocity(point, t, 0.0);
            TEST_ASSERT((v - v_expected).norm() < 1e-12, what + "velocity is the weighted sum");
            const double head = field->GetDynamicPressureHead(point, t, 0.0);
            const double head_expected = (1.0 - s) * old_sea.GetDynamicPressureHead(point, t, 0.0) +
                                         s * new_sea.GetDynamicPressureHead(point, t, 0.0);
            TEST_NEAR(head, head_expected, 1e-12, what + "pressure head is the weighted sum");
            Eigen::Matrix3Xd points(3, 2);
            points.col(0) = point;
            points.col(1) = surface;
            Eigen::VectorXd eta, heads;
            field->GetElevationAndPressureHead(points, t, 0.7, eta, &heads);
            TEST_NEAR(eta[1], 0.7 * field->GetElevation(surface, t), 1e-12, what + "batched elevation agrees");
            TEST_NEAR(heads[0], field->GetDynamicPressureHead(point, t, eta[0]), 1e-12,
                      what + "batched pressure head agrees");
            TEST_NEAR(field->GetElevation(point, t, 0), field->GetElevation(point, t), 1e-12,
                      what + "visualization path (no limit) agrees");

            TEST_ASSERT(field->IsCrossfading(20.0) && !field->IsCrossfading(t0 + duration), what + "IsCrossfading");
            bool busy = false;
            try {
                field->CrossfadeTo(Sea(1.0, 0.0), 20.0, duration);
            } catch (const std::logic_error&) {
                busy = true;
            }
            TEST_ASSERT(busy, what + "a second change during the crossfade is rejected");
            bool mismatch = false;
            try {
                field->CrossfadeTo({Component(1.0, 0.8, 0.0, 0.0)}, 30.0, duration);
            } catch (const std::invalid_argument&) {
                mismatch = true;
            }
            TEST_ASSERT(mismatch, what + "a different component count is rejected");
            if (!turn) {
                TEST_NEAR(field->GetCharacteristicPeriod(), 2.0 * kPi / (0.6 * 1.25), 1e-12,
                          "period: characteristic period of the new sea");
            }

            // A second change after the first one has finished fades out the newest set.
            field->CrossfadeTo(Sea(1.0, 0.0), 40.0, duration);
            TEST_NEAR(field->GetElevation(point, 39.0), new_sea.GetElevation(point, 39.0), 1e-12,
                      what + "second change: unchanged before it");
            TEST_ASSERT(field->GetWaveLayers().size() == 3, what + "three layers after two changes");
        }
    }

    // ---- S3. Frequency-domain excitation on the crossfaded sea ----
    {
        constexpr double t0 = 5.0, duration = 12.0;
        auto field = std::make_shared<LinearDirectionalWaveField>(Sea(1.0, 0.0), 0.0);
        auto plain = MakeExcitation(Sea(1.0, 0.0), nullptr);
        auto live = MakeExcitation(Sea(1.0, 0.0), field);
        bool identical = true;
        for (double t : {0.0, 1.7, 4.9}) {
            const auto a = Force(*plain, t);
            const auto b = Force(*live, t);
            identical = identical && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
        }
        TEST_ASSERT(identical, "no change: bit-identical to an excitation without the wave");

        field->CrossfadeTo(Sea(1.2, kPi / 6), t0, duration);
        const auto all = field->GetComponents();
        auto old_only = MakeExcitation(std::vector<WaveComponent>(all.begin(), all.begin() + 3), nullptr);
        auto new_only = MakeExcitation(std::vector<WaveComponent>(all.begin() + 3, all.end()), nullptr);
        double worst_after = 0.0, worst_during = 0.0;
        for (double t : {20.0, 33.3, 61.0}) {
            worst_after = std::max(worst_after, MaxRelDiff(Force(*live, t), Force(*new_only, t)));
        }
        for (double t : {5.0, 7.5, 11.0, 16.9}) {
            const double s = HalfCosine(t, t0, duration);
            const auto f_old = Force(*old_only, t);
            const auto f_new = Force(*new_only, t);
            std::vector<double> expected(6);
            for (int i = 0; i < 6; ++i) expected[i] = (1.0 - s) * f_old[i] + s * f_new[i];
            worst_during = std::max(worst_during, MaxRelDiff(Force(*live, t), expected));
        }
        TEST_ASSERT(worst_after < 1e-13, "after the crossfade: the force of the new sea (rel. " +
                                             std::to_string(worst_after) + ")");
        TEST_ASSERT(worst_during < 1e-13, "during the crossfade: the weighted sum (rel. " +
                                              std::to_string(worst_during) + ")");

        // Without a transfer function the new components cannot be handled.
        ExcitationComponent::WaveComponentData data;
        data.amplitudes = Eigen::Vector3d(0.5, 0.3, 0.1);
        data.omegas = Eigen::Vector3d(0.6, 0.8, 1.1);
        data.phases = Eigen::Vector3d(0.2, 1.3, 4.0);
        ExcitationTFData tf = Transfer(Sea(1.0, 0.0));
        ExcitationComponent no_transfer(std::move(data), tf.re, tf.im, 1, 0.0, nullptr, field, nullptr);
        bool threw = false;
        try {
            Force(no_transfer, 20.0);
        } catch (const std::logic_error&) {
            threw = true;
        }
        TEST_ASSERT(threw, "changed wave without a transfer function: error");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
