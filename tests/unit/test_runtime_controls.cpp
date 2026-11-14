// RuntimeControls: parameter registry and ownership, the bit-identical swap of
// Chrono's linear spring-damper functors, step-boundary application, and the
// schedule file round trip.

#include <seastack/adapters/chrono/runtime_controls.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkRSDA.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/utils/ChForceFunctors.h>

#include "test_macros.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace cu = ::chrono::utils;

constexpr double kDt = 1e-3;   // [s]
constexpr int kSteps = 400;

/// A body hanging from the ground on a TSDA, spinning about Z against an
/// RSDA; starts moving so springs, dampers and stops all act.
struct Rig {
    std::unique_ptr<::chrono::ChSystemNSC> system;
    std::shared_ptr<::chrono::ChBody> body;
    std::shared_ptr<::chrono::ChLinkTSDA> tsda;
    std::shared_ptr<::chrono::ChLinkRSDA> rsda;
};

Rig MakeRig(std::shared_ptr<::chrono::ChLinkTSDA::ForceFunctor> force,
            std::shared_ptr<::chrono::ChLinkRSDA::TorqueFunctor> torque) {
    Rig rig;
    rig.system = std::make_unique<::chrono::ChSystemNSC>();
    rig.system->SetGravitationalAcceleration({0.0, 0.0, -9.81});
    auto ground = chrono_types::make_shared<::chrono::ChBody>();
    ground->SetFixed(true);
    rig.system->AddBody(ground);
    rig.body = chrono_types::make_shared<::chrono::ChBody>();
    rig.body->SetMass(10.0);
    rig.body->SetInertiaXX({1.0, 1.0, 1.0});
    rig.body->SetPos({0.0, 0.0, -2.0});
    rig.body->SetPosDt({0.0, 0.0, -0.5});
    rig.body->SetAngVelParent({0.0, 0.0, 2.0});
    rig.system->AddBody(rig.body);

    rig.tsda = chrono_types::make_shared<::chrono::ChLinkTSDA>();
    rig.tsda->SetName("hanger");
    rig.tsda->Initialize(ground, rig.body, false, {0.0, 0.0, 0.0}, {0.0, 0.0, -2.0});
    rig.tsda->SetRestLength(1.8);
    if (force) rig.tsda->RegisterForceFunctor(force);
    rig.system->AddLink(rig.tsda);

    rig.rsda = chrono_types::make_shared<::chrono::ChLinkRSDA>();
    rig.rsda->SetName("twist");
    rig.rsda->Initialize(ground, rig.body, ::chrono::ChFramed({0.0, 0.0, -2.0}, ::chrono::QUNIT));
    rig.rsda->SetRestAngle(0.1);
    if (torque) rig.rsda->RegisterTorqueFunctor(torque);
    rig.system->AddLink(rig.rsda);
    return rig;
}

struct Trace {
    std::vector<double> values;  // z, vz, wz, TSDA force, RSDA torque per step
};

/// Steps the rig; `before_step(step, controls)` may queue requests.
Trace Run(Rig& rig, seastack::chrono::RuntimeControls* controls,
          const std::function<void(int)>& before_step = nullptr) {
    Trace trace;
    for (int i = 0; i < kSteps; ++i) {
        if (before_step) before_step(i);
        if (controls) controls->ApplyDue(i, rig.system->GetChTime());
        rig.system->DoStepDynamics(kDt);
        trace.values.push_back(rig.body->GetPos().z());
        trace.values.push_back(rig.body->GetPosDt().z());
        trace.values.push_back(rig.body->GetAngVelParent().z());
        trace.values.push_back(rig.tsda->GetForce());
        trace.values.push_back(rig.rsda->GetTorque());
    }
    return trace;
}

/// First index where the traces differ (bitwise), or -1.
int FirstDifference(const Trace& a, const Trace& b) {
    for (size_t i = 0; i < a.values.size() && i < b.values.size(); ++i) {
        if (std::memcmp(&a.values[i], &b.values[i], sizeof(double)) != 0) {
            return static_cast<int>(i / 5);
        }
    }
    return a.values.size() == b.values.size() ? -1 : 0;
}

struct FunctorCase {
    const char* name;
    std::function<std::shared_ptr<::chrono::ChLinkTSDA::ForceFunctor>()> force;
    std::function<std::shared_ptr<::chrono::ChLinkRSDA::TorqueFunctor>()> torque;
};

}  // namespace

int main() {
    TestResults test_results;
    using seastack::chrono::RuntimeControls;

    // ---- 1. Registry: ids, values, source and owners ----
    {
        auto rig = MakeRig(std::make_shared<cu::LinearSpringDamperForce>(500.0, 40.0, 3.0),
                           std::make_shared<cu::LinearDamperTorque>(7.0));
        RuntimeControls controls(rig.system.get());
        TEST_ASSERT(controls.parameters().size() == 6, "three parameters per TSDA and RSDA");
        const auto* k = controls.Find("tsda.hanger.spring_coefficient");
        const auto* c = controls.Find("tsda.hanger.damping_coefficient");
        const auto* l0 = controls.Find("tsda.hanger.rest_length");
        const auto* rc = controls.Find("rsda.twist.damping_coefficient");
        const auto* rk = controls.Find("rsda.twist.spring_coefficient");
        const auto* a0 = controls.Find("rsda.twist.rest_angle");
        TEST_ASSERT(k && c && l0 && rc && rk && a0, "all parameter ids found");
        if (k && c && l0 && rc && rk && a0) {
            TEST_ASSERT(k->value == 500.0 && c->value == 40.0, "TSDA k, c read from Chrono's functor");
            TEST_ASSERT(l0->value == 1.8, "TSDA rest length read from the link");
            TEST_ASSERT(k->units == "N/m" && c->units == "N*s/m" && l0->units == "m", "TSDA units");
            TEST_ASSERT(rc->value == 7.0 && rc->units == "N*m*s/rad", "RSDA damping and units");
            TEST_ASSERT(rk->value == 0.0 && rk->source == "default", "RSDA spring not in a damper-only law");
            TEST_ASSERT(rc->source == "model YAML", "RSDA damping from the model");
            TEST_ASSERT(a0->value == 0.1 && a0->units == "rad", "RSDA rest angle [rad]");
            TEST_ASSERT(k->owner.empty() && rc->owner.empty() && a0->owner.empty(), "linear laws are editable");
        }
        controls.SetOwner("tsda.hanger.rest_length", "controller");
        std::string error;
        TEST_ASSERT(!controls.Request("tsda.hanger.rest_length", 2.0, &error), "owned parameter rejected");
        TEST_ASSERT(error.find("controller") != std::string::npos, "rejection names the owner");
        TEST_ASSERT(!controls.Request("tsda.nope.damping_coefficient", 1.0, &error), "unknown id rejected");
        TEST_ASSERT(!controls.Request("tsda.hanger.damping_coefficient", -1.0, &error), "negative damping rejected");
        TEST_ASSERT(!controls.Request("tsda.hanger.damping_coefficient", std::nan(""), &error), "NaN rejected");
        TEST_ASSERT(controls.Request("rsda.twist.rest_angle", -0.2, &error), "negative rest angle accepted");
    }
    {
        // Nonlinear and custom laws: coefficients owned, rest length still editable.
        auto nonlinear = std::make_shared<cu::NonlinearSpringForce>();
        nonlinear->add_pointK(-1.0, -100.0);
        nonlinear->add_pointK(1.0, 100.0);
        struct Custom : ::chrono::ChLinkRSDA::TorqueFunctor {
            double evaluate(double, double, double, double, const ::chrono::ChLinkRSDA&) override { return 0.0; }
        };
        auto rig = MakeRig(nonlinear, std::make_shared<Custom>());
        RuntimeControls controls(rig.system.get());
        TEST_ASSERT(controls.Find("tsda.hanger.spring_coefficient")->owner.find("nonlinear") != std::string::npos,
                    "nonlinear spring curve owns k");
        TEST_ASSERT(controls.Find("rsda.twist.damping_coefficient")->owner == "custom force law",
                    "custom torque law owns c");
        TEST_ASSERT(controls.Find("tsda.hanger.rest_length")->owner.empty(), "rest length editable");
    }

    // ---- 2. Gate: swapping in the SEA-Stack functor with equal values is bit-identical ----
    const std::vector<FunctorCase> cases = {
        {"LinearSpringDamper (+stops) / LinearSpringDamperTorque",
         [] {
             auto f = std::make_shared<cu::LinearSpringDamperForce>(500.0, 40.0, 3.0);
             f->enable_stops(1.9, 2.05);  // the hanger starts at 2.0 m moving down
             return f;
         },
         [] { return std::make_shared<cu::LinearSpringDamperTorque>(30.0, 2.0, 0.5); }},
        {"LinearSpring / LinearSpringTorque",
         [] { return std::make_shared<cu::LinearSpringForce>(500.0, 3.0); },
         [] { return std::make_shared<cu::LinearSpringTorque>(30.0, 0.5); }},
        {"LinearDamper / LinearDamperTorque",
         [] { return std::make_shared<cu::LinearDamperForce>(40.0); },
         [] { return std::make_shared<cu::LinearDamperTorque>(2.0); }},
        {"Chrono built-in law (no functor)", [] { return nullptr; }, [] { return nullptr; }},
    };
    for (const auto& fc : cases) {
        auto reference_rig = MakeRig(fc.force(), fc.torque());
        if (!fc.force()) {
            reference_rig.tsda->SetSpringCoefficient(500.0);
            reference_rig.tsda->SetDampingCoefficient(40.0);
            reference_rig.rsda->SetSpringCoefficient(30.0);
            reference_rig.rsda->SetDampingCoefficient(2.0);
        }
        const Trace reference = Run(reference_rig, nullptr);

        auto rig = MakeRig(fc.force(), fc.torque());
        if (!fc.force()) {
            rig.tsda->SetSpringCoefficient(500.0);
            rig.tsda->SetDampingCoefficient(40.0);
            rig.rsda->SetSpringCoefficient(30.0);
            rig.rsda->SetDampingCoefficient(2.0);
        }
        RuntimeControls controls(rig.system.get());
        const auto old_force = rig.tsda->GetForceFunctor();
        // Re-set every coefficient to its own value at step 0: forces the swap.
        for (const auto& p : controls.parameters()) {
            if (p.field != "rest_length" && p.field != "rest_angle") {
                std::string error;
                TEST_ASSERT(controls.Request(p.id, p.value, &error), std::string(fc.name) + ": request " + p.id);
            }
        }
        const Trace swapped = Run(rig, &controls);
        if (old_force) {
            TEST_ASSERT(rig.tsda->GetForceFunctor() != old_force, std::string(fc.name) + ": functor swapped");
        }
        const int diff = FirstDifference(reference, swapped);
        TEST_ASSERT(diff == -1, std::string(fc.name) + ": bit-identical after the swap (first difference at step " +
                                    std::to_string(diff) + ")");
        TEST_ASSERT(controls.events().size() == 4, std::string(fc.name) + ": four changes logged");
    }

    // ---- 3. A change acts from its step on, with the new coefficient ----
    {
        auto make = [] {
            return MakeRig(std::make_shared<cu::LinearSpringDamperForce>(500.0, 40.0, 3.0),
                           std::make_shared<cu::LinearSpringDamperTorque>(30.0, 2.0, 0.5));
        };
        auto reference_rig = make();
        const Trace reference = Run(reference_rig, nullptr);
        constexpr int kChangeStep = 150;
        auto rig = make();
        RuntimeControls controls(rig.system.get());
        std::string error;
        const Trace changed = Run(rig, &controls, [&](int step) {
            if (step == kChangeStep) controls.Request("tsda.hanger.damping_coefficient", 400.0, &error);
        });
        TEST_ASSERT(FirstDifference(reference, changed) == kChangeStep, "results change at the step of the edit");
        const auto& e = controls.events().at(0);
        TEST_ASSERT(e.command.step == kChangeStep && e.previous_value == 40.0 && e.command.value == 400.0 &&
                        e.origin == "gui",
                    "event logs step, previous and new value, origin");
        TEST_NEAR(e.command.time, kChangeStep * kDt, 1e-12, "event time is the boundary time");
        // F = P - k (L - L0) - c dL/dt with the new c (Chrono sign convention).
        auto& t = *rig.tsda;
        const double expected = 3.0 - 500.0 * (t.GetLength() - 1.8) - 400.0 * t.GetVelocity();
        const double f = t.GetForceFunctor()->evaluate(0.0, 1.8, t.GetLength(), t.GetVelocity(), t);
        TEST_NEAR(f, expected, 1e-9 * std::abs(expected) + 1e-12, "new damping in the force law");
        TEST_ASSERT(controls.Find("tsda.hanger.damping_coefficient")->value == 400.0, "registry shows new value");
        TEST_ASSERT(controls.Find("tsda.hanger.damping_coefficient")->default_value == 40.0, "default kept");

        // Rest length is a link property: Chrono's setter.
        auto rig2 = make();
        RuntimeControls controls2(rig2.system.get());
        controls2.Request("tsda.hanger.rest_length", 1.5, &error);
        controls2.ApplyDue(0, 0.0);
        TEST_ASSERT(rig2.tsda->GetRestLength() == 1.5, "rest length applied to the link");
    }

    // ---- 4. Schedule: same changes from a file reproduce the GUI run bit-for-bit ----
    {
        auto make = [] {
            return MakeRig(std::make_shared<cu::LinearSpringDamperForce>(500.0, 40.0, 3.0),
                           std::make_shared<cu::LinearDamperTorque>(2.0));
        };
        auto gui_rig = make();
        RuntimeControls gui(gui_rig.system.get());
        std::string error;
        const Trace gui_trace = Run(gui_rig, &gui, [&](int step) {
            if (step == 37) gui.Request("tsda.hanger.damping_coefficient", 123.456789012345, &error);
            if (step == 37) gui.Request("rsda.twist.damping_coefficient", 0.1 / 3.0, &error);
            if (step == 200) gui.Request("tsda.hanger.spring_coefficient", 750.0, &error);
            if (step == 300) gui.Request("tsda.hanger.damping_coefficient", 40.0, &error);  // reset
        });
        TEST_ASSERT(gui.events().size() == 4 && gui.HasGuiEvents(), "four GUI changes logged");

        const std::string path =
            (std::filesystem::temp_directory_path() / "seastack_test_runtime_schedule.yaml").string();
        seastack::chrono::WriteRuntimeSchedule(path, gui.events(), kDt, "unit test");
        const auto schedule = seastack::chrono::ReadRuntimeSchedule(path);
        TEST_ASSERT(schedule.commands.size() == 4, "schedule has four events");
        TEST_ASSERT(schedule.time_step == kDt, "time step round trip");
        TEST_ASSERT(schedule.commands[1].value == 0.1 / 3.0, "values round trip exactly (17 digits)");

        auto rerun_rig = make();
        RuntimeControls rerun(rerun_rig.system.get());
        rerun.LoadSchedule(schedule, 0.0, kDt);
        const Trace rerun_trace = Run(rerun_rig, &rerun);
        TEST_ASSERT(FirstDifference(gui_trace, rerun_trace) == -1, "schedule rerun is bit-identical to the GUI run");
        TEST_ASSERT(rerun.events().size() == 4 && !rerun.HasGuiEvents(), "rerun logs schedule events");
        bool same_log = rerun.events().size() == gui.events().size();
        for (size_t i = 0; same_log && i < gui.events().size(); ++i) {
            const auto& a = gui.events()[i].command;
            const auto& b = rerun.events()[i].command;
            same_log = a.step == b.step && a.time == b.time && a.parameter == b.parameter && a.value == b.value &&
                       gui.events()[i].previous_value == rerun.events()[i].previous_value;
        }
        TEST_ASSERT(same_log, "rerun event log equals the GUI log (except origin)");
        TEST_ASSERT(rerun.UnappliedScheduledCount() == 0, "all scheduled changes applied");

        // A different time step: events are placed by time, not step.
        auto other_rig = make();
        RuntimeControls other(other_rig.system.get());
        other.LoadSchedule(schedule, 0.0, 2.0 * kDt);
        for (long long s = 0; s < 400; ++s) other.ApplyDue(s, s * 2.0 * kDt);
        TEST_ASSERT(other.events().at(0).command.step == 19, "t = 0.037 s at dt = 2 ms -> step ceil(18.5) = 19");
        std::filesystem::remove(path);
    }
    {
        // Hand-written schedule: time only, and errors that name the entry.
        const std::string path =
            (std::filesystem::temp_directory_path() / "seastack_test_runtime_schedule_hand.yaml").string();
        {
            std::ofstream out(path);
            out << "events:\n  - {time: 0.1, parameter: tsda.hanger.damping_coefficient, value: 80}\n";
        }
        auto rig = MakeRig(std::make_shared<cu::LinearDamperForce>(40.0), nullptr);
        RuntimeControls controls(rig.system.get());
        controls.LoadSchedule(seastack::chrono::ReadRuntimeSchedule(path), 0.0, kDt);
        for (long long s = 0; s < 200; ++s) controls.ApplyDue(s, s * kDt);
        TEST_ASSERT(controls.events().size() == 1 && controls.events()[0].command.step == 100,
                    "time 0.1 s -> step 100 at dt = 1 ms");
        {
            std::ofstream out(path);
            out << "events:\n  - {time: 0.1, parameter: tsda.hanger.spring, value: 80}\n";
        }
        bool threw = false;
        try {
            RuntimeControls bad(rig.system.get());
            bad.LoadSchedule(seastack::chrono::ReadRuntimeSchedule(path), 0.0, kDt);
        } catch (const std::runtime_error& e) {
            threw = std::string(e.what()).find("event 1") != std::string::npos;
        }
        TEST_ASSERT(threw, "unknown parameter in a schedule is an error naming the event");
        {
            std::ofstream out(path);
            out << "events:\n  - {parameter: tsda.hanger.damping_coefficient, value: 80}\n";
        }
        threw = false;
        try {
            seastack::chrono::ReadRuntimeSchedule(path);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        TEST_ASSERT(threw, "event without step or time is an error");
        std::filesystem::remove(path);
    }

    // ---- 5. Registered non-link parameters (e.g. wave height) ----
    {
        auto rig = MakeRig(std::make_shared<cu::LinearDamperForce>(40.0), nullptr);
        RuntimeControls controls(rig.system.get());
        seastack::chrono::RuntimeParameter p;
        p.id = "waves.height";
        p.element = "waves";
        p.field = "height";
        p.label = "H";
        p.units = "m";
        p.default_value = p.value = 2.0;
        p.min_value = 0.0;
        p.source = "hydro YAML";
        std::vector<std::pair<double, double>> applied;  // (value, time)
        controls.AddParameter(p, [&](double value, double time) {
            applied.emplace_back(value, time);
            return time + 16.0;  // a ramped change
        });
        bool threw = false;
        try {
            controls.AddParameter(p, [](double, double time) { return time; });
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        TEST_ASSERT(threw, "duplicate id rejected");

        std::string error;
        TEST_ASSERT(!controls.Request("waves.height", -1.0, &error), "negative height rejected");
        TEST_ASSERT(controls.Request("waves.height", 1.0, &error), "height request accepted");
        controls.ApplyDue(7, 0.007);
        TEST_ASSERT(applied.size() == 1 && applied[0].first == 1.0 && applied[0].second == 0.007,
                    "apply function called at the boundary with value and time");
        const auto* h = controls.Find("waves.height");
        TEST_ASSERT(h && h->value == 1.0 && h->default_value == 2.0, "registry shows the new value");
        TEST_NEAR(h ? h->effective_time : 0.0, 16.007, 1e-12, "effective time from the apply function");
        const auto& e = controls.events().back();
        TEST_ASSERT(e.command.parameter == "waves.height" && e.previous_value == 2.0 && e.command.step == 7,
                    "change logged like a link parameter");

        controls.Request("tsda.hanger.damping_coefficient", 80.0, &error);
        controls.ApplyDue(8, 0.008);
        const auto* c = controls.Find("tsda.hanger.damping_coefficient");
        TEST_NEAR(c ? c->effective_time : 0.0, 0.008, 1e-15, "link changes are in effect at once");

        RuntimeControls rerun(rig.system.get());
        std::vector<double> rerun_values;
        rerun.AddParameter(p, [&](double value, double time) {
            rerun_values.push_back(value);
            return time;
        });
        seastack::chrono::RuntimeSchedule schedule;
        schedule.time_step = kDt;
        schedule.commands.push_back(controls.events().front().command);
        rerun.LoadSchedule(schedule, 0.0, kDt);
        for (long long s = 0; s < 10; ++s) rerun.ApplyDue(s, s * kDt);
        TEST_ASSERT(rerun_values.size() == 1 && rerun_values[0] == 1.0 && rerun.events()[0].command.step == 7,
                    "schedule replays a registered parameter at the same step");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
