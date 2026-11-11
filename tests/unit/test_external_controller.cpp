/*********************************************************************
 * @file  test_external_controller.cpp
 * @brief External controller (preview): channel registry vs docs, every
 *        channel read/written on a small Chrono system, zero-order hold,
 *        validation messages, and an RSDA rest-angle step vs the analytic
 *        damped-oscillator response. No Python: an in-process model stands
 *        in for the IPC module.
 *********************************************************************/

#include "external_controller.h"

#include "test_macros.h"

#include <chrono/functions/ChFunctionConst.h>
#include <chrono/functions/ChFunctionSetpoint.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkMotorLinearPosition.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>
#include <chrono/physics/ChLinkMotorRotationSpeed.h>
#include <chrono/physics/ChLinkRSDA.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystemNSC.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#ifndef SEASTACK_SOURCE_DIR
#error "SEASTACK_SOURCE_DIR must be defined"
#endif

namespace {

using seastack::app::ControllerConfig;
using seastack::app::ExternalController;
namespace ext = seastack::external;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// In-process stand-in for the Python module.
class FakeModel : public ext::IExternalForceModel {
  public:
    using StepFn = std::function<std::vector<double>(double, const std::vector<double>&)>;

    FakeModel(ext::ExternalInit* init_out, StepFn step) : init_out_(init_out), step_(std::move(step)) {}

    ext::ExternalMeta Initialize(const ext::ExternalInit& init) override {
        if (init_out_) {
            *init_out_ = init;
        }
        return {};
    }
    void Evaluate(double time, const std::vector<double>& in, std::vector<double>& out) override {
        out = step_(time, in);
    }
    void Shutdown() override {}

  private:
    ext::ExternalInit* init_out_;
    StepFn step_;
};

std::shared_ptr<chrono::ChBody> AddBody(chrono::ChSystem& sys, const std::string& name,
                                        double izz = 1.0) {
    auto b = chrono_types::make_shared<chrono::ChBody>();
    b->SetName(name);
    b->SetMass(1.0);
    b->SetInertiaXX(chrono::ChVector3d(1.0, 1.0, izz));
    sys.AddBody(b);
    return b;
}

/// ground + rotor (angle motor) + slider (linear motor) + arm (revolute + RSDA)
/// + TSDA ground-slider + a speed motor and a fixed-function angle motor.
struct Rig {
    chrono::ChSystemNSC sys;
    std::shared_ptr<chrono::ChBody> ground, rotor, slider, arm;
    std::shared_ptr<chrono::ChLinkMotorRotationAngle> rot_motor, const_motor;
    std::shared_ptr<chrono::ChLinkMotorLinearPosition> lin_motor;
    std::shared_ptr<chrono::ChLinkMotorRotationSpeed> speed_motor;
    std::shared_ptr<chrono::ChLinkTSDA> spring;
    std::shared_ptr<chrono::ChLinkRSDA> rsda;

    explicit Rig(double arm_izz = 1.0) {
        using namespace chrono;
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
        ground = AddBody(sys, "ground");
        ground->SetFixed(true);
        rotor = AddBody(sys, "rotor");
        slider = AddBody(sys, "slider");
        slider->SetPos(ChVector3d(0, 2, 0));
        arm = AddBody(sys, "arm", arm_izz);
        arm->SetPos(ChVector3d(0, 4, 0));
        auto spinner = AddBody(sys, "spinner");
        spinner->SetPos(ChVector3d(0, 6, 0));
        auto fixed_rotor = AddBody(sys, "fixed_rotor");
        fixed_rotor->SetPos(ChVector3d(0, 8, 0));

        rot_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        rot_motor->SetName("rot_motor");
        rot_motor->Initialize(rotor, ground, ChFrame<>(ChVector3d(0, 0, 0)));
        rot_motor->SetMotorFunction(chrono_types::make_shared<ChFunctionSetpoint>());
        sys.AddLink(rot_motor);

        lin_motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
        lin_motor->SetName("lin_motor");
        lin_motor->Initialize(slider, ground, ChFrame<>(ChVector3d(0, 2, 0)));
        lin_motor->SetMotorFunction(chrono_types::make_shared<ChFunctionSetpoint>());
        sys.AddLink(lin_motor);

        auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
        revolute->Initialize(arm, ground, ChFrame<>(ChVector3d(0, 4, 0)));
        sys.AddLink(revolute);
        rsda = chrono_types::make_shared<ChLinkRSDA>();
        rsda->SetName("hinge");
        rsda->Initialize(ground, arm, ChFrame<>(ChVector3d(0, 4, 0)));
        rsda->SetSpringCoefficient(50.0);
        rsda->SetDampingCoefficient(2.0);
        sys.AddLink(rsda);

        spring = chrono_types::make_shared<ChLinkTSDA>();
        spring->SetName("spring");
        spring->Initialize(ground, slider, false, ChVector3d(-1, 2, 0), ChVector3d(0, 2, 0));
        spring->SetSpringCoefficient(10.0);
        sys.AddLink(spring);

        speed_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        speed_motor->SetName("speed_motor");
        speed_motor->Initialize(spinner, ground, ChFrame<>(ChVector3d(0, 6, 0)));
        sys.AddLink(speed_motor);

        const_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        const_motor->SetName("const_motor");
        const_motor->Initialize(fixed_rotor, ground, ChFrame<>(ChVector3d(0, 8, 0)));
        const_motor->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
        sys.AddLink(const_motor);
    }
};

ControllerConfig AllChannelsConfig() {
    ControllerConfig cfg;
    cfg.command = {"python", "fake.py"};
    cfg.sensors.bodies = {"rotor"};
    cfg.sensors.motors = {"rot_motor", "lin_motor"};
    cfg.sensors.tsdas = {"spring"};
    cfg.sensors.rsdas = {"hinge"};
    cfg.actuators.motors = {"rot_motor", "lin_motor"};
    cfg.actuators.tsdas = {"spring"};
    cfg.actuators.rsdas = {"hinge"};
    return cfg;
}

/// Run `fn`; return the exception message ("" if none was thrown).
std::string ErrorOf(const std::function<void()>& fn) {
    try {
        fn();
    } catch (const std::exception& e) {
        return e.what();
    }
    return "";
}

bool Contains(const std::string& s, const std::string& part) {
    return s.find(part) != std::string::npos;
}

// ---------------------------------------------------------------------------

void TestDocsTableMatchesRegistry(TestResults& test_results) {
    const std::filesystem::path doc =
        std::filesystem::path(SEASTACK_SOURCE_DIR) / "docs/extending/CONTROLLERS.md";
    std::ifstream in(doc);
    TEST_ASSERT(in.good(), "docs/extending/CONTROLLERS.md exists");
    std::stringstream ss;
    ss << in.rdbuf();
    std::string text = ss.str();
    text.erase(std::remove(text.begin(), text.end(), '\r'), text.end());  // CRLF checkouts
    const std::string begin = "<!-- controller-channel-table:begin -->\n";
    const std::string end = "<!-- controller-channel-table:end -->";
    const size_t b = text.find(begin);
    const size_t e = text.find(end);
    TEST_ASSERT(b != std::string::npos && e != std::string::npos && e > b,
                "channel table markers present in CONTROLLERS.md");
    if (b == std::string::npos || e == std::string::npos || e <= b) {
        return;
    }
    const std::string in_doc = text.substr(b + begin.size(), e - b - begin.size());
    const std::string expected = seastack::app::ControllerChannelTableMarkdown();
    TEST_ASSERT(in_doc == expected, "CONTROLLERS.md channel table matches the C++ registry");
    if (in_doc != expected) {
        const std::string out_path = "controller_channel_table.md";
        std::ofstream(out_path) << expected;
        std::cerr << "Replace the table in docs/extending/CONTROLLERS.md with the contents of "
                  << std::filesystem::absolute(out_path).string() << ":\n"
                  << expected << std::endl;
    }
}

void TestEveryChannelReadAndWritten(TestResults& test_results) {
    Rig rig;
    ext::ExternalInit init;
    std::vector<std::vector<double>> seen_inputs;
    auto model = std::make_unique<FakeModel>(
        &init, [&](double, const std::vector<double>& in) {
            seen_inputs.push_back(in);
            // angle, angle_dt (NaN = hold), position, position_dt, rest_length, rest_angle
            return std::vector<double>{0.3, kNaN, 0.2, 0.0, 1.5, 0.25};
        });
    ExternalController ctl(rig.sys, AllChannelsConfig(), 1e-3, std::move(model));

    // Expected names straight from the registry, in publication order.
    std::vector<std::string> expected_in, expected_out;
    for (const auto& c : seastack::app::ControllerChannelRegistry()) {
        const std::string el = c.element;
        const std::string ap = c.applies;
        std::vector<std::string> names;
        if (el == "body") names = {"rotor"};
        if (el == "motor" && ap == "rotation") names = {"rot_motor"};
        if (el == "motor" && ap == "linear") names = {"lin_motor"};
        if (el == "tsda") names = {"spring"};
        if (el == "rsda") names = {"hinge"};
        for (const auto& n : names) {
            expected_in.push_back(el + "." + n + "." + c.field);
            if (c.writable) expected_out.push_back(el + "." + n + "." + c.field);
        }
    }
    // Registry order groups by element; the controller groups by YAML list
    // (bodies, motors, tsdas, rsdas), which is the same order here.
    TEST_ASSERT(init.kind == "controller", "handshake kind is controller");
    TEST_ASSERT(init.in_names == expected_in, "every registry sensor channel published once");
    TEST_ASSERT(init.out_names == expected_out, "every writable registry channel accepted once");
    TEST_ASSERT(init.n_inputs == static_cast<int>(expected_in.size()), "n_inputs");
    TEST_ASSERT(init.n_outputs == 6, "n_outputs = 2 + 2 + 1 + 1");

    ctl.BeforeStep(0.0);
    TEST_ASSERT(seen_inputs.size() == 1, "evaluated on the first step");
    bool all_finite = true;
    for (double v : seen_inputs[0]) all_finite = all_finite && std::isfinite(v);
    TEST_ASSERT(all_finite, "all sensor values finite");

    auto rot_sp = std::dynamic_pointer_cast<chrono::ChFunctionSetpoint>(rig.rot_motor->GetMotorFunction());
    auto lin_sp = std::dynamic_pointer_cast<chrono::ChFunctionSetpoint>(rig.lin_motor->GetMotorFunction());
    TEST_NEAR(rot_sp->GetVal(0.0), 0.3, 1e-15, "rotation motor angle set-point applied");
    TEST_NEAR(rot_sp->GetDer(0.0), 0.0, 1e-15, "NaN rate held at its initial value (0)");
    TEST_NEAR(lin_sp->GetVal(0.0), 0.2, 1e-15, "linear motor position set-point applied");
    TEST_NEAR(rig.spring->GetRestLength(), 1.5, 1e-15, "TSDA rest length applied");
    TEST_NEAR(rig.rsda->GetRestAngle(), 0.25, 1e-15, "RSDA rest angle applied");

    for (int i = 0; i < 2000; ++i) {
        ctl.BeforeStep(rig.sys.GetChTime());
        rig.sys.DoStepDynamics(1e-3);
    }
    TEST_NEAR(rig.rot_motor->GetMotorAngle(), 0.3, 1e-6, "rotation motor reaches the set-point");
    TEST_NEAR(rig.lin_motor->GetMotorPos(), 0.2, 1e-6, "linear motor reaches the set-point");
    const auto& last = seen_inputs.back();
    const auto idx = [&](const std::string& n) {
        return static_cast<size_t>(std::find(init.in_names.begin(), init.in_names.end(), n) -
                                   init.in_names.begin());
    };
    TEST_NEAR(last[idx("motor.rot_motor.angle")], 0.3, 1e-5, "read back motor angle under the written name");
    TEST_NEAR(last[idx("motor.lin_motor.position")], 0.2, 1e-5, "read back motor position");
    TEST_NEAR(last[idx("tsda.spring.rest_length")], 1.5, 1e-15, "read back TSDA rest length");
    TEST_NEAR(last[idx("rsda.hinge.rest_angle")], 0.25, 1e-15, "read back RSDA rest angle");
    TEST_NEAR(last[idx("body.rotor.rot_z")], 0.3, 1e-5, "body Cardan z angle follows the motor");
}

void TestZeroOrderHold(TestResults& test_results) {
    Rig rig;
    std::vector<double> tick_times;
    double command = 0.0;
    ControllerConfig cfg;
    cfg.command = {"fake"};
    cfg.actuators.tsdas = {"spring"};
    cfg.control_dt = 0.005;
    auto model = std::make_unique<FakeModel>(nullptr, [&](double t, const std::vector<double>&) {
        tick_times.push_back(t);
        command += 1.0;
        // Every third tick: hold (NaN).
        return std::vector<double>{tick_times.size() % 3 == 0 ? kNaN : command};
    });
    ExternalController ctl(rig.sys, cfg, 0.001, std::move(model));
    TEST_ASSERT(ctl.sensor_names().empty(), "actuator-only controller has no sensors");
    std::vector<double> rest_seen;
    for (int i = 0; i < 20; ++i) {
        ctl.BeforeStep(rig.sys.GetChTime());
        rest_seen.push_back(rig.spring->GetRestLength());
        rig.sys.DoStepDynamics(0.001);
    }
    TEST_ASSERT(tick_times.size() == 4, "20 steps at control_dt = 5 steps -> 4 ticks");
    TEST_NEAR(tick_times[1], 0.005, 1e-12, "second tick at t = control_dt");
    TEST_NEAR(rest_seen[0], 1.0, 0.0, "tick 1 command applied on its own step");
    TEST_NEAR(rest_seen[4], 1.0, 0.0, "held between ticks");
    TEST_NEAR(rest_seen[5], 2.0, 0.0, "tick 2 command");
    TEST_NEAR(rest_seen[10], 2.0, 0.0, "tick 3 returned NaN: previous command held");
    TEST_NEAR(rest_seen[15], 4.0, 0.0, "tick 4 command");
}

void TestValidationMessages(TestResults& test_results) {
    Rig rig;
    auto make = [&](const ControllerConfig& cfg, double dt = 0.001) {
        ExternalController ctl(rig.sys, cfg, dt,
                               std::make_unique<FakeModel>(nullptr, [](double, const std::vector<double>&) {
                                   return std::vector<double>{};
                               }));
    };
    ControllerConfig cfg;
    cfg.command = {"fake"};

    cfg.sensors.bodies = {"rotr"};
    std::string msg = ErrorOf([&] { make(cfg); });
    TEST_ASSERT(Contains(msg, "no body named 'rotr'") && Contains(msg, "Did you mean 'rotor'") &&
                    Contains(msg, "Bodies in the model: ground, rotor"),
                "unknown body: suggestion and valid names (" + msg + ")");

    cfg = ControllerConfig{};
    cfg.command = {"fake"};
    cfg.actuators.motors = {"speed_motor"};
    msg = ErrorOf([&] { make(cfg); });
    TEST_ASSERT(Contains(msg, "not a position motor") && Contains(msg, "actuation_type: POSITION"),
                "speed motor rejected as actuator (" + msg + ")");

    cfg.actuators.motors = {"const_motor"};
    msg = ErrorOf([&] { make(cfg); });
    TEST_ASSERT(Contains(msg, "fixed actuation_function") && Contains(msg, "{type: CONTROLLER}"),
                "fixed-function motor rejected (" + msg + ")");

    cfg.actuators.motors = {};
    cfg.actuators.tsdas = {"hinge"};
    msg = ErrorOf([&] { make(cfg); });
    TEST_ASSERT(Contains(msg, "no TSDA named 'hinge'") && Contains(msg, "TSDAs in the model: spring"),
                "RSDA name used as TSDA (" + msg + ")");

    cfg.actuators.tsdas = {"spring"};
    cfg.control_dt = 0.0025;
    msg = ErrorOf([&] { make(cfg); });
    TEST_ASSERT(Contains(msg, "whole multiple") && Contains(msg, "0.002") && Contains(msg, "0.003"),
                "control_dt not a multiple of time_step (" + msg + ")");

    cfg.control_dt = 0.0;
    cfg.sensors.motors = {"speed_motor"};
    msg = ErrorOf([&] { make(cfg); });
    TEST_ASSERT(msg.empty(), "speed motor can be read (" + msg + ")");

    // Module error at a tick carries time and message.
    cfg = AllChannelsConfig();
    ExternalController failing(rig.sys, cfg, 0.001,
                               std::make_unique<FakeModel>(nullptr, [](double t, const std::vector<double>&)
                                                                         -> std::vector<double> {
                                   if (t > 0.0015) throw std::runtime_error("evaluate: Traceback\nZeroDivisionError");
                                   return {kNaN, kNaN, kNaN, kNaN, kNaN, kNaN};
                               }));
    failing.BeforeStep(0.0);
    failing.BeforeStep(0.001);
    msg = ErrorOf([&] { failing.BeforeStep(0.002); });
    TEST_ASSERT(Contains(msg, "failed at t = 0.002 s") && Contains(msg, "ZeroDivisionError"),
                "module failure names time and error (" + msg + ")");
}

void TestYamlValidation(TestResults& test_results) {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "seastack_test_controller_yaml";
    fs::create_directories(dir);
    { std::ofstream(dir / "ctl.py") << "# placeholder\n"; }
    auto load = [&](const std::string& yaml) {
        { std::ofstream(dir / "c.controller.yaml") << yaml; }
        return ErrorOf([&] { (void)seastack::app::LoadControllerFile(dir / "c.controller.yaml"); });
    };
    const std::string ok_head = "command: [python, ctl.py]\n";

    TEST_ASSERT(load(ok_head + "sensors:\n  bodies: [a]\n").empty(), "minimal file loads");
    struct Case {
        std::string yaml;
        std::vector<std::string> expect;
    };
    const std::vector<Case> cases = {
        {ok_head + "sensor:\n  bodies: [a]\n", {"unknown key 'sensor'", "Did you mean 'sensors'"}},
        {ok_head + "sensors:\n  body: [a]\n", {"unknown key 'body'", "Did you mean 'bodies'"}},
        {ok_head + "actuators:\n  bodies: [a]\n", {"bodies are read-only"}},
        {ok_head + "sensors:\n  bodies: a\n", {"must be a list of names, e.g. [a]"}},
        {ok_head + "sensors:\n  bodies: [a, a]\n", {"lists 'a' twice"}},
        {ok_head + "sensors: {}\n", {"lists no sensors and no actuators"}},
        {"sensors:\n  bodies: [a]\n", {"'command' is required"}},
        {"command: [python, missing.py]\nsensors:\n  bodies: [a]\n", {"controller script not found", "missing.py"}},
        {ok_head + "control_dt: -0.1\nsensors:\n  bodies: [a]\n", {"control_dt must be > 0"}},
        {ok_head + "control_dt: fast\nsensors:\n  bodies: [a]\n", {"control_dt must be a number"}},
        {ok_head + "timeout_ms: 0\nsensors:\n  bodies: [a]\n", {"timeout_ms must be > 0"}},
        {ok_head + "config: 3\nsensors:\n  bodies: [a]\n", {"'config' must be a map"}},
        {ok_head + "sensors: [a\n", {"YAML syntax error"}},
    };
    for (const auto& c : cases) {
        const std::string msg = load(c.yaml);
        bool ok = !msg.empty();
        for (const auto& part : c.expect) ok = ok && Contains(msg, part);
        TEST_ASSERT(ok, "YAML case -> '" + c.expect.front() + "' (got: " + msg + ")");
    }

    { std::ofstream(dir / "c.controller.yaml") << ok_head << "sensors:\n  bodies: [a]\n"
                                               << "config:\n  gain: 2\n  name: \"1.0\"\n  on: true\n"
                                               << "  nested: {list: [1, 2], text: hi}\n"; }
    const auto cfg = seastack::app::LoadControllerFile(dir / "c.controller.yaml");
    TEST_ASSERT(cfg.config_json ==
                    "{\"gain\":2,\"name\":\"1.0\",\"on\":true,\"nested\":{\"list\":[1,2],\"text\":\"hi\"}}",
                "config block -> JSON keeps nesting, quoted strings and booleans (" + cfg.config_json + ")");
    TEST_ASSERT(fs::path(cfg.command[1]).is_absolute(), "script path made absolute");
    fs::remove_all(dir);
}

/// RSDA rest-angle step: I a'' + c a' + k (a - a0) = 0 from rest at a = 0.
void TestRsdaStepMatchesAnalytic(TestResults& test_results) {
    const double inertia = 2.0;  // [kg m^2] about the hinge (z)
    const double k = 50.0;       // [N m / rad]
    const double c = 2.0;        // [N m s / rad]
    const double a0 = 0.2;       // [rad] commanded rest angle
    const double dt = 1e-4;      // [s]
    Rig rig(inertia);
    ControllerConfig cfg;
    cfg.command = {"fake"};
    cfg.actuators.rsdas = {"hinge"};
    cfg.control_dt = 10 * dt;
    ExternalController ctl(rig.sys, cfg, dt,
                           std::make_unique<FakeModel>(nullptr, [&](double, const std::vector<double>&) {
                               return std::vector<double>{a0};
                           }));
    const double wn = std::sqrt(k / inertia);
    const double zeta = c / (2.0 * std::sqrt(k * inertia));
    const double wd = wn * std::sqrt(1.0 - zeta * zeta);
    double max_err = 0.0;
    for (int i = 0; i < 30000; ++i) {
        ctl.BeforeStep(rig.sys.GetChTime());
        rig.sys.DoStepDynamics(dt);
        const double t = rig.sys.GetChTime();
        const double exact =
            a0 * (1.0 - std::exp(-zeta * wn * t) *
                            (std::cos(wd * t) + zeta / std::sqrt(1.0 - zeta * zeta) * std::sin(wd * t)));
        max_err = std::max(max_err, std::abs(rig.rsda->GetAngle() - exact));
    }
    std::cout << "RSDA step: max |a - a_exact| = " << max_err << " rad over 3 s (a0 = " << a0
              << " rad, dt = " << dt << " s)" << std::endl;
    // Implicit Euler at wn*dt = 5e-4: observed max error ~2.5e-4 a0.
    TEST_ASSERT(max_err < 1e-3 * a0, "RSDA step response within 0.1% of a0 of the analytic solution");
}

/// Tension-only rope: force law, a hanging mass (static stretch m g L0 / EA),
/// free fall while slack, and ropes_file validation.
void TestRopes(TestResults& test_results) {
    using namespace ::chrono;
    const double ea = 1.0e6;  // [N]
    const double c = 2.0e3;   // [N s/m]
    const ChLinkTSDA unused;
    seastack::app::TensionOnlyRopeForce law(ea, c);
    TEST_NEAR(law.evaluate(0, 10.0, 9.0, 0.0, unused), 0.0, 0.0, "slack rope: no force");
    TEST_NEAR(law.evaluate(0, 10.0, 9.0, 5.0, unused), 0.0, 0.0,
              "slack rope closing, far from taut: no force");
    TEST_NEAR(law.evaluate(0, 10.0, 9.999, 1.0, unused), -(ea / 10.0 * -0.001 + c * 1.0), 1e-9,
              "slack rope closing within c v L0 / EA of taut: damping pulls (continuous law)");
    TEST_NEAR(law.evaluate(0, 10.0, 10.01, 0.0, unused), -ea / 10.0 * 0.01, 1e-9,
              "taut rope: -EA/L0 (L - L0), negative = tension");
    TEST_NEAR(law.evaluate(0, 10.0, 10.01, -10.0, unused), 0.0, 0.0,
              "taut rope recoiling fast: clamped at zero tension");

    // Mass m hanging from the ground on a rope with rest length L0.
    const double m = 100.0, g = 9.81, l0 = 10.0, dt = 1e-3;
    auto make_system = [&](double z0, std::shared_ptr<ChBody>& mass,
                           std::shared_ptr<ChLinkTSDA>& rope) {
        auto sys = std::make_unique<ChSystemNSC>();
        sys->SetGravitationalAcceleration(ChVector3d(0, 0, -g));
        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetFixed(true);
        sys->AddBody(ground);
        mass = chrono_types::make_shared<ChBody>();
        mass->SetMass(m);
        mass->SetPos(ChVector3d(0, 0, z0));
        sys->AddBody(mass);
        rope = chrono_types::make_shared<ChLinkTSDA>();
        rope->SetName("hoist");
        rope->Initialize(ground, mass, false, ChVector3d(0, 0, 0), ChVector3d(0, 0, z0));
        rope->SetRestLength(l0);
        rope->SetSpringCoefficient(123.0);  // must be ignored once the rope law is applied
        sys->AddLink(rope);
        seastack::app::ApplyRopes(*sys, {{"hoist", ea, c}});
        return sys;
    };

    std::shared_ptr<ChBody> mass;
    std::shared_ptr<ChLinkTSDA> rope;
    auto sys = make_system(-l0, mass, rope);
    for (int i = 0; i < 5000; ++i) sys->DoStepDynamics(dt);
    const double stretch = m * g * l0 / ea;  // 9.81e-3 m
    TEST_NEAR(-mass->GetPos().z() - l0, stretch, 1e-3 * stretch,
              "hanging mass: static stretch = m g L0 / EA within 0.1%");
    TEST_NEAR(rope->GetForce(), -m * g, 1e-3 * m * g, "hanging mass: rope force = -m g");

    // Start 1 m above the rope's reach: free fall until the rope goes taut.
    sys = make_system(-(l0 - 1.0), mass, rope);
    for (int i = 0; i < 200; ++i) sys->DoStepDynamics(dt);
    const double t = 0.2;  // [s] fall = g t^2 / 2 = 0.196 m < 1 m: still slack
    TEST_NEAR(mass->GetPos().z(), -(l0 - 1.0) - 0.5 * g * t * t, 2e-3,
              "slack rope: free fall (z within 2 mm of -g t^2 / 2 at t = 0.2 s)");
    TEST_NEAR(rope->GetForce(), 0.0, 0.0, "slack rope: zero force");

    std::string msg = ErrorOf([&] {
        ChSystemNSC empty;
        seastack::app::ApplyRopes(empty, {{"hoist", ea, c}});
    });
    TEST_ASSERT(Contains(msg, "no TSDA named 'hoist'"), "ropes: unknown TSDA (" + msg + ")");

    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "seastack_test_ropes_yaml";
    fs::create_directories(dir);
    auto load = [&](const std::string& yaml) {
        { std::ofstream(dir / "r.ropes.yaml") << yaml; }
        return ErrorOf([&] { (void)seastack::app::LoadRopesFile(dir / "r.ropes.yaml"); });
    };
    TEST_ASSERT(load("ropes:\n  - {link: hoist, ea_N: 8.0e6, damping_Ns_per_m: 8.0e4}\n").empty(),
                "ropes_file: valid file loads");
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"rope:\n  - {link: hoist, ea_N: 1}\n", "Did you mean 'ropes'"},
        {"ropes:\n  - {link: hoist, ea: 1}\n", "Did you mean 'ea_N'"},
        {"ropes:\n  - {link: hoist}\n", "'ea_N' (axial stiffness EA [N]) is required"},
        {"ropes:\n  - {link: hoist, ea_N: -1}\n", "ea_N must be > 0"},
        {"ropes:\n  - {link: hoist, ea_N: 1, damping_Ns_per_m: -1}\n", "damping_Ns_per_m must be >= 0"},
        {"ropes:\n  - {link: hoist, ea_N: 1, law: linear}\n", "the only law is tension_only"},
        {"ropes:\n  - {link: hoist, ea_N: 1}\n  - {link: hoist, ea_N: 2}\n", "lists TSDA 'hoist' twice"},
        {"ropes: []\n", "non-empty list"},
    };
    for (const auto& [yaml, expect] : cases) {
        msg = load(yaml);
        TEST_ASSERT(Contains(msg, expect), "ropes_file case -> '" + expect + "' (got: " + msg + ")");
    }
    fs::remove_all(dir);
}

}  // namespace

int main() {
    TestResults test_results;
    TestDocsTableMatchesRegistry(test_results);
    TestEveryChannelReadAndWritten(test_results);
    TestZeroOrderHold(test_results);
    TestValidationMessages(test_results);
    TestYamlValidation(test_results);
    TestRsdaStepMatchesAnalytic(test_results);
    TestRopes(test_results);
    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
