#include <seastack/external/external_force_component.h>
#include <seastack/external/external_force_model.h>
#include <seastack/external/external_pto_model.h>
#include <seastack/external/fmi_external_force_model.h>

#include "test_macros.h"

#include <memory>
#include <stdexcept>
#include <vector>

namespace {

/// In-process mock: F = -c * v for PTO; zeros for body-force layout.
class MockBackend : public seastack::external::IExternalForceModel {
  public:
    explicit MockBackend(double damping = 50.0) : damping_(damping) {}

    seastack::external::ExternalMeta Initialize(
        const seastack::external::ExternalInit& init) override {
        init_ = init;
        ++init_calls_;
        seastack::external::ExternalMeta meta;
        meta.name = "MockBackend";
        meta.version = "test";
        meta.n_states = 0;
        return meta;
    }

    void Evaluate(double /*time*/,
                  const std::vector<double>& in,
                  std::vector<double>& out) override {
        ++eval_calls_;
        out.assign(static_cast<size_t>(init_.n_outputs), 0.0);
        last_in_ = in;
        const bool is_pto = init_.kind == "pto" || init_.kind == "pto_tsda" ||
                            init_.kind == "pto_rsda";
        if (is_pto && in.size() >= 2 && !out.empty()) {
            out[0] = -damping_ * in[1];
        }
    }

    void Reset() override { ++reset_calls_; }
    void Shutdown() override { ++shutdown_calls_; }

    int init_calls_ = 0;
    int eval_calls_ = 0;
    int reset_calls_ = 0;
    int shutdown_calls_ = 0;
    std::vector<double> last_in_;
    seastack::external::ExternalInit init_{};

  private:
    double damping_;
};

}  // namespace

int main() {
    TestResults test_results;
    constexpr double tol = 1e-12;

    // --- ExternalPtoModel time-caching ---
    {
        auto backend = std::make_unique<MockBackend>(50.0);
        MockBackend* raw = backend.get();
        seastack::external::ExternalPtoModel pto(std::move(backend));
        pto.Initialize(0.01, "{\"damping\":50}");

        TEST_NEAR(pto.ComputeForce(0.0, 2.0, 0.0), -100.0, tol,
                  "first evaluate");
        TEST_ASSERT(raw->eval_calls_ == 1, "one backend call at t=0");
        TEST_ASSERT(pto.evaluate_call_count() == 1, "pto call count 1");

        // Same time → cache hit
        TEST_NEAR(pto.ComputeForce(0.5, 9.0, 0.0), -100.0, tol,
                  "cached force ignores new state at same t");
        TEST_ASSERT(raw->eval_calls_ == 1, "no second backend call at t=0");

        // New time → new evaluate
        TEST_NEAR(pto.ComputeForce(0.1, -1.0, 0.01), 50.0, tol,
                  "new time evaluates");
        TEST_ASSERT(raw->eval_calls_ == 2, "second backend call at t=0.01");

        // Backward time → cache
        TEST_NEAR(pto.ComputeForce(0.0, 100.0, 0.005), 50.0, tol,
                  "earlier time returns cache");
        TEST_ASSERT(raw->eval_calls_ == 2, "no call on earlier time");

        pto.Reset();
        TEST_ASSERT(raw->reset_calls_ == 1, "reset forwarded");
        pto.Shutdown();
        TEST_ASSERT(raw->shutdown_calls_ == 1, "shutdown forwarded");
    }

    // --- ExternalForceComponent packing / caching ---
    {
        auto backend = std::make_unique<MockBackend>();
        MockBackend* raw = backend.get();
        seastack::external::ExternalForceComponent comp(std::move(backend), 1);
        comp.Initialize(0.01);

        seastack::hydro::SystemState state;
        state.bodies.resize(1);
        state.bodies[0].linear_velocity.z() = 1.0;

        seastack::hydro::BodyForces forces(1);
        comp.Compute(state, 0.0, forces);
        TEST_ASSERT(raw->eval_calls_ == 1, "body force one evaluate");
        TEST_ASSERT(comp.evaluate_call_count() == 1, "component call count");

        // Same time: no new evaluate; forces accumulate cached contribution again
        forces[0].setZero();
        comp.Compute(state, 0.0, forces);
        TEST_ASSERT(raw->eval_calls_ == 1, "body force cache hit");

        state.bodies[0].linear_velocity.z() = 2.0;
        forces[0].setZero();
        comp.Compute(state, 0.02, forces);
        TEST_ASSERT(raw->eval_calls_ == 2, "body force new time");
        comp.Shutdown();
    }

    // --- FMI prototype throws ---
    {
        bool threw = false;
        try {
            seastack::external::FmiExternalForceModel fmi(
                seastack::external::FmiExternalForceOptions{"dummy.fmu"});
            seastack::external::ExternalInit init;
            init.kind = "pto";
            init.n_inputs = 2;
            init.n_outputs = 1;
            fmi.Initialize(init);
        } catch (const std::runtime_error& e) {
            threw = true;
            TEST_ASSERT(std::string(e.what()).find("FMI") != std::string::npos,
                        "FMI error mentions FMI");
        }
        TEST_ASSERT(threw, "FMI prototype throws on Initialize");
    }

    // --- Rich TSDA packing + FD accel ---
    {
        auto backend = std::make_unique<MockBackend>(10.0);
        MockBackend* raw = backend.get();
        seastack::external::ExternalPtoModel pto(std::move(backend));
        pto.SetLinkKind(seastack::external::ExternalPtoLinkKind::Tsda);
        pto.EnableRichState(true);
        pto.Initialize(0.1);

        TEST_ASSERT(raw->init_.n_inputs == 17, "rich TSDA n_inputs == 17");
        TEST_ASSERT(raw->init_.kind == "pto_tsda", "kind pto_tsda");
        TEST_ASSERT(raw->init_.in_names.size() == 17, "17 in_names");
        TEST_ASSERT(raw->init_.in_names[0] == "displacement", "name[0]");
        TEST_ASSERT(raw->init_.in_names[2] == "length", "name[2]=length");

        seastack::external::ExternalPtoState s0;
        s0.time = 0.0;
        s0.displacement = 0.5;
        s0.velocity = 2.0;
        s0.length = 1.5;
        s0.rest_length = 1.0;
        s0.body1_pos[0] = 3.0;
        s0.body2_vel[2] = -4.0;
        TEST_NEAR(pto.ComputeForce(s0), -20.0, tol, "rich first force");
        TEST_ASSERT(raw->last_in_.size() == 17, "packed 17");
        TEST_NEAR(raw->last_in_[0], 0.5, tol, "disp");
        TEST_NEAR(raw->last_in_[1], 2.0, tol, "vel");
        TEST_NEAR(raw->last_in_[2], 1.5, tol, "length");
        TEST_NEAR(raw->last_in_[3], 1.0, tol, "rest_length");
        TEST_NEAR(raw->last_in_[4], 0.0, tol, "rel_accel first step");
        TEST_NEAR(raw->last_in_[5], 3.0, tol, "body1_pos_x");
        TEST_NEAR(raw->last_in_[16], -4.0, tol, "body2_vel_z");

        seastack::external::ExternalPtoState s1 = s0;
        s1.time = 0.1;
        s1.velocity = 3.0;
        TEST_NEAR(pto.ComputeForce(s1), -30.0, tol, "rich second force");
        TEST_NEAR(raw->last_in_[4], 10.0, tol, "FD rel_accel=(3-2)/0.1");
    }

    // --- Rich RSDA channel names ---
    {
        auto backend = std::make_unique<MockBackend>(1.0);
        MockBackend* raw = backend.get();
        seastack::external::ExternalPtoModel pto(std::move(backend));
        pto.SetLinkKind(seastack::external::ExternalPtoLinkKind::Rsda);
        pto.EnableRichState(true);
        pto.Initialize(0.01);
        TEST_ASSERT(raw->init_.kind == "pto_rsda", "kind pto_rsda");
        TEST_ASSERT(raw->init_.in_names[2] == "angle", "name[2]=angle");
        TEST_ASSERT(raw->init_.in_names[3] == "rest_angle", "name[3]=rest_angle");

        seastack::external::ExternalPtoState s;
        s.time = 0.0;
        s.displacement = 0.25;
        s.velocity = -2.0;
        s.angle = 1.25;
        s.rest_angle = 1.0;
        TEST_NEAR(pto.ComputeForce(s), 2.0, tol, "RSDA torque from -c*w");
        TEST_NEAR(raw->last_in_[2], 1.25, tol, "angle packed");
        TEST_NEAR(raw->last_in_[3], 1.0, tol, "rest_angle packed");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
