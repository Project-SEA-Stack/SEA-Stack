/*********************************************************************
 * @file  test_external_pto_functors.cpp
 * @brief State assembly for ExternalPtoForceFunctor / ExternalPtoTorqueFunctor.
 *********************************************************************/
#include <seastack/adapters/chrono/pto_chrono_adapter.h>
#include <seastack/config.h>
#include <seastack/external/external_force_model.h>
#include <seastack/external/external_pto_model.h>

#include "test_macros.h"

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkRSDA.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystemNSC.h>

#include <memory>
#include <vector>

#ifndef SEASTACK_HAVE_EXTERNAL
#error "test_external_pto_functors requires SEASTACK_HAVE_EXTERNAL"
#endif

namespace {

class CaptureBackend : public seastack::external::IExternalForceModel {
  public:
    seastack::external::ExternalMeta Initialize(
        const seastack::external::ExternalInit& init) override {
        init_ = init;
        seastack::external::ExternalMeta meta;
        meta.name = "CaptureBackend";
        meta.version = "test";
        return meta;
    }

    void Evaluate(double /*time*/,
                  const std::vector<double>& in,
                  std::vector<double>& out) override {
        last_in_ = in;
        out.assign(1, -7.0 * (in.size() > 1 ? in[1] : 0.0));
    }

    void Reset() override {}
    void Shutdown() override {}

    seastack::external::ExternalInit init_{};
    std::vector<double> last_in_;
};

}  // namespace

int main() {
    TestResults test_results;
    constexpr double tol = 1e-9;

    // --- TSDA rich functor ---
    {
        chrono::ChSystemNSC system;
        auto body1 = chrono_types::make_shared<chrono::ChBody>();
        auto body2 = chrono_types::make_shared<chrono::ChBody>();
        body1->SetPos(chrono::ChVector3d(1.0, 2.0, 3.0));
        body1->SetPosDt(chrono::ChVector3d(0.1, 0.2, 0.3));
        body2->SetPos(chrono::ChVector3d(-1.0, -2.0, -3.0));
        body2->SetPosDt(chrono::ChVector3d(-0.1, -0.2, -0.3));
        body1->SetFixed(true);
        system.Add(body1);
        system.Add(body2);

        auto tsda = chrono_types::make_shared<chrono::ChLinkTSDA>();
        tsda->Initialize(body1, body2, false, chrono::ChVector3d(0, 0, 0),
                         chrono::ChVector3d(0, 0, 1.0));
        tsda->SetRestLength(0.5);
        system.AddLink(tsda);

        auto backend = std::make_unique<CaptureBackend>();
        CaptureBackend* raw = backend.get();
        auto model = std::make_shared<seastack::external::ExternalPtoModel>(
            std::move(backend));
        model->SetLinkKind(seastack::external::ExternalPtoLinkKind::Tsda);
        model->EnableRichState(true);
        model->Initialize(0.01);
        tsda->RegisterForceFunctor(
            std::make_shared<seastack::chrono::ExternalPtoForceFunctor>(model));

        // Force one evaluation through Chrono's functor path.
        const double length = tsda->GetLength();
        const double rest = tsda->GetRestLength();
        const double vel = 1.5;
        auto fun = tsda->GetForceFunctor();
        TEST_ASSERT(fun != nullptr, "TSDA force functor attached");
        const double F =
            fun->evaluate(0.0, rest, length, vel, *tsda);
        TEST_NEAR(F, -7.0 * vel, tol, "TSDA force = -7*vel");
        TEST_ASSERT(raw->last_in_.size() == 17, "TSDA packed 17");
        TEST_NEAR(raw->last_in_[0], length - rest, tol, "TSDA disp");
        TEST_NEAR(raw->last_in_[1], vel, tol, "TSDA vel");
        TEST_NEAR(raw->last_in_[2], length, tol, "TSDA length");
        TEST_NEAR(raw->last_in_[3], rest, tol, "TSDA rest");
        TEST_NEAR(raw->last_in_[5], 1.0, tol, "body1_pos_x");
        TEST_NEAR(raw->last_in_[7], 3.0, tol, "body1_pos_z");
        TEST_NEAR(raw->last_in_[11], -1.0, tol, "body2_pos_x");
        TEST_NEAR(raw->last_in_[14], -0.1, tol, "body2_vel_x");
    }

    // --- RSDA rich functor ---
    {
        chrono::ChSystemNSC system;
        auto body1 = chrono_types::make_shared<chrono::ChBody>();
        auto body2 = chrono_types::make_shared<chrono::ChBody>();
        body1->SetFixed(true);
        body1->SetPos(chrono::ChVector3d(0, 0, 0));
        body2->SetPos(chrono::ChVector3d(1, 0, 0));
        body2->SetPosDt(chrono::ChVector3d(0, 0.5, 0));
        system.Add(body1);
        system.Add(body2);

        auto rsda = chrono_types::make_shared<chrono::ChLinkRSDA>();
        rsda->Initialize(body1, body2, chrono::ChFramed(chrono::ChVector3d(0, 0, 0)));
        rsda->SetRestAngle(0.1);
        system.AddLink(rsda);

        auto backend = std::make_unique<CaptureBackend>();
        CaptureBackend* raw = backend.get();
        auto model = std::make_shared<seastack::external::ExternalPtoModel>(
            std::move(backend));
        model->SetLinkKind(seastack::external::ExternalPtoLinkKind::Rsda);
        model->EnableRichState(true);
        model->Initialize(0.01);
        rsda->RegisterTorqueFunctor(
            std::make_shared<seastack::chrono::ExternalPtoTorqueFunctor>(model));

        auto fun = rsda->GetTorqueFunctor();
        TEST_ASSERT(fun != nullptr, "RSDA torque functor attached");
        const double angle = 0.4;
        const double rest = 0.1;
        const double omega = -0.8;
        const double T = fun->evaluate(0.0, rest, angle, omega, *rsda);
        TEST_NEAR(T, -7.0 * omega, tol, "RSDA torque = -7*omega");
        TEST_ASSERT(raw->last_in_.size() == 17, "RSDA packed 17");
        TEST_NEAR(raw->last_in_[0], angle - rest, tol, "RSDA disp=angle-rest");
        TEST_NEAR(raw->last_in_[1], omega, tol, "RSDA vel");
        TEST_NEAR(raw->last_in_[2], angle, tol, "RSDA angle");
        TEST_NEAR(raw->last_in_[3], rest, tol, "RSDA rest_angle");
        TEST_NEAR(raw->last_in_[11], 1.0, tol, "body2_pos_x");
        TEST_NEAR(raw->last_in_[15], 0.5, tol, "body2_vel_y");
    }

    // Lean PTOTorqueFunctor smoke
    {
        class LeanPto : public seastack::pto::IPTOModel {
          public:
            double ComputeForce(double /*d*/, double v, double /*t*/) override {
                return -3.0 * v;
            }
        };
        auto model = std::make_shared<LeanPto>();
        seastack::chrono::PTOTorqueFunctor fun(model);
        chrono::ChSystemNSC system;
        auto b1 = chrono_types::make_shared<chrono::ChBody>();
        auto b2 = chrono_types::make_shared<chrono::ChBody>();
        system.Add(b1);
        system.Add(b2);
        auto rsda = chrono_types::make_shared<chrono::ChLinkRSDA>();
        rsda->Initialize(b1, b2, chrono::ChFramed());
        system.AddLink(rsda);
        TEST_NEAR(fun.evaluate(0.0, 0.0, 0.2, 4.0, *rsda), -12.0, tol,
                  "lean PTOTorqueFunctor");
    }

    // Combined native spring-damper + lean PTO (Chrono LinearSpringDamperForce sign).
    {
        class LeanPto : public seastack::pto::IPTOModel {
          public:
            double ComputeForce(double /*d*/, double v, double /*t*/) override {
                return -5.0 * v;  // external contribution
            }
        };
        seastack::chrono::NativeSpringDamper native;
        native.k = 100.0;
        native.c = 2.0;
        native.preload = 3.0;
        auto model = std::make_shared<LeanPto>();
        seastack::chrono::PTOForceFunctor fun(model, native);
        chrono::ChSystemNSC system;
        auto b1 = chrono_types::make_shared<chrono::ChBody>();
        auto b2 = chrono_types::make_shared<chrono::ChBody>();
        system.Add(b1);
        system.Add(b2);
        auto tsda = chrono_types::make_shared<chrono::ChLinkTSDA>();
        tsda->Initialize(b1, b2, false, chrono::ChVector3d(0, 0, 0),
                         chrono::ChVector3d(0, 0, 1));
        system.AddLink(tsda);

        const double rest = 0.5;
        const double length = 0.8;  // displacement = 0.3
        const double vel = 1.5;
        const double F = fun.evaluate(0.0, rest, length, vel, *tsda);
        // external = -5*1.5 = -7.5
        // native   = 3 - 100*0.3 - 2*1.5 = 3 - 30 - 3 = -30
        TEST_NEAR(F, -37.5, tol, "combined PTOForceFunctor = external + native");

        // Default native zeros: exact no-op on existing path.
        seastack::chrono::PTOForceFunctor fun0(model);
        TEST_NEAR(fun0.evaluate(0.0, rest, length, vel, *tsda), -7.5, tol,
                  "default native zeros leave external-only force");
    }

    // Combined on rich ExternalPtoForceFunctor
    {
        chrono::ChSystemNSC system;
        auto body1 = chrono_types::make_shared<chrono::ChBody>();
        auto body2 = chrono_types::make_shared<chrono::ChBody>();
        body1->SetFixed(true);
        system.Add(body1);
        system.Add(body2);
        auto tsda = chrono_types::make_shared<chrono::ChLinkTSDA>();
        tsda->Initialize(body1, body2, false, chrono::ChVector3d(0, 0, 0),
                         chrono::ChVector3d(0, 0, 1.0));
        tsda->SetRestLength(0.0);
        system.AddLink(tsda);

        auto backend = std::make_unique<CaptureBackend>();
        auto model = std::make_shared<seastack::external::ExternalPtoModel>(
            std::move(backend));
        model->SetLinkKind(seastack::external::ExternalPtoLinkKind::Tsda);
        model->EnableRichState(true);
        model->Initialize(0.01);

        seastack::chrono::NativeSpringDamper native;
        native.c = 4.0;
        auto fun =
            std::make_shared<seastack::chrono::ExternalPtoForceFunctor>(model,
                                                                       native);
        const double length = 1.0;
        const double vel = 2.0;
        // CaptureBackend returns -7*vel = -14; native = -4*2 = -8
        TEST_NEAR(fun->evaluate(0.0, 0.0, length, vel, *tsda), -22.0, tol,
                  "combined ExternalPtoForceFunctor");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
