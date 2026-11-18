// Radiated-wave overlay inputs from BEM files and Chrono bodies (adapter).
//  - B(w) from the RIRF cosine transform matches the stored B(w) (RM3).
//  - Haskind patterns from the excitation carry the power B(w) takes (Wigley,
//    36 headings, B from the RIRF).
//  - Captured velocities are in the heading frame.

#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/adapters/chrono/radiation_viz.h>
#include <seastack/viz_core/radiated_waves.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystemNSC.h>

#include "test_macros.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#ifndef SEASTACK_TEST_RM3_H5_FILE
#error "SEASTACK_TEST_RM3_H5_FILE must be defined"
#endif
#ifndef SEASTACK_TEST_WIGLEY_H5_FILE
#error "SEASTACK_TEST_WIGLEY_H5_FILE must be defined"
#endif

namespace {

struct Model {
    chrono::ChSystemNSC sys;
    std::vector<std::shared_ptr<chrono::ChBody>> bodies;
    std::unique_ptr<seastack::chrono::HydroSystem> hydro;
};

std::unique_ptr<Model> MakeModel(const char* h5_file, int num_bodies) {
    auto m = std::make_unique<Model>();
    for (int i = 0; i < num_bodies; ++i) {
        auto b = chrono_types::make_shared<chrono::ChBody>();
        b->SetName("body" + std::to_string(i + 1));
        b->SetMass(1000.0);
        m->sys.Add(b);
        m->bodies.push_back(b);
    }
    seastack::chrono::HydroCouplingOptions coupling;
    coupling.attach_hydro_force_callbacks = false;
    m->hydro = std::make_unique<seastack::chrono::HydroSystem>(m->bodies, h5_file,
                                                               std::make_shared<seastack::hydro::NoWave>(), coupling);
    return m;
}

}  // namespace

int main() {
    TestResults test_results;
    using seastack::viz::Complex;
    using seastack::viz::RadiatedWaveMethod;

    // RM3: stored B is used; the RIRF transform reproduces it.
    {
        auto m = MakeModel(SEASTACK_TEST_RM3_H5_FILE, 2);
        const auto in = seastack::chrono::MakeRadiationVizInputs(*m->hydro);
        TEST_ASSERT(in.HasDamping() && in.damping_source.find("stored") != std::string::npos, "RM3 uses stored B");
        TEST_ASSERT(in.bodies.size() == 2 && in.bodies[0].name == "body1", "bodies named after Chrono bodies");
        const auto from_k = seastack::chrono::RadiationDampingFromRirf(m->hydro->GetHydroData(), in.omega);
        TEST_ASSERT(from_k.size() == in.omega.size(), "RIRF transform on the BEM grid");
        for (int dof : {2, 0, 8}) {  // body1 heave, body1 surge, body2 heave
            double peak = 0.0;
            for (size_t k = 0; k < in.omega.size(); ++k) {
                peak = std::max(peak, std::abs(in.B(k, dof, dof)));
            }
            // The RM3 RIRF ends at 20 s; its truncation shows above about 4.5 rad/s.
            double worst = 0.0;
            for (size_t k = 0; k < in.omega.size(); ++k) {
                if (in.omega[k] <= 4.0 && std::abs(in.B(k, dof, dof)) > 0.1 * peak) {
                    const double ratio = from_k[k][in.NumDof() * dof + dof] / in.B(k, dof, dof);
                    worst = std::max(worst, std::abs(ratio - 1.0));
                }
            }
            std::cout << "RM3 dof " << dof << ": worst |B_K / B - 1| below 4 rad/s = " << worst << "\n";
            TEST_ASSERT(worst < 0.03, "B from the RIRF matches the stored B where B is significant");
        }
    }

    // Wigley: B from the RIRF; Haskind patterns carry the same power.
    {
        auto m = MakeModel(SEASTACK_TEST_WIGLEY_H5_FILE, 1);
        const auto in = seastack::chrono::MakeRadiationVizInputs(*m->hydro);
        TEST_ASSERT(in.HasDamping() && in.damping_source.find("RIRF") != std::string::npos, "Wigley B from RIRF");
        TEST_ASSERT(in.HasExcitation() && in.headings.size() == 36, "36 excitation headings");
        TEST_ASSERT(seastack::viz::ResolveMethod(RadiatedWaveMethod::kAuto, in) == RadiatedWaveMethod::kHaskind,
                    "Auto picks Haskind");
        const auto model = seastack::viz::BuildHaskindModel(in);
        for (int dof : {2, 0, 1}) {
            double peak = 0.0;
            for (size_t k = 0; k < in.omega.size(); ++k) {
                peak = std::max(peak, in.B(k, dof, dof));
            }
            // This file has a resonance-like feature in B and X near 2.04 rad/s and
            // noisy excitation above 2.1 rad/s (irregular frequencies), so frequency
            // by frequency only below 1.8 rad/s.
            double drawn = 0.0;
            double expected = 0.0;
            double worst_low = 0.0;
            for (size_t k = 0; k < in.omega.size(); ++k) {
                if (in.B(k, dof, dof) > 0.1 * peak) {
                    std::vector<Complex> u(6, Complex(0.0, 0.0));
                    u[dof] = 1.0;
                    const double p = seastack::viz::PatternPower(model, in.rho, k, u);
                    drawn += p;
                    expected += 0.5 * in.B(k, dof, dof);
                    if (in.omega[k] <= 1.8) {
                        worst_low = std::max(worst_low, std::abs(p / (0.5 * in.B(k, dof, dof)) - 1.0));
                    }
                }
            }
            std::cout << "Wigley dof " << dof << ": Haskind power / (B/2) = " << drawn / expected
                      << ", worst below 1.8 rad/s " << worst_low << "\n";
            TEST_NEAR(drawn / expected, 1.0, 0.05, "Haskind power matches B over the band");
            TEST_ASSERT(worst_low < 0.05, "Haskind power matches B at each frequency below 1.8 rad/s");
        }
        for (const std::string& note : model.notes) {
            std::cout << "  " << note << "\n";
        }
    }

    // Heading frame: yawed 90 deg, moving along world +x = heading-frame -y (sway).
    {
        auto m = MakeModel(SEASTACK_TEST_WIGLEY_H5_FILE, 1);
        auto& body = m->bodies[0];
        body->SetPos(chrono::ChVector3d(3.0, 4.0, 0.0));
        body->SetRot(chrono::QuatFromAngleZ(0.5 * 3.14159265358979323846));
        body->SetPosDt(chrono::ChVector3d(1.0, 0.0, 0.0));
        body->SetAngVelParent(chrono::ChVector3d(0.2, 0.0, 0.0));
        const auto s = seastack::chrono::CaptureRadiationSamples(*m->hydro);
        TEST_ASSERT(s.size() == 1, "one sample per hydro body");
        TEST_NEAR(s[0].x, 3.0, 1e-12, "x");
        TEST_NEAR(s[0].y, 4.0, 1e-12, "y");
        TEST_NEAR(s[0].yaw, 0.5 * 3.14159265358979323846, 1e-12, "yaw");
        TEST_NEAR(s[0].velocity[0], 0.0, 1e-12, "surge");
        TEST_NEAR(s[0].velocity[1], -1.0, 1e-12, "sway");
        TEST_NEAR(s[0].velocity[4], -0.2, 1e-12, "world x rotation is heading-frame -pitch");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
