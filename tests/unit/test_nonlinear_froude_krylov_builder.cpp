/*********************************************************************
 * @file  test_nonlinear_froude_krylov_builder.cpp
 * @brief HydroModelBuilder wiring for nonlinear Froude-Krylov (RM3 data).
 *
 * Pass criteria from docs/theory/NONLINEAR_FROUDE_KRYLOV.md, sections 7 and 9:
 *   U9  per-body toggle: float with NL FK, spar linear. Forces on the spar
 *       are bit-identical to an all-linear model.
 *   Build-time checks: NL FK needs nonlinear hydrostatics, excitation in
 *   waves, and a wave with discrete components. The ramp shape follows the
 *   resolved excitation method (cosine for frequency domain, linear for IRF).
 *
 * Reads rm3.h5 and float.obj from the repository; no Chrono.
 *********************************************************************/

#include <seastack/hydro/force_components/nonlinear_froude_krylov_component.h>
#include <seastack/hydro/hydro_model_builder.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/eta_table_wave_field.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro_io/h5_reader.h>

#include "test_macros.h"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#ifndef SEASTACK_TEST_RM3_H5_FILE
#error "SEASTACK_TEST_RM3_H5_FILE must be defined"
#endif
#ifndef SEASTACK_TEST_RM3_FLOAT_OBJ
#error "SEASTACK_TEST_RM3_FLOAT_OBJ must be defined"
#endif

using seastack::hydro::FroudeKrylovModel;
using seastack::hydro::FroudeKrylovPressureModel;
using seastack::hydro::HydroComponentType;
using seastack::hydro::HydroData;
using seastack::hydro::HydroModel;
using seastack::hydro::HydroModelBuilder;
using seastack::hydro::HydrostaticsModel;
using seastack::hydro::NonlinearFroudeKrylovComponent;
using seastack::hydro::SystemState;
using seastack::hydro::WaveBase;

namespace {

using HsConfig = HydroModelBuilder::BodyHydrostaticsConfig;

HydroData LoadRm3() { return seastack::hydro_io::H5FileInfo(SEASTACK_TEST_RM3_H5_FILE, 2).ReadH5Data(); }

std::shared_ptr<WaveBase> MakeWave(const std::string& type) {
    seastack::hydro::SeaStateDefinition def;
    def.type = type;
    if (type == "regular") {
        def.amplitude = 1.0;
        def.omega     = 2.0 * std::acos(-1.0) / 8.0;
    } else {
        seastack::hydro::SeaStatePartition p;
        p.spectrum.type = "jonswap";
        p.spectrum.Hs   = 2.0;
        p.spectrum.Tp   = 8.0;
        def.partitions.push_back(p);
        def.n_omega = 32;
    }
    auto wave = std::make_shared<seastack::hydro::LinearDirectionalWaveField>(
        seastack::hydro::ComponentSampler::Build(def), def.depth);
    wave->SetNumBodies(2);
    return wave;
}

HsConfig Linear() { return HsConfig{}; }

HsConfig NonlinearFk(FroudeKrylovPressureModel pressure = FroudeKrylovPressureModel::kAiryWheeler) {
    HsConfig c;
    c.model         = HydrostaticsModel::kNonlinear;
    c.mesh_file     = SEASTACK_TEST_RM3_FLOAT_OBJ;
    c.froude_krylov = FroudeKrylovModel::kNonlinear;
    c.fk_pressure   = pressure;
    return c;
}

HydroModel Build(std::shared_ptr<WaveBase> wave, std::vector<HsConfig> hs, bool excitation = true) {
    HydroModelBuilder builder;
    builder.FromHydroData(LoadRm3()).WithWave(std::move(wave)).WithBodyHydrostatics(std::move(hs)).EnableRadiation();
    builder.WithRampDuration(20.0);
    if (excitation) builder.EnableExcitation();
    return builder.Build();
}

std::string ThrownMessage(const std::function<void()>& fn) {
    try {
        fn();
    } catch (const std::exception& e) {
        return e.what();
    }
    return "";
}

bool Contains(const std::string& s, const std::string& sub) { return s.find(sub) != std::string::npos; }

const NonlinearFroudeKrylovComponent* FindFk(const HydroModel& model) {
    for (const auto& comp : model.GetForces().GetComponents()) {
        if (comp->Type() == HydroComponentType::kNonlinearFroudeKrylov) {
            return dynamic_cast<const NonlinearFroudeKrylovComponent*>(comp.get());
        }
    }
    return nullptr;
}

/// Both bodies near their BEM CG, slightly displaced, at rest.
SystemState Rm3State(const HydroData& data) {
    SystemState state;
    state.bodies.resize(2);
    for (int b = 0; b < 2; ++b) {
        state.bodies[b].position = data.GetCGVector(b).head<3>() + Eigen::Vector3d(0.05, -0.02, -0.3 + 0.1 * b);
        state.bodies[b].orientation_rpy = Eigen::Vector3d(0.01, -0.02, 0.0);
    }
    return state;
}

}  // namespace

int main() {
    TestResults test_results;
    const SystemState state = Rm3State(LoadRm3());

    // U9: per-body toggle. Spar forces bit-identical to the all-linear model.
    for (const std::string type : {"regular", "irregular"}) {
        HydroModel mixed = Build(MakeWave(type), {NonlinearFk(), Linear()});
        HydroModel linear = Build(MakeWave(type), {Linear(), Linear()});
        for (double t : {5.0, 31.0}) {
            const auto f_mixed = mixed.Evaluate(state, t);
            const auto f_linear = linear.Evaluate(state, t);
            const std::string tag = " (" + type + ", t = " + std::to_string(t) + ")";
            TEST_ASSERT(f_mixed[1].force == f_linear[1].force && f_mixed[1].moment == f_linear[1].moment,
                        "U9 spar forces bit-identical to all-linear" + tag);
            TEST_ASSERT((f_mixed[0].force - f_linear[0].force).norm() > 1.0, "U9 float forces differ" + tag);
        }
        const auto* fk = FindFk(mixed);
        TEST_ASSERT(fk != nullptr, "U9 NL FK component built (" + type + ")");
        TEST_ASSERT(FindFk(linear) == nullptr, "U9 no NL FK component in the all-linear model (" + type + ")");
        if (fk) {
            // Ramp follows the resolved excitation: regular -> frequency domain
            // (cosine), long-crested irregular -> IRF convolution (linear).
            const double expected = (type == "regular") ? 0.5 * (1.0 - std::cos(std::acos(-1.0) * 0.25)) : 0.25;
            TEST_NEAR(fk->RampFactor(5.0), expected, 1e-15, "ramp shape matches excitation (" + type + ")");
            TEST_NEAR(fk->GetLastWettedArea(1), 0.0, 0.0, "spar has no NL FK diagnostics (" + type + ")");
        }
    }

    // Still water: NL FK builds (no excitation needed) and reduces to NL hydrostatics.
    {
        std::string msg = ThrownMessage([] {
            Build(std::make_shared<seastack::hydro::NoWave>(), {NonlinearFk(), Linear()}, false);
        });
        TEST_ASSERT(msg.empty(), "still water without excitation builds: " + msg);
    }

    // Build-time errors.
    {
        HsConfig fk_on_linear = Linear();
        fk_on_linear.froude_krylov = FroudeKrylovModel::kNonlinear;
        std::string msg = ThrownMessage([&] { Build(MakeWave("regular"), {fk_on_linear, Linear()}); });
        TEST_ASSERT(Contains(msg, "hydrostatics = linear") && Contains(msg, "same instantaneous wetted surface"),
                    "NL FK with linear hydrostatics rejected");

        msg = ThrownMessage([] { Build(MakeWave("regular"), {NonlinearFk(), Linear()}, false); });
        TEST_ASSERT(Contains(msg, "needs wave excitation enabled"), "NL FK in waves without excitation rejected");

        HsConfig no_mesh = NonlinearFk();
        no_mesh.mesh_file.clear();
        msg = ThrownMessage([&] { Build(MakeWave("regular"), {no_mesh, Linear()}); });
        TEST_ASSERT(Contains(msg, "no mesh_file"), "NL FK without a mesh rejected");

        auto table = std::make_shared<seastack::hydro::EtaTableWaveField>(
            std::vector<double>{0.0, 1.0, 2.0}, std::vector<double>{0.0, 0.1, 0.0}, 0.0);
        msg = ThrownMessage([&] { Build(table, {NonlinearFk(), Linear()}); });
        TEST_ASSERT(Contains(msg, "discrete components"), "NL FK with an eta-table wave rejected");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
