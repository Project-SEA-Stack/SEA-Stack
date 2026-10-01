/*********************************************************************
 * @file  test_radiation_constant_coefficients.cpp
 * @brief Unit tests for constant-coefficient (regular-wave) radiation.
 *
 * Covers:
 *   1. H5 reader: A(w) = rho*A_file and B(w) = rho*w_k*B_file (reference
 *      values computed independently with h5py from the bundled assets).
 *   2. Full coupled 6N x 6N matrices (RM3 body-to-body terms retained).
 *   3. MatchBemFrequency: equivalent grid point accepted; off-grid,
 *      out-of-range and ambiguous requests rejected.
 *   4. HydroModelBuilder validation (non-regular waves, RIRF-only settings,
 *      off-grid frequency) and dispatch to the B(w) component.
 *   5. RadiationConstantCoefficientComponent: F = -B(w) * v with coupling.
 *   6. Defaults unchanged (RIRF convolution remains the default).
 *   7. Chrono added-mass load: system mass matrix contains A(w) (not A_inf,
 *      not A_inf + A(w)) for constant coefficients, A_inf otherwise; exactly
 *      one ChLoadHydrodynamics is registered.
 *********************************************************************/

#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/config/hydro_config.h>
#include <seastack/hydro/force_components/radiation_cc_component.h>
#include <seastack/hydro/hydro_model_builder.h>
#include <seastack/hydro/radiation/bem_frequency_match.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro_io/h5_reader.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLoadHydrodynamics.h>
#include <chrono/physics/ChSystemNSC.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef SEASTACK_TEST_SPHERE_H5_FILE
#error "SEASTACK_TEST_SPHERE_H5_FILE must be defined"
#endif
#ifndef SEASTACK_TEST_RM3_H5_FILE
#error "SEASTACK_TEST_RM3_H5_FILE must be defined"
#endif

namespace {

using seastack::hydro::BemFrequencyMatch;
using seastack::hydro::HydroData;
using seastack::hydro::HydroModelBuilder;
using seastack::hydro::MatchBemFrequency;
using seastack::hydro::RadiationMethod;

constexpr double kTwoPi = 6.283185307179586;

int g_failures = 0;

void Check(bool condition, const std::string& what) {
    if (condition) {
        std::cout << "  PASS: " << what << "\n";
    } else {
        std::cout << "  FAIL: " << what << "\n";
        ++g_failures;
    }
}

bool RelClose(double actual, double expected, double rel_tol) {
    return std::abs(actual - expected) <= rel_tol * std::abs(expected);
}

/// Returns the exception message from `fn`, or "" if nothing was thrown.
std::string ThrownMessage(const std::function<void()>& fn) {
    try {
        fn();
    } catch (const std::exception& e) {
        return e.what();
    }
    return "";
}

bool Contains(const std::string& s, const std::string& sub) {
    return s.find(sub) != std::string::npos;
}

HydroData LoadSphere() {
    return seastack::hydro_io::H5FileInfo(SEASTACK_TEST_SPHERE_H5_FILE, 1).ReadH5Data();
}

HydroData LoadRm3() {
    return seastack::hydro_io::H5FileInfo(SEASTACK_TEST_RM3_H5_FILE, 2).ReadH5Data();
}

std::shared_ptr<seastack::hydro::LinearDirectionalWaveField> MakeRegularWave(double omega,
                                                                             int num_bodies) {
    seastack::hydro::SeaStateDefinition def;
    def.type = "regular";
    def.amplitude = 1.0;
    def.omega = omega;
    auto wave = std::make_shared<seastack::hydro::LinearDirectionalWaveField>(
        seastack::hydro::ComponentSampler::Build(def), def.depth);
    wave->SetNumBodies(static_cast<unsigned int>(num_bodies));
    return wave;
}

std::shared_ptr<seastack::hydro::LinearDirectionalWaveField> MakeIrregularWave(int num_bodies) {
    seastack::hydro::SeaStateDefinition def;
    def.type = "irregular";
    seastack::hydro::SeaStatePartition p;
    p.spectrum.type = "jonswap";
    p.spectrum.Hs = 2.0;
    p.spectrum.Tp = 8.0;
    def.partitions.push_back(p);
    def.n_omega = 64;
    auto wave = std::make_shared<seastack::hydro::LinearDirectionalWaveField>(
        seastack::hydro::ComponentSampler::Build(def), def.depth);
    wave->SetNumBodies(static_cast<unsigned int>(num_bodies));
    return wave;
}

// RM3 BEM grid index 39: w = 0.7999997589986925 rad/s (T = 7.853984 s).
constexpr int kRm3Index = 39;
constexpr double kRm3Omega = 0.7999997589986925;

// ── 1-2. H5 reader scaling and coupling ─────────────────────────────

void TestReaderScaling() {
    std::cout << "\n[1] H5 reader: A(w) and B(w) dimensional scaling\n";
    const HydroData sphere = LoadSphere();
    Check(sphere.HasFrequencyDomainRadiation(), "sphere: frequency-domain radiation present");
    const Eigen::VectorXd w = sphere.GetBemFrequencies();
    Check(w.size() == 240, "sphere: 240 BEM frequencies");
    Check(w[19] == 1.000000048889152, "sphere: w[19] read exactly");

    const Eigen::MatrixXd a = sphere.GetAddedMassAtFrequencyIndex(19);
    const Eigen::MatrixXd b = sphere.GetRadiationDampingAtFrequencyIndex(19);
    Check(a.rows() == 6 && a.cols() == 6 && b.rows() == 6 && b.cols() == 6, "sphere: 6x6 matrices");
    // Reference: rho * A_file and rho * w_k * B_file from h5py.
    Check(RelClose(a(2, 2), 152080.6, 1e-12), "sphere: A33(w=1.0) = rho*A_file = 152080.6 kg");
    Check(RelClose(b(2, 2), 88539.46432861913, 1e-12),
          "sphere: B33(w=1.0) = rho*w_k*B_file = 88539.464 N.s/m");
    Check(RelClose(a(0, 4), 338119.4, 1e-12), "sphere: A15 surge-pitch coupling retained");
    Check(RelClose(b(0, 0), 27063.621323117433, 1e-12), "sphere: B11 scaled by rho*w_k");
    Check(!RelClose(a(2, 2), sphere.GetInfAddedMassMatrix(0)(2, 2), 1e-3),
          "sphere: A33(w) differs from A33_inf (130876.8 kg)");
}

void TestCoupledMatrices() {
    std::cout << "\n[2] Full coupled 12x12 matrices (RM3)\n";
    const HydroData rm3 = LoadRm3();
    Check(rm3.HasFrequencyDomainRadiation(), "rm3: frequency-domain radiation present");
    Check(rm3.GetBemFrequencies()[kRm3Index] == kRm3Omega, "rm3: w[39] read exactly");
    const Eigen::MatrixXd a = rm3.GetAddedMassAtFrequencyIndex(kRm3Index);
    const Eigen::MatrixXd b = rm3.GetRadiationDampingAtFrequencyIndex(kRm3Index);
    Check(a.rows() == 12 && a.cols() == 12 && b.rows() == 12 && b.cols() == 12, "rm3: 12x12 matrices");
    Check(RelClose(a(2, 2), 1426477.0, 1e-12), "rm3: A(float heave, float heave)");
    Check(RelClose(a(2, 8), -148546.7, 1e-12), "rm3: A(float heave, plate heave) body coupling");
    Check(RelClose(a(8, 2), -148817.3, 1e-12), "rm3: A(plate heave, float heave) body coupling");
    Check(RelClose(a(8, 8), 8901439.0, 1e-12), "rm3: A(plate heave, plate heave)");
    Check(RelClose(b(2, 8), -267685.5993592514, 1e-12), "rm3: B(float heave, plate heave) body coupling");
    Check(RelClose(b(8, 2), -268430.31913490326, 1e-12), "rm3: B(plate heave, float heave) body coupling");
    Check(RelClose(a(0, 4), 1745693.0, 1e-12), "rm3: A(float surge, float pitch) off-diagonal");
    Check(RelClose(b(4, 0), 287334.39344001823, 1e-12), "rm3: B(float pitch, float surge) off-diagonal");
}

// ── 3. Frequency matching ───────────────────────────────────────────

void TestFrequencyMatching() {
    std::cout << "\n[3] MatchBemFrequency\n";
    const Eigen::VectorXd w = LoadSphere().GetBemFrequencies();  // 0.05:0.05:12 rad/s (+~5e-7 noise)

    BemFrequencyMatch m = MatchBemFrequency(w, 1.0, 1e-4);
    Check(m.index == 19 && m.omega_matched == w[19], "nominal w = 1.0 matches grid index 19");
    Check(m.relative_difference < 1e-7, "relative difference ~5e-8 reported");

    m = MatchBemFrequency(w, kTwoPi / 6.283185, 1e-4);
    Check(m.index == 19, "period from H5 T column (7 digits) matches index 19");

    m = MatchBemFrequency(w, 1.0 * (1.0 + 5e-5), 1e-4);
    Check(m.index == 19, "5e-5 relative offset accepted at default tolerance");

    std::string msg = ThrownMessage([&] { MatchBemFrequency(w, 1.0 * (1.0 + 5e-5), 1e-6); });
    Check(Contains(msg, "has no matching hydrodynamic coefficient"),
          "5e-5 relative offset rejected at 1e-6 tolerance");

    msg = ThrownMessage([&] { MatchBemFrequency(w, 1.025, 1e-4); });
    Check(Contains(msg, "has no matching hydrodynamic coefficient"), "between grid points rejected");
    Check(Contains(msg, "Nearest available: T = "), "error lists nearest grid point");
    Check(Contains(msg, "Matching tolerance"), "error lists tolerance");
    Check(Contains(msg, "Available periods (240)"), "error lists available periods");

    msg = ThrownMessage([&] { MatchBemFrequency(w, 0.01, 1e-4); });
    Check(Contains(msg, "outside the BEM frequency range"), "below grid rejected (no extrapolation)");
    msg = ThrownMessage([&] { MatchBemFrequency(w, 13.0, 1e-4); });
    Check(Contains(msg, "outside the BEM frequency range"), "above grid rejected (no extrapolation)");

    msg = ThrownMessage([&] { MatchBemFrequency(w, 1.0, 0.06); });
    Check(Contains(msg, "within the matching tolerance"), "ambiguous (tolerance > grid spacing) rejected");

    msg = ThrownMessage([&] { MatchBemFrequency(w, 1.0, 0.0); });
    Check(Contains(msg, "frequency_tolerance must be finite and > 0"), "non-positive tolerance rejected");
}

// ── 4. Builder validation and dispatch ──────────────────────────────

HydroModelBuilder Rm3CcBuilder(std::shared_ptr<seastack::hydro::WaveBase> wave) {
    HydroModelBuilder builder;
    builder.FromHydroData(LoadRm3())
        .WithWave(std::move(wave))
        .EnableHydrostatics()
        .EnableRadiation()
        .EnableExcitation()
        .WithRadiationMethod(RadiationMethod::kConstantCoefficients);
    return builder;
}

void TestBuilder() {
    std::cout << "\n[4] HydroModelBuilder validation and dispatch\n";

    std::string msg = ThrownMessage([] {
        Rm3CcBuilder(std::make_shared<seastack::hydro::NoWave>()).Build();
    });
    Check(Contains(msg, "only valid for a monochromatic regular wave") && Contains(msg, "no wave"),
          "no wave (free decay) rejected");

    msg = ThrownMessage([] { Rm3CcBuilder(MakeIrregularWave(2)).Build(); });
    Check(Contains(msg, "only valid for a monochromatic regular wave") && Contains(msg, "irregular"),
          "irregular wave rejected");

    msg = ThrownMessage([] {
        auto b = Rm3CcBuilder(MakeRegularWave(kRm3Omega, 2));
        b.WithRadiationOptions(seastack::hydro::RadiationKernelProcessing::DefaultSmoothing());
        b.Build();
    });
    Check(Contains(msg, "smoothing/tapering"), "smoothing/taper rejected");

    msg = ThrownMessage([] {
        auto b = Rm3CcBuilder(MakeRegularWave(kRm3Omega, 2));
        b.WithRadiationTruncationTime(20.0);
        b.Build();
    });
    Check(Contains(msg, "truncation_time"), "radiation truncation rejected");

    msg = ThrownMessage([] { Rm3CcBuilder(MakeRegularWave(kTwoPi / 8.0, 2)).Build(); });
    Check(Contains(msg, "has no matching hydrodynamic coefficient"),
          "T = 8 s (w = 0.785, between RM3 grid points 0.78/0.80) rejected");

    seastack::hydro::HydroModel model = Rm3CcBuilder(MakeRegularWave(kTwoPi / 7.853982, 2)).Build();
    const seastack::hydro::RadiationConstantCoefficientComponent* cc = nullptr;
    int n_radiation = 0;
    for (const auto& comp : model.GetForces().GetComponents()) {
        if (comp->Type() == seastack::hydro::HydroComponentType::kRadiation) {
            ++n_radiation;
            cc = dynamic_cast<const seastack::hydro::RadiationConstantCoefficientComponent*>(comp.get());
        }
    }
    Check(n_radiation == 1 && cc != nullptr, "on-grid regular wave builds one B(w) radiation component");
    if (cc) {
        const Eigen::MatrixXd expected = LoadRm3().GetRadiationDampingAtFrequencyIndex(kRm3Index);
        Check((cc->radiation_damping() - expected).norm() == 0.0,
              "component holds the full coupled B(w_39) matrix");
    }

    // Existing methods unaffected: RIRF build on the same regular wave.
    HydroModelBuilder rirf;
    rirf.FromHydroData(LoadRm3())
        .WithWave(MakeRegularWave(kTwoPi / 8.0, 2))
        .EnableHydrostatics()
        .EnableRadiation()
        .EnableExcitation();
    msg = ThrownMessage([&] { rirf.Build(); });
    Check(msg.empty(), "default (RIRF) build at an off-grid period still succeeds");
}

// ── 5. Force component ──────────────────────────────────────────────

void TestComponent() {
    std::cout << "\n[5] RadiationConstantCoefficientComponent: F = -B v\n";
    const Eigen::MatrixXd b = LoadRm3().GetRadiationDampingAtFrequencyIndex(kRm3Index);
    seastack::hydro::RadiationConstantCoefficientComponent comp(2, b);

    seastack::hydro::SystemState state;
    state.bodies.resize(2);
    state.bodies[0].linear_velocity = Eigen::Vector3d(0.1, -0.2, 0.7);
    state.bodies[0].angular_velocity = Eigen::Vector3d(0.01, 0.03, -0.02);
    state.bodies[1].linear_velocity = Eigen::Vector3d(-0.05, 0.04, -0.3);
    state.bodies[1].angular_velocity = Eigen::Vector3d(0.002, -0.004, 0.001);
    Eigen::VectorXd v(12);
    v << 0.1, -0.2, 0.7, 0.01, 0.03, -0.02, -0.05, 0.04, -0.3, 0.002, -0.004, 0.001;

    seastack::hydro::BodyForces forces(2);
    comp.Compute(state, 0.0, forces);
    const Eigen::VectorXd expected = -b * v;
    double err = 0.0;
    for (int bi = 0; bi < 2; ++bi) {
        for (int d = 0; d < 3; ++d) {
            err = std::max(err, std::abs(forces[bi].force[d] - expected(6 * bi + d)));
            err = std::max(err, std::abs(forces[bi].moment[d] - expected(6 * bi + 3 + d)));
        }
    }
    Check(err < 1e-9 * expected.norm(), "force equals -B(w) * v including cross-body terms");

    // Plate-only velocity must load the float through B(float, plate).
    seastack::hydro::SystemState plate_only;
    plate_only.bodies.resize(2);
    plate_only.bodies[1].linear_velocity = Eigen::Vector3d(0.0, 0.0, 1.0);
    seastack::hydro::BodyForces f2(2);
    comp.Compute(plate_only, 0.0, f2);
    Check(RelClose(f2[0].force[2], -b(2, 8), 1e-12), "plate heave velocity loads float heave via B(2,8)");

    std::string msg = ThrownMessage([] {
        seastack::hydro::RadiationConstantCoefficientComponent bad(2, Eigen::MatrixXd::Zero(6, 6));
    });
    Check(Contains(msg, "expected 12x12"), "wrong matrix size rejected");
}

// ── 6. Defaults ─────────────────────────────────────────────────────

void TestDefaults() {
    std::cout << "\n[6] Defaults unchanged\n";
    seastack::chrono::HydroSystemConfig cfg;
    Check(cfg.radiation_method == RadiationMethod::kRirfConvolution, "HydroSystemConfig default = RIRF");
    seastack::hydro::YAMLHydroData yaml;
    Check(yaml.radiation_method == "rirf_convolution", "YAML default radiation method = rirf_convolution");
    Check(yaml.radiation_frequency_tolerance == 1e-4, "YAML default frequency_tolerance = 1e-4");
    Check(seastack::hydro::ConstantCoefficientOptions{}.frequency_tolerance == 1e-4,
          "ConstantCoefficientOptions default = 1e-4");
    Check(static_cast<int>(RadiationMethod::kRirfConvolution) == 0 &&
              static_cast<int>(RadiationMethod::kStateSpace) == 1,
          "existing enumerator values unchanged");
}

// ── 7. Chrono added-mass load ───────────────────────────────────────

struct Rm3Chrono {
    chrono::ChSystemNSC sys;
    std::vector<std::shared_ptr<chrono::ChBody>> bodies;
    std::unique_ptr<seastack::chrono::HydroSystem> hydro;
};

std::unique_ptr<Rm3Chrono> MakeRm3Chrono(std::shared_ptr<seastack::hydro::WaveBase> wave,
                                         bool attach_force_callbacks) {
    auto s = std::make_unique<Rm3Chrono>();
    s->sys.SetGravitationalAcceleration(chrono::ChVector3d(0, 0, -9.81));
    s->sys.SetSolverType(chrono::ChSolver::Type::GMRES);
    const double masses[2] = {725834.0, 886691.0};
    const chrono::ChVector3d inertias[2] = {{20907301.0, 21306090.66, 37085481.11},
                                            {94419614.57, 94407091.24, 28542224.82}};
    const double z[2] = {-0.72, -21.29};
    for (int i = 0; i < 2; ++i) {
        auto b = chrono_types::make_shared<chrono::ChBody>();
        b->SetName("body" + std::to_string(i + 1));
        b->SetMass(masses[i]);
        b->SetInertiaXX(inertias[i]);
        b->SetPos(chrono::ChVector3d(0, 0, z[i]));
        s->sys.Add(b);
        s->bodies.push_back(b);
    }
    seastack::chrono::HydroCouplingOptions coupling;
    coupling.attach_hydro_force_callbacks = attach_force_callbacks;
    s->hydro = std::make_unique<seastack::chrono::HydroSystem>(
        s->bodies, SEASTACK_TEST_RM3_H5_FILE, std::move(wave), coupling);
    return s;
}

/// System mass matrix minus rigid-body inertia after one tiny step.
Eigen::MatrixXd AddedMassInSystem(Rm3Chrono& s) {
    s.sys.DoStepDynamics(1e-5);
    chrono::ChSparseMatrix m_sparse;
    s.sys.GetMassMatrix(m_sparse);
    Eigen::MatrixXd m = Eigen::MatrixXd(m_sparse);
    for (const auto& b : s.bodies) {
        const int off = static_cast<int>(b->GetOffset_w());
        for (int d = 0; d < 3; ++d) m(off + d, off + d) -= b->GetMass();
        const auto inertia = b->GetInertiaXX();
        m(off + 3, off + 3) -= inertia.x();
        m(off + 4, off + 4) -= inertia.y();
        m(off + 5, off + 5) -= inertia.z();
    }
    return m.topLeftCorner(12, 12);
}

int CountAddedMassLoads(Rm3Chrono& s) {
    int n = 0;
    for (const auto& item : s.sys.GetOtherPhysicsItems()) {
        if (std::dynamic_pointer_cast<chrono::ChLoadHydrodynamics>(item)) ++n;
    }
    return n;
}

void TestChronoAddedMass() {
    std::cout << "\n[7] Chrono added-mass load: A(w) replaces A_inf\n";
    const HydroData rm3 = LoadRm3();
    Eigen::MatrixXd a_inf(12, 12);
    a_inf << rm3.GetInfAddedMassMatrix(0), rm3.GetInfAddedMassMatrix(1);
    const Eigen::MatrixXd a_w = rm3.GetAddedMassAtFrequencyIndex(kRm3Index);
    const double scale = a_w.norm();

    auto s = MakeRm3Chrono(MakeRegularWave(kRm3Omega, 2), false);
    Eigen::MatrixXd added = AddedMassInSystem(*s);
    Check((added - a_inf).norm() < 1e-9 * scale, "default (RIRF): system added mass == A_inf");
    Check(CountAddedMassLoads(*s) == 1, "default: exactly one ChLoadHydrodynamics");

    s->hydro->SetRadiationMethod(RadiationMethod::kConstantCoefficients);
    added = AddedMassInSystem(*s);
    Check((added - a_w).norm() < 1e-9 * scale, "constant_coefficients: system added mass == A(w)");
    Check((added - a_inf).norm() > 1e-3 * scale, "constant_coefficients: system added mass != A_inf");
    Check((added - (a_w + a_inf)).norm() > 0.5 * a_inf.norm(),
          "constant_coefficients: no double counting (!= A(w) + A_inf)");
    Check(std::abs(added(2, 8) - a_w(2, 8)) < 1e-9 * scale, "body-to-body A(w) block present in system mass");
    Check(CountAddedMassLoads(*s) == 1, "constant_coefficients: exactly one ChLoadHydrodynamics");

    s->hydro->SetRadiationMethod(RadiationMethod::kStateSpace);
    added = AddedMassInSystem(*s);
    Check((added - a_inf).norm() < 1e-9 * scale, "switch back to state_space restores A_inf");
    Check(CountAddedMassLoads(*s) == 1, "after switch back: exactly one ChLoadHydrodynamics");

    // Order independence: method first, then the regular wave.
    auto s2 = MakeRm3Chrono(std::make_shared<seastack::hydro::NoWave>(), false);
    s2->hydro->SetRadiationMethod(RadiationMethod::kConstantCoefficients);
    Check((AddedMassInSystem(*s2) - a_inf).norm() < 1e-9 * scale,
          "constant_coefficients + no wave yet: A_inf kept");
    s2->hydro->AddWaves(MakeRegularWave(kRm3Omega, 2));
    Check((AddedMassInSystem(*s2) - a_w).norm() < 1e-9 * scale,
          "AddWaves(regular) after SetRadiationMethod: A(w) applied");

    // Off-grid wave: A_inf kept, and the first force evaluation fails clearly.
    auto s3 = MakeRm3Chrono(MakeRegularWave(kTwoPi / 8.0, 2), true);
    s3->hydro->SetRadiationMethod(RadiationMethod::kConstantCoefficients);
    std::string msg = ThrownMessage([&] { s3->sys.DoStepDynamics(0.01); });
    Check(Contains(msg, "has no matching hydrodynamic coefficient"),
          "off-grid period: first step throws the frequency-matching error");

    // Irregular wave: rejected at model construction.
    auto s4 = MakeRm3Chrono(MakeIrregularWave(2), true);
    s4->hydro->SetRadiationMethod(RadiationMethod::kConstantCoefficients);
    msg = ThrownMessage([&] { s4->sys.DoStepDynamics(0.01); });
    Check(Contains(msg, "only valid for a monochromatic regular wave"),
          "irregular wave + constant_coefficients rejected");

    // Valid configuration runs (force model built, consistency check passes).
    auto s5 = MakeRm3Chrono(MakeRegularWave(kRm3Omega, 2), true);
    s5->hydro->SetRadiationMethod(RadiationMethod::kConstantCoefficients);
    msg = ThrownMessage([&] {
        for (int i = 0; i < 20; ++i) s5->sys.DoStepDynamics(0.01);
    });
    Check(msg.empty(), "valid constant_coefficients configuration steps without error");
    if (!msg.empty()) std::cout << "    error: " << msg << "\n";
}

}  // namespace

int main() {
    std::cout << "=== test_radiation_constant_coefficients ===\n";
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"reader scaling", TestReaderScaling}, {"coupled matrices", TestCoupledMatrices},
        {"frequency matching", TestFrequencyMatching}, {"builder", TestBuilder},
        {"component", TestComponent}, {"defaults", TestDefaults},
        {"chrono added mass", TestChronoAddedMass}};
    for (const auto& [name, fn] : tests) {
        try {
            fn();
        } catch (const std::exception& e) {
            std::cout << "  FAIL: " << name << " threw: " << e.what() << "\n";
            ++g_failures;
        }
    }
    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED") << " (" << g_failures
              << " failure(s))\n";
    return g_failures == 0 ? 0 : 1;
}
