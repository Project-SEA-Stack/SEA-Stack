/**
 * @file test_structure_fea_analytic.cpp
 * @brief StructureSubsystem beams against closed-form Euler-Bernoulli results.
 *
 * Each case builds a Chrono system directly (ground body, optional tip body),
 * attaches a StructureSubsystem from a structure YAML written by the test,
 * and compares a linear static solve (or a short transient) with theory.
 * All cases use an aluminium rectangular hollow section (BOX).
 *
 * Units: SI throughout (m, kg, s, N, Pa).  World Z is up unless the case
 * rotates gravity on purpose.
 *
 *   V1  Cantilever, tip load P:          tip deflection PL^3/(3EI), root
 *       reaction force P and moment PL, reaction sign on the support body.
 *   V2  Cantilever, self-weight w:       tip deflection wL^4/(8EI), root
 *       moment wL^2/2.
 *   V3  Simply supported, self-weight:  midspan 5wL^4/(384EI), end
 *       reactions wL/2 (pin + roller mates).
 *   V4  Cantilever, tip torque T:        twist TL/(GJ), with the default
 *       J = Iyy + Izz and with an explicit torsion_constant.
 *   V5  Orientation: a beam along world Y with y_direction along X, loaded
 *       in both principal planes (strong and weak axes), and a beam in the
 *       XY plane at 53 deg to X.
 *   V6  Mesh convergence of V2 for 1, 2, 4, 8, 16 elements.
 *   V7  First natural frequency of a cantilever, (1.8751^2 / 2pi) *
 *       sqrt(EI / (mu L^4)), from the free response to suddenly applied
 *       self-weight; no hidden damping when rayleigh_beta is omitted; and
 *       the damping ratio that rayleigh_beta produces.
 *   V8  Static load-path check: the mate reactions along gravity equal the
 *       beam weight plus rigid attachments; with gravity off the check
 *       reports that it could not be performed.
 */

#include "app_init.h"
#include "structure_config.h"
#include "structure_subsystem.h"
#include "test_macros.h"

#include <chrono/core/ChQuaternion.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChDirectSolverLS.h>
#include <chrono/timestepper/ChTimestepper.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using ::chrono::ChVector3d;

static TestResults test_results;

namespace {

constexpr double kG = 9.81;          // m/s^2
constexpr double kE = 7.0e10;        // Pa, aluminium
constexpr double kShear = 2.63e10;   // Pa
constexpr double kRho = 2700.0;      // kg/m^3

/// Rectangular hollow section.  `depth` is measured along the beam's
/// y_direction (local y), `width` along local z.
struct Section {
    double width = 0.3;  // m
    double depth = 0.3;  // m
    double wall = 0.01;  // m
    std::optional<double> torsion_constant;  // m^4
    std::optional<double> rayleigh_beta;     // s; omitted from the YAML when empty

    double Area() const { return width * depth - (width - 2 * wall) * (depth - 2 * wall); }
    /// Bending in the local-y direction (about local z): depth cubed.
    double IStrong() const {
        const double wi = width - 2 * wall;
        const double di = depth - 2 * wall;
        return (width * depth * depth * depth - wi * di * di * di) / 12.0;
    }
    /// Bending in the local-z direction (about local y): width cubed.
    double IWeak() const {
        const double wi = width - 2 * wall;
        const double di = depth - 2 * wall;
        return (depth * width * width * width - di * wi * wi * wi) / 12.0;
    }
    /// J used by StructureSubsystem: explicit, or Iyy + Izz by default.
    double J() const { return torsion_constant ? *torsion_constant : IStrong() + IWeak(); }
};

std::string Vec(const ChVector3d& v) {
    std::ostringstream s;
    s << std::setprecision(17) << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
    return s.str();
}

/// One straight beam named `arm`, its mates and any extra YAML lines.
struct BeamCase {
    bool automatic_gravity = false;
    Section section;
    ChVector3d start{0, 0, 1};
    ChVector3d end{6, 0, 1};
    ChVector3d y_direction{0, 0, 1};
    int elements = 4;
    std::string mates;  // YAML list items under `mates:`
    std::string extra;  // YAML lines appended inside `structure:`

    double Length() const { return (end - start).Length(); }

    std::string ToYaml() const {
        std::ostringstream s;
        s << std::setprecision(17);
        s << "structure:\n"
          << "  automatic_gravity: " << (automatic_gravity ? "true" : "false") << "\n"
          << "  materials:\n"
          << "    - { name: alu, density: " << kRho << ", youngs_modulus: " << kE
          << ", shear_modulus: " << kShear << " }\n"
          << "  sections:\n"
          << "    - { name: rhs, type: BOX, material: alu, width: " << section.width
          << ", depth: " << section.depth << ", wall: " << section.wall;
        if (section.torsion_constant) {
            s << ", torsion_constant: " << *section.torsion_constant;
        }
        if (section.rayleigh_beta) {
            s << ", rayleigh_beta: " << *section.rayleigh_beta;
        }
        s << " }\n"
          << "  beams:\n"
          << "    - { name: arm, section: rhs, start: " << Vec(start) << ", end: " << Vec(end)
          << ", elements: " << elements << ", y_direction: " << Vec(y_direction) << " }\n"
          << "  mates:\n"
          << mates << extra;
        return s.str();
    }
};

const char* kClampRoot =
    "    - { name: root, node: arm.first, body: ground, constrain: [true, true, true, true, true, true] }\n";
const char* kClampTip =
    "    - { name: tip, node: arm.last, body: tip, constrain: [true, true, true, true, true, true] }\n";

/// A Chrono system with a fixed `ground` body, an optional free `tip` body at
/// the beam end, and the structure attached.
struct Rig {
    std::unique_ptr<::chrono::ChSystemSMC> system;
    std::shared_ptr<::chrono::ChBody> tip;
    std::shared_ptr<seastack::app::StructureSubsystem> structure;
    seastack::app::StructureSubsystem::StructureFrameSample undeformed;
};

Rig MakeRig(const std::string& name, const BeamCase& c, const ChVector3d& gravity,
            double tip_mass_kg = 0.0, double settle_time_s = std::numeric_limits<double>::infinity()) {
    Rig rig;
    rig.system = std::make_unique<::chrono::ChSystemSMC>();
    rig.system->SetGravitationalAcceleration(gravity);
    auto solver = chrono_types::make_shared<::chrono::ChSolverSparseLU>();
    solver->LockSparsityPattern(true);
    rig.system->SetSolver(solver);

    auto ground = chrono_types::make_shared<::chrono::ChBody>();
    ground->SetName("ground");
    ground->SetFixed(true);
    rig.system->AddBody(ground);

    if (tip_mass_kg > 0.0) {
        rig.tip = chrono_types::make_shared<::chrono::ChBody>();
        rig.tip->SetName("tip");
        rig.tip->SetPos(c.end);
        rig.tip->SetMass(tip_mass_kg);
        rig.tip->SetInertiaXX(ChVector3d(1.0, 1.0, 1.0));
        rig.system->AddBody(rig.tip);
    }

    const fs::path dir = fs::temp_directory_path() / "seastack_test_structure_fea_analytic";
    fs::create_directories(dir);
    const fs::path path = dir / (name + ".structure.yaml");
    {
        std::ofstream out(path);
        out << c.ToYaml();
    }
    rig.structure = std::make_shared<seastack::app::StructureSubsystem>(
        seastack::app::LoadStructureConfigFromYaml(path.string()), settle_time_s);
    rig.structure->Attach(*rig.system);
    rig.undeformed = rig.structure->SampleBeam("arm");
    return rig;
}

/// Displacement of node `i` of beam `arm` since the rig was built [m].
ChVector3d Displacement(const Rig& rig, std::size_t i) {
    const auto now = rig.structure->SampleBeam("arm");
    const auto& p = now.node_positions.at(i);
    const auto& p0 = rig.undeformed.node_positions.at(i);
    return ChVector3d(p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]);
}

/// Rotation of node `i` since the rig was built, as a world rotation vector [rad].
ChVector3d Rotation(const Rig& rig, std::size_t i) {
    const auto now = rig.structure->SampleBeam("arm");
    const auto& q = now.node_orientations_wxyz.at(i);
    const auto& q0 = rig.undeformed.node_orientations_wxyz.at(i);
    const ::chrono::ChQuaterniond qn(q[0], q[1], q[2], q[3]);
    const ::chrono::ChQuaterniond qi(q0[0], q0[1], q0[2], q0[3]);
    return (qn * qi.GetConjugate()).GetRotVec();
}

ChVector3d ReactionForce(const Rig& rig, const std::string& mate) {
    const auto r = rig.structure->GetMateReactionWorld(mate);
    return ChVector3d(r[0], r[1], r[2]);
}

ChVector3d ReactionTorque(const Rig& rig, const std::string& mate) {
    const auto r = rig.structure->GetMateReactionWorld(mate);
    return ChVector3d(r[3], r[4], r[5]);
}

bool SolveStatic(Rig& rig, const std::string& label) {
    const bool ok = rig.system->DoStaticLinear();
    TEST_ASSERT(ok, label + ": linear static solve converged");
    return ok;
}

/// |actual - expected| <= rel_tol * |expected|
#define TEST_REL(actual, expected, rel_tol, message) \
    TEST_NEAR((actual), (expected), (rel_tol) * std::abs(expected), (message))

/// Cantilever tip deflection under self-weight [m], for `elements` elements.
double SelfWeightTip(int elements, double* expected_out = nullptr) {
    BeamCase c;
    c.automatic_gravity = true;
    c.elements = elements;
    c.mates = kClampRoot;
    Rig rig = MakeRig("v6_n" + std::to_string(elements), c, ChVector3d(0, 0, -kG));
    if (!SolveStatic(rig, "V6 n=" + std::to_string(elements))) {
        return 0.0;
    }
    const double w = kRho * c.section.Area() * kG;
    const double L = c.Length();
    if (expected_out) {
        *expected_out = -w * L * L * L * L / (8.0 * kE * c.section.IStrong());
    }
    return Displacement(rig, static_cast<std::size_t>(elements)).z();
}

}  // namespace

int main() {
    std::cout << "test_structure_fea_analytic\n" << std::setprecision(6);
    // The FEA visual shape reads Chrono colormap data on every update, even headless.
    seastack::app::InitChronoEnvironment();

    {  // V1: cantilever with a tip load from a 1000 kg body
        BeamCase c;
        c.mates = std::string(kClampRoot) + kClampTip;
        const double m = 1000.0;
        Rig rig = MakeRig("v1", c, ChVector3d(0, 0, -kG), m);
        if (SolveStatic(rig, "V1")) {
            const double P = m * kG;
            const double L = c.Length();
            const double EI = kE * c.section.IStrong();
            const ChVector3d tip = Displacement(rig, c.elements);
            std::cout << "V1 tip dz = " << tip.z() << " m (theory " << -P * L * L * L / (3 * EI) << ")\n";
            TEST_REL(tip.z(), -P * L * L * L / (3.0 * EI), 0.005, "V1: tip deflection PL^3/3EI");
            TEST_NEAR(tip.x(), 0.0, 1e-6, "V1: no axial displacement");
            TEST_NEAR(tip.y(), 0.0, 1e-9, "V1: no lateral displacement");
            TEST_REL(Rotation(rig, c.elements).y(), P * L * L / (2.0 * EI), 0.005,
                     "V1: tip slope PL^2/2EI (about +Y for a -Z load along +X)");

            // Reaction on the support body (ground): the beam pushes it down
            // with P and twists it with PL; the static check relies on this sign.
            const ChVector3d f = ReactionForce(rig, "root");
            const ChVector3d t = ReactionTorque(rig, "root");
            TEST_REL(f.z(), -P, 0.005, "V1: root reaction on ground points along gravity (-Z), |F| = P");
            TEST_NEAR(f.x(), 0.0, 1e-6 * P, "V1: no axial root reaction");
            TEST_REL(std::abs(t.y()), P * L, 0.005, "V1: root moment PL");
            // Reaction on the tip body: the beam holds it up.
            TEST_REL(ReactionForce(rig, "tip").z(), P, 0.005,
                     "V1: tip mate reaction on the tip body is +P (holds it up)");
        }
    }

    {  // V2: cantilever under self-weight
        BeamCase c;
        c.automatic_gravity = true;
        c.elements = 16;
        c.mates = kClampRoot;
        Rig rig = MakeRig("v2", c, ChVector3d(0, 0, -kG));
        if (SolveStatic(rig, "V2")) {
            const double w = kRho * c.section.Area() * kG;
            const double L = c.Length();
            const double EI = kE * c.section.IStrong();
            const double tip = Displacement(rig, c.elements).z();
            std::cout << "V2 tip dz = " << tip << " m (theory " << -w * L * L * L * L / (8 * EI) << ")\n";
            TEST_REL(tip, -w * L * L * L * L / (8.0 * EI), 0.01, "V2: tip deflection wL^4/8EI");
            TEST_REL(ReactionForce(rig, "root").z(), -w * L, 0.001, "V2: root reaction wL");
            TEST_REL(std::abs(ReactionTorque(rig, "root").y()), w * L * L / 2.0, 0.005,
                     "V2: root moment wL^2/2");
        }
    }

    {  // V3: simply supported span under self-weight (pin at A, roller at B)
        BeamCase c;
        c.automatic_gravity = true;
        c.end = ChVector3d(10, 0, 1);
        c.elements = 16;
        c.mates =
            "    - { name: pin, node: arm.first, body: ground, constrain: [true, true, true, true, false, false] }\n"
            "    - { name: roller, node: arm.last, body: ground, constrain: [false, true, true, false, false, false] }\n";
        Rig rig = MakeRig("v3", c, ChVector3d(0, 0, -kG));
        if (SolveStatic(rig, "V3")) {
            const double w = kRho * c.section.Area() * kG;
            const double L = c.Length();
            const double EI = kE * c.section.IStrong();
            const double mid = Displacement(rig, c.elements / 2).z();
            std::cout << "V3 mid dz = " << mid << " m (theory " << -5 * w * L * L * L * L / (384 * EI) << ")\n";
            TEST_REL(mid, -5.0 * w * L * L * L * L / (384.0 * EI), 0.01, "V3: midspan 5wL^4/384EI");
            TEST_NEAR(Displacement(rig, 0).z(), 0.0, 1e-9, "V3: pin end does not move");
            TEST_NEAR(Displacement(rig, c.elements).z(), 0.0, 1e-9, "V3: roller end does not move");
            TEST_REL(ReactionForce(rig, "pin").z(), -w * L / 2.0, 0.005, "V3: pin reaction wL/2");
            TEST_REL(ReactionForce(rig, "roller").z(), -w * L / 2.0, 0.005, "V3: roller reaction wL/2");
            TEST_NEAR(ReactionTorque(rig, "pin").y(), 0.0, 1e-6 * w * L * L, "V3: pin carries no moment");
        }
    }

    // V4: cantilever twisted by a torque on the tip body, about +X (beam axis)
    for (const bool explicit_j : {false, true}) {
        BeamCase c;
        c.mates = std::string(kClampRoot) + kClampTip;
        if (explicit_j) {
            // Bredt thin-wall value for the 0.3 x 0.3 x 0.01 box: 4 A_m^2 t / perimeter.
            c.section.torsion_constant = 4.0 * std::pow(0.29 * 0.29, 2) * 0.01 / (4 * 0.29);
        }
        const std::string label = explicit_j ? "V4 (torsion_constant)" : "V4 (default J)";
        Rig rig = MakeRig(explicit_j ? "v4_j" : "v4_default", c, ChVector3d(0, 0, 0), 100.0);
        const double T = 1.0e4;  // N*m
        const unsigned int acc = rig.tip->AddAccumulator();
        rig.tip->EmptyAccumulator(acc);
        rig.tip->AccumulateTorque(acc, ChVector3d(T, 0, 0), false);
        if (SolveStatic(rig, label)) {
            const double L = c.Length();
            const double twist = Rotation(rig, c.elements).x();
            const double theory = T * L / (kShear * c.section.J());
            std::cout << label << " twist = " << twist << " rad (theory " << theory << ", J = "
                      << c.section.J() << " m^4)\n";
            TEST_REL(twist, theory, 0.005, label + ": twist TL/GJ");
            TEST_NEAR(Displacement(rig, c.elements).Length(), 0.0, 1e-9, label + ": no bending");
            TEST_REL(ReactionTorque(rig, "root").x(), T, 0.005, label + ": root torque T on ground");
        }
    }

    {  // V5: orientation and principal-axis mapping with a 0.2 x 0.4 section
        Section rect;
        rect.width = 0.2;
        rect.depth = 0.4;
        const double m = 1000.0;
        const double P = m * kG;

        // (a) Beam along +Y, y_direction +X (depth along X), load along -X:
        //     bending about the strong axis.
        {
            BeamCase c;
            c.section = rect;
            c.start = ChVector3d(0, 0, 0);
            c.end = ChVector3d(0, 5, 0);
            c.y_direction = ChVector3d(1, 0, 0);
            c.mates = std::string(kClampRoot) + kClampTip;
            Rig rig = MakeRig("v5a", c, ChVector3d(-kG, 0, 0), m);
            if (SolveStatic(rig, "V5a")) {
                const double L = c.Length();
                const ChVector3d tip = Displacement(rig, c.elements);
                TEST_REL(tip.x(), -P * L * L * L / (3.0 * kE * rect.IStrong()), 0.005,
                         "V5a: load along y_direction bends the strong (depth) axis");
                TEST_NEAR(tip.z(), 0.0, 1e-9, "V5a: no out-of-plane displacement");
            }
        }
        // (b) Same beam, load along -Z (local z): bending about the weak axis.
        {
            BeamCase c;
            c.section = rect;
            c.start = ChVector3d(0, 0, 0);
            c.end = ChVector3d(0, 5, 0);
            c.y_direction = ChVector3d(1, 0, 0);
            c.mates = std::string(kClampRoot) + kClampTip;
            Rig rig = MakeRig("v5b", c, ChVector3d(0, 0, -kG), m);
            if (SolveStatic(rig, "V5b")) {
                const double L = c.Length();
                const ChVector3d tip = Displacement(rig, c.elements);
                TEST_REL(tip.z(), -P * L * L * L / (3.0 * kE * rect.IWeak()), 0.005,
                         "V5b: load across y_direction bends the weak (width) axis");
                TEST_NEAR(tip.x(), 0.0, 1e-9, "V5b: no in-plane displacement");
            }
        }
        // (c) Beam in the XY plane at atan(4/3) to X, y_direction +Z, load -Z.
        {
            BeamCase c;
            c.section = rect;
            c.start = ChVector3d(0, 0, 0);
            c.end = ChVector3d(3, 4, 0);
            c.mates = std::string(kClampRoot) + kClampTip;
            Rig rig = MakeRig("v5c", c, ChVector3d(0, 0, -kG), m);
            if (SolveStatic(rig, "V5c")) {
                const double L = c.Length();
                const ChVector3d tip = Displacement(rig, c.elements);
                TEST_REL(tip.z(), -P * L * L * L / (3.0 * kE * rect.IStrong()), 0.005,
                         "V5c: skewed beam, strong-axis tip deflection");
                TEST_NEAR(std::hypot(tip.x(), tip.y()), 0.0, 1e-6, "V5c: no in-plane displacement");
            }
        }
    }

    {  // V6: mesh convergence of the self-weight cantilever
        double expected = 0.0;
        double previous_error = std::numeric_limits<double>::infinity();
        bool monotonic = true;
        double error16 = 1.0;
        std::cout << "V6 elements  tip_dz_m  rel_error\n";
        for (const int n : {1, 2, 4, 8, 16}) {
            const double tip = SelfWeightTip(n, &expected);
            const double err = std::abs(tip - expected) / std::abs(expected);
            std::cout << "   " << std::setw(8) << n << "  " << tip << "  " << err << "\n";
            monotonic = monotonic && (err <= previous_error + 1e-9);
            previous_error = err;
            error16 = err;
        }
        TEST_ASSERT(monotonic, "V6: error does not grow with mesh refinement");
        TEST_ASSERT(error16 < 0.005, "V6: 16 elements within 0.5% of wL^4/8EI");
    }

    {  // V7: first bending frequency from the free response to sudden self-weight
        const double two_pi = 2.0 * std::acos(-1.0);
        const double dt = 1.0e-4;  // s, ~1000 steps per period

        struct FreeResponse {
            double f_theory = 0.0;          // Hz
            std::vector<double> crossings;  // s, downward crossings of the static deflection
            std::vector<double> peaks;      // m, largest excursion above it in each cycle
        };
        // Tip response to self-weight applied suddenly at t = 0, which excites
        // mainly the first bending mode about the static deflection.
        auto run = [&](const std::string& name, std::optional<double> beta, int cycles) {
            BeamCase c;
            c.automatic_gravity = true;
            c.elements = 16;
            c.section.rayleigh_beta = beta;
            c.mates = kClampRoot;
            Rig rig = MakeRig(name, c, ChVector3d(0, 0, -kG));
            rig.system->SetTimestepperType(::chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

            const double mu = kRho * c.section.Area();  // kg/m
            const double EI = kE * c.section.IStrong();
            const double L = c.Length();
            const double beta1_L = 1.8751040687;
            FreeResponse out;
            out.f_theory = beta1_L * beta1_L / two_pi * std::sqrt(EI / (mu * L * L * L * L));
            const double static_tip = -mu * kG * L * L * L * L / (8.0 * EI);

            const int steps = static_cast<int>(cycles / (out.f_theory * dt));
            double prev = Displacement(rig, c.elements).z() - static_tip;
            double cycle_max = prev;
            for (int i = 0; i < steps; ++i) {
                rig.system->DoStepDynamics(dt);
                const double z = Displacement(rig, c.elements).z() - static_tip;
                cycle_max = std::max(cycle_max, z);
                if (prev > 0.0 && z <= 0.0) {
                    out.crossings.push_back(rig.system->GetChTime() - dt + prev / (prev - z) * dt);
                    out.peaks.push_back(cycle_max);
                    cycle_max = z;
                }
                prev = z;
            }
            return out;
        };

        // Undamped (no rayleigh_beta): frequency, and no hidden damping.
        const FreeResponse free = run("v7_undamped", std::nullopt, 6);
        TEST_ASSERT(free.crossings.size() >= 5, "V7: at least five oscillation cycles recorded");
        if (free.crossings.size() >= 2) {
            const double period = (free.crossings.back() - free.crossings.front()) /
                                  static_cast<double>(free.crossings.size() - 1);
            std::cout << "V7 f1 = " << 1.0 / period << " Hz (theory " << free.f_theory << " Hz)\n";
            TEST_REL(1.0 / period, free.f_theory, 0.02, "V7: first natural frequency");
        }
        if (free.peaks.size() >= 5) {
            const double kept = free.peaks[4] / free.peaks[0];
            std::cout << "V7 undamped amplitude after 4 cycles: " << kept << " of initial\n";
            TEST_ASSERT(kept > 0.85, "V7: with rayleigh_beta omitted the beam is undamped "
                                     "(Chrono's default beta = 0.01 s is not applied)");
        }

        // Stiffness-proportional damping.  Textbook Rayleigh damping R = beta*K
        // gives zeta_1 = beta * omega_1 / 2, but Chrono's ChElementBeamEuler
        // measures twice that, zeta_1 = beta * omega_1, independent of dt
        // (checked for beta = 2.5e-4..2e-3 s, dt = 1e-4 and 2.5e-5 s).  This
        // pins the behaviour users get; implicit Euler adds ~omega_1 * dt / 2.
        const double beta = 1.0e-3;  // s
        const FreeResponse damped = run("v7_damped", beta, 5);
        TEST_ASSERT(damped.peaks.size() >= 3, "V7: damped peaks recorded");
        if (damped.peaks.size() >= 3) {
            const double omega = two_pi * damped.f_theory;
            const double log_dec = std::log(damped.peaks[0] / damped.peaks[2]) / 2.0;
            const double zeta = log_dec / std::sqrt(two_pi * two_pi + log_dec * log_dec);
            const double zeta_expected = beta * omega + omega * dt / 2.0;
            std::cout << "V7 damping ratio = " << zeta << " (Chrono beta*omega " << zeta_expected
                      << ", textbook beta*omega/2 " << beta * omega / 2.0 << ")\n";
            TEST_REL(zeta, zeta_expected, 0.10, "V7: rayleigh_beta damping ratio (Chrono: beta*omega)");
        }
    }

    {  // V8: static load-path check on a fixed-fixed beam with a 500 kg plate
        BeamCase c;
        c.automatic_gravity = true;
        c.end = ChVector3d(8, 0, 1);
        c.elements = 8;
        c.mates =
            "    - { name: left, node: arm.first, body: ground, constrain: [true, true, true, true, true, true] }\n"
            "    - { name: right, node: arm.last, body: ground, constrain: [true, true, true, true, true, true] }\n";
        c.extra =
            "  node_sets:\n"
            "    - { name: centre, nodes: [arm.mid] }\n"
            "  rigid_attachments:\n"
            "    - { name_prefix: plate, node_set: centre, mass: 500.0, box: [1.0, 1.0, 0.05], collide: false }\n";
        Rig rig = MakeRig("v8", c, ChVector3d(0, 0, -kG), 0.0, /*settle_time_s=*/0.0);
        if (SolveStatic(rig, "V8")) {
            rig.structure->OnAfterStep(0.0, 0.0);
            const auto& r = rig.structure->GetStaticCheckResult();
            const double W = (kRho * c.section.Area() * c.Length() + 500.0) * kG;
            TEST_ASSERT(r.valid, "V8: check performed: " + r.invalid_reason);
            TEST_REL(r.weight_kN, W / 1000.0, 1e-9, "V8: structure weight = beam + plate");
            TEST_NEAR(r.error_percent, 0.0, 0.01, "V8: reactions along gravity equal the weight");
            const double fz = ReactionForce(rig, "left").z() + ReactionForce(rig, "right").z();
            TEST_REL(fz, -W, 1e-4, "V8: summed world Fz on ground is -W");
        }

        BeamCase off = c;
        Rig no_g = MakeRig("v8_off", off, ChVector3d(0, 0, 0), 0.0, 0.0);
        if (SolveStatic(no_g, "V8 no gravity")) {
            no_g.structure->OnAfterStep(0.0, 0.0);
            const auto& r = no_g.structure->GetStaticCheckResult();
            TEST_ASSERT(!r.valid && r.invalid_reason.find("no weight") != std::string::npos,
                        "V8: with gravity off the check is not performed: " + r.invalid_reason);
        }
    }

    test_results.Summary();
    return test_results.failed == 0 ? 0 : 1;
}
