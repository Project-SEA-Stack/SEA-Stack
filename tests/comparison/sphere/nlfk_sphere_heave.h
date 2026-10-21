/**
 * Shared heave-only IEA sphere run for the nonlinear Froude-Krylov comparison
 * tests (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9, C1-C4, C6, C7).
 *
 * Mirrors the Python reference (tests/verification/utilities/nlfk_reference.py,
 * simulate_heave): mass 261.8 t, prismatic heave joint, optional linear heave
 * PTO damper, frequency-domain excitation with a cosine ramp, and constant-
 * coefficient (A(w), B(w)) or RIRF radiation.
 *
 * Output: text file "# key value" header lines, then "time heave" rows with
 * heave z_cg - z_eq [m], positive up. Wave: eta = a cos(w t) at x = 0.
 */

#ifndef SEASTACK_TESTS_COMPARISON_NLFK_SPHERE_HEAVE_H
#define SEASTACK_TESTS_COMPARISON_NLFK_SPHERE_HEAVE_H

#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/timestepper/ChTimestepperHHT.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace nlfk_test {

/// Hydrostatic / Froude-Krylov model; names match nlfk_reference.py HYDRO_MODELS.
enum class HydroModel { kLinear, kCalmWaterNonlinear, kNlfkAiryWheeler, kNlfkHydrostatic };

inline const std::vector<HydroModel> kAllModels = {HydroModel::kLinear, HydroModel::kCalmWaterNonlinear,
                                                   HydroModel::kNlfkAiryWheeler, HydroModel::kNlfkHydrostatic};

inline const char* Name(HydroModel m) {
    switch (m) {
        case HydroModel::kLinear: return "linear";
        case HydroModel::kCalmWaterNonlinear: return "nl_hydrostatics";
        case HydroModel::kNlfkAiryWheeler: return "nlfk_airy_wheeler";
        case HydroModel::kNlfkHydrostatic: return "nlfk_hydrostatic";
    }
    return "unknown";
}

struct HeaveCase {
    double amplitude = 1.0;              ///< wave amplitude a [m]
    double omega = 0.8;                  ///< [rad/s]
    double pto_damping = 0.0;            ///< linear heave damper [N s/m]
    bool constant_coefficients = true;   ///< A(w), B(w) radiation; else RIRF convolution
    double t_end = 140.0;                ///< [s]
    double ramp = 20.0;                  ///< cosine ramp duration [s]
    double depth = 200.0;                ///< water depth [m] (sphere.h5)
    double dt = 0.01;                    ///< [s]
    double z0_offset = 0.0;              ///< initial heave from equilibrium [m]
};

constexpr double kSphereMass = 261.8e3;  // [kg]

/// Runs one case and writes the heave time series to `path`. Returns false on I/O failure.
inline bool RunSphereHeave(const std::string& h5, const std::string& mesh, HydroModel model, const HeaveCase& c,
                           double z_eq, const std::string& path) {
    using namespace chrono;
    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht = std::static_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    hht->SetAlpha(-0.2);
    hht->SetMaxIters(50);
    hht->SetRelTolerance(1e-6);
    hht->SetAbsTolerances(1e-6, 1e2);
    hht->SetStepControl(false);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0, 0, -50));
    ground->SetFixed(true);
    system.AddBody(ground);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetName("body1");
    body->SetPos(ChVector3d(0, 0, z_eq + c.z0_offset));
    body->SetMass(kSphereMass);
    system.AddBody(body);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(body, ground, false, ChFramed(ChVector3d(0, 0, z_eq)), ChFramed(ChVector3d(0, 0, -50)));
    system.AddLink(prismatic);

    if (c.pto_damping > 0.0) {
        // Vertical damper (spring stiffness 0): F = -c * heave velocity.
        auto pto = chrono_types::make_shared<ChLinkTSDA>();
        pto->Initialize(body, ground, false, ChVector3d(0, 0, z_eq), ChVector3d(0, 0, -50));
        pto->SetSpringCoefficient(0.0);
        pto->SetDampingCoefficient(c.pto_damping);
        system.AddLink(pto);
    }

    // amplitude 0 means still water (free decay): NoWave, no excitation.
    std::shared_ptr<seastack::hydro::WaveBase> wave = std::make_shared<seastack::hydro::NoWave>();
    if (c.amplitude > 0.0) {
        seastack::hydro::SeaStateDefinition sea_state;
        sea_state.type = "regular";
        sea_state.amplitude = c.amplitude;
        sea_state.omega = c.omega;
        sea_state.depth = c.depth;
        wave = std::make_shared<seastack::hydro::LinearDirectionalWaveField>(
            seastack::hydro::ComponentSampler::Build(sea_state), sea_state.depth);
        wave->SetRampDuration(c.ramp);
    }

    std::vector<std::shared_ptr<ChBody>> bodies = {body};
    seastack::chrono::HydroSystem hydro(bodies, h5);
    hydro.SetRadiationMethod(c.constant_coefficients ? seastack::hydro::RadiationMethod::kConstantCoefficients
                                                     : seastack::hydro::RadiationMethod::kRirfConvolution);
    hydro.SetExcitationMethod(seastack::hydro::ExcitationMethod::kFrequencyDomain);
    if (model != HydroModel::kLinear) {
        seastack::hydro::HydroModelBuilder::BodyHydrostaticsConfig cfg;
        cfg.model = seastack::hydro::HydrostaticsModel::kNonlinear;
        cfg.mesh_file = mesh;
        if (model != HydroModel::kCalmWaterNonlinear) {
            cfg.froude_krylov = seastack::hydro::FroudeKrylovModel::kNonlinear;
            cfg.fk_pressure = (model == HydroModel::kNlfkHydrostatic)
                                  ? seastack::hydro::FroudeKrylovPressureModel::kHydrostatic
                                  : seastack::hydro::FroudeKrylovPressureModel::kAiryWheeler;
        }
        hydro.SetBodyHydrostatics({cfg});
    }
    hydro.AddWaves(wave);

    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error: cannot write " << path << "\n";
        return false;
    }
    out << std::setprecision(12);
    out << "# wave_amplitude_m " << c.amplitude << "\n";
    out << "# omega_rad_s " << c.omega << "\n";
    out << "# pto_damping_Ns_m " << c.pto_damping << "\n";
    out << "# model " << Name(model) << "\n";
    out << "# Time(s)  Heave(m)\n";
    out << std::fixed << std::setprecision(4) << 0.0 << " " << std::setprecision(9) << c.z0_offset << "\n";
    while (system.GetChTime() < c.t_end - c.dt / 2.0) {
        system.DoStepDynamics(c.dt);
        out << std::fixed << std::setprecision(4) << system.GetChTime() << " " << std::setprecision(9)
            << body->GetPos().z() - z_eq << "\n";
    }
    return true;
}

}  // namespace nlfk_test

#endif  // SEASTACK_TESTS_COMPARISON_NLFK_SPHERE_HEAVE_H
