// Shared 5SA (five-segment attenuator) articulated model setup.
// Used by demo_5sa_spreading.cpp and demo_5sa_bimodal.cpp.

#ifndef FIVE_SA_MODEL_SETUP_H
#define FIVE_SA_MODEL_SETUP_H

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>

#include <chrono/assets/ChVisualShapePointPoint.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChLinkUniversal.h>
#include <chrono/physics/ChSystemNSC.h>

#include <filesystem>
#include <string>
#include <vector>
#include <array>

using seastack::chrono::HydroSystem;

/// Matches `linear_damping` in `data/demos/run_seastack/5sa/*/5sa_*.hydro.yaml`:
/// surge, sway, heave, roll, pitch, yaw (see seastack/core/types.h).
inline void ApplyFiveSaHydroLinearDamping(HydroSystem& hydro) {
    constexpr std::array<double, 6> kLinearDamping = {0.0, 100000.0, 0.0, 500000.0, 0.0, 0.0};
    std::vector<std::array<double, 6>> per_body(5, kLinearDamping);
    hydro.SetLinearDamping(per_body);
}

struct FiveSaModel {
    std::vector<std::shared_ptr<chrono::ChBody>> bodies;
    std::shared_ptr<chrono::ChBody> ground;
    std::string h5file;
    std::string moordyn_input;
};

inline FiveSaModel SetupFiveSaModel(chrono::ChSystem& system, const std::filesystem::path& DATADIR,
                                    const std::string& h5_filename) {
    FiveSaModel model;

    auto geom_dir = DATADIR / "demos" / "5sa" / "geometry";
    model.h5file = (DATADIR / "demos" / "5sa" / "hydroData" / h5_filename).lexically_normal().generic_string();
    model.moordyn_input =
        (DATADIR / "demos" / "5sa" / "mooring" / "lines_5sa.txt").lexically_normal().generic_string();

    // Segment positions: 5 segments spaced 36m apart, centered on x-axis
    constexpr double segment_spacing = 36.0;
    constexpr double draft = -1.8;
    constexpr double mass = 438293.0;
    const chrono::ChVector3d inertia(876585.0, 47773884.0, 47773884.0);

    for (int i = 0; i < 5; ++i) {
        double x = 18.0 + i * segment_spacing;
        std::string mesh_file = (geom_dir / ("segment_" + std::to_string(i + 1) + ".obj"))
                                    .lexically_normal().generic_string();
        auto body = chrono_types::make_shared<chrono::ChBodyEasyMesh>(mesh_file, 0, false, true, false);
        body->SetName("body" + std::to_string(i + 1));
        body->SetPos(chrono::ChVector3d(x, 0, draft));
        body->SetMass(mass);
        body->SetInertiaXX(inertia);
        system.Add(body);
        model.bodies.push_back(body);
    }

    // Universal joints between adjacent segments (pitch + yaw)
    for (int i = 0; i < 4; ++i) {
        double joint_x = 36.0 + i * segment_spacing;
        auto joint = chrono_types::make_shared<chrono::ChLinkUniversal>();
        joint->Initialize(model.bodies[i], model.bodies[i + 1],
                          chrono::ChFrame<>(chrono::ChVector3d(joint_x, 0, draft),
                                    chrono::QuatFromAngleX(0)));
        system.AddLink(joint);

        // 4 TSDAs per joint (top, bottom, port, starboard)
        struct TSDAPair { chrono::ChVector3d p1; chrono::ChVector3d p2; };
        std::array<TSDAPair, 4> tsda_points = {{
            {{joint_x - 1, 0, draft + 1.5}, {joint_x + 1, 0, draft + 1.5}},   // top
            {{joint_x - 1, 0, draft - 1.5}, {joint_x + 1, 0, draft - 1.5}},   // bottom
            {{joint_x - 1, -1.5, draft},    {joint_x + 1, -1.5, draft}},       // port
            {{joint_x - 1, 1.5, draft},     {joint_x + 1, 1.5, draft}}         // starboard
        }};

        for (const auto& tp : tsda_points) {
            auto tsda = chrono_types::make_shared<chrono::ChLinkTSDA>();
            tsda->Initialize(model.bodies[i], model.bodies[i + 1], false, tp.p1, tp.p2);
            tsda->SetSpringCoefficient(25000.0);
            tsda->SetDampingCoefficient(500000.0);
            tsda->SetRestLength(2.0);
            // Same spring glyph as YAML `visualization: { type: SPRING, radius, resolution, turns }`
            auto spring_vis = chrono_types::make_shared<chrono::ChVisualShapeSpring>(0.15, 65, 10.0);
            tsda->AddVisualShape(spring_vis);
            system.AddLink(tsda);
        }
    }

    // Ground body
    model.ground = chrono_types::make_shared<chrono::ChBody>();
    system.AddBody(model.ground);
    model.ground->SetFixed(true);
    model.ground->EnableCollision(false);

    return model;
}

#endif  // FIVE_SA_MODEL_SETUP_H
