// Shared trimaran hull bodies (center + port + starboard outriggers).
// Mass, inertia, and CoG z match `demos/trimaran/hydroData/trimaran.h5` (BEM export).
//
// Frame: z up, waterline z = 0, wave heading +X. Facing +X: +Y = port, -Y = starboard.
//
//  body name   H5 group   y (world)        Role
//  ----------  --------   --------------   -------------
//  body1       body1      0                Center
//  body2       body2      -kOutriggerY     Starboard
//  body3       body3      +kOutriggerY     Port
//
// hydro_bodies order is { body1, body2, body3 }. Do not swap outrigger poses without new H5.

#ifndef TRIMARAN_HULLS_H
#define TRIMARAN_HULLS_H

#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemSMC.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace trimaran {

// Deck / spacing geometry (world frame, z up, WL at z = 0).
constexpr double kCenterFreeboard = 3.5;
constexpr double kOutriggerFreeboard = 1.5;
constexpr double kOutriggerY = 15.0;
constexpr double kCenterHalfBeam = 6.0;
constexpr double kOutriggerHalfBeam = 2.0;

// CoG z must match trimaran BEM H5 (body*/properties/cg).
constexpr double kCenterCogZ = -1.75;
constexpr double kOutriggerCogZ = -0.75;

struct TrimaranHulls {
    std::shared_ptr<::chrono::ChBody> center;
    std::shared_ptr<::chrono::ChBody> port;
    std::shared_ptr<::chrono::ChBody> stbd;
    std::vector<std::shared_ptr<::chrono::ChBody>> hydro_bodies;
    std::string h5file;
};

/// Add three Wigley hull bodies (OBJ triangle-mesh visuals, CoG at BEM equilibrium) and return handles.
///
/// Uses ChBodyEasyMesh + ChVisualShapeTriangleMesh (same pattern as RM3 demos) so the Irrlicht path
/// renders native shaded meshes instead of ChVisualShapeModelFile. Mass and inertia are set
/// explicitly from BEM (`compute_mass = false`, `density = 0`); visual frames keep the same
/// z-offset vs CoG as the legacy ChBody + model-file path.
inline TrimaranHulls AddTrimaranHulls(::chrono::ChSystemSMC& system,
                                      const std::filesystem::path& DATADIR) {
    using namespace ::chrono;

    TrimaranHulls out;

    auto geom_dir = DATADIR / "demos" / "trimaran" / "geometry";
    out.h5file = (DATADIR / "demos" / "trimaran" / "hydroData" / "trimaran.h5")
                     .lexically_normal()
                     .generic_string();

    constexpr double center_mesh_z_from_cog = -kCenterCogZ;
    constexpr double outrigger_mesh_z_from_cog = -kOutriggerCogZ;

    auto center_mesh_path =
        (geom_dir / "center.obj").lexically_normal().generic_string();
    auto center_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(center_mesh_path, true, true);
    if (!center_trimesh || center_trimesh->GetNumVertices() == 0) {
        throw std::runtime_error(
            "Trimaran: failed to load hull mesh '" + center_mesh_path +
            "'. C++ demos expect geometry under <data>/demos/trimaran/geometry/ (CMake copies "
            "from demos/run_seastack/trimaran/assets/ into the build tree). If you use --data_dir, "
            "clear SEASTACK_DATA_DIR so it is not overriding your path.");
    }
    out.center = chrono_types::make_shared<ChBodyEasyMesh>(center_trimesh, 0.0, false, false, false);
    out.center->SetName("body1");
    out.center->SetPos(ChVector3d(0, 0, kCenterCogZ));
    out.center->SetMass(956667.0);
    out.center->SetInertiaXX(ChVector3d(16875600.0, 149479167.0, 149479167.0));
    out.center->EnableCollision(false);
    {
        auto center_viz = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        center_viz->SetMesh(center_trimesh);
        center_viz->SetName("center");
        out.center->AddVisualShape(center_viz,
                                   ChFrame<>(ChVector3d(0, 0, center_mesh_z_from_cog)));
    }
    system.Add(out.center);

    auto outrigger_mesh_path =
        (geom_dir / "outrigger.obj").lexically_normal().generic_string();
    auto outrigger_trimesh =
        ChTriangleMeshConnected::CreateFromWavefrontFile(outrigger_mesh_path, true, true);
    if (!outrigger_trimesh || outrigger_trimesh->GetNumVertices() == 0) {
        throw std::runtime_error(
            "Trimaran: failed to load hull mesh '" + outrigger_mesh_path +
            "'. See center.obj error message in trimaran_hulls.h for data directory notes.");
    }

    // body2 H5 block: outrigger at -Y (nautical starboard). ChBody names must stay body2/body3.
    auto hull_minus_y =
        chrono_types::make_shared<ChBodyEasyMesh>(outrigger_trimesh, 0.0, false, false, false);
    hull_minus_y->SetName("body2");
    hull_minus_y->SetPos(ChVector3d(0, -kOutriggerY, kOutriggerCogZ));
    hull_minus_y->SetMass(68333.0);
    hull_minus_y->SetInertiaXX(ChVector3d(133933.0, 2669271.0, 2669271.0));
    hull_minus_y->EnableCollision(false);
    {
        auto viz_minus = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        viz_minus->SetMesh(outrigger_trimesh);
        viz_minus->SetName("outrigger");
        hull_minus_y->AddVisualShape(viz_minus,
                                     ChFrame<>(ChVector3d(0, 0, outrigger_mesh_z_from_cog)));
    }
    system.Add(hull_minus_y);

    auto hull_plus_y =
        chrono_types::make_shared<ChBodyEasyMesh>(outrigger_trimesh, 0.0, false, false, false);
    hull_plus_y->SetName("body3");
    hull_plus_y->SetPos(ChVector3d(0, +kOutriggerY, kOutriggerCogZ));
    hull_plus_y->SetMass(68333.0);
    hull_plus_y->SetInertiaXX(ChVector3d(133933.0, 2669271.0, 2669271.0));
    hull_plus_y->EnableCollision(false);
    {
        auto viz_plus = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        viz_plus->SetMesh(outrigger_trimesh);
        viz_plus->SetName("outrigger");
        hull_plus_y->AddVisualShape(viz_plus,
                                    ChFrame<>(ChVector3d(0, 0, outrigger_mesh_z_from_cog)));
    }
    system.Add(hull_plus_y);

    out.hydro_bodies = {out.center, hull_minus_y, hull_plus_y};
    out.port = hull_plus_y;
    out.stbd = hull_minus_y;
    return out;
}

}  // namespace trimaran

#endif  // TRIMARAN_HULLS_H
