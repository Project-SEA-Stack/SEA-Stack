// Shared VLFP pontoon bodies: six identical box pontoons + pitch hinges.
//
// Model source: Strathclyde VLFP (HINGED_6 configuration). Matches the YAML case
// `data/demos/run_seastack/vlfp/regular_waves/` and the coupled BEMIO database
// `<data>/demos/vlfp/hydroData/vlfp.h5` (body1..body6 H5 groups, cross-coupling).
//
// Frame: z up, still-water level z = 0, wave heading +X (platform longitudinal axis).
// Pontoon CoG at z = +1.6 m; each pontoon is 29 m (x) x 58 m (y) x 5.2 m (z) with its
// local frame at the CoG, so draft = 1.0 m and the deck top sits at z = +4.2 m.
//
//  body name   H5 group   x CoG (world)   Hinge to next at x =
//  ----------  --------   -------------   --------------------
//  body1       body1       14.5            29.0
//  body2       body2       43.5            58.0
//  body3       body3       72.5            87.0
//  body4       body4      101.5           116.0
//  body5       body5      130.5           145.0
//  body6       body6      159.5           (none)
//
// hydro_bodies order is { body1 .. body6 }. Do not reorder without a new H5.
//
// NOTE: The reference YAML model also carries five RSDA hinge dampers with
// k = c = 0 (legacy parity placeholders). They are physically inert and are
// deliberately omitted here.

#ifndef VLFP_PONTOONS_H
#define VLFP_PONTOONS_H

#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/collision/ChCollisionShapeBox.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChContactMaterial.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystem.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace vlfp {

// Platform geometry (world frame, z up, SWL at z = 0).
constexpr int kNumPontoons = 6;
constexpr double kPontoonLengthX = 29.0;   // m, along platform axis
constexpr double kPontoonWidthY = 58.0;    // m
constexpr double kPontoonHeightZ = 5.2;    // m
constexpr double kPontoonCogZ = 1.6;       // m (world z of CoG at equilibrium)
constexpr double kPlatformY = 29.0;        // m (world y of platform centreline)
constexpr double kDeckTopZ = kPontoonCogZ + 0.5 * kPontoonHeightZ;  // +4.2 m

// Mass properties per pontoon (from the Strathclyde reference model; must match
// the BEM H5 file).
constexpr double kPontoonMass = 1732460.0;  // kg
constexpr double kPontoonIxx = 751780112.0;
constexpr double kPontoonIyy = 194629819.0;
constexpr double kPontoonIzz = 929312321.0;

// Soft surge restraint (drift control only; not a physical mooring).
constexpr double kSurgeSpringCoefficient = 1000.0;  // N/m

inline double PontoonCenterX(int index_zero_based) {
    return 14.5 + kPontoonLengthX * index_zero_based;
}

struct VlfpPlatform {
    std::vector<std::shared_ptr<::chrono::ChBody>> pontoons;      // body1..body6
    std::vector<std::shared_ptr<::chrono::ChBody>> hydro_bodies;  // same order as H5 groups
    std::shared_ptr<::chrono::ChBody> ground;
    std::string h5file;
};

/// Add the six VLFP pontoons, five pitch hinges (axis +Y), a fixed ground body,
/// and the soft surge TSDA to `system`; return handles.
///
/// When `enable_deck_collision` is true, each pontoon also receives a box
/// collision shape matching its full extent (29 x 58 x 5.2 m). All pontoons are
/// placed in the same collision family and self-collisions within that family
/// are disabled, so adjacent pontoons never generate contacts at the hinge
/// lines; only external objects (e.g. vehicle tires) contact the decks.
/// `deck_material` must be non-null in that case and must match the system
/// contact method (SMC material for ChSystemSMC).
inline VlfpPlatform AddVlfpPlatform(
        ::chrono::ChSystem& system,
        const std::filesystem::path& DATADIR,
        bool enable_deck_collision = false,
        std::shared_ptr<::chrono::ChContactMaterial> deck_material = nullptr) {
    using namespace ::chrono;

    if (enable_deck_collision && !deck_material) {
        throw std::invalid_argument("VLFP: enable_deck_collision requires a contact material");
    }

    VlfpPlatform out;

    const auto geom_dir = DATADIR / "demos" / "vlfp" / "geometry";
    out.h5file = (DATADIR / "demos" / "vlfp" / "hydroData" / "vlfp.h5")
                     .lexically_normal()
                     .generic_string();

    // Collision family for pontoon decks. Must not use Chrono::Vehicle's reserved
    // families (0=chassis, 1=tire, 2=wheel, 3..6=tracked parts). Terrain demos
    // conventionally use 14; we pick 7 as a free slot for the floating decks.
    constexpr int kPontoonCollisionFamily = 7;

    for (int i = 0; i < kNumPontoons; ++i) {
        const std::string body_name = "body" + std::to_string(i + 1);
        const auto mesh_path = (geom_dir / ("pontoon_" + std::to_string(i + 1) + ".obj"))
                                   .lexically_normal()
                                   .generic_string();
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_path, true, true);
        if (!trimesh || trimesh->GetNumVertices() == 0) {
            throw std::runtime_error(
                "VLFP: failed to load pontoon mesh '" + mesh_path +
                "'. C++ demos expect geometry under <data>/demos/vlfp/geometry/ (CMake copies "
                "from data/demos/run_seastack/vlfp/assets/ into the build tree). If you use "
                "--data_dir, clear SEASTACK_DATA_DIR so it is not overriding your path.");
        }

        // Mass/inertia set explicitly from the reference model (compute_mass = false).
        auto pontoon = chrono_types::make_shared<ChBodyEasyMesh>(trimesh, 0.0, false, false, false);
        pontoon->SetName(body_name);
        pontoon->SetPos(ChVector3d(PontoonCenterX(i), kPlatformY, kPontoonCogZ));
        pontoon->SetMass(kPontoonMass);
        pontoon->SetInertiaXX(ChVector3d(kPontoonIxx, kPontoonIyy, kPontoonIzz));

        {
            auto viz = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            viz->SetMesh(trimesh);
            viz->SetName("pontoon_" + std::to_string(i + 1));
            pontoon->AddVisualShape(viz, ChFrame<>());
        }

        if (enable_deck_collision) {
            auto box = chrono_types::make_shared<ChCollisionShapeBox>(
                deck_material, kPontoonLengthX, kPontoonWidthY, kPontoonHeightZ);
            pontoon->AddCollisionShape(box, ChFrame<>());
            pontoon->GetCollisionModel()->SetFamily(kPontoonCollisionFamily);
            pontoon->GetCollisionModel()->DisallowCollisionsWith(kPontoonCollisionFamily);
            pontoon->EnableCollision(true);
        } else {
            pontoon->EnableCollision(false);
        }

        system.Add(pontoon);
        out.pontoons.push_back(pontoon);
    }
    out.hydro_bodies = out.pontoons;

    // Five pitch hinges between neighbouring pontoons on the deck centreline.
    // ChLinkLockRevolute's free axis is local Z; rotate local Z onto world +Y.
    const ChQuaternion<> hinge_rot = QuatFromAngleX(-CH_PI / 2.0);
    for (int i = 0; i < kNumPontoons - 1; ++i) {
        const double hinge_x = kPontoonLengthX * (i + 1);
        auto hinge = chrono_types::make_shared<ChLinkLockRevolute>();
        hinge->SetName("hinge_" + std::to_string(i + 1) + "_" + std::to_string(i + 2));
        hinge->Initialize(out.pontoons[i], out.pontoons[i + 1],
                          ChFramed(ChVector3d(hinge_x, kPlatformY, kPontoonCogZ), hinge_rot));
        system.AddLink(hinge);
    }

    // Fixed ground body for the surge restraint.
    out.ground = chrono_types::make_shared<ChBody>();
    out.ground->SetName("ground");
    out.ground->SetFixed(true);
    out.ground->EnableCollision(false);
    system.Add(out.ground);

    // Soft surge TSDA on body3 (zero free length, coincident endpoints at
    // equilibrium; same as the YAML reference model).
    auto surge_spring = chrono_types::make_shared<ChLinkTSDA>();
    surge_spring->SetName("surge_mooring");
    const ChVector3d anchor(PontoonCenterX(2), kPlatformY, kPontoonCogZ);
    surge_spring->Initialize(out.ground, out.pontoons[2], false, anchor, anchor);
    surge_spring->SetSpringCoefficient(kSurgeSpringCoefficient);
    surge_spring->SetDampingCoefficient(0.0);
    surge_spring->SetRestLength(0.0);
    system.AddLink(surge_spring);

    return out;
}

}  // namespace vlfp

#endif  // VLFP_PONTOONS_H
