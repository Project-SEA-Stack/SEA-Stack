// Unit tests for the Chrono -> viz_core scene extractor (headless Chrono, no VSG).

#include <seastack/adapters/chrono/viz_extractor.h>

#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapeCapsule.h>
#include <chrono/assets/ChVisualShapePointPoint.h>
#include <chrono/assets/ChVisualShapeSphere.h>
#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyAuxRef.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystemNSC.h>

#include "test_macros.h"

#include <cmath>
#include <string>
#include <vector>

using seastack::chrono::VizExtractor;
using namespace seastack::viz;

int main() {
    TestResults test_results;

    ::chrono::ChSystemNSC system;
    system.SetGravitationalAcceleration({0.0, 0.0, -9.81});

    // Floating body: 2 x 4 x 6 m box offset +1 m in local x, plus a 1-triangle mesh.
    auto hull = chrono_types::make_shared<::chrono::ChBody>();
    hull->SetName("hull");
    hull->SetPos({10.0, 0.0, 0.0});
    hull->SetRot(::chrono::QuatFromAngleZ(0.5 * ::chrono::CH_PI));
    hull->SetPosDt({1.0, 2.0, 3.0});
    hull->SetAngVelParent({0.0, 0.0, 0.5});
    auto box = chrono_types::make_shared<::chrono::ChVisualShapeBox>(2.0, 4.0, 6.0);
    box->SetColor(::chrono::ChColor(0.2f, 0.3f, 0.4f));
    hull->AddVisualShape(box, ::chrono::ChFramed(::chrono::ChVector3d(1.0, 0.0, 0.0)));
    auto tri = chrono_types::make_shared<::chrono::ChTriangleMeshConnected>();
    tri->GetCoordsVertices() = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    tri->GetIndicesVertexes() = {{0, 1, 2}};
    auto mesh_shape = chrono_types::make_shared<::chrono::ChVisualShapeTriangleMesh>();
    mesh_shape->SetMesh(tri);
    mesh_shape->SetScale({2.0, 2.0, 2.0});
    hull->AddVisualShape(mesh_shape);
    system.AddBody(hull);

    // Fixed sphere and a water helper body (must be skipped).
    auto anchor = chrono_types::make_shared<::chrono::ChBody>();
    anchor->SetName("anchor");
    anchor->SetFixed(true);
    anchor->AddVisualShape(chrono_types::make_shared<::chrono::ChVisualShapeSphere>(0.5));
    system.AddBody(anchor);
    auto water = chrono_types::make_shared<::chrono::ChBody>();
    water->SetName("water_surface");
    system.AddBody(water);

    // Body with a reference frame away from its centre of mass.
    auto aux = chrono_types::make_shared<::chrono::ChBodyAuxRef>();
    aux->SetName("aux");
    aux->SetFrameRefToAbs(::chrono::ChFramed(::chrono::ChVector3d(0.0, 0.0, 5.0)));
    aux->SetFrameCOMToRef(::chrono::ChFramed(::chrono::ChVector3d(0.0, 0.0, -1.0)));
    system.AddBody(aux);

    // One mooring line with 3 nodes.
    double line_z = -10.0;
    seastack::viz::MooringVizProvider provider = [&line_z]() {
        seastack::viz::MooringLineVizData line;
        line.node_positions = {{0.0, 0.0, line_z}, {1.0, 0.0, line_z}, {2.0, 0.0, line_z}};
        line.node_tensions = {100.0, 200.0, 300.0};
        line.start_point_type = seastack::viz::MooringPointType::kFixed;
        line.end_point_type = seastack::viz::MooringPointType::kCoupled;
        return std::vector<seastack::viz::MooringLineVizData>{line};
    };

    VizExtractor extractor(&system, provider);
    const SceneDescription& scene = extractor.Scene();

    TEST_ASSERT(scene.bodies.size() == 3, "water helper body skipped");
    TEST_ASSERT(scene.bodies[0].name == "hull" && scene.bodies[1].name == "anchor", "system order kept");
    TEST_ASSERT(scene.bodies[1].fixed, "fixed flag");
    TEST_NEAR(scene.gravity, 9.81, 1e-12, "gravity magnitude");

    const BodyDescription& hull_desc = scene.bodies[0];
    TEST_ASSERT(hull_desc.shapes.size() == 2, "box and mesh captured");
    TEST_ASSERT(hull_desc.shapes[0].kind == ShapeKind::kBox, "box kind");
    TEST_NEAR(hull_desc.shapes[0].size[2], 6.0, 1e-12, "box full length");
    TEST_NEAR(hull_desc.shapes[0].local.position[0], 1.0, 1e-12, "shape offset");
    TEST_NEAR(hull_desc.shapes[0].color[1], 0.3f, 1e-6, "shape colour");
    TEST_ASSERT(hull_desc.shapes[1].kind == ShapeKind::kMesh && hull_desc.shapes[1].mesh, "mesh kind");
    TEST_NEAR(hull_desc.shapes[1].mesh->vertices[1][0], 2.0, 1e-12, "mesh scale applied");
    TEST_NEAR(hull_desc.local_bounds.min[0], 0.0, 1e-12, "local bounds min x (box offset)");
    TEST_NEAR(hull_desc.local_bounds.max[0], 2.0, 1e-12, "local bounds max x");
    TEST_NEAR(hull_desc.local_bounds.max[2], 3.0, 1e-12, "local bounds max z");

    TEST_ASSERT(scene.mooring_lines.size() == 1 && scene.mooring_lines[0].num_nodes == 3, "mooring topology");
    TEST_ASSERT(scene.mooring_lines[0].start_type == MooringEndType::kFixed, "mooring start type");
    TEST_ASSERT(scene.model_bounds.IsValid(), "model bounds computed");
    // Hull rotated +90 deg about Z: local x in [0, 2] maps to world y in [0, 2] around x = 10.
    TEST_NEAR(scene.model_bounds.max[1], 2.0, 1e-9, "model bounds follow body rotation");

    VizFrame frame = extractor.Capture(7);
    TEST_ASSERT(IsConsistent(scene, frame), "frame matches scene");
    TEST_ASSERT(frame.step == 7, "step stamped");
    TEST_NEAR(frame.bodies[0].linear_velocity[2], 3.0, 1e-12, "COM velocity");
    TEST_NEAR(frame.bodies[0].angular_velocity[2], 0.5, 1e-12, "angular velocity world");
    TEST_NEAR(frame.bodies[0].pose.position[0], 10.0, 1e-12, "pose position");
    TEST_NEAR(frame.bodies[2].pose.position[2], 5.0, 1e-12, "aux body: pose is the reference frame");
    TEST_NEAR(frame.bodies[2].com_position[2], 4.0, 1e-12, "aux body: COM separate from pose");
    TEST_NEAR(frame.mooring_lines[0].node_tensions[2], 300.0, 1e-12, "mooring tension");

    // Frames follow the system; the extractor itself never changes it.
    const double t_before = system.GetChTime();
    hull->SetPos({11.0, 0.0, 0.0});
    line_z = -12.0;
    frame = extractor.Capture(8);
    TEST_NEAR(frame.bodies[0].pose.position[0], 11.0, 1e-12, "capture tracks body motion");
    TEST_NEAR(frame.mooring_lines[0].node_positions[1][2], -12.0, 1e-12, "capture tracks mooring");
    TEST_NEAR(system.GetChTime(), t_before, 0.0, "capture does not advance time");

    // TSDA with a SEGMENT visual is exported as a 2-node overlay line after
    // MoorDyn lines; a TSDA without a visual is ignored (station-keeping).
    {
        ::chrono::ChSystemNSC cable_sys;
        auto a = chrono_types::make_shared<::chrono::ChBody>();
        a->SetName("a");
        a->SetPos({0.0, 0.0, 10.0});
        a->SetFixed(true);
        cable_sys.AddBody(a);
        auto b = chrono_types::make_shared<::chrono::ChBody>();
        b->SetName("b");
        b->SetPos({0.0, 0.0, 0.0});
        b->SetFixed(true);
        cable_sys.AddBody(b);

        auto hoist = chrono_types::make_shared<::chrono::ChLinkTSDA>();
        hoist->SetName("hoist");
        hoist->Initialize(a, b, false, {0.0, 0.0, 10.0}, {0.0, 0.0, 0.0});
        hoist->AddVisualShape(chrono_types::make_shared<::chrono::ChVisualShapeSegment>());
        cable_sys.AddLink(hoist);

        auto hidden = chrono_types::make_shared<::chrono::ChLinkTSDA>();
        hidden->SetName("station_keeping");
        hidden->Initialize(a, b, false, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
        cable_sys.AddLink(hidden);

        VizExtractor cable_ex(&cable_sys);
        TEST_ASSERT(cable_ex.Scene().cable_lines.size() == 1, "only visualised TSDA becomes a cable line");
        TEST_ASSERT(cable_ex.Scene().cable_lines[0].kind == CableKind::kSegment, "SEGMENT glyph");
        TEST_ASSERT(cable_ex.Scene().mooring_lines.empty(), "TSDA cables are not MoorDyn lines");
        VizFrame cable_frame = cable_ex.Capture(0);
        TEST_ASSERT(IsConsistent(cable_ex.Scene(), cable_frame), "cable frame matches scene");
        TEST_NEAR(cable_frame.cable_lines[0].point1[2], 10.0, 1e-12, "cable endpoint 1");
        TEST_NEAR(cable_frame.cable_lines[0].point2[2], 0.0, 1e-12, "cable endpoint 2");

        b->SetPos({0.0, 0.0, -2.0});
        // Re-initialise absolute points after moving a fixed body, as Chrono
        // would during a step for a free payload.
        hoist->Initialize(a, b, false, {0.0, 0.0, 10.0}, {0.0, 0.0, -2.0});
        cable_frame = cable_ex.Capture(1);
        TEST_NEAR(cable_frame.cable_lines[0].point2[2], -2.0, 1e-12, "cable tracks endpoint motion");

        TEST_ASSERT(seastack::chrono::FindUndrawableItems(cable_sys).empty(), "SEGMENT TSDA is drawable");
    }

    // Undrawable content is reported so the GUI can fall back to the Chrono viewer.
    TEST_ASSERT(seastack::chrono::FindUndrawableItems(system).empty(), "box, sphere and mesh are drawable");
    {
        ::chrono::ChSystemNSC odd_sys;
        auto body = chrono_types::make_shared<::chrono::ChBody>();
        body->SetName("buoy");
        body->AddVisualShape(chrono_types::make_shared<::chrono::ChVisualShapeSphere>(1.0));
        body->AddVisualShape(chrono_types::make_shared<::chrono::ChVisualShapeCapsule>(0.5, 2.0));
        odd_sys.AddBody(body);
        auto ground = chrono_types::make_shared<::chrono::ChBody>();
        ground->SetFixed(true);
        odd_sys.AddBody(ground);
        auto marker = chrono_types::make_shared<::chrono::ChLinkTSDA>();
        marker->SetName("marker");
        marker->Initialize(body, ground, false, {0.0, 0.0, 0.0}, {0.0, 0.0, -5.0});
        marker->AddVisualShape(chrono_types::make_shared<::chrono::ChVisualShapeSphere>(0.1));
        odd_sys.AddLink(marker);

        const std::vector<std::string> items = seastack::chrono::FindUndrawableItems(odd_sys);
        TEST_ASSERT(items.size() == 2, "capsule and non-glyph link visual reported");
        TEST_ASSERT(items.size() == 2 && items[0].find("body 'buoy'") != std::string::npos &&
                        items[0].find("Capsule") != std::string::npos,
                    "body item names the body and shape type");
        TEST_ASSERT(items.size() == 2 && items[1].find("link 'marker'") != std::string::npos,
                    "link item names the link");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
