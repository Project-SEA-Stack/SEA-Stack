// Round trip of the HDF5 replay data: SimulationExporter writes /visualization
// and the per-step series, LoadReplayRecording reads them back. The replayed
// frames must match what the live viewer would have drawn (VizExtractor).

#include <seastack/adapters/chrono/simulation_export.h>
#include <seastack/adapters/chrono/visualization_recording.h>
#include <seastack/adapters/chrono/viz_extractor.h>
#include <seastack/core/mooring_viz_data.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>

#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapePointPoint.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyAuxRef.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystemNSC.h>

#include "test_macros.h"

#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using namespace seastack::viz;

namespace {

std::vector<MooringLineVizData> FakeMooring(double t) {
    MooringLineVizData line;
    line.node_positions = {{0.0, 0.0, -20.0}, {5.0, 0.0, -10.0 + t}, {10.0, 0.0, 0.0}};
    line.node_tensions = {100.0, 200.0 + t, 300.0};
    line.line_max_tension = 300.0;
    line.start_point_type = MooringPointType::kFixed;
    line.end_point_type = MooringPointType::kCoupled;
    return {line};
}

double QuatDot(const Quat& a, const Quat& b) {
    return std::abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
}

}  // namespace

int main() {
    TestResults test_results;
    const std::string path =
        (std::filesystem::temp_directory_path() / "seastack_test_visualization_recording.h5").string();

    ::chrono::ChSystemNSC system;
    system.SetGravitationalAcceleration({0.0, 0.0, -9.81});

    // Spinning body whose reference (visual) frame is offset and rotated from
    // its centre of mass, so the COM -> visual mapping is exercised.
    auto hull = chrono_types::make_shared<::chrono::ChBodyAuxRef>();
    hull->SetName("hull");
    hull->SetFrameRefToAbs(::chrono::ChFramed(::chrono::ChVector3d(1.0, 2.0, 3.0), ::chrono::QuatFromAngleZ(0.3)));
    hull->SetFrameCOMToRef(
        ::chrono::ChFramed(::chrono::ChVector3d(0.5, 0.0, -1.0), ::chrono::QuatFromAngleX(0.4)));
    hull->SetAngVelParent({0.2, 0.5, 1.0});
    auto box = chrono_types::make_shared<::chrono::ChVisualShapeBox>(2.0, 4.0, 6.0);
    box->SetColor(::chrono::ChColor(0.2f, 0.3f, 0.4f));
    hull->AddVisualShape(box, ::chrono::ChFramed(::chrono::ChVector3d(1.0, 0.0, 0.0)));
    system.AddBody(hull);

    auto anchor = chrono_types::make_shared<::chrono::ChBody>();
    anchor->SetName("anchor");
    anchor->SetFixed(true);
    system.AddBody(anchor);

    auto cable = chrono_types::make_shared<::chrono::ChLinkTSDA>();
    cable->Initialize(anchor, hull, false, {0.0, 0.0, -5.0}, {1.0, 2.0, 3.0});
    cable->SetSpringCoefficient(100.0);
    cable->AddVisualShape(chrono_types::make_shared<::chrono::ChVisualShapeSegment>());
    system.AddLink(cable);

    // Two-component wave field, as the hydro builder would make it.
    std::vector<seastack::hydro::WaveComponent> components(2);
    components[0] = {0.8, 0.8 * 0.8 / 9.81, 0.0, 1.0, 0.3};
    components[1] = {1.3, 1.3 * 1.3 / 9.81, 0.5, 0.4, 1.1};
    seastack::hydro::LinearDirectionalWaveField wave(components, 0.0);

    seastack::chrono::VizExtractor extractor(&system);
    const VizFrame input = extractor.Capture(0);
    std::vector<VizFrame> live;
    constexpr double kDt = 0.01;    // [s]
    constexpr int kSteps = 50;
    {
        seastack::chrono::SimulationExporter::Options options;
        options.output_path = path;
        options.log_final_output_path = false;
        seastack::chrono::SimulationExporter exporter(options);
        exporter.WriteSimulationInfo(&system, "", "test", kDt, kSteps * kDt);
        exporter.WriteModel(&system);
        exporter.BeginResults(&system, kSteps);
        std::vector<double> om, kk, am, ph, dir;
        for (const auto& c : components) {
            om.push_back(c.omega);
            kk.push_back(c.k);
            am.push_back(c.amplitude);
            ph.push_back(c.phase);
            dir.push_back(c.direction);
        }
        exporter.WriteWaveComponents(om, kk, am, ph, dir);
        exporter.WriteVisualization(&system, 5.0, 0.0, 9.81, false);
        for (int i = 0; i < kSteps; ++i) {
            system.DoStepDynamics(kDt);
            exporter.RecordStep(&system);
            exporter.RecordMooringLines(FakeMooring(system.GetChTime()));
            live.push_back(extractor.Capture(i + 1));
        }
        exporter.Finalize();
    }

    seastack::chrono::ReplayRecording rec;
    std::string error;
    TEST_ASSERT(seastack::chrono::LoadReplayRecording(path, size_t{64} << 20, &rec, &error), "load: " + error);
    TEST_ASSERT(rec.schema_version == "1.1", "schema version 1.1");
    TEST_ASSERT(!rec.viewer_colors, "headless colours flagged as model colours");
    TEST_ASSERT(rec.frames.size() == live.size() && rec.frame_stride == 1, "one frame per output sample");
    TEST_ASSERT(rec.scene.bodies.size() == 2 && rec.scene.bodies[0].name == "hull", "bodies and names");
    TEST_ASSERT(rec.scene.bodies[0].shapes.size() == 1, "shape count");
    if (!rec.scene.bodies[0].shapes.empty()) {
        const VisualShape& shape = rec.scene.bodies[0].shapes[0];
        TEST_ASSERT(shape.kind == ShapeKind::kBox, "box kind");
        TEST_NEAR(shape.size[2], 6.0, 1e-12, "box size");
        TEST_NEAR(shape.local.position[0], 1.0, 1e-12, "shape offset");
        TEST_NEAR(shape.color[1], 0.3f, 1e-6, "shape colour");
    }
    TEST_NEAR(rec.scene.bodies[0].local_bounds.max[0], 2.0, 1e-12, "local bounds rebuilt");
    TEST_NEAR(rec.input_frame.time, 0.0, 1e-12, "input time");
    TEST_NEAR(rec.input_frame.bodies[0].pose.position[2], input.bodies[0].pose.position[2], 1e-12,
              "input pose = state before the first step");

    double max_pos_err = 0.0;
    double min_quat_dot = 1.0;
    double max_com_err = 0.0;
    double max_cable_err = 0.0;
    for (size_t f = 0; f < rec.frames.size() && f < live.size(); ++f) {
        const BodyState& a = rec.frames[f].bodies[0];
        const BodyState& b = live[f].bodies[0];
        for (int k = 0; k < 3; ++k) {
            max_pos_err = std::max(max_pos_err, std::abs(a.pose.position[k] - b.pose.position[k]));
            max_com_err = std::max(max_com_err, std::abs(a.com_position[k] - b.com_position[k]));
            max_cable_err = std::max(max_cable_err, std::abs(rec.frames[f].cable_lines[0].point2[k] -
                                                             live[f].cable_lines[0].point2[k]));
        }
        min_quat_dot = std::min(min_quat_dot, QuatDot(a.pose.rotation, b.pose.rotation));
    }
    TEST_NEAR(rec.frames.back().time, live.back().time, 1e-12, "frame times");
    TEST_ASSERT(max_pos_err < 1e-9, "replayed visual-frame positions match live (COM -> visual offset)");
    TEST_ASSERT(min_quat_dot > 1.0 - 1e-12, "replayed visual-frame rotations match live");
    TEST_ASSERT(max_com_err < 1e-9, "replayed COM positions match live");
    TEST_ASSERT(rec.scene.cable_lines.size() == 1 && max_cable_err < 1e-9, "cable endpoints round trip");
    TEST_ASSERT(std::abs(live.back().bodies[0].pose.position[2] - input.bodies[0].pose.position[2]) > 1e-3,
                "test body actually moved");

    TEST_ASSERT(rec.scene.mooring_lines.size() == 1 && rec.scene.mooring_lines[0].num_nodes == 3, "mooring topology");
    if (!rec.scene.mooring_lines.empty()) {
        TEST_ASSERT(rec.scene.mooring_lines[0].start_type == MooringEndType::kFixed &&
                        rec.scene.mooring_lines[0].end_type == MooringEndType::kCoupled,
                    "mooring end types");
        const double t_last = rec.frames.back().time;
        TEST_NEAR(rec.frames.back().mooring_lines[0].node_positions[1][2], -10.0 + t_last, 1e-9, "mooring nodes");
        TEST_NEAR(rec.frames.back().mooring_lines[0].node_tensions[1], 200.0 + t_last, 1e-9, "mooring tensions");
    }

    TEST_ASSERT(rec.wave != nullptr, "wave rebuilt from components");
    if (rec.wave) {
        const Eigen::Vector3d p(3.0, -2.0, 0.0);
        TEST_NEAR(rec.wave->GetElevation(p, 7.3), wave.GetElevation(p, 7.3), 1e-12, "wave elevation round trip");
        TEST_NEAR(rec.wave->GetRampDuration(), 5.0, 1e-12, "wave ramp duration");
    }

    // A small memory budget thins the record but keeps the final state.
    seastack::chrono::ReplayRecording thin;
    TEST_ASSERT(seastack::chrono::LoadReplayRecording(path, 20 * 1024, &thin, &error), "thinned load");
    TEST_ASSERT(thin.frame_stride > 1 && thin.frames.size() < live.size(), "long record thinned");
    TEST_NEAR(thin.frames.back().time, live.back().time, 1e-12, "thinned record ends at the final state");

    std::error_code ec;
    std::filesystem::remove(path, ec);
    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
