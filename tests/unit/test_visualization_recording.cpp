// Round trip of the HDF5 replay data: SimulationExporter writes /visualization
// and the per-step series, LoadReplayRecording reads them back. The replayed
// frames must match what the live viewer would have drawn (VizExtractor).

#include <seastack/adapters/chrono/simulation_export.h>
#include <seastack/adapters/chrono/visualization_recording.h>
#include <seastack/adapters/chrono/viz_extractor.h>
#include <seastack/core/mooring_viz_data.h>
#include <seastack/hydro/waves/amplitude_envelope.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>

#include <H5Cpp.h>

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

    // Excitation acts on both bodies; radiation on the hull only (the anchor's
    // is zero, so the file omits it).
    auto fake_forces = [&system, hull, anchor]() {
        const double t = system.GetChTime();
        seastack::chrono::ForceSample sample;
        sample.sources = {"Excitation", "Radiation"};
        sample.bodies = {hull.get(), anchor.get()};
        sample.wrenches.resize(4);
        sample.wrenches[0].force = {t, 2.0 * t, -3.0 * t};
        sample.wrenches[0].moment = {0.5 * t, 0.0, 1.0};
        sample.wrenches[1].force = {0.0, 0.0, 5.0};
        sample.wrenches[2].force = {0.0, 0.0, -10.0 * t};
        return sample;
    };

    seastack::chrono::VizExtractor extractor(&system);
    extractor.SetForceProvider(fake_forces);
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
        exporter.RecordVisualizationForces(fake_forces);
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
        // Standard output: geometry every 0.1 s (t = 0.01, 0.11, ..., 0.41) in float32;
        // a frame shows the latest sample at or before its time.
        TEST_NEAR(rec.frames.back().mooring_lines[0].node_positions[1][2], -10.0 + 0.41, 1e-5, "mooring nodes");
        TEST_NEAR(rec.frames.back().mooring_lines[0].node_tensions[1], 200.0 + 0.41, 1e-4, "mooring tensions");
        TEST_NEAR(rec.frames[14].mooring_lines[0].node_tensions[1], 200.0 + 0.11, 1e-4,
                  "frame at t = 0.15 s shows the sample at 0.11 s");
    }

    TEST_ASSERT(rec.scene.force_sources == std::vector<std::string>({"Excitation", "Radiation"}),
                "force source names round trip");
    double max_wrench_err = 0.0;
    bool sizes_ok = true;
    for (size_t f = 0; f < rec.frames.size() && f < live.size(); ++f) {
        const auto& a = rec.frames[f].body_wrenches;
        const auto& b = live[f].body_wrenches;
        sizes_ok = sizes_ok && a.size() == 4 && b.size() == 4;
        for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
            for (int k = 0; k < 3; ++k) {
                // Relative to max(1, |live|): loads are stored as float32.
                max_wrench_err = std::max(max_wrench_err, std::abs(a[i].force[k] - b[i].force[k]) /
                                                              std::max(1.0, std::abs(b[i].force[k])));
                max_wrench_err = std::max(max_wrench_err, std::abs(a[i].moment[k] - b[i].moment[k]) /
                                                              std::max(1.0, std::abs(b[i].moment[k])));
            }
        }
    }
    TEST_ASSERT(sizes_ok, "every frame has sources x bodies wrenches");
    TEST_ASSERT(max_wrench_err < 1e-6, "replayed loads match live to float32, including zero loads on the anchor");
    TEST_NEAR(rec.frames.back().body_wrenches[1 * 2 + 0].force[2], -10.0 * rec.frames.back().time, 1e-5,
              "radiation on the hull at the last frame");
    TEST_NEAR(rec.frames.back().body_wrenches[0 * 2 + 1].force[2], 5.0, 1e-12,
              "excitation on the anchor (second body of one source)");

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
    TEST_ASSERT(rec.wave_note.empty(), "no note for a run without sea changes");

    // ---- Live sea changes: replay draws the sea the run applied ----
    // A height ramp, a crossfade to the same component count (phases matched)
    // and one to a different count, recorded as single_run does.
    const std::string path_changes =
        (std::filesystem::temp_directory_path() / "seastack_test_visualization_recording_waves.h5").string();
    const std::string path_old =
        (std::filesystem::temp_directory_path() / "seastack_test_visualization_recording_old.h5").string();
    seastack::hydro::LinearDirectionalWaveField live_wave(components, 0.0);
    live_wave.SetRampDuration(0.8);
    std::vector<double> live_eta;
    auto record_run = [&](const std::string& out_path, bool with_changes) {
        ::chrono::ChSystemNSC sys;
        auto body = chrono_types::make_shared<::chrono::ChBody>();
        body->SetName("float");
        body->AddVisualShape(chrono_types::make_shared<::chrono::ChVisualShapeBox>(1.0, 1.0, 1.0));
        sys.AddBody(body);
        seastack::chrono::SimulationExporter::Options options;
        options.output_path = out_path;
        options.log_final_output_path = false;
        seastack::chrono::SimulationExporter exporter(options);
        exporter.WriteSimulationInfo(&sys, "", "test", kDt, 3.0);
        exporter.WriteModel(&sys);
        exporter.BeginResults(&sys, 300);
        std::vector<double> om, kk, am, ph, dir;
        for (const auto& c : components) {
            om.push_back(c.omega);
            kk.push_back(c.k);
            am.push_back(c.amplitude);
            ph.push_back(c.phase);
            dir.push_back(c.direction);
        }
        exporter.WriteWaveComponents(om, kk, am, ph, dir);
        exporter.WriteVisualization(&sys, 0.8, 0.0, 9.81, false);
        auto crossfade = [&](std::vector<seastack::hydro::WaveComponent> next, double t, double duration,
                             const char* kind) {
            const size_t n_new = next.size();
            live_wave.CrossfadeTo(std::move(next), t, duration);
            seastack::chrono::WaveChangeRecord change;
            change.start_time_s = t;
            change.duration_s = duration;
            change.kind = kind;
            change.parameter = std::string("waves.") + kind;
            const auto& all = *live_wave.GetWaveComponents();
            for (size_t i = all.size() - n_new; i < all.size(); ++i) {
                change.omega_rad_s.push_back(all[i].omega);
                change.k_rad_m.push_back(all[i].k);
                change.amplitude_m.push_back(all[i].amplitude);
                change.phase_rad.push_back(all[i].phase);
                change.direction_rad.push_back(all[i].direction);
            }
            exporter.AddWaveChange(std::move(change));
        };
        for (int i = 0; i < 300; ++i) {
            const double t = sys.GetChTime();
            if (with_changes && i == 50) {
                live_wave.GetAmplitudeEnvelope()->RampTo(t, 1.5, 0.4);
                seastack::chrono::WaveChangeRecord change;
                change.start_time_s = t;
                change.duration_s = 0.4;
                change.kind = "height";
                change.parameter = "waves.height";
                change.value = 1.5;
                change.envelope_target = 1.5;
                exporter.AddWaveChange(std::move(change));
            }
            if (with_changes && i == 100) {
                std::vector<seastack::hydro::WaveComponent> turned = components;
                for (auto& c : turned) c.direction += 0.5;
                crossfade(turned, t, 0.6, "direction");
            }
            if (with_changes && i == 200) {
                std::vector<seastack::hydro::WaveComponent> three = components;
                three.push_back({1.9, 0.0, 0.2, 0.3, 0.7});
                for (auto& c : three) c.omega *= 0.9;
                crossfade(three, t, 0.5, "period");
            }
            sys.DoStepDynamics(kDt);
            exporter.RecordStep(&sys);
            const double t1 = sys.GetChTime();
            const double eta = live_wave.GetExcitationRampForVisualization(t1) *
                               live_wave.GetElevation(Eigen::Vector3d::Zero(), t1);
            exporter.RecordIncidentElevation(eta);
            if (with_changes) live_eta.push_back(eta);
        }
        if (!with_changes) {
            // An older file: the event log names a sea change, /events/waves is absent.
            exporter.SetRuntimeEvents({50}, {0.5}, {"waves.height"}, {1.5}, {1.0}, {"schedule"});
        }
        exporter.Finalize();
    };
    record_run(path_changes, true);
    seastack::chrono::ReplayRecording waves_rec;
    TEST_ASSERT(seastack::chrono::LoadReplayRecording(path_changes, size_t{64} << 20, &waves_rec, &error),
                "load with sea changes: " + error);
    TEST_ASSERT(waves_rec.wave != nullptr && waves_rec.wave_note.empty(),
                "sea changes applied without a note: " + waves_rec.wave_note);
    if (waves_rec.wave) {
        double max_err = 0.0;
        double max_eta = 0.0;
        for (const Eigen::Vector3d p : {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(3.0, -2.0, 0.0)}) {
            for (double t = 0.0; t <= 3.0; t += 0.0137) {
                const double a = waves_rec.wave->GetExcitationRampForVisualization(t) *
                                 waves_rec.wave->GetElevation(p, t);
                const double b = live_wave.GetExcitationRampForVisualization(t) * live_wave.GetElevation(p, t);
                max_err = std::max(max_err, std::abs(a - b));
                max_eta = std::max(max_eta, std::abs(b));
            }
        }
        TEST_ASSERT(max_err == 0.0, "replayed elevation equals the live elevation (max error " +
                                        std::to_string(max_err) + " m)");
        TEST_ASSERT(max_eta > 0.5, "the live sea is not flat");
        TEST_ASSERT(waves_rec.wave->GetWaveComponents()->size() == 7, "two crossfades: 2 + 2 + 3 components");
        // Without the changes the elevation differs: the check has teeth.
        seastack::hydro::LinearDirectionalWaveField initial(components, 0.0);
        initial.SetRampDuration(0.8);
        const double t_late = 2.9;
        TEST_ASSERT(std::abs(initial.GetExcitationRampForVisualization(t_late) *
                                 initial.GetElevation(Eigen::Vector3d::Zero(), t_late) -
                             live_wave.GetExcitationRampForVisualization(t_late) *
                                 live_wave.GetElevation(Eigen::Vector3d::Zero(), t_late)) > 1e-3,
                    "the changed sea differs from the initial one");
    }
    {
        H5::H5File file(path_changes, H5F_ACC_RDONLY);
        H5::DataSet ds = file.openDataSet("/events/waves/free_surface_eta");
        hsize_t n = 0;
        ds.getSpace().getSimpleExtentDims(&n);
        std::vector<double> eta(n);
        ds.read(eta.data(), H5::PredType::NATIVE_DOUBLE);
        TEST_ASSERT(eta == live_eta, "/events/waves/free_surface_eta is the elevation recorded each step");
        double changed = 0.0;
        file.openGroup("/inputs/simulation/waves").openAttribute("changed_during_run")
            .read(H5::PredType::NATIVE_DOUBLE, &changed);
        TEST_ASSERT(changed == 1.0, "/inputs/simulation/waves flagged changed_during_run = 1");
        TEST_ASSERT(file.openGroup("/events/waves").nameExists("change_2/components") &&
                        !file.openGroup("/events/waves").nameExists("change_0/components"),
                    "components recorded for the crossfades, not for the height change");
    }

    record_run(path_old, false);
    seastack::chrono::ReplayRecording old_rec;
    TEST_ASSERT(seastack::chrono::LoadReplayRecording(path_old, size_t{64} << 20, &old_rec, &error),
                "load older file: " + error);
    TEST_ASSERT(old_rec.wave != nullptr && old_rec.wave->GetWaveComponents()->size() == 2,
                "older file: the initial sea, as before");
    TEST_ASSERT(old_rec.wave_note.find("initial sea throughout") != std::string::npos,
                "older file: note says the changes are not drawn: " + old_rec.wave_note);

    // ---- Mooring geometry by output level (300 steps of 0.01 s) ----
    // One file per level: H5Writer leaves group handles open, so HDF5 keeps
    // a written file open (and locked) until the process exits.
    std::vector<std::string> level_paths;
    struct LevelCase {
        seastack::chrono::ExportLevel level;
        const char* name;
        hsize_t samples;  // node geometry samples; 0 = not recorded
        size_t bytes;     // per stored value
    };
    for (const LevelCase& c : {LevelCase{seastack::chrono::ExportLevel::kCompact, "compact", 0, 0},
                               LevelCase{seastack::chrono::ExportLevel::kStandard, "standard", 30, 4},
                               LevelCase{seastack::chrono::ExportLevel::kDetailed, "detailed", 300, 8}}) {
        const std::string path_level = (std::filesystem::temp_directory_path() /
                                        (std::string("seastack_test_visualization_recording_") + c.name + ".h5"))
                                           .string();
        level_paths.push_back(path_level);
        {
            ::chrono::ChSystemNSC sys;
            auto body = chrono_types::make_shared<::chrono::ChBody>();
            body->SetName("float");
            body->AddVisualShape(chrono_types::make_shared<::chrono::ChVisualShapeBox>(1.0, 1.0, 1.0));
            sys.AddBody(body);
            seastack::chrono::SimulationExporter::Options options;
            options.output_path = path_level;
            options.log_final_output_path = false;
            options.export_config.level = c.level;
            seastack::chrono::SimulationExporter exporter(options);
            exporter.WriteSimulationInfo(&sys, "", "test", kDt, 3.0);
            exporter.WriteModel(&sys);
            exporter.BeginResults(&sys, 300);
            exporter.WriteVisualization(&sys, 0.0, 0.0, 9.81, false);
            for (int i = 0; i < 300; ++i) {
                sys.DoStepDynamics(kDt);
                exporter.RecordStep(&sys);
                auto lines = FakeMooring(sys.GetChTime());
                // A snatch between geometry samples must still reach the peak statistics.
                if (i == 154) lines[0].line_max_tension = 9000.0;
                exporter.RecordMooringLines(lines);
            }
            exporter.Finalize();
        }
        const std::string tag = std::string(c.name) + ": ";
        {
            H5::H5File file(path_level, H5F_ACC_RDONLY);
            const H5::Group line = file.openGroup("/results/mooring/line_1");
            double peak = 0.0;
            line.openAttribute("peak_tension_N").read(H5::PredType::NATIVE_DOUBLE, &peak);
            TEST_NEAR(peak, 9000.0, 0.0, tag + "peak tension at full rate");
            hsize_t n_max = 0;
            line.openDataSet("line_max_tension").getSpace().getSimpleExtentDims(&n_max);
            TEST_ASSERT(n_max == 300, tag + "line_max_tension at every output step");
            TEST_ASSERT(line.nameExists("node_positions") == (c.samples > 0), tag + "node geometry present by level");
            if (c.samples > 0) {
                const H5::DataSet pos = line.openDataSet("node_positions");
                hsize_t dims[3] = {0, 0, 0};
                const int rank = pos.getSpace().getSimpleExtentDims(dims);
                TEST_ASSERT(rank == 3 && dims[0] == c.samples && dims[1] == 3 && dims[2] == 3,
                            tag + "node_positions shape [" + std::to_string(dims[0]) + ", " + std::to_string(dims[1]) +
                                ", " + std::to_string(dims[2]) + "]");
                TEST_ASSERT(pos.getDataType().getSize() == c.bytes, tag + "node_positions dtype");
                hsize_t n_t = 0;
                line.openDataSet("sample_time").getSpace().getSimpleExtentDims(&n_t);
                hsize_t dims_t[2] = {0, 0};
                line.openDataSet("node_tensions").getSpace().getSimpleExtentDims(dims_t);
                TEST_ASSERT(n_t == c.samples && dims_t[0] == c.samples && dims_t[1] == 3,
                            tag + "sample_time and node_tensions match");
            }
        }
        seastack::chrono::ReplayRecording level_rec;
        TEST_ASSERT(seastack::chrono::LoadReplayRecording(path_level, size_t{64} << 20, &level_rec, &error),
                    tag + "replay loads: " + error);
        TEST_ASSERT(level_rec.scene.mooring_lines.size() == (c.samples > 0 ? 1u : 0u) &&
                        level_rec.mooring_note.empty() == (c.samples > 0),
                    tag + "replay draws the lines it has, with a note otherwise: " + level_rec.mooring_note);
        if (c.samples > 0 && !level_rec.frames.empty()) {
            // Frame 154 is t = 1.55 s: standard shows the 1.51 s sample, detailed its own.
            const double expected = c.samples == 300 ? 200.0 + 1.55 : 200.0 + 1.51;
            TEST_NEAR(level_rec.frames[154].mooring_lines[0].node_tensions[1], expected, 1e-4,
                      tag + "replay tension at t = 1.55 s");
        }
    }

    std::error_code ec;
    std::filesystem::remove(path, ec);
    std::filesystem::remove(path_changes, ec);
    std::filesystem::remove(path_old, ec);
    for (const std::string& p : level_paths) {
        std::filesystem::remove(p, ec);
    }
    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
