/**
 * @file visualization_recording.cpp
 * @brief Implementation of LoadReplayRecording (see visualization_recording.h).
 */

#include <seastack/adapters/chrono/visualization_recording.h>

#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/viz_core/frame_history.h>

#ifdef SEASTACK_HAVE_HYDRO_IO
#include <H5Cpp.h>
#endif

#include <algorithm>
#include <exception>
#include <sstream>

namespace seastack::chrono {

#ifdef SEASTACK_HAVE_HYDRO_IO

namespace {

/// True if every component of an absolute path exists (H5Lexists needs the parents).
bool PathExists(const H5::H5File& file, const std::string& path) {
    std::string partial;
    std::stringstream ss(path);
    std::string part;
    while (std::getline(ss, part, '/')) {
        if (part.empty()) {
            continue;
        }
        partial += "/" + part;
        if (H5Lexists(file.getId(), partial.c_str(), H5P_DEFAULT) <= 0) {
            return false;
        }
    }
    return true;
}

/// Whole dataset as doubles (float32 datasets are converted), row-major.
std::vector<double> ReadDoubles(const H5::H5File& file, const std::string& path) {
    H5::DataSet ds = file.openDataSet(path);
    const hssize_t n = ds.getSpace().getSimpleExtentNpoints();
    std::vector<double> data(static_cast<size_t>(std::max<hssize_t>(n, 0)));
    if (!data.empty()) {
        ds.read(data.data(), H5::PredType::NATIVE_DOUBLE);
    }
    return data;
}

double ReadDoubleAttr(const H5::H5Object& obj, const std::string& name, double fallback) {
    if (!obj.attrExists(name)) {
        return fallback;
    }
    double value = fallback;
    obj.openAttribute(name).read(H5::PredType::NATIVE_DOUBLE, &value);
    return value;
}

std::string ReadStringAttr(const H5::H5Object& obj, const std::string& name) {
    if (!obj.attrExists(name)) {
        return {};
    }
    H5::Attribute attr = obj.openAttribute(name);
    std::string value;
    attr.read(attr.getStrType(), value);
    return value;
}

viz::Vec3 ToVec3(const std::vector<double>& v, size_t offset = 0) {
    if (v.size() < offset + 3) {
        return {0.0, 0.0, 0.0};
    }
    return {v[offset], v[offset + 1], v[offset + 2]};
}

viz::Quat ToQuat(const std::vector<double>& v, size_t offset = 0) {
    if (v.size() < offset + 4) {
        return viz::kIdentityQuat;
    }
    return {v[offset], v[offset + 1], v[offset + 2], v[offset + 3]};
}

viz::Pose ReadPose(const H5::H5File& file, const std::string& prefix) {
    viz::Pose pose;
    pose.position = ToVec3(ReadDoubles(file, prefix + "_position"));
    pose.rotation = ToQuat(ReadDoubles(file, prefix + "_orientation"));
    return pose;
}

viz::ShapeKind ShapeKindFromName(const std::string& name) {
    if (name == "sphere") return viz::ShapeKind::kSphere;
    if (name == "cylinder") return viz::ShapeKind::kCylinder;
    if (name == "mesh") return viz::ShapeKind::kMesh;
    return viz::ShapeKind::kBox;
}

viz::VisualShape ReadShape(const H5::H5File& file, const std::string& path) {
    const H5::Group g = file.openGroup(path);
    viz::VisualShape shape;
    shape.kind = ShapeKindFromName(ReadStringAttr(g, "kind"));
    shape.local = ReadPose(file, path + "/local");
    shape.size = ToVec3(ReadDoubles(file, path + "/size"));
    const std::vector<double> color = ReadDoubles(file, path + "/color");
    for (size_t i = 0; i < 4 && i < color.size(); ++i) {
        shape.color[i] = static_cast<float>(color[i]);
    }
    shape.roughness = static_cast<float>(ReadDoubleAttr(g, "roughness", shape.roughness));
    shape.metallic = static_cast<float>(ReadDoubleAttr(g, "metallic", shape.metallic));
    if (shape.kind == viz::ShapeKind::kMesh) {
        auto mesh = std::make_shared<viz::TriangleMesh>();
        const std::vector<double> vertices = ReadDoubles(file, path + "/vertices");
        const std::vector<double> faces = ReadDoubles(file, path + "/faces");
        for (size_t i = 0; i + 2 < vertices.size(); i += 3) {
            mesh->vertices.push_back({vertices[i], vertices[i + 1], vertices[i + 2]});
        }
        for (size_t i = 0; i + 2 < faces.size(); i += 3) {
            mesh->faces.push_back({static_cast<int>(faces[i]), static_cast<int>(faces[i + 1]),
                                   static_cast<int>(faces[i + 2])});
        }
        if (PathExists(file, path + "/corner_normals")) {
            const std::vector<double> normals = ReadDoubles(file, path + "/corner_normals");
            for (size_t i = 0; i + 2 < normals.size(); i += 3) {
                mesh->corner_normals.push_back({normals[i], normals[i + 1], normals[i + 2]});
            }
        }
        shape.mesh = std::move(mesh);
    }
    return shape;
}

/// Per-body data needed to build frames.
struct BodyTrack {
    std::string results_group;
    viz::Pose visual_in_com;  ///< Visual (shape) frame relative to the centre-of-mass frame
};

std::shared_ptr<hydro::WaveBase> ReadWave(const H5::H5File& file, double gravity, double depth,
                                          double ramp_duration, std::string* note) {
    const std::string base = "/inputs/simulation/waves/components";
    if (!PathExists(file, base + "/omega")) {
        *note = "No wave components in this file (still water or a tabulated elevation record): "
                "the water is drawn flat.";
        return nullptr;
    }
    const std::vector<double> omega = ReadDoubles(file, base + "/omega");
    auto read_or_zero = [&](const char* name) {
        std::vector<double> v = PathExists(file, base + "/" + name) ? ReadDoubles(file, base + "/" + name)
                                                                    : std::vector<double>{};
        v.resize(omega.size(), 0.0);
        return v;
    };
    const std::vector<double> k = read_or_zero("k");
    const std::vector<double> amplitude = read_or_zero("amplitude");
    const std::vector<double> phase = read_or_zero("phase");
    const std::vector<double> direction = read_or_zero("direction");
    std::vector<hydro::WaveComponent> components(omega.size());
    for (size_t i = 0; i < omega.size(); ++i) {
        components[i].omega = omega[i];
        components[i].k = k[i];
        components[i].amplitude = amplitude[i];
        components[i].phase = phase[i];
        components[i].direction = direction[i];
    }
    auto wave = std::make_shared<hydro::LinearDirectionalWaveField>(std::move(components), depth);
    wave->UpdateEnvironment(gravity, depth);
    wave->SetRampDuration(ramp_duration);
    return wave;
}

}  // namespace

bool LoadReplayRecording(const std::string& path, size_t budget_bytes, ReplayRecording* out,
                         std::string* error) {
    *out = ReplayRecording();
    try {
        H5::Exception::dontPrint();
        const H5::H5File file(path, H5F_ACC_RDONLY);

        if (PathExists(file, "/meta")) {
            out->schema_version = ReadStringAttr(file.openGroup("/meta"), "schema_version");
        }
        if (!PathExists(file, "/visualization/bodies")) {
            *error = path + " has no replay data (/visualization). It was written by an older SEA-Stack "
                     "(results schema " + (out->schema_version.empty() ? std::string("unknown") : out->schema_version) +
                     "; replay needs 1.1 or later). Run the case again with this version to record it.";
            return false;
        }
        if (!PathExists(file, "/results/time/time")) {
            *error = path + " has no recorded results (/results/time/time).";
            return false;
        }

        // ── Scene ────────────────────────────────────────────────────────────
        const H5::Group g_viz = file.openGroup("/visualization");
        viz::SceneDescription& scene = out->scene;
        scene.gravity = ReadDoubleAttr(g_viz, "gravity", scene.gravity);
        scene.water_depth = ReadDoubleAttr(g_viz, "water_depth", scene.water_depth);
        const double ramp_duration = ReadDoubleAttr(g_viz, "wave_ramp_duration", 0.0);
        out->viewer_colors = ReadStringAttr(g_viz, "colors") == "viewer";
        if (PathExists(file, "/visualization/model_bounds_min")) {
            scene.model_bounds.Extend(ToVec3(ReadDoubles(file, "/visualization/model_bounds_min")));
            scene.model_bounds.Extend(ToVec3(ReadDoubles(file, "/visualization/model_bounds_max")));
        }

        out->input_frame.time = ReadDoubleAttr(g_viz, "input_time", 0.0);
        out->input_frame.step = 0;

        std::vector<BodyTrack> tracks;
        const int body_count = static_cast<int>(ReadDoubleAttr(file.openGroup("/visualization/bodies"), "count", 0.0));
        for (int b = 0; b < body_count; ++b) {
            const std::string body_path = "/visualization/bodies/body_" + std::to_string(b);
            const H5::Group g_body = file.openGroup(body_path);
            viz::BodyDescription desc;
            desc.name = ReadStringAttr(g_body, "name");
            desc.fixed = ReadDoubleAttr(g_body, "fixed", 0.0) != 0.0;
            const int shape_count =
                static_cast<int>(ReadDoubleAttr(file.openGroup(body_path + "/shapes"), "count", 0.0));
            for (int s = 0; s < shape_count; ++s) {
                desc.shapes.push_back(ReadShape(file, body_path + "/shapes/shape_" + std::to_string(s)));
                viz::ExtendBounds(desc.shapes.back(), &desc.local_bounds);
            }
            scene.bodies.push_back(std::move(desc));

            BodyTrack track;
            track.results_group = ReadStringAttr(g_body, "results_group");
            track.visual_in_com = ReadPose(file, body_path + "/visual_frame_in_com");
            tracks.push_back(track);

            viz::BodyState input;
            input.pose = ReadPose(file, body_path + "/input_pose");
            input.com_position = ToVec3(ReadDoubles(file, body_path + "/input_com_position"));
            out->input_frame.bodies.push_back(input);
        }

        // ── Frame selection (thin long records to the memory budget) ────────
        const std::vector<double> time = ReadDoubles(file, "/results/time/time");
        const size_t n_samples = time.size();
        out->samples_in_file = n_samples;
        if (n_samples == 0) {
            *error = path + " has no recorded time samples.";
            return false;
        }

        // Mooring and cable topology first, so the frame size estimate is complete.
        std::vector<int> mooring_nodes;
        for (int li = 1; PathExists(file, "/results/mooring/line_" + std::to_string(li)); ++li) {
            const std::string line_path = "/results/mooring/line_" + std::to_string(li);
            viz::MooringLineDescription desc;
            desc.num_nodes = static_cast<int>(ReadDoubleAttr(file.openGroup(line_path), "num_nodes", 0.0));
            const std::string ends_path = "/visualization/mooring_lines/line_" + std::to_string(li);
            if (PathExists(file, ends_path)) {
                const H5::Group g_ends = file.openGroup(ends_path);
                desc.start_type = static_cast<viz::MooringEndType>(
                    static_cast<int>(ReadDoubleAttr(g_ends, "start_point_type", 0.0)));
                desc.end_type = static_cast<viz::MooringEndType>(
                    static_cast<int>(ReadDoubleAttr(g_ends, "end_point_type", 0.0)));
            }
            scene.mooring_lines.push_back(desc);
            mooring_nodes.push_back(desc.num_nodes);
        }
        if (PathExists(file, "/visualization/cables")) {
            const int cable_count =
                static_cast<int>(ReadDoubleAttr(file.openGroup("/visualization/cables"), "count", 0.0));
            for (int c = 0; c < cable_count; ++c) {
                const H5::Group g_cable = file.openGroup("/visualization/cables/cable_" + std::to_string(c));
                viz::CableLineDescription desc;
                desc.kind = ReadStringAttr(g_cable, "kind") == "spring" ? viz::CableKind::kSpring
                                                                        : viz::CableKind::kSegment;
                desc.spring_radius = ReadDoubleAttr(g_cable, "spring_radius", desc.spring_radius);
                desc.spring_resolution =
                    static_cast<int>(ReadDoubleAttr(g_cable, "spring_resolution", desc.spring_resolution));
                desc.spring_turns = ReadDoubleAttr(g_cable, "spring_turns", desc.spring_turns);
                scene.cable_lines.push_back(desc);
            }
        }

        viz::VizFrame prototype;
        prototype.bodies.resize(scene.bodies.size());
        prototype.mooring_lines.resize(scene.mooring_lines.size());
        for (size_t li = 0; li < mooring_nodes.size(); ++li) {
            prototype.mooring_lines[li].node_positions.resize(static_cast<size_t>(std::max(0, mooring_nodes[li])));
            prototype.mooring_lines[li].node_tensions.resize(static_cast<size_t>(std::max(0, mooring_nodes[li])));
        }
        prototype.cable_lines.resize(scene.cable_lines.size());

        const size_t capacity = viz::FrameCapacityForBudget(viz::EstimateFrameBytes(prototype), budget_bytes,
                                                            n_samples);
        const size_t stride = (n_samples + capacity - 1) / capacity;
        out->frame_stride = static_cast<int>(stride);
        std::vector<size_t> samples;
        for (size_t i = 0; i < n_samples; i += stride) {
            samples.push_back(i);
        }
        if (samples.back() != n_samples - 1) {
            samples.push_back(n_samples - 1);  // always end at the final state
        }
        out->frames.assign(samples.size(), prototype);
        for (size_t f = 0; f < samples.size(); ++f) {
            out->frames[f].time = time[samples[f]];
            out->frames[f].step = static_cast<int64_t>(samples[f]) + 1;
        }

        // ── Bodies: centre-of-mass results -> visual frame ───────────────────
        for (size_t b = 0; b < tracks.size(); ++b) {
            const std::string& group = tracks[b].results_group;
            if (group.empty() || !PathExists(file, group + "/position") || !PathExists(file, group + "/orientation")) {
                *error = path + ": body '" + scene.bodies[b].name + "' has no recorded position/orientation.";
                return false;
            }
            const std::vector<double> pos = ReadDoubles(file, group + "/position");
            const std::vector<double> quat = ReadDoubles(file, group + "/orientation");
            const std::vector<double> vel =
                PathExists(file, group + "/velocity") ? ReadDoubles(file, group + "/velocity") : std::vector<double>{};
            const std::vector<double> wvel = PathExists(file, group + "/angular_velocity")
                                                 ? ReadDoubles(file, group + "/angular_velocity")
                                                 : std::vector<double>{};
            for (size_t f = 0; f < samples.size(); ++f) {
                const size_t i = samples[f];
                viz::Pose com;
                com.position = ToVec3(pos, 3 * i);
                com.rotation = ToQuat(quat, 4 * i);
                viz::BodyState& state = out->frames[f].bodies[b];
                state.pose = viz::Compose(com, tracks[b].visual_in_com);
                state.com_position = com.position;
                state.linear_velocity = ToVec3(vel, 3 * i);
                state.angular_velocity = ToVec3(wvel, 3 * i);
            }
        }

        // ── Mooring lines ────────────────────────────────────────────────────
        for (size_t li = 0; li < mooring_nodes.size(); ++li) {
            const std::string line_path = "/results/mooring/line_" + std::to_string(li + 1);
            if (!PathExists(file, line_path + "/node_positions")) {
                continue;
            }
            const size_t nn = static_cast<size_t>(std::max(0, mooring_nodes[li]));
            const std::vector<double> positions = ReadDoubles(file, line_path + "/node_positions");
            const std::vector<double> tensions = ReadDoubles(file, line_path + "/node_tensions");
            for (size_t f = 0; f < samples.size(); ++f) {
                const size_t i = samples[f];
                viz::MooringLineState& state = out->frames[f].mooring_lines[li];
                for (size_t n = 0; n < nn; ++n) {
                    state.node_positions[n] = ToVec3(positions, 3 * (i * nn + n));
                    const size_t t_index = i * nn + n;
                    state.node_tensions[n] = t_index < tensions.size() ? tensions[t_index] : 0.0;
                }
            }
        }

        // ── TSDA glyph endpoints ─────────────────────────────────────────────
        for (size_t c = 0; c < scene.cable_lines.size(); ++c) {
            const std::string cable_path = "/visualization/cables/cable_" + std::to_string(c);
            if (!PathExists(file, cable_path + "/point1")) {
                continue;
            }
            const std::vector<double> p1 = ReadDoubles(file, cable_path + "/point1");
            const std::vector<double> p2 = ReadDoubles(file, cable_path + "/point2");
            const std::vector<double> tension = ReadDoubles(file, cable_path + "/tension");
            for (size_t f = 0; f < samples.size(); ++f) {
                const size_t i = samples[f];
                viz::CableLineState& state = out->frames[f].cable_lines[c];
                state.point1 = ToVec3(p1, 3 * i);
                state.point2 = ToVec3(p2, 3 * i);
                state.tension = i < tension.size() ? tension[i] : 0.0;
            }
        }

        // The input frame carries the first sample's lines (no t = 0 record exists).
        out->input_frame.mooring_lines = out->frames.front().mooring_lines;
        out->input_frame.cable_lines = out->frames.front().cable_lines;

        out->wave = ReadWave(file, scene.gravity, scene.water_depth, ramp_duration, &out->wave_note);
        return true;
    } catch (const H5::Exception& e) {
        *error = "Cannot read " + path + ": " + e.getDetailMsg();
    } catch (const std::exception& e) {
        *error = "Cannot read " + path + ": " + e.what();
    }
    return false;
}

#else  // SEASTACK_HAVE_HYDRO_IO

bool LoadReplayRecording(const std::string& path, size_t, ReplayRecording*, std::string* error) {
    *error = "Cannot replay " + path + ": this build has no HDF5 support (SEASTACK_ENABLE_HYDRO_IO=OFF).";
    return false;
}

#endif  // SEASTACK_HAVE_HYDRO_IO

}  // namespace seastack::chrono
