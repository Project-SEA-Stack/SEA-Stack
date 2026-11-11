/*********************************************************************
 * @file  external_controller.cpp
 * @brief Out-of-process controller (preview). See external_controller.h.
 *********************************************************************/

#include "external_controller.h"

#ifdef SEASTACK_HAVE_EXTERNAL

#include <seastack/external/ipc_external_force_model.h>
#include <seastack/infra/logging.h>

#include <chrono/functions/ChFunctionSetpoint.h>
#include <chrono/physics/ChLinkMotorLinear.h>
#include <chrono/physics/ChLinkMotorLinearPosition.h>
#include <chrono/physics/ChLinkMotorRotation.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>
#include <chrono/physics/ChLinkRSDA.h>
#include <chrono/physics/ChLinkTSDA.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace seastack::app {

// ---------------------------------------------------------------------------
// Channel registry
// ---------------------------------------------------------------------------

const std::vector<ControllerChannelSpec>& ControllerChannelRegistry() {
    static const std::vector<ControllerChannelSpec> kRegistry = {
        {"body", "", "pos_x", "m", false, "Centre-of-mass position, world frame (as HDF5 `position`)"},
        {"body", "", "pos_y", "m", false, "Centre-of-mass position, world frame"},
        {"body", "", "pos_z", "m", false, "Centre-of-mass position, world frame (z up)"},
        {"body", "", "rot_x", "rad", false, "Cardan angle about x, XYZ sequence (as HDF5 `orientation_xyz`)"},
        {"body", "", "rot_y", "rad", false, "Cardan angle about y, XYZ sequence"},
        {"body", "", "rot_z", "rad", false, "Cardan angle about z, XYZ sequence"},
        {"body", "", "quat_w", "-", false, "Orientation quaternion, body to world (scalar part)"},
        {"body", "", "quat_x", "-", false, "Orientation quaternion, body to world"},
        {"body", "", "quat_y", "-", false, "Orientation quaternion, body to world"},
        {"body", "", "quat_z", "-", false, "Orientation quaternion, body to world"},
        {"body", "", "vel_x", "m/s", false, "Centre-of-mass velocity, world frame"},
        {"body", "", "vel_y", "m/s", false, "Centre-of-mass velocity, world frame"},
        {"body", "", "vel_z", "m/s", false, "Centre-of-mass velocity, world frame"},
        {"body", "", "ang_vel_x", "rad/s", false, "Angular velocity, world frame"},
        {"body", "", "ang_vel_y", "rad/s", false, "Angular velocity, world frame"},
        {"body", "", "ang_vel_z", "rad/s", false, "Angular velocity, world frame"},
        {"body", "", "acc_x", "m/s^2", false, "Centre-of-mass acceleration, world frame, from the last completed step"},
        {"body", "", "acc_y", "m/s^2", false, "Centre-of-mass acceleration, world frame, from the last completed step"},
        {"body", "", "acc_z", "m/s^2", false, "Centre-of-mass acceleration, world frame, from the last completed step"},
        {"body", "", "ang_acc_x", "rad/s^2", false, "Angular acceleration, world frame, from the last completed step"},
        {"body", "", "ang_acc_y", "rad/s^2", false, "Angular acceleration, world frame, from the last completed step"},
        {"body", "", "ang_acc_z", "rad/s^2", false, "Angular acceleration, world frame, from the last completed step"},
        {"motor", "rotation", "angle", "rad", true, "Chrono motor angle (`GetMotorAngle`, not wrapped). Write: angle set-point"},
        {"motor", "rotation", "angle_dt", "rad/s", true, "Motor angular speed. Write: optional rate feed-forward for the set-point"},
        {"motor", "rotation", "torque", "N*m", false, "Motor reaction torque (`GetMotorTorque`)"},
        {"motor", "linear", "position", "m", true, "Chrono motor position (`GetMotorPos`). Write: position set-point"},
        {"motor", "linear", "position_dt", "m/s", true, "Motor speed. Write: optional rate feed-forward for the set-point"},
        {"motor", "linear", "force", "N", false, "Motor reaction force (`GetMotorForce`)"},
        {"tsda", "", "length", "m", false, "Distance between the two attachment points"},
        {"tsda", "", "length_dt", "m/s", false, "Rate of change of length, positive when extending"},
        {"tsda", "", "rest_length", "m", true, "Rest (free) length used by the TSDA force law. Write: new rest length"},
        {"tsda", "", "force", "N", false, "Chrono TSDA force, negative in tension (linear law: f - k (L - L0) - c dL/dt)"},
        {"rsda", "", "angle", "rad", false, "Chrono RSDA angle (`GetAngle`) about the RSDA axis"},
        {"rsda", "", "angle_dt", "rad/s", false, "Rate of change of angle"},
        {"rsda", "", "rest_angle", "rad", true, "Rest angle used by the RSDA torque law (radians even if `angle_degrees: true`). Write: new rest angle"},
        {"rsda", "", "torque", "N*m", false, "Chrono RSDA torque, t - k (a - a0) - c da/dt"},
    };
    return kRegistry;
}

std::string ControllerChannelTableMarkdown() {
    std::ostringstream md;
    md << "| Channel | Applies to | Unit | Access | Meaning |\n";
    md << "|---------|------------|------|--------|---------|\n";
    for (const auto& c : ControllerChannelRegistry()) {
        const std::string applies =
            (c.applies[0] != '\0') ? std::string(c.applies) + " " + c.element
                                   : std::string("any ") + c.element;
        md << "| `" << c.element << ".<name>." << c.field << "` | " << applies << " | "
           << c.unit << " | " << (c.writable ? "read/write" : "read") << " | " << c.meaning
           << " |\n";
    }
    return md.str();
}

namespace {

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

size_t EditDistance(const std::string& a, const std::string& b) {
    std::vector<size_t> prev(b.size() + 1);
    std::vector<size_t> cur(b.size() + 1);
    for (size_t j = 0; j <= b.size(); ++j) {
        prev[j] = j;
    }
    for (size_t i = 1; i <= a.size(); ++i) {
        cur[0] = i;
        for (size_t j = 1; j <= b.size(); ++j) {
            const size_t sub = prev[j - 1] + (a[i - 1] == b[j - 1] ? 0 : 1);
            cur[j] = std::min({prev[j] + 1, cur[j - 1] + 1, sub});
        }
        std::swap(prev, cur);
    }
    return prev[b.size()];
}

/// " Did you mean 'x'?" for a close candidate, else "". A candidate that
/// starts with the word wins (ea -> ea_N); otherwise close = small edit
/// distance, or the same first three letters (body -> bodies).
std::string DidYouMean(const std::string& word, const std::vector<std::string>& candidates) {
    std::string best;
    if (word.size() >= 2) {
        for (const auto& c : candidates) {
            if (c.size() > word.size() && c.compare(0, word.size(), word) == 0 &&
                (best.empty() || c.size() < best.size())) {
                best = c;
            }
        }
        if (!best.empty()) {
            return " Did you mean '" + best + "'?";
        }
    }
    size_t best_d = std::numeric_limits<size_t>::max();
    const size_t max_d = std::max<size_t>(2, word.size() / 3);
    for (const auto& c : candidates) {
        const size_t d = EditDistance(word, c);
        const bool same_start = word.size() >= 3 && c.compare(0, 3, word, 0, 3) == 0;
        if ((d <= max_d || same_start) && d < best_d) {
            best_d = d;
            best = c;
        }
    }
    return best.empty() ? std::string() : " Did you mean '" + best + "'?";
}

std::string JoinNames(const std::vector<std::string>& names) {
    if (names.empty()) {
        return "(none)";
    }
    std::string out;
    for (size_t i = 0; i < names.size(); ++i) {
        out += (i == 0 ? "" : ", ") + names[i];
    }
    return out;
}

std::string FormatNumber(double t) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", t);
    return buf;
}

// ---------------------------------------------------------------------------
// YAML -> JSON for the opaque `config:` block
// ---------------------------------------------------------------------------

std::string JsonQuote(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += c; break;
        }
    }
    return out + "\"";
}

std::string YamlToJson(const YAML::Node& node) {
    if (!node || node.IsNull()) {
        return "null";
    }
    if (node.IsScalar()) {
        const std::string s = node.Scalar();
        // Quoted YAML scalars ("1.0", 'true') stay strings.
        if (node.Tag() == "!") {
            return JsonQuote(s);
        }
        if (s == "true" || s == "True" || s == "false" || s == "False") {
            return (s[0] == 't' || s[0] == 'T') ? "true" : "false";
        }
        if (s == "null" || s == "~") {
            return "null";
        }
        try {
            size_t used = 0;
            const double d = std::stod(s, &used);
            if (used == s.size() && std::isfinite(d)) {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "%.17g", d);
                std::string num = buf;
                // Keep 1.0 a float in Python (JSON "1" would load as int).
                const bool yaml_is_float = s.find_first_of(".eE") != std::string::npos;
                if (yaml_is_float && num.find_first_of(".eE") == std::string::npos) {
                    num += ".0";
                }
                return num;
            }
        } catch (...) {
        }
        return JsonQuote(s);
    }
    std::ostringstream oss;
    if (node.IsSequence()) {
        oss << '[';
        for (size_t i = 0; i < node.size(); ++i) {
            oss << (i ? "," : "") << YamlToJson(node[i]);
        }
        oss << ']';
        return oss.str();
    }
    oss << '{';
    bool first = true;
    for (const auto& kv : node) {
        oss << (first ? "" : ",") << JsonQuote(kv.first.as<std::string>()) << ':'
            << YamlToJson(kv.second);
        first = false;
    }
    oss << '}';
    return oss.str();
}

// ---------------------------------------------------------------------------
// YAML schema checks
// ---------------------------------------------------------------------------

void CheckKeys(const YAML::Node& map, const std::vector<std::string>& allowed,
               const std::string& where) {
    for (const auto& kv : map) {
        const std::string key = kv.first.as<std::string>();
        if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
            throw std::runtime_error(where + ": unknown key '" + key + "'." +
                                     DidYouMean(key, allowed) +
                                     " Valid keys: " + JoinNames(allowed) + ".");
        }
    }
}

std::vector<std::string> ReadNameList(const YAML::Node& node, const std::string& where) {
    std::vector<std::string> names;
    if (!node || node.IsNull()) {
        return names;
    }
    if (!node.IsSequence()) {
        throw std::runtime_error(where + " must be a list of names, e.g. [" +
                                 (node.IsScalar() ? node.Scalar() : std::string("name")) +
                                 "]");
    }
    std::set<std::string> seen;
    for (const auto& item : node) {
        if (!item.IsScalar() || item.Scalar().empty()) {
            throw std::runtime_error(where + " must contain only non-empty names");
        }
        const std::string name = item.Scalar();
        if (!seen.insert(name).second) {
            throw std::runtime_error(where + " lists '" + name + "' twice");
        }
        names.push_back(name);
    }
    return names;
}

ControllerConfig::ElementLists ReadElementLists(const YAML::Node& node,
                                                const std::string& where,
                                                bool is_actuators) {
    ControllerConfig::ElementLists lists;
    if (!node || node.IsNull()) {
        return lists;
    }
    if (!node.IsMap()) {
        throw std::runtime_error(where + " must be a map with keys " +
                                 (is_actuators ? "motors, tsdas, rsdas"
                                               : "bodies, motors, tsdas, rsdas"));
    }
    if (is_actuators && node["bodies"]) {
        throw std::runtime_error(
            where + ": bodies are read-only. List them under sensors.bodies and drive "
                    "them through a motor, TSDA or RSDA instead.");
    }
    CheckKeys(node,
              is_actuators ? std::vector<std::string>{"motors", "tsdas", "rsdas"}
                           : std::vector<std::string>{"bodies", "motors", "tsdas", "rsdas"},
              where);
    lists.bodies = ReadNameList(node["bodies"], where + ".bodies");
    lists.motors = ReadNameList(node["motors"], where + ".motors");
    lists.tsdas = ReadNameList(node["tsdas"], where + ".tsdas");
    lists.rsdas = ReadNameList(node["rsdas"], where + ".rsdas");
    return lists;
}

bool IsEmpty(const ControllerConfig::ElementLists& l) {
    return l.bodies.empty() && l.motors.empty() && l.tsdas.empty() && l.rsdas.empty();
}

// ---------------------------------------------------------------------------
// Chrono lookups (errors list the valid names of that kind)
// ---------------------------------------------------------------------------

template <typename LinkT>
std::vector<std::shared_ptr<LinkT>> LinksOfType(::chrono::ChSystem& system) {
    std::vector<std::shared_ptr<LinkT>> out;
    for (const auto& link : system.GetLinks()) {
        if (auto typed = std::dynamic_pointer_cast<LinkT>(link)) {
            out.push_back(typed);
        }
    }
    return out;
}

template <typename LinkT>
std::shared_ptr<LinkT> FindLink(::chrono::ChSystem& system, const std::string& name,
                                const std::string& where, const char* kind) {
    std::vector<std::string> names;
    for (const auto& link : LinksOfType<LinkT>(system)) {
        if (link->GetName() == name) {
            return link;
        }
        names.push_back(link->GetName());
    }
    throw std::runtime_error(where + ": no " + kind + " named '" + name + "' in the model." +
                             DidYouMean(name, names) + " " + kind + "s in the model: " +
                             JoinNames(names) + ".");
}

std::shared_ptr<::chrono::ChBody> FindBody(::chrono::ChSystem& system, const std::string& name,
                                           const std::string& where) {
    std::vector<std::string> names;
    for (const auto& body : system.GetBodies()) {
        if (body->GetName() == name) {
            return body;
        }
        names.push_back(body->GetName());
    }
    throw std::runtime_error(where + ": no body named '" + name + "' in the model." +
                             DidYouMean(name, names) + " Bodies in the model: " +
                             JoinNames(names) + ".");
}

/// Reader for one registry field of a body. Throws if the field is unknown.
std::function<double()> BodyReader(std::shared_ptr<::chrono::ChBody> b, const std::string& f) {
    // Cardan XYZ angles match the HDF5 exporter's orientation_xyz.
    if (f == "pos_x") return [b] { return b->GetPos().x(); };
    if (f == "pos_y") return [b] { return b->GetPos().y(); };
    if (f == "pos_z") return [b] { return b->GetPos().z(); };
    if (f == "rot_x") return [b] { return b->GetRot().GetCardanAnglesXYZ().x(); };
    if (f == "rot_y") return [b] { return b->GetRot().GetCardanAnglesXYZ().y(); };
    if (f == "rot_z") return [b] { return b->GetRot().GetCardanAnglesXYZ().z(); };
    if (f == "quat_w") return [b] { return b->GetRot().e0(); };
    if (f == "quat_x") return [b] { return b->GetRot().e1(); };
    if (f == "quat_y") return [b] { return b->GetRot().e2(); };
    if (f == "quat_z") return [b] { return b->GetRot().e3(); };
    if (f == "vel_x") return [b] { return b->GetPosDt().x(); };
    if (f == "vel_y") return [b] { return b->GetPosDt().y(); };
    if (f == "vel_z") return [b] { return b->GetPosDt().z(); };
    if (f == "ang_vel_x") return [b] { return b->GetAngVelParent().x(); };
    if (f == "ang_vel_y") return [b] { return b->GetAngVelParent().y(); };
    if (f == "ang_vel_z") return [b] { return b->GetAngVelParent().z(); };
    if (f == "acc_x") return [b] { return b->GetPosDt2().x(); };
    if (f == "acc_y") return [b] { return b->GetPosDt2().y(); };
    if (f == "acc_z") return [b] { return b->GetPosDt2().z(); };
    if (f == "ang_acc_x") return [b] { return b->GetAngAccParent().x(); };
    if (f == "ang_acc_y") return [b] { return b->GetAngAccParent().y(); };
    if (f == "ang_acc_z") return [b] { return b->GetAngAccParent().z(); };
    throw std::logic_error("controller registry: no reader for body field " + f);
}

std::function<double()> RotationMotorReader(std::shared_ptr<::chrono::ChLinkMotorRotation> m,
                                             const std::string& f) {
    if (f == "angle") return [m] { return m->GetMotorAngle(); };
    if (f == "angle_dt") return [m] { return m->GetMotorAngleDt(); };
    if (f == "torque") return [m] { return m->GetMotorTorque(); };
    throw std::logic_error("controller registry: no reader for rotation motor field " + f);
}

std::function<double()> LinearMotorReader(std::shared_ptr<::chrono::ChLinkMotorLinear> m,
                                          const std::string& f) {
    if (f == "position") return [m] { return m->GetMotorPos(); };
    if (f == "position_dt") return [m] { return m->GetMotorPosDt(); };
    if (f == "force") return [m] { return m->GetMotorForce(); };
    throw std::logic_error("controller registry: no reader for linear motor field " + f);
}

std::function<double()> TsdaReader(std::shared_ptr<::chrono::ChLinkTSDA> t, const std::string& f) {
    if (f == "length") return [t] { return t->GetLength(); };
    if (f == "length_dt") return [t] { return t->GetVelocity(); };
    if (f == "rest_length") return [t] { return t->GetRestLength(); };
    if (f == "force") return [t] { return t->GetForce(); };
    throw std::logic_error("controller registry: no reader for TSDA field " + f);
}

std::function<double()> RsdaReader(std::shared_ptr<::chrono::ChLinkRSDA> r, const std::string& f) {
    if (f == "angle") return [r] { return r->GetAngle(); };
    if (f == "angle_dt") return [r] { return r->GetVelocity(); };
    if (f == "rest_angle") return [r] { return r->GetRestAngle(); };
    if (f == "torque") return [r] { return r->GetTorque(); };
    throw std::logic_error("controller registry: no reader for RSDA field " + f);
}

/// Registry rows for one element kind / subtype ("" matches rows with no subtype).
std::vector<const ControllerChannelSpec*> RegistryRows(const std::string& element,
                                                       const std::string& applies) {
    std::vector<const ControllerChannelSpec*> rows;
    for (const auto& c : ControllerChannelRegistry()) {
        if (element == c.element && applies == c.applies) {
            rows.push_back(&c);
        }
    }
    return rows;
}

}  // namespace

// ---------------------------------------------------------------------------
// LoadControllerFile
// ---------------------------------------------------------------------------

ControllerConfig LoadControllerFile(const std::filesystem::path& path) {
    namespace fs = std::filesystem;
    if (!fs::exists(path)) {
        throw std::runtime_error("controller_file not found: " + path.string());
    }
    const std::string where = "controller_file " + path.filename().string();
    YAML::Node root;
    try {
        root = YAML::LoadFile(path.string());
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(where + ": YAML syntax error: " + e.what());
    }
    if (!root.IsMap()) {
        throw std::runtime_error(where + " must be a YAML map (command, sensors, actuators, ...)");
    }
    CheckKeys(root,
              {"command", "control_dt", "timeout_ms", "working_directory", "sensors",
               "actuators", "config"},
              where);

    ControllerConfig cfg;
    std::error_code ec;
    const fs::path abs_path = fs::absolute(path, ec);
    cfg.source_path = (ec ? path : abs_path).generic_string();
    const fs::path dir = (ec ? path : abs_path).parent_path();

    // command: "python ctl.py" as a list (preferred) or a single executable.
    const YAML::Node cmd = root["command"];
    if (!cmd || (cmd.IsScalar() && cmd.Scalar().empty()) || (cmd.IsSequence() && cmd.size() == 0)) {
        throw std::runtime_error(where + ": 'command' is required, e.g. command: [python, my_controller.py]");
    }
    if (cmd.IsScalar()) {
        cfg.command.push_back(cmd.Scalar());
    } else if (cmd.IsSequence()) {
        for (const auto& item : cmd) {
            if (!item.IsScalar()) {
                throw std::runtime_error(where + ": 'command' must be a list of strings");
            }
            cfg.command.push_back(item.Scalar());
        }
    } else {
        throw std::runtime_error(where + ": 'command' must be a list, e.g. [python, my_controller.py]");
    }
    // Relative script paths are relative to the controller YAML; the child
    // runs in working_directory, so pass absolute paths.
    for (size_t i = 1; i < cfg.command.size(); ++i) {
        fs::path p(cfg.command[i]);
        if (p.extension() != ".py") {
            continue;
        }
        if (p.is_relative()) {
            p = dir / p;
        }
        if (!fs::exists(p)) {
            throw std::runtime_error(where + ": controller script not found: " + p.string());
        }
        cfg.command[i] = p.lexically_normal().string();
    }

    if (root["control_dt"]) {
        try {
            cfg.control_dt = root["control_dt"].as<double>();
        } catch (const YAML::Exception&) {
            throw std::runtime_error(where + ": control_dt must be a number of seconds, e.g. 0.05");
        }
        if (!(cfg.control_dt > 0.0) || !std::isfinite(cfg.control_dt)) {
            throw std::runtime_error(where + ": control_dt must be > 0 s (omit it to use the simulation time_step)");
        }
    }
    if (root["timeout_ms"]) {
        try {
            cfg.timeout_ms = root["timeout_ms"].as<int>();
        } catch (const YAML::Exception&) {
            throw std::runtime_error(where + ": timeout_ms must be a whole number of milliseconds");
        }
        if (cfg.timeout_ms <= 0) {
            throw std::runtime_error(where + ": timeout_ms must be > 0");
        }
    }
    cfg.working_directory = dir.string();
    if (root["working_directory"]) {
        fs::path wd(root["working_directory"].as<std::string>());
        if (wd.is_relative()) {
            wd = dir / wd;
        }
        if (!fs::is_directory(wd)) {
            throw std::runtime_error(where + ": working_directory does not exist: " + wd.string());
        }
        cfg.working_directory = wd.lexically_normal().string();
    }

    cfg.sensors = ReadElementLists(root["sensors"], where + ": sensors", false);
    cfg.actuators = ReadElementLists(root["actuators"], where + ": actuators", true);
    if (IsEmpty(cfg.sensors) && IsEmpty(cfg.actuators)) {
        throw std::runtime_error(where + " lists no sensors and no actuators");
    }

    if (root["config"] && !root["config"].IsNull()) {
        if (!root["config"].IsMap()) {
            throw std::runtime_error(where + ": 'config' must be a map of settings for the module");
        }
        cfg.config_json = YamlToJson(root["config"]);
    }
    return cfg;
}

// ---------------------------------------------------------------------------
// Tension-only ropes
// ---------------------------------------------------------------------------

std::vector<RopeConfig> LoadRopesFile(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("ropes_file not found: " + path.string());
    }
    const std::string where = "ropes_file " + path.filename().string();
    YAML::Node root;
    try {
        root = YAML::LoadFile(path.string());
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(where + ": YAML syntax error: " + e.what());
    }
    if (!root.IsMap()) {
        throw std::runtime_error(where + " must be a YAML map with a 'ropes:' list");
    }
    CheckKeys(root, {"ropes"}, where);
    const YAML::Node list = root["ropes"];
    if (!list || !list.IsSequence() || list.size() == 0) {
        throw std::runtime_error(where + ": 'ropes' must be a non-empty list, e.g. "
                                 "ropes: [{link: hoist, ea_N: 8.0e6, damping_Ns_per_m: 8.0e4}]");
    }

    auto read_number = [](const YAML::Node& n, const std::string& at) {
        try {
            return n.as<double>();
        } catch (const YAML::Exception&) {
            throw std::runtime_error(at + " must be a number");
        }
    };

    std::vector<RopeConfig> ropes;
    std::set<std::string> seen;
    for (size_t i = 0; i < list.size(); ++i) {
        const YAML::Node item = list[i];
        const std::string at = where + ": ropes[" + std::to_string(i) + "]";
        if (!item.IsMap()) {
            throw std::runtime_error(at + " must be a map with keys link, ea_N, damping_Ns_per_m");
        }
        CheckKeys(item, {"link", "law", "ea_N", "damping_Ns_per_m"}, at);
        if (!item["link"] || !item["link"].IsScalar() || item["link"].Scalar().empty()) {
            throw std::runtime_error(at + ": 'link' (the TSDA name) is required");
        }
        RopeConfig rope;
        rope.link = item["link"].Scalar();
        if (!seen.insert(rope.link).second) {
            throw std::runtime_error(where + " lists TSDA '" + rope.link + "' twice");
        }
        if (item["law"] && item["law"].as<std::string>() != "tension_only") {
            throw std::runtime_error(at + ": law '" + item["law"].as<std::string>() +
                                     "' is not supported; the only law is tension_only");
        }
        if (!item["ea_N"]) {
            throw std::runtime_error(at + ": 'ea_N' (axial stiffness EA [N]) is required");
        }
        rope.ea_N = read_number(item["ea_N"], at + ".ea_N");
        if (!(rope.ea_N > 0.0) || !std::isfinite(rope.ea_N)) {
            throw std::runtime_error(at + ": ea_N must be > 0 N");
        }
        if (item["damping_Ns_per_m"]) {
            rope.damping_Ns_per_m = read_number(item["damping_Ns_per_m"], at + ".damping_Ns_per_m");
            if (!(rope.damping_Ns_per_m >= 0.0) || !std::isfinite(rope.damping_Ns_per_m)) {
                throw std::runtime_error(at + ": damping_Ns_per_m must be >= 0 N*s/m");
            }
        }
        ropes.push_back(rope);
    }
    return ropes;
}

double TensionOnlyRopeForce::evaluate(double /*time*/, double rest_length, double length,
                                      double vel, const ::chrono::ChLinkTSDA& /*link*/) {
    const double l0 = std::max(rest_length, 1e-6);
    const double tension = ea_N_ / l0 * (length - rest_length) + damping_Ns_per_m_ * vel;
    return -std::max(tension, 0.0);
}

void ApplyRopes(::chrono::ChSystem& system, const std::vector<RopeConfig>& ropes) {
    for (const auto& rope : ropes) {
        auto tsda = FindLink<::chrono::ChLinkTSDA>(system, rope.link, "ropes_file", "TSDA");
        // The functor replaces the model-YAML spring-damper law entirely.
        tsda->SetSpringCoefficient(0.0);
        tsda->SetDampingCoefficient(0.0);
        tsda->SetActuatorForce(0.0);
        tsda->RegisterForceFunctor(
            std::make_shared<TensionOnlyRopeForce>(rope.ea_N, rope.damping_Ns_per_m));
        seastack::infra::cli::LogInfo(
            "rope '" + rope.link + "': tension-only, EA = " + FormatNumber(rope.ea_N) +
            " N, c = " + FormatNumber(rope.damping_Ns_per_m) + " N*s/m, rest length = " +
            FormatNumber(tsda->GetRestLength()) + " m, length = " +
            FormatNumber(tsda->GetLength()) + " m");
    }
}

// ---------------------------------------------------------------------------
// ExternalController
// ---------------------------------------------------------------------------

struct ExternalController::Actuator {
    enum class Kind { kMotorSetpoint, kTsdaRestLength, kRsdaRestAngle };
    Kind kind = Kind::kMotorSetpoint;
    std::shared_ptr<::chrono::ChFunctionSetpoint> setpoint;  ///< motors: value, then rate
    std::shared_ptr<::chrono::ChLinkTSDA> tsda;
    std::shared_ptr<::chrono::ChLinkRSDA> rsda;
    size_t index = 0;  ///< First channel in held_ (motors use index and index + 1).
};

ExternalController::ExternalController(
    ::chrono::ChSystem& system,
    const ControllerConfig& config,
    double sim_dt,
    std::unique_ptr<seastack::external::IExternalForceModel> model)
    : config_(config) {
    label_ = config_.command.empty() ? std::string("controller") : config_.command.front();
    for (const auto& arg : config_.command) {
        if (std::filesystem::path(arg).extension() == ".py") {
            label_ = std::filesystem::path(arg).filename().string();
        }
    }

    // Control period: a whole number of simulation steps (zero-order hold).
    if (!(sim_dt > 0.0)) {
        throw std::runtime_error("controller: simulation time_step must be > 0");
    }
    control_dt_ = (config_.control_dt > 0.0) ? config_.control_dt : sim_dt;
    const double ratio = control_dt_ / sim_dt;
    steps_per_tick_ = static_cast<int>(std::llround(ratio));
    if (steps_per_tick_ < 1 || std::abs(ratio - steps_per_tick_) > 1e-6 * ratio) {
        const int lo = std::max(1, static_cast<int>(std::floor(ratio)));
        throw std::runtime_error(
            "controller_file: control_dt = " + FormatNumber(control_dt_) +
            " s must be a whole multiple of the simulation time_step = " + FormatNumber(sim_dt) +
            " s, e.g. " + FormatNumber(lo * sim_dt) + " or " + FormatNumber((lo + 1) * sim_dt) +
            " s");
    }

    ResolveSensors(system);
    ResolveActuators(system);

    // Motors set to CONTROLLER but not driven here would silently hold 0.
    for (const auto& motor : LinksOfType<::chrono::ChLinkMotor>(system)) {
        if (!std::dynamic_pointer_cast<::chrono::ChFunctionSetpoint>(motor->GetMotorFunction())) {
            continue;
        }
        const auto& listed = config_.actuators.motors;
        if (std::find(listed.begin(), listed.end(), motor->GetName()) == listed.end()) {
            seastack::infra::cli::LogWarning(
                "controller: motor '" + motor->GetName() +
                "' has actuation_function CONTROLLER but is not listed under actuators.motors; "
                "it will hold its initial set-point (0).");
        }
    }

    if (!model) {
        seastack::external::IpcExternalForceOptions opts;
        opts.command = config_.command;
        opts.timeout_ms = config_.timeout_ms;
        opts.working_directory = config_.working_directory;
        try {
            model = std::make_unique<seastack::external::IpcExternalForceModel>(std::move(opts));
        } catch (const std::exception& e) {
            throw std::runtime_error("controller '" + label_ + "' could not be started: " + e.what());
        }
    }
    model_ = std::move(model);

    seastack::external::ExternalInit init;
    init.kind = "controller";
    init.n_inputs = static_cast<int>(sensor_names_.size());
    init.n_outputs = static_cast<int>(actuator_names_.size());
    init.dt = control_dt_;
    init.config_json = config_.config_json;
    init.in_names = sensor_names_;
    init.out_names = actuator_names_;
    try {
        model_->Initialize(init);
    } catch (const std::exception& e) {
        PrintModuleLog();
        // On POSIX a bad interpreter fails in the child after fork, so the
        // IPC layer reports "exited before connecting". Map that to the same
        // "could not be started" wording Windows gets from CreateProcess.
        const std::string detail = e.what();
        if (detail.find("exited before connecting") != std::string::npos) {
            throw std::runtime_error("controller '" + label_ + "' could not be started: " + detail);
        }
        throw std::runtime_error("controller '" + label_ + "' failed to start: " + detail);
    }
    PrintModuleLog();
    LogChannelSummary();
}

ExternalController::~ExternalController() {
    if (model_) {
        try {
            model_->Shutdown();
        } catch (...) {
        }
    }
}

void ExternalController::ResolveSensors(::chrono::ChSystem& system) {
    using namespace ::chrono;
    const std::string where = "controller_file sensors";
    auto add = [this](const std::string& prefix, const ControllerChannelSpec* spec,
                      std::function<double()> reader) {
        sensor_names_.push_back(prefix + spec->field);
        sensor_units_.push_back(spec->unit);
        sensor_readers_.push_back(std::move(reader));
    };

    for (const auto& name : config_.sensors.bodies) {
        auto body = FindBody(system, name, where + ".bodies");
        for (const auto* spec : RegistryRows("body", "")) {
            add("body." + name + ".", spec, BodyReader(body, spec->field));
        }
    }
    for (const auto& name : config_.sensors.motors) {
        auto motor = FindLink<ChLinkMotor>(system, name, where + ".motors", "motor");
        if (auto rot = std::dynamic_pointer_cast<ChLinkMotorRotation>(motor)) {
            for (const auto* spec : RegistryRows("motor", "rotation")) {
                add("motor." + name + ".", spec, RotationMotorReader(rot, spec->field));
            }
        } else if (auto lin = std::dynamic_pointer_cast<ChLinkMotorLinear>(motor)) {
            for (const auto* spec : RegistryRows("motor", "linear")) {
                add("motor." + name + ".", spec, LinearMotorReader(lin, spec->field));
            }
        } else {
            throw std::runtime_error(where + ".motors: motor '" + name +
                                     "' is neither a rotation nor a linear motor");
        }
    }
    for (const auto& name : config_.sensors.tsdas) {
        auto tsda = FindLink<ChLinkTSDA>(system, name, where + ".tsdas", "TSDA");
        for (const auto* spec : RegistryRows("tsda", "")) {
            add("tsda." + name + ".", spec, TsdaReader(tsda, spec->field));
        }
    }
    for (const auto& name : config_.sensors.rsdas) {
        auto rsda = FindLink<ChLinkRSDA>(system, name, where + ".rsdas", "RSDA");
        for (const auto* spec : RegistryRows("rsda", "")) {
            add("rsda." + name + ".", spec, RsdaReader(rsda, spec->field));
        }
    }
}

void ExternalController::ResolveActuators(::chrono::ChSystem& system) {
    using namespace ::chrono;
    const std::string where = "controller_file actuators";
    auto add_channels = [this](const std::string& prefix, const std::string& element,
                               const std::string& applies) {
        for (const auto* spec : RegistryRows(element, applies)) {
            if (spec->writable) {
                actuator_names_.push_back(prefix + spec->field);
                actuator_units_.push_back(spec->unit);
            }
        }
    };

    for (const auto& name : config_.actuators.motors) {
        auto motor = FindLink<ChLinkMotor>(system, name, where + ".motors", "motor");
        auto rot = std::dynamic_pointer_cast<ChLinkMotorRotationAngle>(motor);
        auto lin = std::dynamic_pointer_cast<ChLinkMotorLinearPosition>(motor);
        if (!rot && !lin) {
            throw std::runtime_error(
                where + ".motors: motor '" + name +
                "' is not a position motor. In the model YAML set actuation_type: POSITION "
                "and actuation_function: {type: CONTROLLER}.");
        }
        auto setpoint =
            std::dynamic_pointer_cast<ChFunctionSetpoint>(motor->GetMotorFunction());
        if (!setpoint) {
            throw std::runtime_error(
                where + ".motors: motor '" + name +
                "' has a fixed actuation_function. Set actuation_function: {type: CONTROLLER} "
                "in the model YAML so the controller can drive it.");
        }
        Actuator a;
        a.kind = Actuator::Kind::kMotorSetpoint;
        a.setpoint = setpoint;
        a.index = held_.size();
        // Start by holding the current pose so nothing moves before the first command.
        if (rot) {
            held_.push_back(rot->GetMotorAngle());
            held_.push_back(rot->GetMotorAngleDt());
            add_channels("motor." + name + ".", "motor", "rotation");
        } else {
            held_.push_back(lin->GetMotorPos());
            held_.push_back(lin->GetMotorPosDt());
            add_channels("motor." + name + ".", "motor", "linear");
        }
        actuators_.push_back(a);
    }
    for (const auto& name : config_.actuators.tsdas) {
        Actuator a;
        a.kind = Actuator::Kind::kTsdaRestLength;
        a.tsda = FindLink<ChLinkTSDA>(system, name, where + ".tsdas", "TSDA");
        a.index = held_.size();
        held_.push_back(a.tsda->GetRestLength());
        add_channels("tsda." + name + ".", "tsda", "");
        actuators_.push_back(a);
    }
    for (const auto& name : config_.actuators.rsdas) {
        Actuator a;
        a.kind = Actuator::Kind::kRsdaRestAngle;
        a.rsda = FindLink<ChLinkRSDA>(system, name, where + ".rsdas", "RSDA");
        a.index = held_.size();
        held_.push_back(a.rsda->GetRestAngle());
        add_channels("rsda." + name + ".", "rsda", "");
        actuators_.push_back(a);
    }
    if (held_.size() != actuator_names_.size()) {
        throw std::logic_error("controller: actuator registry and held commands out of sync");
    }
    ApplyHeldCommands();
}

void ExternalController::ApplyHeldCommands() {
    for (const auto& a : actuators_) {
        switch (a.kind) {
            case Actuator::Kind::kMotorSetpoint:
                a.setpoint->SetSetpointAndDerivatives(held_[a.index], held_[a.index + 1], 0.0);
                break;
            case Actuator::Kind::kTsdaRestLength:
                a.tsda->SetRestLength(held_[a.index]);
                break;
            case Actuator::Kind::kRsdaRestAngle:
                a.rsda->SetRestAngle(held_[a.index]);
                break;
        }
    }
}

void ExternalController::BeforeStep(double time) {
    const bool tick = (step_index_ % steps_per_tick_) == 0;
    ++step_index_;
    if (!tick) {
        return;
    }

    std::vector<double> in(sensor_readers_.size());
    for (size_t i = 0; i < sensor_readers_.size(); ++i) {
        in[i] = sensor_readers_[i]();
    }
    std::vector<double> out;
    try {
        model_->Evaluate(time, in, out);
    } catch (const std::exception& e) {
        PrintModuleLog();
        std::string hint;
        const std::string what = e.what();
        if (what.find("timeout") != std::string::npos) {
            hint = " (no reply within timeout_ms = " + std::to_string(config_.timeout_ms) +
                   " ms; is step() blocking or very slow?)";
        } else if (what.find("peer closed") != std::string::npos ||
                   what.find("send failed") != std::string::npos) {
            hint = " (the controller process exited; run the script under ControllerHarness "
                   "to see why)";
        }
        throw std::runtime_error("controller '" + label_ + "' failed at t = " +
                                 FormatNumber(time) + " s: " + what + hint);
    }
    PrintModuleLog();

    // NaN means "hold the previous command" (the Python helper sends NaN for
    // channels the module did not set this tick).
    for (size_t i = 0; i < out.size() && i < held_.size(); ++i) {
        if (std::isfinite(out[i])) {
            held_[i] = out[i];
        }
    }
    ApplyHeldCommands();
}

void ExternalController::PrintModuleLog() {
    if (!model_) {
        return;
    }
    constexpr size_t kMaxLines = 200;
    const auto lines = model_->TakeLogLines();
    for (size_t i = 0; i < lines.size() && i < kMaxLines; ++i) {
        seastack::infra::cli::LogInfo("[" + label_ + "] " + lines[i]);
    }
    if (lines.size() > kMaxLines) {
        seastack::infra::cli::LogInfo("[" + label_ + "] ... " +
                                      std::to_string(lines.size() - kMaxLines) +
                                      " more lines not shown");
    }
}

void ExternalController::LogChannelSummary() const {
    std::string summary = "controller '" + label_ + "' attached: " +
                          std::to_string(sensor_names_.size()) + " sensor and " +
                          std::to_string(actuator_names_.size()) +
                          " actuator channels, control_dt = " + FormatNumber(control_dt_) +
                          " s";
    if (steps_per_tick_ > 1) {
        summary += " (every " + std::to_string(steps_per_tick_) + " steps)";
    }
    seastack::infra::cli::LogInfo(summary);
    for (size_t i = 0; i < actuator_names_.size(); ++i) {
        seastack::infra::cli::LogInfo("  writes " + actuator_names_[i] + " [" +
                                      actuator_units_[i] + "]");
    }
    for (size_t i = 0; i < sensor_names_.size(); ++i) {
        seastack::infra::debug::LogDebug("  reads  " + sensor_names_[i] + " [" +
                                         sensor_units_[i] + "]");
    }
}

}  // namespace seastack::app

#endif  // SEASTACK_HAVE_EXTERNAL
