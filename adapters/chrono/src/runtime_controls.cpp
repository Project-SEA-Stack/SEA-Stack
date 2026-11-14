/**
 * @file runtime_controls.cpp
 * @brief Runtime parameter registry and command bus for TSDA/RSDA links.
 */

#include <seastack/adapters/chrono/runtime_controls.h>

#include <seastack/adapters/chrono/pto_chrono_adapter.h>

#include <chrono/physics/ChLinkRSDA.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystem.h>
#include <chrono/utils/ChForceFunctors.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace seastack::chrono {

namespace {

namespace cu = ::chrono::utils;

/// Which terms the linear law has. Each form evaluates exactly the expression
/// of the Chrono functor it replaces, so equal coefficients give equal forces
/// bit-for-bit.
enum class LinearForm { kSpring, kDamper, kSpringDamper };

/// TSDA linear spring-damper with editable coefficients [N/m, N*s/m, N].
/// Keeps Chrono's bump stops (if the replaced functor had them).
class EditableSpringDamperForce : public ::chrono::ChLinkTSDA::ForceFunctor {
  public:
    EditableSpringDamperForce(LinearForm form, double k, double c, double preload,
                              std::shared_ptr<cu::SpringForce> stops)
        : form_(form), k_(k), c_(c), preload_(preload), stops_(std::move(stops)) {}

    double evaluate(double /*time*/, double rest_length, double length, double vel,
                    const ::chrono::ChLinkTSDA& /*link*/) override {
        const double stops = stops_ ? stops_->evaluate_stops(length) : 0.0;
        switch (form_) {
            case LinearForm::kSpring:
                return preload_ - k_ * (length - rest_length) + stops;
            case LinearForm::kDamper:
                return -c_ * vel;
            case LinearForm::kSpringDamper:
            default:
                return preload_ - k_ * (length - rest_length) - c_ * vel + stops;
        }
    }

    void SetSpring(double k) {
        k_ = k;
        if (form_ == LinearForm::kDamper) form_ = LinearForm::kSpringDamper;
    }
    void SetDamping(double c) {
        c_ = c;
        if (form_ == LinearForm::kSpring) form_ = LinearForm::kSpringDamper;
    }

  private:
    LinearForm form_;
    double k_;
    double c_;
    double preload_;
    std::shared_ptr<cu::SpringForce> stops_;
};

/// RSDA linear spring-damper with editable coefficients [N*m/rad, N*m*s/rad, N*m].
class EditableSpringDamperTorque : public ::chrono::ChLinkRSDA::TorqueFunctor {
  public:
    EditableSpringDamperTorque(LinearForm form, double k, double c, double preload)
        : form_(form), k_(k), c_(c), preload_(preload) {}

    double evaluate(double /*time*/, double rest_angle, double angle, double vel,
                    const ::chrono::ChLinkRSDA& /*link*/) override {
        switch (form_) {
            case LinearForm::kSpring:
                return preload_ - k_ * (angle - rest_angle);
            case LinearForm::kDamper:
                return -c_ * vel;
            case LinearForm::kSpringDamper:
            default:
                return preload_ - k_ * (angle - rest_angle) - c_ * vel;
        }
    }

    void SetSpring(double k) {
        k_ = k;
        if (form_ == LinearForm::kDamper) form_ = LinearForm::kSpringDamper;
    }
    void SetDamping(double c) {
        c_ = c;
        if (form_ == LinearForm::kSpring) form_ = LinearForm::kSpringDamper;
    }

  private:
    LinearForm form_;
    double k_;
    double c_;
    double preload_;
};

/// Coefficients of a Chrono linear functor, read through its JSON export
/// (the members are private).
struct LinearCoefficients {
    LinearForm form = LinearForm::kSpringDamper;
    double k = 0.0;
    double c = 0.0;
    double preload = 0.0;
};

template <typename Functor>
LinearCoefficients ReadCoefficients(Functor* functor, LinearForm form) {
    rapidjson::Document doc;
    rapidjson::Value obj = functor->exportJSON(doc.GetAllocator());
    LinearCoefficients out;
    out.form = form;
    if (obj.HasMember("spring coefficient")) out.k = obj["spring coefficient"].GetDouble();
    if (obj.HasMember("damping coefficient")) out.c = obj["damping coefficient"].GetDouble();
    if (obj.HasMember("preload")) out.preload = obj["preload"].GetDouble();
    return out;
}

constexpr const char* kExternalPtoOwner = "external PTO";
constexpr const char* kNonlinearOwner = "nonlinear curve (model YAML)";
constexpr const char* kCustomLawOwner = "custom force law";

std::string FormatValue(double v) {
    std::ostringstream os;
    os << std::setprecision(17) << v;
    return os.str();
}

}  // namespace

// ---------------------------------------------------------------------------

/// How a link's spring and damping are evaluated, and how to change them.
struct RuntimeControls::LinkState {
    enum class Law {
        kBuiltIn,      ///< No functor: Chrono's own k and c on the link
        kLinear,       ///< Chrono linear functor: replaced on the first k/c edit
        kEditable,     ///< Replaced by an Editable* functor
        kNotEditable,  ///< Any other functor
    };

    ::chrono::ChLinkTSDA* tsda = nullptr;
    ::chrono::ChLinkRSDA* rsda = nullptr;
    Law law = Law::kNotEditable;
    LinearCoefficients linear;
    std::shared_ptr<cu::SpringForce> stops;  ///< TSDA only; null if not a SpringForce
    std::shared_ptr<EditableSpringDamperForce> force;
    std::shared_ptr<EditableSpringDamperTorque> torque;

    void SetSpringDamping(bool spring, double value) {
        if (law == Law::kBuiltIn) {
            if (tsda) {
                spring ? tsda->SetSpringCoefficient(value) : tsda->SetDampingCoefficient(value);
            } else {
                spring ? rsda->SetSpringCoefficient(value) : rsda->SetDampingCoefficient(value);
            }
            return;
        }
        if (law == Law::kLinear) {
            if (tsda) {
                force = std::make_shared<EditableSpringDamperForce>(linear.form, linear.k, linear.c,
                                                                    linear.preload, stops);
                tsda->RegisterForceFunctor(force);
            } else {
                torque = std::make_shared<EditableSpringDamperTorque>(linear.form, linear.k, linear.c,
                                                                      linear.preload);
                rsda->RegisterTorqueFunctor(torque);
            }
            law = Law::kEditable;
        }
        if (force) {
            spring ? force->SetSpring(value) : force->SetDamping(value);
        } else if (torque) {
            spring ? torque->SetSpring(value) : torque->SetDamping(value);
        }
    }
};

RuntimeControls::RuntimeControls(::chrono::ChSystem* system) {
    if (system == nullptr) {
        throw std::invalid_argument("RuntimeControls: system must not be null");
    }
    constexpr double kNoMin = -std::numeric_limits<double>::infinity();

    for (const auto& link : system->GetLinks()) {
        auto state = std::make_unique<LinkState>();
        std::string element;
        double k = 0.0;
        double c = 0.0;
        double rest = 0.0;
        bool spring_in_model = false;
        bool damping_in_model = false;
        std::string coeff_owner;
        std::string rest_owner;

        if (auto* tsda = dynamic_cast<::chrono::ChLinkTSDA*>(link.get())) {
            element = "tsda";
            state->tsda = tsda;
            rest = tsda->GetRestLength();
            auto fun = tsda->GetForceFunctor();
            if (!fun) {
                state->law = LinkState::Law::kBuiltIn;
                k = tsda->GetSpringCoefficient();
                c = tsda->GetDampingCoefficient();
            } else if (auto sd = std::dynamic_pointer_cast<cu::LinearSpringDamperForce>(fun)) {
                state->law = LinkState::Law::kLinear;
                state->linear = ReadCoefficients(sd.get(), LinearForm::kSpringDamper);
                state->stops = sd;
                spring_in_model = damping_in_model = true;
            } else if (auto s = std::dynamic_pointer_cast<cu::LinearSpringForce>(fun)) {
                state->law = LinkState::Law::kLinear;
                state->linear = ReadCoefficients(s.get(), LinearForm::kSpring);
                state->stops = s;
                spring_in_model = true;
            } else if (auto d = std::dynamic_pointer_cast<cu::LinearDamperForce>(fun)) {
                state->law = LinkState::Law::kLinear;
                state->linear = ReadCoefficients(d.get(), LinearForm::kDamper);
                damping_in_model = true;
            } else if (std::dynamic_pointer_cast<PTOForceFunctor>(fun)
#ifdef SEASTACK_HAVE_EXTERNAL
                       || std::dynamic_pointer_cast<ExternalPtoForceFunctor>(fun)
#endif
            ) {
                coeff_owner = rest_owner = kExternalPtoOwner;
            } else if (std::dynamic_pointer_cast<cu::NonlinearSpringForce>(fun) ||
                       std::dynamic_pointer_cast<cu::NonlinearDamperForce>(fun) ||
                       std::dynamic_pointer_cast<cu::NonlinearSpringDamperForce>(fun) ||
                       std::dynamic_pointer_cast<cu::DegressiveDamperForce>(fun) ||
                       std::dynamic_pointer_cast<cu::MapSpringDamperForce>(fun)) {
                coeff_owner = kNonlinearOwner;
            } else {
                coeff_owner = kCustomLawOwner;
            }
        } else if (auto* rsda = dynamic_cast<::chrono::ChLinkRSDA*>(link.get())) {
            element = "rsda";
            state->rsda = rsda;
            rest = rsda->GetRestAngle();
            auto fun = rsda->GetTorqueFunctor();
            if (!fun) {
                state->law = LinkState::Law::kBuiltIn;
                k = rsda->GetSpringCoefficient();
                c = rsda->GetDampingCoefficient();
            } else if (auto sd = std::dynamic_pointer_cast<cu::LinearSpringDamperTorque>(fun)) {
                state->law = LinkState::Law::kLinear;
                state->linear = ReadCoefficients(sd.get(), LinearForm::kSpringDamper);
                spring_in_model = damping_in_model = true;
            } else if (auto s = std::dynamic_pointer_cast<cu::LinearSpringTorque>(fun)) {
                state->law = LinkState::Law::kLinear;
                state->linear = ReadCoefficients(s.get(), LinearForm::kSpring);
                spring_in_model = true;
            } else if (auto d = std::dynamic_pointer_cast<cu::LinearDamperTorque>(fun)) {
                state->law = LinkState::Law::kLinear;
                state->linear = ReadCoefficients(d.get(), LinearForm::kDamper);
                damping_in_model = true;
            } else if (std::dynamic_pointer_cast<PTOTorqueFunctor>(fun)
#ifdef SEASTACK_HAVE_EXTERNAL
                       || std::dynamic_pointer_cast<ExternalPtoTorqueFunctor>(fun)
#endif
            ) {
                coeff_owner = rest_owner = kExternalPtoOwner;
            } else if (std::dynamic_pointer_cast<cu::NonlinearSpringTorque>(fun) ||
                       std::dynamic_pointer_cast<cu::NonlinearDamperTorque>(fun) ||
                       std::dynamic_pointer_cast<cu::NonlinearSpringDamperTorque>(fun)) {
                coeff_owner = kNonlinearOwner;
            } else {
                coeff_owner = kCustomLawOwner;
            }
        } else {
            continue;
        }
        if (state->law == LinkState::Law::kLinear) {
            k = state->linear.k;
            c = state->linear.c;
        }

        const std::string name = link->GetName();
        const bool is_tsda = (element == "tsda");
        const std::string prefix = element + "." + name + ".";
        auto add = [&](const std::string& field, const std::string& label, const std::string& units,
                       double value, double min_value, bool in_model, const std::string& owner) {
            RuntimeParameter p;
            p.id = prefix + field;
            p.element = element;
            p.link_name = name;
            p.field = field;
            p.label = label;
            p.units = units;
            p.default_value = value;
            p.value = value;
            p.min_value = min_value;
            p.source = in_model ? "model YAML" : "default";
            p.owner = owner;
            parameters_.push_back(std::move(p));
            link_of_parameter_.push_back(links_.size());
            apply_of_parameter_.emplace_back();
        };
        // A spring term means the model gave a free length / angle.
        const bool rest_in_model = spring_in_model || state->law == LinkState::Law::kBuiltIn ||
                                   state->law == LinkState::Law::kNotEditable;
        add("spring_coefficient", "Spring", is_tsda ? "N/m" : "N*m/rad", k, 0.0, spring_in_model,
            coeff_owner);
        add("damping_coefficient", "Damping", is_tsda ? "N*s/m" : "N*m*s/rad", c, 0.0,
            damping_in_model, coeff_owner);
        if (is_tsda) {
            add("rest_length", "Rest length", "m", rest, 0.0, rest_in_model, rest_owner);
        } else {
            add("rest_angle", "Rest angle", "rad", rest, kNoMin, rest_in_model, rest_owner);
        }
        links_.push_back(std::move(state));
    }
}

RuntimeControls::~RuntimeControls() = default;

void RuntimeControls::SetOwner(const std::string& parameter_id, const std::string& owner) {
    if (auto* p = FindMutable(parameter_id)) {
        p->owner = owner;
    }
}

void RuntimeControls::AddParameter(RuntimeParameter parameter, ApplyFunction apply) {
    if (!apply) {
        throw std::invalid_argument("RuntimeControls::AddParameter: '" + parameter.id + "' needs an apply function");
    }
    if (Find(parameter.id) != nullptr) {
        throw std::invalid_argument("RuntimeControls::AddParameter: '" + parameter.id + "' is already registered");
    }
    parameters_.push_back(std::move(parameter));
    link_of_parameter_.push_back(kNoLink);
    apply_of_parameter_.push_back(std::move(apply));
}

const RuntimeParameter* RuntimeControls::Find(const std::string& parameter_id) const {
    for (const auto& p : parameters_) {
        if (p.id == parameter_id) return &p;
    }
    return nullptr;
}

RuntimeParameter* RuntimeControls::FindMutable(const std::string& parameter_id) {
    return const_cast<RuntimeParameter*>(Find(parameter_id));
}

bool RuntimeControls::Validate(const std::string& parameter_id, double value,
                               std::string* error) const {
    const RuntimeParameter* p = Find(parameter_id);
    std::string msg;
    if (p == nullptr) {
        msg = "unknown parameter '" + parameter_id + "'";
    } else if (!p->owner.empty()) {
        msg = "'" + parameter_id + "' is set by " + p->owner + " and cannot be changed here";
    } else if (!std::isfinite(value)) {
        msg = "'" + parameter_id + "': value must be finite";
    } else if (value < p->min_value) {
        msg = "'" + parameter_id + "': value " + FormatValue(value) + " " + p->units +
              " is below the minimum " + FormatValue(p->min_value);
    }
    if (!msg.empty() && error != nullptr) {
        *error = msg;
    }
    return msg.empty();
}

bool RuntimeControls::Request(const std::string& parameter_id, double value, std::string* error) {
    if (!Validate(parameter_id, value, error)) {
        return false;
    }
    auto same = [&](const RuntimeCommand& r) { return r.parameter == parameter_id; };
    requests_.erase(std::remove_if(requests_.begin(), requests_.end(), same), requests_.end());
    RuntimeCommand cmd;
    cmd.parameter = parameter_id;
    cmd.value = value;
    requests_.push_back(cmd);
    RuntimeParameter* p = FindMutable(parameter_id);
    p->pending = true;
    p->pending_value = value;
    return true;
}

void RuntimeControls::LoadSchedule(const RuntimeSchedule& schedule, double start_time, double dt) {
    if (!(dt > 0.0)) {
        throw std::invalid_argument("RuntimeControls::LoadSchedule: dt must be > 0");
    }
    // Steps only mean the same boundary if the step size is the same.
    const bool same_dt = schedule.time_step > 0.0 &&
                         std::abs(schedule.time_step - dt) <= 1e-12 * dt;
    for (size_t i = 0; i < schedule.commands.size(); ++i) {
        RuntimeCommand cmd = schedule.commands[i];
        std::string error;
        if (!Validate(cmd.parameter, cmd.value, &error)) {
            throw std::runtime_error("schedule event " + std::to_string(i + 1) + ": " + error);
        }
        if (cmd.step < 0 || !same_dt) {
            if (!std::isfinite(cmd.time)) {
                throw std::runtime_error("schedule event " + std::to_string(i + 1) +
                                         ": needs a step or a time");
            }
            // 1e-9 absorbs round-off in times written as multiples of dt.
            cmd.step = static_cast<long long>(std::ceil((cmd.time - start_time) / dt - 1e-9));
            cmd.step = std::max(0LL, cmd.step);
        }
        scheduled_.push_back(cmd);
    }
    std::stable_sort(scheduled_.begin() + static_cast<std::ptrdiff_t>(next_scheduled_), scheduled_.end(),
                     [](const RuntimeCommand& a, const RuntimeCommand& b) { return a.step < b.step; });
}

void RuntimeControls::ApplyDue(long long step, double time) {
    while (next_scheduled_ < scheduled_.size() && scheduled_[next_scheduled_].step <= step) {
        RuntimeCommand cmd = scheduled_[next_scheduled_++];
        cmd.step = step;
        cmd.time = time;
        Apply(cmd, "schedule");
    }
    if (requests_.empty()) {
        return;
    }
    std::vector<RuntimeCommand> requests;
    requests.swap(requests_);
    for (RuntimeCommand& cmd : requests) {
        cmd.step = step;
        cmd.time = time;
        Apply(cmd, "gui");
    }
}

void RuntimeControls::Apply(const RuntimeCommand& command, const std::string& origin) {
    RuntimeParameter* p = FindMutable(command.parameter);
    if (p == nullptr) {
        return;  // validated when queued
    }
    const size_t index = static_cast<size_t>(p - parameters_.data());
    double effective_time = command.time;
    if (link_of_parameter_[index] == kNoLink) {
        effective_time = apply_of_parameter_[index](command.value, command.time);
    } else {
        LinkState& link = *links_[link_of_parameter_[index]];
        if (p->field == "spring_coefficient") {
            link.SetSpringDamping(true, command.value);
        } else if (p->field == "damping_coefficient") {
            link.SetSpringDamping(false, command.value);
        } else if (p->field == "rest_length") {
            link.tsda->SetRestLength(command.value);
        } else if (p->field == "rest_angle") {
            link.rsda->SetRestAngle(command.value);
        }
    }

    RuntimeEvent event;
    event.command = command;
    event.previous_value = p->value;
    event.origin = origin;
    events_.push_back(event);

    p->value = command.value;
    p->effective_time = effective_time;
    if (origin == "gui") {
        p->pending = false;
    }
    if (on_change_) {
        on_change_(*p);
    }
}

size_t RuntimeControls::UnappliedScheduledCount() const {
    return scheduled_.size() - next_scheduled_;
}

bool RuntimeControls::HasGuiEvents() const {
    return std::any_of(events_.begin(), events_.end(),
                       [](const RuntimeEvent& e) { return e.origin == "gui"; });
}

// ---------------------------------------------------------------------------
// Schedule files

RuntimeSchedule ReadRuntimeSchedule(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        throw std::runtime_error("schedule " + path + ": " + e.what());
    }
    const std::string where = "schedule " + path;
    if (!root.IsMap()) {
        throw std::runtime_error(where + ": expected a map with 'events'");
    }
    if (root["schedule_version"] && root["schedule_version"].as<int>() != 1) {
        throw std::runtime_error(where + ": unsupported schedule_version " +
                                 root["schedule_version"].as<std::string>() + " (expected 1)");
    }
    RuntimeSchedule schedule;
    if (root["time_step"]) {
        schedule.time_step = root["time_step"].as<double>();
    }
    const YAML::Node events = root["events"];
    if (!events || !events.IsSequence()) {
        throw std::runtime_error(where + ": 'events' must be a list");
    }
    for (size_t i = 0; i < events.size(); ++i) {
        const YAML::Node e = events[i];
        const std::string at = where + ", event " + std::to_string(i + 1);
        try {
            if (!e["parameter"] || !e["value"]) {
                throw std::runtime_error("needs 'parameter' and 'value'");
            }
            if (!e["step"] && !e["time"]) {
                throw std::runtime_error("needs 'step' or 'time'");
            }
            RuntimeCommand cmd;
            cmd.parameter = e["parameter"].as<std::string>();
            cmd.value = e["value"].as<double>();
            cmd.step = e["step"] ? e["step"].as<long long>() : -1;
            cmd.time = e["time"] ? e["time"].as<double>() : std::numeric_limits<double>::quiet_NaN();
            if (e["step"] && cmd.step < 0) {
                throw std::runtime_error("'step' must be >= 0");
            }
            schedule.commands.push_back(cmd);
        } catch (const YAML::Exception& ex) {
            throw std::runtime_error(at + ": " + ex.what());
        } catch (const std::runtime_error& ex) {
            throw std::runtime_error(at + ": " + ex.what());
        }
    }
    return schedule;
}

void WriteRuntimeSchedule(const std::string& path, const std::vector<RuntimeEvent>& events,
                          double time_step, const std::string& comment) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        throw std::runtime_error("cannot write schedule " + path);
    }
    // 17 significant digits: values and times read back to the same doubles.
    out << std::setprecision(17);
    out << "# SEA-Stack runtime schedule. Rerun with: run_seastack <case> --schedule <this file>\n";
    if (!comment.empty()) {
        out << "# " << comment << "\n";
    }
    out << "schedule_version: 1\n";
    if (time_step > 0.0) {
        out << "time_step: " << time_step << "  # [s]\n";
    }
    out << "events:\n";
    if (events.empty()) {
        out << "  []\n";
    }
    for (const auto& e : events) {
        out << "  - {step: " << e.command.step << ", time: " << e.command.time
            << ", parameter: \"" << e.command.parameter << "\", value: " << e.command.value
            << "}  # was " << e.previous_value << ", from " << e.origin << "\n";
    }
    if (!out) {
        throw std::runtime_error("error writing schedule " + path);
    }
}

}  // namespace seastack::chrono
