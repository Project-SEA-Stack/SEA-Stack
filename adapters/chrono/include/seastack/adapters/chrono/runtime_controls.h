/**
 * @file runtime_controls.h
 * @brief Runtime parameter registry, command bus and event log for TSDA/RSDA
 *        spring-dampers and other registered parameters such as the wave
 *        height (GUI edits and scripted schedules).
 *
 * Every ChLinkTSDA / ChLinkRSDA in the system gets three parameters:
 *   tsda.<link>.spring_coefficient   [N/m]          rsda.<link>.spring_coefficient   [N*m/rad]
 *   tsda.<link>.damping_coefficient  [N*s/m]        rsda.<link>.damping_coefficient  [N*m*s/rad]
 *   tsda.<link>.rest_length          [m]            rsda.<link>.rest_angle           [rad]
 *
 * Sign convention (Chrono): F = preload - k (L - L0) - c dL/dt along the link,
 * positive pushing the end points apart; RSDA likewise with the relative angle.
 *
 * A parameter is editable only if SEA-Stack can change it without changing
 * the force law: Chrono's built-in law, or the linear force functors Chrono's
 * YAML parser creates (LinearSpring/Damper/SpringDamper Force and Torque).
 * Otherwise `owner` names who sets it (external PTO, controller, nonlinear
 * curve, custom force law) and requests are rejected.
 *
 * Nothing changes until the first edit of a link: unedited runs are
 * bit-identical to runs without RuntimeControls. The first spring or damping
 * edit replaces Chrono's linear functor by an equivalent SEA-Stack functor
 * that evaluates the same expression with the new coefficients.
 *
 * Commands take effect at step boundaries: ApplyDue(step, time) must be called
 * immediately before every DoStepDynamics, with `step` the number of steps
 * already taken. A run with the same schedule therefore reproduces the same
 * results bit-for-bit, whether the commands came from the GUI or a file.
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace chrono {
class ChSystem;
class ChLinkTSDA;
class ChLinkRSDA;
}  // namespace chrono

namespace seastack::chrono {

/// One runtime-adjustable parameter: a link field, or one added with AddParameter.
struct RuntimeParameter {
    std::string id;           ///< "tsda.<link>.<field>", "rsda.<link>.<field>", or e.g. "waves.height"
    std::string element;      ///< "tsda", "rsda", or e.g. "waves"
    std::string link_name;    ///< Chrono link name; empty for non-link parameters
    std::string field;        ///< spring_coefficient, damping_coefficient, rest_length, rest_angle, height, ...
    std::string label;        ///< Short label for the GUI, e.g. "Damping"
    std::string units;        ///< SI units, e.g. "N*s/m"
    double default_value = 0.0;  ///< Value at the start of the run
    double value = 0.0;          ///< Value in effect now
    double min_value = 0.0;      ///< Lowest accepted value (inclusive); -inf for none
    std::string source;          ///< "model YAML" or "default" (not set in the model)
    std::string owner;           ///< Empty: editable here. Otherwise who sets it.
    bool pending = false;        ///< A request waits for the next step boundary
    double pending_value = 0.0;  ///< Value of that request
    /// Simulation time [s] from which the last change is fully in effect.
    /// Link fields change at once; a wave-height change ramps in, so this is
    /// later than the step boundary while the transition runs.
    double effective_time = 0.0;
};

/// A parameter change at a step boundary.
struct RuntimeCommand {
    long long step = -1;    ///< Applied before step `step` (steps already taken); -1 = use time
    double time = 0.0;      ///< Simulation time at that boundary [s]
    std::string parameter;  ///< RuntimeParameter::id
    double value = 0.0;     ///< New value [SI units of the parameter]
};

/// A command as applied, for the event log (HDF5 /events and the schedule).
struct RuntimeEvent {
    RuntimeCommand command;     ///< step and time are the actual boundary
    double previous_value = 0.0;
    std::string origin;         ///< "gui" or "schedule"
};

/// Runtime schedule file (YAML):
///   schedule_version: 1
///   time_step: 0.01              # [s] of the run that recorded it (optional)
///   events:
///     - {step: 1200, time: 12.0, parameter: tsda.PTO.damping_coefficient, value: 2.4e6}
/// `step` or `time` may be omitted (not both).
struct RuntimeSchedule {
    double time_step = 0.0;  ///< [s]; 0 = not given
    std::vector<RuntimeCommand> commands;
};

/// Read a schedule file. @throws std::runtime_error with the file and entry on errors.
RuntimeSchedule ReadRuntimeSchedule(const std::string& path);

/// Write the events as a schedule file (overwrites). @throws std::runtime_error on I/O errors.
void WriteRuntimeSchedule(const std::string& path, const std::vector<RuntimeEvent>& events,
                          double time_step, const std::string& comment);

class RuntimeControls {
  public:
    /// Registers every TSDA and RSDA in `system`, reading the current values.
    /// Call after all force laws are attached (ropes, external PTOs, controller).
    explicit RuntimeControls(::chrono::ChSystem* system);
    ~RuntimeControls();

    RuntimeControls(const RuntimeControls&) = delete;
    RuntimeControls& operator=(const RuntimeControls&) = delete;

    /// Mark a parameter as set by another component (e.g. the controller
    /// drives a rest length). Unknown ids are ignored.
    void SetOwner(const std::string& parameter_id, const std::string& owner);

    /// Applies a value at a step boundary at `time` [s] and returns the time
    /// from which it is fully in effect (`time` for an instant change).
    using ApplyFunction = std::function<double(double value, double time)>;

    /// Why `value` cannot be applied at `time` [s] (e.g. the wave data does
    /// not cover it, or a transition is still running); empty if it can.
    using CheckFunction = std::function<std::string(double value, double time)>;

    /// Register a parameter that is not a link field (e.g. the wave height).
    /// `parameter.value` must equal `parameter.default_value`. Requests,
    /// schedules and the event log treat it like the link parameters.
    /// `check` (optional) runs in Request(), at the last boundary time, and
    /// again at the boundary that applies the value. There a refused GUI
    /// request is dropped (see TakeRefusal) and a refused scheduled command
    /// throws, since the schedule could not be reproduced.
    /// @throws std::invalid_argument if the id is taken or `apply` is empty.
    void AddParameter(RuntimeParameter parameter, ApplyFunction apply, CheckFunction check = nullptr);

    const std::vector<RuntimeParameter>& parameters() const { return parameters_; }
    const RuntimeParameter* Find(const std::string& parameter_id) const;

    /// Queue a change for the next step boundary (GUI). Returns false and sets
    /// `error` if the id is unknown, the parameter is owned elsewhere, the
    /// value is not finite or below min_value, or its check refuses it.
    bool Request(const std::string& parameter_id, double value, std::string* error);

    /// The last GUI request refused at a step boundary by its check, as a
    /// message; empty if none since the last call. Clears it.
    std::string TakeRefusal();

    /// Add scheduled commands. Commands without a step are placed at the first
    /// boundary at or after their time: step = ceil((time - start_time) / dt).
    /// If the schedule's time_step differs from dt, times are used instead of
    /// steps. @throws std::runtime_error if a command is invalid (as Request).
    void LoadSchedule(const RuntimeSchedule& schedule, double start_time, double dt);

    /// Apply scheduled commands for this boundary, then pending GUI requests.
    /// Call immediately before DoStepDynamics; `step` = steps already taken.
    void ApplyDue(long long step, double time);

    /// Scheduled commands not applied yet (their step was not reached).
    size_t UnappliedScheduledCount() const;

    const std::vector<RuntimeEvent>& events() const { return events_; }
    bool HasGuiEvents() const;

    /// Called after each applied change with the link and new value, so
    /// derived outputs (exporter PTO power) can follow.
    using ChangeCallback = std::function<void(const RuntimeParameter&)>;
    void SetChangeCallback(ChangeCallback callback) { on_change_ = std::move(callback); }

  private:
    struct LinkState;

    bool Validate(const std::string& parameter_id, double value, std::string* error) const;
    void Apply(const RuntimeCommand& command, const std::string& origin);
    RuntimeParameter* FindMutable(const std::string& parameter_id);

    std::vector<RuntimeParameter> parameters_;
    std::vector<std::unique_ptr<LinkState>> links_;
    std::vector<size_t> link_of_parameter_;  ///< Index into links_ for each parameter; kNoLink if none
    std::vector<ApplyFunction> apply_of_parameter_;  ///< Set for AddParameter parameters
    std::vector<CheckFunction> check_of_parameter_;  ///< Optional, AddParameter parameters
    double last_boundary_time_ = 0.0;  ///< Time of the last ApplyDue [s]
    std::string refusal_;
    static constexpr size_t kNoLink = static_cast<size_t>(-1);
    std::vector<RuntimeCommand> scheduled_;  ///< Sorted by step (stable)
    size_t next_scheduled_ = 0;
    std::vector<RuntimeCommand> requests_;   ///< GUI requests in arrival order
    std::vector<RuntimeEvent> events_;
    ChangeCallback on_change_;
};

}  // namespace seastack::chrono
