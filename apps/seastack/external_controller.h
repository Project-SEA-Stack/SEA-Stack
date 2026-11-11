/*********************************************************************
 * @file  external_controller.h
 * @brief Out-of-process controller (preview): reads named sensor channels
 *        and writes actuator set-points on Chrono motors, TSDAs and RSDAs.
 *
 * Wiring used by run_seastack:
 *   setup `controller_file` -> LoadControllerFile -> ExternalController
 *   (spawns the module, IPC kind "controller") -> BeforeStep() before every
 *   DoStepDynamics.
 *
 * Channel names are `<element>.<name>.<field>`, e.g. `body.float.pos_z` or
 * `tsda.winch_rope.rest_length`. An actuator is written under the same name
 * it is read. The full list of fields, units and frames is the registry
 * returned by ControllerChannelRegistry() (also in docs/extending/CONTROLLERS.md).
 *
 * Timing: at each control tick t_k the module sees the state at t_k; its
 * commands are held (zero-order hold) over [t_k, t_k + control_dt).
 *
 * See docs/extending/CONTROLLERS.md.
 *********************************************************************/
#ifndef SEASTACK_APP_EXTERNAL_CONTROLLER_H
#define SEASTACK_APP_EXTERNAL_CONTROLLER_H

#include <seastack/config.h>

#ifdef SEASTACK_HAVE_EXTERNAL

#include <seastack/external/external_force_model.h>

#include <chrono/physics/ChSystem.h>

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace seastack::app {

/// Parsed controller YAML (`controller_file:` in the setup YAML).
struct ControllerConfig {
    /// Element names listed under `sensors:` or `actuators:`.
    struct ElementLists {
        std::vector<std::string> bodies;
        std::vector<std::string> motors;
        std::vector<std::string> tsdas;
        std::vector<std::string> rsdas;
    };

    std::string source_path;             ///< Controller YAML path (for messages).
    std::vector<std::string> command;    ///< Child argv; relative .py paths resolved.
    std::string working_directory;       ///< Child cwd (default: controller YAML dir).
    int timeout_ms = 10000;              ///< Per-call IPC timeout [ms].
    double control_dt = 0.0;             ///< Control period [s]; 0 = simulation time_step.
    ElementLists sensors;                ///< Read-only channels published each tick.
    ElementLists actuators;              ///< Channels the module may write (no bodies).
    std::string config_json = "{}";      ///< `config:` block as a JSON object.
};

/// Read and validate a controller YAML file. Throws std::runtime_error with
/// a message that names the file, the bad key and the valid choices.
ControllerConfig LoadControllerFile(const std::filesystem::path& path);

/// One row of the channel registry. Units are SI.
struct ControllerChannelSpec {
    const char* element;   ///< "body", "motor", "tsda" or "rsda"
    const char* applies;   ///< Element subtype, e.g. "rotation motor"; "" = all
    const char* field;     ///< Channel suffix, e.g. "pos_z"
    const char* unit;      ///< e.g. "m", "rad/s", "-"
    bool writable;         ///< true if it may be listed as an actuator
    const char* meaning;   ///< Frame, sign convention and source
};

/// Every channel SEA-Stack can publish or accept, in publication order.
const std::vector<ControllerChannelSpec>& ControllerChannelRegistry();

/// The registry as the Markdown table used in docs/extending/CONTROLLERS.md.
std::string ControllerChannelTableMarkdown();

/**
 * @brief Runs one external controller module against a Chrono system.
 *
 * Construction resolves every listed element (errors list the valid names),
 * builds the channel lists, and performs the IPC handshake. Call
 * BeforeStep(time) immediately before every DoStepDynamics.
 */
class ExternalController {
  public:
    /// @param sim_dt  Simulation time step [s]; control_dt must be a whole
    ///                multiple of it.
    /// @param model   Module transport; nullptr spawns `config.command` over IPC.
    ExternalController(::chrono::ChSystem& system,
                       const ControllerConfig& config,
                       double sim_dt,
                       std::unique_ptr<seastack::external::IExternalForceModel> model = nullptr);
    ~ExternalController();

    ExternalController(const ExternalController&) = delete;
    ExternalController& operator=(const ExternalController&) = delete;

    /// Evaluate the module on control ticks and (re)apply the held commands.
    /// Throws std::runtime_error (with the module's message) on failure.
    void BeforeStep(double time);

    const std::vector<std::string>& sensor_names() const { return sensor_names_; }
    const std::vector<std::string>& actuator_names() const { return actuator_names_; }
    double control_dt() const { return control_dt_; }
    /// Latest held command per actuator channel (same order as actuator_names()).
    const std::vector<double>& held_commands() const { return held_; }

  private:
    struct Actuator;

    void ResolveSensors(::chrono::ChSystem& system);
    void ResolveActuators(::chrono::ChSystem& system);
    void ApplyHeldCommands();
    void PrintModuleLog();
    void LogChannelSummary() const;

    ControllerConfig config_;
    std::string label_;  ///< Script file name, for messages.
    double control_dt_ = 0.0;
    int steps_per_tick_ = 1;
    long long step_index_ = 0;

    std::vector<std::string> sensor_names_;
    std::vector<std::string> sensor_units_;
    std::vector<std::function<double()>> sensor_readers_;
    std::vector<std::string> actuator_names_;
    std::vector<std::string> actuator_units_;
    std::vector<Actuator> actuators_;
    std::vector<double> held_;  ///< Held command per actuator channel.

    std::unique_ptr<seastack::external::IExternalForceModel> model_;
};

}  // namespace seastack::app

#endif  // SEASTACK_HAVE_EXTERNAL

#endif  // SEASTACK_APP_EXTERNAL_CONTROLLER_H
