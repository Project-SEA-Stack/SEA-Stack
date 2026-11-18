#pragma once

#include <seastack/core/mooring_viz_data.h>
#include <seastack/viz_core/simulation_pacer.h>
#include <seastack/viz_core/visual_theme.h>

#include <memory>
#include <string>

namespace chrono {
class ChSystem;
}

namespace seastack::hydro { class WaveBase; }
namespace seastack::chrono {
class HydroSystem;
struct ReplayRecording;
class RuntimeControls;
}  // namespace seastack::chrono

namespace seastack::viz {

/// Which renderer draws the GUI (command line: --viewer=seastack|chrono).
enum class ViewerBackend {
    kChrono,    ///< Chrono::VSG visual system (fallback; to be removed)
    kSeastack,  ///< SEA-Stack's own VSG viewer (default)
};

/// Parse "chrono" or "seastack"; returns false for anything else.
bool ParseViewerBackend(const std::string& name, ViewerBackend* backend);

/// Message of the exception being handled. Call only inside a catch block.
/// Reads std::exception and vsg::Exception (which does not derive from
/// std::exception and carries a Vulkan result code).
std::string CurrentExceptionMessage();

/// Viewer options fixed at start-up (from simulation.visualization YAML).
/// Visual only; never affect the simulation.
struct ViewerOptions {
    VisualTheme theme = VisualTheme::kDark;  ///< Initial theme (switchable in the GUI)
    bool theme_from_model = false;           ///< theme was set in YAML; overrides the saved preference
    bool use_model_colors = true;            ///< Keep colours set in the model YAML; palette for the rest
    /// Fixed mooring tension colour range [N]; otherwise the range auto-tracks.
    bool has_mooring_tension_range = false;
    float mooring_tension_min = 0.0f;
    float mooring_tension_max = 0.0f;
};

/// SEA-Stack User Interface.
class UI {
  public:
    UI();
    virtual ~UI() {}

    UI(const UI&)            = delete;
    UI& operator=(const UI&) = delete;

    /// Set start-up viewer options. Call before Init(). No effect in headless UI.
    virtual void SetViewerOptions(const ViewerOptions& options);

    /**@brief Initialize the system
     *
     * Should be called after the given ChSystem is fully initialized
     * The best is to call it just before the simulation loop that call IsRunning
     *
     */
    virtual void Init(::chrono::ChSystem*, const char* title);

    /// Initialize for offline replay of a recorded run (no ChSystem; nothing
    /// is stepped). Always uses the SEA-Stack viewer. Call instead of Init().
    /// @throws std::runtime_error if this UI cannot show a replay.
    virtual void InitReplay(std::unique_ptr<seastack::chrono::ReplayRecording> recording, const char* title);

    /**@brief Set Camera position and direction
     *
     */
    virtual void SetCamera(double x, double y, double z, double dirx, double diry, double dirz);

    /**@brief To call during simulation loop
     *
     */
    virtual bool IsRunning(double timestep);

    /**@brief Set the wave model for animated free-surface rendering.
     *
     * Should be called after Init() and before the simulation loop.
     * If not called, a static flat water plane is rendered.
     *
     * @param wave Shared pointer to the wave model (may be nullptr for still water).
     */
    virtual void SetWaveModel(std::shared_ptr<seastack::hydro::WaveBase> wave);

    /**@brief Set the water grid extent for visualization.
     *
     * Configures the dimensions and position of the water surface mesh.
     * Call after Init() and before SetWaveModel().
     *
     * @param width Grid extent in X direction [m] (default: 100m)
     * @param length Grid extent in Y direction [m] (default: 100m)
     * @param center_x Grid center X coordinate [m] (default: 0)
     * @param center_y Grid center Y coordinate [m] (default: 0)
     */
    virtual void SetWaterGridExtent(double width, double length,
                                    double center_x = 0.0, double center_y = 0.0);

    /**@brief Set a callback that provides mooring line node positions each frame.
     *
     * When set, the GUI will call the provider every frame to obtain line
     * geometry and render tube meshes for each mooring line.
     * Call after Init() and before the simulation loop.
     */
    virtual void SetMooringLineProvider(MooringVizProvider provider);

    /**@brief Optional radii [m] for MoorDyn line / endpoint / node VSG geometry.
     *
     * Pass a negative value for any component to keep the SEA-Stack default for
     * that component. Intended to be driven from `hydrodynamics.moordyn` YAML
     * (see `visualization_*_radius` keys). No effect in headless UI.
     */
    virtual void SetMooringVisualizationRadii(double line_radius_m,
                                              double endpoint_radius_m,
                                              double node_marker_radius_m);

    /**@brief Show the TSDA/RSDA parameters and accept edits (SEA-Stack viewer).
     *
     * Edits are queued in `controls` and applied by the simulation loop at the
     * next step boundary. `schedule_path` is where "Export schedule" writes
     * the applied changes (empty: no output directory, export disabled);
     * `time_step` [s] is recorded in that file. Call after Init(). The
     * controls must outlive the GUI loop. No effect in headless UI.
     */
    virtual void SetRuntimeControls(seastack::chrono::RuntimeControls* controls,
                                    const std::string& schedule_path, double time_step);

    /**@brief Show the hydro model's loads per component (SEA-Stack viewer).
     *
     * Force vectors and plots read the per-component forces of `hydro` from
     * each step (a copy; results are unchanged). Call after Init(). `hydro`
     * must outlive the GUI loop. No effect in headless UI or the Chrono viewer.
     */
    virtual void SetHydroForces(seastack::chrono::HydroSystem* hydro);

    /**@brief Call after every solver step (simulated time `sim_time` [s]).
     *
     * Lets the GUI record body motion between rendered frames (radiated-wave
     * overlay). Read-only; never changes the system. No effect in headless UI.
     */
    virtual void AfterStep(double sim_time);

    /**@brief return the internal system.
     *
     * Should be called after init.
     */
    ::chrono::ChSystem* GetSystem() const { return pSystem; }

    bool simulationStarted = false;  // VSG viewer provides Start button

    /// Requested playback speed [simulated s per wall s], set from the GUI.
    /// <= 0 means unlimited (step as fast as possible). Never changes dt.
    double playbackSpeed = 1.0;

    /// Run progress written by the simulation loop, displayed by the GUI.
    RunStatus runStatus;

    /// Set by the GUI while it animates without stepping (timeline playback),
    /// so the loop renders at the running frame rate instead of the paused one.
    bool animatingWithoutStepping = false;

  protected:
    ::chrono::ChSystem* pSystem;  // Do not manage the memory
};

// -----------------------------------------------------------------------------

class GUIImpl;

/// SEA-Stack Graphical User Interface.
class GUI : public UI {
  public:
    explicit GUI(ViewerBackend backend = ViewerBackend::kSeastack);
    GUI(const GUI&)            = delete;
    GUI& operator=(const GUI&) = delete;

    void SetViewerOptions(const ViewerOptions& options) override;
    void Init(::chrono::ChSystem*, const char* title) override;
    void InitReplay(std::unique_ptr<seastack::chrono::ReplayRecording> recording, const char* title) override;
    void SetCamera(double x, double y, double z, double dirx, double diry, double dirz) override;
    bool IsRunning(double timestep) override;
    void SetWaveModel(std::shared_ptr<seastack::hydro::WaveBase> wave) override;
    void SetWaterGridExtent(double width, double length,
                            double center_x = 0.0, double center_y = 0.0) override;
    void SetMooringLineProvider(MooringVizProvider provider) override;
    void SetMooringVisualizationRadii(double line_radius_m,
                                      double endpoint_radius_m,
                                      double node_marker_radius_m) override;
    void SetRuntimeControls(seastack::chrono::RuntimeControls* controls,
                            const std::string& schedule_path, double time_step) override;
    void SetHydroForces(seastack::chrono::HydroSystem* hydro) override;
    void AfterStep(double sim_time) override;

  private:
    std::shared_ptr<seastack::viz::GUIImpl> pImpl;
    ViewerBackend backend_;
    ViewerOptions viewer_options_;  ///< Re-applied if Init() falls back to the Chrono viewer
};

/**@brief Factory to create UI or GUI
 *
 */
std::shared_ptr<seastack::viz::UI> CreateUI(bool visualizationOn = true,
                                            ViewerBackend backend = ViewerBackend::kSeastack);

}  // namespace seastack::viz
