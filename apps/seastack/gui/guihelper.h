#pragma once

#include <seastack/core/mooring_viz_data.h>

#include <memory>

namespace chrono {
class ChSystem;
}

namespace seastack::hydro { class WaveBase; }

namespace seastack::viz {

/// SEA-Stack User Interface.
class UI {
  public:
    UI();
    virtual ~UI() {}

    UI(const UI&)            = delete;
    UI& operator=(const UI&) = delete;

    /**@brief Initialize the system
     *
     * Should be called after the given ChSystem is fully initialized
     * The best is to call it just before the simulation loop that call IsRunning
     *
     */
    virtual void Init(::chrono::ChSystem*, const char* title);

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

    /**@brief return the internal system.
     *
     * Should be called after init.
     */
    ::chrono::ChSystem* GetSystem() const { return pSystem; }

    bool simulationStarted = false;  // VSG viewer provides Start button

  protected:
    ::chrono::ChSystem* pSystem;  // Do not manage the memory
};

// -----------------------------------------------------------------------------

class GUIImpl;

/// SEA-Stack Graphical User Interface.
class GUI : public UI {
  public:
    GUI();
    GUI(const GUI&)            = delete;
    GUI& operator=(const GUI&) = delete;

    void Init(::chrono::ChSystem*, const char* title) override;
    void SetCamera(double x, double y, double z, double dirx, double diry, double dirz) override;
    bool IsRunning(double timestep) override;
    void SetWaveModel(std::shared_ptr<seastack::hydro::WaveBase> wave) override;
    void SetWaterGridExtent(double width, double length,
                            double center_x = 0.0, double center_y = 0.0) override;
    void SetMooringLineProvider(MooringVizProvider provider) override;
    void SetMooringVisualizationRadii(double line_radius_m,
                                      double endpoint_radius_m,
                                      double node_marker_radius_m) override;

  private:
    std::shared_ptr<seastack::viz::GUIImpl> pImpl;
};

/**@brief Factory to create UI or GUI
 *
 */
std::shared_ptr<seastack::viz::UI> CreateUI(bool visualizationOn = true);

}  // namespace seastack::viz
