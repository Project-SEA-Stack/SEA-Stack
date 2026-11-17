#pragma once

#include <seastack/viz_core/radiated_wave_synthesis.h>
#include <seastack/viz_core/radiated_waves.h>
#include <seastack/viz_core/scene_model.h>

#include <array>

namespace seastack::chrono {
class HydroSystem;
}

namespace seastack::viz {

struct ViewerSettings;

/// Radiated waves of a live run for the water surface (visual only; never
/// affects the simulation). Records the hydro bodies' motion after every
/// solver step, builds the far-field model for the method chosen in the
/// panel from the BEM data, and synthesises the field at the displayed time.
class RadiatedWaveOverlay {
  public:
    /// Live hydro model (not owned; must outlive the overlay). Without one
    /// (e.g. replaying a recording) every method is reported unavailable.
    void SetHydro(const seastack::chrono::HydroSystem* hydro);

    /// Record the hydro bodies' state after a solver step at simulated time t [s].
    void RecordStep(double t);

    /// Write the panel readouts, follow the chosen method and the water grid
    /// (field drawn out to the grid diagonal, wavelengths down to 4 cells),
    /// and, if `advance`, bring the field to display time t [s]. Returns the
    /// field to draw, or null if none.
    const RadiatedWaveSynthesizer* Update(ViewerSettings* settings, const SceneDescription& scene, double t,
                                          bool advance);

  private:
    /// Rebuild the model and the synthesiser for `method` and the grid.
    void Configure(RadiatedWaveMethod method, double r_max, double omega_cap, const SceneDescription& scene);

    const seastack::chrono::HydroSystem* hydro_ = nullptr;
    bool inputs_built_ = false;
    RadiationVizInputs inputs_;
    std::array<MethodAvailability, kNumRadiatedWaveMethods> availability_{};
    RadiationVelocityHistory history_{0.1};
    RadiatedWaveMethod configured_method_ = RadiatedWaveMethod::kOff;
    double configured_r_max_ = 0.0;      ///< [m]
    double configured_omega_cap_ = 0.0;  ///< [rad/s]
    RadiatedWaveModel model_;
    RadiatedWaveSynthesizer synth_;
};

}  // namespace seastack::viz
