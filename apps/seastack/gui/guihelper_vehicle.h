#pragma once

// Vehicle-aware SEA-Stack GUI (only built when SEASTACK_ENABLE_VEHICLE and
// SEASTACK_ENABLE_VSG are both ON).
//
// VehicleGUI renders the scene through Chrono::Vehicle's
// ChWheeledVehicleVisualSystemVSG instead of the plain ChVisualSystemVSG, which
// adds (on top of the standard SEA-Stack water surface and ImGui overlay):
//   - WASD keyboard driving (throttle/brake/steer) via ChInteractiveDriver
//   - mouse orbit / scroll-zoom to set a fixed viewpoint, then WASD to drive
//     (trackball keyboard bindings are cleared so they do not steal WASD)
//   - key 2: shallow rear-right third-person (vehicle + deck + waves in frame)
//   - key 3: cinematic follow (closer, nearly level; bearing locked on entry)
//   - keys 1/4: Chrono chase / inside; key 5: Free / mouse view
//   - a vehicle ImGui panel (speed, gear, engine, driver inputs)
//
// Usage (see demos/vlfp/demo_vlfp_vehicle.cpp):
//   1. Construct VehicleGUI.
//   2. AttachVehicle(&vehicle, &driver, ...)  -- BEFORE Init().
//   3. Init(&system, title), SetWaveModel(...), etc. as with the standard GUI.
//   4. Each simulation step: SynchronizeVehicleVis(time, inputs) and
//      AdvanceVehicleVis(step) alongside the vehicle/driver Synchronize/Advance.

#include <seastack/config.h>

#if !defined(SEASTACK_HAVE_VEHICLE) || !defined(SEASTACK_HAVE_VSG)
#error "guihelper_vehicle.h requires SEASTACK_ENABLE_VEHICLE and SEASTACK_ENABLE_VSG"
#endif

#include <gui/guihelper.h>

#include <chrono/core/ChVector3.h>
#include <chrono_vehicle/ChDriver.h>

namespace chrono::vehicle {
class ChVehicle;
}

namespace seastack::viz {

class GUIImplVehicleVSG;

/// SEA-Stack GUI variant for interactive Chrono::Vehicle demos.
class VehicleGUI : public GUI {
  public:
    VehicleGUI();

    /// Attach the vehicle, the interactive driver, and the chase camera.
    /// Must be called before Init(). `chase_point` is on the chassis in the
    /// vehicle reference frame; distance/height are behind/above that point.
    void AttachVehicle(::chrono::vehicle::ChVehicle* vehicle,
                       ::chrono::vehicle::ChDriver* driver,
                       const ::chrono::ChVector3d& chase_point,
                       double chase_distance,
                       double chase_height);

    /// Update the vehicle visual system (driver-input display) at `time`.
    void SynchronizeVehicleVis(double time, const ::chrono::vehicle::DriverInputs& inputs);

    /// Advance the chase-camera dynamics by `step` seconds.
    void AdvanceVehicleVis(double step);

  private:
    GUIImplVehicleVSG* VehicleImpl() const;
};

}  // namespace seastack::viz
