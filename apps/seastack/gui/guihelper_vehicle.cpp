#include "guihelper_vehicle.h"

#include "guihelper_impl.h"

#include <chrono/utils/ChConstants.h>
#include <chrono/utils/ChUtilsChaseCamera.h>
#include <chrono_vehicle/ChVehicle.h>
#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h>

#include <vsg/all.h>

#include <cmath>
#include <utility>

namespace seastack::viz {

namespace {

/// Chrono::Vehicle disables the VSG trackball so the chase camera owns the view.
/// SEA-Stack demos expect the usual mouse orbit / scroll-zoom, so we re-add the
/// trackball after Initialize() and only let the chase camera write the look-at
/// when it is not in Free mode (key 5, also the default).
///
/// Key 2 (Follow) is remapped to a shallow rear-right third-person view.
/// Key 3 (Track) keeps Chrono's cinematic framing but translates with the car
/// (stock Track freezes the eye and only pans the look-at).
class SeastackWheeledVehicleVisualSystemVSG
    : public ::chrono::vehicle::ChWheeledVehicleVisualSystemVSG {
  public:
    void Initialize() override {
        ::chrono::vehicle::ChWheeledVehicleVisualSystemVSG::Initialize();

        if (m_viewer && m_vsg_camera) {
            // Mouse orbit / pan / zoom only. The default Trackball binds W/A/S/D
            // (and Q/E) to camera motion, which steals those keys from the
            // vehicle driver — remap them so WASD always drives.
            auto trackball = vsg::Trackball::create(m_vsg_camera);
            trackball->turnLeftKey = vsg::KEY_Undefined;
            trackball->turnRightKey = vsg::KEY_Undefined;
            trackball->pitchUpKey = vsg::KEY_Undefined;
            trackball->pitchDownKey = vsg::KEY_Undefined;
            trackball->rollLeftKey = vsg::KEY_Undefined;
            trackball->rollRightKey = vsg::KEY_Undefined;
            m_viewer->addEventHandler(trackball);
        }

        // Free = fixed viewpoint owned by the mouse trackball while you drive.
        // Key 2 = shallow rear-right third-person; key 3 = cinematic follow;
        // keys 1/4 are Chrono Chase / Inside; key 5 returns to Free.
        SetChaseCameraState(::chrono::utils::ChChaseCamera::Free);
    }

    void Advance(double step) override {
        using ::chrono::utils::ChChaseCamera;
        const auto state = GetChaseCamera().GetState();
        if (state == ChChaseCamera::Free) {
            m_prev_state = state;
            return;
        }
        if (state == ChChaseCamera::Follow) {
            ApplyRearRightThirdPerson();
            m_prev_state = state;
            return;
        }
        if (state == ChChaseCamera::Track) {
            ApplyCinematicTrack();
            m_prev_state = state;
            return;
        }
        m_prev_state = state;
        ::chrono::vehicle::ChWheeledVehicleVisualSystemVSG::Advance(step);
    }

  private:
    /// Shallow rear-right chase: ~35 deg off pure aft toward starboard, low
    /// enough to read the deck and waves without looking nearly top-down.
    void ApplyRearRightThirdPerson() {
        if (!m_vehicle || !m_lookAt) {
            return;
        }
        const auto chassis = m_vehicle->GetChassisBody();
        if (!chassis) {
            return;
        }

        constexpr double kDist = 18.0;   // m, horizontal range to chase point
        constexpr double kHeight = 4.5;  // m, above chase point (shallower pitch)
        constexpr double kAzimuth = 35.0 * (::chrono::CH_DEG_TO_RAD);  // aft -> starboard

        const ::chrono::ChVector3d target =
            chassis->TransformPointLocalToParent(m_camera_point);
        // Local offset: -X aft, -Y starboard (right when facing vehicle +X).
        const ::chrono::ChVector3d offset_local(-kDist * std::cos(kAzimuth),
                                                -kDist * std::sin(kAzimuth), kHeight);
        const ::chrono::ChVector3d eye =
            target + chassis->TransformDirectionLocalToParent(offset_local);

        SetLookAt(eye, target);
    }

    /// On Track entry, keep the current compass bearing to the car but force a
    /// closer, nearly horizontal eye height. Then translate that offset with the
    /// chase point each frame (cinematic follow without the top-down pitch).
    void ApplyCinematicTrack() {
        if (!m_vehicle || !m_lookAt) {
            return;
        }
        const auto chassis = m_vehicle->GetChassisBody();
        if (!chassis) {
            return;
        }

        const ::chrono::ChVector3d target =
            chassis->TransformPointLocalToParent(m_camera_point);

        if (m_prev_state != ::chrono::utils::ChChaseCamera::Track) {
            constexpr double kDist = 9.0;    // m, closer than a wide establishing shot
            constexpr double kHeight = 1.6;  // m, shallow / nearly level look

            const ::chrono::ChVector3d eye(m_lookAt->eye.x, m_lookAt->eye.y,
                                           m_lookAt->eye.z);
            ::chrono::ChVector3d horiz = eye - target;
            horiz.z() = 0;
            if (horiz.Length2() < 1e-6) {
                // Fallback: aft of the vehicle if the eye is directly above.
                horiz = chassis->TransformDirectionLocalToParent(
                    ::chrono::ChVector3d(-1, 0, 0));
            } else {
                horiz.Normalize();
            }
            m_track_offset = horiz * kDist + ::chrono::ChVector3d(0, 0, kHeight);
        }

        SetLookAt(target + m_track_offset, target);
    }

    void SetLookAt(const ::chrono::ChVector3d& eye, const ::chrono::ChVector3d& target) {
        m_vsg_cameraEye.set(eye.x(), eye.y(), eye.z());
        m_vsg_cameraTarget.set(target.x(), target.y(), target.z());
        m_lookAt->eye.set(eye.x(), eye.y(), eye.z());
        m_lookAt->center.set(target.x(), target.y(), target.z());
    }

    ::chrono::utils::ChChaseCamera::State m_prev_state =
        ::chrono::utils::ChChaseCamera::Free;
    ::chrono::ChVector3d m_track_offset{0, 0, 0};
};

}  // namespace

/// GUIImplVSG variant whose VSG visual system is a ChWheeledVehicleVisualSystemVSG.
/// All SEA-Stack rendering (water surface, materials, overlay) is inherited; this
/// class only exposes the vehicle-specific hooks of the visual system.
class GUIImplVehicleVSG : public GUIImplVSG {
  public:
    GUIImplVehicleVSG()
        : GUIImplVSG(::chrono_types::make_shared<SeastackWheeledVehicleVisualSystemVSG>()) {}

    ::chrono::vehicle::ChWheeledVehicleVisualSystemVSG& VehicleVis() {
        return static_cast<::chrono::vehicle::ChWheeledVehicleVisualSystemVSG&>(*pVis);
    }

  protected:
    /// Paint only the VLFP pontoons / ground. Chrono::Vehicle chassis, wheels,
    /// and tires keep their mesh materials and textures (otherwise the global
    /// industrial-yellow pass makes the HMMWV blend into the deck).
    bool ShouldApplyScenePaint(const ::chrono::ChBody& body) const override {
        const std::string& name = body.GetName();
        if (name == "ground") {
            return true;
        }
        // vlfp_pontoons.h names bodies body1 .. body6 (must match H5 groups).
        if (name.size() == 5 && name.compare(0, 4, "body") == 0 &&
            name[4] >= '1' && name[4] <= '6') {
            return true;
        }
        return false;
    }
};

VehicleGUI::VehicleGUI() : GUI(std::make_shared<GUIImplVehicleVSG>()) {}

GUIImplVehicleVSG* VehicleGUI::VehicleImpl() const {
    return static_cast<GUIImplVehicleVSG*>(pImpl.get());
}

void VehicleGUI::AttachVehicle(::chrono::vehicle::ChVehicle* vehicle,
                               ::chrono::vehicle::ChDriver* driver,
                               const ::chrono::ChVector3d& chase_point,
                               double chase_distance,
                               double chase_height) {
    auto& vis = VehicleImpl()->VehicleVis();
    // Order matters: AttachVehicle creates the chase camera that SetChaseCamera
    // configures; both must precede Init() (which calls vis.Initialize()).
    vis.AttachVehicle(vehicle);
    vis.AttachDriver(driver);
    vis.SetChaseCamera(chase_point, chase_distance, chase_height);
}

void VehicleGUI::SynchronizeVehicleVis(double time,
                                       const ::chrono::vehicle::DriverInputs& inputs) {
    VehicleImpl()->VehicleVis().Synchronize(time, inputs);
}

void VehicleGUI::AdvanceVehicleVis(double step) {
    VehicleImpl()->VehicleVis().Advance(step);
}

}  // namespace seastack::viz
