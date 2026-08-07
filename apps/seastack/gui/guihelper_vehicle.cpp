#include "guihelper_vehicle.h"

#include "guihelper_impl.h"

#include <chrono/physics/ChBody.h>
#include <chrono/utils/ChConstants.h>
#include <chrono/utils/ChUtilsChaseCamera.h>
#include <chrono_vehicle/ChSubsysDefs.h>
#include <chrono_vehicle/ChVehicle.h>
#include <chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h>
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
/// Key 3 (Track) keeps Chrono's cinematic framing but translates with the vehicle
/// (stock Track freezes the eye and only pans the look-at).
///
/// Templated on the Chrono vehicle visual system so the wheeled and tracked
/// variants share one implementation. Everything used here lives on the common
/// base ChVehicleVisualSystemVSG.
template <typename VisBase>
class SeastackVehicleVisualSystemVSG : public VisBase {
  public:
    void Initialize() override {
        VisBase::Initialize();

        if (this->m_viewer && this->m_vsg_camera) {
            // Mouse orbit / pan / zoom only. The default Trackball binds W/A/S/D
            // (and Q/E) to camera motion, which steals those keys from the
            // vehicle driver — remap them so WASD always drives.
            auto trackball = vsg::Trackball::create(this->m_vsg_camera);
            trackball->turnLeftKey = vsg::KEY_Undefined;
            trackball->turnRightKey = vsg::KEY_Undefined;
            trackball->pitchUpKey = vsg::KEY_Undefined;
            trackball->pitchDownKey = vsg::KEY_Undefined;
            trackball->rollLeftKey = vsg::KEY_Undefined;
            trackball->rollRightKey = vsg::KEY_Undefined;
            this->m_viewer->addEventHandler(trackball);
        }

        // Free = fixed viewpoint owned by the mouse trackball while you drive.
        // Key 2 = shallow rear-right third-person; key 3 = cinematic follow;
        // keys 1/4 are Chrono Chase / Inside; key 5 returns to Free.
        this->SetChaseCameraState(::chrono::utils::ChChaseCamera::Free);
    }

    void Advance(double step) override {
        using ::chrono::utils::ChChaseCamera;
        const auto state = this->GetChaseCamera().GetState();
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
        VisBase::Advance(step);
    }

  private:
/// Shallow rear-right chase: ~35 deg off pure aft toward starboard, low
/// enough to read the deck and waves without looking nearly top-down.
/// The framing below is fixed for this camera key; the YAML `chase_camera`
/// block configures Chrono's own chase camera (key 1), not this view.
void ApplyRearRightThirdPerson() {
        if (!this->m_vehicle || !this->m_lookAt) {
            return;
        }
        const auto chassis = this->m_vehicle->GetChassisBody();
        if (!chassis) {
            return;
        }

        constexpr double kDist = 18.0;   // m, horizontal range to chase point
        constexpr double kHeight = 4.5;  // m, above chase point (shallower pitch)
        constexpr double kAzimuth = 35.0 * (::chrono::CH_DEG_TO_RAD);  // aft -> starboard

        const ::chrono::ChVector3d target =
            chassis->TransformPointLocalToParent(this->m_camera_point);
        // Local offset: -X aft, -Y starboard (right when facing vehicle +X).
        const ::chrono::ChVector3d offset_local(-kDist * std::cos(kAzimuth),
                                                -kDist * std::sin(kAzimuth), kHeight);
        const ::chrono::ChVector3d eye =
            target + chassis->TransformDirectionLocalToParent(offset_local);

        SetLookAt(eye, target);
    }

/// On Track entry, keep the current compass bearing to the vehicle but force a
/// closer, nearly horizontal eye height. Then translate that offset with the
/// chase point each frame (cinematic follow without the top-down pitch).
/// Like the Follow view, this framing is fixed and independent of the YAML
/// `chase_camera` block.
void ApplyCinematicTrack() {
        if (!this->m_vehicle || !this->m_lookAt) {
            return;
        }
        const auto chassis = this->m_vehicle->GetChassisBody();
        if (!chassis) {
            return;
        }

        const ::chrono::ChVector3d target =
            chassis->TransformPointLocalToParent(this->m_camera_point);

        if (m_prev_state != ::chrono::utils::ChChaseCamera::Track) {
            constexpr double kDist = 9.0;    // m, closer than a wide establishing shot
            constexpr double kHeight = 1.6;  // m, shallow / nearly level look

            const ::chrono::ChVector3d eye(this->m_lookAt->eye.x, this->m_lookAt->eye.y,
                                           this->m_lookAt->eye.z);
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
        this->m_vsg_cameraEye.set(eye.x(), eye.y(), eye.z());
        this->m_vsg_cameraTarget.set(target.x(), target.y(), target.z());
        this->m_lookAt->eye.set(eye.x(), eye.y(), eye.z());
        this->m_lookAt->center.set(target.x(), target.y(), target.z());
    }

    ::chrono::utils::ChChaseCamera::State m_prev_state =
        ::chrono::utils::ChChaseCamera::Free;
    ::chrono::ChVector3d m_track_offset{0, 0, 0};
};

/// True when `body` is a Chrono::Vehicle part.  Every ChPart tags the bodies it
/// creates with VehicleObjTag::Generate(vehicle_tag, part_tag), so the part tag
/// identifies vehicle-owned bodies without relying on names.  Untagged bodies
/// carry ChObj's default tag of -1, whose low half (0xFFFF) matches no part.
bool IsVehiclePart(const ::chrono::ChBody& body) {
    namespace veh = ::chrono::vehicle;
    switch (veh::VehicleObjTag::ExtractPartTag(body.GetTag())) {
        case veh::VehiclePartTag::CHASSIS:
        case veh::VehiclePartTag::CHASSIS_REAR:
        case veh::VehiclePartTag::SUBCHASSIS:
        case veh::VehiclePartTag::SUSPENSION:
        case veh::VehiclePartTag::STEERING:
        case veh::VehiclePartTag::ANTIROLLBAR:
        case veh::VehiclePartTag::WHEEL:
        case veh::VehiclePartTag::TIRE:
        case veh::VehiclePartTag::SPROCKET:
        case veh::VehiclePartTag::IDLER:
        case veh::VehiclePartTag::TRACK_WHEEL:
        case veh::VehiclePartTag::SHOE:
        case veh::VehiclePartTag::TRACK_SUSPENSION:
            return true;
        default:
            return false;
    }
}

/// Scene paint filter for vehicle scenes.  The global painted-metal pass would
/// otherwise overwrite two sets of materials that carry their own appearance:
/// Chrono::Vehicle part meshes (chassis, wheels, tires, track shoes), which
/// would blend into the deck, and RigidTerrain patches, which are textured from
/// the terrain spec.  Chrono names patch bodies "patch_<n>" (RigidTerrain::AddPatch).
/// Everything else — hulls, decks, structures — is painted as usual.
bool ScenePaintFilterForVehicleScenes(const ::chrono::ChBody& body) {
    if (IsVehiclePart(body)) {
        return false;
    }
    return body.GetName().rfind("patch_", 0) != 0;
}

}  // namespace

/// Non-template handle onto the wrapped Chrono vehicle visual system, so
/// VehicleGUI does not need to know which concrete type the impl holds.
class VehicleVisAccess {
  public:
    virtual ~VehicleVisAccess() = default;
    virtual ::chrono::vehicle::ChVehicleVisualSystemVSG& VehicleVis() = 0;
};

/// GUIImplVSG variant whose VSG visual system is a Chrono vehicle visual system.
/// All SEA-Stack rendering (water surface, materials, overlay) is inherited; this
/// class only exposes the vehicle-specific hooks and the paint filter.
template <typename VisBase>
class GUIImplVehicleVSG : public GUIImplVSG, public VehicleVisAccess {
  public:
    GUIImplVehicleVSG()
        : GUIImplVSG(::chrono_types::make_shared<SeastackVehicleVisualSystemVSG<VisBase>>()) {}

    ::chrono::vehicle::ChVehicleVisualSystemVSG& VehicleVis() override {
        return static_cast<::chrono::vehicle::ChVehicleVisualSystemVSG&>(*pVis);
    }

  protected:
    bool ShouldApplyScenePaint(const ::chrono::ChBody& body) const override {
        return ScenePaintFilterForVehicleScenes(body);
    }
};

using WheeledImpl = GUIImplVehicleVSG<::chrono::vehicle::ChWheeledVehicleVisualSystemVSG>;
using TrackedImpl = GUIImplVehicleVSG<::chrono::vehicle::ChTrackedVehicleVisualSystemVSG>;

VehicleGUI::VehicleGUI() : GUI(std::make_shared<WheeledImpl>()) {}

VehicleGUI::VehicleGUI(std::shared_ptr<seastack::viz::GUIImpl> impl) : GUI(std::move(impl)) {}

TrackedVehicleGUI::TrackedVehicleGUI() : VehicleGUI(std::make_shared<TrackedImpl>()) {}

VehicleVisAccess* VehicleGUI::VehicleImpl() const {
    return dynamic_cast<VehicleVisAccess*>(pImpl.get());
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
