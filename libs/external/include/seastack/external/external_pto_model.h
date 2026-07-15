/*********************************************************************
 * @file  external_pto_model.h
 * @brief IPTOModel bridge over IExternalForceModel with time-caching.
 *
 * Lean inputs:  [displacement, velocity]
 * Rich inputs:  TSDA or RSDA 17-channel layouts (see ExternalPtoChannelNames*).
 * Output:       [force]  (N on TSDA, N·m on RSDA)
 *
 * Caching matches RectifiedHydraulicPTO: one Evaluate round-trip per new
 * simulation time; repeated/earlier times return the cached force.
 *********************************************************************/
#ifndef SEASTACK_EXTERNAL_EXTERNAL_PTO_MODEL_H
#define SEASTACK_EXTERNAL_EXTERNAL_PTO_MODEL_H

#include <seastack/external/external_force_model.h>
#include <seastack/pto/pto_model.h>

#include <memory>
#include <string>
#include <vector>

namespace seastack {
namespace external {

/// Link geometry that gathers kinematics into ExternalPtoState.
enum class ExternalPtoLinkKind {
    Tsda = 0,
    Rsda = 1,
};

/// Packed kinematics for an external 1-DOF PTO (Chrono-free POD).
///
/// Universal: time, displacement, velocity, rel_accel, body1/2 pos/vel.
/// TSDA extras: length, rest_length.  RSDA extras: angle, rest_angle.
struct ExternalPtoState {
    double time = 0.0;
    double displacement = 0.0;
    double velocity = 0.0;
    double length = 0.0;
    double rest_length = 0.0;
    double angle = 0.0;
    double rest_angle = 0.0;
    double rel_accel = 0.0;
    double body1_pos[3] = {0.0, 0.0, 0.0};
    double body1_vel[3] = {0.0, 0.0, 0.0};
    double body2_pos[3] = {0.0, 0.0, 0.0};
    double body2_vel[3] = {0.0, 0.0, 0.0};
};

/// Lean channel names (n_inputs = 2).
inline const std::vector<std::string>& ExternalPtoChannelNamesLean() {
    static const std::vector<std::string> k = {"displacement", "velocity"};
    return k;
}

/// Rich TSDA channel names (n_inputs = 17).
inline const std::vector<std::string>& ExternalPtoChannelNamesTsda() {
    static const std::vector<std::string> k = {
        "displacement", "velocity", "length", "rest_length", "rel_accel",
        "body1_pos_x",  "body1_pos_y", "body1_pos_z",
        "body1_vel_x",  "body1_vel_y", "body1_vel_z",
        "body2_pos_x",  "body2_pos_y", "body2_pos_z",
        "body2_vel_x",  "body2_vel_y", "body2_vel_z",
    };
    return k;
}

/// Rich RSDA channel names (n_inputs = 17).
inline const std::vector<std::string>& ExternalPtoChannelNamesRsda() {
    static const std::vector<std::string> k = {
        "displacement", "velocity", "angle", "rest_angle", "rel_accel",
        "body1_pos_x",  "body1_pos_y", "body1_pos_z",
        "body1_vel_x",  "body1_vel_y", "body1_vel_z",
        "body2_pos_x",  "body2_pos_y", "body2_pos_z",
        "body2_vel_x",  "body2_vel_y", "body2_vel_z",
    };
    return k;
}

class ExternalPtoModel : public seastack::pto::IPTOModel {
  public:
    /// Takes ownership of the transport. Call Initialize() before ComputeForce.
    explicit ExternalPtoModel(std::unique_ptr<IExternalForceModel> backend);

    ~ExternalPtoModel() override;

    /// Select TSDA vs RSDA channel schema. Call before Initialize().
    void SetLinkKind(ExternalPtoLinkKind kind);

    /// When true, Initialize publishes the 17-channel rich layout + in_names.
    /// Default false preserves the lean 2-channel contract for replay/tests.
    void EnableRichState(bool enable);

    ExternalPtoLinkKind link_kind() const { return link_kind_; }
    bool rich_state() const { return rich_state_; }

    /// Handshake with the module. config_json is forwarded as-is (JSON object).
    ExternalMeta Initialize(double dt, const std::string& config_json = "{}");

    /// Lean IPTOModel entry (disp, vel only). Uses lean packing even if rich
    /// was enabled — prefer ComputeForce(ExternalPtoState) on the Chrono path.
    double ComputeForce(double displacement,
                        double velocity,
                        double time) override;

    /// Full-state evaluation (rich or lean pack based on EnableRichState).
    double ComputeForce(const ExternalPtoState& state);

    void Reset();
    void Shutdown();

    int evaluate_call_count() const { return evaluate_call_count_; }

  private:
    std::vector<double> PackLean(double displacement, double velocity) const;
    std::vector<double> PackRich(const ExternalPtoState& state) const;
    double EvaluatePacked(double time, const std::vector<double>& in);

    std::unique_ptr<IExternalForceModel> backend_;
    bool initialized_ = false;
    bool rich_state_ = false;
    ExternalPtoLinkKind link_kind_ = ExternalPtoLinkKind::Tsda;
    double prev_time_ = -1.0;
    double cached_force_ = 0.0;
    double last_dt_ = 0.0;
    double prev_velocity_ = 0.0;
    bool have_prev_velocity_ = false;
    int evaluate_call_count_ = 0;
    int expected_n_inputs_ = 2;
};

}  // namespace external
}  // namespace seastack

#endif  // SEASTACK_EXTERNAL_EXTERNAL_PTO_MODEL_H
