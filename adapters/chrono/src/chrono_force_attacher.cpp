/*********************************************************************
 * @file  chrono_force_attacher.cpp
 * @brief Implementation of ChronoForceAttacher and Chrono force callbacks.
 *
 * Force flow: Chrono -> ComponentFunc::GetVal() ->
 *             ChronoForceAttacher::CoordinateFuncForBody() ->
 *             ForceEvaluator callback.
 *
 * ComponentFunc and BodyForceCallbacks are implementation details defined
 * entirely in this translation unit. They are not part of the public API.
 *********************************************************************/

#include "seastack/adapters/chrono/chrono_force_attacher.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>

#include <seastack/core/math_constants.h>
#include <seastack/infra/debug_trace.h>
#include <seastack/infra/logging.h>
#include "chrono_state_utils.h"

namespace seastack::chrono {

using ::chrono::ChBody;
using ::chrono::ChForce;

// ────────────────────────────────────────────────────────────────────
// ComponentFunc — per-DOF ChFunction callback (implementation detail)
// ────────────────────────────────────────────────────────────────────
//
// Each instance routes Chrono's ChForce::GetVal(time) call directly to
// ChronoForceAttacher::CoordinateFuncForBody(body_num, dof).
//
// LIFETIME: holds a non-owning raw pointer to ChronoForceAttacher. The
// attacher MUST outlive all Chrono force evaluations. This is guaranteed
// by the HydroSystem ownership hierarchy (HydroSystem owns the attacher
// and the Chrono body references), but is not enforced at compile time.

class ComponentFunc : public ::chrono::ChFunction {
  public:
    ComponentFunc() = default;

    ComponentFunc(ChronoForceAttacher* attacher, int body_num, int dof_index)
        : attacher_(attacher), body_num_(body_num), dof_index_(dof_index) {}

    ComponentFunc* Clone() const override { return new ComponentFunc(*this); }

    double GetVal(double) const override {
        if (attacher_ == nullptr) {
            LOG_ERROR("ComponentFunc::GetVal: attacher pointer is null "
                      "(body " << body_num_ << " DOF " << dof_index_ << ")");
            return 0.0;
        }
        return attacher_->CoordinateFuncForBody(body_num_, dof_index_);
    }

  private:
    ChronoForceAttacher* attacher_ = nullptr;
    int body_num_ = 0;
    int dof_index_ = 0;
};

// ────────────────────────────────────────────────────────────────────
// BodyForceCallbacks — per-body grouping of Chrono force objects
// ────────────────────────────────────────────────────────────────────

struct ChronoForceAttacher::BodyForceCallbacks {
    std::shared_ptr<ChForce> force;
    std::shared_ptr<ChForce> torque;
    std::array<std::shared_ptr<ComponentFunc>, kDofPerBody> funcs;
};

// ────────────────────────────────────────────────────────────────────
// ChronoForceAttacher
// ────────────────────────────────────────────────────────────────────

int ChronoForceAttacher::ParseBodyNumber(const std::string& body_name) {
    if (body_name.size() < 5 || body_name.compare(0, 4, "body") != 0) {
        LOG_ERROR("ParseBodyNumber: invalid ChBody name "
                  "(expected \"body\" prefix and at least one digit): \""
                  << body_name << "\"");
        throw std::invalid_argument(
            "ChBody name must start with \"body\" and be at least "
            "5 characters (e.g. body1), got: \"" + body_name + "\"");
    }
    const std::string suffix = body_name.substr(4);
    if (!std::all_of(suffix.begin(), suffix.end(),
                     [](unsigned char c) { return std::isdigit(c) != 0; })) {
        LOG_ERROR("ParseBodyNumber: invalid ChBody name "
                  "(suffix after \"body\" must be decimal digits): \""
                  << body_name << "\"");
        throw std::invalid_argument(
            "ChBody name must match pattern body<N> with N composed of "
            "decimal digits only, got: \"" + body_name + "\"");
    }
    return std::stoi(suffix);
}

ChronoForceAttacher::ChronoForceAttacher(
    std::vector<std::shared_ptr<::chrono::ChBody>> bodies,
    ForceEvaluator evaluator,
    bool attach_forces_to_bodies,
    DivergenceLimits limits)
    : bodies_(std::move(bodies)),
      num_bodies_(static_cast<int>(bodies_.size())),
      force_evaluator_(std::move(evaluator)),
      prev_time_(-1),
      attach_forces_to_bodies_(attach_forces_to_bodies),
      limits_(limits) {

    int total_dofs = kDofPerBody * num_bodies_;
    total_force_.assign(total_dofs, 0.0);

    if (!attach_forces_to_bodies_) {
        return;
    }

    callbacks_.reserve(num_bodies_);
    for (int b = 0; b < num_bodies_; ++b) {
        const int body_num = ParseBodyNumber(bodies_[b]->GetName());

        BodyForceCallbacks cb;
        for (int i = 0; i < kDofPerBody; ++i) {
            cb.funcs[i] = std::make_shared<ComponentFunc>(this, body_num, i);
        }

        cb.force = std::make_shared<ChForce>();
        cb.force->SetAlign(ChForce::AlignmentFrame::WORLD_DIR);
        cb.force->SetName("hydroforce");
        cb.force->SetF_x(cb.funcs[0]);
        cb.force->SetF_y(cb.funcs[1]);
        cb.force->SetF_z(cb.funcs[2]);

        cb.torque = std::make_shared<ChForce>();
        cb.torque->SetAlign(ChForce::AlignmentFrame::WORLD_DIR);
        cb.torque->SetMode(ChForce::ForceType::TORQUE);
        cb.torque->SetName("hydrotorque");
        cb.torque->SetF_x(cb.funcs[3]);
        cb.torque->SetF_y(cb.funcs[4]);
        cb.torque->SetF_z(cb.funcs[5]);

        bodies_[b]->AddForce(cb.force);
        bodies_[b]->AddForce(cb.torque);

        callbacks_.push_back(std::move(cb));
    }
}

ChronoForceAttacher::~ChronoForceAttacher() = default;

void ChronoForceAttacher::CheckBodyStateDivergence() {
    for (int bi = 0; bi < static_cast<int>(cached_state_.bodies.size()); ++bi) {
        const auto& bs = cached_state_.bodies[bi];
        const double pos_mag = bs.position.norm();
        const double vel_mag = bs.linear_velocity.norm();
        const double angvel_mag = bs.angular_velocity.norm();
        const double roll  = std::abs(bs.orientation_rpy[0]);
        const double pitch = std::abs(bs.orientation_rpy[1]);

        // Non-finite values always trip; magnitude caps only when enabled.
        const bool non_finite = !std::isfinite(pos_mag) || !std::isfinite(vel_mag)
                || !std::isfinite(angvel_mag) || !std::isfinite(roll) || !std::isfinite(pitch);
        bool magnitude_bad = false;
        if (limits_.enabled) {
            if (limits_.max_position_m > 0.0 && pos_mag > limits_.max_position_m) {
                magnitude_bad = true;
            }
            if (limits_.max_velocity_ms > 0.0 && vel_mag > limits_.max_velocity_ms) {
                magnitude_bad = true;
            }
            if (limits_.max_ang_vel_rads > 0.0 && angvel_mag > limits_.max_ang_vel_rads) {
                magnitude_bad = true;
            }
            if (limits_.max_roll_pitch_rad > 0.0
                    && (roll > limits_.max_roll_pitch_rad
                        || pitch > limits_.max_roll_pitch_rad)) {
                magnitude_bad = true;
            }
        }

        if ((non_finite || magnitude_bad) && !diverged_) {
            diverged_ = true;
            if (!divergence_logged_) {
                divergence_logged_ = true;
                LOG_ERROR("Simulation divergence detected at t="
                    << prev_time_ << " s on body" << (bi + 1)
                    << ": pos=" << pos_mag << " m, vel=" << vel_mag
                    << " m/s, ang_vel=" << angvel_mag << " rad/s"
                    << ", roll=" << (roll * kRadToDeg) << " deg"
                    << ", pitch=" << (pitch * kRadToDeg) << " deg"
                    << " (limits enabled=" << (limits_.enabled ? "true" : "false")
                    << ", max_pos=" << limits_.max_position_m << " m"
                    << ", max_vel=" << limits_.max_velocity_ms << " m/s"
                    << ", max_ang_vel=" << limits_.max_ang_vel_rads << " rad/s"
                    << ", max_roll_pitch="
                    << (limits_.max_roll_pitch_rad * kRadToDeg) << " deg)");
            }
            return;
        }
    }
}

void ChronoForceAttacher::CheckForceValidity() {
    const int total_dofs = kDofPerBody * num_bodies_;
    for (int i = 0; i < total_dofs; ++i) {
        const bool non_finite = !std::isfinite(total_force_[i]);
        const bool magnitude_bad = limits_.enabled
                && limits_.max_force_magnitude > 0.0
                && std::abs(total_force_[i]) > limits_.max_force_magnitude;
        if (non_finite || magnitude_bad) {
            const int body_idx = i / kDofPerBody;
            const int dof = i % kDofPerBody;
            if (!divergence_logged_) {
                divergence_logged_ = true;
                LOG_ERROR("Invalid force detected at t=" << prev_time_
                    << " s on body" << (body_idx + 1) << " DOF " << dof
                    << " value=" << total_force_[i]
                    << " (limits enabled=" << (limits_.enabled ? "true" : "false")
                    << ", max_force=" << limits_.max_force_magnitude << ")"
                    << " — zeroing all forces");
            }
            diverged_ = true;
            std::fill(total_force_.begin(), total_force_.end(), 0.0);
            return;
        }
    }
}

double ChronoForceAttacher::CoordinateFuncForBody(int b, int dof_index) {
    if (dof_index < 0 || dof_index >= kDofPerBody || b < 1 || b > num_bodies_) {
        throw std::out_of_range("Invalid index in CoordinateFuncForBody: "
            "b=" + std::to_string(b) + " (valid: 1-" + std::to_string(num_bodies_) + "), "
            "dof=" + std::to_string(dof_index) + " (valid: 0-5)");
    }

    const int body_num_offset = kDofPerBody * (b - 1);
    const int total_dofs      = kDofPerBody * num_bodies_;

    if (bodies_.empty() || !bodies_[0]) {
        throw std::runtime_error("bodies_ array is empty or invalid in CoordinateFuncForBody");
    }

    if (diverged_) {
        return 0.0;
    }

    if (!attach_forces_to_bodies_) {
        return 0.0;
    }

    if (bodies_[0]->GetChTime() == prev_time_) {
        return total_force_[body_num_offset + dof_index];
    }

    prev_time_ = bodies_[0]->GetChTime();

    seastack::chrono::BuildSystemStateFromChronoBodies(bodies_, cached_state_);

    CheckBodyStateDivergence();
    if (diverged_) {
        std::fill(total_force_.begin(), total_force_.end(), 0.0);
        return 0.0;
    }

    SEASTACK_TRACE_ONCE(
        std::string("ChronoForceAttacher entering ForceEvaluator at t=") +
        std::to_string(prev_time_));
    seastack::hydro::BodyForces body_forces = force_evaluator_(prev_time_);
    SEASTACK_TRACE_ONCE(
        std::string("ChronoForceAttacher returned from ForceEvaluator at t=") +
        std::to_string(prev_time_));

    std::fill(total_force_.begin(), total_force_.end(), 0.0);
    for (int body_idx = 0; body_idx < num_bodies_; ++body_idx) {
        const int offset = kDofPerBody * body_idx;
        auto vec = body_forces[body_idx].ToVector6d();
        for (int dof = 0; dof < kDofPerBody; ++dof) {
            total_force_[offset + dof] = vec[dof];
        }
    }

    CheckForceValidity();
    if (diverged_) {
        return 0.0;
    }

    if (body_num_offset + dof_index < 0 || body_num_offset >= total_dofs) {
        throw std::out_of_range("Accessing out-of-bounds index in CoordinateFuncForBody");
    }

    return total_force_[body_num_offset + dof_index];
}

}  // namespace seastack::chrono
