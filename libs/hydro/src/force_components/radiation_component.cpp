/*********************************************************************
 * @file  radiation_component.cpp
 * @brief Implementation of radiation damping force component.
 *********************************************************************/

#include <seastack/hydro/force_components/radiation_component.h>
#include <seastack/core/force_component.h>
#include <seastack/hydro/heading_frame.h>
#include <seastack/hydro/hydro_data.h>

#include <Eigen/Dense>
#include <algorithm>
#include <stdexcept>

namespace seastack::hydro {

RadiationComponent::RadiationComponent(
    const HydroData& file_info,
    int num_bodies,
    int rirf_steps,
    const Eigen::VectorXd& rirf_time_vector,
    const Eigen::VectorXd& rirf_width_vector,
    const RadiationKernelProcessing& kernel_processing,
    const std::string& diagnostics_output_dir)
    : file_info_(file_info),
      num_bodies_(num_bodies),
      kernel_processing_(kernel_processing),
      diagnostics_output_dir_(diagnostics_output_dir),
      convolution_(std::make_unique<RadiationRirfConvolution>(
          num_bodies, rirf_steps, rirf_time_vector, rirf_width_vector)),
      rirf_time_vector_(rirf_time_vector),
      rirf_width_vector_(rirf_width_vector),
      rirf_processed_ready_(false) {
    // Require at least one body for radiation damping computation.
    if (num_bodies_ <= 0) {
        throw std::invalid_argument(
            "RadiationComponent: num_bodies must be > 0 (got " + std::to_string(num_bodies_) + ")");
    }

    // Validate RIRF time series data from HDF5.
    if (rirf_steps <= 0) {
        throw std::invalid_argument(
            "RadiationComponent: rirf_steps must be > 0 (got " + std::to_string(rirf_steps) +
            "). Check that HDF5 file contains valid RIRF data.");
    }

    // Validate RIRF time and width vectors match the expected size.
    if (static_cast<int>(rirf_time_vector_.size()) != rirf_steps) {
        throw std::invalid_argument(
            "RadiationComponent: rirf_time_vector size mismatch (expected " +
            std::to_string(rirf_steps) + ", got " + std::to_string(rirf_time_vector_.size()) + ")");
    }
    if (static_cast<int>(rirf_width_vector_.size()) != rirf_steps) {
        throw std::invalid_argument(
            "RadiationComponent: rirf_width_vector size mismatch (expected " +
            std::to_string(rirf_steps) + ", got " + std::to_string(rirf_width_vector_.size()) + ")");
    }
    
    // Pre-allocate body velocities buffer (num_bodies_ x 6 DOF)
    body_velocities_buffer_.resize(num_bodies_);
    for (int b = 0; b < num_bodies_; ++b) {
        body_velocities_buffer_[b].resize(6);
    }
}

void RadiationComponent::EnsureProcessedRIRF() {
    if (rirf_processed_ready_) {
        return;
    }

    const int steps = static_cast<int>(rirf_time_vector_.size());

    auto get_rirf_val = [this](int body, int row_dof, int col, int step) -> double {
        return file_info_.GetRIRFVal(body, row_dof, col, step);
    };

    rirf_processed_ = ProcessRirfKernels(
        num_bodies_, steps, rirf_time_vector_, get_rirf_val, kernel_processing_, diagnostics_output_dir_);

    rirf_processed_ready_ = true;
}

double RadiationComponent::GetRIRFval(int row, int col, int st) {
    if (row < 0 || row >= kDofPerBody * num_bodies_ || col < 0 || col >= kDofPerBody * num_bodies_ || st < 0 ||
        st >= file_info_.GetRIRFDims(2)) {
        throw std::out_of_range("rirfval index out of range in RadiationComponent");
    }

    int body_index = row / kDofPerBody;
    int row_dof    = row % kDofPerBody;

    if (kernel_processing_.RequiresProcessing()) {
        EnsureProcessedRIRF();
        const auto& tensor = rirf_processed_[body_index];
        return tensor(row_dof, col, st);
    }

    return file_info_.GetRIRFVal(body_index, row_dof, col, st);
}

void RadiationComponent::Compute(const SystemState& state,
                             double time,
                             BodyForces& inout_forces) {
    // Internal consistency check: state and forces must match expected body count.
    if (static_cast<int>(state.bodies.size()) != num_bodies_) {
        throw std::runtime_error(
            "RadiationComponent::Compute: state.bodies.size() mismatch (expected " +
            std::to_string(num_bodies_) + ", got " + std::to_string(state.bodies.size()) + ")");
    }
    if (static_cast<int>(inout_forces.size()) != num_bodies_) {
        throw std::runtime_error(
            "RadiationComponent::Compute: inout_forces.size() mismatch (expected " +
            std::to_string(num_bodies_) + ", got " + std::to_string(inout_forces.size()) + ")");
    }

    if (kernel_processing_.RequiresProcessing()) {
        EnsureProcessedRIRF();
    }

    // Prepare body velocities for recording (use member buffer), in each
    // body's heading frame (heading_frame.h).
    for (int b = 0; b < num_bodies_; ++b) {
        const auto& body_state = state.bodies[b];
        const Eigen::Matrix3d world_to_heading = HeadingToWorld(body_state).transpose();
        const Eigen::Vector3d v_lin = world_to_heading * body_state.linear_velocity;
        const Eigen::Vector3d v_ang = world_to_heading * body_state.angular_velocity;
        for (int d = 0; d < 3; ++d) {
            body_velocities_buffer_[b][d]     = v_lin[d];
            body_velocities_buffer_[b][3 + d] = v_ang[d];
        }
    }

    // Record velocities in the convolution module
    convolution_->RecordVelocities(time, body_velocities_buffer_);

    auto get_rirf_val = [this](int row, int col, int step) -> double {
        return GetRIRFval(row, col, step);
    };

    const std::vector<Eigen::Tensor<double, 3>>* processed_rirf = 
        (kernel_processing_.RequiresProcessing() && rirf_processed_ready_) 
        ? &rirf_processed_ : nullptr;

    std::vector<double> force_flat = convolution_->ComputeForces(get_rirf_val, processed_rirf);

    // Map flat vector to per-body forces and add to inout_forces.
    // Radiation damping opposes motion, so we SUBTRACT (add with negative sign).
    // This ensures HydroSystem::Evaluate() produces: total = hydrostatics - radiation + waves.
    for (int b = 0; b < num_bodies_; ++b) {
        const int body_offset = kDofPerBody * b;
        const Eigen::Matrix3d heading_to_world = HeadingToWorld(state.bodies[b]);
        const Eigen::Vector3d f_heading(force_flat[body_offset + 0], force_flat[body_offset + 1],
                                        force_flat[body_offset + 2]);
        const Eigen::Vector3d m_heading(force_flat[body_offset + 3], force_flat[body_offset + 4],
                                        force_flat[body_offset + 5]);
        inout_forces[b].force  -= heading_to_world * f_heading;
        inout_forces[b].moment -= heading_to_world * m_heading;
    }
}

}  // namespace seastack::hydro

