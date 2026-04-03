/*********************************************************************
 * @file  excitation_transfer.h
 * @brief Interpolate excitation transfer functions from H5 data for
 *        a set of wave components.
 *
 * This is the standalone version of the interpolation logic previously
 * embedded in LinearDirectionalWaveField::PrecomputeExcitationTransfer().
 * Used by HydroModelBuilder to construct ExcitationComponent with
 * owned TF data, separating excitation from wave kinematics.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_EXCITATION_TRANSFER_H
#define SEASTACK_HYDRO_EXCITATION_TRANSFER_H

#include <seastack/hydro/hydro_data.h>
#include <seastack/hydro/excitation_types.h>
#include <seastack/hydro/waves/wave_component.h>

#include <Eigen/Dense>
#include <vector>

namespace seastack::hydro {

/// Interpolated excitation transfer function data in Cartesian form.
/// excitation_re[body](dof, component_index) = Re(H*)
/// excitation_im[body](dof, component_index) = Im(H*)
struct ExcitationTFData {
    std::vector<Eigen::MatrixXd> re;
    std::vector<Eigen::MatrixXd> im;
};

/// Interpolate excitation transfer functions from H5 BEM data for each
/// wave component (frequency + direction), producing per-body Re/Im matrices.
///
/// @param components   Wave component list (provides omega_i, direction_i).
/// @param reg_h5_data  Per-body regular wave info from H5 (mag + phase tensors).
/// @param h5_wave_directions  Wave heading angles from H5 [rad].
/// @param interp_method  Cartesian (Re/Im) or Polar (mag/phase) interpolation.
/// @return Per-body interpolated excitation TF in conjugate Cartesian form.
ExcitationTFData InterpolateExcitationTransfer(
    const std::vector<WaveComponent>& components,
    const std::vector<HydroData::RegularWaveInfo>& reg_h5_data,
    const Eigen::VectorXd& h5_wave_directions,
    ExcitationInterpolation interp_method);

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_EXCITATION_TRANSFER_H
