/*********************************************************************
 * @file  hydro_setup_helpers.h
 * @brief Helper functions for constructing force components from HydroData.
 *
 * These utilities extract the physics-level quantities needed by force
 * components (equilibrium positions, buoyancy offsets, RIRF integration
 * weights) from HydroData.  They live in the hydro library so that
 * any solver adapter — or standalone code — can use them without
 * duplicating the logic.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_SETUP_HELPERS_H
#define SEASTACK_HYDRO_SETUP_HELPERS_H

#include <seastack/core/types.h>
#include <seastack/hydro/hydro_data.h>
#include <Eigen/Dense>
#include <vector>

namespace seastack::hydro {

constexpr int kDofLinOrRot = 3;

/// Compute the equilibrium position vector [6N] from HydroData.
/// Layout: (x, y, z, 0, 0, 0) for body 0, then body 1, etc.
inline std::vector<double> ComputeEquilibrium(const HydroData& data,
                                              int num_bodies) {
    std::vector<double> equilibrium(kDofPerBody * num_bodies, 0.0);
    for (int b = 0; b < num_bodies; ++b) {
        for (int i = 0; i < kDofLinOrRot; ++i) {
            equilibrium[i + kDofPerBody * b] = data.GetCGVector(b)[i];
        }
    }
    return equilibrium;
}

/// Compute the center-of-buoyancy minus center-of-gravity offset [3N].
/// Layout: (x, y, z) for body 0, then body 1, etc.
inline std::vector<double> ComputeCbMinusCg(const HydroData& data,
                                            int num_bodies) {
    std::vector<double> cb_minus_cg(kDofLinOrRot * num_bodies, 0.0);
    for (int b = 0; b < num_bodies; ++b) {
        for (int i = 0; i < kDofLinOrRot; ++i) {
            cb_minus_cg[i + kDofLinOrRot * b] =
                data.GetCBVector(b)[i] - data.GetCGVector(b)[i];
        }
    }
    return cb_minus_cg;
}

/// Compute the width (trapezoidal integration weight) vector for RIRF.
inline Eigen::VectorXd ComputeRirfWidthVector(const Eigen::VectorXd& time_vector) {
    Eigen::VectorXd widths(time_vector.size());
    for (Eigen::Index i = 0; i < widths.size(); ++i) {
        widths[i] = 0.0;
        if (i < time_vector.size() - 1)
            widths[i] += 0.5 * std::abs(time_vector[i + 1] - time_vector[i]);
        if (i > 0)
            widths[i] += 0.5 * std::abs(time_vector[i] - time_vector[i - 1]);
    }
    return widths;
}

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_SETUP_HELPERS_H
