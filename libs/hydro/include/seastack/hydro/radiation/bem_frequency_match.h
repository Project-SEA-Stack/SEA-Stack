/*********************************************************************
 * @file  bem_frequency_match.h
 * @brief Match a regular-wave frequency to a BEM grid point (no interpolation).
 *
 * Used by the constant-coefficient radiation method, which applies A(w) and
 * B(w) exactly as stored in the BEM data. A requested frequency is accepted
 * only if it is a numerically equivalent representation of one grid point;
 * frequencies between grid points or outside the grid are rejected.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_RADIATION_BEM_FREQUENCY_MATCH_H
#define SEASTACK_HYDRO_RADIATION_BEM_FREQUENCY_MATCH_H

#include <Eigen/Dense>

namespace seastack::hydro {

/// Result of matching a requested wave frequency to the BEM grid.
struct BemFrequencyMatch {
    int index = -1;               ///< Index into the BEM frequency list
    double omega_requested = 0.0; ///< Requested angular frequency [rad/s]
    double omega_matched = 0.0;   ///< Stored BEM grid frequency w_k [rad/s]
    double relative_difference = 0.0;  ///< |w_k - w_requested| / w_k [-]
};

/**
 * @brief Find the BEM grid point that represents `omega_requested`.
 *
 * A grid point w_k matches when |w_k - omega_requested| / w_k <= relative_tolerance.
 *
 * @param bem_omegas         BEM frequency grid [rad/s], as stored in the H5 file.
 * @param omega_requested    Regular-wave angular frequency [rad/s] (2*pi/T).
 * @param relative_tolerance Relative matching tolerance [-], > 0.
 * @return The unique matching grid point.
 * @throws std::runtime_error if omega_requested lies outside the grid, between
 *         grid points (no match within tolerance), or within tolerance of more
 *         than one grid point. Messages list the requested and nearest
 *         period/frequency, the tolerance, and the available periods.
 */
BemFrequencyMatch MatchBemFrequency(const Eigen::VectorXd& bem_omegas,
                                    double omega_requested,
                                    double relative_tolerance);

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_RADIATION_BEM_FREQUENCY_MATCH_H
