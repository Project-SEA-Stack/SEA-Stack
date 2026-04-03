/*********************************************************************
 * @file  wave_utilities.h
 * @brief Shared helper routines for wave models.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_WAVES_WAVE_UTILITIES_H
#define SEASTACK_HYDRO_WAVES_WAVE_UTILITIES_H

#include <cstddef>
#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>

namespace seastack::hydro {

bool is_in_deep_water(double wavenumber, double water_depth);

Eigen::Vector3d GetWheelerStretchedPosition(const Eigen::Vector3d& position,
                                            double                eta,
                                            double                water_depth,
                                            double                mwl);

double GetFreeSurfaceElevation(const Eigen::VectorXd& freqs_hz,
                               const Eigen::VectorXd& spectral_densities,
                               const Eigen::VectorXd& spectral_widths,
                               const Eigen::VectorXd& wave_phases,
                               const Eigen::VectorXd& wavenumbers,
                               const Eigen::Vector3d& position,
                               double time_value,
                               double water_depth);

std::vector<double> GetFreeSurfaceElevationTimeSeries(const Eigen::VectorXd& freqs_hz,
                                                      const Eigen::VectorXd& spectral_densities,
                                                      const Eigen::VectorXd& spectral_widths,
                                                      const Eigen::VectorXd& wave_phases,
                                                      const Eigen::VectorXd& wavenumbers,
                                                      const Eigen::Vector3d& position,
                                                      const Eigen::VectorXd& time_array,
                                                      double water_depth);

double GetEta(const Eigen::Vector3d& position,
              double time,
              double omega,
              double amplitude,
              double phase,
              double wavenumber);

/**
 * @brief Returns ∂η/∂x for a single regular wave component.
 *
 * Analytic derivative of GetEta(): ∂η/∂x = −A·k·sin(k·x − ω·t + φ).
 * Waves propagate in +X direction; ∂η/∂y = 0 by construction.
 *
 * @return Slope in x-direction (dimensionless).
 */
double GetEtaGradientX(const Eigen::Vector3d& position,
                       double time,
                       double omega,
                       double amplitude,
                       double phase,
                       double wavenumber);

double GetEtaIrregular(const Eigen::Vector3d& position,
                       double time,
                       const Eigen::VectorXd& freqs_hz,
                       const Eigen::VectorXd& spectral_densities,
                       const Eigen::VectorXd& spectral_widths,
                       const Eigen::VectorXd& wave_phases,
                       const Eigen::VectorXd& wavenumbers);

/**
 * @brief Returns ∂η/∂x for irregular (spectral) waves by summing component gradients.
 *
 * Analytic derivative: ∂η/∂x = Σ[−Aᵢ·kᵢ·sin(kᵢ·x − ωᵢ·t + φᵢ)].
 * Waves propagate in +X direction; ∂η/∂y = 0 by construction.
 *
 * @return Slope in x-direction (dimensionless).
 */
double GetEtaGradientXIrregular(const Eigen::Vector3d& position,
                                double time,
                                const Eigen::VectorXd& freqs_hz,
                                const Eigen::VectorXd& spectral_densities,
                                const Eigen::VectorXd& spectral_widths,
                                const Eigen::VectorXd& wave_phases,
                                const Eigen::VectorXd& wavenumbers);

std::vector<double> GetEtaIrregularTimeSeries(const Eigen::Vector3d& position,
                                              const std::vector<double> time_index,
                                              const Eigen::VectorXd& freqs_hz,
                                              const Eigen::VectorXd& spectral_densities,
                                              const Eigen::VectorXd& spectral_widths,
                                              const Eigen::VectorXd& wave_phases,
                                              const Eigen::VectorXd& wavenumbers);

Eigen::Vector3d GetWaterVelocity(const Eigen::Vector3d& position,
                                 double time,
                                 double omega,
                                 double amplitude,
                                 double phase,
                                 double wavenumber,
                                 double water_depth,
                                 double mwl);

Eigen::Vector3d GetWaterAcceleration(const Eigen::Vector3d& position,
                                     double time,
                                     double omega,
                                     double amplitude,
                                     double phase,
                                     double wavenumber,
                                     double water_depth,
                                     double mwl);

Eigen::Vector3d GetWaterVelocityIrregular(const Eigen::Vector3d& position,
                                          double time,
                                          const Eigen::VectorXd& freqs_hz,
                                          const Eigen::VectorXd& spectral_densities,
                                          const Eigen::VectorXd& spectral_widths,
                                          const Eigen::VectorXd& wave_phases,
                                          const Eigen::VectorXd& wavenumbers,
                                          double water_depth,
                                          double mwl);

Eigen::Vector3d GetWaterAccelerationIrregular(const Eigen::Vector3d& position,
                                              double time,
                                              const Eigen::VectorXd& freqs_hz,
                                              const Eigen::VectorXd& spectral_densities,
                                              const Eigen::VectorXd& spectral_widths,
                                              const Eigen::VectorXd& wave_phases,
                                              const Eigen::VectorXd& wavenumbers,
                                              double water_depth,
                                              double mwl);

double ComputeWaveNumber(double omega,
                         double water_depth,
                         double g,
                         double tolerance = 1e-6,
                         int max_iterations = 100);

Eigen::VectorXd ComputeWaveNumbers(const Eigen::VectorXd& omegas,
                                   double water_depth,
                                   double g,
                                   double tolerance = 1e-6,
                                   int max_iterations = 100);

Eigen::VectorXd PiersonMoskowitzSpectrumHz(Eigen::VectorXd& f, double Hs, double Tp);

Eigen::VectorXd JONSWAPSpectrumHz(Eigen::VectorXd& f,
                                  double Hs,
                                  double Tp,
                                  double gamma       = 3.3,
                                  bool is_normalized = true);

Eigen::VectorXd GetWidthArray(const Eigen::VectorXd& input_array);

// ─────────────────────────────────────────────────────────────────────────────
// Directional spreading functions
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Evaluate the cos-2s directional spreading function D(theta).
 *
 * D(theta) = C_s * cos^{2s}((theta - theta_mean) / 2)
 *
 * where C_s is the normalization constant ensuring integral over [-pi, pi] = 1.
 * The convention is that D(theta) is a probability density in direction space:
 *   integral_{-pi}^{pi} D(theta) d_theta = 1
 *
 * @param theta         Direction to evaluate at [rad]
 * @param theta_mean    Mean spreading direction [rad]
 * @param s             Spreading parameter (higher = narrower)
 * @return D(theta) [1/rad]
 */
double Cos2sSpreading(double theta, double theta_mean, double s);

/**
 * @brief Compute the normalization constant C_s for cos-2s spreading.
 *
 * C_s = Gamma(s+1) / (2*sqrt(pi)*Gamma(s+0.5))
 *
 * @param s  Spreading parameter
 * @return C_s [1/rad]
 */
double Cos2sNormalization(double s);

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVES_WAVE_UTILITIES_H

