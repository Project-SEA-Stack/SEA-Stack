/*********************************************************************
 * @file  eta_table_wave_field.h
 * @brief Wave model that stores and interpolates a tabulated eta
 *        time series without spectral decomposition.
 *
 * Intended for eta-driven time-domain convolution workflows only.
 * Frequency-domain excitation is not supported (no discrete wave components);
 * pair with ExcitationMethod::kIrfConvolution (selected automatically via kAuto
 * because GetWaveMode() returns kIrregular).
 *
 * Wave kinematics (velocity, acceleration) are not available from a
 * point-elevation record and return zero.  This is acceptable for
 * radiation/diffraction models that need only surface elevation.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_WAVES_ETA_TABLE_WAVE_FIELD_H
#define SEASTACK_HYDRO_WAVES_ETA_TABLE_WAVE_FIELD_H

#include <seastack/hydro/waves/wave_base.h>
#include <string>
#include <vector>

namespace seastack::hydro {

class EtaTableWaveField : public WaveBase {
  public:
    /// Construct from a "time:eta" text file (same format as
    /// ComponentSampler::BuildFromEtaFile).
    /// @param eta_file_path  Path to the file (lines of "time:elevation").
    /// @param depth          Water depth [m] (stored for metadata only).
    EtaTableWaveField(const std::string& eta_file_path, double depth);

    /// Construct from pre-loaded vectors.
    /// @param time  Monotonically increasing time stamps [s].
    /// @param eta   Corresponding surface elevations [m].
    /// @param depth Water depth [m].
    EtaTableWaveField(std::vector<double> time, std::vector<double> eta,
                      double depth);

    void Initialize() override {}
    WaveMode GetWaveMode() const override { return WaveMode::kIrregular; }

    /// Linear interpolation of the stored eta table.
    /// Returns 0.0 for times outside the table range.
    double GetElevation(const Eigen::Vector3d& position,
                        double time) const override;

    /// Not supported -- uses base class default (returns zero vector).

    /// Not available from eta-only data -- returns zero.
    Eigen::Vector3d GetVelocity(const Eigen::Vector3d& position, double time,
                                double elevation) const override;

    /// Not available from eta-only data -- returns zero.
    Eigen::Vector3d GetAcceleration(const Eigen::Vector3d& position,
                                    double time,
                                    double elevation) const override;

    void SetRampDuration(double seconds) override { ramp_duration_ = seconds; }
    double GetRampDuration() const override { return ramp_duration_; }

    /// Linear ramp matching ExcitationIrfComponent::RampFactor (eta-table + IRF workflows).
    double GetExcitationRampForVisualization(double time) const override;

  private:
    void ValidateTable() const;

    std::vector<double> time_table_;
    std::vector<double> eta_table_;
    double ramp_duration_ = 0.0;
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVES_ETA_TABLE_WAVE_FIELD_H
