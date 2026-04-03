/*********************************************************************
 * @file  hydro_data.h
 * @brief HydroData: Chrono-free container for BEM hydrodynamic coefficients.
 *
 * This class holds all coefficients parsed from BEMIO-formatted HDF5 files.
 * It uses only Eigen types and has no dependency on HDF5 or Chrono.
 * Populated by H5FileInfo::ReadH5Data() (in SEAStack::HydroIO).
 *
 * ## Encapsulation
 * Data members are private. Population is via the friend H5FileInfo reader.
 * All downstream access goes through const getters. Getters that apply
 * scaling (rho, rho*g) are documented; unscaled raw data is not directly
 * accessible, eliminating the risk of inconsistent access.
 *********************************************************************/
#ifndef SEASTACK_HYDRO_DATA_H
#define SEASTACK_HYDRO_DATA_H

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

namespace seastack::hydro_io {
class H5FileInfo;
}

namespace seastack::hydro {

class HydroData {
  public:
    struct BodyInfo {
        std::string body_name;                         ///< Name from the H5 file
        int body_num = 0;                              ///< 0-based body index
        double disp_vol = 0.0;                         ///< Displaced volume [m^3]
        Eigen::VectorXd rirf_time_vector;              ///< RIRF time samples [s]
        double rirf_timestep = 0.0;                    ///< RIRF uniform time step [s]
        Eigen::VectorXd cg;                            ///< Centre of gravity [m], 3-vector. In the BEM reference frame (typically the mean free surface frame).
        Eigen::VectorXd cb;                            ///< Centre of buoyancy [m], 3-vector. In the BEM reference frame (typically the mean free surface frame).
        Eigen::MatrixXd lin_matrix;                    ///< Hydrostatic stiffness, 6x6. Non-dimensional; scaled by (rho * g) when accessed via GetHydrostaticStiffnessVal().
        Eigen::MatrixXd inf_added_mass;                ///< Infinite-frequency added mass for this body's rows: typically 6×(6N) when BEMIO stores full hydrodynamic coupling (N = number of hydro bodies, nDoF=6N), or 6×6 for a self-block-only export. Already scaled by rho during H5 import. Dimensional [kg or kg.m^2].
        Eigen::Tensor<double, 3> rirf_matrix;          ///< Radiation impulse response. Non-dimensional; scaled by rho when accessed via GetRIRFVal(). 6x6xN tensor indexed by (DOF_i, DOF_j, time_sample).
    };
    struct SimulationParameters {
        std::string h5_file_name;                      ///< Source HDF5 file path
        double rho = 0.0;                              ///< Water density [kg/m^3]
        double g = 0.0;                                ///< Gravitational acceleration [m/s^2]
        double water_depth = 0.0;                      ///< Water depth [m] (Inf = deep water)
        Eigen::VectorXd wave_directions;               ///< Wave directions [rad]
    };
    struct RegularWaveInfo {
        Eigen::VectorXd freq_list;                     ///< Wave frequencies [rad/s]
        Eigen::Tensor<double, 3> excitation_mag_matrix;   ///< Excitation force magnitude [N/m], 6xNdir x Nfreq
        Eigen::Tensor<double, 3> excitation_phase_matrix; ///< Excitation force phase [rad], 6 x Ndir x Nfreq
    };
    struct IrregularWaveInfo {
        Eigen::VectorXd excitation_irf_time;           ///< Excitation IRF time vector [s]
        Eigen::MatrixXd excitation_irf_matrix;         ///< Excitation IRF [N/m/s], 6 x Nt
        std::optional<Eigen::MatrixXd> excitation_irf_resampled;      ///< Resampled excitation IRF [N/m/s]
        std::optional<Eigen::MatrixXd> excitation_irf_time_resampled; ///< Resampled time vector [s]
    };

    HydroData() = default;

    /// Resize internal storage for the given number of bodies.
    void resize(int num_bodies);

    /// Number of hydrodynamic bodies in this dataset.
    int num_bodies() const { return static_cast<int>(body_data_.size()); }

    // ── Scaled accessors (apply rho or rho*g) ────────────────────────

    Eigen::MatrixXd GetInfAddedMassMatrix(int b) const;
    /// Returns hydrostatic stiffness (i,j) scaled by rho*g [N/m or N.m/rad].
    double GetHydrostaticStiffnessVal(int b, int i, int j) const;
    Eigen::MatrixXd GetLinMatrix(int b) const;
    /// Returns RIRF value scaled by rho [kg/s or kg.m^2/s].
    double GetRIRFVal(int b, int dof, int col, int s) const;
    double GetDispVolVal(int b) const;
    Eigen::VectorXd GetCGVector(int b) const;
    Eigen::VectorXd GetCBVector(int b) const;
    double GetExcitationIRFVal(int b, int dof, int s) const;
    Eigen::MatrixXd GetExcitationIRF(int b) const;
    int GetRIRFDims(int i) const;
    Eigen::VectorXd GetRIRFTimeVector() const;
    double GetRhoVal() const;

    // ── Const collection accessors ────────────────────────────────────

    const std::vector<BodyInfo>& GetBodyInfos() const { return body_data_; }
    const SimulationParameters& GetSimulationInfo() const { return sim_data_; }
    const std::vector<RegularWaveInfo>& GetRegularWaveInfos() const { return reg_wave_data_; }
    const std::vector<IrregularWaveInfo>& GetIrregularWaveInfos() const { return irreg_wave_data_; }

  private:
    friend class seastack::hydro_io::H5FileInfo;

    std::vector<BodyInfo> body_data_;
    SimulationParameters sim_data_;
    std::vector<RegularWaveInfo> reg_wave_data_;
    std::vector<IrregularWaveInfo> irreg_wave_data_;
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_DATA_H
