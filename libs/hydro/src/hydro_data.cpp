#include <seastack/hydro/hydro_data.h>
#include <cmath>
#include <stdexcept>

namespace seastack::hydro {

void HydroData::resize(int num_bodies) {
    body_data_.resize(num_bodies);
    reg_wave_data_.resize(num_bodies);
    irreg_wave_data_.resize(num_bodies);
}

Eigen::MatrixXd HydroData::GetInfAddedMassMatrix(int b) const {
    return body_data_[b].inf_added_mass;
}

double HydroData::GetHydrostaticStiffnessVal(int b, int i, int j) const {
    // lin_matrix is non-dimensional; apply (rho * g) to get [N/m or N.m/rad].
    return body_data_[b].lin_matrix(i, j) * sim_data_.rho * sim_data_.g;
}

Eigen::MatrixXd HydroData::GetLinMatrix(int b) const {
    return body_data_[b].lin_matrix;
}

double HydroData::GetRIRFVal(int b, int dof, int col, int s) const {
    // rirf_matrix is non-dimensional; apply rho to get [kg/s or kg.m^2/s].
    return body_data_[b].rirf_matrix(dof, col, s) * sim_data_.rho;
}

double HydroData::GetDispVolVal(int b) const {
    return body_data_[b].disp_vol;
}

Eigen::VectorXd HydroData::GetCGVector(int b) const {
    return body_data_[b].cg;
}

Eigen::VectorXd HydroData::GetCBVector(int b) const {
    return body_data_[b].cb;
}

double HydroData::GetRhoVal() const {
    return sim_data_.rho;
}

int HydroData::GetRIRFDims(int i) const {
    return body_data_[0].rirf_matrix.dimension(i);
}

double HydroData::GetExcitationIRFVal(int b, int dof, int s) const {
    return irreg_wave_data_[b].excitation_irf_matrix(dof, s);
}

Eigen::MatrixXd HydroData::GetExcitationIRF(int b) const {
    return irreg_wave_data_[b].excitation_irf_matrix;
}

Eigen::VectorXd HydroData::GetRIRFTimeVector() const {
    constexpr double kRirfTimeTolerance = 1e-10;
    auto& rirf_time_vector = body_data_[0].rirf_time_vector;
    for (size_t ii = 1; ii < body_data_.size(); ii++) {
        for (Eigen::Index jj = 0; jj < body_data_[ii].rirf_time_vector.size(); jj++) {
            if (std::abs(body_data_[ii].rirf_time_vector[jj] - rirf_time_vector[jj]) >
                kRirfTimeTolerance) {
                throw std::runtime_error(
                    "RIRF time vectors have to be exactly the same for all bodies. Difference found in body " +
                    std::to_string(ii) + " at time index " + std::to_string(jj) + ".");
            }
        }
    }
    return rirf_time_vector;
}

}  // namespace seastack::hydro
