#include <seastack/hydro/hydro_data.h>
#include <cmath>
#include <stdexcept>
#include <string>

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

namespace {

/// Stack each body's 6 x 6N slice at frequency index k into a 6N x 6N matrix.
/// Returns the raw (normalized) H5 values.
template <typename SliceGetter>
Eigen::MatrixXd StackBodyRowsAtFrequency(int num_bodies, int k, SliceGetter get_tensor,
                                         const char* what) {
    const int n_dofs = 6 * num_bodies;
    Eigen::MatrixXd out(n_dofs, n_dofs);
    for (int b = 0; b < num_bodies; ++b) {
        const Eigen::Tensor<double, 3>& t = get_tensor(b);
        if (t.dimension(0) != 6 || t.dimension(1) != n_dofs || k < 0 || k >= t.dimension(2)) {
            throw std::runtime_error(
                std::string("HydroData: ") + what + " for body " + std::to_string(b + 1) +
                " has shape " + std::to_string(t.dimension(0)) + "x" +
                std::to_string(t.dimension(1)) + "x" + std::to_string(t.dimension(2)) +
                "; expected 6x" + std::to_string(n_dofs) + "xNf with frequency index " +
                std::to_string(k) + " in range.");
        }
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < n_dofs; ++j) {
                out(6 * b + i, j) = t(i, j, k);
            }
        }
    }
    return out;
}

}  // namespace

bool HydroData::HasFrequencyDomainRadiation() const {
    const int n = num_bodies();
    if (n == 0 || reg_wave_data_.empty()) {
        return false;
    }
    const Eigen::Index n_freq = reg_wave_data_[0].freq_list.size();
    if (n_freq == 0) {
        return false;
    }
    for (const auto& body : body_data_) {
        for (const auto* t : {&body.added_mass_all, &body.radiation_damping_all}) {
            if (t->dimension(0) != 6 || t->dimension(1) != 6 * n || t->dimension(2) != n_freq) {
                return false;
            }
        }
    }
    return true;
}

Eigen::VectorXd HydroData::GetBemFrequencies() const {
    if (reg_wave_data_.empty()) {
        return Eigen::VectorXd();
    }
    return reg_wave_data_[0].freq_list;
}

Eigen::MatrixXd HydroData::GetAddedMassAtFrequencyIndex(int k) const {
    // added_mass_all is normalized by rho; A = rho * A_file.
    Eigen::MatrixXd a = StackBodyRowsAtFrequency(
        num_bodies(), k, [this](int b) -> const Eigen::Tensor<double, 3>& {
            return body_data_[b].added_mass_all;
        },
        "added_mass/all");
    return a * sim_data_.rho;
}

Eigen::MatrixXd HydroData::GetRadiationDampingAtFrequencyIndex(int k) const {
    // radiation_damping_all is normalized by rho*w; B = rho * w_k * B_file,
    // using the stored grid frequency w_k.
    Eigen::MatrixXd b_mat = StackBodyRowsAtFrequency(
        num_bodies(), k, [this](int b) -> const Eigen::Tensor<double, 3>& {
            return body_data_[b].radiation_damping_all;
        },
        "radiation_damping/all");
    const Eigen::VectorXd w = GetBemFrequencies();
    if (k < 0 || k >= w.size()) {
        throw std::runtime_error("HydroData: frequency index " + std::to_string(k) +
                                 " is outside the BEM frequency list (size " +
                                 std::to_string(w.size()) + ").");
    }
    return b_mat * (sim_data_.rho * w[k]);
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
