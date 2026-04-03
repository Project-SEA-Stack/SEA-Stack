/*********************************************************************
 * @file  eta_table_wave_field.cpp
 * @brief EtaTableWaveField -- stores and interpolates a tabulated eta
 *        time series for time-domain convolution workflows.
 *********************************************************************/

#include <seastack/hydro/waves/eta_table_wave_field.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace seastack::hydro {

EtaTableWaveField::EtaTableWaveField(const std::string& eta_file_path,
                                     double depth) {
    water_depth_ = depth;

    std::ifstream file(eta_file_path);
    if (!file) {
        throw std::runtime_error(
            "EtaTableWaveField: cannot open '" + eta_file_path + "'");
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double t, eta;
        char delimiter;
        if (!(ss >> t >> delimiter >> eta) || delimiter != ':') {
            throw std::runtime_error(
                "EtaTableWaveField: parse error on line: " + line);
        }
        time_table_.push_back(t);
        eta_table_.push_back(eta);
    }

    ValidateTable();
}

EtaTableWaveField::EtaTableWaveField(std::vector<double> time,
                                     std::vector<double> eta, double depth)
    : time_table_(std::move(time)), eta_table_(std::move(eta)) {
    water_depth_ = depth;
    ValidateTable();
}

void EtaTableWaveField::ValidateTable() const {
    if (time_table_.size() < 2) {
        throw std::runtime_error(
            "EtaTableWaveField: table must have at least 2 samples (got " +
            std::to_string(time_table_.size()) + ")");
    }
    if (time_table_.size() != eta_table_.size()) {
        throw std::runtime_error(
            "EtaTableWaveField: time and eta vectors must have equal length");
    }
}

double EtaTableWaveField::GetElevation(const Eigen::Vector3d& /*position*/,
                                       double time) const {
    if (time < time_table_.front() || time > time_table_.back()) {
        return 0.0;
    }

    auto it = std::lower_bound(time_table_.begin(), time_table_.end(), time);
    auto idx = static_cast<size_t>(it - time_table_.begin());

    if (idx == 0) {
        return eta_table_[0];
    }

    const double t0 = time_table_[idx - 1];
    const double t1 = time_table_[idx];
    const double frac = (time - t0) / (t1 - t0);
    return eta_table_[idx - 1] + frac * (eta_table_[idx] - eta_table_[idx - 1]);
}

Eigen::Vector3d EtaTableWaveField::GetVelocity(
    const Eigen::Vector3d&, double, double) const {
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d EtaTableWaveField::GetAcceleration(
    const Eigen::Vector3d&, double, double) const {
    return Eigen::Vector3d::Zero();
}

double EtaTableWaveField::GetExcitationRampForVisualization(double time) const {
    if (ramp_duration_ <= 0.0 || time >= ramp_duration_) {
        return 1.0;
    }
    if (time <= 0.0) {
        return 0.0;
    }
    return time / ramp_duration_;
}

}  // namespace seastack::hydro
