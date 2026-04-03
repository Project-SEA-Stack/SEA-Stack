/*********************************************************************
 * @file  excitation_transfer.cpp
 * @brief Standalone excitation TF interpolation.
 *
 * Extracted from LinearDirectionalWaveField::PrecomputeExcitationTransfer()
 * to decouple excitation force data from wave kinematics.
 *********************************************************************/

#include <seastack/hydro/excitation_transfer.h>
#include <seastack/core/types.h>
#include <seastack/infra/logging.h>

#include <algorithm>
#include <cmath>
#include <set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace seastack::hydro {

namespace {
constexpr double kTwoPi = 2.0 * M_PI;
}  // namespace

ExcitationTFData InterpolateExcitationTransfer(
    const std::vector<WaveComponent>& components,
    const std::vector<HydroData::RegularWaveInfo>& reg_h5_data,
    const Eigen::VectorXd& h5_wave_directions,
    ExcitationInterpolation interp_method) {

    const int n_bodies = static_cast<int>(reg_h5_data.size());
    const Eigen::Index n_comp = static_cast<Eigen::Index>(components.size());
    const Eigen::Index n_h5_freq = (n_bodies > 0 && reg_h5_data[0].freq_list.size() > 0)
        ? reg_h5_data[0].freq_list.size() : 0;
    const Eigen::Index n_h5_dir = h5_wave_directions.size();

    if (n_h5_dir <= 1 && n_comp > 1) {
        std::set<double> unique_dirs;
        for (const auto& c : components) unique_dirs.insert(c.direction);
        if (unique_dirs.size() > 1) {
            LOG_WARNING("H5 data contains only " << n_h5_dir
                << " wave heading(s) but the sea state has "
                << unique_dirs.size() << " distinct component directions. "
                << "Excitation forces will use a single-heading approximation.");
        }
    }

    const Eigen::VectorXd& h5_freq_list = (n_bodies > 0) ? reg_h5_data[0].freq_list : Eigen::VectorXd();
    const double omega_min_h5 = (n_h5_freq > 0) ? h5_freq_list[0] : 0.0;
    const double omega_max_h5 = (n_h5_freq > 0) ? h5_freq_list[n_h5_freq - 1] : 0.0;
    const double omega_range  = omega_max_h5 - omega_min_h5;

    ExcitationTFData result;
    result.re.resize(n_bodies);
    result.im.resize(n_bodies);

    for (int b = 0; b < n_bodies; ++b) {
        result.re[b].resize(kDofPerBody, n_comp);
        result.im[b].resize(kDofPerBody, n_comp);

        for (Eigen::Index ci = 0; ci < n_comp; ++ci) {
            int f_lo = 0;
            int f_hi = 0;
            double f_frac = 0.0;
            if (n_h5_freq == 1) {
                f_lo = 0;
                f_hi = 0;
                f_frac = 0.0;
            } else if (n_h5_freq > 1) {
                double freq_idx = 0.0;
                if (omega_range > 0.0) {
                    freq_idx = (components[ci].omega - omega_min_h5) / omega_range
                               * static_cast<double>(n_h5_freq - 1);
                }
                f_lo = static_cast<int>(std::floor(freq_idx));
                f_lo = std::clamp(f_lo, 0, static_cast<int>(n_h5_freq - 2));
                f_hi = f_lo + 1;
                f_frac = std::clamp(freq_idx - static_cast<double>(f_lo), 0.0, 1.0);
            }

            int h_lo = 0;
            int h_hi = 0;
            double h_frac = 0.0;

            if (n_h5_dir > 1) {
                constexpr double kDirEps = 1e-12;

                double comp_dir = std::fmod(components[ci].direction, kTwoPi);
                if (comp_dir < 0.0) comp_dir += kTwoPi;

                const double* dir_begin = h5_wave_directions.data();
                const double* dir_end   = dir_begin + n_h5_dir;
                const double* it = std::lower_bound(dir_begin, dir_end, comp_dir);

                if (it == dir_begin || it == dir_end) {
                    h_lo = static_cast<int>(n_h5_dir - 1);
                    h_hi = 0;
                    double gap = (h5_wave_directions[0] + kTwoPi) - h5_wave_directions[h_lo];
                    if (gap > kDirEps) {
                        double dist = comp_dir - h5_wave_directions[h_lo];
                        if (dist < 0.0) dist += kTwoPi;
                        h_frac = dist / gap;
                        h_frac = std::clamp(h_frac, 0.0, 1.0);
                    }
                } else {
                    h_lo = static_cast<int>(it - dir_begin - 1);
                    h_hi = h_lo + 1;
                    const double dir_lo = h5_wave_directions[h_lo];
                    const double dir_hi = h5_wave_directions[h_hi];
                    if (std::abs(dir_hi - dir_lo) > kDirEps) {
                        h_frac = (comp_dir - dir_lo) / (dir_hi - dir_lo);
                        h_frac = std::clamp(h_frac, 0.0, 1.0);
                    }
                }
            }

            for (int dof = 0; dof < kDofPerBody; ++dof) {
                if (interp_method == ExcitationInterpolation::kPolar) {
                    auto interp_freq_polar = [&](int h_idx) -> std::pair<double, double> {
                        double m0 = reg_h5_data[b].excitation_mag_matrix(dof, h_idx, f_lo);
                        double p0 = reg_h5_data[b].excitation_phase_matrix(dof, h_idx, f_lo);
                        double m1 = reg_h5_data[b].excitation_mag_matrix(dof, h_idx, f_hi);
                        double p1 = reg_h5_data[b].excitation_phase_matrix(dof, h_idx, f_hi);
                        double mag   = m0 + f_frac * (m1 - m0);
                        double phase = p0 + f_frac * (p1 - p0);
                        return {mag, phase};
                    };

                    auto [mag_lo, phase_lo] = interp_freq_polar(h_lo);
                    double mag_final, phase_final;
                    if (h_lo == h_hi) {
                        mag_final   = mag_lo;
                        phase_final = phase_lo;
                    } else {
                        auto [mag_hi, phase_hi] = interp_freq_polar(h_hi);
                        mag_final   = mag_lo + h_frac * (mag_hi - mag_lo);
                        phase_final = phase_lo + h_frac * (phase_hi - phase_lo);
                    }
                    result.re[b](dof, ci) =  mag_final * std::cos(phase_final);
                    result.im[b](dof, ci) = -mag_final * std::sin(phase_final);
                } else {
                    auto interp_freq = [&](int h_idx) -> std::pair<double, double> {
                        double m0 = reg_h5_data[b].excitation_mag_matrix(dof, h_idx, f_lo);
                        double p0 = reg_h5_data[b].excitation_phase_matrix(dof, h_idx, f_lo);
                        double m1 = reg_h5_data[b].excitation_mag_matrix(dof, h_idx, f_hi);
                        double p1 = reg_h5_data[b].excitation_phase_matrix(dof, h_idx, f_hi);
                        double re0 = m0 * std::cos(p0), im0 = -m0 * std::sin(p0);
                        double re1 = m1 * std::cos(p1), im1 = -m1 * std::sin(p1);
                        double re  = re0 + f_frac * (re1 - re0);
                        double im  = im0 + f_frac * (im1 - im0);
                        return {re, im};
                    };

                    auto [re_lo, im_lo] = interp_freq(h_lo);
                    if (h_lo == h_hi) {
                        result.re[b](dof, ci) = re_lo;
                        result.im[b](dof, ci) = im_lo;
                    } else {
                        auto [re_hi, im_hi] = interp_freq(h_hi);
                        result.re[b](dof, ci) = re_lo + h_frac * (re_hi - re_lo);
                        result.im[b](dof, ci) = im_lo + h_frac * (im_hi - im_lo);
                    }
                }
            }
        }
    }

    return result;
}

}  // namespace seastack::hydro
