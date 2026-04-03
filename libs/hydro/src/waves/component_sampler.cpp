/*********************************************************************
 * @file  component_sampler.cpp
 * @brief Implementation of ComponentSampler.
 *********************************************************************/

#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/core/math_constants.h>
#include <seastack/infra/logging.h>
#include "wave_utilities.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace {

constexpr double kDegToRad = M_PI / 180.0;

constexpr double kDefaultOmegaMinFactor = 0.25;
constexpr double kDefaultOmegaMaxFactor = 4.0;

std::pair<double, double> FindPeakOmegaRange(
    const std::vector<seastack::hydro::SeaStatePartition>& partitions) {
    double omega_p_min = std::numeric_limits<double>::max();
    double omega_p_max = 0.0;
    for (const auto& p : partitions) {
        if (p.spectrum.Tp > 0.0) {
            double wp = 2.0 * M_PI / p.spectrum.Tp;
            omega_p_min = std::min(omega_p_min, wp);
            omega_p_max = std::max(omega_p_max, wp);
        }
    }
    if (omega_p_max <= 0.0) return {0.0, 0.0};
    return {omega_p_min, omega_p_max};
}

}  // namespace

namespace seastack::hydro {

std::vector<WaveComponent> ComponentSampler::Build(const SeaStateDefinition& def) {
    // Eta-import path takes precedence when an eta file is specified.
    if (!def.eta_file_path.empty()) {
        double f_min = (def.omega_min > 0.0) ? def.omega_min / (2.0 * M_PI) : 0.001;
        double f_max = (def.omega_max > 0.0) ? def.omega_max / (2.0 * M_PI) : 1.0;
        int nf = (def.n_omega > 0) ? def.n_omega : 1000;
        double dir = def.direction_deg * kDegToRad;
        return BuildFromEtaFile(def.eta_file_path, def.depth, def.g,
                                nf, f_min, f_max, dir);
    }

    std::string type = def.type;
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);

    if (type == "regular") {
        if (def.omega <= 0.0) {
            throw std::invalid_argument("ComponentSampler::Build: regular wave requires omega > 0");
        }
        if (def.amplitude <= 0.0) {
            throw std::invalid_argument("ComponentSampler::Build: regular wave requires amplitude > 0");
        }
        WaveComponent c;
        c.omega     = def.omega;
        c.k         = ComputeWaveNumber(def.omega, def.depth, def.g);
        c.direction = def.direction_deg * kDegToRad;
        c.amplitude = def.amplitude;
        c.phase     = def.phase_rad;
        return {c};
    }

    if (type == "irregular") {
        if (def.partitions.empty()) {
            throw std::invalid_argument(
                "ComponentSampler::Build: irregular sea state requires at least one partition");
        }

        // Resolve frequency limits.
        double omega_min = def.omega_min;
        double omega_max = def.omega_max;
        if (omega_min <= 0.0 || omega_max <= 0.0) {
            auto [omega_p_min, omega_p_max] = FindPeakOmegaRange(def.partitions);
            if (omega_p_max <= 0.0) {
                throw std::invalid_argument(
                    "ComponentSampler::Build: cannot determine frequency range (Tp not set)");
            }
            if (omega_min <= 0.0) omega_min = kDefaultOmegaMinFactor * omega_p_min;
            if (omega_max <= 0.0) omega_max = kDefaultOmegaMaxFactor * omega_p_max;
        }
        if (omega_min >= omega_max) {
            throw std::invalid_argument(
                "ComponentSampler::Build: omega_min must be < omega_max");
        }

        std::mt19937 rng(static_cast<unsigned int>(def.seed));

        std::vector<WaveComponent> all_components;
        for (const auto& partition : def.partitions) {
            auto partition_components = SamplePartition(
                partition, def.n_omega, def.n_theta,
                omega_min, omega_max, def.depth, def.g, rng);
            all_components.insert(all_components.end(),
                                  std::make_move_iterator(partition_components.begin()),
                                  std::make_move_iterator(partition_components.end()));
        }

        PruneComponents(all_components);
        return all_components;
    }

    throw std::invalid_argument("ComponentSampler::Build: unsupported sea state type '" + def.type + "'");
}

std::vector<WaveComponent> ComponentSampler::SamplePartition(
    const SeaStatePartition& partition,
    int n_omega,
    int n_theta,
    double omega_min,
    double omega_max,
    double depth,
    double g,
    std::mt19937& rng) {

    const auto& spec = partition.spectrum;
    const auto& spread = partition.spreading;

    if (spec.Hs <= 0.0 || spec.Tp <= 0.0) {
        throw std::invalid_argument("ComponentSampler: partition requires Hs > 0 and Tp > 0");
    }

    // --- Frequency discretization ---
    const double f_min = omega_min / (2.0 * M_PI);
    const double f_max = omega_max / (2.0 * M_PI);
    Eigen::VectorXd freqs_hz = Eigen::VectorXd::LinSpaced(n_omega, f_min, f_max);

    // Compute 1D spectral densities S(f) [m^2/Hz].
    Eigen::VectorXd S_f;
    std::string spec_type = spec.type;
    std::transform(spec_type.begin(), spec_type.end(), spec_type.begin(), ::tolower);
    if (spec_type == "jonswap") {
        S_f = JONSWAPSpectrumHz(freqs_hz, spec.Hs, spec.Tp, spec.gamma);
    } else if (spec_type == "pierson_moskowitz" || spec_type == "pm") {
        S_f = PiersonMoskowitzSpectrumHz(freqs_hz, spec.Hs, spec.Tp);
    } else {
        throw std::invalid_argument("ComponentSampler: unsupported spectrum type '" + spec.type + "'");
    }

    const Eigen::VectorXd df = GetWidthArray(freqs_hz);

    const Eigen::VectorXd omegas = 2.0 * M_PI * freqs_hz;
    const Eigen::VectorXd wavenumbers = ComputeWaveNumbers(omegas, depth, g);

    // --- Directional discretization ---
    std::string spread_type = spread.type;
    std::transform(spread_type.begin(), spread_type.end(), spread_type.begin(), ::tolower);

    if (spread_type != "none" && spread_type != "cos2s" && !spread_type.empty()) {
        throw std::invalid_argument(
            "ComponentSampler: unsupported spreading type '" + spread.type +
            "' (valid: \"none\", \"cos2s\")");
    }
    if (spread_type == "cos2s" && spread.s <= 0.0) {
        throw std::invalid_argument(
            "ComponentSampler: cos2s spreading requires s > 0 (got s=" +
            std::to_string(spread.s) + ")");
    }

    const bool is_long_crested = (spread_type == "none" || spread_type.empty() || n_theta <= 1);

    const double theta_mean = spread.mean_direction_deg * kDegToRad;

    std::vector<double> thetas;
    std::vector<double> d_thetas;
    std::vector<double> D_vals;

    if (is_long_crested) {
        // Long-crested: single direction.
        thetas.push_back(theta_mean);
        d_thetas.push_back(1.0);  // integral of D over all theta = 1, one bin captures everything
        D_vals.push_back(1.0);
    } else {
        // Directional bins centered on theta_mean, spanning +-pi.
        double theta_min = theta_mean - M_PI;
        double d_theta = (2.0 * M_PI) / static_cast<double>(n_theta);
        thetas.resize(n_theta);
        d_thetas.resize(n_theta, d_theta);
        D_vals.resize(n_theta);

        for (int j = 0; j < n_theta; ++j) {
            thetas[j] = theta_min + (static_cast<double>(j) + 0.5) * d_theta;
            D_vals[j] = Cos2sSpreading(thetas[j], theta_mean, spread.s);
        }
    }

    // --- Build components ---
    std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * M_PI);

    std::vector<WaveComponent> components;
    components.reserve(static_cast<size_t>(n_omega) * thetas.size());

    for (int i = 0; i < n_omega; ++i) {
        for (size_t j = 0; j < thetas.size(); ++j) {
            const double a = std::sqrt(2.0 * S_f[i] * df[i] * D_vals[j] * d_thetas[j]);
            const double phase = phase_dist(rng);
            if (a <= 0.0) continue;

            WaveComponent c;
            c.omega     = omegas[i];
            c.k         = wavenumbers[i];
            c.direction = thetas[j];
            c.amplitude = a;
            c.phase     = phase;
            components.push_back(c);
        }
    }

    return components;
}

std::vector<WaveComponent> ComponentSampler::BuildFromEtaFile(
    const std::string& path,
    double depth,
    double g,
    int nf,
    double f_min_hz,
    double f_max_hz,
    double direction_rad) {

    // 1. Read the time:eta file.
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error(
            "ComponentSampler::BuildFromEtaFile: cannot open '" + path + "'");
    }

    std::vector<double> times;
    std::vector<double> etas;
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double t, eta;
        char delimiter;
        if (!(ss >> t >> delimiter >> eta) || delimiter != ':') {
            throw std::runtime_error(
                "ComponentSampler::BuildFromEtaFile: parse error on line: " + line);
        }
        times.push_back(t);
        etas.push_back(eta);
    }

    const size_t N = etas.size();
    if (N < 2) {
        throw std::runtime_error(
            "ComponentSampler::BuildFromEtaFile: eta file has fewer than 2 samples");
    }

    const double dt = (times.back() - times.front()) / static_cast<double>(N - 1);
    const double t0 = times.front();

    // 2. Build the frequency grid from natural Fourier frequencies.
    //
    // The DFT amplitude formula  A = 2*|F|/N  is exact only when evaluated
    // at the orthogonal Fourier frequencies  f_k = k / (N * dt).  Using an
    // arbitrary (finer) grid causes spectral leakage between adjacent bins,
    // inflating extracted amplitudes and corrupting the reconstruction.
    const double T_window = static_cast<double>(N) * dt;
    const double df_natural = 1.0 / T_window;

    int k_min = std::max(1, static_cast<int>(std::ceil(f_min_hz / df_natural)));
    int k_max = static_cast<int>(std::floor(f_max_hz / df_natural));
    if (k_max < k_min) {
        throw std::runtime_error(
            "ComponentSampler::BuildFromEtaFile: no Fourier bins in ["
            + std::to_string(f_min_hz) + ", " + std::to_string(f_max_hz)
            + "] Hz (resolution " + std::to_string(df_natural) + " Hz)");
    }

    int nf_full = k_max - k_min + 1;
    int stride = 1;
    if (nf > 0 && nf_full > nf) {
        stride = std::max(1, (nf_full + nf - 1) / nf);
    }

    Eigen::VectorXd freqs_hz(0);
    {
        std::vector<double> freq_vec;
        freq_vec.reserve(static_cast<size_t>(nf_full / stride + 1));
        for (int k = k_min; k <= k_max; k += stride) {
            freq_vec.push_back(static_cast<double>(k) * df_natural);
        }
        freqs_hz.resize(static_cast<Eigen::Index>(freq_vec.size()));
        for (Eigen::Index i = 0; i < freqs_hz.size(); ++i) {
            freqs_hz[i] = freq_vec[static_cast<size_t>(i)];
        }
    }

    const int nf_actual = static_cast<int>(freqs_hz.size());
    Eigen::VectorXd omegas = 2.0 * M_PI * freqs_hz;
    Eigen::VectorXd wavenumbers = ComputeWaveNumbers(omegas, depth, g);

    // 3. DFT at the (orthogonal) Fourier frequencies.
    std::vector<WaveComponent> components;
    components.reserve(static_cast<size_t>(nf_actual));

    for (int i = 0; i < nf_actual; ++i) {
        const double omega = omegas[i];
        double re = 0.0, im = 0.0;
        for (size_t j = 0; j < N; ++j) {
            const double angle = omega * (t0 + static_cast<double>(j) * dt);
            re += etas[j] * std::cos(angle);
            im += etas[j] * std::sin(angle);
        }
        const double amplitude = 2.0 * std::sqrt(re * re + im * im)
                               / static_cast<double>(N);
        const double phase = std::atan2(im, re);

        if (amplitude <= 0.0) continue;

        WaveComponent c;
        c.omega     = omega;
        c.k         = wavenumbers[i];
        c.direction = direction_rad;
        c.amplitude = amplitude;
        c.phase     = phase;
        components.push_back(c);
    }

    // 4. Parseval energy check: sum(A^2/2) should approximate var(eta) for the
    // retained Fourier bins only. When nf caps force stride>1, most in-band energy
    // is omitted from `components` while var_eta is the full-record variance — the
    // ratio is not meaningful, so skip the warning.
    if (stride == 1) {
        double mean_eta = 0.0;
        for (size_t j = 0; j < N; ++j) mean_eta += etas[j];
        mean_eta /= static_cast<double>(N);

        double var_eta = 0.0;
        for (size_t j = 0; j < N; ++j) {
            double d = etas[j] - mean_eta;
            var_eta += d * d;
        }
        var_eta /= static_cast<double>(N);

        double spectral_energy = 0.0;
        for (const auto& c : components) {
            spectral_energy += 0.5 * c.amplitude * c.amplitude;
        }

        if (var_eta > 0.0) {
            double ratio = spectral_energy / var_eta;
            if (ratio < 0.5 || ratio > 2.0) {
                LOG_WARNING("BuildFromEtaFile: Parseval check -- spectral energy / signal variance = "
                    << ratio << " (expected ~1.0). "
                    << "spectral_energy=" << spectral_energy
                    << ", var_eta=" << var_eta
                    << ", nf_actual=" << nf_actual
                    << ", N=" << N << ", dt=" << dt);
            }
        }
    }

    PruneComponents(components);
    return components;
}

void ComponentSampler::PruneComponents(std::vector<WaveComponent>& components,
                                       double threshold_fraction) {
    if (components.empty()) return;

    const auto it = std::max_element(
        components.cbegin(), components.cend(),
        [](const WaveComponent& a, const WaveComponent& b) { return a.amplitude < b.amplitude; });
    const double max_amp = it->amplitude;

    if (max_amp <= 0.0) return;

    const double threshold = threshold_fraction * max_amp;
    components.erase(
        std::remove_if(components.begin(), components.end(),
                       [threshold](const WaveComponent& c) { return c.amplitude < threshold; }),
        components.end());
}

}  // namespace seastack::hydro
