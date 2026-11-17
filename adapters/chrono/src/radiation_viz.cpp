#include <seastack/adapters/chrono/radiation_viz.h>

#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/hydro_data.h>

#include <chrono/physics/ChBody.h>

#include <Eigen/Geometry>

#include <cmath>
#include <complex>
#include <string>

namespace seastack::chrono {

namespace viz = seastack::viz;

std::vector<std::vector<double>> RadiationDampingFromRirf(const seastack::hydro::HydroData& data,
                                                          const std::vector<double>& omega) {
    const int nb = data.num_bodies();
    const int n = 6 * nb;
    const Eigen::VectorXd t = data.GetRIRFTimeVector();
    std::vector<Eigen::Index> idx;
    for (Eigen::Index s = 0; s < t.size(); ++s) {
        if (t[s] >= -1e-12) {
            idx.push_back(s);
        }
    }
    if (idx.size() < 2) {
        return {};
    }
    for (int b = 0; b < nb; ++b) {
        const auto& rirf = data.GetBodyInfos()[b].rirf_matrix;
        if (rirf.dimension(0) != 6 || rirf.dimension(1) != n || rirf.dimension(2) != t.size()) {
            return {};
        }
    }
    // Same scaling as HydroData::GetRIRFVal (K = rho * K_file), applied once.
    const double rho = data.GetSimulationInfo().rho;
    std::vector<std::vector<double>> damping(omega.size(), std::vector<double>(n * n, 0.0));
    for (size_t s = 0; s < idx.size(); ++s) {
        const double dt_prev = (s > 0) ? t[idx[s]] - t[idx[s - 1]] : 0.0;
        const double dt_next = (s + 1 < idx.size()) ? t[idx[s + 1]] - t[idx[s]] : 0.0;
        const double trapezoid = 0.5 * (dt_prev + dt_next) * rho;
        for (size_t k = 0; k < omega.size(); ++k) {
            const double weight = trapezoid * std::cos(omega[k] * t[idx[s]]);
            for (int b = 0; b < nb; ++b) {
                const auto& rirf = data.GetBodyInfos()[b].rirf_matrix;
                for (int j = 0; j < n; ++j) {
                    for (int i = 0; i < 6; ++i) {
                        damping[k][static_cast<size_t>(6 * b + i) * n + j] += weight * rirf(i, j, idx[s]);
                    }
                }
            }
        }
    }
    return damping;
}

viz::RadiationVizInputs MakeRadiationVizInputs(const HydroSystem& hydro) {
    const seastack::hydro::HydroData& data = hydro.GetHydroData();
    const auto& sim = data.GetSimulationInfo();
    const auto& chrono_bodies = hydro.GetBodies();
    const int nb = data.num_bodies();
    const int n = 6 * nb;

    viz::RadiationVizInputs in;
    in.rho = sim.rho;
    in.gravity = sim.g;
    in.depth = sim.water_depth;
    const Eigen::VectorXd w = data.GetBemFrequencies();
    in.omega.assign(w.data(), w.data() + w.size());
    for (int b = 0; b < nb; ++b) {
        viz::RadiationBodyInputs body;
        body.name = (b < static_cast<int>(chrono_bodies.size())) ? chrono_bodies[b]->GetName()
                                                                 : data.GetBodyInfos()[b].body_name;
        const Eigen::VectorXd cg = data.GetCGVector(b);
        if (cg.size() >= 2) {
            body.ref_x = cg[0];
            body.ref_y = cg[1];
        }
        body.disp_volume = data.GetDispVolVal(b);
        in.bodies.push_back(body);
    }
    if (in.omega.empty() || nb == 0) {
        return in;
    }

    if (data.HasFrequencyDomainRadiation()) {
        for (size_t k = 0; k < in.omega.size(); ++k) {
            const Eigen::MatrixXd b_mat = data.GetRadiationDampingAtFrequencyIndex(static_cast<int>(k));
            std::vector<double> row_major(static_cast<size_t>(n) * n);
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    row_major[static_cast<size_t>(i) * n + j] = b_mat(i, j);
                }
            }
            in.damping.push_back(std::move(row_major));
        }
        in.damping_source = "stored radiation damping B(w)";
    } else {
        in.damping = RadiationDampingFromRirf(data, in.omega);
        if (!in.damping.empty()) {
            in.damping_source = "cosine transform of the RIRF K(t)";
        }
    }

    const auto& waves = data.GetRegularWaveInfos();
    const Eigen::Index nd = sim.wave_directions.size();
    bool excitation_ok = nd > 0 && static_cast<int>(waves.size()) == nb;
    for (int b = 0; excitation_ok && b < nb; ++b) {
        const auto& mag = waves[b].excitation_mag_matrix;
        const auto& phase = waves[b].excitation_phase_matrix;
        excitation_ok = mag.dimension(0) == 6 && mag.dimension(1) == nd &&
                        mag.dimension(2) == static_cast<Eigen::Index>(in.omega.size()) &&
                        phase.dimension(0) == 6 && phase.dimension(1) == nd && phase.dimension(2) == mag.dimension(2);
    }
    if (excitation_ok) {
        in.headings.assign(sim.wave_directions.data(), sim.wave_directions.data() + nd);
        in.excitation.resize(in.omega.size() * static_cast<size_t>(nd) * n);
        for (size_t k = 0; k < in.omega.size(); ++k) {
            for (Eigen::Index d = 0; d < nd; ++d) {
                for (int b = 0; b < nb; ++b) {
                    for (int j = 0; j < 6; ++j) {
                        const Eigen::Index kk = static_cast<Eigen::Index>(k);
                        in.excitation[(k * nd + d) * n + 6 * b + j] = std::polar(
                            waves[b].excitation_mag_matrix(j, d, kk), waves[b].excitation_phase_matrix(j, d, kk));
                    }
                }
            }
        }
    }
    return in;
}

std::vector<viz::RadiationBodySample> CaptureRadiationSamples(const HydroSystem& hydro) {
    const auto& bodies = hydro.GetBodies();
    std::vector<viz::RadiationBodySample> samples(bodies.size());
    for (size_t b = 0; b < bodies.size(); ++b) {
        const ::chrono::ChBody& body = *bodies[b];
        const double yaw = body.GetRot().GetCardanAnglesXYZ().z();
        const Eigen::Matrix3d world_to_heading =
            Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Vector3d v = world_to_heading * body.GetPosDt().eigen();
        const Eigen::Vector3d omega = world_to_heading * body.GetAngVelParent().eigen();
        viz::RadiationBodySample& s = samples[b];
        s.x = body.GetPos().x();
        s.y = body.GetPos().y();
        s.yaw = yaw;
        for (int i = 0; i < 3; ++i) {
            s.velocity[i] = v[i];
            s.velocity[3 + i] = omega[i];
        }
    }
    return samples;
}

}  // namespace seastack::chrono
