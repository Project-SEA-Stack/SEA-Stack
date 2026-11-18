/*********************************************************************
 * @file  test_irf_layer_release.cpp
 * @brief IRF excitation after many live period changes holds only the
 *        component sets its kernel can still reach.
 *
 * RM3 BEM data (2 bodies), kernel truncated at 20 s, a 30-component sea.
 * Ten alternating period changes, 50 s apart, each crossfading over 20 s.
 * Pass criteria:
 *   R1  During the run the cached sets never exceed three (the outgoing,
 *       the incoming and one still in the kernel window); at the end, two
 *       sets at most, while the wave holds all eleven.
 *   R2  The force stays finite and non-zero throughout.
 * Force equivalence with the data kept is checked across binaries in
 * tests/regression/run_seastack/test_runtime_sea_state.py.
 *********************************************************************/

#include <seastack/hydro/force_components/excitation_irf_component.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro_io/h5_reader.h>

#include "test_macros.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using seastack::hydro::BodyForces;
using seastack::hydro::ExcitationIrfComponent;
using seastack::hydro::LinearDirectionalWaveField;
using seastack::hydro::SystemState;
using seastack::hydro::WaveComponent;

namespace {

constexpr int kComponents = 30;

std::vector<WaveComponent> Sea(double omega_scale) {
    std::vector<WaveComponent> sea;
    for (int i = 0; i < kComponents; ++i) {
        WaveComponent c;
        c.omega = omega_scale * (0.3 + 0.04 * i);
        c.k = c.omega * c.omega / 9.81;
        c.amplitude = 0.1;
        c.phase = 0.7 * i;
        c.direction = 0.0;
        sea.push_back(c);
    }
    return sea;
}

}  // namespace

int main() {
    TestResults test_results;
    std::cout << "test_irf_layer_release\n";

    const std::string rm3 = std::string(SEASTACK_SOURCE_DIR) + "/data/demos/run_seastack/rm3/assets/hydroData/rm3.h5";
    seastack::hydro_io::H5FileInfo reader(rm3, 2);
    const seastack::hydro::HydroData data = reader.ReadH5Data();

    auto field = std::make_shared<LinearDirectionalWaveField>(Sea(1.0), 0.0);
    ExcitationIrfComponent irf(data, field, 2, 20.0, 0.0);

    SystemState state;
    state.bodies.resize(2);
    Eigen::Index max_cached = 0;
    bool finite = true;
    double last_norm = 0.0;
    int changes = 0;
    for (int step = 0; step <= 1200; ++step) {
        const double t = 0.5 * step;
        if (step > 0 && step % 100 == 0 && changes < 10) {
            ++changes;
            field->CrossfadeTo(Sea(changes % 2 ? 1.2 : 1.0), t, 20.0);
        }
        BodyForces forces(2);
        for (auto& f : forces) {
            f.force.setZero();
            f.moment.setZero();
        }
        irf.Compute(state, t, forces);
        last_norm = forces[0].force.norm() + forces[1].force.norm();
        finite = finite && std::isfinite(last_norm);
        if (changes > 0) max_cached = std::max(max_cached, irf.CachedComponentCount());
    }

    TEST_ASSERT(changes == 10 && field->GetComponents().size() == 11u * kComponents,
                "the wave keeps all 11 sets (" + std::to_string(field->GetComponents().size()) + " components)");
    TEST_ASSERT(max_cached <= 3 * kComponents,
                "at most 3 sets cached during the run (" + std::to_string(max_cached) + " components)");
    TEST_ASSERT(irf.CachedComponentCount() <= 2 * kComponents,
                "at most 2 sets cached at the end (" + std::to_string(irf.CachedComponentCount()) + " components)");
    TEST_ASSERT(finite && last_norm > 0.0, "force finite and non-zero");

    test_results.Summary();
    return test_results.failed == 0 ? 0 : 1;
}
