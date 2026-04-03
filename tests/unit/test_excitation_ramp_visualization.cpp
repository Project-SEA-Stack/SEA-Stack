#include <seastack/hydro/waves/eta_table_wave_field.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include "test_macros.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    TestResults test_results;
    constexpr double tol = 1e-12;

    // LinearDirectionalWaveField: no ramp -> unity factor (matches default physics).
    {
        seastack::hydro::WaveComponent c;
        c.omega = 1.0;
        c.k = 0.01;
        c.amplitude = 0.5;
        c.phase = 0.0;
        seastack::hydro::LinearDirectionalWaveField field({c}, 0.0);

        TEST_NEAR(field.GetExcitationRampForVisualization(-1.0), 1.0, tol, "no ramp t<0");
        TEST_NEAR(field.GetExcitationRampForVisualization(0.0), 1.0, tol, "no ramp t=0");
        TEST_NEAR(field.GetExcitationRampForVisualization(100.0), 1.0, tol, "no ramp large t");
    }

    // Cosine ramp matches ExcitationComponent frequency-domain scaling.
    {
        seastack::hydro::WaveComponent c;
        c.omega = 1.0;
        c.k = 0.01;
        c.amplitude = 0.5;
        c.phase = 0.0;
        seastack::hydro::LinearDirectionalWaveField field({c}, 0.0);
        const double T = 10.0;
        field.SetRampDuration(T);

        TEST_NEAR(field.GetExcitationRampForVisualization(0.0), 0.0, tol, "ramp start");
        const double t_mid = 0.5 * T;
        const double expected_mid = 0.5 * (1.0 - std::cos(M_PI * t_mid / T));
        TEST_NEAR(field.GetExcitationRampForVisualization(t_mid), expected_mid, tol, "ramp mid");
        TEST_NEAR(field.GetExcitationRampForVisualization(T), 1.0, tol, "ramp at T");
        TEST_NEAR(field.GetExcitationRampForVisualization(T + 1.0), 1.0, tol, "after ramp");
    }

    // EtaTableWaveField: linear ramp for IRF consistency.
    {
        std::vector<double> times = {0.0, 1.0, 2.0};
        std::vector<double> eta = {0.0, 0.0, 0.0};
        seastack::hydro::EtaTableWaveField field(std::move(times), std::move(eta), 0.0);
        field.SetRampDuration(8.0);

        TEST_NEAR(field.GetExcitationRampForVisualization(0.0), 0.0, tol, "eta table ramp 0");
        TEST_NEAR(field.GetExcitationRampForVisualization(4.0), 0.5, tol, "eta table ramp half");
        TEST_NEAR(field.GetExcitationRampForVisualization(8.0), 1.0, tol, "eta table ramp end");
        TEST_NEAR(field.GetExcitationRampForVisualization(20.0), 1.0, tol, "eta table after");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
