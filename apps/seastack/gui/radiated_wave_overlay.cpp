#include "radiated_wave_overlay.h"

#include <seastack/adapters/chrono/radiation_viz.h>
#include <seastack/infra/logging.h>

#include "vsg_gui_component.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>

namespace seastack::viz {

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Change in r_max or omega_cap that triggers a rebuild [-].
constexpr double kReconfigureFraction = 0.1;

bool Differs(double a, double b) { return std::abs(a - b) > kReconfigureFraction * std::max(std::abs(b), 1e-9); }

}  // namespace

void RadiatedWaveOverlay::SetHydro(const seastack::chrono::HydroSystem* hydro) {
    hydro_ = hydro;
    inputs_built_ = false;
    history_.Clear();
    configured_method_ = RadiatedWaveMethod::kOff;
}

void RadiatedWaveOverlay::RecordStep(double t) {
    if (hydro_) {
        history_.Add(t, seastack::chrono::CaptureRadiationSamples(*hydro_));
    }
}

const RadiatedWaveSynthesizer* RadiatedWaveOverlay::Update(ViewerSettings* settings, const SceneDescription& scene,
                                                           double t, bool advance) {
    if (!settings) {
        return nullptr;
    }
    if (!inputs_built_) {
        inputs_built_ = true;
        std::string failure;
        if (!hydro_) {
            failure = "needs the live hydro model (not available when replaying a recording)";
        } else {
            try {
                inputs_ = seastack::chrono::MakeRadiationVizInputs(*hydro_);
            } catch (const std::exception& e) {
                inputs_ = RadiationVizInputs{};
                failure = std::string("could not read the BEM data (") + e.what() + ")";
            }
        }
        for (int i = 0; i < kNumRadiatedWaveMethods; ++i) {
            const auto method = static_cast<RadiatedWaveMethod>(i);
            availability_[i] = CheckAvailability(method, inputs_);
            if (!failure.empty() && method != RadiatedWaveMethod::kOff) {
                availability_[i] = {false, failure};
            }
        }
        if (hydro_ && !inputs_.damping_source.empty()) {
            seastack::infra::cli::LogInfo("[RadiatedWaves] Radiation damping from the " + inputs_.damping_source);
        }
    }
    settings->radiation_availability = availability_;

    const RadiatedWaveMethod requested = settings->radiation_method;
    const RadiatedWaveMethod method = availability_[static_cast<int>(requested)].available
                                          ? ResolveMethod(requested, inputs_)
                                          : RadiatedWaveMethod::kOff;
    if (method != RadiatedWaveMethod::kOff) {
        const double width = settings->grid_width > 0.0 ? settings->grid_width : 100.0;
        const double length = settings->grid_length > 0.0 ? settings->grid_length : 100.0;
        const double r_max = std::hypot(width, length);
        const double cell = settings->water_cell_m > 0.0 ? settings->water_cell_m : std::max(width, length) / 64.0;
        const double omega_cap = DispersionFrequency(2.0 * kPi / (4.0 * cell), inputs_.gravity, inputs_.depth);
        if (method != configured_method_ || Differs(r_max, configured_r_max_) ||
            Differs(omega_cap, configured_omega_cap_)) {
            Configure(method, r_max, omega_cap, scene);
            advance = true;
        }
    } else if (configured_method_ != RadiatedWaveMethod::kOff) {
        configured_method_ = RadiatedWaveMethod::kOff;
        synth_ = RadiatedWaveSynthesizer{};
    }

    settings->radiation_active = synth_.IsConfigured() ? configured_method_ : RadiatedWaveMethod::kOff;
    settings->radiation_label = model_.label;
    settings->radiation_approximate = model_.approximate;
    if (settings->radiation_active == RadiatedWaveMethod::kOff) {
        return nullptr;
    }
    if (advance) {
        synth_.Update(history_, t);
    }
    return synth_.HasField() ? &synth_ : nullptr;
}

void RadiatedWaveOverlay::Configure(RadiatedWaveMethod method, double r_max, double omega_cap,
                                    const SceneDescription& scene) {
    model_ = BuildRadiatedWaveModel(inputs_, method);
    synth_.Configure(model_, r_max, omega_cap, history_.Dt());
    configured_method_ = method;
    configured_r_max_ = r_max;
    configured_omega_cap_ = omega_cap;

    for (size_t s = 0; s < synth_.NumSources(); ++s) {
        for (const BodyDescription& body : scene.bodies) {
            if (body.name == synth_.SourceName(s) && body.local_bounds.IsValid()) {
                const Vec3 size = body.local_bounds.Size();
                synth_.SetSourceRadius(s, 0.5 * std::max(size[0], size[1]));
            }
        }
    }

    char summary[256];
    std::snprintf(summary, sizeof(summary),
                  "[RadiatedWaves] %s: %zu bodies, w <= %.2f rad/s (%zu frequencies), r <= %.0f m, "
                  "history window %.0f s",
                  RadiatedWaveMethodName(method), synth_.NumSources(), synth_.OmegaMax(), synth_.NumFrequencies(),
                  synth_.MaxRadius(), synth_.Window());
    seastack::infra::cli::LogInfo(summary);
    seastack::infra::cli::LogInfo("[RadiatedWaves] " + model_.label);
    for (const std::string& note : model_.notes) {
        seastack::infra::cli::LogInfo("[RadiatedWaves]   " + note);
    }
}

}  // namespace seastack::viz
