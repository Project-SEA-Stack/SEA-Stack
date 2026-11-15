#include <seastack/config.h>
#include "guihelper.h"
#include <seastack/infra/logging.h>
#include <seastack/hydro/waves/wave_base.h>

#include "guihelper_impl.h"

#include <seastack/adapters/chrono/visualization_recording.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

using namespace seastack::viz;

std::shared_ptr<seastack::viz::UI> seastack::viz::CreateUI(bool visualizationOn, ViewerBackend backend) {
    if (visualizationOn) {
        return std::make_shared<seastack::viz::GUI>(backend);
    } else {
        return std::make_shared<seastack::viz::UI>();
    }
}

bool seastack::viz::ParseViewerBackend(const std::string& name, ViewerBackend* backend) {
    if (name == "chrono") {
        *backend = ViewerBackend::kChrono;
        return true;
    }
    if (name == "seastack") {
        *backend = ViewerBackend::kSeastack;
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------

UI::UI() : pSystem(nullptr) {}

void UI::SetViewerOptions(const ViewerOptions& /*options*/) {
    // Default (headless) UI has no viewer.
}

void UI::Init(::chrono::ChSystem* system, const char* title) {
    pSystem = system;
}

void UI::InitReplay(std::unique_ptr<seastack::chrono::ReplayRecording> /*recording*/, const char* /*title*/) {
    throw std::runtime_error("Replay needs the GUI (it cannot run with --nogui).");
}

void UI::SetCamera(double x, double y, double z, double dirx, double diry, double dirz) {}

bool UI::IsRunning(double timestep) {
    return true;
}

void UI::SetWaveModel(std::shared_ptr<seastack::hydro::WaveBase> /*wave*/) {
    // Default (headless) UI does not render waves.
}

void UI::SetWaterGridExtent(double /*width*/, double /*length*/,
                            double /*center_x*/, double /*center_y*/) {
    // Default (headless) UI does not render water surface.
}

void UI::SetMooringLineProvider(MooringVizProvider /*provider*/) {
    // Default (headless) UI does not render mooring lines.
}

void UI::SetMooringVisualizationRadii(double /*line_radius_m*/,
                                      double /*endpoint_radius_m*/,
                                      double /*node_marker_radius_m*/) {
    // Default (headless) UI does not render mooring lines.
}

void UI::SetRuntimeControls(seastack::chrono::RuntimeControls* /*controls*/,
                            const std::string& /*schedule_path*/, double /*time_step*/) {
    // Default (headless) UI has no parameter panel.
}

void UI::SetHydroForces(seastack::chrono::HydroSystem* /*hydro*/) {
    // Default (headless) UI does not draw forces.
}

// -----------------------------------------------------------------------------

GUI::GUI(ViewerBackend backend) : backend_(backend) {
#if defined(SEASTACK_HAVE_VSG)
    if (backend == ViewerBackend::kSeastack) {
        pImpl = std::make_shared<seastack::viz::GUIImplSeastack>();
    } else {
        pImpl = std::make_shared<seastack::viz::GUIImplVSG>();
    }
    simulationStarted = false;  // VSG viewer provides Start button
#else
    (void)backend;
    pImpl = std::make_shared<seastack::viz::GUIImpl>();
#endif
}

void GUI::SetViewerOptions(const ViewerOptions& options) {
    viewer_options_ = options;
    pImpl->SetViewerOptions(options);
}

void GUI::Init(::chrono::ChSystem* system, const char* title) {
    UI::Init(system, title);
#if defined(SEASTACK_HAVE_VSG)
    // The SEA-Stack viewer draws only what VizExtractor exports; anything else
    // would silently vanish, so use the Chrono viewer for such models.
    if (backend_ == ViewerBackend::kSeastack && system) {
        const std::vector<std::string> missing = seastack::chrono::FindUndrawableItems(*system);
        if (!missing.empty()) {
            std::string message = "SEA-Stack viewer cannot draw this model; using the Chrono viewer. Not supported:";
            constexpr size_t kMaxListed = 5;
            for (size_t i = 0; i < std::min(missing.size(), kMaxListed); ++i) {
                message += "\n  - " + missing[i];
            }
            if (missing.size() > kMaxListed) {
                message += "\n  - ... and " + std::to_string(missing.size() - kMaxListed) + " more";
            }
            seastack::infra::cli::LogWarning(message);
            backend_ = ViewerBackend::kChrono;
            pImpl = std::make_shared<seastack::viz::GUIImplVSG>();
            pImpl->SetViewerOptions(viewer_options_);
        }
    }
#endif
    pImpl->Init(*this, system, title);
}

void GUI::InitReplay(std::unique_ptr<seastack::chrono::ReplayRecording> recording, const char* title) {
    UI::Init(nullptr, title);
#if defined(SEASTACK_HAVE_VSG)
    if (backend_ != ViewerBackend::kSeastack) {
        backend_ = ViewerBackend::kSeastack;  // the Chrono viewer needs a ChSystem
        pImpl = std::make_shared<seastack::viz::GUIImplSeastack>();
        pImpl->SetViewerOptions(viewer_options_);
    }
    pImpl->InitReplay(*this, std::move(recording), title);
#else
    (void)recording;
    throw std::runtime_error("Replay needs a build with the VSG viewer (SEASTACK_HAVE_VSG).");
#endif
}

void GUI::SetCamera(double x, double y, double z, double dirx, double diry, double dirz) {
    pImpl->SetCamera(x, y, z, dirx, diry, dirz);
}

bool GUI::IsRunning(double timestep) {
    return pImpl->IsRunning(timestep);
}

void GUI::SetWaveModel(std::shared_ptr<seastack::hydro::WaveBase> wave) {
    pImpl->SetWaveModel(wave);
}

void GUI::SetWaterGridExtent(double width, double length, double center_x, double center_y) {
    pImpl->SetWaterGridExtent(width, length, center_x, center_y);
}

void GUI::SetMooringLineProvider(MooringVizProvider provider) {
    pImpl->SetMooringLineProvider(std::move(provider));
}

void GUI::SetMooringVisualizationRadii(double line_radius_m,
                                      double endpoint_radius_m,
                                      double node_marker_radius_m) {
    pImpl->SetMooringVisualizationRadii(line_radius_m, endpoint_radius_m, node_marker_radius_m);
}

void GUI::SetRuntimeControls(seastack::chrono::RuntimeControls* controls,
                             const std::string& schedule_path, double time_step) {
    pImpl->SetRuntimeControls(controls, schedule_path, time_step);
}

void GUI::SetHydroForces(seastack::chrono::HydroSystem* hydro) {
    pImpl->SetHydroForces(hydro);
}
