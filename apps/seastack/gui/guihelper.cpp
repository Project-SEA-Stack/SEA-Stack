#include <seastack/config.h>
#include "guihelper.h"
#include <seastack/infra/logging.h>
#include <seastack/hydro/waves/wave_base.h>

#include "guihelper_impl.h"

using namespace seastack::viz;

std::shared_ptr<seastack::viz::UI> seastack::viz::CreateUI(bool visualizationOn) {
    if (visualizationOn) {
        return std::make_shared<seastack::viz::GUI>();
    } else {
        return std::make_shared<seastack::viz::UI>();
    }
}

// -----------------------------------------------------------------------------

UI::UI() : pSystem(nullptr) {}

void UI::Init(::chrono::ChSystem* system, const char* title) {
    pSystem = system;
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

// -----------------------------------------------------------------------------

GUI::GUI() {
#if defined(SEASTACK_HAVE_VSG)
    pImpl = std::make_shared<seastack::viz::GUIImplVSG>();
    simulationStarted = false;  // VSG viewer provides Start button
#else
    pImpl = std::make_shared<seastack::viz::GUIImpl>();
#endif
}

void GUI::Init(::chrono::ChSystem* system, const char* title) {
    UI::Init(system, title);
    pImpl->Init(*this, system, title);
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
