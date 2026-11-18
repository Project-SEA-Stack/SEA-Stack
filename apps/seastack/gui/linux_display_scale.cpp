// =============================================================================
// Linux desktop UI scale
// =============================================================================
// X11 sizes a window in pixels. The Mac viewer can leave the window in points
// and scale fonts from the Retina framebuffer; here the desktop's Xft.dpi
// (px/in, 96 at 100 %) is that factor for both the window and the font.
// =============================================================================

#include "vsg_theme.h"

#include <X11/Xlib.h>

#include <cstdlib>

namespace seastack::viz {

float LinuxDisplayScale() {
    Display* display = XOpenDisplay(nullptr);
    if (display == nullptr) {
        return 1.0f;
    }
    const char* text = XGetDefault(display, "Xft", "dpi");
    float dpi = 0.0f;
    if (text != nullptr && text[0] != '\0') {
        char* end = nullptr;
        dpi = std::strtof(text, &end);
        if (end == text) {
            dpi = 0.0f;
        }
    }
    XCloseDisplay(display);
    return UiScaleFromDpi(dpi);
}

}  // namespace seastack::viz
