#pragma once

#include <seastack/viz_core/radiated_wave_synthesis.h>
#include <seastack/viz_core/radiated_waves.h>

#include <vector>

namespace seastack::hydro {
class HydroData;
}

namespace seastack::chrono {

class HydroSystem;

/// Coupled radiation damping B(w_k) [SI], row-major 6N x 6N per frequency,
/// from the RIRF: B(w) = int_0^T K(t) cos(w t) dt (trapezoid over the t >= 0
/// samples). Empty if the RIRF is missing or not 6 x 6N per body.
std::vector<std::vector<double>> RadiationDampingFromRirf(const seastack::hydro::HydroData& data,
                                                          const std::vector<double>& omega);

/// BEM data of `hydro` for the radiated-wave overlay (seastack::viz).
///  - Bodies in hydro order, named after their Chrono bodies; reference point
///    and displaced volume from the BEM file.
///  - B(w) from the stored radiation damping if present, else from the RIRF
///    (RadiationDampingFromRirf; the inverse of K(t) = (2/pi) int B cos(w t) dw).
///  - Excitation X = |X| e^{i phase} (e^{+i w t}) for every BEM heading, when
///    it is on the same frequency grid.
seastack::viz::RadiationVizInputs MakeRadiationVizInputs(const HydroSystem& hydro);

/// State of each hydro body now, for the overlay's velocity history: centre
/// of mass x, y, yaw, and the velocities the radiation force uses (centre of
/// mass linear velocity and angular velocity, rotated into the heading frame).
std::vector<seastack::viz::RadiationBodySample> CaptureRadiationSamples(const HydroSystem& hydro);

}  // namespace seastack::chrono
