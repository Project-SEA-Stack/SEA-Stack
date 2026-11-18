/*********************************************************************
 * @file  sea_state_change.h
 * @brief Checks for live wave period and direction changes.
 *
 * A live change crossfades to a new component set with the same seed
 * (LinearDirectionalWaveField::CrossfadeTo). It is refused
 * when the new sea needs data the model does not have; at startup the same
 * situations only warn, but a live change is a choice the user can undo, so
 * the GUI says why and keeps the current sea:
 *   - more than `max_energy_outside` of the wave energy (sum of A_i^2) at
 *     frequencies outside the BEM range (a period change), or at directions
 *     outside the BEM headings (a direction change);
 *   - a period change with constant-coefficient radiation, whose A(w) and
 *     B(w) are fixed at the starting wave frequency (A(w) is in the Chrono
 *     added-mass load);
 *   - a direction change with IRF excitation whose H5 file has one kernel
 *     for several BEM headings.
 *
 * Headings are covered from the first to the last BEM heading, and across
 * the wrap from the last to the first + 2 pi only if that gap is no wider
 * than the largest step between headings (data around the full circle).
 *********************************************************************/

#ifndef SEASTACK_HYDRO_WAVES_SEA_STATE_CHANGE_H
#define SEASTACK_HYDRO_WAVES_SEA_STATE_CHANGE_H

#include <seastack/hydro/waves/wave_component.h>

#include <Eigen/Core>
#include <string>
#include <vector>

namespace seastack::hydro {

/// What the model's BEM data and settings allow for a live change.
struct SeaStateChangeLimits {
    Eigen::VectorXd bem_frequencies;  ///< Excitation frequencies [rad/s], ascending; empty = no check
    Eigen::VectorXd bem_headings;     ///< BEM headings [rad], ascending; empty = one heading at 0
    bool constant_coefficient_radiation = false;  ///< Radiation uses A(w), B(w) of the start frequency
    bool heading_kernels = true;  ///< false: IRF excitation with one kernel for several headings
    double max_energy_outside = 0.01;  ///< Largest share of wave energy outside the BEM data [-]
};

/// What the user changed. The component frequencies alone cannot tell: with a
/// fixed frequency band (frequency_min/max) a period change keeps them and
/// changes only the amplitudes, or changes how many components survive pruning.
enum class SeaStateChangeKind {
    kPeriod,     ///< Peak period Tp: radiation and BEM frequency-range checks
    kDirection,  ///< Mean direction: heading-coverage and IRF-kernel checks
};

/// Why a change of `kind` to the component set `proposed` is refused, or an
/// empty string if it is allowed. `proposed` may have any number of components.
std::string CheckSeaStateChange(SeaStateChangeKind kind, const std::vector<WaveComponent>& proposed,
                                const SeaStateChangeLimits& limits);

/// Why the change from `current` to `proposed` (matched one to one) is
/// refused, or an empty string if it is allowed. Infers the kind from the
/// frequencies and directions and refuses a different component count; prefer
/// the overload that takes a SeaStateChangeKind.
std::string CheckSeaStateChange(const std::vector<WaveComponent>& current,
                                const std::vector<WaveComponent>& proposed,
                                const SeaStateChangeLimits& limits);

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVES_SEA_STATE_CHANGE_H
