/*********************************************************************
 * @file  excitation_types.h
 * @brief Public types for wave excitation force configuration.
 *
 * Mirrors the RadiationMethod / RadiationKernelProcessing pattern
 * from radiation_types.h.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_EXCITATION_TYPES_H
#define SEASTACK_HYDRO_EXCITATION_TYPES_H

namespace seastack::hydro {

/**
 * @brief Wave excitation force calculation method.
 *
 * Selects the approach for computing excitation forces:
 *   - kIrfConvolution:  Convolve the excitation IRF K_exc(tau) from the H5
 *                       file with the wave elevation history eta(t-tau).
 *                       Matches WEC-Sim / legacy SEA-Stack irregular-wave
 *                       behaviour.  Only valid for long-crested seas
 *                       (single incident heading).
 *   - kFrequencyDomain: Evaluate forces via frequency-domain superposition
 *                       of the H5 excitation transfer function H(omega,beta).
 *                       Supports arbitrary directional and multi-modal seas.
 *
 * kAuto: frequency-domain for regular waves and for irregular seas with
 * multiple distinct wave headings; IRF convolution for long-crested irregular
 * (single-heading) seas only.
 */
enum class ExcitationMethod {
    kAuto,             ///< Auto-select: frequency-domain for regular and multi-heading
                       ///< irregular waves; IRF convolution for long-crested irregular
                       ///< (single-heading) seas
    kIrfConvolution,   ///< Time-domain convolution with excitation IRF
    kFrequencyDomain   ///< Frequency-domain superposition of excitation transfer function
};

/**
 * @brief Interpolation method for excitation transfer function from H5 data.
 *
 *   - kCartesian: Convert H5 mag/phase to Re/Im before interpolation.
 *                 Avoids catastrophic errors near phase-wrap boundaries.
 *                 Default for new SEA-Stack workflows.
 *   - kPolar:     Interpolate magnitude and phase independently, matching
 *                 HydroChrono behaviour faithfully.  Use for legacy
 *                 regression reproduction and back-comparison.
 */
enum class ExcitationInterpolation {
    kCartesian,  ///< Re/Im interpolation (default, avoids phase-wrap artefacts)
    kPolar       ///< Mag/phase interpolation (HydroChrono-compatible)
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_EXCITATION_TYPES_H
