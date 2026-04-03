/*********************************************************************
 * @file  wave_factory.h
 * @brief One-step wave construction from parameters + H5 data.
 *
 * Collapses construct -> SetNumBodies -> ApplySimulationEnvironment -> Initialize
 * into a single validated call. Excitation TFs are built by ExcitationComponent
 * (InterpolateExcitationTransfer), not stored on the wave.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_WAVE_FACTORY_H
#define SEASTACK_HYDRO_WAVE_FACTORY_H

#include <seastack/hydro/hydro_data.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include <memory>

namespace seastack::hydro {

/// Construct a fully initialized LinearDirectionalWaveField from a sea state.
///
/// This is the one-step factory for all wave types (regular, irregular,
/// directional, bimodal). It samples components, applies simulation
/// gravity (and keeps sea-state depth), and initializes the wave.
///
/// @param def         Declarative sea-state definition.
/// @param data        H5 hydrodynamic data.
/// @param num_bodies  Number of bodies in the simulation.
/// @param ramp_duration  Excitation ramp duration [s]; 0 = no ramp.
/// @return Shared pointer to the ready-to-use wave.
std::shared_ptr<LinearDirectionalWaveField> MakeWave(
    const SeaStateDefinition& def,
    const HydroData& data,
    int num_bodies,
    double ramp_duration = 0.0);

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVE_FACTORY_H
