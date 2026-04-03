/*********************************************************************
 * @file  trimaran_sea_state.h
 * @brief Shared sea state definition for all trimaran demos.
 *
 * Returns a JONSWAP irregular sea state with cos-2s directional
 * spreading (Hs = 3 m, Tp = 10 s, gamma = 3.3, s = 10, depth = 250 m).
 *
 * NOTE: The trimaran BEM data (trimaran.h5) contains only a single
 * wave heading (0 deg).  Spreading is therefore cosmetic in the wave
 * surface; excitation forces use the single-heading approximation
 * and a LOG_WARNING is emitted at startup.  To get physically
 * directional excitation forces, re-run the BEM with multiple
 * headings.
 *********************************************************************/

#ifndef TRIMARAN_SEA_STATE_H
#define TRIMARAN_SEA_STATE_H

#include <seastack/hydro/waves/wave_component.h>

namespace trimaran {

inline seastack::hydro::SeaStateDefinition MakeTrimaranDemoIrregularSea() {
    using namespace seastack::hydro;

    SeaStateDefinition def;
    def.type    = "irregular";
    def.depth   = 250.0;
    def.n_omega = 128;
    def.n_theta = 21;
    def.seed    = 42;

    SeaStatePartition partition;
    partition.spectrum.type  = "jonswap";
    partition.spectrum.Hs    = 3.0;
    partition.spectrum.Tp    = 10.0;
    partition.spectrum.gamma = 3.3;

    partition.spreading.type               = "cos2s";
    partition.spreading.mean_direction_deg  = 0.0;
    partition.spreading.s                   = 10.0;

    def.partitions.push_back(partition);
    return def;
}

}  // namespace trimaran

#endif  // TRIMARAN_SEA_STATE_H
