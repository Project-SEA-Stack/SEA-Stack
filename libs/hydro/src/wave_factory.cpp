/*********************************************************************
 * @file  wave_factory.cpp
 * @brief Implements one-step wave construction.
 *********************************************************************/

#include <seastack/hydro/wave_factory.h>
#include <seastack/hydro/waves/component_sampler.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace seastack::hydro {

std::shared_ptr<LinearDirectionalWaveField> MakeWave(
    const SeaStateDefinition& def,
    const HydroData& data,
    int num_bodies,
    double ramp_duration) {
    if (num_bodies <= 0) {
        throw std::invalid_argument(
            "MakeWave: num_bodies must be > 0 (got " +
            std::to_string(num_bodies) + ")");
    }
    if (def.g <= 0.0) {
        throw std::invalid_argument(
            "MakeWave: gravity (g) must be > 0 (got " +
            std::to_string(def.g) + ")");
    }
    if (def.depth <= 0.0 && !std::isinf(def.depth)) {
        throw std::invalid_argument(
            "MakeWave: water depth must be > 0 or infinite (got " +
            std::to_string(def.depth) + ")");
    }
    auto components = ComponentSampler::Build(def);
    auto wave = std::make_shared<LinearDirectionalWaveField>(
        std::move(components), def.depth);
    wave->SetNumBodies(static_cast<unsigned int>(num_bodies));
    wave->ApplySimulationEnvironment(data.GetSimulationInfo(), false);
    if (ramp_duration > 0.0) {
        wave->SetRampDuration(ramp_duration);
    }
    wave->Initialize();
    return wave;
}

}  // namespace seastack::hydro
