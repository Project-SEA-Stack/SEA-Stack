#include <seastack/pto/hydraulic/hydraulic_accumulator.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace seastack {
namespace pto {

namespace {
// Minimum fraction of total volume that must remain as gas to avoid
// a pressure singularity (V_gas -> 0).
constexpr double kMinGasFraction = 0.01;
}  // namespace

HydraulicAccumulator::HydraulicAccumulator(const AccumulatorParams& params)
    : params_(params), oil_volume_(0.0) {
    if (params_.total_volume <= 0.0)
        throw std::invalid_argument("Accumulator total_volume must be > 0");
    if (params_.precharge_pressure <= 0.0)
        throw std::invalid_argument("Accumulator precharge_pressure must be > 0");
    if (params_.gamma <= 0.0)
        throw std::invalid_argument("Accumulator gamma must be > 0");
}

double HydraulicAccumulator::Pressure() const {
    return PressureForOilVolume(oil_volume_);
}

void HydraulicAccumulator::SetOilVolume(double volume) {
    oil_volume_ = std::clamp(volume, 0.0, MaxOilVolume());
}

double HydraulicAccumulator::PressureForOilVolume(double oil_volume) const {
    double v_gas = params_.total_volume - oil_volume;
    if (v_gas <= 0.0)
        v_gas = params_.total_volume * kMinGasFraction;

    // P * V_gas^gamma = P_0 * V_0^gamma
    // P = P_0 * (V_0 / V_gas)^gamma
    double ratio = params_.total_volume / v_gas;
    return params_.precharge_pressure * std::pow(ratio, params_.gamma);
}

double HydraulicAccumulator::MaxOilVolume() const {
    return params_.total_volume * (1.0 - kMinGasFraction);
}

}  // namespace pto
}  // namespace seastack
