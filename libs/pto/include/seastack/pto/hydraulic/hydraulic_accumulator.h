/*********************************************************************
 * @file  hydraulic_accumulator.h
 * @brief Gas-charged hydraulic accumulator (polytropic model).
 *
 * Pressure–volume relationship:
 *   P * V_gas^gamma = P_0 * V_0^gamma   (polytropic)
 * where V_gas = V_total - V_oil.
 *
 * State: current oil volume.
 *
 * Solver-agnostic: C++ stdlib only (no Eigen, no Chrono).
 *********************************************************************/
#ifndef SEASTACK_PTO_HYDRAULIC_ACCUMULATOR_H
#define SEASTACK_PTO_HYDRAULIC_ACCUMULATOR_H

namespace seastack {
namespace pto {

struct AccumulatorParams {
    double total_volume;        // V_total [m^3]
    double precharge_pressure;  // P_0 [Pa]
    double gamma;               // polytropic exponent (1.4 for adiabatic)
};

class HydraulicAccumulator {
  public:
    explicit HydraulicAccumulator(const AccumulatorParams& params);

    /// Current gas-side pressure [Pa].
    double Pressure() const;

    /// Current oil volume in the accumulator [m^3].
    double OilVolume() const { return oil_volume_; }

    /// Set the oil volume directly (clamped to [0, MaxOilVolume()]).
    void SetOilVolume(double volume);

    /// Pressure that would result from a given oil volume.
    double PressureForOilVolume(double oil_volume) const;

    /// Maximum oil that can enter before gas is fully compressed.
    /// Leaves a small residual gas volume to avoid singularity.
    double MaxOilVolume() const;

    /// Total gas volume at precharge (= total_volume) [m^3].
    double PrechargeGasVolume() const { return params_.total_volume; }

  private:
    AccumulatorParams params_;
    double oil_volume_;
};

}  // namespace pto
}  // namespace seastack

#endif  // SEASTACK_PTO_HYDRAULIC_ACCUMULATOR_H
