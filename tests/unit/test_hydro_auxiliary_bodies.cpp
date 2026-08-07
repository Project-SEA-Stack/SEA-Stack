/*********************************************************************
 * @file  test_hydro_auxiliary_bodies.cpp
 * @brief Auxiliary (mooring-only) body slots in HydroForces.
 *
 * Auxiliary bodies are non-hydrodynamic bodies (e.g. a vehicle chassis) that
 * MoorDyn couples to.  They are appended after the hydrodynamic bodies in both
 * the SystemState and the force buffer.  The whole vehicle-mooring path rests
 * on two properties:
 *
 *   1. BEM components leave the auxiliary slots untouched, so a body with no
 *      hydrodynamic data never picks up buoyancy, radiation or excitation.
 *   2. Hydrodynamic body indices do not shift when auxiliary bodies are added,
 *      so an existing case gives identical results with an aux body appended.
 *
 * Chrono-free: uses stub components in place of real BEM components.
 *********************************************************************/

#include <seastack/hydro/hydro_forces.h>

#include "test_macros.h"

#include <memory>
#include <stdexcept>
#include <vector>

using seastack::hydro::BodyForces;
using seastack::hydro::HydroComponentType;
using seastack::hydro::HydroForces;
using seastack::hydro::IHydroForceComponent;
using seastack::hydro::SystemState;

namespace {

TestResults test_results;

/// Stands in for a BEM component: writes a per-body constant into the first
/// `num_bodies` slots and nothing beyond them, exactly as the real components do.
class StubBemComponent : public IHydroForceComponent {
  public:
    StubBemComponent(HydroComponentType type, int num_bodies, double magnitude)
        : type_(type), num_bodies_(num_bodies), magnitude_(magnitude) {}

    HydroComponentType Type() const override { return type_; }

    void Compute(const SystemState& state, double /*time*/, BodyForces& inout_forces) override {
        if (static_cast<int>(state.bodies.size()) < num_bodies_ ||
            static_cast<int>(inout_forces.size()) < num_bodies_) {
            throw std::runtime_error("StubBemComponent: buffer too small");
        }
        for (int i = 0; i < num_bodies_; ++i) {
            inout_forces[i].force.z() += magnitude_ * (i + 1);
        }
    }

  private:
    HydroComponentType type_;
    int num_bodies_;
    double magnitude_;
};

/// Stands in for the mooring component: writes into every coupled slot,
/// auxiliary bodies included.
class StubMooringComponent : public IHydroForceComponent {
  public:
    explicit StubMooringComponent(double magnitude) : magnitude_(magnitude) {}

    HydroComponentType Type() const override { return HydroComponentType::kMooring; }

    void Compute(const SystemState& /*state*/, double /*time*/,
                 BodyForces& inout_forces) override {
        for (auto& force : inout_forces) {
            force.force.x() += magnitude_;
        }
    }

  private:
    double magnitude_;
};

SystemState MakeState(int num_bodies) {
    SystemState state;
    state.bodies.resize(static_cast<std::size_t>(num_bodies));
    return state;
}

std::vector<std::unique_ptr<IHydroForceComponent>> MakeComponents(int num_hydro_bodies) {
    std::vector<std::unique_ptr<IHydroForceComponent>> components;
    components.push_back(std::make_unique<StubBemComponent>(HydroComponentType::kHydrostatics,
                                                            num_hydro_bodies, 100.0));
    components.push_back(std::make_unique<StubBemComponent>(HydroComponentType::kRadiation,
                                                            num_hydro_bodies, 10.0));
    components.push_back(std::make_unique<StubMooringComponent>(7.0));
    return components;
}

/// One hydro body plus one auxiliary body: BEM stays out of the aux slot,
/// mooring reaches it.
void TestAuxiliarySlotIsBemFree() {
    constexpr int kHydroBodies = 1;
    constexpr int kCoupledBodies = 2;

    HydroForces forces(kHydroBodies, MakeComponents(kHydroBodies), kCoupledBodies);
    const BodyForces result = forces.Evaluate(MakeState(kCoupledBodies), 0.0);

    TEST_ASSERT(result.size() == static_cast<std::size_t>(kCoupledBodies),
                "Evaluate returns one wrench per coupled body");
    TEST_ASSERT(forces.num_bodies() == kHydroBodies,
                "num_bodies() reports hydrodynamic bodies only");

    // Hydro body 0: hydrostatics 100 + radiation 10, plus mooring in x.
    TEST_NEAR(result[0].force.z(), 110.0, 1e-12, "Hydro body carries the BEM contributions");
    TEST_NEAR(result[0].force.x(), 7.0, 1e-12, "Hydro body carries the mooring contribution");

    // Auxiliary body 1: mooring only, no BEM.
    TEST_NEAR(result[1].force.z(), 0.0, 1e-12, "Auxiliary slot carries no BEM force");
    TEST_NEAR(result[1].force.x(), 7.0, 1e-12, "Auxiliary slot carries the mooring force");
}

/// Adding an auxiliary body must not renumber the hydrodynamic bodies.
void TestHydroIndicesDoNotShift() {
    constexpr int kHydroBodies = 2;

    HydroForces without_aux(kHydroBodies, MakeComponents(kHydroBodies));
    const BodyForces baseline = without_aux.Evaluate(MakeState(kHydroBodies), 0.0);

    HydroForces with_aux(kHydroBodies, MakeComponents(kHydroBodies), kHydroBodies + 1);
    const BodyForces appended = with_aux.Evaluate(MakeState(kHydroBodies + 1), 0.0);

    TEST_ASSERT(baseline.size() == static_cast<std::size_t>(kHydroBodies),
                "Without auxiliary bodies the buffer is the hydro body count");
    TEST_ASSERT(appended.size() == static_cast<std::size_t>(kHydroBodies + 1),
                "With one auxiliary body the buffer grows by one");

    for (int i = 0; i < kHydroBodies; ++i) {
        TEST_NEAR(appended[i].force.z(), baseline[i].force.z(), 1e-12,
                  "Hydro body force is unchanged by appending an auxiliary body");
    }
    TEST_NEAR(appended[kHydroBodies].force.z(), 0.0, 1e-12,
              "The appended slot is the auxiliary body, not a shifted hydro body");
}

/// A state that does not cover every coupled body is an integration bug and
/// must throw rather than silently drop auxiliary wrenches.
void TestMismatchedStateThrows() {
    constexpr int kHydroBodies = 1;
    constexpr int kCoupledBodies = 2;

    HydroForces forces(kHydroBodies, MakeComponents(kHydroBodies), kCoupledBodies);

    bool threw_on_short_state = false;
    try {
        forces.Evaluate(MakeState(kHydroBodies), 0.0);
    } catch (const std::runtime_error&) {
        threw_on_short_state = true;
    }
    TEST_ASSERT(threw_on_short_state, "A state missing the auxiliary body throws");

    bool threw_on_long_state = false;
    try {
        forces.Evaluate(MakeState(kCoupledBodies + 1), 0.0);
    } catch (const std::runtime_error&) {
        threw_on_long_state = true;
    }
    TEST_ASSERT(threw_on_long_state, "A state longer than the coupled body count throws");
}

}  // namespace

int main() {
    std::cout << "=== HydroForces auxiliary coupled bodies ===" << std::endl;

    TestAuxiliarySlotIsBemFree();
    TestHydroIndicesDoNotShift();
    TestMismatchedStateThrows();

    test_results.Summary();
    return test_results.failed == 0 ? 0 : 1;
}
