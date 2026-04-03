# SEAStack::PTO — Power Take-Off Module

The PTO module provides solver-independent models for power take-off
devices. It has **no external dependencies** beyond the C++ standard library —
no Eigen, no HDF5, no Chrono.

## Key types

| Type | Header | Purpose |
|------|--------|---------|
| `IPTOModel` | `include/seastack/pto/pto_model.h` | Abstract interface for 1-DOF PTO devices |
| `LinearPTO` | `include/seastack/pto/linear_pto.h` | Spring-damper: `F = -kx - cv` |
| `RectifiedHydraulicPTO` | `include/seastack/pto/rectified_hydraulic_pto.h` | Check-valve bridge, HP/LP accumulators, motor-generator, optional controller |

## Interface

```cpp
namespace seastack::pto {

class IPTOModel {
  public:
    virtual ~IPTOModel() = default;
    virtual double ComputeForce(double displacement,
                                double velocity,
                                double time) = 0;
};

}  // namespace seastack::pto
```

Units: displacement [m or rad], velocity [m/s or rad/s], time [s],
force [N or N.m]. Sign convention: force opposes motion.

## CMake target

```cmake
find_package(SEAStack REQUIRED)
target_link_libraries(my_app PRIVATE SEAStack::PTO)
```

## Standalone usage

PTO works without any simulation engine. See
[`examples/standalone_controller/`](../../examples/standalone_controller/)
for a runnable example that uses `LinearPTO` with a proportional controller
in a simple Euler integration loop.

## Chrono integration

When used inside a Chrono simulation, `PTOForceFunctor` in
`adapters/chrono/` wraps any `IPTOModel` and connects it to a
`ChLinkTSDA`. No adapter changes are needed for new PTO models.

## Extending

To add a new PTO model, implement `IPTOModel` and add the source to this
module. See [docs/extending/EXTENDING.md](../../docs/extending/EXTENDING.md)
for the full walkthrough.
