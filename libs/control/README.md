# SEAStack::Control — Controller Module

The Control module provides a solver-independent interface for feedback
controllers. It is **header-only** with **no external dependencies** beyond
the C++ standard library — no Eigen, no HDF5, no Chrono.

## Key types

| Type | Header | Purpose |
|------|--------|---------|
| `IController` | `include/seastack/control/controller.h` | Abstract interface for scalar feedback controllers |
| `PIController` | `include/seastack/control/pi_controller.h` | Proportional-integral controller with anti-windup |

## Interface

```cpp
namespace seastack::control {

class IController {
  public:
    virtual ~IController() = default;
    virtual double Compute(double measurement, double time) = 0;
    virtual void Reset() {}
};

}  // namespace seastack::control
```

Units are application-dependent. Document them in each concrete controller.

## CMake target

```cmake
find_package(SEAStack REQUIRED)
target_link_libraries(my_app PRIVATE SEAStack::Control)
```

## Standalone usage

Control works without any simulation engine. See
[`examples/standalone_controller/`](../../examples/standalone_controller/)
for a runnable example that pairs a proportional controller with a LinearPTO
in a simple Euler integration loop.

## Integration with PTO

Controllers are typically connected to PTO models. For example,
`RectifiedHydraulicPTO` accepts an optional `IController*` to regulate
motor speed.

## Extending

To add a new controller, implement `IController` in a header under this
module. See [docs/extending/EXTENDING.md](../../docs/extending/EXTENDING.md)
for the full walkthrough.
