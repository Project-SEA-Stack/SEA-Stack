# External Controllers (preview)

An external controller is a script that reads the model's state every control
tick and returns new set-points for motors, spring rest lengths and spring rest
angles. It runs in its own process and talks to SEA-Stack over the same
loopback protocol as [external force modules](EXTERNAL_FORCE_MODULES.md).
Python is supported today; MATLAB is not yet (see [MATLAB](#matlab)).

**Preview status (v1.1):** the YAML keys, channel names and Python API may
change in v1.2. Controller channels are not yet written to HDF5; print or log
what you need (see [Printing and logging](#printing-and-logging)).

## Quick start

The worked example is in `demos/iea_sphere/controller_winch/` (source tree:
`data/demos/run_seastack/iea_sphere/controller_winch/`). A rope (a TSDA) holds
the IEA sphere from a fixed point above it. After 20 s, a winch hauls in 1 m of
rope by lowering the rope's rest length.

1. Point the setup file at a controller file:

   ```yaml
   # iea_sphere_controller_winch.setup.yaml
   controller_file: winch.controller.yaml
   ```

2. Say which channels the script reads and writes:

   ```yaml
   # winch.controller.yaml
   command: [python, winch_controller.py]   # relative to this file
   control_dt: 0.05                         # [s] whole multiple of time_step
   sensors:
     bodies: [body1]
     tsdas: [winch_rope]
   actuators:
     tsdas: [winch_rope]
   config:                                  # passed to setup(cfg)
     haul_start_s: 20.0
     haul_length_m: 1.0
     haul_speed_m_per_s: 0.2
   ```

3. Write the controller:

   ```python
   from seastack_external import ControllerModule, ControllerState, run

   class WinchController(ControllerModule):
       def setup(self, cfg):
           self.t_start = float(cfg["haul_start_s"])
           self.haul = float(cfg["haul_length_m"])
           self.speed = float(cfg["haul_speed_m_per_s"])
           self.rest0 = None

       def step(self, state: ControllerState):
           rope = state.tsda("winch_rope")
           if self.rest0 is None:
               self.rest0 = rope.rest_length
           hauled = min(max(0.0, (state.time - self.t_start) * self.speed), self.haul)
           return {"tsda.winch_rope.rest_length": self.rest0 - hauled}

   if __name__ == "__main__":
       run(WinchController())
   ```

4. Run it like any other case:

   ```bash
   run_seastack demos/iea_sphere/controller_winch/iea_sphere_controller_winch.setup.yaml
   ```

   At start-up SEA-Stack prints what the controller is attached to:

   ```
   controller 'winch_controller.py' attached: 26 sensor and 1 actuator channels, control_dt = 0.05 s (every 5 steps)
     writes tsda.winch_rope.rest_length [m]
   ```

**Check against theory.** The rope (stiffness k) acts in series with the
sphere's hydrostatic stiffness \(\rho g A_{wp}\), where \(A_{wp} = \pi R^2\) and
R = 5 m. Hauling in \(\Delta L_0\) lifts the sphere by

\[
\Delta z = \frac{k\,\Delta L_0}{k + \rho g \pi R^2}
\]

With k = 770 kN/m, \(\rho\) = 1000 kg/m³, g = 9.81 m/s² and
\(\Delta L_0\) = 1 m, this gives \(\Delta z\) = 0.49985 m. SEA-Stack gives
0.49987 m (relative difference 6e-5). The run takes under a second.

## Channels

Every channel is named `<element>.<name>.<field>`, for example
`body.body1.pos_z` or `motor.slew.angle`. **You write an actuator with the
same name you read it by.** Units are SI. Angles are in radians even if the
model YAML uses degrees.

List an element under `sensors:` to read all of its channels. List it under
`actuators:` to write its writable channels. To read back a channel you also
write (for example a TSDA's `rest_length`), list the element under both.

The table below is generated from the C++ channel registry. A unit test
(`test_external_controller`) fails if the two drift apart.

<!-- controller-channel-table:begin -->
| Channel | Applies to | Unit | Access | Meaning |
|---------|------------|------|--------|---------|
| `body.<name>.pos_x` | any body | m | read | Centre-of-mass position, world frame (as HDF5 `position`) |
| `body.<name>.pos_y` | any body | m | read | Centre-of-mass position, world frame |
| `body.<name>.pos_z` | any body | m | read | Centre-of-mass position, world frame (z up) |
| `body.<name>.rot_x` | any body | rad | read | Cardan angle about x, XYZ sequence (as HDF5 `orientation_xyz`) |
| `body.<name>.rot_y` | any body | rad | read | Cardan angle about y, XYZ sequence |
| `body.<name>.rot_z` | any body | rad | read | Cardan angle about z, XYZ sequence |
| `body.<name>.quat_w` | any body | - | read | Orientation quaternion, body to world (scalar part) |
| `body.<name>.quat_x` | any body | - | read | Orientation quaternion, body to world |
| `body.<name>.quat_y` | any body | - | read | Orientation quaternion, body to world |
| `body.<name>.quat_z` | any body | - | read | Orientation quaternion, body to world |
| `body.<name>.vel_x` | any body | m/s | read | Centre-of-mass velocity, world frame |
| `body.<name>.vel_y` | any body | m/s | read | Centre-of-mass velocity, world frame |
| `body.<name>.vel_z` | any body | m/s | read | Centre-of-mass velocity, world frame |
| `body.<name>.ang_vel_x` | any body | rad/s | read | Angular velocity, world frame |
| `body.<name>.ang_vel_y` | any body | rad/s | read | Angular velocity, world frame |
| `body.<name>.ang_vel_z` | any body | rad/s | read | Angular velocity, world frame |
| `body.<name>.acc_x` | any body | m/s^2 | read | Centre-of-mass acceleration, world frame, from the last completed step |
| `body.<name>.acc_y` | any body | m/s^2 | read | Centre-of-mass acceleration, world frame, from the last completed step |
| `body.<name>.acc_z` | any body | m/s^2 | read | Centre-of-mass acceleration, world frame, from the last completed step |
| `body.<name>.ang_acc_x` | any body | rad/s^2 | read | Angular acceleration, world frame, from the last completed step |
| `body.<name>.ang_acc_y` | any body | rad/s^2 | read | Angular acceleration, world frame, from the last completed step |
| `body.<name>.ang_acc_z` | any body | rad/s^2 | read | Angular acceleration, world frame, from the last completed step |
| `motor.<name>.angle` | rotation motor | rad | read/write | Chrono motor angle (`GetMotorAngle`, not wrapped). Write: angle set-point |
| `motor.<name>.angle_dt` | rotation motor | rad/s | read/write | Motor angular speed. Write: optional rate feed-forward for the set-point |
| `motor.<name>.torque` | rotation motor | N*m | read | Motor reaction torque (`GetMotorTorque`) |
| `motor.<name>.position` | linear motor | m | read/write | Chrono motor position (`GetMotorPos`). Write: position set-point |
| `motor.<name>.position_dt` | linear motor | m/s | read/write | Motor speed. Write: optional rate feed-forward for the set-point |
| `motor.<name>.force` | linear motor | N | read | Motor reaction force (`GetMotorForce`) |
| `tsda.<name>.length` | any tsda | m | read | Distance between the two attachment points |
| `tsda.<name>.length_dt` | any tsda | m/s | read | Rate of change of length, positive when extending |
| `tsda.<name>.rest_length` | any tsda | m | read/write | Rest (free) length used by the TSDA force law. Write: new rest length |
| `tsda.<name>.force` | any tsda | N | read | Chrono TSDA force, negative in tension (linear law: f - k (L - L0) - c dL/dt) |
| `rsda.<name>.angle` | any rsda | rad | read | Chrono RSDA angle (`GetAngle`) about the RSDA axis |
| `rsda.<name>.angle_dt` | any rsda | rad/s | read | Rate of change of angle |
| `rsda.<name>.rest_angle` | any rsda | rad | read/write | Rest angle used by the RSDA torque law (radians even if `angle_degrees: true`). Write: new rest angle |
| `rsda.<name>.torque` | any rsda | N*m | read | Chrono RSDA torque, t - k (a - a0) - c da/dt |
<!-- controller-channel-table:end -->

Notes on conventions:

- **Body channels** use the same conventions as the HDF5 body output: the
  position is the centre of mass in the world frame, `rot_*` are Cardan XYZ
  angles, and velocities are world-frame. Accelerations are those of the last
  completed step (zero before the first step).
- **TSDA force** follows Chrono's sign: negative in tension. Chrono computes
  the native spring-damper force as \(f - k(L - L_0) - c\,\dot L\) using the
  link's *current* rest length \(L_0\). Writing `rest_length` therefore moves
  the spring's neutral point.
- **RSDA torque** likewise is \(\tau - k(\theta - \theta_0) - c\,\dot\theta\),
  with the current rest angle \(\theta_0\).
- **Motor angle / position** are Chrono's motor coordinates (body 1 relative
  to body 2, about or along the motor axis).

## Driving motors

Only position motors can be driven. In the model YAML, set
`actuation_type: POSITION` and `actuation_function: {type: CONTROLLER}`:

```yaml
motors:
  - name: slew
    type: ROTATION
    body1: tower
    body2: ground
    location: [0, 0, 10]
    axis: [0, 0, 1]
    actuation_type: POSITION
    actuation_function:
      type: CONTROLLER
```

Then list the motor under `actuators: motors: [slew]`. A rotation motor takes
`angle` [rad] and an optional feed-forward rate `angle_dt` [rad/s]. A linear
motor takes `position` [m] and an optional `position_dt` [m/s]. If you set a
rate, keep it consistent with how the position moves, or leave it unset (it
then holds its last value, which starts at the motor's initial rate).

SEA-Stack stops with an error if a listed motor is a SPEED or FORCE motor, or
if its `actuation_function` is not `CONTROLLER`. It warns if a `CONTROLLER`
motor is not listed under `actuators:`, because that motor would hold its
initial set-point for the whole run.

## Tension-only ropes (ropes_file)

A TSDA in the model YAML is a linear spring-damper: it pushes as well as
pulls. For a winch rope or hoist, add a `ropes_file` to the setup file to
replace the force law of named TSDAs with a tension-only rope:

```yaml
# my_case.setup.yaml
ropes_file: ../common/my.ropes.yaml
```

```yaml
# my.ropes.yaml
ropes:
  - link: winch_rope          # TSDA name in the model YAML
    law: tension_only         # the only law in v1.1
    ea_N: 1.0e8               # [N] axial stiffness EA (> 0)
    damping_Ns_per_m: 2.0e3   # [N s/m] optional, default 0
```

With L the TSDA length, L0 its rest length and dL/dt the extension rate, the
rope tension is

\[
T = \max\!\left(0,\ \frac{EA}{L_0}(L - L_0) + c\,\frac{dL}{dt}\right).
\]

The rope never pushes. The law is continuous, which keeps the implicit solver
from chattering when the rope goes slack or snaps taut. The TSDA's own spring
and damper coefficients are ignored. The `force` sensor channel reports
\(-T\) (Chrono's sign: negative in tension). Drive `rest_length` from a
controller to model a winch. `ropes_file` works with or without a
`controller_file`. Unknown keys, unknown TSDA names and duplicates stop the
run before it starts.

## Timing: control_dt and the zero-order hold

The controller runs at time 0 and then every `control_dt` seconds.
`control_dt` must be a whole multiple of the simulation `time_step`. If you
leave it out, it defaults to `time_step`. At each tick:

1. SEA-Stack reads the sensor channels at time t (the state at the start of
   the step).
2. Your `step(state)` returns new commands.
3. SEA-Stack applies them and holds them, unchanged, over
   \([t, t + \text{control\_dt})\).

Every actuator starts at its value in the model (the rest length or rest angle
from the model YAML, or the motor's initial position), so nothing moves until
you command it.

**Holding a value.** Any actuator you leave out of the returned dict keeps its
previous value. A NaN or infinite value is also held. The script prints a
warning the first time this happens for each channel.

## Controller file reference

```yaml
command: [python, my_controller.py]  # required; argv of the child process
control_dt: 0.05                     # [s] optional; default = time_step
timeout_ms: 10000                    # optional; per-tick reply timeout
working_directory: .                 # optional; default = this file's folder
sensors:                             # optional
  bodies: [float]
  motors: [slew]
  tsdas: [winch_rope]
  rsdas: [hinge]
actuators:                           # optional; no bodies (read-only)
  motors: [slew]
  tsdas: [winch_rope]
  rsdas: [hinge]
config: {gain: 2.0}                  # optional; passed to setup(cfg)
```

- Relative `.py` paths in `command` are resolved against the controller file.
  The interpreter rules are the same as for external PTOs: `python` must be on
  `PATH` on Windows, and macOS/Linux fall back to `python3`.
- Each case has at most one `controller_file`. It can be used together with
  `external_pto_file`.
- Validation is strict. Unknown keys, unknown element names, duplicates and
  read-only actuators stop the run before it starts. Error messages list the
  valid names and suggest the closest match:

  ```
  controller_file c.controller.yaml: sensors: unknown key 'body'. Did you mean 'bodies'? Valid keys: bodies, motors, tsdas, rsdas.
  ```

## Python API

Import from `seastack_external` (shipped as `python/seastack_external.py` in
release ZIPs; standard library only).

`ControllerModule` hooks:

| Hook | When | Return |
|------|------|--------|
| `setup(self, cfg)` | once, before t = 0 | optional `{"name", "version"}` |
| `step(self, state)` | every tick | `{actuator_channel: value}` |
| `reset(self)` | on reset | nothing |

`ControllerState`:

- `state.time` [s] and `state.dt` (= `control_dt`) [s].
- `state["body.float.pos_z"]` reads one channel. An unknown name raises
  `KeyError` with a suggestion and the list of channels.
- `state.body("float")`, `state.motor("slew")`, `state.tsda("rope")` and
  `state.rsda("hinge")` return objects with one attribute per field. Bodies
  also get tuples: `pos`, `rot`, `quat_wxyz`, `vel`, `ang_vel`, `acc`,
  `ang_acc`.
- `state.get(name, default)`, `name in state` and `state.as_dict()` work as
  for a dict.

`step()` must return a dict (or `None` to hold everything). The run stops with
a clear error if the dict contains a sensor channel, an unknown channel or a
non-numeric value.

### Testing without SEA-Stack

`ControllerHarness` drives a controller in-process, applying the same
hold rules as SEA-Stack:

```python
from seastack_external import ControllerHarness

h = ControllerHarness(
    WinchController(),
    sensors={"bodies": ["body1"], "tsdas": ["winch_rope"]},
    actuators={"tsdas": ["winch_rope"]},
    config={"haul_start_s": 1.0, "haul_length_m": 1.0, "haul_speed_m_per_s": 0.5},
    control_dt=0.5,
)
rest = "tsda.winch_rope.rest_length"
assert h.step(2.0, {rest: 30.0})[rest] == 29.5
print(h.log)   # anything the controller printed
```

For motors, pass `motor_types={"slew": "rotation"}` (or `"linear"`). The winch
script runs its own harness test with `python winch_controller.py --selftest`.

### Printing and logging

On Windows the child process has no console, so SEA-Stack captures
`print()` output (and `self.log(msg)`) from `setup()` and `step()` and shows
it in the SEA-Stack console, prefixed with the script name:

```
[winch_controller.py] t = 25.00 s: hauled 1.00 m, heave z = +0.500 m
```

Up to 200 lines per tick are shown. If `step()` raises, the run stops and the
console shows your traceback (frames from `seastack_external.py` are hidden)
and the simulation time.

## Failures

A controller error stops the run with a non-zero exit code (in a campaign, the
cell is marked failed). The message names the script and the time, for example
`controller 'winch_controller.py' failed at t = 20.05 s: ...`. Typical causes:

| Message ends with | Cause |
|-------------------|-------|
| a Python traceback | `step()` or `setup()` raised |
| `(no reply within timeout_ms = ...)` | `step()` blocked or was too slow |
| `(the controller process exited ...)` | the script exited (for example `sys.exit`) or crashed |
| `could not be started` | wrong interpreter or missing script |

## Cost

Each tick costs one loopback round trip. With `control_dt = time_step` this
adds roughly 0.1 to 0.5 ms per step, depending on the machine and what
`step()` does. Use a larger `control_dt` if the controller does not need
every step.

## MATLAB

There is no MATLAB set-point helper yet. The wire protocol is the same as for
external force modules: the `initialize` message has `kind: "controller"`, an
`in_names` array (sensor channels) and an `out_names` array (actuator
channels); each `evaluate` reply returns one value per `out_names` entry, with
NaN meaning "hold". See
[EXTERNAL_FORCE_MODULES.md](EXTERNAL_FORCE_MODULES.md#wire-protocol-v1).

## Limits (v1.1 preview)

- Controller channels are not written to HDF5 (planned for v1.2).
- No wave-elevation sensor yet (planned for v1.2).
- Coupling is explicit: commands are computed from the state at the start of
  the tick. Fast loops need a small `control_dt`.
- Only position motors, TSDA rest lengths and RSDA rest angles can be driven.
- `ropes_file` has one law (`tension_only`); no rope mass, drum or
  hydrodynamic drag.
  Bodies are read-only; apply body forces with an external PTO or force
  module instead.
