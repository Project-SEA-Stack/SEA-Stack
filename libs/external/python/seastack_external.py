"""
SEA-Stack external force module client (Python).

Speaks the v1 length-prefixed JSON protocol over TCP to 127.0.0.1.
See docs/extending/EXTERNAL_FORCE_MODULES.md and docs/extending/CONTROLLERS.md.

Author a controller (preview) by subclassing ControllerModule and returning
named set-points from step(state); test it offline with ControllerHarness.

Author a 1-DOF PTO (TSDA or RSDA) by subclassing PtoModule and implementing
force(state). Optional setup(cfg) reads config; optional reset / commit /
rollback handle stateful controllers:

    from seastack_external import PtoModule, run

    class MyPTO(PtoModule):
        def setup(self, cfg):
            self.c = float(cfg.get("damping", 50.0))

        def force(self, state):
            return -self.c * state.velocity

    if __name__ == "__main__":
        run(MyPTO())
"""

from __future__ import annotations

import argparse
import contextlib
import difflib
import io
import json
import math
import os
import socket
import struct
import sys
import traceback
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


PROTOCOL_VERSION = 1

# Lean protocol (replay / IPC unit tests): channels 0/1 only.
DEFAULT_PTO_CHANNELS: List[str] = ["displacement", "velocity"]

# Rich schemas published by ExternalPtoModel when EnableRichState is on.
PTO_TSDA_RICH_CHANNELS: List[str] = [
    "displacement",
    "velocity",
    "length",
    "rest_length",
    "rel_accel",
    "body1_pos_x",
    "body1_pos_y",
    "body1_pos_z",
    "body1_vel_x",
    "body1_vel_y",
    "body1_vel_z",
    "body2_pos_x",
    "body2_pos_y",
    "body2_pos_z",
    "body2_vel_x",
    "body2_vel_y",
    "body2_vel_z",
]

PTO_RSDA_RICH_CHANNELS: List[str] = [
    "displacement",
    "velocity",
    "angle",
    "rest_angle",
    "rel_accel",
    "body1_pos_x",
    "body1_pos_y",
    "body1_pos_z",
    "body1_vel_x",
    "body1_vel_y",
    "body1_vel_z",
    "body2_pos_x",
    "body2_pos_y",
    "body2_pos_z",
    "body2_vel_x",
    "body2_vel_y",
    "body2_vel_z",
]


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed while receiving")
        buf.extend(chunk)
    return bytes(buf)


def recv_message(sock: socket.socket) -> dict:
    hdr = _recv_exact(sock, 4)
    (length,) = struct.unpack(">I", hdr)
    payload = _recv_exact(sock, length) if length else b""
    return json.loads(payload.decode("utf-8"))


def send_message(sock: socket.socket, obj: dict) -> None:
    payload = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    sock.sendall(struct.pack(">I", len(payload)) + payload)


def _vec3(
    values: Sequence[float], names: Sequence[str], sx: str, sy: str, sz: str
) -> Tuple[float, float, float]:
    by_name = {n: float(v) for n, v in zip(names, values)}
    return (
        float(by_name.get(sx, 0.0)),
        float(by_name.get(sy, 0.0)),
        float(by_name.get(sz, 0.0)),
    )


@dataclass
class PtoState:
    """Named PTO inputs for force(state) / torque(state) authoring.

    Universal fields (always meaningful when present in the channel list):
      time, dt, displacement, velocity, rel_accel, body1/body2 position & velocity.

    Link-specific (set when the host publishes them via in_names):
      length, rest_length  (TSDA)
      angle, rest_angle    (RSDA)

    Units: m, m/s, N on a TSDA; rad, rad/s, N·m on an RSDA.
    The return value of force() is force [N] or torque [N·m].
    """

    time: float = 0.0
    dt: float = 0.0
    displacement: float = 0.0
    velocity: float = 0.0
    length: float = 0.0
    rest_length: float = 0.0
    angle: float = 0.0
    rest_angle: float = 0.0
    rel_accel: float = 0.0
    body1_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    body1_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    body2_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    body2_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    link_kind: str = ""
    raw: List[float] = field(default_factory=list)
    names: List[str] = field(default_factory=list)

    def get(self, name: str, default: float = 0.0) -> float:
        by_name = {n: float(v) for n, v in zip(self.names, self.raw)}
        return float(by_name.get(name, default))

    @classmethod
    def from_inputs(
        cls,
        t: float,
        dt: float,
        inputs: Sequence[float],
        names: Optional[Sequence[str]] = None,
        link_kind: str = "",
    ) -> "PtoState":
        channel_names = list(names) if names else list(DEFAULT_PTO_CHANNELS)
        # Pad / truncate names to match inputs for zip safety.
        if len(channel_names) < len(inputs):
            channel_names = channel_names + [
                f"in_{i}" for i in range(len(channel_names), len(inputs))
            ]
        elif len(channel_names) > len(inputs):
            channel_names = channel_names[: len(inputs)]
        by_name = {n: float(v) for n, v in zip(channel_names, inputs)}
        disp = float(by_name.get("displacement", inputs[0] if inputs else 0.0))
        vel = float(by_name.get("velocity", inputs[1] if len(inputs) > 1 else 0.0))
        return cls(
            time=float(t),
            dt=float(dt),
            displacement=disp,
            velocity=vel,
            length=float(by_name.get("length", 0.0)),
            rest_length=float(by_name.get("rest_length", 0.0)),
            angle=float(by_name.get("angle", 0.0)),
            rest_angle=float(by_name.get("rest_angle", 0.0)),
            rel_accel=float(by_name.get("rel_accel", 0.0)),
            body1_position=_vec3(
                inputs, channel_names, "body1_pos_x", "body1_pos_y", "body1_pos_z"
            ),
            body1_velocity=_vec3(
                inputs, channel_names, "body1_vel_x", "body1_vel_y", "body1_vel_z"
            ),
            body2_position=_vec3(
                inputs, channel_names, "body2_pos_x", "body2_pos_y", "body2_pos_z"
            ),
            body2_velocity=_vec3(
                inputs, channel_names, "body2_vel_x", "body2_vel_y", "body2_vel_z"
            ),
            link_kind=str(link_kind or ""),
            raw=[float(x) for x in inputs],
            names=list(channel_names),
        )


def _attach_log(reply: Dict[str, Any], module: "ExternalForceModule") -> None:
    """Add pending module log lines as the reply's last key (`log`)."""
    lines = module._take_log()
    if lines:
        reply["log"] = [str(x) for x in lines]


class ExternalForceModule:
    """Subclass and implement initialize / evaluate / shutdown."""

    # Populated by ExternalForceModule.run() from the initialize handshake.
    _host_in_names: Optional[List[str]] = None
    _host_out_names: Optional[List[str]] = None
    _host_kind: str = ""

    def _take_log(self) -> List[str]:
        """Text lines to send with the next reply (`log` field). Default: none."""
        return []

    def _format_error(self, exc: BaseException) -> str:
        """Message sent to the host when a call raises."""
        return str(exc)

    def initialize(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        return {"name": self.__class__.__name__, "version": "1.0", "n_states": 0}

    def evaluate(self, t: float, dt: float, inputs: List[float]) -> List[float]:
        raise NotImplementedError

    def reset(self) -> None:
        pass

    def commit(self) -> None:
        pass

    def rollback(self) -> None:
        pass

    def shutdown(self) -> None:
        pass

    @classmethod
    def run(cls, instance: Optional["ExternalForceModule"] = None) -> None:
        module = instance if instance is not None else cls()
        parser = argparse.ArgumentParser(description="SEA-Stack external force module")
        parser.add_argument("--seastack-port", type=int, required=True)
        args = parser.parse_args()

        sock = socket.create_connection(("127.0.0.1", args.seastack_port), timeout=30.0)
        sock.settimeout(60.0)
        try:
            while True:
                msg = recv_message(sock)
                op = msg.get("op")
                if op == "initialize":
                    proto = int(msg.get("protocol", 0))
                    if proto and proto != PROTOCOL_VERSION:
                        send_message(
                            sock,
                            {
                                "status": "error",
                                "message": f"unsupported protocol {proto}",
                            },
                        )
                        break
                    cfg = msg.get("config") or {}
                    if isinstance(cfg, str):
                        cfg = json.loads(cfg) if cfg else {}
                    # Additive host channel names (ignored by older modules).
                    names = msg.get("in_names")
                    if isinstance(names, list):
                        module._host_in_names = [str(x) for x in names]
                    else:
                        module._host_in_names = None
                    out_names = msg.get("out_names")
                    if isinstance(out_names, list):
                        module._host_out_names = [str(x) for x in out_names]
                    else:
                        module._host_out_names = None
                    module._host_kind = str(msg.get("kind") or "")
                    try:
                        meta = module.initialize(cfg) or {}
                    except Exception as exc:  # noqa: BLE001 — report to host
                        reply = {"status": "error", "message": module._format_error(exc)}
                        _attach_log(reply, module)
                        send_message(sock, reply)
                        break
                    reply = {
                        "status": "ok",
                        "protocol": PROTOCOL_VERSION,
                        "name": meta.get("name", module.__class__.__name__),
                        "version": meta.get("version", "1.0"),
                        "n_states": int(meta.get("n_states", 0)),
                    }
                    _attach_log(reply, module)
                    send_message(sock, reply)
                elif op == "evaluate":
                    t = float(msg.get("t", 0.0))
                    dt = float(msg.get("dt", 0.0))
                    inputs = [float(x) for x in msg.get("in", [])]
                    try:
                        outputs = module.evaluate(t, dt, inputs)
                        reply = {"status": "ok", "out": [float(x) for x in outputs]}
                    except Exception as exc:  # noqa: BLE001 — report to host
                        reply = {"status": "error", "message": module._format_error(exc)}
                    _attach_log(reply, module)
                    send_message(sock, reply)
                elif op == "reset":
                    module.reset()
                    send_message(sock, {"status": "ok"})
                elif op == "commit":
                    module.commit()
                    send_message(sock, {"status": "ok"})
                elif op == "rollback":
                    module.rollback()
                    send_message(sock, {"status": "ok"})
                elif op == "shutdown":
                    module.shutdown()
                    send_message(sock, {"status": "ok"})
                    break
                else:
                    send_message(
                        sock,
                        {"status": "error", "message": f"unknown op: {op}"},
                    )
        finally:
            sock.close()


class PtoModule(ExternalForceModule):
    """1-DOF PTO authoring base: implement force(state) -> float.

    Works for both ChLinkTSDA (force [N]) and ChLinkRSDA (torque [N·m]).
    Optionally override setup(cfg), reset, commit, rollback, shutdown.
    """

    def __init__(self) -> None:
        super().__init__()
        self._channel_names: List[str] = list(DEFAULT_PTO_CHANNELS)
        self._link_kind: str = ""
        self._meta_name: str = self.__class__.__name__
        self._meta_version: str = "1.0"
        self._n_states: int = 0

    def setup(self, cfg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Optional configuration hook. Return meta overrides if desired."""
        return None

    def force(self, state: PtoState) -> float:
        raise NotImplementedError

    def torque(self, state: PtoState) -> float:
        """Alias for force(); return value is N·m on an RSDA."""
        return self.force(state)

    def initialize(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        if self._host_in_names:
            self._channel_names = list(self._host_in_names)
        else:
            self._channel_names = list(DEFAULT_PTO_CHANNELS)
        kind = self._host_kind or str(cfg.get("link_kind", "") or "")
        self._link_kind = kind
        meta = self.setup(cfg) or {}
        self._meta_name = str(meta.get("name", self.__class__.__name__))
        self._meta_version = str(meta.get("version", "1.0"))
        self._n_states = int(meta.get("n_states", 0))
        return {
            "name": self._meta_name,
            "version": self._meta_version,
            "n_states": self._n_states,
        }

    def evaluate(self, t: float, dt: float, inputs: List[float]) -> List[float]:
        state = PtoState.from_inputs(
            t, dt, inputs, names=self._channel_names, link_kind=self._link_kind
        )
        return [float(self.force(state))]


# ---------------------------------------------------------------------------
# Controllers (preview) — see docs/extending/CONTROLLERS.md
# ---------------------------------------------------------------------------

# Channel fields per element type, in the host's publication order. The host
# registry (apps/seastack/external_controller.cpp) is the source of truth; a
# unit test checks both against the table in docs/extending/CONTROLLERS.md.
CONTROLLER_FIELDS: Dict[str, Tuple[str, ...]] = {
    "body": (
        "pos_x", "pos_y", "pos_z",
        "rot_x", "rot_y", "rot_z",
        "quat_w", "quat_x", "quat_y", "quat_z",
        "vel_x", "vel_y", "vel_z",
        "ang_vel_x", "ang_vel_y", "ang_vel_z",
        "acc_x", "acc_y", "acc_z",
        "ang_acc_x", "ang_acc_y", "ang_acc_z",
    ),
    "rotation motor": ("angle", "angle_dt", "torque"),
    "linear motor": ("position", "position_dt", "force"),
    "tsda": ("length", "length_dt", "rest_length", "force"),
    "rsda": ("angle", "angle_dt", "rest_angle", "torque"),
}

# Fields that may be listed as actuators (written by the controller).
CONTROLLER_WRITABLE: Dict[str, Tuple[str, ...]] = {
    "rotation motor": ("angle", "angle_dt"),
    "linear motor": ("position", "position_dt"),
    "tsda": ("rest_length",),
    "rsda": ("rest_angle",),
}


def _suggest(word: str, choices: Sequence[str]) -> str:
    close = difflib.get_close_matches(word, list(choices), n=1, cutoff=0.6)
    return f" Did you mean '{close[0]}'?" if close else ""


def _list_names(names: Sequence[str], limit: int = 12) -> str:
    names = list(names)
    if not names:
        return "(none)"
    shown = ", ".join(names[:limit])
    return shown + (f", ... ({len(names)} in total)" if len(names) > limit else "")


class ControllerState:
    """Sensor values for one control tick, looked up by channel name.

    state["body.float.pos_z"]      -> float (KeyError lists close matches)
    state.body("float").pos        -> (x, y, z) tuple, world frame [m]
    state.tsda("winch_rope").length
    state.motor("slew").angle      (rotation) / .position (linear)
    state.rsda("hinge").angle
    state.time, state.dt           control tick time and control_dt [s]
    """

    def __init__(self, time: float, dt: float, names: Sequence[str], values: Sequence[float]):
        self.time = float(time)
        self.dt = float(dt)
        self.names: Tuple[str, ...] = tuple(names)
        self._values: Dict[str, float] = {n: float(v) for n, v in zip(names, values)}

    def __getitem__(self, name: str) -> float:
        try:
            return self._values[name]
        except KeyError:
            raise KeyError(
                f"no sensor channel '{name}'.{_suggest(name, self.names)} "
                f"Sensor channels: {_list_names(self.names)}. Add the element under "
                f"'sensors:' in the controller YAML to read it."
            ) from None

    def __contains__(self, name: str) -> bool:
        return name in self._values

    def get(self, name: str, default: float = 0.0) -> float:
        return self._values.get(name, default)

    def as_dict(self) -> Dict[str, float]:
        return dict(self._values)

    def _element(self, kind: str, name: str) -> SimpleNamespace:
        prefix = f"{kind}.{name}."
        fields = {n[len(prefix):]: v for n, v in self._values.items() if n.startswith(prefix)}
        if not fields:
            known = sorted({n.split(".")[1] for n in self.names if n.startswith(kind + ".")})
            raise KeyError(
                f"no {kind} '{name}' among the sensors.{_suggest(name, known)} "
                f"Sensor {kind}s: {_list_names(known)}."
            )
        return SimpleNamespace(**fields)

    def body(self, name: str) -> SimpleNamespace:
        """Body channels plus tuples: pos, rot, quat_wxyz, vel, ang_vel, acc, ang_acc."""
        b = self._element("body", name)
        b.pos = (b.pos_x, b.pos_y, b.pos_z)
        b.rot = (b.rot_x, b.rot_y, b.rot_z)
        b.quat_wxyz = (b.quat_w, b.quat_x, b.quat_y, b.quat_z)
        b.vel = (b.vel_x, b.vel_y, b.vel_z)
        b.ang_vel = (b.ang_vel_x, b.ang_vel_y, b.ang_vel_z)
        b.acc = (b.acc_x, b.acc_y, b.acc_z)
        b.ang_acc = (b.ang_acc_x, b.ang_acc_y, b.ang_acc_z)
        return b

    def motor(self, name: str) -> SimpleNamespace:
        return self._element("motor", name)

    def tsda(self, name: str) -> SimpleNamespace:
        return self._element("tsda", name)

    def rsda(self, name: str) -> SimpleNamespace:
        return self._element("rsda", name)


def _user_traceback(exc: BaseException) -> str:
    """Traceback text without frames from this helper module."""
    here = os.path.normcase(os.path.abspath(__file__))
    frames = [
        f for f in traceback.extract_tb(exc.__traceback__)
        if os.path.normcase(os.path.abspath(f.filename)) != here
    ]
    lines = ["Traceback (most recent call last):\n"]
    lines += traceback.format_list(frames)
    lines += traceback.format_exception_only(type(exc), exc)
    return "".join(lines).rstrip()


class ControllerModule(ExternalForceModule):
    """Controller authoring base (preview): implement step(state) -> dict.

    step() returns {actuator_channel: value}, e.g.
        {"tsda.winch_rope.rest_length": 19.5, "motor.slew.angle": 0.1}
    Any actuator channel you leave out keeps its previous value (zero-order
    hold); so does an explicit NaN, which also logs a warning once.

    Optional hooks: setup(cfg) (reads `config:` from the controller YAML and
    may return {"name", "version"}), reset().

    print() output from setup() and step() is shown in the SEA-Stack console,
    prefixed with the script name. self.log(msg) does the same.

    Units are SI. Sign conventions and frames are listed per channel in
    docs/extending/CONTROLLERS.md.
    """

    def __init__(self) -> None:
        super().__init__()
        self.sensors: Tuple[str, ...] = ()
        self.actuators: Tuple[str, ...] = ()
        self._log_lines: List[str] = []
        self._warned_nan: set = set()

    # -- user hooks -------------------------------------------------------
    def setup(self, cfg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Optional: read `config:` (dict). Return meta overrides if desired."""
        return None

    def step(self, state: ControllerState) -> Optional[Mapping[str, float]]:
        raise NotImplementedError("implement step(state) and return a dict of set-points")

    def log(self, message: Any) -> None:
        """Show a line in the SEA-Stack console."""
        self._log_lines.extend(str(message).splitlines() or [""])

    # -- protocol plumbing ------------------------------------------------
    def _take_log(self) -> List[str]:
        lines, self._log_lines = self._log_lines, []
        return lines

    def _format_error(self, exc: BaseException) -> str:
        return _user_traceback(exc)

    def _call_captured(self, fn, *args):
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
                return fn(*args)
        finally:
            text = buf.getvalue()
            if text:
                self._log_lines.extend(text.rstrip("\n").splitlines())

    def initialize(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        self.sensors = tuple(self._host_in_names or ())
        self.actuators = tuple(self._host_out_names or ())
        meta = self._call_captured(self.setup, cfg) or {}
        return {
            "name": str(meta.get("name", self.__class__.__name__)),
            "version": str(meta.get("version", "1.0")),
            "n_states": int(meta.get("n_states", 0)),
        }

    def reset(self) -> None:
        pass

    def evaluate(self, t: float, dt: float, inputs: List[float]) -> List[float]:
        state = ControllerState(t, dt, self.sensors, inputs)
        commands = self._call_captured(self.step, state)
        return self._pack_commands(commands)

    def _pack_commands(self, commands: Any) -> List[float]:
        if commands is None:
            commands = {}
        if not isinstance(commands, Mapping):
            raise TypeError(
                "step() must return a dict of {actuator_channel: value}, "
                f"got {type(commands).__name__}"
            )
        valid = set(self.actuators)
        for key in commands:
            if key in valid:
                continue
            if key in self.sensors:
                raise KeyError(
                    f"step() returned '{key}', which is a sensor channel. Only actuator "
                    f"channels can be written: {_list_names(self.actuators)}. List the "
                    f"element under 'actuators:' in the controller YAML to drive it."
                )
            raise KeyError(
                f"step() returned unknown actuator channel '{key}'."
                f"{_suggest(str(key), self.actuators)} "
                f"Actuator channels: {_list_names(self.actuators)}."
            )
        out: List[float] = []
        for name in self.actuators:
            if name not in commands:
                out.append(float("nan"))  # hold
                continue
            raw = commands[name]
            try:
                if isinstance(raw, (str, bytes)):
                    raise TypeError
                value = float(raw)
            except (TypeError, ValueError):
                raise TypeError(
                    f"step() value for '{name}' must be a number, got {raw!r}"
                ) from None
            if not math.isfinite(value):
                if name not in self._warned_nan:
                    self._warned_nan.add(name)
                    self.log(
                        f"warning: step() returned {value} for '{name}'; holding the "
                        f"previous value (reported once per channel)"
                    )
                value = float("nan")
            out.append(value)
        return out


class ControllerHarness:
    """Drive a ControllerModule offline (no SEA-Stack, no sockets).

    harness = ControllerHarness(
        MyController(),
        sensors={"bodies": ["float"], "tsdas": ["winch_rope"]},
        actuators={"tsdas": ["winch_rope"]},
        motor_types={},            # {"slew": "rotation"} or "linear"
        config={"gain": 2.0},
        control_dt=0.05,
    )
    cmds = harness.step(0.0, {"body.float.pos_z": -0.1})   # -> held commands
    harness.log                                             # captured output

    Unlisted sensor values default to 0.0; unknown names raise KeyError.
    Commands start at `initial` (default 0.0) and are held between steps,
    exactly as the host does.
    """

    def __init__(
        self,
        module: ControllerModule,
        sensors: Optional[Mapping[str, Sequence[str]]] = None,
        actuators: Optional[Mapping[str, Sequence[str]]] = None,
        motor_types: Optional[Mapping[str, str]] = None,
        config: Optional[Dict[str, Any]] = None,
        control_dt: float = 0.01,
        initial: Optional[Mapping[str, float]] = None,
    ) -> None:
        self.module = module
        self.control_dt = float(control_dt)
        motor_types = dict(motor_types or {})
        self.sensor_names = self._channels(sensors or {}, motor_types, writable=False)
        self.actuator_names = self._channels(actuators or {}, motor_types, writable=True)
        self.held: Dict[str, float] = {n: 0.0 for n in self.actuator_names}
        for name, value in (initial or {}).items():
            if name not in self.held:
                raise KeyError(f"initial value for unknown actuator '{name}'")
            self.held[name] = float(value)
        self.log: List[str] = []
        module._host_in_names = list(self.sensor_names)
        module._host_out_names = list(self.actuator_names)
        module._host_kind = "controller"
        try:
            module.initialize(dict(config or {}))
        finally:
            self.log.extend(module._take_log())

    @staticmethod
    def _channels(
        lists: Mapping[str, Sequence[str]], motor_types: Mapping[str, str], writable: bool
    ) -> List[str]:
        allowed = ("motors", "tsdas", "rsdas") if writable else ("bodies", "motors", "tsdas", "rsdas")
        names: List[str] = []
        for key in lists:
            if key not in allowed:
                raise KeyError(f"unknown element list '{key}'.{_suggest(key, allowed)} Use: {allowed}")
        table = CONTROLLER_WRITABLE if writable else CONTROLLER_FIELDS
        for key, element, kind in (
            ("bodies", "body", "body"),
            ("motors", "motor", None),
            ("tsdas", "tsda", "tsda"),
            ("rsdas", "rsda", "rsda"),
        ):
            for name in lists.get(key, ()):
                if kind is None:
                    mtype = motor_types.get(name)
                    if mtype not in ("rotation", "linear"):
                        raise ValueError(
                            f"motor_types['{name}'] must be 'rotation' or 'linear'"
                        )
                    fields = table[f"{mtype} motor"]
                else:
                    fields = table[kind]
                names += [f"{element}.{name}.{f}" for f in fields]
        return names

    def step(self, t: float, values: Optional[Mapping[str, float]] = None) -> Dict[str, float]:
        """Run one control tick; returns the held command for every actuator."""
        values = dict(values or {})
        for name in values:
            if name not in self.sensor_names:
                raise KeyError(
                    f"'{name}' is not a sensor channel.{_suggest(name, self.sensor_names)}"
                )
        inputs = [float(values.get(n, 0.0)) for n in self.sensor_names]
        try:
            out = self.module.evaluate(t, self.control_dt, inputs)
        finally:
            self.log.extend(self.module._take_log())
        for name, value in zip(self.actuator_names, out):
            if math.isfinite(value):
                self.held[name] = value
        return dict(self.held)


def run(module: ExternalForceModule) -> None:
    """Connect `module` to SEA-Stack and serve the IPC loop until shutdown."""
    ExternalForceModule.run(module)


if __name__ == "__main__":
    sys.stderr.write(
        "seastack_external.py is the IPC helper only.\n"
        "Run a demo module instead, e.g.\n"
        "  demos/rm3/external_pto/linear_damper_pto.py\n"
        "  demos/rm3/external_pto_adaptive/adaptive_damping_pto.py\n"
        "  demos/rm3/external_pto_hydraulic/hydraulic_accumulator_pto.py\n"
    )
    sys.exit(2)
